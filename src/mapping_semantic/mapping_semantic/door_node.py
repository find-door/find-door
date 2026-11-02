"""동기화 RGB-D·문 검출을 관측 시각 TF로 연결하는 문 지도 ROS 노드."""

from collections import Counter, deque
from copy import deepcopy
import json
import math
import signal
import time

from mapping_semantic.core import (correction_delta, finite_vector, load_json, normalize_depth,
                                   rotation_matrix, save_exclusive, stamp_ns, transform_point,
                                   validate_camera_inputs)
from mapping_semantic.door_map import DoorMap
from mapping_semantic.door_position import threshold_point
from mapping_semantic.door_state import estimate_state
from mapping_semantic.doorway import estimate_doorway


def detection_seed(array, header, width, height, threshold):
    """배열·개별 header와 모든 점수를 검사한 뒤 최고 점수 door를 선택한다."""
    expected = stamp_ns(header.stamp)
    for message in [array, *array.detections]:
        if message.header.frame_id != header.frame_id or stamp_ns(message.header.stamp) != expected:
            raise ValueError('검출 header가 RGB와 다릅니다')
    candidates = []
    for detection in array.detections:
        for item in detection.results:
            score = item.hypothesis.score
            if not math.isfinite(score) or not 0 <= score <= 1:
                raise ValueError('검출 점수 범위 오류')
            if item.hypothesis.class_id != 'door' or score < threshold:
                continue
            bbox = detection.bbox
            x, y, w, h, theta = finite_vector([bbox.center.position.x, bbox.center.position.y,
                bbox.size_x, bbox.size_y, bbox.center.theta], 5, 'bbox')
            if w <= 0 or h <= 0 or theta != 0:
                raise ValueError('양수 비회전 bbox 필요')
            box = [max(0., x-w/2), max(0., y-h/2), min(float(width), x+w/2), min(float(height), y+h/2)]
            if box[0] >= box[2] or box[1] >= box[3]:
                raise ValueError('검출 bbox가 영상 밖에 있습니다')
            candidates.append((score, box))
    return max(candidates, key=lambda row: row[0])[1] if candidates else None


def correction_vector(transform):
    q = transform.rotation
    r = rotation_matrix([q.x, q.y, q.z, q.w])
    return finite_vector([transform.translation.x, transform.translation.y,
                          math.atan2(r[1, 0], r[0, 0])], 3, 'map odom correction').tolist()


def main(args=None):
    import message_filters
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener
    from vision_msgs.msg import Detection2DArray
    from visualization_msgs.msg import Marker, MarkerArray

    class DoorMapper(Node):
        def __init__(self):
            super().__init__('door_mapper')
            defaults = dict(map_id='', map_frame='map', odom_frame='odom',
                optical_frame='camera_color_optical_frame', rgb_topic='/camera/color/image_raw',
                depth_topic='/camera/depth/image_raw', camera_info_topic='/camera/camera_info',
                detections_topic='/semantic/detections2d', output_path='', input_path='',
                association_distance=.25, minimum_observations=3, state_ttl_sec=2.,
                confidence_threshold=.1, queue_size=20, tf_wait_seconds=1., max_source_age_sec=10.,
                reset_translation=.2, reset_rotation=.1)
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.p = {name: self.get_parameter(name).value for name in defaults}
            for key in ['tf_wait_seconds', 'max_source_age_sec', 'reset_translation', 'reset_rotation']:
                if not math.isfinite(self.p[key]) or self.p[key] <= 0:
                    raise ValueError(f'양수 유한 설정 필요: {key}')
            if (type(self.p['queue_size']) is not int or not 1 <= self.p['queue_size'] <= 1000
                    or not math.isfinite(self.p['confidence_threshold'])
                    or not 0 <= self.p['confidence_threshold'] <= 1):
                raise ValueError('큐/검출 임계값 설정 오류')
            if any(not self.p[name].strip() for name in ['map_frame', 'odom_frame', 'optical_frame']):
                raise ValueError('프레임 이름을 명시해야 합니다')
            self.memory = DoorMap(self.p['map_id'], self.p['map_frame'], self.p['association_distance'],
                                  self.p['minimum_observations'], self.p['state_ttl_sec'])
            self.bridge, self.buffer = CvBridge(), Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.pending, self.counters = deque(), Counter()
            self.last_source = self.last_input = self.last_now = self.epoch_start = 0
            self.clear_sync_pending = False
            self.reference = self.current_correction = None
            self.epoch = 0
            self.last_error = ''
            self.subscribers = [message_filters.Subscriber(self, typ, self.p[name], qos_profile=qos_profile_sensor_data)
                for typ, name in [(Image, 'rgb_topic'), (Image, 'depth_topic'),
                                  (CameraInfo, 'camera_info_topic'), (Detection2DArray, 'detections_topic')]]
            for subscriber, name in zip(self.subscribers, ['rgb', 'depth', 'camera_info', 'detections']):
                subscriber.registerCallback(self.received, name)
            self.sync = message_filters.TimeSynchronizer(self.subscribers, self.p['queue_size'])
            self.sync.registerCallback(self.images)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.map_pub = self.create_publisher(String, '/semantic/door_map', latched)
            self.marker_pub = self.create_publisher(MarkerArray, '/semantic/door_markers', latched)
            self.create_service(Trigger, '/semantic/doors/save', self.save)
            self.create_service(Trigger, '/semantic/doors/load', self.load)
            wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(.05, self.tick, clock=wall_clock)

        def reject(self, reason, error):
            self.counters[reason] += 1
            self.last_error = str(error)

        def received(self, message, name):
            self.counters[f'received_{name}'] += 1

        def reset(self, now, reason):
            self.memory.reset()
            self.pending.clear()
            if reason == 'clock_resets':
                # Humble Buffer는 clock 역행 시 이전 epoch TF를 자동으로 지우지 않는다.
                self.buffer.clear()
                # TimeSynchronizer callback 내부에서는 아직 일치 항목 삭제가 남아 있다.
                # 그 dict를 여기서 비우지 않고 callback 밖의 timer에서 정리한다.
                self.clear_sync_pending = True
            self.last_source = self.last_input = 0
            self.epoch_start = max(0, now)
            self.reference = self.current_correction = None
            self.epoch += 1
            self.counters[reason] += 1

        def clock_now(self):
            now = self.get_clock().now().nanoseconds
            if self.last_now > 0 and now < self.last_now:
                self.reset(now, 'clock_resets')
            self.last_now = now
            return now

        def images(self, rgb, depth, info, detections):
            try:
                now = self.clock_now()
                # 콜백이 먼저 역행을 발견했다면 이미 epoch가 섞인 tuple일 수 있다.
                # 동기화 큐는 callback 밖 timer에서 비운 뒤 새 batch만 허용한다.
                if self.clear_sync_pending:
                    self.reject('epoch_batch_rejected', '시각 역행 직후 동기화 batch 폐기')
                    return
                stamp, k = validate_camera_inputs(rgb, depth, info, self.p['optical_frame'])
                if stamp <= self.last_input or stamp < self.epoch_start:
                    raise ValueError('중복·역행·이전 epoch 영상')
                if now <= 0 or not -100_000_000 <= now-stamp <= self.p['max_source_age_sec']*1e9:
                    raise ValueError('RGB 시각이 현재 clock과 맞지 않습니다')
                seed = detection_seed(detections, rgb.header, rgb.width, rgb.height, self.p['confidence_threshold'])
                self.last_input = stamp
                self.counters['synchronized'] += 1
                if seed is None:
                    self.counters['no_detection'] += 1
                    return
                color = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
                metric = normalize_depth(self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough'), depth.encoding)
                if color.shape != (info.height, info.width, 3) or metric.shape != (info.height, info.width):
                    raise ValueError('영상 변환 후 해상도 불일치')
                if len(self.pending) >= self.p['queue_size']:
                    self.pending.popleft()
                    self.counters['queue_dropped'] += 1
                self.pending.append((rgb.header.stamp, stamp, time.monotonic(), color, metric, k, seed))
            except Exception as exc:
                self.reject('input_rejected', exc)

        def watch_correction(self, now):
            try:
                tf = self.buffer.lookup_transform(self.p['map_frame'], self.p['odom_frame'], Time())
                correction = correction_vector(tf.transform)
            except TransformException:
                return
            if self.reference is not None:
                distance, angle = correction_delta(self.reference, correction)
                if distance > self.p['reset_translation'] or angle > self.p['reset_rotation']:
                    self.reset(now, 'correction_resets')
            if self.reference is None:
                self.reference = correction
            self.current_correction = correction

        def tick(self):
            now = self.clock_now()
            if self.clear_sync_pending:
                for queue in self.sync.queues:
                    queue.clear()
                self.clear_sync_pending = False
            self.watch_correction(now)
            while self.pending:
                stamp, ns, queued, color, depth, k, seed = self.pending[0]
                if time.monotonic()-queued > self.p['tf_wait_seconds']:
                    self.pending.popleft()
                    self.reject('tf_timeout', '관측 시각 TF 대기 초과')
                    continue
                if ns < self.epoch_start or not -100_000_000 <= now-ns <= self.p['max_source_age_sec']*1e9:
                    self.pending.popleft()
                    self.reject('stale_dropped', '이전 epoch 또는 오래된 관측')
                    continue
                try:
                    when = Time.from_msg(stamp)
                    camera = self.buffer.lookup_transform(self.p['map_frame'], self.p['optical_frame'], when).transform
                    correction = correction_vector(self.buffer.lookup_transform(
                        self.p['map_frame'], self.p['odom_frame'], when).transform)
                except TransformException:
                    break
                self.pending.popleft()
                try:
                    if self.reference is None:
                        self.reference = correction
                    distance, angle = correction_delta(self.reference, correction)
                    if distance > self.p['reset_translation'] or angle > self.p['reset_rotation']:
                        self.reject('epoch_tf_rejected', '관측 시각의 지도 보정이 현재 epoch와 다릅니다')
                        continue
                    proposal = estimate_doorway(color, depth, k, seed)
                    if proposal['status'] != 'estimated':
                        self.reject('doorway_unavailable', proposal['reason'])
                        continue
                    prediction = estimate_state(depth, k, proposal['bbox'])
                    evidence = prediction['evidence']
                    if 'wall_plane_normal' not in evidence:
                        self.reject('position_unavailable', prediction['reason'])
                        continue
                    point = threshold_point(k, proposal['bbox'], evidence['wall_plane_normal'], evidence['wall_plane_offset_m'])
                    t, q = camera.translation, camera.rotation
                    position = transform_point(point, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]).tolist()
                    self.memory.observe(position, prediction['state'], ns)
                    self.last_source = ns
                    self.counters['processed'] += 1
                except Exception as exc:
                    self.reject('processing_rejected', exc)
            self.publish(now)

        def visible(self, now):
            if self.last_source <= 0:
                return []
            records = deepcopy(self.memory.snapshot(self.last_source))
            for record in records:
                if now-record['last_stamp_ns'] > self.p['state_ttl_sec']*1e9:
                    record['state'] = 'unknown'
            return records

        def publish(self, now):
            records = self.visible(now)
            message = String()
            message.data = json.dumps(dict(map_id=self.p['map_id'], frame=self.p['map_frame'],
                stamp_ns=now, observation_stamp_ns=self.last_source, epoch=self.epoch, objects=records,
                counters=dict(self.counters), last_error=self.last_error), ensure_ascii=False, allow_nan=False)
            self.map_pub.publish(message)
            markers = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            markers.markers.append(clear)
            for record in records:
                number = int(record['id'][5:])
                for label in (False, True):
                    marker = Marker()
                    marker.header.frame_id = self.p['map_frame']
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = 'door_text' if label else 'doors'
                    marker.id = number
                    marker.type = Marker.TEXT_VIEW_FACING if label else Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = record['position']
                    marker.pose.position.z += .3 if label else .1
                    marker.pose.orientation.w = 1.
                    marker.scale.x = marker.scale.y = .18
                    marker.scale.z = .16 if label else .18
                    marker.color.a = 1.
                    color = {'open': (0., 1., 0.), 'closed': (1., 0., 0.), 'unknown': (1., 1., 0.)}[record['state']]
                    marker.color.r, marker.color.g, marker.color.b = color
                    marker.text = f"{record['id']} {record['state']}"
                    markers.markers.append(marker)
            self.marker_pub.publish(markers)

        def save(self, request, response):
            try:
                now = self.clock_now()
                self.watch_correction(now)
                if self.last_source <= 0 or self.current_correction is None:
                    raise ValueError('관측과 현재 지도 보정 TF가 필요합니다')
                document = self.memory.dump(self.last_source)
                document['objects'] = self.visible(now)
                if not document['objects']:
                    raise ValueError('저장할 문이 없습니다')
                save_exclusive(self.p['output_path'], dict(door_session_schema_version=1,
                    correction_reference=self.reference, memory=document))
                response.success, response.message = True, '문 지도 저장 완료'
            except Exception as exc:
                response.success, response.message = False, str(exc)
            return response

        def load(self, request, response):
            try:
                now = self.clock_now()
                # 실패한 load가 현재 메모리를 초기화하지 않도록 조회만 수행한다.
                correction = correction_vector(self.buffer.lookup_transform(
                    self.p['map_frame'], self.p['odom_frame'], Time()).transform)
                document = load_json(self.p['input_path'])
                if (not isinstance(document, dict) or set(document) != {'door_session_schema_version', 'correction_reference', 'memory'}
                        or type(document['door_session_schema_version']) is not int or document['door_session_schema_version'] != 1):
                    raise ValueError('문 세션 schema 오류')
                reference = finite_vector(document['correction_reference'], 3, 'saved correction').tolist()
                distance, angle = correction_delta(reference, correction)
                if distance > self.p['reset_translation'] or angle > self.p['reset_rotation']:
                    raise ValueError('저장 지도와 현재 TF 보정 불일치')
                self.memory.load(document['memory'], now)
                self.pending.clear()
                self.last_source = self.last_input = now
                self.reference, self.current_correction = reference, correction
                response.success, response.message = True, '문 지도 복원 완료'
                self.publish(now)
            except Exception as exc:
                response.success, response.message = False, str(exc)
            return response

    rclpy.init(args=args)
    node = None
    try:
        node = DoorMapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT, previous)
