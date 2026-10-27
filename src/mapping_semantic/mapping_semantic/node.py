"""실제 RGB-D 메시지로 합성 fixture 관측과 정적 객체 지도를 발행한다."""

from collections import Counter, deque
import json
import math
import sys
import time
import uuid
from types import SimpleNamespace

from mapping_semantic.core import (
    ObjectMap, ObjectRecord, detect_color_fixtures, load_json, project_mask,
    rotation_matrix, save_exclusive, stamp_ns, transform_point, validate_camera_inputs,
    external_masks, normalize_depth, validate_sync_policy,
)


def main(args=None):
    import message_filters
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener
    from tf2_msgs.msg import TFMessage
    from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose
    from visualization_msgs.msg import Marker, MarkerArray

    class SemanticMapper(Node):
        def __init__(self):
            super().__init__('semantic_mapper')
            self.instance_id = uuid.uuid4().hex
            defaults = {
                'detector_mode': 'color_fixture', 'map_frame': 'map', 'odom_frame': 'odom',
                'optical_frame': 'camera_color_optical_frame', 'map_id': '',
                'rgb_topic': '/camera/color/image_raw', 'depth_topic': '/camera/depth/image_raw',
                'camera_info_topic': '/camera/camera_info', 'output_path': '', 'input_path': '',
                'association_distance': 0.25, 'minimum_observations': 3,
                'reset_translation': 0.2, 'reset_rotation': 0.1,
                'minimum_pixels': 80, 'tf_wait_seconds': 1.0, 'queue_size': 20,
                'detections_topic': '/semantic/detections2d',
                'static_classes': ['chair', 'cup', 'bottle'],
                'sync_policy': 'exact', 'max_sync_skew_sec': 0.03,
            }
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.p = {name: self.get_parameter(name).value for name in defaults}
            validate_sync_policy(self.p['sync_policy'], self.p['max_sync_skew_sec'])
            if self.p['detector_mode'] not in ('color_fixture', 'external_detections'):
                raise ValueError('지원하지 않는 detector_mode')
            if self.p['detector_mode'] == 'color_fixture' and not self.get_parameter('use_sim_time').value:
                raise ValueError('이 합성 fixture 검증 노드는 use_sim_time=true를 요구합니다')
            for key in ('map_frame', 'odom_frame', 'optical_frame'):
                if not self.p[key]:
                    raise ValueError(f'{key}는 비어 있을 수 없습니다')
            if not math.isfinite(self.p['tf_wait_seconds']) or self.p['tf_wait_seconds'] <= 0:
                raise ValueError('tf_wait_seconds는 유한한 양수여야 합니다')
            if self.p['queue_size'] < 1 or self.p['minimum_pixels'] < 20:
                raise ValueError('queue_size>=1, minimum_pixels>=20이어야 합니다')
            self.object_map = ObjectMap(
                self.p['association_distance'], self.p['minimum_observations'],
                self.p['reset_translation'], self.p['reset_rotation'],
                allowed_classes=(self.p['static_classes'] if self.p['detector_mode'] == 'external_detections'
                                 else ('red_fixture', 'blue_fixture')),
                detector_mode=self.p['detector_mode'])
            self.bridge = CvBridge()
            self.buffer = Buffer(node=self)
            self.listener = TransformListener(self.buffer, self)
            self.pending = deque()
            self.counters = Counter()
            self.last_rejection = ''
            self.last_input_ns = 0
            self.last_clock_ns = None
            self.last_rgb_wall = None
            self.last_processed_wall = None
            self.latest_correction = None
            self.last_processed_stamp = None
            self.last_correction_ns = 0
            self.epoch_start_ns = 0
            self.cpu_seconds = 0.0
            self.latency_seconds = 0.0
            self.latency_max_seconds = 0.0
            self.subscribers = []
            for name, message_type, topic in [
                ('rgb', Image, self.p['rgb_topic']),
                ('depth', Image, self.p['depth_topic']),
                ('info', CameraInfo, self.p['camera_info_topic']),
            ]:
                subscriber = message_filters.Subscriber(self, message_type, topic,
                                                         qos_profile=qos_profile_sensor_data)
                subscriber.registerCallback(self.received, name)
                self.subscribers.append(subscriber)
            if self.p['detector_mode'] == 'external_detections':
                subscriber = message_filters.Subscriber(self, Detection2DArray, self.p['detections_topic'],
                                                         qos_profile=qos_profile_sensor_data)
                subscriber.registerCallback(self.received, 'detections')
                self.subscribers.append(subscriber)
            if self.p['sync_policy'] == 'exact':
                self.sync = message_filters.TimeSynchronizer(self.subscribers, self.p['queue_size'])
                self.sync.registerCallback(self.on_images)
            else:
                sources = self.subscribers[:3]
                if self.p['detector_mode'] == 'external_detections':
                    # 외부 검출은 approximate matching에 넣지 않는다. 같은 RGB를 먼저 확보한다.
                    self.rgb_detection_pair = message_filters.SimpleFilter()
                    self.exact_detections = message_filters.TimeSynchronizer(
                        [self.subscribers[0], self.subscribers[3]], self.p['queue_size'])
                    self.exact_detections.registerCallback(self.on_rgb_detection_pair)
                    sources = [self.rgb_detection_pair, *self.subscribers[1:3]]
                self.sync = message_filters.ApproximateTimeSynchronizer(
                    sources, self.p['queue_size'], self.p['max_sync_skew_sec'] + 1e-9,
                    allow_headerless=False)
                # message_filters는 strict '< slop'을 쓰므로 1ns를 더하고 core에서 <= 한계를 재검사한다.
                self.sync.registerCallback(self.on_approximate_images)
            self.observations_pub = self.create_publisher(Detection3DArray, '/semantic/observations', 10)
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.objects_pub = self.create_publisher(Detection3DArray, '/semantic/objects', latched)
            self.markers_pub = self.create_publisher(MarkerArray, '/semantic/markers', latched)
            self.status_pub = self.create_publisher(String, '/semantic/status', 10)
            self.create_subscription(TFMessage, '/tf', self.on_tf, qos_profile_sensor_data)
            self.create_service(Trigger, '/semantic/save', self.save)
            self.create_service(Trigger, '/semantic/load', self.load)
            self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(0.03, self.process_pending, clock=self.wall_clock)
            self.create_timer(1.0, self.status, clock=self.wall_clock)
            self.get_logger().info(f"{self.p['detector_mode']} RGB-D 표면 대표점 투영 시작")

        def received(self, message, name):
            self.counters[f'{name}_received'] += 1
            if name == 'rgb':
                self.last_rgb_wall = time.monotonic()
            if name == 'info':
                try:
                    stamp_ns(message.header.stamp)
                except (ValueError, TypeError) as exc:
                    self.reject('camera_info_rejected', exc)

        def on_rgb_detection_pair(self, rgb, detections):
            try:
                self.rgb_detection_pair.signalMessage(
                    SimpleNamespace(header=rgb.header, rgb=rgb, detections=detections))
            except Exception as exc:
                self.reject('input_rejected', f'{type(exc).__name__}: {exc}')

        def on_approximate_images(self, rgb_or_pair, depth, info):
            if self.p['detector_mode'] == 'external_detections':
                self.on_images(rgb_or_pair.rgb, depth, info, rgb_or_pair.detections)
            else:
                self.on_images(rgb_or_pair, depth, info)

        def reject(self, counter, reason):
            self.counters[counter] += 1
            self.last_rejection = str(reason)

        def check_clock(self):
            current = self.get_clock().now().nanoseconds
            if self.last_clock_ns is not None and current < self.last_clock_ns:
                self.pending.clear()
                self.last_input_ns = 0
                self.latest_correction = None
                self.last_processed_stamp = None
                self.last_correction_ns = 0
                self.epoch_start_ns = 0
                self.object_map.clear()
                self.counters['clock_resets'] += 1
                self.publish_map(self.get_clock().now().to_msg())
            self.last_clock_ns = current
            return current

        def on_tf(self, message):
            # 카메라가 끊겨도 SLAM 보정 변화 시 기존 객체를 즉시 무효화한다.
            # 이 구독은 지도 epoch 감시에만 사용하며 투영 TF는 관측 시각 조회한다.
            self.check_clock()
            for item in message.transforms:
                if item.header.frame_id != self.p['map_frame'] or item.child_frame_id != self.p['odom_frame']:
                    continue
                try:
                    timestamp = stamp_ns(item.header.stamp)
                    if timestamp < self.last_correction_ns:
                        continue
                    q, t = item.transform.rotation, item.transform.translation
                    matrix = rotation_matrix([q.x, q.y, q.z, q.w])
                    correction = [t.x, t.y, math.atan2(matrix[1, 0], matrix[0, 0])]
                    reset = self.object_map.observe_correction(correction)
                    self.latest_correction = correction
                    self.last_correction_ns = timestamp
                    if reset:
                        self.epoch_start_ns = timestamp
                        self.counters['map_resets'] += 1
                        self.counters['epoch_dropped'] += len(self.pending)
                        self.pending.clear()
                        self.last_processed_stamp = None
                        self.last_processed_wall = None
                        self.publish_map(item.header.stamp)
                except (ValueError, TypeError) as exc:
                    self.reject('correction_rejected', exc)

        def on_images(self, rgb_message, depth_message, info, detections=None):
            self.counters['synchronized'] += 1
            start_cpu = time.process_time()
            try:
                now_ns = self.check_clock()
                timestamp, k = validate_camera_inputs(
                    rgb_message, depth_message, info, self.p['optical_frame'],
                    self.p['sync_policy'], self.p['max_sync_skew_sec'])
                if timestamp <= self.last_input_ns:
                    raise ValueError('중복 또는 역행한 영상 timestamp')
                if timestamp < self.epoch_start_ns:
                    raise ValueError('이전 지도 epoch의 영상')
                if now_ns <= 0 or not -100_000_000 <= now_ns - timestamp <= 2_000_000_000:
                    raise ValueError('영상 timestamp가 현재 시뮬 시간과 맞지 않음')
                # bgr8 입력도 채널만 RGB로 바꾸며 픽셀 좌표와 원본 header는 유지한다.
                rgb = self.bridge.imgmsg_to_cv2(rgb_message, desired_encoding='rgb8')
                depth = normalize_depth(
                    self.bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough'),
                    depth_message.encoding)
                if rgb.shape != (info.height, info.width, 3) or depth.shape != (info.height, info.width):
                    raise ValueError('변환된 영상 배열 크기 불일치')
                observations = []
                masks = (external_masks(detections, rgb_message.header, depth.shape, self.p['static_classes'])
                         if detections is not None else
                         [(label, mask, 1.0) for label, mask in detect_color_fixtures(rgb, self.p['minimum_pixels'])])
                for label, mask, score in masks:
                    self.counters['detections_2d'] += 1
                    try:
                        observations.append((label, project_mask(depth, k, mask), score))
                    except ValueError as exc:
                        self.reject('depth_rejected', exc)
                self.last_input_ns = timestamp
                if len(self.pending) >= self.p['queue_size']:
                    self.pending.popleft()
                    self.counters['queue_dropped'] += 1
                self.pending.append((rgb_message.header.stamp, timestamp, time.monotonic(), observations))
            except Exception as exc:
                # 동기화 콜백에서 예외가 새면 message_filters 내부 lock이 남을 수 있다.
                self.reject('input_rejected', f'{type(exc).__name__}: {exc}')
            finally:
                self.cpu_seconds += time.process_time() - start_cpu

        def process_pending(self):
            now_ns = self.check_clock()
            while self.pending:
                stamp, timestamp, queued_at, observations = self.pending[0]
                if time.monotonic() - queued_at > self.p['tf_wait_seconds']:
                    self.pending.popleft()
                    self.reject('tf_timeout', '관측 시각 TF 대기 제한 시간 초과')
                    continue
                if not -100_000_000 <= now_ns - timestamp <= 2_000_000_000:
                    self.pending.popleft()
                    self.reject('stale_dropped', 'TF 대기 중 관측이 오래되거나 시각 역행')
                    continue
                try:
                    when = Time.from_msg(stamp)
                    camera_tf = self.buffer.lookup_transform(
                        self.p['map_frame'], self.p['optical_frame'], when).transform
                    correction_tf = self.buffer.lookup_transform(
                        self.p['map_frame'], self.p['odom_frame'], when).transform
                except TransformException:
                    return
                self.pending.popleft()
                start_cpu = time.process_time()
                try:
                    r = correction_tf.rotation
                    matrix = rotation_matrix([r.x, r.y, r.z, r.w])
                    correction = [correction_tf.translation.x, correction_tf.translation.y,
                                  math.atan2(matrix[1, 0], matrix[0, 0])]
                    # 초기 감시 TF가 아직 처리되지 않았다면 현재 관측의 correction으로 시작.
                    if self.object_map.reference is None:
                        self.object_map.observe_correction(correction)
                    q, t = camera_tf.rotation, camera_tf.translation
                    transformed = [(label, transform_point(point, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]), score)
                                   for label, point, score in observations]
                    static = [(label, point, score) for label, point, score in transformed
                              if label in self.object_map.allowed_classes]
                    records = self.object_map.update([(label, point) for label, point, _ in static], timestamp)
                    for record, (_, _, score) in zip(records, static):
                        record.score = score  # 가장 최근 모델 confidence; 위치 융합 신뢰도가 아님.
                    # 관측 토픽은 누적 평균이 아니라 이번 측정값을 발행한다.
                    measured = [ObjectRecord(record.id, label, point, 1, timestamp, score)
                                for record, (label, point, score) in zip(records, static)]
                    measured.extend(ObjectRecord('', label, point, 1, timestamp, score)
                                    for label, point, score in transformed
                                    if label not in self.object_map.allowed_classes)
                    self.observations_pub.publish(self.detection_array(measured, stamp))
                    self.publish_map(stamp)
                    self.last_processed_stamp = stamp
                    self.last_processed_wall = time.monotonic()
                    self.counters['processed'] += 1
                    self.counters['observations_3d'] += len(measured)
                    age = max(0.0, (self.get_clock().now().nanoseconds - timestamp) / 1e9)
                    self.latency_seconds += age
                    self.latency_max_seconds = max(self.latency_max_seconds, age)
                except (ValueError, TypeError, OverflowError) as exc:
                    self.reject('projection_rejected', exc)
                finally:
                    self.cpu_seconds += time.process_time() - start_cpu

        def detection_array(self, records, stamp):
            message = Detection3DArray()
            message.header.frame_id = self.p['map_frame']
            message.header.stamp = stamp
            for record in records:
                detection = Detection3D()
                detection.header = message.header
                detection.id = record.id
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = record.class_id
                hypothesis.hypothesis.score = float(record.score)
                hypothesis.pose.pose.position.x = float(record.position[0])
                hypothesis.pose.pose.position.y = float(record.position[1])
                hypothesis.pose.pose.position.z = float(record.position[2])
                hypothesis.pose.pose.orientation.w = 1.0
                # 표면 대표점의 보수적 표시용 covariance. 실측 uncertainty 추정 아님.
                for index in (0, 7, 14):
                    hypothesis.pose.covariance[index] = 0.01
                for index in (21, 28, 35):
                    hypothesis.pose.covariance[index] = 1e6
                detection.results = [hypothesis]
                detection.bbox.center = hypothesis.pose.pose
                # 미추정 object 크기는 0. bbox 대신 pose 위치만 사용한다.
                message.detections.append(detection)
            return message

        def publish_map(self, stamp):
            records = self.object_map.confirmed()
            self.objects_pub.publish(self.detection_array(records, stamp))
            array = MarkerArray()
            clear = Marker()
            clear.header.frame_id = self.p['map_frame']
            clear.header.stamp = stamp
            clear.action = Marker.DELETEALL
            array.markers.append(clear)
            for record in records:
                marker = Marker()
                marker.header = clear.header
                marker.ns = record.class_id
                marker.id = int(record.id.rsplit('_', 1)[1])
                marker.action = Marker.ADD
                marker.type = Marker.SPHERE
                marker.pose.position.x = float(record.position[0])
                marker.pose.position.y = float(record.position[1])
                marker.pose.position.z = float(record.position[2])
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.12
                marker.color.a = 1.0
                if record.class_id == 'red_fixture':
                    marker.color.r = 1.0
                else:
                    marker.color.b = 1.0
                array.markers.append(marker)
            self.markers_pub.publish(array)

        def status(self):
            now = time.monotonic()
            count = self.counters['processed']
            message = String()
            message.data = json.dumps({
                'instance_id': self.instance_id,
                'detector_mode': self.p['detector_mode'], 'map_id': self.p['map_id'],
                'sync_policy': self.p['sync_policy'], 'max_sync_skew_sec': self.p['max_sync_skew_sec'],
                'depth_unit': 'm', 'observation_timestamp_source': 'rgb',
                'frame': self.p['map_frame'], 'epoch': self.object_map.epoch,
                'objects_confirmed': len(self.object_map.confirmed()),
                'objects_pending': len(self.object_map.objects) - len(self.object_map.confirmed()),
                'tf_queue_size': len(self.pending), 'counters': dict(self.counters),
                'camera_active': self.last_rgb_wall is not None and now - self.last_rgb_wall <= 2.0,
                'pipeline_active': self.last_processed_wall is not None and now - self.last_processed_wall <= 2.0,
                'cpu_seconds_total': self.cpu_seconds,
                'mean_cpu_ms_per_synchronized_frame': 1000 * self.cpu_seconds / max(1, self.counters['synchronized']),
                'mean_sim_latency_ms': 1000 * self.latency_seconds / max(1, count),
                'max_sim_latency_ms': 1000 * self.latency_max_seconds,
                'last_rejection': self.last_rejection,
            }, ensure_ascii=False, allow_nan=False)
            self.status_pub.publish(message)

        def save(self, request, response):
            try:
                document = self.object_map.to_document(self.p['map_id'], self.p['map_frame'])
                save_exclusive(self.get_parameter('output_path').value, document)
                response.success, response.message = True, '객체 지도 저장 완료'
            except (OSError, ValueError, TypeError) as exc:
                response.success, response.message = False, str(exc)
            return response

        def load(self, request, response):
            try:
                if self.latest_correction is None or self.last_processed_stamp is None:
                    raise ValueError('현재 관측 시각의 TF 확인 후에만 load 가능')
                if self.last_processed_wall is None or time.monotonic() - self.last_processed_wall > 2.0:
                    raise ValueError('현재 TF 검증 관측이 오래됨')
                document = load_json(self.get_parameter('input_path').value)
                self.object_map.load_document(document, self.p['map_id'],
                                              self.latest_correction, self.p['map_frame'])
                self.pending.clear()
                self.counters['loads'] += 1
                self.publish_map(self.last_processed_stamp)
                response.success, response.message = True, '동일 map_id 객체 지도 불러오기 완료'
            except (OSError, ValueError, TypeError, KeyError) as exc:
                response.success, response.message = False, str(exc)
            return response

    rclpy.init(args=args)
    node = None
    try:
        node = SemanticMapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
