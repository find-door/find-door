"""합성 RGB-D fixture 두 개의 투영·ID 유지·저장/재시작/복원을 검증한다."""

from collections import deque
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from mapping_eval.drive_route import body_command, freshness_problem, quaternion_yaw
from mapping_eval.drive_route import require_simulation
from mapping_eval.replay_benchmark import cleanup_children
from mapping_eval.run_benchmark import digest, positive
from mapping_eval.validation import stamp_ns, validate_odom, validate_scan


def finite_vector(values, size):
    if not isinstance(values, (list, tuple)) or len(values) != size:
        raise ValueError(f'{size}개 좌표가 필요합니다')
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in values):
        raise ValueError('좌표는 유한한 숫자여야 합니다')
    return [float(value) for value in values]


def transform_xyz(point, translation, quaternion):
    """전체 quaternion 회전으로 3D 위치를 변환한다."""
    point, translation = finite_vector(point, 3), finite_vector(translation, 3)
    q = finite_vector(quaternion, 4)
    norm = math.sqrt(sum(value * value for value in q))
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError('quaternion norm이 유효하지 않습니다')
    x, y, z, w = [value / norm for value in q]
    px, py, pz = point
    tx, ty, tz = 2 * (y * pz - z * py), 2 * (z * px - x * pz), 2 * (x * py - y * px)
    return [px + w * tx + y * tz - z * ty + translation[0],
            py + w * ty + z * tx - x * tz + translation[1],
            pz + w * tz + x * ty - y * tx + translation[2]]


def inverse_transform(translation, quaternion):
    q = finite_vector(quaternion, 4)
    inverse_q = [-q[0], -q[1], -q[2], q[3]]
    inverse_t = transform_xyz([-value for value in finite_vector(translation, 3)],
                              [0, 0, 0], inverse_q)
    return {'translation': inverse_t, 'quaternion': inverse_q}


def validate_truth(document):
    if not isinstance(document, dict) or document.get('frame') != 'world':
        raise ValueError('truth frame은 world여야 합니다')
    if document.get('map_id') != 'semantic_v1':
        raise ValueError('이 벤치마크는 map_id=semantic_v1만 지원합니다')
    objects = document.get('objects')
    if not isinstance(objects, dict) or set(objects) != {'red_fixture', 'blue_fixture'}:
        raise ValueError('truth에는 red_fixture와 blue_fixture가 하나씩 필요합니다')
    for point in objects.values():
        finite_vector(point, 3)
    tolerance = document.get('tolerance_m')
    if isinstance(tolerance, bool) or not isinstance(tolerance, (int, float)):
        raise ValueError('truth tolerance_m은 숫자여야 합니다')
    if not math.isfinite(tolerance) or not 0 < tolerance <= 0.1:
        raise ValueError('truth tolerance_m은 0보다 크고 0.1 m 이하여야 합니다')
    return document


def score_objects(records, truth, anchor, expected_ids=None):
    """class별 하나의 객체만 허용하고 고정 초기 정합의 3D 오차를 계산한다."""
    validate_truth(truth)
    if not isinstance(records, list) or len(records) != len(truth['objects']):
        raise ValueError('확정된 객체가 정확히 두 개여야 합니다')
    rows, identifiers, seen_ids = {}, {}, set()
    for item in records:
        if not isinstance(item, dict):
            raise ValueError('객체 record는 사전이어야 합니다')
        label, identifier = item.get('class_id'), item.get('id')
        if label not in truth['objects'] or label in rows:
            raise ValueError('객체 class가 중복되거나 정답과 다릅니다')
        if not isinstance(identifier, str) or not identifier or identifier in seen_ids:
            raise ValueError('객체 ID가 비어 있거나 중복됩니다')
        seen_ids.add(identifier)
        world = transform_xyz(item.get('position'), anchor['translation'], anchor['quaternion'])
        error = math.dist(world, truth['objects'][label])
        rows[label] = {'id': identifier, 'world_position': world,
                       'truth_position': truth['objects'][label], 'error_m': error}
        identifiers[label] = identifier
    if expected_ids is not None and identifiers != expected_ids:
        raise ValueError('객체 ID가 이전 관측과 달라졌습니다')
    maximum = max(row['error_m'] for row in rows.values())
    return {'objects': rows, 'ids': identifiers, 'max_error_m': maximum,
            'position_pass': maximum <= truth['tolerance_m']}


def saved_records_match(records, saved, tolerance=1e-8):
    """호출 구간의 좌표 일치만 확인한다. 발행을 load의 결과로 인과 식별하지 않는다."""
    if not isinstance(records, list) or not isinstance(saved, list) or len(records) != len(saved):
        return False
    try:
        for item in records + saved:
            finite_vector(item['position'], 3)
        saved_by_id = {item['id']: item for item in saved}
        if len(saved_by_id) != len(saved) or len({item['id'] for item in records}) != len(records):
            return False
        return all(item['id'] in saved_by_id
                   and item['class_id'] == saved_by_id[item['id']]['class_id']
                   and math.dist(item['position'], saved_by_id[item['id']]['position']) <= tolerance
                   for item in records)
    except (ValueError, TypeError, KeyError):
        return False


def sole_publisher_gid(endpoints):
    """Humble 메시지 콜백 대신 현재 graph의 유일한 발행자만 채택한다."""
    if len(endpoints) != 1:
        return None
    gid = bytes(endpoints[0].endpoint_gid)
    return gid.hex() if gid and any(gid) else None


def new_instance(status, previous=None):
    identifier = status.get('instance_id')
    return isinstance(identifier, str) and bool(identifier) and identifier != previous


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='새 결과 디렉토리')
    parser.add_argument('--confirm-simulation', action='store_true')
    parser.add_argument('--record-camera', action='store_true', help='RGB-D 포함 큰 bag 기록')
    parser.add_argument('--timeout', type=positive, default=180.0)
    options = parser.parse_args(args)
    try:
        require_simulation(options.confirm_simulation, os.environ)
    except ValueError as exc:
        parser.error(str(exc))
    output = options.output.expanduser().resolve()
    if output.exists():
        parser.error('기존 출력 경로를 덮어쓰지 않습니다')
    output.mkdir(parents=True, exist_ok=False)
    report = {'schema_version': 1, 'status': 'running', 'map_id': 'semantic_v1',
              'record_camera': options.record_camera,
              'limitations': 'synthetic color fixtures only; no trained object detector; '
                             'world truth is visible front-face center, not volume center',
              'movement': {'start': [1.0, 1.0], 'target': [1.2, 1.0], 'speed_mps': 0.15},
              'stages': {}}
    children, handles = [], []
    node, ros_initialized = None, False
    started = time.monotonic()
    deadline = started + options.timeout
    simulation = semantic = None
    active_semantic = False
    env = os.environ.copy()
    env['ROS_LOG_DIR'] = str(output / 'roslogs')

    def start(command, label):
        handle = (output / (label + '.log')).open('w')
        handles.append(handle)
        child = subprocess.Popen(command, env=env, stdout=handle,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        report.setdefault('commands', {})[label] = command
        return child

    try:
        import yaml
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        from cv_bridge import CvBridge
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from PIL import Image as PillowImage
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.time import Time
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import Image, LaserScan
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformException, TransformListener
        from vision_msgs.msg import Detection3DArray

        share = Path(get_package_share_directory('mapping_eval'))
        world = Path(get_package_share_directory('mapping_gazebo')) / 'worlds/semantic_v1.sdf'
        truth_path = share / 'config/semantic_v1_truth.yaml'
        slam_params = Path(get_package_share_directory('mapping_slam')) / 'config/slam.yaml'
        truth = validate_truth(yaml.safe_load(truth_path.read_text()))
        inputs = output / 'inputs'
        inputs.mkdir()
        for path in (world, truth_path, slam_params):
            (inputs / path.name).write_bytes(path.read_bytes())
        report['input_sha256'] = {path.name: digest(path) for path in inputs.iterdir()}
        import mapping_semantic.core as semantic_core
        report['code_sha256'] = {}
        for package, directory in (
                ('mapping_eval', Path(__file__).resolve().parent),
                ('mapping_semantic', Path(semantic_core.__file__).resolve().parent)):
            for path in sorted(directory.rglob('*.py')):
                report['code_sha256'][package + '/' + str(path.relative_to(directory))] = digest(path)
        wf_share = Path(get_package_share_directory('wf_gazebo'))
        report['wf_gazebo_sha256'] = {
            str(path.relative_to(wf_share)): digest(path)
            for directory in ('launch', 'config', 'urdf')
            for path in sorted((wf_share / directory).rglob('*'))
            if path.is_file() and '__pycache__' not in path.parts
        }
        report['wf_gazebo_source'] = str(wf_share)
        for package in ('mapping_gazebo', 'mapping_slam', 'mapping_semantic'):
            launch_dir = Path(get_package_share_directory(package)) / 'launch'
            for path in sorted(launch_dir.glob('*.py')):
                report['code_sha256'][package + '/launch/' + path.name] = digest(path)

        class Observer(Node):
            def __init__(self):
                super().__init__('semantic_benchmark_observer', parameter_overrides=[
                    Parameter('use_sim_time', value=True)])
                self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
                self.buffer = Buffer()
                self.listener = TransformListener(self.buffer, self)
                self.odom = self.scan = self.rgb = None
                self.odom_at = self.scan_at = self.clock_at = None
                self.clock_ns = None
                self.status_document = None
                self.status_gid = None
                self.status_at = None
                self.objects = None
                self.object_gid = None
                self.object_stamp = None
                self.object_at = None
                self.anchor = None
                self.anchor_stamp_ns = None
                self.events = deque(maxlen=300)
                self.sequence = 0
                self.status_history = []
                self.fault = None
                self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
                self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
                self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
                self.create_subscription(Image, '/camera/color/image_raw', self.on_rgb,
                                         qos_profile_sensor_data)
                self.create_subscription(String, '/semantic/status', self.on_status, 10)
                self.create_subscription(Detection3DArray, '/semantic/objects', self.on_objects,
                                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                                    durability=DurabilityPolicy.TRANSIENT_LOCAL))

            def on_clock(self, message):
                current = stamp_ns(message.clock)
                if self.clock_ns is not None:
                    if current < self.clock_ns:
                        self.fault = '/clock 역행'
                    elif current > self.clock_ns:
                        self.clock_at = time.monotonic()
                self.clock_ns = current

            def on_odom(self, message):
                problems = validate_odom(message)
                if problems:
                    self.fault = '; '.join(problems)
                self.odom, self.odom_at = message, time.monotonic()

            def on_scan(self, message):
                problems = validate_scan(message)
                if problems:
                    self.fault = '; '.join(problems)
                else:
                    valid = [value for value in message.ranges if math.isfinite(value)
                             and message.range_min <= value <= message.range_max]
                    if min(valid) < 0.25:
                        self.fault = 'LiDAR 장애물 거리 < 0.25 m'
                self.scan, self.scan_at = message, time.monotonic()

            def on_rgb(self, message):
                self.rgb = message

            def on_status(self, message):
                try:
                    gid = sole_publisher_gid(self.get_publishers_info_by_topic('/semantic/status'))
                    if gid is None:
                        return
                    self.status_document = json.loads(message.data)
                    self.status_gid = gid
                    self.status_at = time.monotonic()
                    self.status_history.append({'gid': self.status_gid,
                                                'status': self.status_document})
                except (ValueError, TypeError) as exc:
                    self.fault = 'status JSON 오류: ' + str(exc)

            def on_objects(self, message):
                try:
                    gid = sole_publisher_gid(self.get_publishers_info_by_topic('/semantic/objects'))
                    if gid is None:
                        return
                    if message.header.frame_id != 'map':
                        raise ValueError('semantic objects frame은 map이어야 합니다')
                    records = []
                    for detection in message.detections:
                        if len(detection.results) != 1:
                            raise ValueError('객체별 hypothesis 하나가 필요합니다')
                        result = detection.results[0]
                        point = result.pose.pose.position
                        records.append({'id': detection.id,
                                        'class_id': result.hypothesis.class_id,
                                        'position': finite_vector([point.x, point.y, point.z], 3)})
                    self.objects = records
                    self.object_stamp = message.header.stamp
                    self.object_gid = gid
                    self.object_at = time.monotonic()
                    self.sequence += 1
                    self.events.append({'sequence': self.sequence,
                                        'stamp_ns': stamp_ns(message.header.stamp),
                                        'gid': self.object_gid, 'records': records})
                except ValueError as exc:
                    self.fault = str(exc)

            def publish_zero(self):
                self.publisher.publish(Twist())

            def ready(self, old_status_gid=None, old_object_gid=None, after_stamp=0,
                      old_instance=None):
                if self.status_document is None or self.objects is None or len(self.objects) != 2:
                    return False
                status = self.status_document
                if not new_instance(status, old_instance):
                    return False
                if (sole_publisher_gid(self.get_publishers_info_by_topic('/semantic/status')) != self.status_gid
                        or sole_publisher_gid(self.get_publishers_info_by_topic('/semantic/objects')) != self.object_gid):
                    return False
                now = time.monotonic()
                if (now - self.status_at > 2 or now - self.object_at > 2
                        or stamp_ns(self.object_stamp) <= after_stamp
                        or status.get('map_id') != 'semantic_v1'
                        or status.get('counters', {}).get('processed', 0) < 10
                        or not status.get('camera_active') or not status.get('pipeline_active')):
                    return False
                if self.status_gid == old_status_gid or self.object_gid == old_object_gid:
                    return False
                if self.anchor is None:
                    try:
                        relation = self.buffer.lookup_transform(
                            'map', 'odom', Time.from_msg(self.object_stamp)).transform
                    except TransformException:
                        return False
                    t, q = relation.translation, relation.rotation
                    self.anchor = inverse_transform([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
                    self.anchor_stamp_ns = stamp_ns(self.object_stamp)
                return True

        rclpy.init(args=[])
        ros_initialized = True
        node = Observer()

        def spin():
            if time.monotonic() > deadline:
                raise RuntimeError('전체 벽시계 제한 시간 초과')
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.fault:
                raise RuntimeError(node.fault)
            if simulation is not None and simulation.poll() is not None:
                raise RuntimeError('시뮬레이션이 조기 종료했습니다')
            if active_semantic and semantic.poll() is not None:
                raise RuntimeError('semantic mapper가 조기 종료했습니다')

        def wait_for(predicate, timeout, label):
            end = time.monotonic() + timeout
            while not predicate():
                if time.monotonic() > end:
                    raise RuntimeError(label + ' 제한 시간 초과')
                spin()

        def trigger(service):
            client = node.create_client(Trigger, service)
            try:
                wait_for(client.service_is_ready, 10, service + ' 준비')
                future = client.call_async(Trigger.Request())
                wait_for(future.done, 10, service + ' 응답')
                return future.result()
            finally:
                node.destroy_client(client)

        def dump_semantic_parameters(label):
            instance = node.status_document['instance_id']
            gid = node.status_gid
            child = start(['ros2', 'param', 'dump', '/semantic_mapper'], label)
            wait_for(lambda: child.poll() is not None, 15, label)
            if child.returncode != 0:
                raise RuntimeError('semantic 파라미터 dump 실패: ' + label)
            path = output / (label + '.log')
            parameters = yaml.safe_load(path.read_text())
            if (not isinstance(parameters, dict) or '/semantic_mapper' not in parameters
                    or not isinstance(parameters['/semantic_mapper'].get('ros__parameters'), dict)):
                raise RuntimeError('semantic 파라미터 dump 형식 오류')
            if (node.status_document.get('instance_id') != instance
                    or sole_publisher_gid(node.get_publishers_info_by_topic('/semantic/status')) != gid):
                raise RuntimeError('파라미터 dump 중 semantic instance가 변경되었습니다')
            report.setdefault('effective_semantic_parameters', {})[label] = {
                'path': path.name, 'sha256': digest(path), 'instance_id': instance,
                'status_endpoint_gid': gid, 'parameters': parameters,
                'attribution': 'isolated owned process; unchanged instance and unique status graph endpoint',
            }

        end = time.monotonic() + 3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if any(node.count_publishers(topic) for topic in
                   ('/clock', '/odom', '/tf', '/semantic/objects')):
                raise RuntimeError('도메인 12에 기존 ROS 발행자가 있어 실행을 거부합니다')
        simulation = start(['ros2', 'launch', 'mapping_slam', 'simulation.launch.py',
                            'world:=' + str(inputs / world.name), 'gui:=false',
                            'headless_rendering:=true',
                            'params_file:=' + str(inputs / slam_params.name)], 'simulation')
        saved_path = output / 'objects.json'
        semantic_command = ['ros2', 'launch', 'mapping_semantic', 'semantic.launch.py',
                            'map_id:=semantic_v1', 'output_path:=' + str(saved_path),
                            'input_path:=' + str(saved_path)]
        semantic = start(semantic_command, 'semantic_initial')
        active_semantic = True
        recorder = None
        if options.record_camera:
            recorder = start(['ros2', 'bag', 'record', '-o', str(output / 'camera_bag'),
                              '/camera/color/image_raw', '/camera/depth/image_raw',
                              '/camera/camera_info', '/tf', '/tf_static', '/clock', '/odom',
                              '/scan', '/semantic/objects', '/semantic/status'], 'record_camera')
            wait_for(lambda: recorder.poll() is not None or 'Recording...' in
                     (output / 'record_camera.log').read_text(), 15, 'camera bag recorder')
            if recorder.poll() is not None:
                raise RuntimeError('camera bag recorder 시작 실패')
        wait_for(node.ready, 60, '초기 RGB-D 객체 두 개 확정')
        dump_semantic_parameters('semantic_initial_params')
        initial = score_objects(node.objects, truth, node.anchor)
        if not initial['position_pass']:
            raise RuntimeError('초기 객체 3D 위치 오차 > 0.1 m')
        report['stages']['initial'] = {'score': initial, 'status': node.status_document,
                                       'object_stamp_ns': stamp_ns(node.object_stamp)}
        report['initial_map_to_world'] = node.anchor
        report['initial_transform_stamp_ns'] = node.anchor_stamp_ns
        if node.rgb is None:
            raise RuntimeError('RGB 스냅샷 입력이 없습니다')
        PillowImage.fromarray(CvBridge().imgmsg_to_cv2(node.rgb, desired_encoding='rgb8')).save(
            output / 'camera_initial.png')

        # 자기가 생성한 simulation에서만 0.2m 이동. 기존 drive_route의 몸체 명령 사용.
        if node.odom is None:
            raise RuntimeError('이동 전 odom이 없습니다')
        start_pose = node.odom.pose.pose.position
        if math.dist([start_pose.x, start_pose.y], [1, 1]) > 0.05:
            raise RuntimeError('시뮬 초기 위치가 지정한 (1,1)과 다릅니다')
        move_started_stamp = stamp_ns(node.object_stamp)
        move_end = time.monotonic() + 20
        try:
            while True:
                spin()
                issue = freshness_problem(time.monotonic(), node.clock_at, node.odom_at, node.scan_at)
                if issue or node.count_publishers('/clock') != 1:
                    raise RuntimeError(issue or '/clock 발행자 중복')
                for message in (node.odom, node.scan):
                    if not -100_000_000 <= node.clock_ns - stamp_ns(message.header.stamp) <= 2_000_000_000:
                        raise RuntimeError('이동 중 센서 시각이 현재 clock과 다릅니다')
                position = node.odom.pose.pose.position
                if math.dist([position.x, position.y], [1, 1]) > 0.25:
                    raise RuntimeError('허용 이동 범위 0.25 m를 넘었습니다')
                command, reached = body_command(
                    (position.x, position.y), quaternion_yaw(node.odom.pose.pose.orientation),
                    (1.2, 1.0), 0.15, tolerance=0.02)
                if reached:
                    break
                if time.monotonic() > move_end:
                    raise RuntimeError('0.2 m 이동 제한 시간 초과')
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = command
                node.publisher.publish(message)
        finally:
            for _ in range(3):
                node.publish_zero()
                time.sleep(0.05)
        processed_before = node.status_document['counters'].get('processed', 0)
        wait_for(lambda: node.ready(after_stamp=move_started_stamp)
                 and node.status_document['counters'].get('processed', 0) >= processed_before + 10,
                 20, '이동 후 객체 재관측')
        moved = score_objects(node.objects, truth, node.anchor, initial['ids'])
        if not moved['position_pass']:
            raise RuntimeError('이동 후 객체 3D 위치 오차 > 0.1 m')
        report['stages']['after_move'] = {'score': moved, 'status': node.status_document,
                                         'odom_xy': [node.odom.pose.pose.position.x,
                                                     node.odom.pose.pose.position.y]}
        PillowImage.fromarray(CvBridge().imgmsg_to_cv2(node.rgb, desired_encoding='rgb8')).save(
            output / 'camera_after_move.png')
        saved = trigger('/semantic/save')
        if not saved.success or not saved_path.is_file():
            raise RuntimeError('객체 지도 save 실패: ' + saved.message)
        document = json.loads(saved_path.read_text())
        if document.get('map_id') != 'semantic_v1' or document.get('frame') != 'map':
            raise RuntimeError('저장된 객체 지도의 map_id/frame 불일치')
        if any(item.get('observations', 0) < 3 for item in document.get('objects', [])):
            raise RuntimeError('저장 객체의 관측 횟수가 3회 미만입니다')
        saved_score = score_objects(document.get('objects'), truth, node.anchor, initial['ids'])
        if not saved_score['position_pass']:
            raise RuntimeError('저장 객체 위치 오차 초과')
        saved_hash = digest(saved_path)
        denied = trigger('/semantic/save')
        if denied.success or digest(saved_path) != saved_hash:
            raise RuntimeError('save가 기존 JSON 덮어쓰기를 거부하지 않았습니다')
        report['save'] = {'success': True, 'sha256': saved_hash,
                          'overwrite_rejected': True, 'score': saved_score,
                          'minimum_observations': min(item['observations'] for item in document['objects'])}

        old_status_gid, old_object_gid = node.status_gid, node.object_gid
        old_instance = node.status_document['instance_id']
        from mapping_eval.run_benchmark import stop_process
        active_semantic = False
        if stop_process(semantic) is False:
            raise RuntimeError('재시작 전 semantic 프로세스 그룹 종료 실패')
        restart_stamp = node.clock_ns
        semantic = start(semantic_command, 'semantic_restart')
        active_semantic = True
        wait_for(lambda: node.ready(old_status_gid, old_object_gid, restart_stamp, old_instance),
                 45, '재시작 mapper의 새 관측')
        report['restart'] = {'new_status_publisher_gid': node.status_gid,
                             'previous_instance_id': old_instance,
                             'new_instance_id': node.status_document['instance_id'],
                             'previous_status_publisher_gid': old_status_gid,
                             'processed_before_load': node.status_document['counters']['processed']}
        dump_semantic_parameters('semantic_restart_params')
        before_sequence = node.sequence
        loaded = trigger('/semantic/load')
        if not loaded.success:
            raise RuntimeError('객체 지도 load 실패: ' + loaded.message)
        wait_for(lambda: node.status_document.get('instance_id') == report['restart']['new_instance_id']
                 and node.status_document['counters'].get('loads', 0) >= 1,
                 10, 'load counter 확인')
        restored_events = [event for event in node.events
                           if event['sequence'] > before_sequence and event['gid'] == node.object_gid
                           and saved_records_match(event['records'], document['objects'])]
        if not restored_events:
            raise RuntimeError('load 호출 구간에 저장 좌표와 일치하는 객체 event가 없습니다')
        restored = score_objects(restored_events[0]['records'], truth, node.anchor, initial['ids'])
        latest = score_objects(node.objects, truth, node.anchor, initial['ids'])
        if not restored['position_pass'] or not latest['position_pass']:
            raise RuntimeError('복원 후 객체 위치 오차 초과')
        report['stages']['after_load'] = {
            'restored_score': restored, 'latest_score': latest,
            'status': node.status_document, 'loads': node.status_document['counters']['loads'],
            'saved_coordinates_matching_event_stamp_ns': restored_events[0]['stamp_ns'],
            'evidence_limit': 'load 호출 구간 저장 좌표 일치 event이며 load 유발 발행의 독립적인 증명은 아님; GID는 수신 metadata가 아닌 유일 graph endpoint 기준',
            'latest_contains_new_observations': not saved_records_match(node.objects, document['objects']),
            'latest_stamp_ns': stamp_ns(node.object_stamp),
        }
        if recorder is not None and recorder.poll() is not None:
            raise RuntimeError('camera bag recorder가 검증 도중 종료했습니다')
        report['status'] = 'completed'
        report['quality_pass'] = True
        print(f'합성 시맨틱 검증 완료: {output}', flush=True)
        return 0
    except (Exception, KeyboardInterrupt) as exc:
        report['status'] = 'failed'
        report['quality_pass'] = False
        report['error'] = str(exc) or type(exc).__name__
        print('합성 시맨틱 검증 실패: ' + report['error'], file=sys.stderr, flush=True)
        return 1
    finally:
        if node is not None and simulation is not None and simulation.poll() is None:
            for _ in range(3):
                try:
                    node.publish_zero()
                    time.sleep(0.05)
                except Exception:
                    break
        if node is not None:
            report['semantic_status_history'] = node.status_history
            report['last_semantic_status'] = node.status_document
        report['cleanup_errors'] = cleanup_children(children, handles)
        if report['cleanup_errors']:
            report['status'] = 'failed'
            report['quality_pass'] = False
            report['error'] = '자식 프로세스 정리 실패: ' + '; '.join(report['cleanup_errors'])
        report['elapsed_wall_sec'] = time.monotonic() - started
        if node is not None:
            node.destroy_node()
        if ros_initialized and rclpy.ok():
            rclpy.shutdown()
        (output / 'semantic_benchmark.json').write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
        if report['cleanup_errors']:
            return 1


if __name__ == '__main__':
    sys.exit(main())
