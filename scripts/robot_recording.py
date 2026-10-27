#!/usr/bin/env python3
"""확정 실물 프로파일의 센서 기록과 격리된 센서 전용 재생. 구동 명령 발행 없음."""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import signal
import subprocess
import sys
import time

import yaml

# 저장소 스크립트와 설치된 mapping_robot 양쪽에서 동일 프로파일 계약을 사용한다.
try:
    from mapping_robot.profile import TOPIC_TYPES, validate_profile
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/mapping_robot'))
    from mapping_robot.profile import TOPIC_TYPES, validate_profile

TF_TYPE = 'tf2_msgs/msg/TFMessage'


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def load_profile(path):
    path = Path(path).expanduser().resolve()
    raw = path.read_bytes()
    profile = yaml.safe_load(raw)
    issues = validate_profile(profile, check_files=False)
    if issues:
        raise ValueError('; '.join(issues))
    if any(profile['topics'][key] is not None for key in ('rgb', 'depth', 'camera_info')) and profile['frames']['optical'] is None:
        raise ValueError('카메라 토픽을 기록하려면 optical frame을 확정해야 합니다')
    return profile, hashlib.sha256(raw).hexdigest()


def topic_types(profile):
    keys = ['scan', 'odom', 'rgb', 'depth', 'camera_info']
    if profile['semantic']['enabled']:
        keys.append('detections')
    result = {profile['topics'][key]: TOPIC_TYPES[key] for key in keys
              if profile['topics'][key] is not None}
    result.update({'/tf': TF_TYPE, '/tf_static': TF_TYPE})
    return result


def qos_profiles(profile, replay=False):
    result = {topic: {'reliability': 'reliable' if replay else 'best_effort',
                      'durability': 'volatile', 'history': 'keep_last', 'depth': 100}
              for topic in topic_types(profile)}
    result['/tf_static'] = {'reliability': 'reliable', 'durability': 'transient_local',
                            'history': 'keep_last', 'depth': 100}
    return result


def new_path(path, source=None):
    path = Path(path).expanduser().resolve()
    if path.exists():
        raise ValueError('기존 출력을 덮어쓰지 않습니다: ' + str(path))
    if source is not None:
        source = Path(source).expanduser().resolve()
        if path == source or source in path.parents:
            raise ValueError('출력은 원본 bag 밖이어야 합니다')
    if not path.parent.is_dir():
        raise ValueError('출력 부모 디렉터리가 없습니다: ' + str(path.parent))
    return path


def stamp_ns(stamp, allow_zero=False):
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        raise ValueError('잘못된 센서/TF timestamp')
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    if value == 0 and not allow_zero:
        raise ValueError('센서/TF timestamp는 양수여야 합니다')
    return value


def validate_sensor(topic, message, profile, previous):
    topics, frames = profile['topics'], profile['frames']
    current = stamp_ns(message.header.stamp)
    if current < previous.get(topic, 0):
        raise ValueError('센서 timestamp 역행: ' + topic)
    expected = (frames['laser'] if topic == topics['scan'] else
                frames['odom'] if topic == topics['odom'] else frames['optical'])
    if message.header.frame_id != expected:
        raise ValueError('프로파일과 sensor frame 불일치: ' + topic)
    if topic == topics['odom'] and message.child_frame_id != frames['base']:
        raise ValueError('odom child_frame_id 불일치')
    if topic == topics['detections']:
        for detection in message.detections:
            if (detection.header.frame_id != expected
                    or stamp_ns(detection.header.stamp) != current):
                raise ValueError('Detection2D 개별 header 불일치')
    previous[topic] = current
    return current


def static_edges_for_profile(edges, profile):
    """확인된 base→센서 경로만 유지. 가동 센서 TF·map/odom 정적 연결은 지원 안 함."""
    frames = profile['frames']
    forbidden = {frames['map'], frames['odom']}
    parents = {}
    for parent, child in edges:
        if parent in forbidden or child in forbidden:
            continue
        parents.setdefault(child, set()).add(parent)
    targets = [frames['laser']]
    if any(profile['topics'][key] is not None for key in ('rgb', 'depth', 'camera_info')):
        targets.append(frames['optical'])
    selected = set()
    for target in targets:
        child, visited = target, set()
        while child != frames['base']:
            if child in visited:
                raise ValueError('센서 static TF 경로에 순환이 있습니다')
            visited.add(child)
            candidates = parents.get(child, set())
            if len(candidates) != 1:
                raise ValueError('base→센서 static TF가 없거나 부모가 중복됩니다: ' + child)
            parent = next(iter(candidates))
            selected.add((parent, child))
            child = parent
    return selected


def transform_values(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    values = tuple(float(v) for v in (t.x, t.y, t.z, q.x, q.y, q.z, q.w))
    if (not all(math.isfinite(v) for v in values)
            or abs(math.hypot(*values[3:]) - 1.0) > 1e-3):
        raise ValueError('TF 좌표/quaternion이 유효하지 않습니다 (단위 norm 오차 <= 0.001 필요)')
    return values


def select_tf(topic, transforms, profile, static_edges, previous):
    frames = profile['frames']
    allowed = {(frames['odom'], frames['base'])} if topic == '/tf' else static_edges
    selected = []
    for item in transforms:
        edge = (item.header.frame_id, item.child_frame_id)
        if edge not in allowed:
            continue
        transform_values(item)
        if topic == '/tf':
            current = stamp_ns(item.header.stamp)
            if current < previous.get(edge, 0):
                raise ValueError('odom→base TF timestamp 역행')
            previous[edge] = current
        else:
            stamp_ns(item.header.stamp, allow_zero=True)
        # 정적 TF는 ROS 관례상 stamp=0을 허용한다. 동적/센서는 양수 필수.
        selected.append(item)
    return selected


def filter_bag(source, destination, profile):
    """두 번 읽어 TF 경로를 확정한 뒤 센서 bag을 새로 쓴다. 원본 수정 없음."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message, serialize_message
    from rosidl_runtime_py.utilities import get_message
    from tf2_msgs.msg import TFMessage

    source = Path(source).expanduser().resolve()
    if not (source / 'metadata.yaml').is_file():
        raise ValueError('metadata.yaml이 있는 원본 bag이 필요합니다')
    destination = new_path(destination, source) if destination is not None else None
    allowed = topic_types(profile)

    def open_reader():
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(source), storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
        return reader

    reader = open_reader()
    metadata = reader.get_all_topics_and_types()
    types = {topic.name: topic for topic in metadata}
    for name, kind in allowed.items():
        if name in types and (types[name].type != kind or types[name].serialization_format != 'cdr'):
            raise ValueError('원본 토픽 타입/직렬화 불일치: ' + name)
    classes = {name: get_message(kind) for name, kind in allowed.items() if name in types}
    static_values = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/clock':
            raise ValueError('원본에 /clock이 있습니다. 실물 기록 전용 재생 도구입니다')
        if topic != '/tf_static':
            continue
        message = deserialize_message(data, TFMessage)
        for item in message.transforms:
            edge = (item.header.frame_id, item.child_frame_id)
            static_values.setdefault(edge, []).append(item)
    del reader
    static_edges = static_edges_for_profile(static_values, profile)
    for edge in static_edges:
        values = [transform_values(t) for t in static_values[edge]]
        reference = values[0]
        for current in values[1:]:
            # q/-q는 같은 회전. 정적 장착 변환이 변하면 다른 좌표계를 섞지 않는다.
            qnorm = math.hypot(*reference[3:])
            cnorm = math.hypot(*current[3:])
            rotation = min(math.dist([v/qnorm for v in reference[3:]],
                                     [sign*v/cnorm for v in current[3:]]) for sign in (-1, 1))
            if math.dist(reference[:3], current[:3]) > 1e-9 or rotation > 1e-9:
                raise ValueError('동일 static TF edge 값이 변했습니다: ' + str(edge))
    reader = open_reader()
    writer = rosbag2_py.SequentialWriter() if destination is not None else None
    kept, dropped, previous, previous_tf = Counter(), Counter(), {}, {}
    payload = hashlib.sha256()
    first = last = None
    try:
        if writer is not None:
            writer.open(rosbag2_py.StorageOptions(uri=str(destination), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            for topic in metadata:
                if topic.name in allowed:
                    writer.create_topic(topic)
        while reader.has_next():
            topic, data, recorded = reader.read_next()
            if topic not in allowed:
                dropped[topic] += 1
                continue
            if recorded < 0 or (last is not None and recorded < last):
                raise ValueError('bag 기록 timestamp가 음수이거나 역행했습니다')
            message = deserialize_message(data, classes[topic])
            if topic in ('/tf', '/tf_static'):
                selected = select_tf(topic, message.transforms, profile, static_edges, previous_tf)
                if not selected:
                    dropped[topic] += 1
                    continue
                if len(selected) != len(message.transforms):
                    message.transforms = selected
                    data = serialize_message(message)
            else:
                validate_sensor(topic, message, profile, previous)
            if writer is not None:
                writer.write(topic, data, recorded)
            payload.update(topic.encode() + b'\0' + str(recorded).encode() + b'\0')
            payload.update(len(data).to_bytes(8, 'big') + bytes(data))
            kept[topic] += 1
            first = recorded if first is None else first
            last = recorded
        required = [profile['topics']['scan'], profile['topics']['odom'], '/tf', '/tf_static']
        if profile['semantic']['enabled']:
            required.extend(profile['topics'][key] for key in ('rgb', 'depth', 'camera_info', 'detections'))
        if any(kept[topic] == 0 for topic in required):
            raise ValueError('필터 후 필수 센서/TF 토픽이 비어 있습니다')
    finally:
        if writer is not None:
            writer.close()
        del reader
    return {'kept': dict(kept), 'dropped': dict(dropped),
            'static_edges': sorted(static_edges),
            'dynamic_edges': [[profile['frames']['odom'], profile['frames']['base']]],
            'first_record_ns': first, 'last_record_ns': last,
            'retained_payload_sha256': payload.hexdigest(),
            'timestamp_policy': 'positive nondecreasing sensor/dynamic TF headers; static stamp may be zero',
            'limits': '고정 센서 장착·직접 odom→base TF만 지원; 센서 간 동기화/기하 보정의 증명 아님'}


def stop_child(child):
    """start_new_session으로 직접 만든 프로세스 그룹만 종료한다."""
    for sig, seconds in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            child.poll()
            return True
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            child.poll()
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                return True
            time.sleep(.05)
    return False


def write_json(path, report, first=False):
    if first:
        with path.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
    else:
        temporary = path.with_name(path.name + '.tmp')
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        temporary.replace(path)


def bundle_provenance(root):
    path = Path(root) / 'bundle_manifest.json'
    if not path.is_file():
        return {}
    document = json.loads(path.read_text(encoding='utf-8'))
    commit = document.get('git_commit') if isinstance(document, dict) else None
    if not isinstance(commit, str) or not re.fullmatch(r'[0-9a-fA-F]{40,64}', commit):
        raise ValueError('bundle_manifest.json git_commit이 유효하지 않습니다')
    return {'bundle_commit': commit, 'bundle_manifest_sha256': sha256(path),
            'bundle_provenance': 'manifest 선언·해시; 파일 전체 무결성은 robot_workspace verify-bundle로 별도 확인'}


def source_metadata(profile, profile_sha):
    root = Path(__file__).resolve().parents[1]
    def git(*args):
        try:
            result = subprocess.run(['git', '-C', str(root), *args], capture_output=True, text=True)
            return result.stdout.strip() if result.returncode == 0 else None
        except OSError:
            return None
    return {**bundle_provenance(root), 'schema_version': 1, 'status': 'starting', 'profile': profile,
            'profile_sha256': profile_sha, 'script_sha256': sha256(__file__),
            'source_commit': git('rev-parse', 'HEAD'), 'source_status': git('status', '--short'),
            'environment': {key: os.environ.get(key) for key in
                            ('ROS_DISTRO', 'ROS_DOMAIN_ID', 'GZ_PARTITION', 'RMW_IMPLEMENTATION')},
            'host': platform.node(), 'machine': platform.machine(),
            'python': platform.python_version()}


def assert_domain(environment, confirmed):
    if (not confirmed or environment.get('ROS_DOMAIN_ID') != '12'
            or environment.get('GZ_PARTITION') != 'codex'):
        raise ValueError('재생은 --confirm-isolated, ROS_DOMAIN_ID=12, GZ_PARTITION=codex 필요')


def sensor_publisher_issues(counts, profile):
    required = {profile['topics'][key] for key in ('scan', 'odom')}
    if profile['semantic']['enabled']:
        required.update(profile['topics'][key] for key in ('rgb', 'depth', 'camera_info', 'detections'))
    issues = []
    for topic in topic_types(profile):
        if topic in ('/tf', '/tf_static'):
            continue  # 서로 다른 TF edge를 여러 노드가 발행하는 것은 정상이다.
        count = counts.get(topic, 0)
        if count > 1 or (topic in required and count != 1):
            issues.append(f'{topic}: publisher={count}, 필수 센서는 정확히 1개·선택 센서는 최대 1개 필요')
    return issues


def replay_tf_issues(topic, transforms, profile, playing=False, static_edges=()):
    """시작 전에는 SLAM 출력만 허용. 재생 중에는 필터 입력 edge도 허용한다."""
    frames = profile['frames']
    allowed = {(frames['map'], frames['odom'])} if topic == '/tf' else set()
    if playing:
        allowed |= ({(frames['odom'], frames['base'])} if topic == '/tf' else set(static_edges))
    issues = []
    for item in transforms:
        edge = (item.header.frame_id, item.child_frame_id)
        if edge not in allowed:
            issues.append('재생과 충돌하는 실제 TF: ' + topic + ':' + '→'.join(edge))
        try:
            transform_values(item)
        except ValueError as exc:
            issues.append(str(exc))
    return issues


def receiver_node_name(value):
    if not isinstance(value, str) or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*', value):
        raise ValueError('receiver-node는 /namespace/node 형식의 절대 ROS 노드 이름이어야 합니다')
    return value


def receiver_baseline_issues(snapshot, receiver_node):
    receiver_node_name(receiver_node)
    issues = []
    endpoints = snapshot.get('/tf', [])
    if len(endpoints) > 1:
        gids = [e.get('gid', '') for e in endpoints]
        if (any(not isinstance(gid, str) or not re.fullmatch(r'[0-9a-fA-F]{32,}', gid) for gid in gids)
                or len({gid[:24].lower() for gid in gids}) != 1):
            issues.append('시작 전 /tf receiver가 여러 DDS participant이거나 GID를 확인할 수 없습니다')
    if any(e['node'] != receiver_node for e in endpoints):
        issues.append('시작 전 /tf endpoint가 지정 receiver-node와 다릅니다')
    if snapshot.get('/tf_static'):
        issues.append('시작 전 /tf_static 발행자는 허용하지 않습니다')
    return issues


def replay_endpoint_issues(snapshot, baseline_tf):
    """graph GID로 사전 SLAM endpoint와 단일 player를 추적한다. 메시지 출처 증명은 아님."""
    issues = []
    for topic, endpoints in snapshot.items():
        if topic not in ('/tf', '/tf_static'):
            if len(endpoints) > 1:
                issues.append('재생 중 센서/clock publisher 중복: ' + topic)
            continue
        new = [e for e in endpoints if e['gid'] not in baseline_tf.get(topic, set())]
        if len(new) > 1 or any(e['name'] != 'rosbag2_player' for e in new):
            issues.append('재생 중 허용되지 않은 TF publisher: ' + topic)
    return issues


def endpoint_snapshot(node, profile):
    snapshot = {}
    for topic in [*topic_types(profile), '/clock']:
        endpoints = node.get_publishers_info_by_topic(topic)
        snapshot[topic] = [{'node': e.node_namespace.rstrip('/') + '/' + e.node_name,
                            'name': e.node_name, 'gid': bytes(e.endpoint_gid).hex(),
                            'type': e.topic_type,
                            'qos': {'reliability': e.qos_profile.reliability.name,
                                    'durability': e.qos_profile.durability.name,
                                    'history': e.qos_profile.history.name,
                                    'depth': e.qos_profile.depth}} for e in endpoints]
    return snapshot


class ReplayGuard:
    def __init__(self, node, profile, static_edges, receiver_node='/slam_toolbox'):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from tf2_msgs.msg import TFMessage
        self.profile, self.static_edges = profile, {tuple(edge) for edge in static_edges}
        self.receiver_node = receiver_node_name(receiver_node)
        self.playing = False
        self.errors = []
        self.received = Counter()
        self.baseline_tf = {}
        self.subscriptions = []
        for topic, qos in [('/tf', qos_profile_sensor_data),
                           ('/tf_static', QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                                    durability=DurabilityPolicy.TRANSIENT_LOCAL))]:
            self.subscriptions.append(node.create_subscription(
                TFMessage, topic, lambda message, topic=topic: self.on_tf(topic, message), qos))

    def on_tf(self, topic, message):
        self.received[topic] += 1
        problems = replay_tf_issues(topic, message.transforms, self.profile, self.playing, self.static_edges)
        self.errors.extend(problems[:max(0, 20 - len(self.errors))])

    def check(self):
        if self.errors:
            raise ValueError('; '.join(self.errors))

    def begin(self, snapshot):
        self.check()
        issues = receiver_baseline_issues(snapshot, self.receiver_node)
        if issues:
            raise ValueError('; '.join(issues))
        self.baseline_tf = {topic: {e['gid'] for e in snapshot[topic]}
                            for topic in ('/tf', '/tf_static')}
        self.playing = True

    def metadata(self):
        return {'receiver_node': self.receiver_node, 'received_tf_messages': dict(self.received), 'errors': self.errors,
                'baseline_tf_gids': {topic: sorted(gids) for topic, gids in self.baseline_tf.items()},
                'limits': '시작 전 실제 TF 검사; 실행 중 edge·graph endpoint 검사. Humble callback은 publisher GID를 제공하지 않아 개별 메시지 발행원 인과 식별은 아님. 노드 이름은 인증·권한 또는 발행원 보증이 아님. 여러 receiver endpoint는 DDS GID 첫 12바이트 participant prefix 일치만 검사하며 participant와 노드/프로세스의 일대일 관계를 보증하지 않음'}


def inspect_publishers(node, profile, replay=False, guard=None):
    """DDS 발견을 기다린 뒤 endpoint QoS와 GID를 기록한다. 조용한 SLAM TF 광고는 허용."""
    import rclpy
    until = time.monotonic() + 3.0
    forbidden = ['/clock']
    if replay:
        forbidden += [topic for topic in topic_types(profile) if topic not in ('/tf', '/tf_static')]
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.1)
        if guard is not None:
            guard.check()
        if any(node.count_publishers(topic) for topic in forbidden):
            raise ValueError('기존 발행자가 있어 실행을 거부합니다: ' + ', '.join(
                topic for topic in forbidden if node.count_publishers(topic)))
    snapshot = endpoint_snapshot(node, profile)
    for topic, kind in topic_types(profile).items():
        if any(e['type'] != kind for e in snapshot[topic]):
            raise ValueError('발행 토픽 타입 불일치: ' + topic)
    if not replay:
        issues = sensor_publisher_issues({topic: len(endpoints) for topic, endpoints in snapshot.items()}, profile)
        if issues:
            raise ValueError('; '.join(issues))
    return snapshot


def record_command(output, qos, profile):
    return ['ros2', 'bag', 'record', '-o', str(output), '--qos-profile-overrides-path',
            str(qos), *topic_types(profile)]


def replay_command(bag, qos):
    return ['ros2', 'bag', 'play', str(bag), '--clock', '100',
            '--disable-keyboard-controls', '--qos-profile-overrides-path', str(qos)]


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('유한한 양수가 필요합니다')
    return value


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    record = sub.add_parser('record', help='실물 센서만 기록; 모델 파일 불필요')
    replay = sub.add_parser('replay', help='격리된 개발 환경에서 센서만 재생; 구동 연결 금지')
    for command in (record, replay):
        command.add_argument('--profile', required=True, type=Path)
        command.add_argument('--output', required=True, type=Path)
    record.add_argument('--duration', type=positive)
    replay.add_argument('--bag', required=True, type=Path)
    replay.add_argument('--confirm-isolated', action='store_true')
    replay.add_argument('--receiver-node', default='/slam_toolbox',
                        help='시작 전 조용한 /tf endpoint를 허용할 SLAM 노드 FQN; 기본 /slam_toolbox')
    options = parser.parse_args(args)
    try:
        profile, profile_sha = load_profile(options.profile)
        if options.mode == 'replay':
            assert_domain(os.environ, options.confirm_isolated)
            receiver_node_name(options.receiver_node)
            source = options.bag.expanduser().resolve()
            if not (source / 'metadata.yaml').is_file():
                raise ValueError('원본 bag metadata.yaml이 없습니다')
            output = new_path(options.output, source)
            output.mkdir()
            report_path, qos_path = output / 'replay.json', output / 'playback-qos.yaml'
        else:
            output = new_path(options.output)
            report_path = new_path(output.with_name(output.name + '.recording.json'))
            qos_path = new_path(output.with_name(output.name + '.recording-qos.yaml'))
        report = source_metadata(profile, profile_sha)
        report['mode'], report['output'] = options.mode, str(output)
        report['qos_overrides'] = qos_profiles(profile, options.mode == 'replay')
        with qos_path.open('x', encoding='utf-8') as stream:
            yaml.safe_dump(report['qos_overrides'], stream)
        report['qos_sha256'] = sha256(qos_path)
        write_json(report_path, report, first=True)
    except (ValueError, OSError, yaml.YAMLError) as exc:
        parser.exit(1, str(exc) + '\n')

    child, handle, node, initialized = None, None, None, False
    guard = None
    exit_code = 1
    try:
        if options.mode == 'replay':
            report['source_bag'] = str(source)
            report['receiver_node'] = options.receiver_node
            report['source_sha256'] = {str(p.relative_to(source)): sha256(p)
                                      for p in sorted(source.rglob('*')) if p.is_file()}
            report['filter'] = filter_bag(source, output / 'filtered_bag', profile)
            command = replay_command(output / 'filtered_bag', qos_path)
            report['clock_policy'] = 'original /clock forbidden; player alone generates /clock'
            report['limits'] = '센서 전용 player. 실물 robot.launch(use_sim_time=false)와 연결 금지; 별도 개발용 수신 노드는 use_sim_time=true 필요'
            log_path = output / 'playback.log'
        else:
            command = record_command(output, qos_path, profile)
            report['clock_policy'] = 'live /clock publishers forbidden; host system time'
            log_path = new_path(output.with_name(output.name + '.recording.log'))
        import rclpy
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node('robot_recording_guard')
        if options.mode == 'replay':
            guard = ReplayGuard(node, profile, report['filter']['static_edges'], options.receiver_node)
        report['input_qos'] = inspect_publishers(node, profile, options.mode == 'replay', guard)
        if guard is not None:
            guard.begin(report['input_qos'])
        report['command'], report['status'] = command, 'running'
        write_json(report_path, report)
        handle = log_path.open('x', encoding='utf-8')
        child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        report['child_pid'] = report['child_pgid'] = child.pid
        write_json(report_path, report)
        started = time.monotonic()
        while child.poll() is None:
            rclpy.spin_once(node, timeout_sec=.1)
            if options.mode == 'record':
                issues = sensor_publisher_issues({topic: node.count_publishers(topic)
                                                 for topic in topic_types(profile)}, profile)
                if issues:
                    raise RuntimeError('; '.join(issues))
            if guard is not None:
                guard.check()
                report['latest_endpoints'] = endpoint_snapshot(node, profile)
                issues = replay_endpoint_issues(report['latest_endpoints'], guard.baseline_tf)
                if issues:
                    raise RuntimeError('; '.join(issues))
            clocks = node.count_publishers('/clock')
            if clocks > (1 if options.mode == 'replay' else 0):
                raise RuntimeError('/clock 발행자가 계약과 다릅니다')
            if options.mode == 'record' and options.duration and time.monotonic() - started >= options.duration:
                break
        if child.poll() is not None and child.returncode != 0:
            raise RuntimeError('bag 프로세스 종료 코드: ' + str(child.returncode))
        report['elapsed_wall_sec'] = time.monotonic() - started
        report['status'], exit_code = 'completed', 0
    except KeyboardInterrupt:
        report['status'], exit_code = 'stopped_by_user', 0
    except Exception as exc:
        report['status'], report['error'] = 'failed', str(exc)
        print('실물 bag 작업 실패: ' + str(exc), file=sys.stderr)
    finally:
        if guard is not None:
            report['tf_guard'] = guard.metadata()
            try:
                report['final_endpoints'] = endpoint_snapshot(node, profile)
            except Exception as exc:
                report['endpoint_snapshot_error'] = str(exc)
        if child is not None:
            try:
                if not stop_child(child):
                    raise RuntimeError('자기 bag 프로세스 그룹 종료 실패')
            except Exception as exc:
                report['status'], report['cleanup_error'], exit_code = 'failed', str(exc), 1
        if handle is not None:
            try:
                handle.close()
            except Exception as exc:
                report['status'], report['log_close_error'], exit_code = 'failed', str(exc), 1
        if options.mode == 'record' and exit_code == 0 and not (output / 'metadata.yaml').is_file():
            report['status'], report['error'], exit_code = 'failed', '정상 종료된 bag metadata.yaml이 없습니다', 1
        if options.mode == 'record' and exit_code == 0:
            try:
                report['recorded_input_audit'] = filter_bag(output, None, profile)
                report['bag_metadata_sha256'] = sha256(output / 'metadata.yaml')
            except Exception as exc:
                report['status'], report['error'], exit_code = 'failed', '기록 입력 감사 실패: ' + str(exc), 1
        for cleanup in ([node.destroy_node] if node is not None else []) + ([
                lambda: rclpy.shutdown() if rclpy.ok() else None] if initialized else []):
            try:
                cleanup()
            except Exception as exc:
                report.setdefault('ros_cleanup_errors', []).append(str(exc))
                report['status'], exit_code = 'failed', 1
        write_json(report_path, report)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
