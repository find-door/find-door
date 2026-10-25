"""SLAM 출력을 제거하고 동일 센서 입력만 남기는 오프라인 rosbag 필터."""

from collections import Counter
import hashlib
import json
from pathlib import Path


TOPICS = {
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/odom': 'nav_msgs/msg/Odometry',
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}
# 허용된 방향·토픽까지 일치해야 한다. map 프레임과 임의 odom 자식은 제외한다.
DYNAMIC_EDGES = frozenset({('odom', 'base_footprint')})
STATIC_EDGES = frozenset({
    ('base_footprint', 'base_link'), ('base_link', 'camera_link0'),
    ('camera_link0', 'camera_color_optical_frame'), ('base_link', 'imu_link'),
    ('base_link', 'lidar_frame'), ('lidar_frame', 'lidar_sim_frame'),
})


def select_transforms(topic, transforms):
    """알려진 로봇 TF edge만 유지한다. 입력 배열을 변경하지 않는다."""
    allowed = DYNAMIC_EDGES if topic == '/tf' else STATIC_EDGES if topic == '/tf_static' else ()
    return [item for item in transforms
            if (item.header.frame_id, item.child_frame_id) in allowed]


def validate_output(input_path, output_path):
    source, destination = Path(input_path).resolve(), Path(output_path).resolve()
    if not source.is_dir() or not (source / 'metadata.yaml').is_file():
        raise ValueError('입력은 metadata.yaml이 있는 rosbag 디렉토리여야 합니다')
    if destination == source or source in destination.parents:
        raise ValueError('출력을 입력 bag 내부에 만들 수 없습니다')
    if destination.exists():
        raise ValueError('기존 출력 경로를 덮어쓰지 않습니다: ' + str(destination))
    return source, destination


def replay_timing_issues(expected_last_odom_ns, last_sample_ns, last_clock_ns, last_map_ns):
    """관측 자체의 마지막 시각이 아닌 원본 bag 끝을 기준으로 누락을 검출한다."""
    trajectory, occupancy_map = [], []
    endpoint_valid = expected_last_odom_ns is not None and expected_last_odom_ns > 0
    clock_valid = last_clock_ns is not None and last_clock_ns > 0
    if not endpoint_valid:
        trajectory.append('원본 bag의 마지막 odom 시각이 없거나 유효하지 않습니다')
    if last_sample_ns is None:
        trajectory.append('TF를 확보한 궤적 표본이 없습니다')
    elif endpoint_valid and not 0 <= expected_last_odom_ns - last_sample_ns <= 1_000_000_000:
        trajectory.append('마지막 궤적 표본이 원본 bag odom 끝에서 1초 넘게 누락되거나 미래입니다')
    if not clock_valid:
        trajectory.append('양수 /clock을 수신하지 못했습니다')
    elif endpoint_valid and not 0 <= last_clock_ns - expected_last_odom_ns <= 1_000_000_000:
        trajectory.append('마지막 /clock이 원본 bag odom 끝보다 과거이거나 1초 넘게 차이납니다')
    if last_map_ns is None or last_map_ns <= 0:
        occupancy_map.append('마지막 지도의 양수 header stamp가 없습니다')
    elif not clock_valid or not 0 <= last_clock_ns - last_map_ns <= 2_000_000_000:
        occupancy_map.append('마지막 지도는 종료 /clock 기준 2초보다 오래되었거나 미래입니다')
    return {'trajectory': trajectory, 'map': occupancy_map}


def _file_hashes(folder):
    result = {}
    for path in sorted(folder.rglob('*')):
        if path.is_file():
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(block)
            result[str(path.relative_to(folder))] = digest.hexdigest()
    return result


def filter_bag(input_path, output_path):
    """입력 순서·레코드 시각·토픽 QoS를 유지하며 새 sqlite3 bag을 생성한다.

    output_path는 새 결과 폴더이며 bag/ 및 filter_manifest.json을 담는다.
    /tf의 허용된 TransformStamped는 그대로 두고 TFMessage만 다시 직렬화한다.
    """
    source, destination = validate_output(input_path, output_path)
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message, serialize_message
    from tf2_msgs.msg import TFMessage

    destination.mkdir(parents=True, exist_ok=False)
    report = {
        'schema_version': 1, 'status': 'running', 'source': str(source),
        'bag': str(destination / 'bag'), 'input_file_sha256': _file_hashes(source),
        'topic_allowlist': TOPICS,
        'dynamic_edge_allowlist': sorted(DYNAMIC_EDGES),
        'static_edge_allowlist': sorted(STATIC_EDGES),
        'input_counts': {}, 'kept_counts': {}, 'dropped_counts': {},
        'kept_tf_edges': {}, 'dropped_tf_edges': {},
        'record_timestamp_policy': 'unchanged; SequentialReader iteration order retained',
        'qos_policy': 'original offered_qos_profiles string unchanged',
    }
    writer = None
    incoming, kept, dropped = Counter(), Counter(), Counter()
    kept_edges, dropped_edges = Counter(), Counter()
    payload_digest = hashlib.sha256()
    first_timestamp = last_timestamp = None
    first_odom_stamp = last_odom_stamp = None
    try:
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(source), storage_id=''),
                    rosbag2_py.ConverterOptions('', ''))
        metadata = reader.get_metadata()
        report['record_duration_sec'] = metadata.duration.nanoseconds / 1_000_000_000
        topics = reader.get_all_topics_and_types()
        report['topic_metadata'] = []
        for topic in topics:
            if topic.name in TOPICS:
                if topic.type != TOPICS[topic.name] or topic.serialization_format != 'cdr':
                    raise ValueError('예상하지 않은 bag 토픽 타입/직렬화: ' + topic.name)
                report['topic_metadata'].append({
                    'name': topic.name, 'type': topic.type,
                    'serialization_format': topic.serialization_format,
                    'offered_qos_profiles': topic.offered_qos_profiles,
                })
        if set(TOPICS) - {item['name'] for item in report['topic_metadata']}:
            raise ValueError('필수 입력 토픽이 bag에 없습니다: ' + str(sorted(set(TOPICS))))
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(destination / 'bag'), storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('', ''))
        for topic in topics:
            if topic.name in TOPICS:
                writer.create_topic(topic)
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            incoming[topic] += 1
            if topic not in TOPICS:
                dropped[topic] += 1
                continue
            if topic == '/odom':
                stamp = deserialize_message(data, Odometry).header.stamp
                odom_stamp = stamp.sec * 1_000_000_000 + stamp.nanosec
                first_odom_stamp = odom_stamp if first_odom_stamp is None else first_odom_stamp
                last_odom_stamp = odom_stamp
            if topic in ('/tf', '/tf_static'):
                message = deserialize_message(data, TFMessage)
                selected = select_transforms(topic, message.transforms)
                allowed = DYNAMIC_EDGES if topic == '/tf' else STATIC_EDGES
                for transform in message.transforms:
                    edge = (transform.header.frame_id, transform.child_frame_id)
                    counter = kept_edges if edge in allowed else dropped_edges
                    counter[topic + ':' + edge[0] + '->' + edge[1]] += 1
                if not selected:
                    dropped[topic] += 1
                    continue
                # 모든 edge가 허용된 메시지는 원본 CDR 바이트도 보존한다.
                # 재직렬화 시 CDR padding 바이트가 달라질 수 있다.
                if len(selected) != len(message.transforms):
                    message.transforms = selected
                    data = serialize_message(message)
            writer.write(topic, data, timestamp)
            payload_digest.update(topic.encode() + b'\0' + str(timestamp).encode() + b'\0')
            payload_digest.update(len(data).to_bytes(8, 'big') + bytes(data))
            kept[topic] += 1
            first_timestamp = timestamp if first_timestamp is None else first_timestamp
            last_timestamp = timestamp
        if any(kept[topic] == 0 for topic in TOPICS):
            raise ValueError('필터 후 비어 있는 필수 입력 토픽이 있습니다')
        if set(STATIC_EDGES) - {
                tuple(edge.split(':', 1)[1].split('->'))
                for edge in kept_edges if edge.startswith('/tf_static:')}:
            raise ValueError('필터 후 로봇 고정 TF 트리가 불완전합니다')
        report['status'] = 'completed'
    except Exception as exc:
        report['status'] = 'failed'
        report['error'] = str(exc)
        raise
    finally:
        if writer is not None:
            writer.close()
        report.update({
            'input_counts': dict(incoming), 'kept_counts': dict(kept),
            'dropped_counts': dict(dropped), 'kept_tf_edges': dict(kept_edges),
            'dropped_tf_edges': dict(dropped_edges),
            'first_record_timestamp_ns': first_timestamp,
            'last_record_timestamp_ns': last_timestamp,
            'first_odom_header_stamp_ns': first_odom_stamp,
            'last_odom_header_stamp_ns': last_odom_stamp,
            'retained_record_payload_sha256': payload_digest.hexdigest(),
            'output_file_sha256': _file_hashes(destination / 'bag'),
        })
        (destination / 'filter_manifest.json').write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    return report
