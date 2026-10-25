"""속도 명령을 발행하지 않는 실물 센서·시각·TF 사전 검사."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import time

from mapping_robot.profile import TOPIC_TYPES, finite, load_profile


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('음수 또는 잘못된 stamp')
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quaternion_valid(q):
    values = [q.x, q.y, q.z, q.w]
    return all(math.isfinite(v) for v in values) and abs(math.hypot(*values) - 1) <= 0.01


def range_tolerance(value):
    """거리 float32 직렬화 오차만 허용: 최소 1µm 또는 상대 1ppm."""
    return max(1e-6, abs(value) * 1e-6)


def validate_image_storage(message):
    formats = {'rgb8': (3, 1), 'bgr8': (3, 1), '32FC1': (1, 4), '16UC1': (1, 2)}
    if message.encoding not in formats:
        raise ValueError('지원하지 않는 image encoding')
    channels, size = formats[message.encoding]
    if (message.width <= 0 or message.height <= 0
            or message.step < message.width * channels * size
            or len(message.data) != message.step * message.height
            or message.is_bigendian not in (0, 1)):
        raise ValueError('image data/step/해상도 계약 오류')


def validate_camera_arrays(rgb, depth, depth_encoding):
    """CvBridge가 복호한 실제 배열 검사. 전영상 유효 depth가 ROI 정확도를 보증하지 않는다."""
    import numpy as np
    from mapping_semantic.core import normalize_depth
    if rgb.ndim != 3 or rgb.shape[2] != 3 or rgb.dtype != np.uint8:
        raise ValueError('RGB 배열 shape/dtype 오류')
    metres = normalize_depth(depth, depth_encoding)
    if rgb.shape[:2] != metres.shape:
        raise ValueError('복호 RGB/depth 해상도 불일치')
    count = int(np.count_nonzero(np.isfinite(metres) & (metres >= .2) & (metres <= 10.)))
    if count < 20:
        raise ValueError('전영상에 0.2~10m 유효 depth 20개 미만')
    return count


def pose_tf_error(pose, transform):
    """같은 stamp의 odom pose와 TF. q/-q를 동일한 회전으로 취급한다."""
    p, t = pose.position, transform.translation
    a, b = pose.orientation, transform.rotation
    if (not all(math.isfinite(v) for v in (p.x,p.y,p.z,t.x,t.y,t.z))
            or not quaternion_valid(a) or not quaternion_valid(b)):
        raise ValueError('odom/TF pose 비교 입력 수치 오류')
    qa, qb = [a.x,a.y,a.z,a.w], [b.x,b.y,b.z,b.w]
    dot = abs(sum(x*y for x,y in zip(qa,qb)) / (math.hypot(*qa)*math.hypot(*qb)))
    distance = math.dist([p.x,p.y,p.z], [t.x,t.y,t.z])
    angle = 2 * math.acos(min(1., dot))
    return {'translation_m': distance, 'rotation_rad': angle,
            'valid': distance <= .02 and angle <= .02}


def message_issues(kind, message, profile):
    errors = []
    expected = profile['frames']['laser' if kind == 'scan' else 'odom' if kind == 'odom' else 'optical']
    if message.header.frame_id != expected:
        errors.append(kind + ' header frame 불일치')
    try:
        if stamp_ns(message.header.stamp) <= 0:
            errors.append(kind + ' stamp는 양수여야 합니다')
    except ValueError as exc:
        errors.append(str(exc))
    if kind == 'scan':
        fields = (message.angle_min, message.angle_max, message.angle_increment,
                  message.range_min, message.range_max, message.time_increment, message.scan_time)
        if (not all(math.isfinite(v) for v in fields) or message.angle_increment <= 0
                or message.angle_max <= message.angle_min or message.range_min < 0
                or message.range_max <= message.range_min or message.time_increment < 0
                or message.scan_time < 0 or not message.ranges):
            errors.append('scan geometry/range 계약 오류')
        else:
            if abs((len(message.ranges) - 1) * message.angle_increment
                   - (message.angle_max - message.angle_min)) > message.angle_increment * 1.5:
                errors.append('scan beam 수와 각도 범위 불일치')
            valid = [r for r in message.ranges if math.isfinite(r) and message.range_min <= r <= message.range_max]
            if not valid or any(math.isnan(r) or r == -math.inf or
                                (math.isfinite(r) and not message.range_min <= r <= message.range_max)
                                for r in message.ranges):
                errors.append('scan 유효 거리 없음 또는 잘못된 거리값')
            if (profile['slam']['min_laser_range'] < message.range_min - range_tolerance(message.range_min)
                    or profile['slam']['max_laser_range'] > message.range_max + range_tolerance(message.range_max)):
                errors.append('SLAM laser 범위가 센서 측정 범위를 벗어납니다')
    if kind == 'odom':
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if message.child_frame_id != profile['frames']['base']:
            errors.append('odom child frame 불일치')
        if not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or not quaternion_valid(q):
            errors.append('odom position/quaternion 오류')
    return errors


def stream_summary(samples, now_ros_ns, now_wall, minimum_hz, max_age):
    """벽시계 수신률과 ROS header 최근성을 별도 검사한다."""
    issues = []
    stamps = [s[0] for s in samples]
    rate = ((len(samples) - 1) / (samples[-1][1] - samples[0][1])
            if len(samples) >= 2 and samples[-1][1] > samples[0][1] else 0.0)
    if len(samples) < 3 or any(b <= a for a, b in zip(stamps, stamps[1:])):
        issues.append('표본 부족 또는 stamp 미진행/역행')
    if rate < minimum_hz:
        issues.append('최소 수신률 미달')
    if samples:
        age = (now_ros_ns - stamps[-1]) / 1e9
        if age < -0.1 or age > max_age or now_wall - samples[-1][1] > max_age:
            issues.append('최신 표본이 오래되었거나 미래입니다')
    else:
        age = None
    return {'count': len(samples), 'received_hz_wall': rate,
            'first_stamp_ns': stamps[0] if stamps else None,
            'last_stamp_ns': stamps[-1] if stamps else None,
            'age_sec': age, 'issues': issues}


def fixture_guard(enabled, environ):
    if enabled and (environ.get('ROS_DOMAIN_ID') != '12' or environ.get('GZ_PARTITION') != 'codex'):
        raise ValueError('fixture는 ROS_DOMAIN_ID=12, GZ_PARTITION=codex가 필수입니다')


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--require-camera', action='store_true')
    fixture = parser.add_mutually_exclusive_group()
    fixture.add_argument('--simulation-fixture', action='store_true')
    fixture.add_argument('--synthetic-wall-fixture', action='store_true')
    options = parser.parse_args(args)
    try:
        fixture_guard(options.simulation_fixture or options.synthetic_wall_fixture, os.environ)
        profile, sha = load_profile(options.profile, require_camera=options.require_camera)
        if not finite(options.duration) or not 5 <= options.duration <= 120:
            raise ValueError('duration은 5~120초여야 합니다')
        if options.output.exists() or not options.output.parent.is_dir():
            raise ValueError('출력은 존재하는 폴더의 새 파일이어야 합니다')
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    report = {'schema_version': 1, 'profile': str(Path(options.profile).resolve()),
              'profile_sha256': sha, 'context': ('simulation_fixture' if options.simulation_fixture else
                          'synthetic_wall_fixture' if options.synthetic_wall_fixture else 'hardware'),
              'hardware_verified': False, 'issues': [], 'streams': {}, 'publishers': {}, 'tf': {},
              'limitations': '구동·안전정지·실측 보정·지도 정확도 검증을 대체하지 않음; 수신률은 벽시계 기준'}
    node = None
    initialized = False
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.time import Time
        from rosidl_runtime_py.utilities import get_message
        from tf2_ros import Buffer, TransformListener, TransformException
        from mapping_semantic.core import validate_camera_inputs
        from cv_bridge import CvBridge
        bridge = CvBridge()
        rclpy.init(args=[])
        initialized = True
        node = Node('mapping_robot_preflight', parameter_overrides=[
            Parameter('use_sim_time', value=options.simulation_fixture)])
        buffer = Buffer(node=node)
        listener = TransformListener(buffer, node)
        required = ['scan', 'odom'] + (['rgb', 'depth', 'camera_info'] if options.require_camera else [])
        samples = {k: [] for k in required}
        pending = deque()
        matched_camera = 0
        odom_poses = {}
        pose_errors = []
        camera = {k: {} for k in ('rgb', 'depth', 'camera_info')}
        tf_count = {'odom_base': 0, 'base_laser': 0, 'odom_pose_consistency': 0}
        if options.require_camera:
            tf_count['base_optical'] = 0
        seen_clock = False
        def receive(kind, message):
            nonlocal matched_camera
            report['issues'].extend(message_issues(kind, message, profile))
            timestamp = stamp_ns(message.header.stamp)
            samples[kind].append((timestamp, time.monotonic()))
            if kind == 'odom':
                odom_poses[timestamp] = message.pose.pose
                pending.append(('odom_pose_consistency', profile['frames']['odom'],
                                profile['frames']['base'], message.header.stamp))
            if kind == 'scan':
                pending.append(('odom_base', profile['frames']['odom'], profile['frames']['base'], message.header.stamp))
                pending.append(('base_laser', profile['frames']['base'], profile['frames']['laser'], message.header.stamp))
            if kind in camera:
                camera[kind][timestamp] = message
                if len(camera[kind]) > 100:
                    del camera[kind][next(iter(camera[kind]))]
                policy = profile['semantic']['sync_policy']
                skew = profile['semantic']['max_sync_skew_sec'] if policy == 'approximate' else 0
                chosen = {key: min(values, key=lambda t: abs(t - timestamp))
                          for key, values in camera.items() if values}
                if len(chosen) == 3 and max(chosen.values()) - min(chosen.values()) <= round(skew * 1e9):
                    try:
                        validate_camera_inputs(camera['rgb'][chosen['rgb']], camera['depth'][chosen['depth']],
                                               camera['camera_info'][chosen['camera_info']],
                                               profile['frames']['optical'], policy,
                                               profile['semantic']['max_sync_skew_sec'])
                        rgb_message = camera['rgb'][chosen['rgb']]
                        depth_message = camera['depth'][chosen['depth']]
                        validate_image_storage(rgb_message)
                        validate_image_storage(depth_message)
                        validate_camera_arrays(bridge.imgmsg_to_cv2(rgb_message, desired_encoding='rgb8'),
                                               bridge.imgmsg_to_cv2(depth_message, desired_encoding='passthrough'),
                                               depth_message.encoding)
                        matched_camera += 1
                        pending.append(('base_optical', profile['frames']['base'], profile['frames']['optical'],
                                        camera['rgb'][chosen['rgb']].header.stamp))
                    except ValueError as exc:
                        report['issues'].append('RGB-D 동기 입력 오류: ' + str(exc))
                    for key, values in camera.items():
                        values.pop(chosen[key])
        subscriptions = [node.create_subscription(get_message(TOPIC_TYPES[k]), profile['topics'][k],
                         lambda m, key=k: receive(key, m), qos_profile_sensor_data) for k in required]
        started = time.monotonic()
        while time.monotonic() - started < options.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.count_publishers('/clock'):
                seen_clock = True
            for key in required:
                if node.count_publishers(profile['topics'][key]) > 1:
                    report['issues'].append('중복 발행자: ' + key)
            for _ in range(len(pending)):
                key, target, source, stamp = pending.popleft()
                try:
                    transform = buffer.lookup_transform(target, source, Time.from_msg(stamp)).transform
                    if not quaternion_valid(transform.rotation) or not all(math.isfinite(v) for v in
                        (transform.translation.x, transform.translation.y, transform.translation.z)):
                        report['issues'].append('TF 수치 오류: ' + key)
                    else:
                        if key == 'odom_pose_consistency':
                            error = pose_tf_error(odom_poses.pop(stamp_ns(stamp)), transform)
                            pose_errors.append(error)
                            if not error['valid']:
                                report['issues'].append('동일 시각 odom pose/TF 불일치 (>0.02m 또는 >0.02rad)')
                        tf_count[key] += 1
                except TransformException:
                    pending.append((key, target, source, stamp))
        now, wall = node.get_clock().now().nanoseconds, time.monotonic()
        for key in required:
            minimum = profile['thresholds'].get('min_' + key + '_hz', 0.0)
            result = stream_summary(samples[key], now, wall, minimum, profile['thresholds']['max_age_sec'])
            if samples[key] and samples[key][-1][1] - samples[key][0][1] < options.duration * 0.7:
                result['issues'].append('전체 관측기간의 70% 이상 연속 관측하지 못했습니다')
            report['streams'][key] = result
            report['issues'].extend(key + ': ' + issue for issue in result['issues'])
            endpoints = node.get_publishers_info_by_topic(profile['topics'][key])
            report['publishers'][key] = [{'node': e.node_name, 'type': e.topic_type,
                'reliability': str(e.qos_profile.reliability), 'durability': str(e.qos_profile.durability),
                'depth': e.qos_profile.depth, 'gid': bytes(e.endpoint_gid).hex()} for e in endpoints]
            if len(endpoints) != 1 or any(e.topic_type != TOPIC_TYPES[key] for e in endpoints):
                report['issues'].append(key + ': 발행자 1개 및 타입 계약 불일치')
        if seen_clock and not options.simulation_fixture:
            report['issues'].append('실물 모드에서 /clock 발행자를 발견했습니다')
        if options.simulation_fixture and node.count_publishers('/clock') != 1:
            report['issues'].append('fixture /clock 발행자는 정확히 1개여야 합니다')
        report['tf'] = {'successful_queries': tf_count, 'pending': len(pending),
                        'pose_consistency': {'translation_tolerance_m': .02, 'rotation_tolerance_rad': .02,
                            'sample_count': len(pose_errors),
                            'max_translation_m': max((e['translation_m'] for e in pose_errors), default=None),
                            'max_rotation_rad': max((e['rotation_rad'] for e in pose_errors), default=None)},
                        'edge_authority_verified': False,
                        'authority_limitation': 'Humble 단일 메시지 callback은 발행자 GID를 제공하지 않아 '
                            'odom→base edge 중복 발행 주체는 미검증. TF topic 전체 endpoint만 기록하며 '
                            'pose 일치도 단일 authority의 증명은 아님',
                        'publishers': {topic: [
                            {'node': e.node_name, 'namespace': e.node_namespace,
                             'gid': bytes(e.endpoint_gid).hex(), 'type': e.topic_type,
                             'reliability': str(e.qos_profile.reliability),
                             'durability': str(e.qos_profile.durability)}
                            for e in node.get_publishers_info_by_topic(topic)]
                            for topic in ('/tf', '/tf_static')}}
        if any(v < 3 for v in tf_count.values()) or pending:
            report['issues'].append('센서 시각 TF 조회 부족 또는 미해결 조회')
        report['camera_matches'] = matched_camera
        report['camera_sync_policy'] = profile['semantic']['sync_policy']
        if options.require_camera and matched_camera < 3:
            report['issues'].append('지정 동기화 계약 RGB/depth/CameraInfo 표본 부족')
    except Exception as exc:
        report['issues'].append(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.shutdown()
    report['issues'] = sorted(set(report['issues']))
    report['passed'] = not report['issues']
    report['hardware_verified'] = report['passed'] and not (options.simulation_fixture or options.synthetic_wall_fixture)
    with options.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print('센서 preflight ' + ('통과' if report['passed'] else '실패') + ': ' + str(options.output))
    return 0 if report['passed'] else 1
