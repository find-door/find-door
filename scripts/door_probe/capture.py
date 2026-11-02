"""격리된 Gazebo의 문 상태·거리·관측각별 RGB-D와 관측 시각 TF를 저장한다."""

import argparse
from collections import deque
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
STATES = {'closed': 0, 'half': 45, 'open': 90, 'negative': None}
# 외부 mecanum.xacro + rosorin.gz.xacro의 광학 원점. 실행 시 TF로 재확인한다.
CAMERA_OFFSET_XY = (0.082373, 0.000079091)
TRUTH_CORNERS = np.array([[x, y, z] for x in [-0.105]
                          for y in [-0.58, 0.58] for z in [0., 2.1]])


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    # 부모가 실행 중인 worker 기록을 읽을 때 잘린 JSON을 보지 않게 교체한다.
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def rotation(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('유효한 quaternion이 필요합니다')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def spawn_pose(distance, angle):
    """광학 중심의 수평 거리를 맞추도록 base 스폰 위치를 보정한다."""
    a = math.radians(angle)
    camera = np.array([-distance * math.cos(a), distance * math.sin(a)])
    yaw = -a
    rot = np.array([[math.cos(yaw), -math.sin(yaw)],
                    [math.sin(yaw), math.cos(yaw)]])
    base = camera - rot @ CAMERA_OFFSET_XY
    return [float(base[0]), float(base[1]), yaw]


def projected_truth(translation, quaternion, k, width, height):
    """고정 문틀 영역의 기하 투영. 가림은 판정하지 않고 영상 경계만 자른다."""
    points = (TRUTH_CORNERS - translation) @ rotation(quaternion)
    if np.any(points[:, 2] <= 0):
        raise ValueError('문틀 정답이 카메라 뒤에 있습니다')
    pixels = points @ np.asarray(k).reshape(3, 3).T
    pixels = pixels[:, :2] / pixels[:, 2:]
    raw = [float(pixels[:, 0].min()), float(pixels[:, 1].min()),
           float(pixels[:, 0].max()), float(pixels[:, 1].max())]
    clipped = [max(0., raw[0]), max(0., raw[1]), min(float(width), raw[2]),
               min(float(height), raw[3])]
    if clipped[0] >= clipped[2] or clipped[1] >= clipped[3]:
        raise ValueError('문틀 정답이 영상 밖에 있습니다')
    return clipped, raw, clipped != raw


def optical_heading_error(quaternion, angle):
    """광학 +z 축이 요청한 수평 시선 방향에서 벗어난 3D 각도를 반환한다."""
    a = math.radians(angle)
    expected = np.array([math.cos(a), -math.sin(a), 0.])
    actual = rotation(quaternion)[:, 2]
    return math.degrees(math.acos(float(np.clip(actual @ expected, -1., 1.))))


def process_identity(pid):
    """PID 재사용 방지용 Linux starttime과 process group을 읽는다."""
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    except FileNotFoundError:
        return None
    return {'pgid': int(fields[2]), 'start_ticks': int(fields[19])}


def stop_group(child, expected_start_ticks=None):
    """이번 실행에서 생성한 프로세스 그룹만 정리한다."""
    for sig, seconds in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
        identity = process_identity(child.pid)
        if identity is not None and (identity['pgid'] != child.pid or
                (expected_start_ticks is not None and identity['start_ticks'] != expected_start_ticks)):
            raise RuntimeError(f'프로세스 그룹 {child.pid} 소유권 불일치: 신호를 보내지 않음')
        try:
            os.killpg(child.pid, sig)
        except ProcessLookupError:
            child.wait()
            return
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            child.poll()
            try:
                os.killpg(child.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(.1)
    raise RuntimeError(f'생성한 프로세스 그룹 {child.pid} 종료 실패')


def cleanup_recorded_launch(condition_path, worker_pid):
    """방금 기다린 worker가 기록한 launch만 예외 경로에서 정리한다."""
    if not condition_path.is_file():
        return
    record = json.loads(condition_path.read_text())
    pgid = record.get('launch_pgid')
    if pgid is None:
        return
    if record.get('worker_pid') != worker_pid or not isinstance(record.get('launch_start_ticks'), int):
        raise RuntimeError('worker의 launch 소유권 기록을 확인할 수 없습니다')

    class RecordedGroup:
        pid = pgid

        @staticmethod
        def wait():
            return None

        @staticmethod
        def poll():
            return None

    stop_group(RecordedGroup(), record['launch_start_ticks'])


def run_worker(command, condition_path):
    """부모 중단 시 worker의 finally를 기다린 뒤 자신이 만든 launch를 확인한다."""
    child = subprocess.Popen(command, start_new_session=True)
    try:
        return child.wait()
    except BaseException:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                # worker의 Gazebo 그룹 정리 최대 18초보다 긴 유예를 준다.
                child.wait(timeout=25)
            except subprocess.TimeoutExpired:
                stop_group(child)
        raise
    finally:
        cleanup_recorded_launch(condition_path, child.pid)


def condition_summary(path, name, state, distance, angle, returncode):
    """실패 조건과 이미 저장된 프레임도 전체 시도 목록에 남긴다."""
    condition = (json.loads(path.read_text()) if path.is_file() else
                 {'status': 'failed', 'error': 'worker 조건 기록 없음', 'frames': [],
                  'state': state, 'distance_m': distance, 'view_angle_deg': angle,
                  'door_angle_deg': STATES[state]})
    condition['id'] = name
    condition['worker_returncode'] = returncode
    if returncode != 0 or condition['status'] != 'complete':
        condition['status'] = 'failed'
    for frame in condition['frames']:
        frame['rgb'] = f'{name}/{frame["rgb"]}'
        frame['metadata'] = f'{name}/{frame["metadata"]}'
    return condition


def tf_record(transform):
    t, q = transform.translation, transform.rotation
    return {'translation': [t.x, t.y, t.z], 'quaternion': [q.x, q.y, q.z, q.w]}


def stamp_ns(message):
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def collect_one(options):
    import message_filters
    from cv_bridge import CvBridge
    from PIL import Image as PILImage
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener, TransformException

    out = options.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    world = ROOT / f'src/mapping_gazebo/worlds/door_probe_{options.state}.sdf'
    if not world.is_file():
        raise ValueError(f'world 없음: {world}')
    rclpy.init()
    node = Node('door_probe_capture', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    bridge = CvBridge()
    queue = deque(maxlen=20)
    subscribers = [message_filters.Subscriber(node, typ, topic, qos_profile=qos_profile_sensor_data)
                   for typ, topic in [(Image, '/camera/color/image_raw'),
                                      (Image, '/camera/depth/image_raw'),
                                      (CameraInfo, '/camera/camera_info')]]
    synchronizer = message_filters.TimeSynchronizer(subscribers, 30)
    synchronizer.registerCallback(lambda *messages: queue.append(messages))
    child = None
    frames = []
    record = {'status': 'running', 'state': options.state, 'distance_m': options.distance,
              'view_angle_deg': options.angle, 'door_angle_deg': STATES[options.state],
              'world_sha256': digest(world), 'worker_pid': os.getpid(), 'frames': frames}
    log = (out / 'launch.log').open('w')
    try:
        # DDS graph를 먼저 발견해 다른 실행과 합쳐지는 것을 거부한다.
        until = time.monotonic() + 2
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        if node.count_publishers('/clock'):
            raise RuntimeError('이 도메인에 이미 /clock 발행자가 있습니다')
        x, y, yaw = spawn_pose(options.distance, options.angle)
        command = ['ros2', 'launch', 'mapping_gazebo', 'indoor.launch.py',
                   'gui:=false', 'headless_rendering:=true', f'world:={world}',
                   f'x:={x}', f'y:={y}', f'yaw:={yaw}']
        record['command'] = command
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = str(out / 'roslogs')
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True, env=env)
        record['launch_pgid'] = child.pid
        identity = process_identity(child.pid)
        if identity is None:
            raise RuntimeError('launch PID 기록 전에 프로세스가 종료됨')
        record['launch_start_ticks'] = identity['start_ticks']
        write_json(out / 'condition.json', record)
        deadline = time.monotonic() + options.timeout
        last_stamp = 0
        errors = {}
        while len(frames) < options.frames:
            if child.poll() is not None:
                raise RuntimeError('Gazebo launch 조기 종료: launch.log 확인')
            if time.monotonic() > deadline:
                raise TimeoutError(f'RGB-D 수집 시간 초과; 수집 {len(frames)}, 거부 {errors}')
            rclpy.spin_once(node, timeout_sec=.05)
            if not queue:
                continue
            rgb, depth, info = queue[-1]
            ns = stamp_ns(rgb)
            if ns < 3_000_000_000 or ns - last_stamp < 1_000_000_000:
                continue
            if node.count_publishers('/clock') != 1:
                raise RuntimeError('/clock 발행자는 정확히 하나여야 합니다')
            if not (stamp_ns(depth) == ns == stamp_ns(info)):
                raise RuntimeError('동기화 시각 불일치')
            if not all(m.header.frame_id == 'camera_color_optical_frame' for m in (rgb, depth, info)):
                raise RuntimeError('RGB-D 광학 프레임 불일치')
            if not ((rgb.width, rgb.height) == (depth.width, depth.height) == (info.width, info.height)):
                raise RuntimeError('RGB-D 해상도 불일치')
            k = np.asarray(info.k).reshape(3, 3)
            if (not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
                    or not np.allclose(k[2], [0, 0, 1])
                    or not np.isfinite(info.d).all() or not np.allclose(info.d, 0)
                    or info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
                    or any((info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height))):
                raise RuntimeError('정답 투영에는 무왜곡·비축소 pinhole CameraInfo가 필요합니다')
            try:
                when = Time.from_msg(rgb.header.stamp)
                camera = tf_record(buffer.lookup_transform('odom', 'camera_color_optical_frame', when).transform)
                base = tf_record(buffer.lookup_transform('odom', 'base_footprint', when).transform)
            except TransformException:
                errors['tf_unavailable'] = errors.get('tf_unavailable', 0) + 1
                continue
            actual_distance = math.hypot(*camera['translation'][:2])
            actual_angle = math.degrees(math.atan2(camera['translation'][1], -camera['translation'][0]))
            heading_error = optical_heading_error(camera['quaternion'], options.angle)
            if (abs(actual_distance-options.distance) > .02 or abs(actual_angle-options.angle) > 1
                    or heading_error > 1):
                raise RuntimeError(f'카메라 pose 계약 불일치: 거리 {actual_distance}, '
                                   f'관측 위치 각도 {actual_angle}, 광축 방향 오차 {heading_error}')
            color = bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
            metric = bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
            if depth.encoding == '16UC1':
                metric = metric.astype(np.float32) * .001
            elif depth.encoding != '32FC1':
                raise RuntimeError(f'지원하지 않는 깊이 인코딩 {depth.encoding}')
            valid = np.isfinite(metric) & (metric >= .2) & (metric <= 10)
            if not valid.any():
                raise RuntimeError('유효 깊이가 없는 프레임')
            box, unclipped, truncated = (None, None, False) if options.state == 'negative' else projected_truth(
                camera['translation'], camera['quaternion'], info.k, info.width, info.height)
            index = len(frames)
            rgb_path, depth_path = out / f'{index:02d}.png', out / f'{index:02d}.depth.npy'
            PILImage.fromarray(color).save(rgb_path)
            np.save(depth_path, metric, allow_pickle=False)
            metadata = {
                'environment': 'gazebo', 'stamp_ns': ns, 'frame_id': rgb.header.frame_id,
                'rgb_encoding': rgb.encoding, 'source_depth_encoding': depth.encoding,
                'saved_depth_unit': 'm', 'width': info.width, 'height': info.height,
                'camera_info': {'k': list(info.k), 'd': list(info.d), 'p': list(info.p),
                                'distortion_model': info.distortion_model},
                'odom_from_camera': camera, 'odom_from_base': base,
                'world_odom_contract': 'Gazebo pose odometry: odom origin equals world origin',
                'actual_distance_m': actual_distance, 'actual_view_angle_deg': actual_angle,
                'optical_heading_error_deg': heading_error,
                'truth_bbox_xyxy': box, 'truth_bbox_unclipped_xyxy': unclipped,
                'truth_truncated': truncated,
                'truth_definition': 'fixed doorway outer frame projection; occlusion not assessed',
                'depth_valid_fraction': float(valid.mean()),
                'rgb_sha256': digest(rgb_path), 'depth_sha256': digest(depth_path),
                'depth': depth_path.name,
            }
            meta_path = out / f'{index:02d}.json'
            write_json(meta_path, metadata)
            frames.append({'rgb': rgb_path.name, 'metadata': meta_path.name})
            write_json(out / 'condition.json', record)
            last_stamp = ns
        record['status'] = 'complete'
    except BaseException as exc:
        record['status'] = 'failed'
        record['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        try:
            if child is not None:
                stop_group(child)
        except Exception as exc:
            record['status'] = 'failed'
            record['cleanup_error'] = str(exc)
            raise
        finally:
            log.close()
            write_json(out / 'condition.json', record)
            node.destroy_node()
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--confirm-simulation', action='store_true')
    parser.add_argument('--states', nargs='+', choices=list(STATES), default=list(STATES))
    parser.add_argument('--distances', nargs='+', type=float, default=[1., 2., 3.])
    parser.add_argument('--angles', nargs='+', type=float, default=[0., -30., 30.])
    parser.add_argument('--frames', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--state', choices=list(STATES), help=argparse.SUPPRESS)
    parser.add_argument('--distance', type=float, help=argparse.SUPPRESS)
    parser.add_argument('--angle', type=float, help=argparse.SUPPRESS)
    options = parser.parse_args()
    if not options.confirm_simulation or (os.environ.get('ROS_DOMAIN_ID'), os.environ.get('GZ_PARTITION')) != ('12', 'codex'):
        parser.error('--confirm-simulation 및 ROS_DOMAIN_ID=12 GZ_PARTITION=codex 필요')
    if options.output.exists():
        parser.error('기존 출력 폴더는 덮어쓰지 않습니다')
    if (not 1 <= options.frames <= 30 or not math.isfinite(options.timeout) or options.timeout <= 0
            or any(d not in (1., 2., 3.) for d in options.distances)
            or any(a not in (-30., 0., 30.) for a in options.angles)):
        parser.error('거리 1/2/3, 각도 0/±30, 프레임 1~30, 양수 timeout 필요')
    if options.worker:
        if options.state is None or options.distance not in (1., 2., 3.) or options.angle not in (-30., 0., 30.):
            parser.error('worker 조건 오류')
        collect_one(options)
        return
    combos = list(itertools.product(options.states, options.distances, options.angles))
    if len(combos) != len(set(combos)):
        parser.error('중복 조건은 허용하지 않습니다')
    out = options.output.resolve()
    out.mkdir(parents=True)
    # negative는 같은 거리/각도 행렬로 기록하여 분모를 숨기지 않는다.
    manifest = {'schema_version': 1, 'environment': 'gazebo', 'status': 'running',
                'frames_per_condition': options.frames, 'total_conditions': len(combos),
                'git_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                'git_status': subprocess.check_output(['git', 'status', '--short'], cwd=ROOT, text=True),
                'capture_sha256': digest(__file__), 'conditions': [],
                'planned_conditions': [{'id': f'{state}_d{distance:g}_a{angle:g}', 'state': state,
                                        'distance_m': distance, 'view_angle_deg': angle}
                                       for state, distance, angle in combos],
                'limitations': ['기하 bbox 정답은 가림을 평가하지 않음', '동일 장면 반복 프레임은 독립 환경이 아님',
                                'Gazebo 성능을 실물 성능으로 해석하지 않음']}
    wf = Path('/home/blind/worker_following_ws/src/wf_gazebo')
    manifest['external_hashes'] = {str(p.relative_to(wf)): digest(p) for p in
                                  [wf / 'urdf/mecanum.xacro', wf / 'urdf/rosorin.gz.xacro',
                                   wf / 'config/gz_bridge.yaml', wf / 'launch/spawn_robot.launch.py']}
    write_json(out / 'captures.json', manifest)
    try:
        for state, distance, angle in combos:
            name = f'{state}_d{distance:g}_a{angle:g}'
            print(f'수집 {len(manifest["conditions"])+1}/{len(combos)}: {name}', flush=True)
            command = [sys.executable, str(Path(__file__).resolve()), '--worker', '--confirm-simulation',
                       '--output', str(out / name), '--state', state, '--distance', str(distance),
                       '--angle', str(angle), '--frames', str(options.frames), '--timeout', str(options.timeout)]
            condition_path = out / name / 'condition.json'
            returncode = None
            try:
                returncode = run_worker(command, condition_path)
            finally:
                condition = condition_summary(condition_path, name, state, distance, angle, returncode)
                manifest['conditions'].append(condition)
                write_json(out / 'captures.json', manifest)
            if condition['status'] != 'complete':
                raise RuntimeError(f'{name} 수집 실패 또는 부분 완료, 종료 코드 {returncode}')
        manifest['status'] = 'complete'
    except BaseException as exc:
        manifest['status'] = 'failed'
        manifest['error'] = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        write_json(out / 'captures.json', manifest)


if __name__ == '__main__':
    main()
