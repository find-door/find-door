"""실제 Gazebo RGB-D→YOLOE→문 상태와 LiDAR SLAM을 함께 실행하는 정지 관측 진단."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np
from PIL import Image as PILImage, ImageDraw


def motion_odom(message, now_ns):
    """주행 검사에 쓰는 odom의 프레임·수치·관측 시각을 검증한다."""
    if message is None or message.header.frame_id != 'odom' or message.child_frame_id != 'base_footprint':
        raise ValueError('주행 odom 프레임 불일치')
    p, q, stamp = message.pose.pose.position, message.pose.pose.orientation, message.header.stamp
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
        raise ValueError('주행 odom에 유한하지 않은 좌표가 있습니다')
    if not .99 <= math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w) <= 1.01:
        raise ValueError('주행 odom quaternion 오류')
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('주행 odom 시각 오류')
    ns = stamp.sec*1_000_000_000+stamp.nanosec
    if ns <= 0 or not -100_000_000 <= now_ns-ns <= 500_000_000:
        raise ValueError('주행 odom이 오래됐거나 미래 시각입니다')
    return p.x, p.y


def angle_delta(angle, reference):
    """±π 경계를 넘는 두 yaw의 부호 있는 최소 차이."""
    return math.atan2(math.sin(angle-reference), math.cos(angle-reference))


def motion_yaw(message, now_ns):
    """검증된 odom quaternion에서 정규화된 yaw를 구한다."""
    motion_odom(message, now_ns)
    q = message.pose.pose.orientation
    norm_sq = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
    return math.atan2(2*(q.w*q.z+q.x*q.y), norm_sq-2*(q.y*q.y+q.z*q.z))


def rotation_observation(snapshot, initial, expected_state):
    """Polling한 지도에서 ID·anchor·반대 known 상태를 검사한다."""
    objects = snapshot['objects']
    if len(objects) != 1 or objects[0]['id'] != initial['id']:
        raise RuntimeError('회전 중 문 ID 손실·중복')
    observed = objects[0]
    if observed['position'] != initial['position']:
        raise RuntimeError('회전 중 문 anchor 변경')
    if observed['state'] not in (expected_state, 'unknown'):
        raise RuntimeError('회전 중 반대 known 문 상태 출력')
    return observed


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Image, LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from cv_bridge import CvBridge
    from capture import stop_group, spawn_pose

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', choices=['closed', 'open'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--recon-root', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=120.)
    parser.add_argument('--transition', action='store_true', help='닫힘 시작 후 같은 문을 열고 닫는 실제 센서 전이 검사')
    parser.add_argument('--motion', action='store_true', help='격리된 Gazebo 로봇을 후진·복귀시키는 관측 검사')
    parser.add_argument('--rotate', action='store_true', help='격리된 Gazebo 로봇의 제자리 회전·복귀 관측 검사')
    parser.add_argument('--world', type=Path, help='정지 관측용 별도 world. 기대 상태는 --state로 지정')
    parser.add_argument('--known-door-cycle', action='store_true', help='별도 반열림 world에서 알려진 문의 무검출 재관측 비교')
    parser.add_argument('--reobserve-known-doors', choices=['true', 'false'], default='true')
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID'), os.environ.get('GZ_PARTITION')) != ('12', 'codex'):
        parser.error('ROS_DOMAIN_ID=12 GZ_PARTITION=codex 필요')
    if args.output.exists() or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('새 출력 경로와 양수 timeout이 필요합니다')
    if args.transition and args.state != 'closed':
        parser.error('전이 검사는 --state closed로 시작합니다')
    if sum([args.motion, args.transition, args.rotate, args.known_door_cycle]) > 1:
        parser.error('주행·전이·회전·기존 문 재관측은 별도 실행으로 검사합니다')
    if args.world and (not args.world.is_file() or any([args.motion, args.transition, args.rotate])):
        parser.error('--world는 존재하는 파일로 정지 관측에서만 사용합니다')
    if args.known_door_cycle and (args.world is None or args.state != 'open'):
        parser.error('--known-door-cycle은 별도 half world와 --state open이 필요합니다')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True)
    root = Path(__file__).resolve().parents[2]
    rclpy.init()
    node = Node('live_door_check')
    bridge = CvBridge()
    data = {'maps': [], 'doors': [], 'detector': [], 'scan_count': 0}
    image = [None]
    odom = [None]
    command_pub = node.create_publisher(Twist, '/cmd_vel', 10) if args.motion or args.rotate else None
    motion_control_started = False
    if args.motion or args.rotate:
        node.create_subscription(Odometry, '/odom', lambda m: odom.__setitem__(0, m), qos_profile_sensor_data)
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', lambda m: data['maps'].append(m), transient)
    node.create_subscription(String, '/semantic/door_map', lambda m: data['doors'].append(json.loads(m.data)), transient)
    node.create_subscription(String, '/doors/detector_status', lambda m: data['detector'].append(json.loads(m.data)), 10)
    node.create_subscription(Image, '/camera/color/image_raw', lambda m: image.__setitem__(0, m), qos_profile_sensor_data)

    def scan(message):
        data['scan_count'] += 1

    node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data)
    children, logs = [], []
    report = {'state': args.state, 'mode': 'live_gazebo_yoloe_lidar_slam_stationary', 'status': 'running',
              'limitations': ['단일 문 모델·2m 정면 정지 관측', '지도 정확도·실물·독립 장소 성능 시험 아님',
                              '상태별 별도 시뮬레이션이며 실제 문 상태 전이 시험 아님']}

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if any(child.poll() is not None for child in children):
                raise RuntimeError('실행 프로세스 조기 종료: 로그 확인')

    def start(label, command):
        handle = (args.output / f'{label}.log').open('w')
        logs.append(handle)
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = str(args.output / 'roslogs')
        child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        children.append(child)
        report[label] = {'command': command, 'pgid': child.pid}

    try:
        pump(2)
        if node.count_publishers('/clock'):
            raise RuntimeError('도메인에 기존 /clock 발행자 존재')
        world = args.world.resolve() if args.world else root / f'src/mapping_gazebo/worlds/door_probe_{args.state}.sdf'
        report['world_path'] = str(world)
        report['world_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
        x, y, yaw = spawn_pose(2, 0)
        start('simulation', ['ros2', 'launch', 'mapping_slam', 'simulation.launch.py', 'gui:=false',
                             f'world:={world}', f'x:={x}', f'y:={y}', f'yaw:={yaw}'])
        start('doors', ['ros2', 'launch', 'mapping_semantic', 'live_doors.launch.py',
                        f'recon_root:={args.recon_root.resolve()}', f'reobserve_known_doors:={args.reobserve_known_doors}', f'map_id:=live-door-probe-{args.state}-{args.output.name}',
                        f'output_path:={args.output / "doors.json"}'])
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            pump(.2)
            if (data['maps'] and data['doors'] and data['detector'] and data['scan_count'] > 5
                    and any(obj['state'] == args.state for obj in data['doors'][-1]['objects'])):
                break
        else:
            raise TimeoutError('LiDAR 지도·실시간 문 상태 확인 시간 초과')
        if args.motion:
            if odom[0] is None:
                raise RuntimeError('주행 odom 입력 없음')
            initial = next(obj for obj in data['doors'][-1]['objects'] if obj['state'] == args.state)
            origin_x, origin_y = motion_odom(odom[0], data['doors'][-1]['stamp_ns'])
            report.update(mode='live_gazebo_yoloe_lidar_slam_motion', motion=[],
                          limitations=['단일 문 모델·정면 후진과 복귀', 'Gazebo pose odom, 휠 오차·실물·독립 장소 시험 아님'])
            for label, velocity in [('retreat', -.1), ('return', .1)]:
                begin_ns = data['doors'][-1]['stamp_ns']
                begin_x = motion_odom(odom[0], begin_ns)[0]
                stage = {'name': label, 'command_mps': velocity, 'duration_sim_sec': 5.,
                         'begin_sim_ns': begin_ns, 'begin_odom_x': begin_x, 'history': []}
                report['motion'].append(stage)
                deadline = time.monotonic() + 30
                while (data['doors'][-1]['stamp_ns']-begin_ns)/1e9 < 5.:
                    if time.monotonic() > deadline:
                        raise TimeoutError('주행 중 시뮬레이션 시각 정체')
                    message = Twist()
                    message.linear.x = velocity
                    motion_control_started = True
                    command_pub.publish(message)
                    pump(.1)
                    current = data['doors'][-1]
                    px, py = motion_odom(odom[0], current['stamp_ns'])
                    stage['history'].append({'map': current, 'odom_xy': [px, py]})
                    if len(current['objects']) != 1 or current['objects'][0]['id'] != initial['id']:
                        raise RuntimeError('주행 중 문 ID 손실·중복')
                    if current['objects'][0]['position'] != initial['position']:
                        raise RuntimeError('주행 중 문 anchor 위치 변경')
                    if abs(py-origin_y) > .1:
                        raise RuntimeError('예상 직선 경로에서 벗어남')
                command_pub.publish(Twist())
                pump(.4)
                stage['stop_boundary_sim_ns'] = data['doors'][-1]['stamp_ns']
                stage['end_odom_x'] = motion_odom(odom[0], stage['stop_boundary_sim_ns'])[0]
                stage['travel_x_m'] = stage['end_odom_x']-begin_x
                if abs(stage['travel_x_m']-velocity*5) > .1:
                    raise RuntimeError('실제 odom 이동량이 목표 허용 범위 밖입니다')
                settle = time.monotonic() + 10
                while time.monotonic() < settle:
                    current = data['doors'][-1]
                    if (current['objects'] and current['objects'][0]['id']==initial['id']
                            and current['objects'][0]['state']==args.state
                            and current['objects'][0]['last_stamp_ns']>stage['stop_boundary_sim_ns']):
                        stage['final'] = current
                        break
                    pump(.1)
                else:
                    raise TimeoutError('주행 후 새 관측의 문 상태 확인 실패')
            report['return_error_m'] = abs(motion_odom(odom[0], data['doors'][-1]['stamp_ns'])[0]-origin_x)
            if report['return_error_m'] > .1:
                raise RuntimeError('초기 위치 복귀 오차 초과')
        if args.rotate:
            initial = next(obj for obj in data['doors'][-1]['objects'] if obj['state'] == args.state)
            now_ns = data['doors'][-1]['stamp_ns']
            origin_x, origin_y = motion_odom(odom[0], now_ns)
            origin_yaw = motion_yaw(odom[0], now_ns)
            report.update(mode='live_gazebo_yoloe_lidar_slam_rotate', rotation=[],
                          limitations=['단일 문 모델·2m 정면 시작·제자리 약 14도 회전과 복귀',
                                       'Gazebo pose odom, 휠 오차·독립 장소·실물 시험 아님'])
            for label, velocity in [('turn_left', .1), ('return', -.1)]:
                begin_ns = data['doors'][-1]['stamp_ns']
                begin_yaw = motion_yaw(odom[0], begin_ns)
                stage = dict(name=label, command_radps=velocity, duration_sim_sec=2.5,
                             begin_sim_ns=begin_ns, begin_odom_yaw=begin_yaw, history=[])
                report['rotation'].append(stage)
                deadline = time.monotonic() + 30
                while (data['doors'][-1]['stamp_ns']-begin_ns)/1e9 < 2.5:
                    if time.monotonic() > deadline:
                        raise TimeoutError('회전 중 시뮬레이션 시각 정체')
                    message = Twist()
                    message.angular.z = velocity
                    motion_control_started = True
                    command_pub.publish(message)
                    pump(.1)
                    current = data['doors'][-1]
                    px, py = motion_odom(odom[0], current['stamp_ns'])
                    yaw = motion_yaw(odom[0], current['stamp_ns'])
                    stage['history'].append(dict(map=current, odom_xy=[px, py], odom_yaw=yaw))
                    rotation_observation(current, initial, args.state)
                    if math.hypot(px-origin_x, py-origin_y) > .1:
                        raise RuntimeError('제자리 회전 중 위치 이탈')
                command_pub.publish(Twist())
                pump(.4)
                stage['stop_boundary_sim_ns'] = data['doors'][-1]['stamp_ns']
                stage['end_odom_yaw'] = motion_yaw(odom[0], stage['stop_boundary_sim_ns'])
                stage['travel_yaw_rad'] = angle_delta(stage['end_odom_yaw'], begin_yaw)
                if abs(stage['travel_yaw_rad']-velocity*2.5) > .04:
                    raise RuntimeError('실제 odom 회전량이 목표 허용 범위 밖입니다')
                # 정지 후 얻은 새 관측만 회전 구간의 성공 근거로 인정한다.
                stage['settle_history'] = []
                settle = time.monotonic() + 10
                while time.monotonic() < settle:
                    current = data['doors'][-1]
                    stage['settle_history'].append(current)
                    rotation_observation(current, initial, args.state)
                    if (len(current['objects']) == 1 and current['objects'][0]['id'] == initial['id']
                            and current['objects'][0]['position'] == initial['position']
                            and current['objects'][0]['state'] == args.state
                            and current['objects'][0]['last_stamp_ns'] > stage['stop_boundary_sim_ns']):
                        stage['final'] = current
                        break
                    pump(.1)
                else:
                    raise TimeoutError('회전 정지 후 새 관측의 문 상태 확인 실패')
                if image[0] is not None:
                    path = args.output / f'{label}-rgb.png'
                    PILImage.fromarray(bridge.imgmsg_to_cv2(image[0], desired_encoding='rgb8')).save(path)
                    stage['rgb'] = path.name
                    stage['rgb_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
                    stage['rgb_stamp_ns'] = image[0].header.stamp.sec*1_000_000_000+image[0].header.stamp.nanosec
            now_ns = data['doors'][-1]['stamp_ns']
            report['return_yaw_error_rad'] = abs(angle_delta(motion_yaw(odom[0], now_ns), origin_yaw))
            px, py = motion_odom(odom[0], now_ns)
            report['return_xy_error_m'] = math.hypot(px-origin_x, py-origin_y)
            if report['return_yaw_error_rad'] > .04 or report['return_xy_error_m'] > .1:
                raise RuntimeError('초기 방향·위치 복귀 오차 초과')
        if args.known_door_cycle:
            import xml.etree.ElementTree as ET
            xml = ET.parse(world).getroot().find('world')
            leaf = xml.find("model[@name='door_leaf']")
            hinge = list(map(float, leaf.findtext('pose').split()))[:3]
            initial = next(obj for obj in data['doors'][-1]['objects'] if obj['state'] == 'open')
            enabled = args.reobserve_known_doors == 'true'
            report.update(mode='known_door_rgbd_reobservation', initial_state='open',
                          reobserve_known_doors=enabled, known_door_cycle=[],
                          limitations=['단일 반열림 문에서 먼저 검출 확인한 뒤 순간 각도 변경',
                                       '처음 보는 열린 문 검출·독립 장소·실물·연속 경첩 시험 아님'])
            for expected, angle in [('open', -math.pi/2), ('closed', 0.)]:
                begin_ns = data['doors'][-1]['stamp_ns']
                initial_reobserved = data['doors'][-1]['counters'].get('reobserved', 0)
                initial_no_detection = data['doors'][-1]['counters'].get('no_detection', 0)
                request = (f'name: "door_leaf", position: {{x: {hinge[0]}, y: {hinge[1]}, z: {hinge[2]}}}, '
                           f'orientation: {{z: {math.sin(angle/2)}, w: {math.cos(angle/2)}}}')
                command = ['gz', 'service', '-s', f'/world/{xml.get("name")}/set_pose', '--reqtype', 'gz.msgs.Pose',
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request]
                action = subprocess.run(command, capture_output=True, text=True, timeout=10)
                event = dict(expected_state=expected, required_output=expected if enabled else 'unknown',
                             begin_sim_ns=begin_ns, command=command, stdout=action.stdout,
                             stderr=action.stderr, history=[])
                report['known_door_cycle'].append(event)
                if action.returncode or 'data: true' not in action.stdout:
                    raise RuntimeError('기존 문 각도 변경 실패')
                deadline = time.monotonic() + args.timeout
                while time.monotonic() < deadline:
                    pump(.1)
                    current = data['doors'][-1]
                    event['history'].append(current)
                    if len(current['objects']) != 1 or current['objects'][0]['id'] != initial['id']:
                        raise RuntimeError('기존 문 재관측 중 ID 손실·중복')
                    observed = current['objects'][0]
                    if observed['position'] != initial['position']:
                        raise RuntimeError('기존 문 재관측 중 anchor 변경')
                    reobserved = current['counters'].get('reobserved', 0)-initial_reobserved
                    no_detection = current['counters'].get('no_detection', 0)-initial_no_detection
                    age_ns = current['stamp_ns']-observed['last_stamp_ns']
                    fresh = (observed['last_stamp_ns'] > begin_ns and age_ns < 500_000_000) if enabled else age_ns > 2_000_000_000
                    gain_ok = reobserved > 0 if enabled else reobserved == 0
                    if (current['stamp_ns']-begin_ns >= 3_000_000_000 and no_detection > 0 and gain_ok and fresh
                            and observed['state'] == event['required_output']):
                        event.update(final=current, reobserved_gain=reobserved, no_detection_gain=no_detection,
                                     unknown_seen=any(h['objects'][0]['state']=='unknown' for h in event['history']),
                                     elapsed_sim_sec=(current['stamp_ns']-begin_ns)/1e9)
                        if image[0] is not None:
                            path = args.output/f'known-{expected}-rgb.png'
                            PILImage.fromarray(bridge.imgmsg_to_cv2(image[0], desired_encoding='rgb8')).save(path)
                            event['rgb'] = path.name
                            event['rgb_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
                            event['rgb_stamp_ns'] = image[0].header.stamp.sec*1_000_000_000+image[0].header.stamp.nanosec
                        break
                else:
                    raise TimeoutError('기존 문 무검출 재관측 조건 확인 시간 초과')
            report['final_truth_state'] = 'closed'
        if args.transition:
            initial = next(obj for obj in data['doors'][-1]['objects'] if obj['state'] == 'closed')
            if image[0] is not None:
                PILImage.fromarray(bridge.imgmsg_to_cv2(image[0], desired_encoding='rgb8')).save(args.output / 'initial-rgb.png')
            report['mode'] = 'live_gazebo_yoloe_lidar_slam_door_transition'
            report['limitations'] = ['단일 문 모델·2m 정면·로봇 정지', '지도 정확도·실물·독립 장소 시험 아님']
            report['transitions'] = []
            for expected, angle in [('open', -math.pi/2), ('closed', 0.)]:
                begin_ns = data['doors'][-1]['stamp_ns']
                begin_wall = time.monotonic()
                request = (f'name: "door_leaf", position: {{x: 0, y: -0.5, z: 0}}, '
                           f'orientation: {{z: {math.sin(angle/2)}, w: {math.cos(angle/2)}}}')
                command = ['gz', 'service', '-s', '/world/door_probe_v1/set_pose', '--reqtype', 'gz.msgs.Pose',
                           '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', request]
                action = subprocess.run(command, capture_output=True, text=True, timeout=10)
                event = {'expected_state': expected, 'command': command, 'stdout': action.stdout,
                         'stderr': action.stderr, 'begin_sim_ns': begin_ns}
                report['transitions'].append(event)
                if action.returncode or 'data: true' not in action.stdout:
                    raise RuntimeError(f'문 pose 변경 실패: {action.stdout} {action.stderr}')
                history = []
                end = time.monotonic() + args.timeout
                while time.monotonic() < end:
                    pump(.1)
                    current = data['doors'][-1]
                    objects = current['objects']
                    history.append(current)
                    matching = [obj for obj in objects if obj['id'] == initial['id']]
                    if len(objects) != 1 or len(matching) != 1:
                        event['history'] = history
                        raise RuntimeError('전이 중 문 ID 손실·중복 발생')
                    observed = matching[0]
                    if observed['position'] != initial['position']:
                        raise RuntimeError('전이 중 고정 문 위치 변경')
                    if observed['state'] == expected and observed['last_stamp_ns'] > begin_ns:
                        event.update(history=history, final=observed,
                                     confirmation_wall_sec=time.monotonic()-begin_wall,
                                     confirmation_sim_sec=(observed['last_stamp_ns']-begin_ns)/1e9,
                                     unknown_seen=any(any(o['state']=='unknown' for o in h['objects']) for h in history))
                        if image[0] is not None:
                            path = args.output / f'transition-{expected}-rgb.png'
                            PILImage.fromarray(bridge.imgmsg_to_cv2(image[0], desired_encoding='rgb8')).save(path)
                            event['rgb'] = path.name
                            event['rgb_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
                            event['rgb_stamp_ns'] = image[0].header.stamp.sec*1_000_000_000+image[0].header.stamp.nanosec
                        break
                else:
                    event['history'] = history
                    raise TimeoutError(f'문 {expected} 상태 확정 시간 초과')
        if node.count_publishers('/clock') != 1:
            raise RuntimeError('/clock 발행자 수 오류')
        grid = data['maps'][-1]
        if grid.info.width <= 0 or grid.info.height <= 0 or not any(v >= 0 for v in grid.data):
            raise RuntimeError('알려진 셀이 있는 지도 없음')
        endpoints = node.get_publishers_info_by_topic('/map')
        if len(endpoints) != 1 or endpoints[0].node_name != 'slam_toolbox':
            raise RuntimeError('실제 slam_toolbox 지도 발행자를 확인할 수 없습니다')
        client = node.create_client(Trigger, '/semantic/doors/save')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('문 저장 서비스 없음')
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            pump(.1)
        if not future.done() or not future.result().success:
            raise RuntimeError('문 저장 실패')
        report.update(status='complete', door_map=data['doors'][-1], detector_status=data['detector'][-1],
                      scan_count=data['scan_count'], map_publishers=[p.node_name for p in endpoints],
                      map_updates=len(data['maps']), clock_publishers=node.count_publishers('/clock'))
        occupancy = np.array(grid.data).reshape(grid.info.height, grid.info.width)
        pixels = np.where(occupancy < 0, 205, 254*(1-occupancy/100)).clip(0, 255).astype(np.uint8)
        canvas = PILImage.new('RGB', (grid.info.width*4+80, grid.info.height*4+80), (205, 205, 205))
        raw_map = PILImage.fromarray(np.flipud(pixels)).convert('RGB').resize((grid.info.width*4, grid.info.height*4), PILImage.NEAREST)
        canvas.paste(raw_map, (40, 40))
        draw = ImageDraw.Draw(canvas)
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        q = grid.info.origin.orientation
        theta = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        for obj in report['door_map']['objects']:
            px, py = obj['position'][0]-ox, obj['position'][1]-oy
            gx = (math.cos(theta)*px+math.sin(theta)*py)/grid.info.resolution
            gy = (-math.sin(theta)*px+math.cos(theta)*py)/grid.info.resolution
            u, v = gx*4+40, (grid.info.height-1-gy)*4+40
            draw.ellipse((u-4, v-4, u+4, v+4), fill={'open':'green', 'closed':'red', 'unknown':'orange'}[obj['state']])
            draw.text((u+5, v+5), obj['state'], fill='blue')
        canvas.save(args.output / 'map-doors.png')
        report['map_metadata'] = {'width': grid.info.width, 'height': grid.info.height,
                                  'resolution': grid.info.resolution, 'origin': [ox, oy, theta]}
        np.save(args.output / 'occupancy.npy', occupancy, allow_pickle=False)
        if image[0] is not None:
            PILImage.fromarray(bridge.imgmsg_to_cv2(image[0], desired_encoding='rgb8')).save(args.output / 'rgb.png')
            report['rgb_stamp_ns'] = image[0].header.stamp.sec*1_000_000_000+image[0].header.stamp.nanosec
    except BaseException as exc:
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}',
                      last_door_map=data['doors'][-1] if data['doors'] else None,
                      last_detector=data['detector'][-1] if data['detector'] else None)
        raise
    finally:
        cleanup_errors = []
        if command_pub is not None and motion_control_started and rclpy.ok():
            try:
                command_pub.publish(Twist())
            except Exception as exc:
                cleanup_errors.append(f'정지 명령 실패: {exc}')
        for child in reversed(children):
            try:
                stop_group(child)
            except Exception as exc:
                cleanup_errors.append(str(exc))
        for handle in logs:
            handle.close()
        status = report.get('detector_status') or (data['detector'][-1] if data['detector'] else {})
        pgid, start_ticks = status.get('worker_pgid'), status.get('worker_start_ticks')
        if type(pgid) is int and type(start_ticks) is int:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                report['worker_group_gone'] = True
            else:
                cleanup_errors.append(f'검출 worker 그룹 {pgid} 정리 지연')
                from capture import cleanup_recorded_launch
                record = args.output / 'worker-cleanup.json'
                record.write_text(json.dumps(dict(launch_pgid=pgid, launch_start_ticks=start_ticks, worker_pid=os.getpid())))
                cleanup_recorded_launch(record, os.getpid())
        if cleanup_errors:
            report.update(status='failed', cleanup_errors=cleanup_errors)
        if report['status'] == 'failed' and image[0] is not None:
            try:
                path = args.output / 'failure-rgb.png'
                PILImage.fromarray(bridge.imgmsg_to_cv2(image[0], desired_encoding='rgb8')).save(path)
                report['failure_rgb'] = dict(path=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                                            stamp_ns=image[0].header.stamp.sec*1_000_000_000+image[0].header.stamp.nanosec)
            except Exception as exc:
                report.setdefault('cleanup_errors', []).append(f'실패 영상 보존 오류: {exc}')
        report['source_sha256'] = {name: hashlib.sha256((root / 'src/mapping_semantic/mapping_semantic' / name).read_bytes()).hexdigest()
                                  for name in ['live_detector.py', 'detector_worker.py', 'door_node.py', 'doorway.py', 'door_state.py', 'door_map.py', 'door_position.py', 'door_reobservation.py']}
        report['harness_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        report['dds_profile'] = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
        (args.output / 'check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
