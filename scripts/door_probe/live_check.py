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


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid
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
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID'), os.environ.get('GZ_PARTITION')) != ('12', 'codex'):
        parser.error('ROS_DOMAIN_ID=12 GZ_PARTITION=codex 필요')
    if args.output.exists() or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('새 출력 경로와 양수 timeout이 필요합니다')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True)
    root = Path(__file__).resolve().parents[2]
    rclpy.init()
    node = Node('live_door_check')
    bridge = CvBridge()
    data = {'maps': [], 'doors': [], 'detector': [], 'scan_count': 0}
    image = [None]
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
        world = root / f'src/mapping_gazebo/worlds/door_probe_{args.state}.sdf'
        report['world_sha256'] = hashlib.sha256(world.read_bytes()).hexdigest()
        x, y, yaw = spawn_pose(2, 0)
        start('simulation', ['ros2', 'launch', 'mapping_slam', 'simulation.launch.py', 'gui:=false',
                             f'world:={world}', f'x:={x}', f'y:={y}', f'yaw:={yaw}'])
        start('doors', ['ros2', 'launch', 'mapping_semantic', 'live_doors.launch.py',
                        f'recon_root:={args.recon_root.resolve()}', f'map_id:=live-door-probe-{args.state}-{args.output.name}',
                        f'output_path:={args.output / "doors.json"}'])
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            pump(.2)
            if (data['maps'] and data['doors'] and data['detector'] and data['scan_count'] > 5
                    and any(obj['state'] == args.state for obj in data['doors'][-1]['objects'])):
                break
        else:
            raise TimeoutError('LiDAR 지도·실시간 문 상태 확인 시간 초과')
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
            draw.ellipse((u-4, v-4, u+4, v+4), fill='green' if obj['state']=='open' else 'red')
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
        report['source_sha256'] = {name: hashlib.sha256((root / 'src/mapping_semantic/mapping_semantic' / name).read_bytes()).hexdigest()
                                  for name in ['live_detector.py', 'detector_worker.py', 'door_node.py', 'doorway.py', 'door_state.py', 'door_map.py', 'door_position.py']}
        report['harness_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        report['dds_profile'] = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
        (args.output / 'check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
