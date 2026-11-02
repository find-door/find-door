"""저장 RGB-D·C 검출을 실제 ROS로 재생하는 문 mapper 통합 진단. SLAM 시험은 아니다."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from PIL import Image as PILImage


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
    from visualization_msgs.msg import MarkerArray
    from cv_bridge import CvBridge
    from capture import stop_group

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captures', type=Path, required=True)
    parser.add_argument('--detections', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID'), os.environ.get('GZ_PARTITION')) != ('12', 'codex'):
        parser.error('ROS_DOMAIN_ID=12 GZ_PARTITION=codex 필요')
    if args.output.exists():
        parser.error('기존 결과 폴더는 덮어쓰지 않습니다')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True)
    rclpy.init()
    node = Node('door_ros_check')
    discover_until = time.monotonic() + 2
    while time.monotonic() < discover_until:
        rclpy.spin_once(node, timeout_sec=.1)
    if node.count_publishers('/clock'):
        node.destroy_node()
        rclpy.shutdown()
        raise RuntimeError('이 도메인에 이미 /clock 발행자가 있습니다')
    bridge = CvBridge()
    latest, marker_frames = [], []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/semantic/door_map', lambda m: latest.append(json.loads(m.data)), latched)
    node.create_subscription(MarkerArray, '/semantic/door_markers', lambda m: marker_frames.append(m), latched)
    publishers = {name: node.create_publisher(typ, name, 10) for name, typ in [
        ('/clock', Clock), ('/tf', TFMessage), ('/camera/color/image_raw', Image),
        ('/camera/depth/image_raw', Image), ('/camera/camera_info', CameraInfo),
        ('/semantic/detections2d', Detection2DArray)]}
    process = None
    log = (args.output / 'mapper.log').open('w')
    report = {'mode': 'recorded_rgbd_c_replay_synthetic_clock', 'status': 'running',
              'limitations': ['저장된 Gazebo RGB-D와 실제 C 검출 재사용', '인위적 시각의 상태 순서 연결',
                              '항등 map→odom 제공; LiDAR SLAM 지도 시험 아님', 'GUI 수동 확인 아님'], 'observations': []}

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            if process is not None and process.poll() is not None:
                raise RuntimeError('mapper 조기 종료')

    def clock(ns):
        msg = Clock()
        msg.clock.sec, msg.clock.nanosec = divmod(ns, 1_000_000_000)
        publishers['/clock'].publish(msg)
        return msg.clock

    def tf_at(stamp, camera, shift=0.):
        transforms = []
        for parent, child, xyz, q in [('map', 'odom', [shift, 0., 0.], [0., 0., 0., 1.]),
                                     ('odom', 'camera_color_optical_frame', camera['translation'], camera['quaternion'])]:
            msg = TransformStamped()
            msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, parent, child
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = xyz
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = q
            transforms.append(msg)
        publishers['/tf'].publish(TFMessage(transforms=transforms))

    def service(name, expected=True):
        client = node.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError(f'서비스 없음 {name}')
        future = client.call_async(Trigger.Request())
        end = time.monotonic() + 5
        while not future.done() and time.monotonic() < end:
            pump(.05)
        if not future.done() or future.result().success != expected:
            raise RuntimeError(f'서비스 실패 {name}: {future.result() if future.done() else "timeout"}')
        return future.result().message

    try:
        saved_path = args.output / 'doors.json'
        command = ['ros2', 'run', 'mapping_semantic', 'door_mapper', '--ros-args',
                   '-p', 'use_sim_time:=true', '-p', 'map_id:=ros-recording-check',
                   '-p', f'output_path:={saved_path}', '-p', f'input_path:={saved_path}']
        env = os.environ.copy()
        env['ROS_LOG_DIR'] = str(args.output / 'roslogs')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
        pump(3)
        source = json.loads(args.detections.read_text())
        detections = {row['frame_id']: row for row in source['frames']}
        ns = 10_000_000_000
        for state in ['closed', 'open', 'closed']:
            for index in range(3):
                name = f'{state}_d2_a0'
                meta = json.loads((args.captures / name / f'{index:02d}.json').read_text())
                stamp = clock(ns)
                pump(.1)
                tf_at(stamp, meta['odom_from_camera'])
                pump(.1)
                rgb = bridge.cv2_to_imgmsg(np.asarray(PILImage.open(args.captures / name / f'{index:02d}.png')), encoding='rgb8')
                depth = bridge.cv2_to_imgmsg(np.load(args.captures / name / meta['depth'], allow_pickle=False), encoding='32FC1')
                info = CameraInfo(width=meta['width'], height=meta['height'], k=meta['camera_info']['k'],
                                  p=meta['camera_info']['p'], d=meta['camera_info']['d'],
                                  distortion_model=meta['camera_info']['distortion_model'])
                array = Detection2DArray()
                for msg in (rgb, depth, info, array):
                    msg.header.stamp, msg.header.frame_id = stamp, 'camera_color_optical_frame'
                for item in detections[f'{name}_{index:02d}']['door_detections']:
                    detection = Detection2D()
                    detection.header = array.header
                    x0, y0, x1, y1 = item['bbox']
                    detection.bbox.center.position.x = (x0+x1)/2
                    detection.bbox.center.position.y = (y0+y1)/2
                    detection.bbox.size_x, detection.bbox.size_y = x1-x0, y1-y0
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = 'door', item['confidence']
                    detection.results = [hypothesis]
                    array.detections.append(detection)
                for topic, msg in zip(['/camera/color/image_raw', '/camera/depth/image_raw', '/camera/camera_info', '/semantic/detections2d'], [rgb, depth, info, array]):
                    publishers[topic].publish(msg)
                pump(1.)
                if not latest or not latest[-1]['objects']:
                    raise RuntimeError('문 map 출력 없음')
                report['observations'].append(latest[-1])
                ns += 500_000_000
        rows = report['observations']
        ids = {obj['id'] for row in rows for obj in row['objects']}
        if len(ids) != 1 or [rows[i]['objects'][0]['state'] for i in [2, 5, 8]] != ['closed', 'open', 'closed']:
            raise RuntimeError('ID 또는 상태 전환 실패')
        report['save'] = service('/semantic/doors/save')
        report['overwrite_refused'] = service('/semantic/doors/save', expected=False)
        report['load'] = service('/semantic/doors/load')
        # exact timestamp를 유지하되 검출 프레임 이름을 틀리게 하여 입력 거부를 확인한다.
        stamp = clock(ns)
        tf_at(stamp, meta['odom_from_camera'])
        pump(.1)
        for msg in (rgb, depth, info, array):
            msg.header.stamp = stamp
        array.header.frame_id = 'incorrect_optical_frame'
        for topic, msg in zip(['/camera/color/image_raw', '/camera/depth/image_raw', '/camera/camera_info', '/semantic/detections2d'], [rgb, depth, info, array]):
            publishers[topic].publish(msg)
        pump(1.)
        if latest[-1]['counters'].get('input_rejected', 0) < 1:
            raise RuntimeError('잘못된 검출 header를 거부하지 않았습니다')
        report['invalid_header_rejected'] = True
        clock(ns + 3_000_000_000)
        pump(1.)
        if not latest or latest[-1]['objects'][0]['state'] != 'unknown':
            raise RuntimeError('관측 중단 만료 실패')
        if not marker_frames or not any(m.action == 0 for arr in marker_frames for m in arr.markers):
            raise RuntimeError('마커 출력 없음')
        report['expired'] = latest[-1]
        stamp = clock(ns + 4_000_000_000)
        tf_at(stamp, meta['odom_from_camera'], shift=.3)
        pump(1.)
        if latest[-1]['objects'] or latest[-1]['counters'].get('correction_resets', 0) < 1:
            raise RuntimeError('지도 보정 초기화 실패')
        report['correction_reset'] = latest[-1]
        report['incompatible_load_refused'] = service('/semantic/doors/load', expected=False)
        clock(2_000_000_000)
        pump(1.)
        if latest[-1]['objects'] or latest[-1]['counters'].get('clock_resets', 0) < 1:
            raise RuntimeError('시각 역행 초기화 실패')
        report['clock_reset'] = latest[-1]
        # 이전 epoch에 TF가 있던 동일 시각에 영상만 보내도 과거 TF를 재사용하면 안 된다.
        stamp = clock(10_000_000_000)
        for msg in (rgb, depth, info, array):
            msg.header.stamp, msg.header.frame_id = stamp, 'camera_color_optical_frame'
        for detection in array.detections:
            detection.header = array.header
        for topic, msg in zip(['/camera/color/image_raw', '/camera/depth/image_raw', '/camera/camera_info', '/semantic/detections2d'], [rgb, depth, info, array]):
            publishers[topic].publish(msg)
        pump(1.5)
        if latest[-1]['objects'] or latest[-1]['counters'].get('tf_timeout', 0) < 1:
            raise RuntimeError('이전 epoch TF가 재사용되거나 TF 대기 만료가 누락됐습니다')
        report['old_epoch_tf_rejected'] = latest[-1]
        stamp = clock(10_500_000_000)
        moved_camera = {**meta['odom_from_camera'], 'translation': list(meta['odom_from_camera']['translation'])}
        moved_camera['translation'][0] += .4
        tf_at(stamp, moved_camera)
        pump(.1)
        for msg in (rgb, depth, info, array):
            msg.header.stamp = stamp
        for topic, msg in zip(['/camera/color/image_raw', '/camera/depth/image_raw', '/camera/camera_info', '/semantic/detections2d'], [rgb, depth, info, array]):
            publishers[topic].publish(msg)
        pump(1.)
        if not latest[-1]['objects']:
            raise RuntimeError('새 epoch TF 관측이 처리되지 않았습니다')
        expected_x = report['observations'][0]['objects'][0]['position'][0] + .4
        if abs(latest[-1]['objects'][0]['position'][0] - expected_x) > 1e-5:
            raise RuntimeError('새 epoch의 변경된 TF 좌표가 사용되지 않았습니다')
        report.update(status='complete', new_epoch_tf=latest[-1], marker_messages=len(marker_frames))
    except BaseException as exc:
        report.update(status='failed', error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        report['last_map'] = latest[-1] if latest else None
        package = Path(__file__).resolve().parents[2] / 'src/mapping_semantic/mapping_semantic'
        report['source_sha256'] = {name: hashlib.sha256((package / name).read_bytes()).hexdigest()
                                   for name in ['door_node.py', 'doorway.py', 'door_position.py', 'door_state.py', 'door_map.py']}
        report['dds_profile'] = os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
        if process is not None:
            stop_group(process)
        log.close()
        (args.output / 'check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
