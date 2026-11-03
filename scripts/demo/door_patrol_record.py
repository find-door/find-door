"""전용 Xephyr의 실제 Gazebo·RViz와 ROS 순찰 출력을 통합 MP4로 녹화한다.

GUI나 시뮬레이션을 시작하지 않는다. 준비가 끝나면 recording-ready.json을 쓰고,
지정 stop-file을 받으면 녹화를 마친다. 순찰·문 인식 성공 여부의 채점은 호출자 몫이다.
"""

import argparse
from collections import Counter
from copy import deepcopy
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import time


OUTPUTS = ('find-door-patrol-gazebo-rviz.mp4', 'frames.json',
           'door-map-history.json', 'recording-checks.json', 'recording-ready.json')


def write_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def dedicated_display(display):
    """사용자의 기본 데스크톱 대신 명시한 Xephyr 화면만 허용한다."""
    if re.fullmatch(r':[1-9][0-9]*(?:\.0)?', display) is None:
        raise ValueError('로컬 전용 Xephyr DISPLAY가 필요합니다')
    requested = display.split('.')[0]
    commands = subprocess.run(['ps', '-C', 'Xephyr', '-o', 'args='],
                              capture_output=True, text=True, check=False)
    for line in commands.stdout.splitlines():
        words = shlex.split(line)
        if words and Path(words[0]).name == 'Xephyr' and requested in words[1:]:
            return
    raise ValueError('요청한 DISPLAY의 전용 Xephyr 프로세스가 없습니다')


def arrange_windows(display):
    tree = subprocess.check_output(['xwininfo', '-display', display, '-root', '-tree'], text=True)
    gazebo = re.findall(r'(0x[0-9a-f]+) "Gazebo Sim"[^\n]*? (\d+)x(\d+)\+', tree)
    rviz = re.findall(r'(0x[0-9a-f]+) "[^"\n]* - RViz"[^\n]*? (\d+)x(\d+)\+', tree)
    gazebo = [row for row in gazebo if int(row[1]) > 100 and int(row[2]) > 100]
    rviz = [row for row in rviz if int(row[1]) > 100 and int(row[2]) > 100]
    if len(gazebo) != 1 or len(rviz) != 1:
        raise ValueError('전용 화면에 Gazebo Sim·RViz 창이 각각 하나 있어야 합니다')
    xlib = ctypes.CDLL('libX11.so.6')
    xlib.XOpenDisplay.argtypes = [ctypes.c_char_p]
    xlib.XOpenDisplay.restype = ctypes.c_void_p
    xlib.XCloseDisplay.argtypes = [ctypes.c_void_p]
    xlib.XMoveResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                                     ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
    xlib.XFlush.argtypes = [ctypes.c_void_p]
    connection = xlib.XOpenDisplay(display.encode())
    if not connection:
        raise ValueError('전용 X 화면에 연결할 수 없습니다')
    try:
        for column, row in enumerate([gazebo[0], rviz[0]]):
            xlib.XMoveResizeWindow(connection, int(row[0], 16), column * 960, 0, 960, 840)
        xlib.XFlush(connection)
    finally:
        xlib.XCloseDisplay(connection)
    return dict(gazebo_xid=gazebo[0][0], rviz_xid=rviz[0][0], display=display)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--expected-map-id', required=True)
    parser.add_argument('--display', default=':92')
    parser.add_argument('--fps', type=int, default=10)
    parser.add_argument('--timeout', type=float, default=600.)
    parser.add_argument('--stop-file', type=Path, required=True)
    args = parser.parse_args()
    if args.fps < 1 or args.fps > 30 or not math.isfinite(args.timeout) or args.timeout < 10:
        parser.error('fps는 1~30, timeout은 유한한 10초 이상이어야 합니다')
    out = args.output_dir.expanduser().resolve()
    stop = args.stop_file.expanduser().resolve()
    if not out.is_dir() or not args.expected_map_id.strip():
        parser.error('존재하는 새 출력 디렉터리와 map_id가 필요합니다')
    if stop.exists() or any((out / name).exists() for name in OUTPUTS):
        parser.error('기존 녹화·종료 파일을 덮어쓰지 않습니다')
    if (os.environ.get('ROS_DOMAIN_ID'), os.environ.get('GZ_PARTITION')) != ('12', 'codex'):
        parser.error('ROS_DOMAIN_ID=12·GZ_PARTITION=codex 격리 환경이 필요합니다')
    dedicated_display(args.display)
    os.environ['DISPLAY'] = args.display
    windows = arrange_windows(args.display)

    # --help·인자 검사에는 ROS·Qt·인코더가 필요하지 않다.
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import Image as ImageMsg
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtGui import QImage
    from PIL import Image, ImageDraw, ImageFont

    app = QApplication([])
    rclpy.init()
    node = Node('door_patrol_video_recorder')
    current = dict(rgb=None, map=None, detector=None, patrol=None)
    history, frames, errors = [], [], []
    bridge = CvBridge()
    writer = None
    started = ended = None
    encoded = filled = captures = 0
    last_pixels = last_record = None
    seen_stages = set()
    endpoints = {}
    checks = dict(status='failed', reason='녹화 준비 중', **windows)

    def camera(message):
        current['rgb'] = (bridge.imgmsg_to_cv2(message, desired_encoding='rgb8'),
                          message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)

    def door_map(message):
        value = json.loads(message.data)
        if value.get('map_id') != args.expected_map_id:
            raise ValueError('다른 map_id 수신: 전용 도메인을 확인하세요')
        if not isinstance(value.get('objects'), list):
            raise ValueError('문 지도 객체 목록 오류')
        current['map'] = value
        history.append(dict(wall_ns=time.time_ns(), value=value))

    def detector(message):
        current['detector'] = json.loads(message.data)

    def patrol(message):
        value = json.loads(message.data)
        if not isinstance(value, dict):
            raise ValueError('순찰 상태는 JSON 객체여야 합니다')
        current['patrol'] = value

    node.create_subscription(ImageMsg, '/camera/color/image_raw', camera, qos_profile_sensor_data)
    node.create_subscription(String, '/semantic/door_map', door_map,
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/doors/detector_status', detector, 10)
    node.create_subscription(String, '/demo/patrol_status', patrol, 10)

    def spin(timeout=0.):
        rclpy.spin_once(node, timeout_sec=timeout)
        app.processEvents()

    def assert_endpoints():
        publishers = [v.node_name for v in node.get_publishers_info_by_topic('/map')]
        marker_subscriptions = [v.node_name for v in
                                node.get_subscriptions_info_by_topic('/semantic/door_markers')]
        if node.count_publishers('/clock') != 1:
            raise RuntimeError('/clock 발행자는 1개여야 합니다')
        if publishers != ['slam_toolbox']:
            raise RuntimeError('실제 slam_toolbox 지도 발행자 1개가 필요합니다')
        if not any(name in ('rviz', 'rviz2') for name in marker_subscriptions):
            raise RuntimeError('RViz의 실제 문 마커 구독을 확인할 수 없습니다')
        return dict(clock_publishers=1, map_publishers=publishers,
                    rviz_marker_subscriptions=marker_subscriptions)

    def encode(pixels, record, duplicate=False):
        nonlocal encoded, filled
        writer.write(pixels)
        row = deepcopy(record)
        row.update(frame=encoded, video_time_sec=encoded / args.fps, duplicated_capture=duplicate)
        frames.append(row)
        encoded += 1
        filled += int(duplicate)

    try:
        deadline = time.monotonic() + min(60., args.timeout)
        while time.monotonic() < deadline:
            spin(.05)
            if stop.exists():
                raise RuntimeError('준비 중 녹화 종료 요청 수신')
            if current['rgb'] is not None and current['map'] is not None and current['detector']:
                try:
                    endpoints = assert_endpoints()
                    break
                except RuntimeError:
                    continue
        else:
            raise TimeoutError('녹화 전 RGB·문 지도·검출·GUI 구독 확인 시간 초과')
        screen = app.primaryScreen()
        if screen is None or (screen.size().width(), screen.size().height()) != (1920, 840):
            raise RuntimeError('전용 Xephyr 화면은 1920×840이어야 합니다')
        fontpath = '/usr/share/fonts/truetype/nanum/NanumGothic.ttf'
        large = ImageFont.truetype(fontpath, 31)
        medium = ImageFont.truetype(fontpath, 25)
        small = ImageFont.truetype(fontpath, 21)
        video = out / OUTPUTS[0]
        escaped = str(video).replace('\\', '\\\\').replace('"', '\\"')
        pipeline = ('appsrc ! videoconvert ! video/x-raw,format=I420 ! '
                    'x264enc tune=zerolatency speed-preset=veryfast bitrate=6500 key-int-max=20 ! '
                    'video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! '
                    f'filesink location="{escaped}"')
        writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, args.fps, (1920, 1080), True)
        if not writer.isOpened():
            raise RuntimeError('H.264 MP4 인코더 초기화 실패')
        started = time.monotonic()
        write_json(out / 'recording-ready.json', dict(wall_ns=time.time_ns(), map_id=args.expected_map_id,
                                                     fps=args.fps, **windows, **endpoints))
        next_check, next_progress = 0., 0.
        while not stop.exists():
            elapsed = time.monotonic() - started
            if elapsed >= args.timeout:
                raise TimeoutError('순찰 녹화 종료 파일을 기다리다 timeout 초과')
            target = started + encoded / args.fps
            if time.monotonic() < target:
                spin(min(.01, target - time.monotonic()))
                continue
            for _ in range(3):
                spin()
            if elapsed >= next_check:
                endpoints = assert_endpoints()
                dedicated_display(args.display)
                next_check = elapsed + 5
            capture_wall = time.time_ns()
            q = screen.grabWindow(0).toImage().convertToFormat(QImage.Format_RGB888)
            if (q.width(), q.height()) != (1920, 840) or q.isNull():
                raise RuntimeError('전용 화면 크기 변경 또는 캡처 실패')
            image = Image.frombytes('RGB', (q.width(), q.height()),
                                    q.bits().asstring(q.byteCount()), 'raw', 'RGB', q.bytesPerLine())
            canvas = Image.new('RGB', (1920, 1080), (18, 25, 34))
            canvas.paste(image, (0, 80))
            draw = ImageDraw.Draw(canvas)
            draw.text((22, 12), 'Find Door · 건물 내부 순찰과 문 지도', font=large, fill='white')
            draw.text((1110, 18), 'Gazebo 시뮬레이션 · 실제 주행 실행 화면',
                      font=medium, fill=(180, 215, 240))
            draw.text((22, 52), 'Gazebo 3D 장면', font=small, fill=(200, 210, 220))
            draw.text((980, 52), 'RViz2 · LiDAR 지도 + 이동 경로 + 실제 문 마커',
                      font=small, fill=(200, 210, 220))
            rgb, rgb_stamp = current['rgb']
            canvas.paste(Image.fromarray(rgb).resize((213, 160)), (0, 920))
            draw.text((5, 925), '로봇 RGB', font=small, fill='white', stroke_width=1, stroke_fill='black')
            mapping = current['map']
            objects = mapping['objects']
            counts = Counter(obj.get('last_confirmed_state', obj.get('state', 'unknown'))
                             for obj in objects)
            observation_counts = Counter(obj.get('observation_status', 'unavailable')
                                         for obj in objects)
            draw.text((240, 934), f'마지막 확인: 문 {len(objects)}개  ·  열림 {counts["open"]} / '
                      f'닫힘 {counts["closed"]} / 미확정 {counts["unknown"]}', font=large, fill='white')
            progress = current['patrol'] or dict(stage='waiting', label='순찰 시작 대기')
            label = str(progress.get('label', progress.get('stage', '상태 대기')))[:58]
            draw.text((240, 982), '현재 동작: ' + label, font=medium, fill=(180, 225, 255))
            pose = progress.get('robot_odom')
            pose_text = ''
            if isinstance(pose, list) and len(pose) == 3:
                pose_text = f' · odom ({pose[0]:.2f}, {pose[1]:.2f})m'
            draw.text((240, 1027), f'영상 경과 {elapsed:05.1f}s{pose_text} · 마지막 상태와 현재 관측 여부 분리',
                      font=small, fill=(185, 200, 215))
            observation_label = (f'현재 관측: 확정 {observation_counts["confirmed"]} / '
                                 f'불명 {observation_counts["uncertain"]} / '
                                 f'만료 {observation_counts["stale"]}')
            if observation_counts['unavailable']:
                observation_label += f' / 미상 {observation_counts["unavailable"]}'
            draw.text((1420, 942), observation_label, font=small, fill=(200, 215, 230))
            draw.text((1420, 981), '옅은색: 과거 확인 · 현재 미확인', font=small, fill=(200, 215, 230))
            draw.text((1420, 1021), '빨강: 닫힘 / 초록: 열림 · 실물 촬영 아님',
                      font=small, fill=(200, 215, 230))
            pixels = cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR)
            record = dict(capture=captures, capture_wall_ns=capture_wall, elapsed_sec=elapsed,
                          map_stamp_ns=mapping.get('stamp_ns'), rgb_stamp_ns=rgb_stamp,
                          objects=deepcopy(objects), patrol=deepcopy(progress))
            # 늦어진 구간은 직전 실제 캡처를 반복해 벽시계 재생 길이를 보존한다.
            # 반복 프레임은 metadata에 표시하며 새 센서 관측으로 세지 않는다.
            due = int((time.monotonic() - started) * args.fps)
            if last_pixels is not None:
                while encoded < due:
                    encode(last_pixels, last_record, duplicate=True)
            encode(pixels, record)
            last_pixels, last_record = pixels, record
            captures += 1
            stage = str(progress.get('stage', 'waiting'))
            if stage not in seen_stages:
                safe = re.sub('[^a-zA-Z0-9_-]+', '-', stage)[:50] or 'stage'
                canvas.save(out / f'preview-{len(seen_stages):02d}-{safe}.png')
                seen_stages.add(stage)
            if elapsed >= next_progress:
                print(f'{elapsed:.1f}s frames={encoded} 문={len(objects)} stage={stage}', flush=True)
                next_progress = elapsed + 5
        ended = time.monotonic()
        if ended - started < 10:
            raise RuntimeError('완료 녹화는 실제 10초 이상이어야 합니다')
        if last_pixels is not None:
            while encoded < round((ended - started) * args.fps):
                encode(last_pixels, last_record, duplicate=True)
        checks.update(status='complete', reason='종료 파일 수신')
    except BaseException as exc:
        errors.append(f'{type(exc).__name__}: {exc}')
        checks.update(status='failed', reason=errors[-1])
        raise
    finally:
        if ended is None:
            ended = time.monotonic()
        if writer is not None:
            writer.release()
        actual = ended - started if started is not None else 0.
        duration = encoded / args.fps
        checks.update(recorded_frames=encoded, actual_captures=captures, repeated_frames=filled,
                      fps=args.fps, video_duration_sec=duration, wall_duration_sec=actual,
                      duration_error_sec=duration - actual, errors=errors,
                      map_id=args.expected_map_id, endpoints=endpoints,
                      observed_ids=sorted({obj['id'] for row in frames for obj in row['objects']}),
                      final_door_map=current['map'], final_detector=current['detector'],
                      final_patrol=current['patrol'],
                      source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      capture_scope='전용 Xephyr의 실제 앱 화면; 사용자 바탕화면 제외')
        if checks['status'] == 'complete' and abs(duration - actual) > max(.25, 2 / args.fps):
            checks.update(status='failed', reason='영상과 벽시계 길이 차이 초과')
        write_json(out / 'frames.json', frames)
        write_json(out / 'door-map-history.json', history)
        write_json(out / 'recording-checks.json', checks)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if checks['status'] != 'complete':
        raise RuntimeError(checks['reason'])
    print(json.dumps({key: value for key, value in checks.items()
                      if key not in ['final_door_map', 'final_detector', 'final_patrol']},
                     ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
