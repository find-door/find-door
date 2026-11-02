"""정확히 동기화된 최신 RGB-D 한 묶음을 검출하고 원본 시각으로 함께 재발행한다."""

from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import tempfile
import time

from mapping_semantic.core import stamp_ns, validate_camera_inputs


def validate_reply(reply, request_id, width, height):
    if not isinstance(reply, dict) or type(reply.get('id')) is not int or reply.get('id') != request_id or 'error' in reply:
        raise ValueError(f'검출 worker 실패: {reply}')
    detections = reply.get('detections')
    if not isinstance(detections, list):
        raise ValueError('검출 목록 누락')
    for item in detections:
        if not isinstance(item, dict):
            raise ValueError('검출 record가 사전이 아닙니다')
        box, score = item.get('bbox'), item.get('confidence')
        if (item.get('class') not in ('door', 'person') or item.get('report_class') != item.get('class')
                or isinstance(score, bool) or not isinstance(score, (int, float))
                or not math.isfinite(score) or not 0 <= score <= 1
                or not isinstance(box, list) or len(box) != 4
                or any(isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x) for x in box)
                or not 0 <= box[0] < box[2] <= width or not 0 <= box[1] < box[3] <= height):
            raise ValueError('검출 결과 좌표·클래스·점수 오류')
    return detections


class Worker:
    def __init__(self, root, config, directory, timeout):
        self.timeout, self.directory, self.buffer = timeout, Path(directory), b''
        self.closed = False
        root = Path(root).expanduser().resolve()
        python = root / '.venv/bin/python'
        if not python.is_file() or not Path(config).is_file():
            raise ValueError('기존 recon-report 가상환경·설정이 필요합니다')
        self.log = (self.directory / 'worker.log').open('w')
        env = os.environ.copy()
        for name in ['PYTHONPATH', 'PYTHONHOME']:
            env.pop(name, None)
        command = [str(python), '-u', str(Path(__file__).with_name('detector_worker.py')),
                   '--recon-root', str(root), '--config', str(Path(config).resolve()), '--runtime', str(self.directory)]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, start_new_session=True, env=env)
        try:
            self.start_ticks = int(Path(f'/proc/{self.process.pid}/stat').read_text().rsplit(')', 1)[1].split()[19])
        except FileNotFoundError:
            self.start_ticks = None

    def request(self, identifier, image_path):
        payload = json.dumps(dict(id=identifier, image_path=str(image_path))).encode() + b'\n'
        self.process.stdin.write(payload)
        self.process.stdin.flush()
        deadline = time.monotonic() + self.timeout
        while b'\n' not in self.buffer:
            remaining = deadline-time.monotonic()
            if remaining <= 0 or not select.select([self.process.stdout], [], [], max(0., remaining))[0]:
                raise TimeoutError('검출 worker 응답 시간 초과: ' + self.log_tail())
            chunk = os.read(self.process.stdout.fileno(), 65536)
            if not chunk:
                raise RuntimeError('검출 worker 조기 종료: ' + self.log_tail())
            self.buffer += chunk
            if len(self.buffer) > 4_000_000:
                raise ValueError('worker 응답 크기 초과')
        line, self.buffer = self.buffer.split(b'\n', 1)
        return json.loads(line)

    def log_tail(self):
        self.log.flush()
        path = self.directory / 'worker.log'
        with path.open('rb') as source:
            source.seek(max(0, path.stat().st_size - 4096))
            return source.read().decode('utf-8', errors='replace')

    def close(self):
        if self.closed:
            return
        for sig in [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]:
            try:
                os.killpg(self.process.pid, sig)
            except ProcessLookupError:
                break
            try:
                self.process.wait(timeout=5)
                break
            except subprocess.TimeoutExpired:
                continue
        self.process.wait(timeout=5)
        for stream in [self.process.stdin, self.process.stdout, self.log]:
            stream.close()
        self.closed = True


def main(args=None):
    import message_filters
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.clock import Clock, ClockType
    from cv_bridge import CvBridge
    from sensor_msgs.msg import Image, CameraInfo
    from std_msgs.msg import String
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

    class LiveDetector(Node):
        def __init__(self):
            super().__init__('live_door_detector')
            defaults = dict(recon_root='', detector_config='', worker_timeout_sec=60.,
                            rgb_topic='/camera/color/image_raw', depth_topic='/camera/depth/image_raw',
                            camera_info_topic='/camera/camera_info', optical_frame='camera_color_optical_frame')
            for key, value in defaults.items():
                self.declare_parameter(key, value)
            self.p = {key: self.get_parameter(key).value for key in defaults}
            if not math.isfinite(self.p['worker_timeout_sec']) or not 0 < self.p['worker_timeout_sec'] <= 120:
                raise ValueError('worker timeout은 0~120초 범위여야 합니다')
            root = Path(self.p['recon_root']).expanduser().resolve()
            config = self.p['detector_config'] or str(root / 'experiments/configs/yoloe-door-person.json')
            self.temp = tempfile.TemporaryDirectory(prefix='find-door-live-')
            self.worker = Worker(root, config, self.temp.name, self.p['worker_timeout_sec'])
            self.executor_pool = ThreadPoolExecutor(max_workers=1)
            self.bridge, self.counters = CvBridge(), Counter()
            self.pending = self.active = self.future = None
            self.last_stamp = self.last_now = self.epoch = self.request_id = 0
            self.metadata, self.last_error = {}, ''
            self.latency_sec = self.source_age_sec = None
            self.clear_sync_pending = False
            self.subscribers = [message_filters.Subscriber(self, typ, self.p[key], qos_profile=qos_profile_sensor_data)
                                for typ, key in [(Image, 'rgb_topic'), (Image, 'depth_topic'), (CameraInfo, 'camera_info_topic')]]
            self.sync = message_filters.TimeSynchronizer(self.subscribers, 20)
            self.sync.registerCallback(self.images)
            self.bundle_publishers = [self.create_publisher(typ, topic, 10) for typ, topic in [
                (Image, '/doors/input/rgb'), (Image, '/doors/input/depth'),
                (CameraInfo, '/doors/input/camera_info'), (Detection2DArray, '/doors/input/detections2d')]]
            self.status = self.create_publisher(String, '/doors/detector_status', 10)
            self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

        def now(self):
            now = self.get_clock().now().nanoseconds
            if self.last_now > 0 and now < self.last_now:
                self.epoch += 1
                self.last_stamp = 0
                self.pending = None
                self.clear_sync_pending = True
                self.counters['clock_resets'] += 1
            self.last_now = now
            return now

        def images(self, rgb, depth, info):
            try:
                now = self.now()
                if self.clear_sync_pending:
                    raise ValueError('epoch 변경 직후 batch 거부')
                stamp, _ = validate_camera_inputs(rgb, depth, info, self.p['optical_frame'])
                if stamp <= self.last_stamp or now <= 0 or not -100_000_000 <= now-stamp <= 10_000_000_000:
                    raise ValueError('중복·역행·오래된 RGB-D')
                self.last_stamp = stamp
                self.counters['received'] += 1
                if self.pending is not None:
                    self.counters['replaced'] += 1
                self.pending = (rgb, depth, info, self.epoch)
            except Exception as exc:
                self.counters['input_rejected'] += 1
                self.last_error = str(exc)

        def tick(self):
            now = self.now()
            if self.clear_sync_pending:
                for queue in self.sync.queues:
                    queue.clear()
                self.clear_sync_pending = False
            if self.future is not None and self.future.done():
                rgb, depth, info, epoch = self.active
                try:
                    reply = self.future.result()
                    detections = validate_reply(reply, self.request_id, rgb.width, rgb.height)
                    if epoch != self.epoch:
                        self.counters['old_epoch_dropped'] += 1
                    else:
                        array = Detection2DArray()
                        array.header = deepcopy(rgb.header)
                        for item in detections:
                            detection = Detection2D()
                            detection.header = deepcopy(rgb.header)
                            x0, y0, x1, y1 = item['bbox']
                            detection.bbox.center.position.x, detection.bbox.center.position.y = (x0+x1)/2, (y0+y1)/2
                            detection.bbox.size_x, detection.bbox.size_y = x1-x0, y1-y0
                            hypothesis = ObjectHypothesisWithPose()
                            hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = item['class'], item['confidence']
                            detection.results = [hypothesis]
                            array.detections.append(detection)
                        for publisher, message in zip(self.bundle_publishers, [rgb, depth, info, array]):
                            publisher.publish(message)
                        self.metadata = reply.get('metadata', {})
                        self.latency_sec = time.monotonic()-self.submitted_at
                        self.source_age_sec = (now-stamp_ns(rgb.header.stamp))/1e9
                        self.counters['completed'] += 1
                except Exception as exc:
                    self.counters['errors'] += 1
                    self.last_error = str(exc)
                    # 응답 유실 뒤 다음 id와 응답을 혼동하지 않도록 worker를 재사용하지 않는다.
                    self.worker.close()
                    self.pending = None
                    self.failed = True
                self.future = self.active = None
            if self.future is None and self.pending is not None and not getattr(self, 'failed', False):
                self.active, self.pending = self.pending, None
                try:
                    rgb = self.active[0]
                    image = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='rgb8')
                    from PIL import Image as PILImage
                    path = Path(self.temp.name) / 'inflight.png'
                    PILImage.fromarray(image).save(path)
                    self.request_id += 1
                    self.submitted_at = time.monotonic()
                    self.future = self.executor_pool.submit(self.worker.request, self.request_id, path)
                    self.counters['submitted'] += 1
                except Exception as exc:
                    self.counters['errors'] += 1
                    self.last_error = str(exc)
                    self.active = None
            message = String()
            message.data = json.dumps(dict(counters=dict(self.counters), epoch=self.epoch,
                worker_pgid=getattr(getattr(self.worker, 'process', None), 'pid', None),
                worker_start_ticks=getattr(self.worker, 'start_ticks', None),
                latency_sec=self.latency_sec, source_age_sec=self.source_age_sec,
                metadata=self.metadata, last_error=self.last_error), ensure_ascii=False, allow_nan=False)
            self.status.publish(message)

        def close(self):
            self.timer.cancel()
            self.pending = None
            self.worker.close()
            self.executor_pool.shutdown(wait=True, cancel_futures=True)
            self.temp.cleanup()

    rclpy.init(args=args)
    node = None
    try:
        node = LiveDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        previous = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            signal.signal(signal.SIGINT, previous)
