"""실제 ROS Node 초기화에서 예약 속성 충돌과 자원 정리를 검사한다."""

import os
import signal
import time
from concurrent.futures import Future
import pytest


def test_live_node_initializes_and_closes(monkeypatch, tmp_path):
    if (os.environ.get('ROS_DOMAIN_ID'), os.environ.get('GZ_PARTITION')) != ('12', 'codex'):
        pytest.skip('명시적 Codex ROS 격리 환경 필요')
    rclpy = pytest.importorskip('rclpy')
    from mapping_semantic import live_detector
    from sensor_msgs.msg import Image, CameraInfo
    workers = []

    class FakeWorker:
        def __init__(self, *args):
            self.closed = False
            workers.append(self)

        def close(self):
            if signal.getsignal(signal.SIGINT) != signal.SIG_IGN:
                raise RuntimeError('정리 중 중복 SIGINT 보호 없음')
            os.kill(os.getpid(), signal.SIGINT)
            self.closed = True

    def inspect(node):
        if len(node.bundle_publishers) != 4:
            raise RuntimeError('묶음 publisher 초기화 실패')
        if node.active is not None or node.pending is not None:
            raise RuntimeError('초기 작업 상태 오류')
        for identifier in [1, 2]:
            rgb, depth, info = Image(), Image(), CameraInfo()
            stamp = node.get_clock().now().to_msg()
            for msg in (rgb, depth, info):
                msg.header.stamp = stamp
                msg.header.frame_id = 'camera_color_optical_frame'
                msg.width, msg.height = 640, 480
            node.request_id = identifier
            node.submitted_at = time.monotonic()
            node.active = (rgb, depth, info, node.epoch)
            node.future = Future()
            node.future.set_result({'id': identifier, 'detections': [], 'metadata': {}})
            node.tick()
        if node.counters['completed'] != 2 or node.counters['errors'] or workers[0].closed:
            raise RuntimeError('완료 프레임 연속 발행 실패')
        if node.source_age_sec is None or node.source_age_sec < 0:
            raise RuntimeError('원본 header 시각의 지연 계산 실패')

    monkeypatch.setattr(live_detector, 'Worker', FakeWorker)
    monkeypatch.setattr(rclpy, 'spin', inspect)
    live_detector.main(['--ros-args', '-p', f'recon_root:={tmp_path}'])
    assert len(workers) == 1 and workers[0].closed
