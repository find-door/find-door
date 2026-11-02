"""명시적 격리 ROS 환경에서 동기화 callback이 먼저 시각 역행을 발견하는 경로 검사."""

import os
import pytest


def test_callback_first_epoch_reset_discards_batch(monkeypatch):
    if (os.environ.get('ROS_DOMAIN_ID'), os.environ.get('GZ_PARTITION')) != ('12', 'codex'):
        pytest.skip('명시적인 Codex ROS 격리 환경 필요')
    rclpy = pytest.importorskip('rclpy')
    from mapping_semantic.door_node import main
    from sensor_msgs.msg import Image

    def exercise(node):
        partial = Image()
        partial.header.stamp.nanosec = 1
        node.sync.add(partial, node.sync.queues[0], 0)
        node.last_now = node.get_clock().now().nanoseconds + 10_000_000_000
        # batch 내용을 읽기 전에 epoch 혼합 가능성을 이유로 거부해야 한다.
        node.images(None, None, None, None)
        if node.counters['clock_resets'] != 1 or node.counters['epoch_batch_rejected'] != 1:
            raise RuntimeError('callback 우선 epoch 거부 경로 실패')
        if node.pending or not node.clear_sync_pending:
            raise RuntimeError('역행 batch가 대기 큐에 남았습니다')
        if not any(node.sync.queues):
            raise RuntimeError('동기화 callback 안에서 부분 큐를 지웠습니다')
        node.tick()
        if node.clear_sync_pending or any(node.sync.queues):
            raise RuntimeError('timer가 동기화 큐를 비우지 않았습니다')

    monkeypatch.setattr(rclpy, 'spin', exercise)
    main(['--ros-args', '-p', 'map_id:=epoch-guard-test'])
