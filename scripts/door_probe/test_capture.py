"""촬영 위치와 문틀 투영의 좌표 계약을 검사한다."""

import math
import numpy as np
import pytest

from capture import (
    CAMERA_OFFSET_XY, condition_summary, optical_heading_error,
    projected_truth, rotation, spawn_pose, write_json,
)


@pytest.mark.parametrize('angle', [-30, 0, 30])
@pytest.mark.parametrize('distance', [1, 2, 3])
def test_optical_center_distance_and_heading(distance, angle):
    x, y, yaw = spawn_pose(distance, angle)
    c, s = math.cos(yaw), math.sin(yaw)
    camera = np.array([x, y]) + np.array([[c, -s], [s, c]]) @ CAMERA_OFFSET_XY
    assert np.linalg.norm(camera) == pytest.approx(distance)
    assert math.degrees(math.atan2(camera[1], -camera[0])) == pytest.approx(angle)
    assert np.dot(np.array([c, s]), -camera / distance) == pytest.approx(1)


def test_rep103_projection_is_clipped_and_uses_optical_depth():
    # optical x→world -y, optical y→world -z, optical z→world +x
    q = [-.5, .5, -.5, .5]
    assert np.allclose(rotation(q) @ [0, 0, 1], [1, 0, 0])
    box, raw, clipped = projected_truth([-2, 0, .12], q,
                                        [500, 0, 320, 0, 500, 240, 0, 0, 1], 640, 480)
    assert clipped
    assert raw[1] < 0
    assert box[1] == 0
    assert box[0] == pytest.approx(320 - 500 * .58 / 1.895)
    assert box[2] == pytest.approx(320 + 500 * .58 / 1.895)


def test_behind_camera_truth_is_rejected():
    with pytest.raises(ValueError, match='카메라 뒤'):
        projected_truth([1, 0, .12], [-.5, .5, -.5, .5], np.eye(3), 640, 480)


@pytest.mark.parametrize('angle', [-30, 0, 30])
def test_actual_optical_axis_matches_requested_heading(angle):
    x, y, z, w = [-.5, .5, -.5, .5]
    c, s = math.cos(math.radians(-angle) / 2), math.sin(math.radians(-angle) / 2)
    q = [c*x-s*y, c*y+s*x, c*z+s*w, c*w-s*z]
    assert optical_heading_error(q, angle) < 1e-5
    assert optical_heading_error(q, angle + 10) == pytest.approx(10)
    # 카메라 위치가 맞아도 광학축이 수직이면 허용하지 않는다.
    assert optical_heading_error([0, 0, 0, 1], angle) == pytest.approx(90)


def test_failed_worker_partial_frames_remain_in_summary(tmp_path):
    condition_path = tmp_path / 'condition.json'
    write_json(condition_path, {'status': 'failed', 'error': '수집 시간 초과',
                               'frames': [{'rgb': '00.png', 'metadata': '00.json'}]})
    summary = condition_summary(condition_path, 'closed_d2_a0', 'closed', 2, 0, 1)
    assert summary['status'] == 'failed'
    assert summary['worker_returncode'] == 1
    assert summary['frames'][0]['rgb'] == 'closed_d2_a0/00.png'
    assert summary['error'] == '수집 시간 초과'
    assert condition_path.read_text().find('closed_d2_a0') == -1


def test_worker_without_condition_record_is_counted_as_failed(tmp_path):
    summary = condition_summary(tmp_path / 'missing.json', 'open_d1_a30', 'open', 1, 30, None)
    assert summary['status'] == 'failed'
    assert summary['id'] == 'open_d1_a30'
    assert summary['state'] == 'open'
    assert summary['frames'] == []


def test_parent_interrupt_waits_for_worker_and_separate_launch_group(tmp_path):
    """실제 부모→worker→별도 PGID 자식에서 Ctrl-C 정리 유예를 검증한다."""
    import json
    import os
    from pathlib import Path
    import signal
    import subprocess
    import sys
    import time

    from capture import process_identity, stop_group

    module_dir = str(Path(__file__).resolve().parent)
    condition_path = tmp_path / 'condition.json'
    ready = tmp_path / 'launch-ready'
    fake_launch = tmp_path / 'fake_launch.py'
    fake_launch.write_text('''import signal, sys, time
from pathlib import Path
def shutdown(*args):
    time.sleep(0.8)
    raise SystemExit(0)
signal.signal(signal.SIGINT, shutdown)
Path(sys.argv[1]).write_text('ready')
while True:
    time.sleep(.05)
''')
    worker = tmp_path / 'worker.py'
    worker.write_text('''import os, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from capture import process_identity, stop_group, write_json
condition = Path(sys.argv[2])
child = subprocess.Popen([sys.executable, sys.argv[3], sys.argv[4]], start_new_session=True)
record = {'status': 'running', 'frames': [], 'worker_pid': os.getpid(),
          'launch_pgid': child.pid, 'launch_start_ticks': process_identity(child.pid)['start_ticks']}
write_json(condition, record)
try:
    while True:
        time.sleep(.05)
except KeyboardInterrupt:
    record['status'] = 'failed'
finally:
    stop_group(child)
    record['cleanup_done'] = True
    write_json(condition, record)
''')
    driver = tmp_path / 'driver.py'
    driver.write_text('''import sys
from pathlib import Path
sys.path.insert(0, sys.argv[1])
from capture import run_worker
try:
    run_worker([sys.executable, *sys.argv[2:]], Path(sys.argv[4]))
except KeyboardInterrupt:
    pass
''')
    # driver argv: module_dir, worker.py, module_dir, condition, fake_launch, ready
    child = subprocess.Popen([sys.executable, str(driver), module_dir, str(worker), module_dir,
                              str(condition_path), str(fake_launch), str(ready)], start_new_session=True)
    record = None
    try:
        deadline = time.monotonic() + 8
        while not (ready.is_file() and condition_path.is_file()):
            assert child.poll() is None
            assert time.monotonic() < deadline
            time.sleep(.02)
        record = json.loads(condition_path.read_text())
        # 부모 PID에만 SIGINT. worker/Gazebo에는 전달 책임이 자동으로 생기지 않는다.
        os.kill(child.pid, signal.SIGINT)
        assert child.wait(timeout=8) == 0
        assert json.loads(condition_path.read_text())['cleanup_done'] is True
        assert process_identity(record['worker_pid']) is None
        assert process_identity(record['launch_pgid']) is None
    finally:
        if child.poll() is None:
            stop_group(child)
        # 테스트 실패 시에도 이번 테스트가 만든 별도 그룹만 정리한다.
        if record is not None:
            from capture import cleanup_recorded_launch
            cleanup_recorded_launch(condition_path, record['worker_pid'])
