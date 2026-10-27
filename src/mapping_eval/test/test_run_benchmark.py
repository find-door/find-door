"""프로세스 격리 및 잘못된 CLI 실행 시 부작용을 검증한다."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from mapping_eval.run_benchmark import main, stop_process


def test_missing_confirmation_does_not_create_output(tmp_path):
    target = tmp_path / 'untouched'
    with pytest.raises(SystemExit) as exc:
        main(['--output', str(target)])
    assert exc.value.code == 2
    assert not target.exists()


def test_wrong_domain_does_not_create_output(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    target = tmp_path / 'untouched'
    with pytest.raises(SystemExit) as exc:
        main(['--output', str(target), '--confirm-simulation'])
    assert exc.value.code == 2
    assert not target.exists()


def test_stops_child_even_after_group_leader_exits(tmp_path):
    marker = tmp_path / 'child-stopped'
    ready = tmp_path / 'ready'
    script = '''
import os, signal, sys, time
from pathlib import Path
pid = os.fork()
if pid:
    sys.exit(0)
def stop(signum, frame):
    Path(sys.argv[1]).write_text('stopped')
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
Path(sys.argv[2]).touch()
while True:
    time.sleep(.1)
'''
    child = subprocess.Popen([sys.executable, '-c', script, str(marker), str(ready)],
                             start_new_session=True)
    try:
        child.wait(timeout=5)
        deadline = time.monotonic() + 5
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        assert ready.exists()
        stop_process(child)
        assert marker.read_text() == 'stopped'
    finally:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def test_custom_parameters_are_snapshotted_before_domain_probe(tmp_path, monkeypatch):
    import hashlib
    import json
    import types
    shares = {}
    for name in ['mapping_eval', 'mapping_slam', 'mapping_gazebo', 'wf_gazebo']:
        shares[name] = tmp_path / name
        shares[name].mkdir()
    config = shares['mapping_eval'] / 'config'
    config.mkdir()
    (config / 'indoor_v1_route.yaml').write_text('route')
    (config / 'indoor_v1_truth.yaml').write_text('truth')
    worlds = shares['mapping_gazebo'] / 'worlds'
    worlds.mkdir()
    (worlds / 'indoor_v1.sdf').write_text('world')
    custom = tmp_path / 'custom-name.yaml'
    custom.write_text('slam_toolbox:\n  ros__parameters:\n    do_loop_closing: false\n')
    packages = types.ModuleType('ament_index_python.packages')
    packages.get_package_share_directory = lambda name: str(shares[name])
    monkeypatch.setitem(sys.modules, 'ament_index_python', types.ModuleType('ament_index_python'))
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    ros = types.ModuleType('rclpy')
    ros.init = lambda **kwargs: None
    ros.shutdown = lambda: None
    ros.spin_once = lambda *args, **kwargs: None
    ros.create_node = lambda name: types.SimpleNamespace(
        count_publishers=lambda topic: 1, destroy_node=lambda: None)
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setenv('ROS_DOMAIN_ID', '11')
    monkeypatch.setenv('GZ_PARTITION', 'claude')
    output = tmp_path / 'result'
    assert main(['--output', str(output), '--params-file', str(custom),
                 '--confirm-simulation']) == 1
    snapshot = output / 'inputs/slam.yaml'
    assert snapshot.read_bytes() == custom.read_bytes()
    report = json.loads((output / 'manifest.json').read_text())
    assert report['input_sha256']['slam.yaml'] == hashlib.sha256(custom.read_bytes()).hexdigest()
    assert report['status'] == 'failed'
    assert '기존 시뮬/로봇' in report['error']
    assert report['configuration']['ROS_DOMAIN_ID'] == 11
    assert report['configuration']['GZ_PARTITION'] == 'claude'
    custom.write_text('changed after snapshot')
    assert 'do_loop_closing: false' in snapshot.read_text()


def test_recording_banner_does_not_mean_sensor_subscriptions_are_ready():
    from mapping_eval.run_benchmark import CAPTURE_TOPICS, recorder_ready
    assert not recorder_ready('Recording...')
    subscribed = ["Subscribed to topic '" + topic + "'" for topic in CAPTURE_TOPICS]
    assert not recorder_ready('\n'.join(subscribed[:-1]))
    assert recorder_ready('\n'.join(subscribed))


def test_stale_ready_marker_does_not_hide_dead_process():
    from types import SimpleNamespace
    from mapping_eval.run_benchmark import wait_until
    with pytest.raises(RuntimeError, match='프로세스 조기 종료'):
        wait_until(lambda: True, [SimpleNamespace(poll=lambda: 1)], 1, 'test')
