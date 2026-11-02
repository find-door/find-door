"""상류 실행 오류의 승계와 world 정답 계약을 검증한다."""

import hashlib
import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]


def test_upstream_failure_is_not_normal_miss(tmp_path):
    source = {
        'status': 'complete', 'environment': 'gazebo', 'expected_frames': 1,
        'captures_manifest': {'conditions': [{'state': 'closed', 'world_sha256': hashlib.sha256(
            (ROOT / 'src/mapping_gazebo/worlds/door_probe_closed.sdf').read_bytes()).hexdigest()}]},
        'frames': [{'frame_id': 'closed_d2_a0_00', 'condition_id': 'closed_d2_a0',
                    'state': 'closed', 'status': 'runtime_error', 'error': 'timeout'}],
    }
    path = tmp_path / 'source.json'
    path.write_text(json.dumps(source))
    out = tmp_path / 'out'
    completed = subprocess.run([sys.executable, str(ROOT / 'scripts/door_probe/estimated_benchmark.py'),
                                '--detections', str(path), '--output', str(out)], capture_output=True, text=True)
    assert completed.returncode == 0, completed.stderr
    report = json.loads((out / 'results.json').read_text())
    assert report['execution_errors'] == 1
    assert report['total_positive'] == 1 and report['correct_states'] == 0
    assert '상류 C 실행 오류' in report['frames'][0]['error']
    assert report['frames'][0].get('reason') != 'no_door_detection'
    source['captures_manifest']['conditions'][0]['world_sha256'] = 'foreign-world'
    path.write_text(json.dumps(source))
    other = tmp_path / 'other'
    failed = subprocess.run([sys.executable, str(ROOT / 'scripts/door_probe/estimated_benchmark.py'),
                             '--detections', str(path), '--output', str(other)], capture_output=True, text=True)
    assert failed.returncode != 0
    assert not other.exists()
