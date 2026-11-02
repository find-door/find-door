"""기하 대응·음성 대조군·실행 실패 분모 및 부분 캡처 거부를 확인한다."""

import importlib.util
import json
import os
from pathlib import Path
import sys

from PIL import Image
import pytest


SPEC = importlib.util.spec_from_file_location('door_probe_evaluate', Path(__file__).with_name('evaluate.py'))
EVAL = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EVAL)


def detection(box, label='door'):
    return {'class': label, 'report_class': label, 'bbox': box, 'confidence': 0.8}


def test_iou_boundary_and_duplicate_predictions():
    truth = [0, 0, 10, 10]
    assert EVAL.iou(truth, [0, 0, 10, 20]) == 0.5
    assert EVAL.iou(truth, [20, 20, 30, 30]) == 0
    scored = EVAL.score_frame(truth, [detection([0, 0, 10, 20]), detection(truth)], 100, 100)
    assert scored['success']
    assert (scored['tp'], scored['fn'], scored['fp']) == (1, 0, 1)
    assert scored['matched_index'] == 1
    scored = EVAL.score_frame(truth, [detection([0, 0, 10, 20])], 100, 100)
    assert scored['success']


def test_mislocalized_door_is_both_miss_and_false_positive():
    scored = EVAL.score_frame([0, 0, 10, 10], [detection([20, 20, 30, 30])], 100, 100)
    assert (scored['tp'], scored['fn'], scored['fp']) == (0, 1, 1)
    assert not scored['success']


def test_negative_control_counts_door_only():
    assert EVAL.score_frame(None, [detection([0, 0, 10, 10], 'person')], 100, 100)['success']
    scored = EVAL.score_frame(None, [detection([0, 0, 10, 10])], 100, 100)
    assert scored['status'] == 'false_positive'
    assert (scored['tp'], scored['fn'], scored['fp']) == (0, 0, 1)


def test_runtime_error_remains_in_denominator_without_invented_fp_fn():
    base = dict(condition_id='closed_d1_a0', state='closed', door_angle_deg=0,
                distance_m=1, view_angle_deg=0)
    rows = [{**base, **EVAL.score_frame([0, 0, 10, 10], [detection([0, 0, 10, 10])], 100, 100)},
            {**base, 'status': 'runtime_error', 'success': False, 'tp': None, 'fn': None, 'fp': None}]
    summary = EVAL.summarize(rows)[0]
    assert summary['frames'] == 2
    assert summary['success_rate'] == 0.5
    assert summary['runtime_errors'] == 1
    assert summary['fp_observed'] == 0 and summary['fn'] == 0


@pytest.mark.parametrize('box', [[0, 0, 0, 1], [-1, 0, 1, 1], [0, 0, float('nan'), 1], [False, 0, 1, 1]])
def test_invalid_boxes_rejected(box):
    with pytest.raises(ValueError):
        EVAL.iou(box, [0, 0, 1, 1])


def capture_fixture(tmp_path):
    Image.new('RGB', (100, 100)).save(tmp_path / 'rgb.png')
    (tmp_path / 'frame.json').write_text(json.dumps({'truth_bbox_xyxy': [10, 10, 90, 90]}))
    manifest = {'schema_version': 1, 'environment': 'gazebo', 'status': 'complete',
                'frames_per_condition': 1, 'total_conditions': 1,
                'conditions': [{'id': 'closed_d1_a0', 'state': 'closed', 'door_angle_deg': 0,
                                'distance_m': 1, 'view_angle_deg': 0,
                                'frames': [{'rgb': 'rgb.png', 'metadata': 'frame.json'}]}]}
    (tmp_path / 'captures.json').write_text(json.dumps(manifest))
    return manifest


def test_complete_capture_and_missing_file(tmp_path):
    capture_fixture(tmp_path)
    _, frames = EVAL.load_captures(tmp_path)
    assert frames[0]['truth_bbox_xyxy'] == [10, 10, 90, 90]
    (tmp_path / 'rgb.png').rename(tmp_path / 'moved.png')
    with pytest.raises(ValueError, match='파일'):
        EVAL.load_captures(tmp_path)


@pytest.mark.parametrize('change', [{'status': 'running'}, {'status': 'failed'},
                                   {'frames_per_condition': 3}, {'total_conditions': 27}])
def test_partial_capture_rejected(tmp_path, change):
    manifest = capture_fixture(tmp_path)
    manifest.update(change)
    (tmp_path / 'captures.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError):
        EVAL.load_captures(tmp_path)


def test_duplicate_frame_rejected(tmp_path):
    manifest = capture_fixture(tmp_path)
    manifest['frames_per_condition'] = 2
    manifest['conditions'][0]['frames'] *= 2
    (tmp_path / 'captures.json').write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='중복'):
        EVAL.load_captures(tmp_path)


def test_cli_raw_output_and_timeout_process_cleanup(tmp_path):
    command = [sys.executable, '-c', 'import time; time.sleep(30)']
    execution, _ = EVAL.run_cli(command, tmp_path, 0.05, tmp_path)
    assert execution['timed_out'] and execution['returncode'] != 0
    assert (tmp_path / 'stdout.json').is_file()
    assert (tmp_path / 'stderr.txt').is_file()
    assert EVAL.read_json(tmp_path / 'execution.json')['command'] == command
    with pytest.raises(ProcessLookupError):
        os.kill(execution['pid'], 0)


def test_report_lifecycle_with_fake_cli_and_no_model(tmp_path, monkeypatch):
    captures = tmp_path / 'captures'
    captures.mkdir()
    capture_fixture(captures)
    recon = tmp_path / 'recon'
    (recon / '.venv/bin').mkdir(parents=True)
    (recon / '.venv/bin/python').touch()
    monkeypatch.setattr(EVAL, 'provenance', lambda root: {'config_path': str(root / 'config.json')})

    def fake_cli(command, cwd, timeout, directory):
        assert command[:5] == ['env', '-u', 'PYTHONPATH', '-u', 'PYTHONHOME']
        assert cwd == recon
        return {'returncode': 0, 'timed_out': False}, json.dumps({
            'schema_valid': True, 'error': None,
            'report': {'scope': 'door-person', 'method': 'C', 'states': [{'state': 'unknown'}]},
            'detections': [detection([10, 10, 90, 90])],
        })

    monkeypatch.setattr(EVAL, 'run_cli', fake_cli)
    output = tmp_path / 'results'
    assert EVAL.main(['--captures', str(captures), '--output', str(output),
                      '--recon-root', str(recon)]) == 0
    result = EVAL.read_json(output / 'results.json')
    assert result['status'] == 'complete'
    assert result['summary'][0]['success_rate'] == 1.0
    assert (output / 'summary.csv').is_file()
    assert (output / 'report.md').is_file()
    with Image.open(output / result['frames'][0]['overlay']) as image:
        assert image.size == (100, 100)
    with pytest.raises(SystemExit):
        EVAL.main(['--captures', str(captures), '--output', str(output), '--recon-root', str(recon)])
