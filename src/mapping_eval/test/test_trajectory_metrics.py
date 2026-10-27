"""실제 이동과 SLAM 잔차 변화를 구분하고 시간 누락을 숨기지 않는지 검사."""

import csv
import json
import math

import pytest

from mapping_eval.trajectory_metrics import COLUMNS, analyze_trajectory, main


def write_trajectory(tmp_path, samples):
    path = tmp_path / 'trajectory.csv'
    with path.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        for stamp, odom, estimated in samples:
            writer.writerow({
                'stamp_ns': stamp, 'odom_x': odom[0], 'odom_y': odom[1],
                'map_x': estimated[0], 'map_y': estimated[1],
                'estimated_world_x': estimated[0], 'estimated_world_y': estimated[1],
                'error_m': math.hypot(estimated[0] - odom[0], estimated[1] - odom[1]),
            })
    return path


def test_normal_motion_is_not_a_residual_jump(tmp_path):
    path = write_trajectory(tmp_path, [(0, (0, 0), (0, 0)),
                                      (50_000_000, (1, 0), (1, 0)),
                                      (100_000_000, (2, 1), (2, 1))])
    report = analyze_trajectory(path)
    assert report['sample_count'] == 3
    assert report['residual_position']['rmse_m'] == 0
    assert report['adjacent_residual_step_max']['step_m'] == 0
    assert report['jump_event_count'] == 0
    assert report['gap_count'] == 0
    assert report['max_gap_sec'] == 0.05


def test_constant_offset_is_error_but_not_jump(tmp_path):
    path = write_trajectory(tmp_path, [(0, (0, 0), (3, 4)),
                                      (50_000_000, (1, 1), (4, 5))])
    report = analyze_trajectory(path)
    assert report['residual_position']['rmse_m'] == pytest.approx(5)
    assert report['residual_position']['max_m'] == 5
    assert report['jump_event_count'] == 0


def test_residual_vector_direction_change_is_detected(tmp_path):
    # 잔차 크기는 0.5로 같지만 벡터 변화량은 1m이다.
    path = write_trajectory(tmp_path, [(0, (0, 0), (0.5, 0)),
                                      (50_000_000, (1, 0), (0.5, 0))])
    report = analyze_trajectory(path)
    assert report['jump_event_count'] == 1
    assert report['eligible_residual_step_max']['step_m'] == 1
    assert report['eligible_residual_step_max']['stamp_ns'] == 50_000_000
    assert report['eligible_residual_step_max']['previous_stamp_ns'] == 0


def test_tf_gap_excluded_from_jump_but_reported_in_observed_max(tmp_path):
    path = write_trajectory(tmp_path, [(0, (0, 0), (0, 0)),
                                      (500_000_000, (1, 0), (2, 0)),
                                      (550_000_000, (2, 0), (3, 0))])
    report = analyze_trajectory(path)
    assert report['gap_count'] == 1
    assert report['max_gap_sec'] == 0.5
    assert report['adjacent_residual_step_max']['step_m'] == 1
    assert report['eligible_residual_step_max']['step_m'] == 0
    assert report['jump_event_count'] == 0
    assert report['eligible_pair_count'] == 1


def test_threshold_and_gap_boundary_are_inclusive(tmp_path):
    path = write_trajectory(tmp_path, [(0, (0, 0), (0, 0)),
                                      (200_000_000, (1, 0), (1.5, 0))])
    report = analyze_trajectory(path)
    assert report['jump_event_count'] == 1
    assert report['gap_count'] == 0


def test_single_sample_has_no_adjacent_max(tmp_path):
    path = write_trajectory(tmp_path, [(100, (0, 0), (0, 0))])
    report = analyze_trajectory(path)
    assert report['adjacent_residual_step_max'] is None
    assert report['eligible_residual_step_max'] is None
    assert report['max_gap_sec'] == 0


@pytest.mark.parametrize('stamps', [(100, 100), (100, 99), (100, -1)])
def test_observation_order_must_strictly_increase(tmp_path, stamps):
    path = write_trajectory(tmp_path, [(stamp, (0, 0), (0, 0)) for stamp in stamps])
    with pytest.raises(ValueError, match='stamp_ns'):
        analyze_trajectory(path)


@pytest.mark.parametrize('column', COLUMNS[1:])
@pytest.mark.parametrize('value', ['nan', 'inf', '-inf', 'not-a-number', ''])
def test_all_required_numeric_fields_must_be_finite(tmp_path, column, value):
    path = tmp_path / 'invalid.csv'
    row = dict.fromkeys(COLUMNS, '0')
    row[column] = value
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerow(row)
    with pytest.raises(ValueError):
        analyze_trajectory(path)


def test_stored_error_is_not_used_for_metrics(tmp_path):
    path = write_trajectory(tmp_path, [(0, (0, 0), (3, 4))])
    content = path.read_text().replace('5.0', '999.0')
    path.write_text(content)
    assert analyze_trajectory(path)['residual_position']['max_m'] == 5


@pytest.mark.parametrize('content', ['', ','.join(COLUMNS) + '\n',
                                     'stamp_ns,odom_x\n0,0\n'])
def test_empty_or_missing_columns_rejected(tmp_path, content):
    path = tmp_path / 'invalid.csv'
    path.write_text(content)
    with pytest.raises(ValueError):
        analyze_trajectory(path)


@pytest.mark.parametrize('threshold,gap', [(0, 0.2), (-1, 0.2), (float('nan'), 0.2),
                                          (0.5, 0), (0.5, float('inf'))])
def test_invalid_thresholds_rejected(tmp_path, threshold, gap):
    path = write_trajectory(tmp_path, [(0, (0, 0), (0, 0))])
    with pytest.raises(ValueError):
        analyze_trajectory(path, threshold, gap)


def test_cli_json_and_no_overwrite(tmp_path):
    path = write_trajectory(tmp_path, [(0, (0, 0), (0, 0))])
    output = tmp_path / 'report.json'
    arguments = ['--trajectory', str(path), '--output', str(output)]
    assert main(arguments) == 0
    report = json.loads(output.read_text())
    assert report['sample_count'] == 1
    assert len(report['input']['sha256']) == 64
    before = output.read_bytes()
    assert main(arguments) == 1
    assert output.read_bytes() == before


def test_cli_rejects_input_as_output(tmp_path):
    path = write_trajectory(tmp_path, [(0, (0, 0), (0, 0))])
    before = path.read_bytes()
    assert main(['--trajectory', str(path), '--output', str(path)]) == 1
    assert path.read_bytes() == before
