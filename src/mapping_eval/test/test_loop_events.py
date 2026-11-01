import json
import math

import pytest

from mapping_eval.loop_events import analyze_loops, main


def edge(source, target, link):
    return {'source': {'id': 1, 'time': 1.0, 'odom': source, 'corrected': source},
            'target': {'id': 9, 'time': 9.0, 'odom': target, 'corrected': target},
            'link': link, 'variance': [0.1, 0.1, 0.1]}


def write(tmp_path, records):
    path = tmp_path / 'loops.jsonl'
    path.write_text(''.join(json.dumps(record) + '\n' for record in records))
    return path


def record(seq, pending):
    return {'seq': seq, 'pending': pending, 'correction_count': 5}


def test_link_matching_truth_is_consistent(tmp_path):
    report = analyze_loops(write(tmp_path, [record(0, [edge([0, 0, 0], [1, 0, 0], [1.02, 0, 0])])]))
    assert report['consistent'] == 1 and report['contradicts_truth'] == 0
    assert report['events'][0]['translation_error_m'] == pytest.approx(0.02)


def test_relative_transform_uses_source_heading(tmp_path):
    source = [1, 1, math.pi / 2]
    report = analyze_loops(write(tmp_path, [record(0, [edge(source, [1, 3, math.pi / 2], [2, 0, 0])])]))
    assert report['events'][0]['translation_error_m'] == pytest.approx(0, abs=1e-9)
    assert report['events'][0]['true_distance_m'] == pytest.approx(2)


def test_link_across_wall_contradicts_truth(tmp_path):
    report = analyze_loops(write(tmp_path, [record(0, [edge([10, 6.7, 0], [10, 4, 0], [0.1, 0, 0])])]))
    assert report['contradicts_truth'] == 1
    assert report['events'][0]['translation_error_m'] == pytest.approx(math.hypot(0.1, 2.7))


def test_yaw_error_alone_contradicts_truth(tmp_path):
    report = analyze_loops(write(tmp_path, [record(0, [edge([0, 0, 0], [1, 0, 0], [1, 0, 0.2])])]))
    assert report['contradicts_truth'] == 1


def test_far_but_correct_link_is_not_false(tmp_path):
    report = analyze_loops(write(tmp_path, [record(0, [edge([0, 0, 0], [3, 0, 0], [3, 0, 0])])]))
    assert report['consistent'] == 1


def test_last_pending_edge_is_used(tmp_path):
    report = analyze_loops(write(tmp_path, [record(0, [
        edge([0, 0, 0], [5, 0, 0], [0, 0, 0]), edge([0, 0, 0], [1, 0, 0], [1, 0, 0])])]))
    assert report['consistent'] == 1
    assert report['events'][0]['pending_edge_count'] == 2


def test_no_new_edge_is_unknown(tmp_path):
    report = analyze_loops(write(tmp_path, [record(0, [])]))
    assert report['unknown'] == 1 and report['consistent'] == 0


def test_empty_log_means_zero_accepted(tmp_path):
    path = tmp_path / 'loops.jsonl'
    path.write_text('')
    assert analyze_loops(path)['accepted_count'] == 0


@pytest.mark.parametrize('line', ['not json', '{"seq": 0}', '{"seq":0,"pending":[{}],"correction_count":1}'])
def test_invalid_records_are_rejected(tmp_path, line):
    path = tmp_path / 'loops.jsonl'
    path.write_text(line + '\n')
    with pytest.raises(ValueError):
        analyze_loops(path)


def test_cli_does_not_overwrite(tmp_path, capsys):
    log = write(tmp_path, [record(0, [])])
    output = tmp_path / 'out.json'
    assert main(['--log', str(log), '--output', str(output)]) == 0
    assert main(['--log', str(log), '--output', str(output)]) == 1
    assert '루프 분석 실패' in capsys.readouterr().err


def test_sequential_edge_is_not_counted_as_loop(tmp_path):
    sequential = edge([0, 0, 0], [0.14, 0, 0], [0.14, 0, 0])
    sequential['source']['id'] = 8
    report = analyze_loops(write(tmp_path, [record(0, [sequential])]))
    assert report['unknown'] == 1 and report['consistent'] == 0
