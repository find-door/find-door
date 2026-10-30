import json

import pytest

from mapping_eval.graph_motion import analyze_graph, main


def write_graph(tmp_path, snapshots):
    path = tmp_path / 'graph.jsonl'
    lines = []
    for index, nodes in enumerate(snapshots):
        lines.append(json.dumps({'receive_ros_ns': index * 1_000_000_000, 'nodes': [
            {'marker_id': key, 'position': [*value, 0.0], 'frame_id': 'map'}
            for key, value in nodes.items()]}))
    path.write_text('\n'.join(lines) + '\n')
    return path


def test_small_corrections_and_new_nodes_are_not_candidates(tmp_path):
    report = analyze_graph(write_graph(tmp_path, [
        {0: (0, 0)}, {0: (0.05, 0), 1: (5, 0)}, {0: (0.08, 0), 1: (5, 0), 2: (9, 0)}]))
    assert not report['anomaly_candidate']
    assert report['max_step_m'] == pytest.approx(0.05)
    assert report['max_final_drift_m'] == pytest.approx(0.08)
    assert report['final_node_count'] == 3


def test_past_node_displacement_is_reported_as_event(tmp_path):
    report = analyze_graph(write_graph(tmp_path, [
        {0: (0, 0), 1: (1, 0)}, {0: (0, 3.5), 1: (1, 0), 2: (2, 0)}]))
    assert report['anomaly_candidate']
    assert report['step_events'] == [{
        'receive_ros_ns': 1_000_000_000, 'max_step_m': pytest.approx(3.5),
        'moved_node_count': 1, 'compared_node_count': 2, 'moved_node_ids': [0]}]
    assert report['drifted_node_ids'] == [0]


def test_slow_accumulated_drift_is_candidate_without_step_event(tmp_path):
    report = analyze_graph(write_graph(tmp_path, [
        {0: (0, 0)}, {0: (0.3, 0)}, {0: (0.6, 0)}]))
    assert report['step_event_count'] == 0
    assert report['drifted_node_ids'] == [0]
    assert report['anomaly_candidate']


def test_empty_snapshots_do_not_count(tmp_path):
    path = write_graph(tmp_path, [{}, {0: (0, 0)}])
    with pytest.raises(ValueError, match='2개 미만'):
        analyze_graph(path)


@pytest.mark.parametrize('line', [
    'not json',
    '{"receive_ros_ns": 0, "nodes": [{"marker_id": 0, "position": [NaN, 0], "frame_id": "map"}]}',
    '{"receive_ros_ns": 0, "nodes": [{"marker_id": 0, "position": [0, 0], "frame_id": "odom"}]}',
    '{"nodes": []}',
])
def test_invalid_records_are_rejected(tmp_path, line):
    path = tmp_path / 'graph.jsonl'
    path.write_text(line + '\n')
    with pytest.raises(ValueError):
        analyze_graph(path)


def test_clock_regression_is_rejected(tmp_path):
    path = write_graph(tmp_path, [{0: (0, 0)}, {0: (0, 0)}])
    records = path.read_text().splitlines()
    path.write_text(records[1] + '\n' + records[0] + '\n')
    with pytest.raises(ValueError, match='역행'):
        analyze_graph(path)


@pytest.mark.parametrize('threshold', [0, -1, float('nan'), float('inf')])
def test_invalid_threshold_is_rejected(tmp_path, threshold):
    with pytest.raises(ValueError):
        analyze_graph(write_graph(tmp_path, [{0: (0, 0)}, {0: (0, 0)}]), threshold)


def test_cli_does_not_overwrite_output(tmp_path, capsys):
    graph = write_graph(tmp_path, [{0: (0, 0)}, {0: (0, 0)}])
    output = tmp_path / 'out/graph_motion.json'
    assert main(['--graph', str(graph), '--output', str(output)]) == 0
    assert json.loads(output.read_text())['anomaly_candidate'] is False
    assert main(['--graph', str(graph), '--output', str(output)]) == 1
    assert '그래프 분석 실패' in capsys.readouterr().err
