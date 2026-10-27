"""시작 누락을 후반 수신이나 불완전 diagnostics로 감추지 않는지 검사한다."""
from copy import deepcopy

from mapping_eval.startup_audit import EVENT_TOPIC, evaluate_startup_records

EDGES = {('base', 'lidar')}


def records():
    rows = [
        {'topic': '/clock', 'record_ns': 1, 'clock_ns': 1_000_000_000},
        {'topic': '/scan', 'record_ns': 2, 'header_ns': 1_000_000_000},
        {'topic': '/odom', 'record_ns': 3, 'header_ns': 1_000_000_000},
        {'topic': '/tf_static', 'record_ns': 4, 'static_edges': list(EDGES)},
        {'topic': EVENT_TOPIC, 'record_ns': 5, 'event': {
            'event': 'slam_start', 'receive_ros_ns': 1_000_000_000,
            'receive_monotonic_ns': 100, 'seq': 1,
            'first_scan_stamp_ns': 1_000_000_000, 'scan_count': 1}},
        {'topic': '/scan', 'record_ns': 6, 'header_ns': 2_000_000_000},
        {'topic': '/odom', 'record_ns': 7, 'header_ns': 2_000_000_000},
        {'topic': '/clock', 'record_ns': 8, 'clock_ns': 2_000_000_000},
        {'topic': EVENT_TOPIC, 'record_ns': 9, 'event': {
            'event': 'capture_end', 'receive_ros_ns': 2_000_000_000,
            'receive_monotonic_ns': 200, 'seq': 2}},
    ]
    rows.extend([
        {'topic': '/tf', 'record_ns': 3, 'dynamic_edges': [
            {'edge': ('odom', 'base_footprint'), 'header_ns': 1_000_000_000}]},
        {'topic': '/tf', 'record_ns': 8, 'dynamic_edges': [
            {'edge': ('odom', 'base_footprint'), 'header_ns': 2_000_000_000}]},
    ])
    return rows


def evaluate(rows=None, stamps=None, diagnostics=None):
    return evaluate_startup_records(
        records() if rows is None else rows,
        [1_000_000_000, 2_000_000_000] if stamps is None else stamps, EDGES,
        diagnostics={'status': 'completed', 'errors': [], 'counters': {
            'clock_received': 2, 'map_odom_received': 2, 'graph_received': 1}}
        if diagnostics is None else diagnostics)


def test_complete_start_and_endpoint_pass():
    assert evaluate()['startup_coverage_valid']


def test_missing_early_scan_fails_despite_later_input():
    rows = records()
    del rows[1]
    result = evaluate(rows)
    assert not result['startup_coverage_valid']
    assert result['counts']['bag_missing_observed_scans'] == 1


def test_duplicate_event_rejected():
    rows = records()
    rows.append(deepcopy(rows[4]))
    assert not evaluate(rows)['startup_coverage_valid']


def test_zero_or_future_start_clock_rejected():
    for clock in (0, 999_999_999):
        rows = records()
        rows[4]['event']['receive_ros_ns'] = clock
        assert not evaluate(rows)['startup_coverage_valid']


def test_static_after_start_does_not_count():
    rows = records()
    rows[3]['record_ns'] = 10
    result = evaluate(rows)
    assert not result['startup_coverage_valid']
    assert result['missing_static_edges'] == [['base', 'lidar']]


def test_partial_diagnostics_fail_even_when_bag_complete():
    assert not evaluate(diagnostics={'status': 'running'})['diagnostic_valid']
    assert not evaluate(diagnostics={})['diagnostic_valid']
    assert evaluate(diagnostics={})['startup_coverage_valid']
    assert not evaluate(stamps=[])['startup_coverage_valid']


def test_multiset_duplicate_missing_is_not_hidden_by_set():
    result = evaluate(stamps=[1_000_000_000, 1_000_000_000, 2_000_000_000])
    assert result['counts']['bag_missing_observed_scans'] == 1


def test_source_file_partial_against_event_count_rejected():
    rows = records()
    rows[4]['event']['scan_count'] = 3
    assert not evaluate(rows)['startup_coverage_valid']


def test_truncated_end_clock_rejected():
    rows = records()
    rows[7]['clock_ns'] = 1_500_000_000
    assert not evaluate(rows)['startup_coverage_valid']


def test_end_monotonic_or_record_order_rejected():
    rows = records()
    rows[8]['event']['receive_monotonic_ns'] = 99
    assert not evaluate(rows)['startup_coverage_valid']
    rows = records()
    rows[8]['record_ns'] = 4
    assert not evaluate(rows)['startup_coverage_valid']


def test_missing_dynamic_tf_or_reversed_edge_rejected():
    rows = [r for r in records() if r['topic'] != '/tf']
    assert not evaluate(rows)['startup_coverage_valid']
    rows = records()
    for row in rows:
        if row['topic'] == '/tf':
            row['dynamic_edges'][0]['edge'] = ('base_footprint', 'odom')
    assert not evaluate(rows)['startup_coverage_valid']


def test_dynamic_endpoint_must_cover_capture_end():
    rows = records()
    rows[-1]['dynamic_edges'][0]['header_ns'] = 1_800_000_000
    assert not evaluate(rows)['startup_coverage_valid']


def test_initial_zero_clock_allowed_but_negative_or_reversal_rejected():
    rows = records()
    rows.insert(0, {'topic': '/clock', 'record_ns': 0, 'clock_ns': 0})
    assert evaluate(rows)['startup_coverage_valid']
    rows[0]['clock_ns'] = -1
    assert not evaluate(rows)['startup_coverage_valid']
    rows[0]['clock_ns'] = 1_500_000_000
    assert not evaluate(rows)['startup_coverage_valid']


def test_diagnostic_errors_are_separate_from_sensor_coverage():
    from mapping_eval.startup_audit import evaluate_diagnostics
    good = {'status': 'completed', 'errors': [], 'counters': {
        'clock_received': 2, 'map_odom_received': 2, 'graph_received': 1}}
    assert evaluate_diagnostics(good)['diagnostic_valid']
    for counter in ('scan_invalid', 'clock_backwards', 'tf_stamp_backwards', 'stream_close_errors'):
        bad = deepcopy(good)
        bad['counters'][counter] = 1
        result = evaluate(diagnostics=bad)
        assert not result['diagnostic_valid']
        assert result['startup_coverage_valid']
    bad = deepcopy(good)
    bad['counters']['graph_received'] = 0
    assert not evaluate_diagnostics(bad)['diagnostic_valid']
    bad = deepcopy(good)
    bad['errors'] = ['error']
    assert not evaluate_diagnostics(bad)['diagnostic_valid']
    good['counters']['cleanup_interrupt_or_error'] = 1
    assert evaluate_diagnostics(good)['diagnostic_valid']
    assert evaluate_diagnostics(good)['diagnostic_warnings']
