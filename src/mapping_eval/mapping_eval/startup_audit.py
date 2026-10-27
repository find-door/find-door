"""원본 bag의 SLAM 시작 이전 입력과 독립 관측 window를 오프라인 감사한다."""

from collections import Counter
import argparse
import json
from pathlib import Path
import sys

from mapping_eval.bag_filter import STATIC_EDGES, TOPICS

EVENT_TOPIC = '/mapping_capture/events'


def positive_ns(value):
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def evaluate_diagnostics(summary):
    """센서 startup coverage와 별도로 진단 수집 자체의 완전성을 검사한다."""
    issues, warnings = [], []
    if summary.get('status') != 'completed':
        issues.append('diagnostics summary가 completed 상태가 아닙니다')
    counters = summary.get('counters', {})
    if not isinstance(counters, dict):
        counters = {}
        issues.append('diagnostics counters 형식 오류')
    for name in ('clock_received', 'map_odom_received', 'graph_received'):
        if not positive_ns(counters.get(name)):
            issues.append('diagnostics 필수 관측 누락: ' + name)
    if summary.get('errors') != []:
        issues.append('diagnostics errors가 비어 있는 목록이 아닙니다')
    for name, value in counters.items():
        if name.endswith('_invalid') or name in (
                'clock_backwards', 'tf_stamp_backwards', 'stream_close_errors'):
            if isinstance(value, bool) or not isinstance(value, int) or value != 0:
                issues.append('diagnostics 오류 counter: ' + name)
    if counters.get('cleanup_interrupt_or_error', 0):
        warnings.append('cleanup_interrupt_or_error: 반복 SIGINT 가능; 다른 오류와 별도로 검토 필요')
    return {'diagnostic_valid': not issues, 'diagnostic_issues': issues,
            'diagnostic_warnings': warnings}


def evaluate_startup_records(records, observed_scan_stamps, requiredstaticedges,
                             *, diagnostics=None):
    """정규화 record와 관측 stamp multiset 검사. 독립 observer의 무손실은 보장하지 않는다.

    record: topic, record_ns, header_ns/clock_ns/static_edges/event 중 해당 필드.
    diagnostics는 summary.json 사전이며 별도 diagnostic_valid에 완료·오류 상태를 기록한다.
    observed_scan_stamps는 capture_end monotonic 시각까지 수신한 scan만 전달한다.
    """
    issues = []
    diagnostics = diagnostics or {}
    scans = [r for r in records if r['topic'] == '/scan']
    odoms = [r for r in records if r['topic'] == '/odom']
    clocks = [r for r in records if r['topic'] == '/clock']
    dynamic = [dict(t, record_ns=r['record_ns']) for r in records if r['topic'] == '/tf'
               for t in r.get('dynamic_edges', [])
               if tuple(t.get('edge', ())) == ('odom', 'base_footprint')]
    for previous, current in zip(clocks, clocks[1:]):
        if isinstance(previous.get('clock_ns'), int) and isinstance(current.get('clock_ns'), int):
            if current['clock_ns'] < previous['clock_ns']:
                issues.append('bag clock이 역행했습니다')
                break
    if any(not isinstance(r.get('clock_ns'), int) or isinstance(r.get('clock_ns'), bool)
           or r['clock_ns'] < 0 for r in clocks):
        issues.append('bag clock은 음수가 아닌 정수여야 합니다')
    events = [r for r in records if r['topic'] == EVENT_TOPIC]
    starts = [r for r in events if r['event'].get('event') == 'slam_start']
    ends = [r for r in events if r['event'].get('event') == 'capture_end']
    if len(starts) != 1:
        issues.append('slam_start event는 정확히 하나여야 합니다')
    if len(ends) != 1:
        issues.append('capture_end event는 정확히 하나여야 합니다')
    for topic, rows, field in (('/scan', scans, 'header_ns'), ('/odom', odoms, 'header_ns')):
        if not rows or any(not positive_ns(r.get(field)) for r in rows):
            issues.append(topic + ' 입력이 없거나 양수 시각이 아닙니다')
    if not observed_scan_stamps or any(not positive_ns(s) for s in observed_scan_stamps):
        issues.append('독립 scan 관측이 없거나 stamp가 유효하지 않습니다')
    valid_scans = [r for r in scans if positive_ns(r.get('header_ns'))]
    bag_counts = Counter(r['header_ns'] for r in valid_scans)
    observed = Counter(s for s in observed_scan_stamps if positive_ns(s))
    missing = observed - bag_counts
    if missing:
        issues.append('독립 관측 scan 중 bag에 없는 표본이 있습니다')
    stage_clock = None
    missing_edges = set(requiredstaticedges)
    if len(starts) == 1:
        start = starts[0]
        event = start['event']
        stage_clock = event.get('receive_ros_ns')
        for key in ('receive_ros_ns', 'receive_monotonic_ns', 'seq',
                    'first_scan_stamp_ns', 'scan_count'):
            if not positive_ns(event.get(key)):
                issues.append('slam_start 진단 값 오류: ' + key)
        before = [r for r in valid_scans if r['record_ns'] < start['record_ns']]
        first = event.get('first_scan_stamp_ns')
        if positive_ns(first) and not any(r['header_ns'] == first for r in before):
            issues.append('진단 첫 scan이 시작 전 bag 기록에 없습니다')
        if positive_ns(first) and first not in observed:
            issues.append('진단 첫 scan이 sensor_stamps 기록에 없습니다')
        if positive_ns(event.get('scan_count')) and sum(observed.values()) < event['scan_count']:
            issues.append('sensor_stamps 기록이 시작 event의 scan_count보다 적습니다')
        if not before:
            issues.append('slam_start 레코드 전에 기록된 scan이 없습니다')
        if positive_ns(stage_clock):
            if not before or min(r['header_ns'] for r in before) > stage_clock:
                issues.append('시작 전 scan header가 SLAM 시작 clock보다 미래입니다')
            if (positive_ns(event.get('first_scan_stamp_ns'))
                    and event['first_scan_stamp_ns'] > stage_clock):
                issues.append('진단 첫 scan이 SLAM 시작 clock보다 미래입니다')
            if not any(r['record_ns'] < start['record_ns'] and
                       positive_ns(r.get('clock_ns')) and r['clock_ns'] <= stage_clock
                       for r in clocks):
                issues.append('SLAM 시작 전 양수 clock 기록이 없습니다')
        if not any(r['record_ns'] < start['record_ns'] and positive_ns(r.get('header_ns'))
                   and positive_ns(stage_clock) and r['header_ns'] <= stage_clock for r in dynamic):
            issues.append('SLAM 시작 전 odom→base_footprint TF가 없습니다')
        if not any(r['record_ns'] < start['record_ns'] for r in odoms):
            issues.append('SLAM 시작 전 odom 기록이 없습니다')
        for row in records:
            if row['topic'] == '/tf_static' and row['record_ns'] < start['record_ns']:
                missing_edges.difference_update(tuple(edge) for edge in row['static_edges'])
        if missing_edges:
            issues.append('SLAM 시작 전 필수 static TF edge가 누락되었습니다')
    end_clock = None
    if len(ends) == 1:
        end_clock = ends[0]['event'].get('receive_ros_ns')
        end_mono = ends[0]['event'].get('receive_monotonic_ns')
        if not positive_ns(end_mono):
            issues.append('capture_end monotonic 시각이 양수가 아닙니다')
        if len(starts) == 1:
            start_mono = starts[0]['event'].get('receive_monotonic_ns')
            if (ends[0]['record_ns'] <= starts[0]['record_ns']
                    or (positive_ns(end_mono) and positive_ns(start_mono)
                        and end_mono <= start_mono)):
                issues.append('capture_end event 순서가 SLAM 시작 이후가 아닙니다')
        if not positive_ns(end_clock):
            issues.append('capture_end clock이 양수가 아닙니다')
        else:
            if positive_ns(stage_clock) and end_clock <= stage_clock:
                issues.append('capture_end clock이 SLAM 시작 이후가 아닙니다')
            for label, rows, field, tolerance in (
                    ('clock', clocks, 'clock_ns', 0),
                    ('odom', odoms, 'header_ns', 100_000_000),
                    ('odom_base_tf', dynamic, 'header_ns', 100_000_000),
                    ('scan', scans, 'header_ns', 300_000_000)):
                stamps = [r[field] for r in rows if positive_ns(r.get(field))]
                if not stamps or max(stamps) < end_clock - tolerance:
                    issues.append('bag 후반 ' + label + '이 capture_end까지 포함하지 않습니다')
    timestamps = {}
    for topic, rows, key in (('scan', scans, 'header_ns'), ('odom', odoms, 'header_ns'),
                             ('clock', clocks, 'clock_ns'), ('odom_base_tf', dynamic, 'header_ns')):
        stamps = [r[key] for r in rows if positive_ns(r.get(key))]
        timestamps[topic] = {'first_ns': min(stamps) if stamps else None,
                             'last_ns': max(stamps) if stamps else None}
    return {'schema_version': 1, 'startup_coverage_valid': not issues, 'issues': issues,
            **evaluate_diagnostics(diagnostics),
            'counts': {'bag_scan': len(scans), 'bag_odom': len(odoms), 'bag_clock': len(clocks),
                       'bag_odom_base_tf': len(dynamic),
                       'slam_start_events': len(starts), 'capture_end_events': len(ends),
                       'observed_scan': len(observed_scan_stamps),
                       'bag_missing_observed_scans': sum(missing.values())},
            'missing_scan_stamp_counts': dict(sorted(missing.items())),
            'missing_static_edges': [list(e) for e in sorted(missing_edges)],
            'timestamps': timestamps, 'slam_start_clock_ns': stage_clock,
            'capture_end_clock_ns': end_clock,
            'diagnostic_readiness': {'status': diagnostics.get('status'),
                                     'completed': diagnostics.get('status') == 'completed'},
            'limitations': '독립 observer도 손실될 수 있으므로 완전 무손실 증명이 아님; '
                           '초기 clock=0은 허용하나 시작 전 양수 clock 필수, 음수/역행 거부; '
                           '후반 coverage는 capture_end 기준이며 odom/TF 0.1s, scan 0.3s 허용'}


def audit_startup(bagpath, diagnostics_dir):
    """ROS 노드를 만들지 않고 닫힌 원본 bag을 읽는다. 입력은 수정하지 않는다."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bagpath, diagnostics_dir = Path(bagpath), Path(diagnostics_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bagpath), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    expected = dict(TOPICS, **{EVENT_TOPIC: 'std_msgs/msg/String'})
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in expected:
        if topic in types and types[topic] != expected[topic]:
            raise ValueError('bag 토픽 타입 불일치: ' + topic)
    classes = {name: get_message(types[name]) for name in expected if name in types}
    records = []
    def ns(stamp):
        return stamp.sec * 1_000_000_000 + stamp.nanosec
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        if topic not in classes:
            continue
        message = deserialize_message(payload, classes[topic])
        row = {'topic': topic, 'record_ns': timestamp}
        if topic in ('/scan', '/odom'):
            row['header_ns'] = ns(message.header.stamp)
        elif topic == '/clock':
            row['clock_ns'] = ns(message.clock)
        elif topic == '/tf':
            row['dynamic_edges'] = [
                {'edge': (t.header.frame_id, t.child_frame_id), 'header_ns': ns(t.header.stamp)}
                for t in message.transforms]
        elif topic == '/tf_static':
            row['static_edges'] = [(t.header.frame_id, t.child_frame_id) for t in message.transforms]
        else:
            row['event'] = json.loads(message.data)
        records.append(row)
    del reader
    summary_path = diagnostics_dir / 'summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.is_file() else {}
    end_events = [r['event'] for r in records if r['topic'] == EVENT_TOPIC
                  and r['event'].get('event') == 'capture_end']
    cutoff = end_events[0].get('receive_monotonic_ns') if len(end_events) == 1 else None
    stamps, malformed = [], False
    source = diagnostics_dir / 'sensor_stamps.jsonl'
    if source.is_file():
        for line in source.read_text().splitlines():
            row = json.loads(line)
            if row.get('topic') != '/scan':
                continue
            if not positive_ns(row.get('receive_monotonic_ns')):
                malformed = True
            elif positive_ns(cutoff) and row['receive_monotonic_ns'] <= cutoff:
                stamps.append(row.get('header_stamp_ns'))
    result = evaluate_startup_records(records, stamps, STATIC_EDGES, diagnostics=summary)
    if malformed or not positive_ns(cutoff):
        result['issues'].append('진단 scan window 또는 capture_end monotonic 시각이 유효하지 않습니다')
        result['startup_coverage_valid'] = False
    result['bag'] = str(bagpath.resolve())
    result['diagnostics'] = str(diagnostics_dir.resolve())
    result['scan_comparison_window'] = {'end_receive_monotonic_ns': cutoff,
                                       'policy': 'observer receive <= capture_end; bag 전체 multiset과 비교'}
    return result


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--diagnostics', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    options = parser.parse_args(args)
    output = options.output.resolve()
    if output.exists() or any(p.resolve() in output.parents for p in (options.bag, options.diagnostics)):
        parser.error('출력은 입력 바깥의 새 파일이어야 합니다')
    try:
        result = audit_startup(options.bag, options.diagnostics)
        with output.open('x') as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
        valid = result['startup_coverage_valid'] and result['diagnostic_valid']
        print('시작 입력·진단 감사: ' + ('통과' if valid else '실패'))
        return 0 if valid else 1
    except (Exception, KeyboardInterrupt) as exc:
        print('시작 입력 감사 실패: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
