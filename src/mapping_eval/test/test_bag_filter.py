"""필터의 입력 보존·SLAM 출력 차단·실패 시 자식 정리 계약 검사."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest

from mapping_eval.bag_filter import STATIC_EDGES, TOPICS, filter_bag, select_transforms
from mapping_eval.bag_filter import validate_output
from mapping_eval.bag_filter import replay_timing_issues
from mapping_eval.replay_benchmark import cleanup_children, playback_command


def transform(parent, child):
    return NS(header=NS(frame_id=parent, stamp=NS(sec=3, nanosec=4)),
              child_frame_id=child)


def test_dynamic_only_allows_exact_odom_base_edge():
    good = transform('odom', 'base_footprint')
    bad = [transform('map', 'odom'), transform('map', 'base_footprint'),
           transform('base_footprint', 'odom'), transform('/odom', 'base_footprint'),
           transform('odom', 'camera_link0')]
    assert select_transforms('/tf', [*bad, good]) == [good]
    assert len(bad) == 5


def test_static_only_allows_exact_robot_tree_edges():
    good = [transform(*edge) for edge in STATIC_EDGES]
    bad = [transform('map', 'odom'), transform('odom', 'base_footprint'),
           transform('base_link', 'unknown_camera')]
    assert select_transforms('/tf_static', good + bad) == good
    assert select_transforms('/other', good) == []


def test_output_must_be_new_and_outside_input_bag(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'metadata.yaml').write_text('test')
    for output in (source, source / 'filtered'):
        with pytest.raises(ValueError):
            validate_output(source, output)
    existing = tmp_path / 'exists'
    existing.mkdir()
    with pytest.raises(ValueError):
        validate_output(source, existing)


def test_playback_never_synthesizes_clock_and_starts_paused():
    command = playback_command('/tmp/bag', 0.5)
    assert '--clock' not in command
    assert '--start-paused' in command
    assert '--disable-keyboard-controls' in command
    assert command[command.index('--rate') + 1] == '0.5'


def test_cleanup_continues_after_one_child_fails():
    stopped, closed = [], []

    def stop(child):
        stopped.append(child)
        if child == 'second':
            raise RuntimeError('second 종료 실패')

    handles = [NS(close=lambda: closed.append('closed'))]
    issues = cleanup_children(['first', 'second'], handles, stop=stop)
    assert stopped == ['second', 'first']
    assert closed == ['closed']
    assert issues == ['second 종료 실패']


def fake_rosbag(monkeypatch, rows, invalid_type=False):
    topics = [NS(name=name, type=kind, serialization_format='cdr',
                 offered_qos_profiles='original_qos:' + name) for name, kind in TOPICS.items()]
    if invalid_type:
        topics[0].type = 'std_msgs/msg/String'
    written, created, closed = [], [], []

    class Reader:
        def open(self, *args):
            self.index = 0

        def get_metadata(self):
            return NS(duration=NS(nanoseconds=123_000_000))

        def get_all_topics_and_types(self):
            return topics

        def has_next(self):
            return self.index < len(rows)

        def read_next(self):
            row = rows[self.index]
            self.index += 1
            return row

    class Writer:
        def open(self, storage, converter):
            Path(storage.uri).mkdir()

        def create_topic(self, topic):
            created.append(topic)

        def write(self, *record):
            written.append(record)

        def close(self):
            closed.append(True)

    class FakeOdometry:
        pass

    def deserialize(data, kind):
        if kind is FakeOdometry:
            return NS(header=NS(stamp=NS(sec=3, nanosec=4)))
        return NS(transforms=[transform(*edge) for edge in json.loads(data)])

    def serialize(message):
        return json.dumps([(item.header.frame_id, item.child_frame_id)
                           for item in message.transforms]).encode()

    monkeypatch.setitem(sys.modules, 'rosbag2_py', NS(
        SequentialReader=Reader, SequentialWriter=Writer,
        StorageOptions=lambda **kwargs: NS(**kwargs), ConverterOptions=lambda *args: args))
    monkeypatch.setitem(sys.modules, 'rclpy', NS())
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', NS(
        deserialize_message=deserialize, serialize_message=serialize))
    monkeypatch.setitem(sys.modules, 'tf2_msgs', NS())
    monkeypatch.setitem(sys.modules, 'tf2_msgs.msg', NS(TFMessage=object))
    monkeypatch.setitem(sys.modules, 'nav_msgs', NS())
    monkeypatch.setitem(sys.modules, 'nav_msgs.msg', NS(Odometry=FakeOdometry))
    return written, created, closed


def source_bag(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'metadata.yaml').write_text('source metadata unchanged')
    return source


def sample_rows():
    return [('/clock', b'original clock', 100),
            ('/scan', b'original scan', 101),
            ('/cmd_vel', b'never replay commands', 101),
            ('/tf', json.dumps([['map', 'odom'], ['odom', 'base_footprint']]).encode(), 102),
            ('/odom', b'original odom', 102),
            ('/tf_static', json.dumps(sorted(STATIC_EDGES)).encode(), 103),
            ('/map', b'never replay SLAM outputs', 104),
            ('/tf', json.dumps([['map', 'base_footprint']]).encode(), 105)]


def test_filter_preserves_order_timestamp_payload_and_qos(monkeypatch, tmp_path):
    written, created, closed = fake_rosbag(monkeypatch, sample_rows())
    source = source_bag(tmp_path)
    before = (source / 'metadata.yaml').read_bytes()
    report = filter_bag(source, tmp_path / 'result')
    assert [row[0] for row in written] == ['/clock', '/scan', '/tf', '/odom', '/tf_static']
    assert [row[2] for row in written] == [100, 101, 102, 102, 103]
    assert written[1][1] == b'original scan'
    assert json.loads(written[2][1]) == [['odom', 'base_footprint']]
    assert written[4][1] == sample_rows()[5][1]  # 허용된 static TF는 CDR 그대로 보존.
    assert all(topic.offered_qos_profiles == 'original_qos:' + topic.name for topic in created)
    assert report['status'] == 'completed'
    assert report['dropped_counts']['/cmd_vel'] == 1
    assert report['dropped_tf_edges']['/tf:map->odom'] == 1
    assert report['first_odom_header_stamp_ns'] == 3_000_000_004
    assert report['last_odom_header_stamp_ns'] == 3_000_000_004
    assert closed == [True]
    assert len(report['retained_record_payload_sha256']) == 64
    assert (source / 'metadata.yaml').read_bytes() == before


def test_filter_failure_closes_writer_and_keeps_failure_manifest(monkeypatch, tmp_path):
    written, _, closed = fake_rosbag(monkeypatch, sample_rows()[:5])
    with pytest.raises(ValueError):
        filter_bag(source_bag(tmp_path), tmp_path / 'failed')
    report = json.loads((tmp_path / 'failed/filter_manifest.json').read_text())
    assert report['status'] == 'failed'
    assert report['kept_counts']['/odom'] == 1
    assert written
    assert closed == [True]


def test_type_mismatch_rejected_before_writer(monkeypatch, tmp_path):
    written, _, closed = fake_rosbag(monkeypatch, sample_rows(), invalid_type=True)
    with pytest.raises(ValueError, match='타입'):
        filter_bag(source_bag(tmp_path), tmp_path / 'failed')
    assert written == []
    assert closed == []


def test_replay_timing_uses_original_bag_endpoint_not_last_received_sample():
    # 수신 odom과 TF 표본이 둘 다 8초에 멈춰도 원본 bag은 10초까지 있다.
    issues = replay_timing_issues(10_000_000_000, 8_000_000_000,
                                 10_050_000_000, 10_000_000_000)
    assert issues['trajectory']
    assert issues['map'] == []


def test_replay_timing_accepts_recent_trace_clock_and_map():
    assert replay_timing_issues(10_000_000_000, 9_950_000_000,
                               10_050_000_000, 9_000_000_000) == {
        'trajectory': [], 'map': []}


@pytest.mark.parametrize('clock', [None, 0, 9_999_999_999, 11_000_000_001])
def test_replay_clock_must_reach_and_stay_near_original_end(clock):
    issues = replay_timing_issues(10_000_000_000, 10_000_000_000, clock, 10_000_000_000)
    assert issues['trajectory']


@pytest.mark.parametrize('stamp', [None, 0, 8_049_999_999, 10_050_000_001])
def test_replay_final_map_must_be_within_two_seconds_and_not_future(stamp):
    issues = replay_timing_issues(10_000_000_000, 10_000_000_000,
                                 10_050_000_000, stamp)
    assert issues['map']


def test_replay_missing_endpoint_or_future_trace_is_rejected():
    assert replay_timing_issues(None, 10, 10, 10)['trajectory']
    assert replay_timing_issues(10, 11, 10, 10)['trajectory']


def test_cleanup_reports_process_group_that_survives():
    from types import SimpleNamespace
    from mapping_eval.replay_benchmark import cleanup_children
    problems = cleanup_children([SimpleNamespace(pid=123)], [], stop=lambda child: False)
    assert len(problems) == 1 and '123' in problems[0]


def test_replay_profile_restores_static_durability_without_synthetic_clock():
    command = playback_command('/tmp/bag', qos_overrides='/tmp/replay.yaml')
    assert '--clock' not in command
    assert command[command.index('--qos-profile-overrides-path') + 1] == '/tmp/replay.yaml'
