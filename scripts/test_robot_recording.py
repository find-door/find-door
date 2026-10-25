"""실물 bag 도구의 원본 보호·TF 필터·시간·종료 계약. ROS/모델 다운로드 없음."""
import importlib.util
from pathlib import Path
import pickle
import signal
import sys
from types import SimpleNamespace as NS

import pytest
import yaml

spec = importlib.util.spec_from_file_location('robot_recording', Path(__file__).with_name('robot_recording.py'))
r = importlib.util.module_from_spec(spec)
spec.loader.exec_module(r)


@pytest.fixture
def profile():
    return {
        'schema_version': 1, 'robot_id': 'test_robot', 'hardware_confirmed': True,
        'frames': {'map': 'map', 'odom': 'odom', 'base': 'base_link', 'laser': 'laser', 'optical': None},
        'topics': {'scan': '/lidar/scan', 'odom': '/wheel/odom', 'rgb': None, 'depth': None,
                   'camera_info': None, 'detections': None},
        'thresholds': {'min_scan_hz': 5., 'min_odom_hz': 10., 'max_age_sec': .5},
        'slam': {'resolution': .05, 'min_laser_range': .1, 'max_laser_range': 10., 'do_loop_closing': False},
        'semantic': {'enabled': False, 'map_id': None, 'model_path': None, 'model_sha256': None,
                     'output_path': None, 'sync_policy': 'exact', 'max_sync_skew_sec': .03,
                     'aligned_depth_confirmed': False, 'calibration_id': None,
                     'registration_note': None, 'static_classes': ['chair', 'cup', 'bottle']}}


def header(frame, sec=1):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=0))


def transform(parent, child, sec=1, x=0.):
    return NS(header=header(parent, sec), child_frame_id=child,
              transform=NS(translation=NS(x=x, y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))


def test_profile_confirmation_and_optional_camera(tmp_path, profile):
    path = tmp_path / 'profile.yaml'
    path.write_text(yaml.safe_dump(profile))
    loaded, digest = r.load_profile(path)
    assert loaded == profile and len(digest) == 64
    assert set(r.topic_types(profile)) == {'/lidar/scan', '/wheel/odom', '/tf', '/tf_static'}
    profile['hardware_confirmed'] = False
    path.write_text(yaml.safe_dump(profile))
    with pytest.raises(ValueError, match='hardware_confirmed'):
        r.load_profile(path)


def test_record_and_replay_commands_never_send_velocity(profile):
    cmd = r.record_command('/tmp/new', '/tmp/qos', profile)
    assert '/cmd_vel' not in cmd and '/clock' not in cmd
    cmd = r.replay_command('/tmp/filtered', '/tmp/qos')
    assert cmd.count('--clock') == 1 and '100' in cmd
    qos = r.qos_profiles(profile, replay=True)
    assert qos['/tf_static']['durability'] == 'transient_local'
    assert r.qos_profiles(profile)['/lidar/scan']['reliability'] == 'best_effort'


@pytest.mark.parametrize('env,confirmed', [({}, True), ({'ROS_DOMAIN_ID': '0', 'GZ_PARTITION': 'codex'}, True),
                                         ({'ROS_DOMAIN_ID': '12', 'GZ_PARTITION': 'codex'}, False)])
def test_replay_isolation_required(env, confirmed):
    with pytest.raises(ValueError):
        r.assert_domain(env, confirmed)


def test_existing_nested_and_symlink_output_rejected(tmp_path):
    bag = tmp_path / 'bag'
    bag.mkdir()
    alias = tmp_path / 'alias'
    alias.symlink_to(bag, target_is_directory=True)
    for output in [bag, bag / 'nested', alias / 'nested']:
        with pytest.raises(ValueError):
            r.new_path(output, bag)
    assert r.new_path(tmp_path / 'new', bag) == tmp_path / 'new'


def test_static_paths_exclude_slam_and_other_branches(profile):
    edges = [('base_link', 'mount'), ('mount', 'laser'), ('map', 'odom'),
             ('odom', 'base_link'), ('other', 'camera')]
    assert r.static_edges_for_profile(edges, profile) == {('base_link', 'mount'), ('mount', 'laser')}


@pytest.mark.parametrize('edges', [[('map', 'laser')], [('odom', 'laser')],
                                   [('base_link', 'laser'), ('other', 'laser')],
                                   [('mount', 'laser'), ('laser', 'mount')]])
def test_missing_ambiguous_or_cyclic_static_paths_rejected(profile, edges):
    with pytest.raises(ValueError):
        r.static_edges_for_profile(edges, profile)


def test_dynamic_tf_only_direct_profile_edge_and_nonnegative_static(profile):
    messages = [transform('map', 'odom'), transform('odom', 'base_link'),
                transform('base_link', 'odom'), transform('odom', 'camera')]
    previous = {}
    assert r.select_tf('/tf', messages, profile, set(), previous) == [messages[1]]
    with pytest.raises(ValueError):
        r.select_tf('/tf', [transform('odom', 'base_link', 0)], profile, set(), previous)
    item = transform('base_link', 'laser', 0)
    assert r.select_tf('/tf_static', [item], profile, {('base_link', 'laser')}, {}) == [item]


def test_sensor_frames_positive_and_monotonic(profile):
    previous = {}
    msg = NS(header=header('laser', 2))
    r.validate_sensor('/lidar/scan', msg, profile, previous)
    with pytest.raises(ValueError, match='역행'):
        r.validate_sensor('/lidar/scan', NS(header=header('laser', 1)), profile, previous)
    with pytest.raises(ValueError, match='frame'):
        r.validate_sensor('/lidar/scan', NS(header=header('wrong', 3)), profile, previous)
    with pytest.raises(ValueError, match='child'):
        r.validate_sensor('/wheel/odom', NS(header=header('odom'), child_frame_id='wrong'), profile, {})


def fake_rosbag(monkeypatch, rows, profile, wrong_type=False):
    metadata = [NS(name=topic, type=kind, serialization_format='cdr', offered_qos_profiles='')
                for topic, kind in r.topic_types(profile).items()]
    if wrong_type:
        metadata[0].type = 'std_msgs/msg/String'
    written, closed = [], []

    class Reader:
        def open(self, *args):
            self.index = 0

        def get_all_topics_and_types(self):
            return metadata

        def has_next(self):
            return self.index < len(rows)

        def read_next(self):
            row = rows[self.index]
            self.index += 1
            return row

    class Writer:
        def open(self, storage, converter):
            Path(storage.uri).mkdir()

        def create_topic(self, metadata):
            pass

        def write(self, *record):
            written.append(record)

        def close(self):
            closed.append(True)

    monkeypatch.setitem(sys.modules, 'rosbag2_py', NS(
        SequentialReader=Reader, SequentialWriter=Writer,
        StorageOptions=lambda **kw: NS(**kw), ConverterOptions=lambda *args: args))
    monkeypatch.setitem(sys.modules, 'rclpy', NS())
    monkeypatch.setitem(sys.modules, 'rclpy.serialization', NS(
        deserialize_message=lambda data, cls: pickle.loads(data), serialize_message=pickle.dumps))
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py', NS())
    monkeypatch.setitem(sys.modules, 'rosidl_runtime_py.utilities', NS(get_message=lambda name: object))
    monkeypatch.setitem(sys.modules, 'tf2_msgs', NS())
    monkeypatch.setitem(sys.modules, 'tf2_msgs.msg', NS(TFMessage=object))
    return written, closed


def rows():
    return [(topic, pickle.dumps(message), index + 1) for index, (topic, message) in enumerate([
        ('/tf_static', NS(transforms=[transform('base_link', 'laser', 0), transform('map', 'odom', 0)])),
        ('/lidar/scan', NS(header=header('laser'))),
        ('/wheel/odom', NS(header=header('odom'), child_frame_id='base_link')),
        ('/tf', NS(transforms=[transform('odom', 'base_link'), transform('map', 'odom')])),
        ('/map', NS()), ('/cmd_vel', NS()),
    ])]


def make_bag(tmp_path):
    source = tmp_path / 'source'
    source.mkdir()
    (source / 'metadata.yaml').write_text('original metadata')
    return source


def test_filter_keeps_original_sensor_bytes_drops_map_and_velocity(tmp_path, monkeypatch, profile):
    inputs = rows()
    written, closed = fake_rosbag(monkeypatch, inputs, profile)
    source = make_bag(tmp_path)
    report = r.filter_bag(source, tmp_path / 'new', profile)
    assert (source / 'metadata.yaml').read_text() == 'original metadata'
    assert [row[0] for row in written] == ['/tf_static', '/lidar/scan', '/wheel/odom', '/tf']
    assert written[1] == inputs[1]
    assert report['dropped'] == {'/map': 1, '/cmd_vel': 1}
    for topic, data, _ in written:
        if topic in ('/tf', '/tf_static'):
            assert all(t.header.frame_id != 'map' for t in pickle.loads(data).transforms)
    assert closed == [True]


@pytest.mark.parametrize('problem', ['clock', 'type', 'missing_odom', 'backwards', 'static_changed'])
def test_filter_rejects_invalid_inputs(tmp_path, monkeypatch, profile, problem):
    inputs = rows()
    if problem == 'clock':
        inputs.append(('/clock', b'never used', 10))
    elif problem == 'missing_odom':
        inputs = [row for row in inputs if row[0] != '/wheel/odom']
    elif problem == 'backwards':
        inputs.append(('/lidar/scan', pickle.dumps(NS(header=header('laser', 0))), 10))
    elif problem == 'static_changed':
        inputs.append(('/tf_static', pickle.dumps(NS(transforms=[transform('base_link', 'laser', x=.1)])), 10))
    fake_rosbag(monkeypatch, inputs, profile, wrong_type=problem == 'type')
    with pytest.raises(ValueError):
        r.filter_bag(make_bag(tmp_path), tmp_path / 'new', profile)


def test_record_audit_does_not_create_writer(tmp_path, monkeypatch, profile):
    written, closed = fake_rosbag(monkeypatch, rows(), profile)
    report = r.filter_bag(make_bag(tmp_path), None, profile)
    assert report['kept']['/lidar/scan'] == 1
    assert written == [] and closed == []


def test_stop_child_reaps_only_owned_group(monkeypatch):
    calls, polls = [], []
    def killpg(pid, sig):
        calls.append((pid, sig))
        if sig == 0:
            raise ProcessLookupError()
    monkeypatch.setattr(r.os, 'killpg', killpg)
    child = NS(pid=123456, poll=lambda: polls.append(True))
    assert r.stop_child(child)
    assert calls == [(123456, signal.SIGINT), (123456, 0)]
    assert polls


def test_stop_child_escalates_when_descendants_remain(monkeypatch):
    calls = []
    now = iter(range(0, 1000, 20))
    monkeypatch.setattr(r.time, 'monotonic', lambda: next(now))
    monkeypatch.setattr(r.os, 'killpg', lambda pid, sig: calls.append((pid, sig)))
    child = NS(pid=456789, poll=lambda: 0)
    assert not r.stop_child(child)
    assert calls == [(456789, signal.SIGINT), (456789, signal.SIGTERM), (456789, signal.SIGKILL)]


def test_negative_static_stamp_rejected(profile):
    with pytest.raises(ValueError):
        r.select_tf('/tf_static', [transform('base_link', 'laser', -1)],
                    profile, {('base_link', 'laser')}, {})


def test_conflicting_optical_branch_not_silently_mixed(profile):
    profile['frames']['optical'] = 'optical'
    profile['topics']['rgb'] = '/camera/rgb'
    with pytest.raises(ValueError):
        r.static_edges_for_profile([('base_link', 'laser')], profile)
    assert r.static_edges_for_profile([('base_link', 'laser'), ('base_link', 'camera'),
                                       ('camera', 'optical')], profile) == {
        ('base_link', 'laser'), ('base_link', 'camera'), ('camera', 'optical')}


@pytest.mark.parametrize('w', [0., 2., float('nan'), float('inf')])
def test_nonunit_quaternion_rejected_without_normalizing(w):
    item = transform('odom', 'base_link')
    item.transform.rotation.w = w
    with pytest.raises(ValueError, match='quaternion'):
        r.transform_values(item)
    assert item.transform.rotation.w == w or (w != w)


def test_sensor_publishers_unique_but_tf_multiple_allowed(profile):
    counts = {'/lidar/scan': 1, '/wheel/odom': 1, '/tf': 4, '/tf_static': 3}
    assert not r.sensor_publisher_issues(counts, profile)
    assert r.sensor_publisher_issues({**counts, '/lidar/scan': 2}, profile)
    assert r.sensor_publisher_issues({**counts, '/wheel/odom': 0}, profile)
    profile['topics']['rgb'] = '/camera/rgb'
    assert not r.sensor_publisher_issues(counts, profile)  # 선택 카메라는 없을 수 있음.
    assert r.sensor_publisher_issues({**counts, '/camera/rgb': 2}, profile)


def test_semantic_requires_each_camera_and_detector_publisher(profile):
    profile['semantic']['enabled'] = True
    profile['topics'].update(rgb='/rgb', depth='/depth', camera_info='/info', detections='/det')
    counts = {topic: 1 for topic in r.topic_types(profile)}
    assert not r.sensor_publisher_issues(counts, profile)
    for topic in ('/rgb', '/depth', '/info', '/det'):
        assert r.sensor_publisher_issues({**counts, topic: 0}, profile)
        assert r.sensor_publisher_issues({**counts, topic: 2}, profile)


def test_bundle_commit_is_separate_and_manifest_hash_tracks_bytes(tmp_path):
    assert r.bundle_provenance(tmp_path) == {}
    path = tmp_path / 'bundle_manifest.json'
    path.write_text('{"git_commit": "' + 'a' * 40 + '"}')
    provenance = r.bundle_provenance(tmp_path)
    assert provenance['bundle_commit'] == 'a' * 40
    assert provenance['bundle_manifest_sha256'] == r.sha256(path)
    assert 'source_commit' not in provenance
    path.write_text('{"git_commit": null}')
    with pytest.raises(ValueError):
        r.bundle_provenance(tmp_path)


def test_replay_startup_allows_advertised_silent_tf_and_slam_output(profile):
    assert not r.replay_tf_issues('/tf', [], profile)
    assert not r.replay_tf_issues('/tf', [transform('map', 'odom')], profile)
    assert r.replay_tf_issues('/tf', [transform('odom', 'base_link')], profile)
    assert r.replay_tf_issues('/tf_static', [transform('base_link', 'laser', 0)], profile)


def test_replay_running_allows_only_filtered_input_and_slam_output(profile):
    static = {('base_link', 'laser')}
    assert not r.replay_tf_issues('/tf', [transform('odom', 'base_link'), transform('map', 'odom')],
                                   profile, True, static)
    assert not r.replay_tf_issues('/tf_static', [transform('base_link', 'laser', 0)], profile, True, static)
    assert r.replay_tf_issues('/tf', [transform('other_odom', 'base_link')], profile, True, static)
    assert r.replay_tf_issues('/tf_static', [transform('map', 'odom', 0)], profile, True, static)


def test_replay_endpoint_guard_distinguishes_slam_advertisement_from_input_collision():
    baseline = {'/tf': {'slam_gid'}, '/tf_static': set()}
    slam = {'name': 'slam_toolbox', 'gid': 'slam_gid'}
    player = {'name': 'rosbag2_player', 'gid': 'player_gid'}
    driver = {'name': 'robot_driver', 'gid': 'driver_gid'}
    snapshot = {'/tf': [slam], '/tf_static': [], '/scan': [], '/clock': []}
    assert not r.replay_endpoint_issues(snapshot, baseline)
    snapshot.update({'/tf': [slam, player], '/tf_static': [player], '/scan': [player], '/clock': [player]})
    assert not r.replay_endpoint_issues(snapshot, baseline)
    assert r.replay_endpoint_issues({**snapshot, '/tf': [slam, player, driver]}, baseline)
    assert r.replay_endpoint_issues({**snapshot, '/scan': [player, driver]}, baseline)
    assert r.replay_endpoint_issues({**snapshot, '/tf_static': [driver]}, baseline)


def test_replay_baseline_only_designated_receiver_advertisement():
    slam = {'node': '/slam_toolbox', 'gid': 'slam', 'name': 'slam_toolbox'}
    driver = {'node': '/robot_driver', 'gid': 'driver', 'name': 'robot_driver'}
    assert not r.receiver_baseline_issues({'/tf': [], '/tf_static': []}, '/slam_toolbox')
    assert not r.receiver_baseline_issues({'/tf': [slam], '/tf_static': []}, '/slam_toolbox')
    assert r.receiver_baseline_issues({'/tf': [driver], '/tf_static': []}, '/slam_toolbox')
    assert r.receiver_baseline_issues({'/tf': [slam, {**slam, 'gid': 'duplicate'}]}, '/slam_toolbox')
    assert r.receiver_baseline_issues({'/tf': [slam], '/tf_static': [driver]}, '/slam_toolbox')
    namespaced = {**slam, 'node': '/robot/slam_toolbox'}
    assert not r.receiver_baseline_issues({'/tf': [namespaced]}, '/robot/slam_toolbox')
    assert r.receiver_baseline_issues({'/tf': [namespaced]}, '/slam_toolbox')


@pytest.mark.parametrize('name', ['slam_toolbox', '/', '//slam_toolbox', '/bad/node/', '/bad node'])
def test_receiver_node_requires_valid_fqn(name):
    with pytest.raises(ValueError):
        r.receiver_node_name(name)


def test_same_receiver_multiple_entities_share_participant():
    receiver = {'node': '/slam_toolbox', 'name': 'slam_toolbox'}
    first = {**receiver, 'gid': '010fa25cdddab1ee00000100000018030000000000000000'}
    second = {**receiver, 'gid': '010fa25cdddab1ee00000100000028030000000000000000'}
    assert not r.receiver_baseline_issues({'/tf': [first, second]}, '/slam_toolbox')
    other = {**receiver, 'gid': '010fa25caaaaaaaa00000100000028030000000000000000'}
    assert r.receiver_baseline_issues({'/tf': [first, other]}, '/slam_toolbox')
    assert r.receiver_baseline_issues({'/tf': [first, {**second, 'node': '/driver'}]}, '/slam_toolbox')
    assert r.receiver_baseline_issues({'/tf': [first, {**second, 'gid': ''}]}, '/slam_toolbox')
