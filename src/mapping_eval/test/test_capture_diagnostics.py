"""TF 변화·그래프 후보 ID는 순수 함수로 검증한다."""

import math
from types import SimpleNamespace as NS

import pytest

from mapping_eval.diagnostics import parse_graph, quaternion_yaw, stamp_ns, transform_record
from mapping_eval.diagnostics_node import atomic_json


def header(sec=1):
    return NS(stamp=NS(sec=sec, nanosec=0), frame_id='map')


def point(x=0, y=0, z=0):
    return NS(x=x, y=y, z=z)


def pose(x=0, y=0):
    return NS(position=point(x, y), orientation=NS(x=0, y=0, z=0, w=1))


def tf(x=0, yaw=0, sec=1):
    return NS(header=header(sec), child_frame_id='odom',
              transform=NS(translation=point(x),
                           rotation=NS(x=0, y=0, z=math.sin(yaw/2), w=math.cos(yaw/2))))


def node(identifier, x, y=0):
    return NS(header=header(), ns='slam_toolbox', id=identifier, type=2, action=0, pose=pose(x,y))


def edge(points):
    return NS(header=header(), ns='slam_toolbox_edges', id=0, type=5, action=0,
              points=[point(*p) for p in points], pose=pose())


def test_first_transform_is_not_jump_and_keeps_source_stamp():
    record = transform_record(tf(sec=2), transform_timeout=.2)
    assert record['header_stamp_ns'] == 2_000_000_000
    assert record['source_scan_stamp_estimate_ns'] == 1_800_000_000
    assert record['translation_delta_m'] is None and not record['jump']


def test_unchanged_and_quaternion_sign_flip_are_not_new_records():
    message = tf()
    previous = transform_record(message)
    message.header.stamp.sec = 2
    message.transform.rotation.w = -1
    assert transform_record(message, previous) is None


def test_normalized_yaw_wrap_and_translation_jump():
    old = transform_record(tf(yaw=math.pi-.01))
    new = transform_record(tf(yaw=-math.pi+.01), old)
    assert new['yaw_delta_rad'] == pytest.approx(.02)
    assert not new['jump']
    assert transform_record(tf(x=.51), old)['jump']
    assert transform_record(tf(yaw=.21), transform_record(tf()))['jump']
    assert quaternion_yaw([0,0,2,2]) == pytest.approx(math.pi/2)


@pytest.mark.parametrize('values', [[0,0,0,0], [0,0,math.nan,1], [0,0,1]])
def test_bad_quaternion(values):
    with pytest.raises(ValueError): quaternion_yaw(values)


def test_invalid_tf_and_negative_timeout():
    message = tf()
    message.transform.translation.x = math.inf
    with pytest.raises(ValueError): transform_record(message)
    with pytest.raises(ValueError): transform_record(tf(), transform_timeout=-1)


def test_zero_stamp_allowed_before_clock_but_invalid_nanoseconds_rejected():
    assert stamp_ns(NS(sec=0,nanosec=0)) == 0
    with pytest.raises(ValueError): stamp_ns(NS(sec=1,nanosec=1_000_000_000))


def test_graph_retains_geometry_and_unambiguous_candidates():
    result = parse_graph(NS(markers=[node(10,0), node(20,1), edge([(0,0),(1,0)])]))
    assert result['node_count'] == 2 and result['edge_count'] == 1
    assert result['edges'][0]['endpoint_candidate_ids'] == [[10],[20]]
    assert result['edges'][0]['points'] == [[0,0,0],[1,0,0]]
    assert 'yaw' not in result['nodes'][0]


def test_graph_ambiguous_and_missing_endpoints_are_not_invented():
    result = parse_graph(NS(markers=[node(1,0),node(2,0),edge([(0,0),(5,0)])]))
    assert result['edges'][0]['endpoint_candidate_ids'] == [[1,2],None]


def test_graph_odd_points_and_nonidentity_pose_retained_without_false_id():
    marker = edge([(0,0),(1,0),(2,0)])
    marker.pose.position.x = 1
    result = parse_graph(NS(markers=[node(1,0),marker]))
    assert set(result['warnings']) == {'odd_edge_point_count','nonidentity_edge_marker_pose'}
    assert result['edges'][-1]['points'] == [[2,0,0]]
    assert result['edges'][0]['endpoint_candidate_ids'] == [None,None]


def test_deleteall_and_empty_graph():
    marker = NS(header=header(0), ns='', id=0, type=0, action=3)
    result = parse_graph(NS(markers=[marker]))
    assert result['nodes'] == [] and result['edges'] == []
    assert result['markers'][0]['action'] == 3


def test_cross_frame_edge_never_matches_and_nonfinite_rejected():
    marker = edge([(0,0),(1,0)])
    marker.header.frame_id = 'other'
    result = parse_graph(NS(markers=[node(1,0),marker]))
    assert result['edges'][0]['endpoint_candidate_ids'] == [None,None]
    marker.points[0].x = math.nan
    with pytest.raises(ValueError): parse_graph(NS(markers=[marker]))


def test_atomic_ready_and_summary_document(tmp_path):
    import json
    path = tmp_path/'ready.json'
    atomic_json(path, {'ready':True})
    assert json.loads(path.read_text()) == {'ready':True}
    assert not path.with_suffix('.json.tmp').exists()
