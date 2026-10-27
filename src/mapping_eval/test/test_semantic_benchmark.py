"""합성 객체 평가가 ID·3D 위치·저장 데이터의 차이를 감추지 않는지 확인한다."""

from copy import deepcopy
from types import SimpleNamespace
import math

import pytest

from mapping_eval.run_semantic_benchmark import (
    inverse_transform, saved_records_match, score_objects, transform_xyz, validate_truth,
    sole_publisher_gid, new_instance,
)


TRUTH = {'frame': 'world', 'map_id': 'semantic_v1', 'tolerance_m': 0.1,
         'objects': {'red_fixture': [2.95, 0.55, 0.35],
                     'blue_fixture': [2.95, 1.45, 0.35]}}
ANCHOR = {'translation': [0, 0, 0], 'quaternion': [0, 0, 0, 1]}


def test_graph_gid_requires_exactly_one_nonzero_endpoint():
    old = SimpleNamespace(endpoint_gid=[1] * 24)
    new = SimpleNamespace(endpoint_gid=[2] * 24)
    assert sole_publisher_gid([]) is None
    assert sole_publisher_gid([old, new]) is None
    assert sole_publisher_gid([SimpleNamespace(endpoint_gid=[0] * 24)]) is None
    assert sole_publisher_gid([new]) == bytes([2] * 24).hex()


def test_restart_rejects_queued_old_status_instance():
    assert not new_instance({}, 'old')
    assert not new_instance({'instance_id': ''}, 'old')
    assert not new_instance({'instance_id': 'old'}, 'old')
    assert new_instance({'instance_id': 'new'}, 'old')


def records():
    return [{'id': label + '_000001', 'class_id': label, 'position': point[:],
             'observations': 10} for label, point in TRUTH['objects'].items()]


def test_exact_records_pass_and_order_does_not_matter():
    expected = score_objects(records(), TRUTH, ANCHOR)
    actual = score_objects(list(reversed(records())), TRUTH, ANCHOR, expected['ids'])
    assert actual['position_pass']
    assert actual['max_error_m'] == 0


def test_vertical_error_is_measured_not_discarded():
    observed = records()
    observed[0]['position'][2] += 0.15
    score = score_objects(observed, TRUTH, ANCHOR)
    assert score['max_error_m'] == pytest.approx(0.15)
    assert not score['position_pass']


def test_full_3d_rotation_and_inverse_preserve_position():
    # x축 90도 회전: 평면 yaw만 처리하면 이 검사를 통과할 수 없다.
    quaternion = [math.sin(math.pi / 4), 0, 0, math.cos(math.pi / 4)]
    translation = [1, -2, 0.5]
    point = [2, 3, 4]
    transformed = transform_xyz(point, translation, quaternion)
    inverse = inverse_transform(translation, quaternion)
    restored = transform_xyz(transformed, inverse['translation'], inverse['quaternion'])
    assert restored == pytest.approx(point)


def test_fixed_transform_is_applied_to_both_objects():
    observed = records()
    for item in observed:
        item['position'][0] -= 1.0
    anchor = {'translation': [1, 0, 0], 'quaternion': [0, 0, 0, 1]}
    assert score_objects(observed, TRUTH, anchor)['position_pass']
    assert not score_objects(observed, TRUTH, ANCHOR)['position_pass']


def test_id_change_fails_even_when_position_is_correct():
    before = score_objects(records(), TRUTH, ANCHOR)
    observed = records()
    observed[0]['id'] += '_changed'
    with pytest.raises(ValueError, match='ID'):
        score_objects(observed, TRUTH, ANCHOR, before['ids'])


@pytest.mark.parametrize('change', ['missing', 'extra', 'duplicate_class', 'duplicate_id', 'nan'])
def test_invalid_object_set_rejected(change):
    observed = records()
    if change == 'missing':
        observed.pop()
    elif change == 'extra':
        observed.append(deepcopy(observed[0]))
    elif change == 'duplicate_class':
        observed[1]['class_id'] = observed[0]['class_id']
    elif change == 'duplicate_id':
        observed[1]['id'] = observed[0]['id']
    elif change == 'nan':
        observed[0]['position'][0] = float('nan')
    with pytest.raises(ValueError):
        score_objects(observed, TRUTH, ANCHOR)


def test_saved_publication_match_detects_later_observation_average():
    saved = records()
    assert saved_records_match(list(reversed(saved)), saved)
    updated = records()
    updated[0]['position'][0] += 0.001
    assert not saved_records_match(updated, saved)
    assert score_objects(updated, TRUTH, ANCHOR)['position_pass']


def test_saved_match_rejects_duplicate_ids_and_invalid_positions():
    observed = records()
    observed[1]['id'] = observed[0]['id']
    assert not saved_records_match(observed, records())
    observed = records()
    observed[0]['position'] = [0, 1, float('nan')]
    assert not saved_records_match(observed, records())


@pytest.mark.parametrize('quaternion', [[0, 0, 0, 0], [0, 0, float('inf'), 1], [0, 1]])
def test_invalid_quaternion_rejected(quaternion):
    with pytest.raises(ValueError):
        inverse_transform([0, 0, 0], quaternion)


@pytest.mark.parametrize('field,value', [('frame', 'map'), ('map_id', ''),
                                        ('tolerance_m', 0.2), ('tolerance_m', float('nan'))])
def test_truth_cannot_relax_frame_or_tolerance(field, value):
    truth = deepcopy(TRUTH)
    truth[field] = value
    with pytest.raises(ValueError):
        validate_truth(truth)
