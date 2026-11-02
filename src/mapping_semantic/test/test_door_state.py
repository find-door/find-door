"""주변 벽과 문 깊이의 실제 투영 기하로 상태 후보를 검사한다."""

import json
import numpy as np
import pytest

from mapping_semantic.door_state import estimate_state


K = np.array([[150., 0, 80], [0, 150., 20], [0, 0, 1.]])
BOX = [50, 10, 110, 100]


def plane_depth(distance=2., tilt=0.):
    rows, cols = np.indices((120, 160))
    return (distance / (np.cos(tilt)+np.sin(tilt)*(cols-80)/150)).astype(np.float32)


@pytest.mark.parametrize('tilt', [0, .45, -.45])
def test_closed_and_open_with_slanted_wall(tilt):
    wall = plane_depth(2., tilt)
    closed = wall.copy()
    closed[10:100, 50:110] = plane_depth(2.02, tilt)[10:100, 50:110]
    result = estimate_state(closed, K, BOX)
    assert result['state'] == 'closed'
    assert result['evidence']['near_wall_fraction'] > .99
    opened = wall.copy()
    opened[10:100, 50:110] = plane_depth(3., tilt)[10:100, 50:110]
    result = estimate_state(opened, K, BOX)
    assert result['state'] == 'open'
    assert result['evidence']['behind_wall_fraction'] > .99
    json.dumps(result, allow_nan=False)


def test_closed_door_with_floor_in_bottom_of_bbox():
    depth = plane_depth()
    depth[80:] = 5.
    assert estimate_state(depth, K, BOX)['state'] == 'closed'


def test_partial_opening_and_contaminated_wall_samples():
    depth = plane_depth()
    depth[10:100, 80:110] = 3.
    depth[10:20, :50] = 5.  # 일부 벽 strip 이상치
    result = estimate_state(depth, K, BOX)
    assert result['state'] == 'open'
    assert result['evidence']['wall_plane_inlier_fraction'] >= .75


@pytest.mark.parametrize('region,reason', [('wall', 'insufficient_wall_depth'),
                                         ('interior', 'insufficient_interior_depth')])
def test_missing_depth_is_unknown(region, reason):
    depth = plane_depth()
    if region == 'wall':
        depth[:, :50] = np.nan
        depth[:, 110:] = np.nan
        depth[:10] = np.nan
    else:
        depth[10:100, 50:110] = 0
    result = estimate_state(depth, K, BOX)
    assert result['state'] == 'unknown'
    assert result['reason'] == reason
    json.dumps(result, allow_nan=False)


def test_ambiguous_recess_and_foreground_are_unknown():
    depth = plane_depth()
    depth[10:100, 50:110] = 2.12
    assert estimate_state(depth, K, BOX)['reason'] == 'ambiguous_depth'
    depth[10:100, 50:110] = 1.
    result = estimate_state(depth, K, BOX)
    assert result['state'] == 'unknown'
    assert result['reason'] == 'foreground_occlusion'


def test_floor_cannot_be_used_as_wall_plane():
    rows, _ = np.indices((120, 160))
    depth = (30. / (rows+1)).astype(np.float32)
    # 위아래 구도가 모두 floor 평면인 카메라: K의 cy=-1로 정확한 y 상수 평면 생성.
    k = K.copy()
    k[1, 2] = -1
    result = estimate_state(depth, k, BOX)
    assert result['state'] == 'unknown'
    assert result['reason'] == 'wall_plane_unreliable'


@pytest.mark.parametrize('box', [[-1, 10, 110, 100], [50, 10, 161, 100],
                               [50, 10, 50, 100], [50, np.nan, 110, 100]])
def test_invalid_roi_rejected(box):
    with pytest.raises(ValueError):
        estimate_state(plane_depth(), K, box)


@pytest.mark.parametrize('config', [{'closed_tolerance_m': .3}, {'minimum_samples': 2},
                                  {'unknown_key': 1}, {'minimum_depth_m': 12},
                                  {'interior_inset_fraction': .6}, {'maximum_plane_points': 10},
                                  {'minimum_open_fraction': .1}, {'plane_tolerance_m': True}])
def test_invalid_and_contradictory_config_rejected(config):
    with pytest.raises(ValueError):
        estimate_state(plane_depth(), K, BOX, config)


def test_invalid_k_and_depth_rejected():
    with pytest.raises(ValueError):
        estimate_state(np.ones((20, 20), dtype=np.uint16), K, [1, 1, 10, 10])
    k = K.copy()
    k[0, 1] = .1
    with pytest.raises(ValueError):
        estimate_state(plane_depth(), k, BOX)


def test_fit_is_deterministic_and_config_is_not_mutated():
    depth = plane_depth(2, .3)
    config = {'ransac_iterations': 32}
    first = estimate_state(depth, K, BOX, config)
    second = estimate_state(depth, K, BOX, config)
    assert first == second
    assert config == {'ransac_iterations': 32}
