"""특정 문 색·world 없이 출입구 후보의 근거와 거부 조건을 확인한다."""

import json
import numpy as np
import pytest

from mapping_semantic.doorway import estimate_doorway


def scene(color=(40, 90, 150), frame=True, leaf=True):
    rgb = np.full((180, 240, 3), 200, dtype=np.uint8)
    depth = np.full((180, 240), 2., dtype=np.float32)
    if leaf:
        rgb[28:150, 68:152] = color
        depth[28:150, 68:152] = 2.03
    if frame:
        rgb[20:150, 60:68] = color
        rgb[20:150, 152:160] = color
        rgb[20:28, 60:160] = color
        depth[20:150, 60:68] = 1.94
        depth[20:150, 152:160] = 1.94
        depth[20:28, 60:160] = 1.94
    return rgb, depth, np.array([[200., 0, 120], [0, 200., 90], [0, 0, 1]])


@pytest.mark.parametrize('color', [(40, 90, 150), (180, 30, 70), (20, 20, 20)])
def test_frame_expands_leaf_seed_without_color_constant(color):
    rgb, depth, k = scene(color)
    result = estimate_doorway(rgb, depth, k, [75, 40, 130, 140])
    assert result['status'] == 'estimated'
    assert result['bbox'] == [60., 20., 160., 150.]
    json.dumps(result, allow_nan=False)


def test_narrow_open_leaf_seed_does_not_bound_doorway():
    rgb, depth, k = scene(leaf=False)
    rgb[28:150, 140:152] = [40, 90, 150]
    depth[28:150, 140:152] = 3.
    result = estimate_doorway(rgb, depth, k, [141, 40, 151, 140])
    assert result['bbox'] == [60., 20., 160., 150.]


@pytest.mark.parametrize('frame,leaf', [(False, False), (False, True)])
def test_plain_wall_and_leaf_without_frame_are_unavailable(frame, leaf):
    rgb, depth, k = scene(frame=frame, leaf=leaf)
    result = estimate_doorway(rgb, depth, k, [75, 40, 130, 140])
    assert result['status'] == 'unavailable'
    assert result['bbox'] is None


def test_missing_jamb_and_depth_are_unavailable():
    rgb, depth, k = scene()
    depth[:, 152:160] = np.nan
    assert estimate_doorway(rgb, depth, k, [75, 40, 130, 140])['status'] == 'unavailable'
    depth[:] = np.nan
    assert estimate_doorway(rgb, depth, k, [75, 40, 130, 140])['status'] == 'unavailable'


def test_multiple_door_frames_are_not_combined():
    rgb, depth, k = scene()
    rgb[20:150, 185:193] = [40, 90, 150]
    depth[20:150, 185:193] = 1.94
    assert estimate_doorway(rgb, depth, k, [75, 40, 130, 140])['status'] == 'unavailable'


def test_visible_header_required():
    rgb, depth, k = scene(leaf=False)
    rgb[20:28, 68:152] = 200
    depth[20:28, 68:152] = 2.
    result = estimate_doorway(rgb, depth, k, [75, 40, 130, 140])
    assert result['reason'] == 'visible_header_missing'


@pytest.mark.parametrize('box', [[0, 0, 0, 5], [-1, 0, 50, 50], [0, 0, 241, 50], [0, np.nan, 50, 50]])
def test_invalid_bbox_is_rejected(box):
    rgb, depth, k = scene()
    with pytest.raises(ValueError):
        estimate_doorway(rgb, depth, k, box)


def test_invalid_rgb_depth_k_are_rejected():
    rgb, depth, k = scene()
    for image, metric, calibration in [(rgb.astype(float), depth, k), (rgb, depth[:, :-1], k),
                                       (rgb, depth, np.eye(2)), (rgb, depth, k*float('nan'))]:
        with pytest.raises(ValueError):
            estimate_doorway(image, metric, calibration, [75, 40, 130, 140])
