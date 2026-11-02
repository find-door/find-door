"""문턱 교점 기하와 잘못된 평면 입력을 검증한다."""

import numpy as np
import pytest
from mapping_semantic.door_position import threshold_point


def test_front_wall():
    k = [[500, 0, 320], [0, 500, 240], [0, 0, 1]]
    assert threshold_point(k, [195, 0, 445, 270], [0, 0, 1], -2) == pytest.approx([0, .12, 2])


def test_slanted_wall_average_of_endpoints():
    k = np.array([[500, 0, 320], [0, 500, 240], [0, 0, 1.]])
    n = np.array([.5, 0, 1.]); n /= np.linalg.norm(n)
    left, right = [100, 280, 1], [500, 280, 1]
    expected = []
    for pixel in [left, right]:
        ray = np.linalg.inv(k) @ pixel
        expected.append(ray * (2 / (ray[2] + .5 * ray[0])))
    assert threshold_point(k, [100, 0, 500, 280], n, -2/np.sqrt(1.25)) == pytest.approx(np.mean(expected, axis=0))


@pytest.mark.parametrize('normal,offset', [([0, 0, 1], 2), ([0, 0, 1], -11), ([0, 0, 0], -2)])
def test_invalid_or_behind(normal, offset):
    with pytest.raises(ValueError):
        threshold_point(np.eye(3), [0, 0, 1, 1], normal, offset)
