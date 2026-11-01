"""수동 정답·경로가 저장소 world의 벽과 일치하는지 확인한다."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

WORLDS = ('indoor_v1', 'indoor_v2')


def inputs(world):
    source = Path(__file__).resolve().parents[2]
    world = source / f'mapping_gazebo/worlds/{world}.sdf'
    boxes = []
    for collision in ET.parse(world).findall('.//model[@name="walls"]/link/collision'):
        pose = list(map(float, collision.findtext('pose').split()))
        size = list(map(float, collision.findtext('geometry/box/size').split()))
        assert pose[3:] == [0.0, 0.0, 0.0]
        center, half = np.array(pose[:2]), np.array(size[:2]) / 2
        boxes.append((center - half, center + half))
    config = source / 'mapping_eval/config'
    return boxes, config


@pytest.mark.parametrize('world', WORLDS)
def test_truth_is_exposed_collision_boundary(world):
    boxes, config = inputs(world)
    truth = yaml.safe_load((config / f'{world}_truth.yaml').read_text())
    assert truth['frame'] == 'world'
    for segment in truth['segments']:
        a, b = np.array(segment[:2]), np.array(segment[2:])
        for ratio in np.linspace(0.001, 0.999, 30):
            point = a + ratio * (b - a)
            on_boundary = False
            for low, high in boxes:
                assert not np.all((point > low + 1e-7) & (point < high - 1e-7))
                if np.all(point >= low - 1e-7) and np.all(point <= high + 1e-7):
                    on_boundary |= bool(np.any(np.isclose(point, low)) or
                                        np.any(np.isclose(point, high)))
            assert on_boundary, point


@pytest.mark.parametrize('world', WORLDS)
def test_route_has_body_clearance_and_returns_to_spawn(world):
    boxes, config = inputs(world)
    route = yaml.safe_load((config / f'{world}_route.yaml').read_text())
    points = np.array(route['waypoints'])
    assert route['frame'] == 'odom'
    np.testing.assert_allclose(points[0], [1.0, 1.0])
    np.testing.assert_allclose(points[-1], points[0])
    for a, b in zip(points, points[1:]):
        for ratio in np.linspace(0, 1, 100):
            point = a + ratio * (b - a)
            for low, high in boxes:
                distance = np.linalg.norm(np.maximum(np.maximum(low - point, point - high), 0))
                assert distance >= 0.30, (point, distance)
