"""주행 좌표 변환, 초기 정합과 실행 가드를 ROS 없이 검증한다."""

import argparse
import math
from types import SimpleNamespace as NS

import pytest

from mapping_eval.drive_route import (
    body_command, bounded_speed, freshness_problem, inverse_se2,
    planar_transform, quaternion_yaw, require_simulation, transform_point, validate_route,
)


def test_world_forward_becomes_body_right_at_quarter_turn():
    command, reached = body_command((0, 0), math.pi / 2, (1, 0), 0.35)
    assert not reached
    assert command == pytest.approx((0, -0.35, -0.3))


def test_diagonal_speed_is_bounded_and_goal_stops():
    command, _ = body_command((0, 0), 0, (2, 2), 0.35)
    assert math.hypot(*command[:2]) == pytest.approx(0.35)
    assert body_command((0, 0), 1, (0.04, 0.04), 0.35) == ((0, 0, 0), True)


def test_initial_transform_inverse_recovers_world_point():
    world_to_map = (2.0, -3.0, math.pi / 3)
    world = (1.0, 4.0)
    map_point = transform_point(world_to_map, world)
    assert transform_point(inverse_se2(world_to_map), map_point) == pytest.approx(world)
    # 초기 변환을 고정하면 이후 지도 좌표의 변형이 오차로 남는다.
    changed = (map_point[0] + 0.5, map_point[1])
    estimate = transform_point(inverse_se2(world_to_map), changed)
    assert math.dist(estimate, world) == pytest.approx(0.5)


@pytest.mark.parametrize('document', [None, {}, {'frame': 'map', 'waypoints': [[1, 1]]},
                                     {'frame': 'odom', 'waypoints': []},
                                     {'frame': 'odom', 'waypoints': [[True, 1]]},
                                     {'frame': 'odom', 'waypoints': [[float('nan'), 1]]}])
def test_invalid_routes_are_rejected(document):
    with pytest.raises(ValueError):
        validate_route(document)


def test_valid_route():
    assert validate_route({'frame': 'odom', 'waypoints': [[1, 1], [2, 1]]}) == [(1, 1), (2, 1)]


@pytest.mark.parametrize('confirmed,environment', [
    (False, {'ROS_DOMAIN_ID': '12', 'GZ_PARTITION': 'codex'}),
    (True, {}), (True, {'ROS_DOMAIN_ID': '0', 'GZ_PARTITION': 'codex'}),
    (True, {'ROS_DOMAIN_ID': '12', 'GZ_PARTITION': 'claude'}),
    (True, {'ROS_DOMAIN_ID': '11', 'GZ_PARTITION': 'codex'}),
    (True, {'ROS_DOMAIN_ID': '11'}),
])
def test_simulation_guard_rejects_unconfirmed_or_shared_domain(confirmed, environment):
    with pytest.raises(ValueError):
        require_simulation(confirmed, environment)


@pytest.mark.parametrize('domain,partition', [('11', 'claude'), ('12', 'codex')])
def test_simulation_guard_accepts_isolated_confirmed_run(domain, partition):
    assert require_simulation(True, {'ROS_DOMAIN_ID': domain, 'GZ_PARTITION': partition}) == {
        'ROS_DOMAIN_ID': int(domain), 'GZ_PARTITION': partition}


@pytest.mark.parametrize('value', ['0', '-1', '0.51', 'nan', 'inf'])
def test_invalid_speed(value):
    with pytest.raises(argparse.ArgumentTypeError):
        bounded_speed(value)


def test_freshness_requires_all_three_sources():
    assert freshness_problem(10, 9, 9, 9) is None
    assert '/clock' in freshness_problem(10, 7, 9, 9)
    assert '/odom' in freshness_problem(10, 9, None, 9)
    assert '/scan' in freshness_problem(10, 9, 9, 7)


def test_quaternion_yaw_normalizes_and_rejects_zero():
    assert quaternion_yaw(NS(x=0, y=0, z=2, w=2)) == pytest.approx(math.pi / 2)
    with pytest.raises(ValueError):
        quaternion_yaw(NS(x=0, y=0, z=0, w=0))


def test_invalid_tf_translation_is_rejected():
    with pytest.raises(ValueError):
        planar_transform(NS(translation=NS(x=float('nan'), y=0),
                            rotation=NS(x=0, y=0, z=0, w=1)))
