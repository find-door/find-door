"""정상 입력과 오염된 인터페이스가 구분되는지 확인한다."""

import argparse
from types import SimpleNamespace as NS

import pytest

from mapping_eval.check_interfaces import positive_timeout
from mapping_eval.validation import (
    validate_map, validate_odom, validate_odom_age, validate_scan,
)


def header(frame):
    return NS(frame_id=frame, stamp=NS(sec=1, nanosec=100))


def scan():
    return NS(header=header('lidar_sim_frame'), ranges=[1.0, float('inf'), 2.0],
              range_min=0.15, range_max=12.0, angle_increment=0.01)


def occupancy_map():
    return NS(header=header('map'), info=NS(resolution=0.05, width=2, height=2),
              data=[-1, 0, 50, 100])


def test_scan_accepts_valid_ranges_with_out_of_range_beams():
    assert validate_scan(scan()) == []


def test_scan_wrong_frame_is_rejected():
    message = scan()
    message.header.frame_id = 'base_link'
    assert any('frame_id' in error for error in validate_scan(message))


@pytest.mark.parametrize('ranges', [[], [float('nan'), float('inf')], [0.0, 20.0]])
def test_scan_without_valid_measurement_is_rejected(ranges):
    message = scan()
    message.ranges = ranges
    assert validate_scan(message)


def test_scan_zero_timestamp_is_rejected():
    message = scan()
    message.header.stamp = NS(sec=0, nanosec=0)
    assert any('타임스탬프' in error for error in validate_scan(message))


@pytest.mark.parametrize('minimum,maximum', [(12.0, 0.15), (float('nan'), 12.0)])
def test_scan_invalid_range_bounds_are_rejected(minimum, maximum):
    message = scan()
    message.range_min, message.range_max = minimum, maximum
    assert validate_scan(message)


def test_odom_requires_expected_parent_and_child():
    message = NS(header=header('odom'), child_frame_id='base_footprint')
    assert validate_odom(message) == []
    message.child_frame_id = 'base_link'
    assert validate_odom(message)
    message.child_frame_id = 'base_footprint'
    message.header.frame_id = 'world'
    assert validate_odom(message)


@pytest.mark.parametrize('age_ns', [0, 50_000_000, 999_999_999])
def test_recent_odom_timestamp_is_accepted(age_ns):
    message = NS(header=header('odom'), child_frame_id='base_footprint')
    assert validate_odom_age(message, 1_000_000_100 + age_ns) == []


@pytest.mark.parametrize('age_ns', [-1, -50_000_000, 1_000_000_000, 5_000_000_000])
def test_future_or_repeated_stale_odom_timestamp_is_rejected(age_ns):
    message = NS(header=header('odom'), child_frame_id='base_footprint')
    assert validate_odom_age(message, 1_000_000_100 + age_ns)


def test_odom_timestamp_needs_clock():
    message = NS(header=header('odom'), child_frame_id='base_footprint')
    assert validate_odom_age(message, None)


def test_map_accepts_observed_cells_and_unknown_cells():
    assert validate_map(occupancy_map()) == []


def test_empty_map_is_rejected():
    message = occupancy_map()
    message.info.width = 0
    message.data = []
    assert validate_map(message)


@pytest.mark.parametrize('data', [[0], [-1, -1, -1, -1], [0, 0, -2, 0],
                                 [0, 0, 101, 0]])
def test_invalid_map_data_is_rejected(data):
    message = occupancy_map()
    message.data = data
    assert validate_map(message)


@pytest.mark.parametrize('resolution', [0.0, -0.05, float('inf'), float('nan')])
def test_invalid_map_resolution_is_rejected(resolution):
    message = occupancy_map()
    message.info.resolution = resolution
    assert validate_map(message)


def test_map_wrong_frame_is_rejected():
    message = occupancy_map()
    message.header.frame_id = 'odom'
    assert validate_map(message)


@pytest.mark.parametrize('value', ['0', '-1', 'nan', 'inf', 'bad'])
def test_timeout_must_be_finite_and_positive(value):
    with pytest.raises(argparse.ArgumentTypeError):
        positive_timeout(value)


def test_positive_timeout():
    assert positive_timeout('2.5') == 2.5
