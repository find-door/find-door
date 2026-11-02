"""주행 검사에서 NaN·잘못된 프레임·오래된 odom을 거부한다."""

from types import SimpleNamespace as N
import pytest
from live_check import motion_odom


def odom():
    return N(header=N(frame_id='odom', stamp=N(sec=1, nanosec=0)), child_frame_id='base_footprint',
             pose=N(pose=N(position=N(x=-2., y=0., z=0.), orientation=N(x=0., y=0., z=0., w=1.))))


def test_valid():
    assert motion_odom(odom(), 1_100_000_000) == (-2., 0.)


@pytest.mark.parametrize('kind', ['nan', 'frame', 'child', 'stale', 'future', 'quaternion'])
def test_rejects(kind):
    message = odom()
    now = 1_100_000_000
    if kind == 'nan': message.pose.pose.position.x = float('nan')
    elif kind == 'frame': message.header.frame_id = 'map'
    elif kind == 'child': message.child_frame_id = 'other_robot'
    elif kind == 'stale': now = 2_000_000_000
    elif kind == 'future': now = 500_000_000
    else: message.pose.pose.orientation.w = 0.
    with pytest.raises(ValueError):
        motion_odom(message, now)
