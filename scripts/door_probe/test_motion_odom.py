"""주행 검사에서 NaN·잘못된 프레임·오래된 odom을 거부한다."""

from types import SimpleNamespace as N
import pytest
from live_check import angle_delta, motion_odom, motion_yaw, rotation_observation
import math


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


@pytest.mark.parametrize('yaw', [-math.pi, -2.9, -.25, 0., .25, 2.9, math.pi])
def test_yaw(yaw):
    message = odom()
    message.pose.pose.orientation.z = math.sin(yaw/2)
    message.pose.pose.orientation.w = math.cos(yaw/2)
    assert angle_delta(motion_yaw(message, 1_100_000_000), yaw) == pytest.approx(0.)


def test_yaw_wrap():
    assert angle_delta(-math.pi+.1, math.pi-.1) == pytest.approx(.2)
    assert angle_delta(math.pi-.1, -math.pi+.1) == pytest.approx(-.2)


def test_yaw_normalizes_quaternion():
    message = odom()
    message.pose.pose.orientation.z = math.sin(.25/2)*1.005
    message.pose.pose.orientation.w = math.cos(.25/2)*1.005
    assert motion_yaw(message, 1_100_000_000) == pytest.approx(.25)


def test_yaw_rejects_stale():
    with pytest.raises(ValueError):
        motion_yaw(odom(), 2_000_000_000)


@pytest.mark.parametrize('state', ['closed', 'unknown'])
def test_rotation_allows_expected_or_unknown(state):
    initial = dict(id='door_1', position=[0., 0., 0.], state='closed')
    observed = dict(initial, state=state)
    assert rotation_observation(dict(objects=[observed]), initial, 'closed') == observed


@pytest.mark.parametrize('kind', ['wrong_state', 'id', 'anchor', 'missing', 'duplicate'])
def test_rotation_rejects_wrong_known_and_identity(kind):
    initial = dict(id='door_1', position=[0., 0., 0.], state='closed')
    observed = dict(initial)
    snapshot = dict(objects=[observed])
    if kind == 'wrong_state': observed['state'] = 'open'
    elif kind == 'id': observed['id'] = 'door_2'
    elif kind == 'anchor': observed['position'] = [1., 0., 0.]
    elif kind == 'missing': snapshot['objects'] = []
    else: snapshot['objects'].append(dict(observed))
    with pytest.raises(RuntimeError):
        rotation_observation(snapshot, initial, 'closed')
