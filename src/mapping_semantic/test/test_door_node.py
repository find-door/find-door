"""ROS를 띄우지 않고 문 검출 입력 계약을 검사한다."""

from copy import deepcopy
from types import SimpleNamespace as N
import pytest
from mapping_semantic.door_node import detection_seed, observation_view, marker_appearance
from mapping_semantic.door_map import DoorMap


def sample():
    header = N(stamp=N(sec=1, nanosec=0), frame_id='optical')
    detection = N(header=deepcopy(header),
                  results=[N(hypothesis=N(class_id='door', score=.3))],
                  bbox=N(center=N(position=N(x=5., y=10.), theta=0.), size_x=20., size_y=10.))
    return header, N(header=deepcopy(header), detections=[detection])


def test_clipped_original_pixels():
    header, array = sample()
    assert detection_seed(array, header, 100, 100, .1) == [0., 5., 15., 15.]
    assert detection_seed(array, header, 100, 100, .5) is None


@pytest.mark.parametrize('mutation', ['array_frame', 'detection_stamp', 'nan_score', 'theta', 'negative_size'])
def test_invalid_contract(mutation):
    header, array = sample()
    detection = array.detections[0]
    if mutation == 'array_frame':
        array.header.frame_id = 'other'
    elif mutation == 'detection_stamp':
        detection.header.stamp.nanosec = 1
    elif mutation == 'nan_score':
        detection.results[0].hypothesis.score = float('nan')
    elif mutation == 'theta':
        detection.bbox.center.theta = .2
    else:
        detection.bbox.size_x = -1.
    with pytest.raises(ValueError):
        detection_seed(array, header, 100, 100, .1)


def test_expired_view_retains_confirmed_color_with_explicit_stale_label():
    memory=DoorMap('view',minimum_observations=1)
    record=memory.observe([0.,0.,0.],'closed',1_000_000_000)
    view=observation_view(record,4_000_000_000,2.)
    assert view['state']=='unknown' and view['last_confirmed_state']=='closed'
    assert view['observation_status']=='stale' and view['last_confirmed_age_sec']==3.
    color,alpha,text=marker_appearance(view)
    assert color==(1.,0.,0.) and alpha<1 and 'last:closed' in text and 'stale' in text
    assert record['state']=='closed' and 'observation_status' not in record


def test_recent_uncertain_observation_does_not_claim_last_state_as_current():
    memory=DoorMap('view',minimum_observations=1)
    memory.observe([0.,0.,0.],'open',1_000_000_000)
    record=memory.observe([0.,0.,0.],'unknown',2_000_000_000)
    view=observation_view(record,2_100_000_000,2.)
    assert view['state']=='unknown' and view['last_confirmed_state']=='open'
    assert view['observation_status']=='uncertain' and view['last_confirmed_age_sec']==1.1
    assert 'uncertain' in marker_appearance(view)[2]


def test_initial_unknown_and_freshness_boundary_are_explicit():
    memory=DoorMap('view',minimum_observations=1)
    record=memory.observe([0.,0.,0.],'unknown',1_000_000_000)
    view=observation_view(record,1_000_000_000,2.)
    assert view['last_confirmed_age_sec']is None and 'unconfirmed' in marker_appearance(view)[2]
    record=memory.observe([0.,0.,0.],'closed',2_000_000_000)
    assert observation_view(record,4_000_000_000,2.)['observation_status']=='confirmed'
    assert observation_view(record,4_000_000_001,2.)['observation_status']=='stale'
    assert observation_view(record,1_999_999_999,2.)['state']=='unknown'
