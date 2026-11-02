"""ROS를 띄우지 않고 문 검출 입력 계약을 검사한다."""

from copy import deepcopy
from types import SimpleNamespace as N
import pytest
from mapping_semantic.door_node import detection_seed


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
