"""문짝·출입구 구분, 미검수 라벨, 분할 누출과 능력 과장을 검증한다."""

from copy import deepcopy
import pytest
from mapping_semantic.door_dataset import validate_dataset


def frame(identifier='first', checksum='a'*64):
    return {'id': identifier, 'environment': 'gazebo', 'group_id': 'room-1', 'split': 'development',
            'rgb': {'path': '/tmp/frame.png', 'sha256': checksum, 'width': 640, 'height': 480},
            'capabilities': {'depth': True, 'camera_info': True, 'tf': True},
            'review_status': 'draft', 'doors': [{'id': 'door_1'}]}


def dataset(*frames):
    return {'schema_version': 1, 'dataset_id': 'doors', 'frames': list(frames or [frame()])}


def reviewed():
    f = frame()
    f['review_status'] = 'reviewed'
    f['doors'][0].update(state='unknown', leaf_bbox=None, doorway_bbox=[1, 2, 300, 480],
                         leaf_visibility='occluded', doorway_visibility='visible')
    return f


def test_draft_defaults_are_only_added_to_deep_copy():
    source = dataset()
    source['frames'][0]['legacy_labels'] = {'bbox': [0, 0, 640, 480], 'state': 'closed'}
    original = deepcopy(source)
    checked = validate_dataset(source)
    assert source == original
    door = checked['frames'][0]['doors'][0]
    assert door['state'] == 'unknown' and door['leaf_bbox'] is None and door['doorway_bbox'] is None
    assert checked['frames'][0]['legacy_labels'] == source['frames'][0]['legacy_labels']
    checked['frames'][0]['legacy_labels']['bbox'][0] = 10
    assert source == original


def test_reviewed_unknown_state_and_separate_boxes():
    checked = validate_dataset(dataset(reviewed()))
    assert checked['frames'][0]['doors'][0]['state'] == 'unknown'
    assert checked['frames'][0]['doors'][0]['leaf_bbox'] is None


@pytest.mark.parametrize('split', ['train', 'validation', 'test', 'unassigned'])
def test_same_group_cannot_cross_splits(split):
    second = frame('second', 'b'*64)
    second['split'] = split
    with pytest.raises(ValueError, match='분할 누출'):
        validate_dataset(dataset(frame(), second))


def test_same_group_can_have_distinct_frames_in_same_split():
    assert len(validate_dataset(dataset(frame(), frame('second', 'b'*64)))['frames']) == 2


def test_duplicate_rgb_rejected_even_with_different_groups():
    second = frame('second')
    second['group_id'], second['split'] = 'other-room', 'test'
    with pytest.raises(ValueError, match='중복 RGB'):
        validate_dataset(dataset(frame(), second))


def test_gazebo_and_real_cannot_share_group():
    second = frame('second', 'b'*64)
    second['environment'] = 'real_rgb'
    second['capabilities'] = dict(depth=False, camera_info=False, tf=False)
    with pytest.raises(ValueError, match='혼합'):
        validate_dataset(dataset(frame(), second))


def test_real_rgbd_requires_depth_but_metadata_may_be_missing():
    f = frame(); f['environment'] = 'real_rgbd'; f['capabilities']['tf'] = False
    assert validate_dataset(dataset(f))
    f['capabilities']['camera_info'] = False
    assert validate_dataset(dataset(f))
    f['capabilities']['depth'] = False
    with pytest.raises(ValueError, match='depth'):
        validate_dataset(dataset(f))


@pytest.mark.parametrize('field', ['depth', 'camera_info', 'tf'])
def test_real_rgb_cannot_claim_rgbd_capability(field):
    f = frame(); f['environment'] = 'real_rgb'
    f['capabilities'] = dict(depth=False, camera_info=False, tf=False)
    f['capabilities'][field] = True
    if field == 'depth':
        with pytest.raises(ValueError, match='real_rgb'):
            validate_dataset(dataset(f))
    else:
        assert validate_dataset(dataset(f))


@pytest.mark.parametrize('box', [None, [0, 0, 0, 10], [-1, 0, 10, 10], [0, 0, 641, 10],
                                  [False, 0, 10, 10], [0, 0, float('nan'), 10], [0, 0, 10, float('inf')],
                                  [0, 0, 10**1000, 10]])
def test_visible_box_must_be_finite_and_inside_image(box):
    f = reviewed(); f['doors'][0]['doorway_bbox'] = box
    with pytest.raises(ValueError, match='xyxy'):
        validate_dataset(dataset(f))


def test_occluded_box_cannot_retain_guessed_extent():
    f = reviewed(); f['doors'][0]['leaf_bbox'] = [0, 0, 50, 50]
    with pytest.raises(ValueError, match='null'):
        validate_dataset(dataset(f))


@pytest.mark.parametrize('field', ['state', 'leaf_bbox', 'doorway_bbox', 'leaf_visibility', 'doorway_visibility'])
def test_reviewed_requires_explicit_label_fields(field):
    f = reviewed(); del f['doors'][0][field]
    with pytest.raises(ValueError, match='누락'):
        validate_dataset(dataset(f))


@pytest.mark.parametrize('change', [
    lambda d: d.update(schema_version=True),
    lambda d: d.update(dataset_id=' '),
    lambda d: d.update(frames={}),
    lambda d: d['frames'][0]['rgb'].update(width=True),
    lambda d: d['frames'][0]['rgb'].update(sha256='xyz'),
    lambda d: d['frames'][0]['capabilities'].update(depth=1),
    lambda d: d['frames'][0].update(review_status='pending'),
    lambda d: d['frames'][0].update(environment=[]),
    lambda d: d['frames'][0].update(split=[]),
    lambda d: d['frames'][0]['doors'][0].update(state=None),
    lambda d: d['frames'][0].pop('group_id'),
])
def test_invalid_contract_rejected(change):
    source = dataset(); change(source)
    with pytest.raises(ValueError):
        validate_dataset(source)


def test_duplicate_frame_and_door_ids_rejected():
    with pytest.raises(ValueError, match='frame ID'):
        validate_dataset(dataset(frame(), frame('first', 'b'*64)))
    f = frame(); f['doors'].append({'id': 'door_1'})
    with pytest.raises(ValueError, match='문 ID'):
        validate_dataset(dataset(f))
