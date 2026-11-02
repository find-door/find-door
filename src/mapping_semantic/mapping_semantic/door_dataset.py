"""문짝·출입구 라벨과 장면 단위 분할을 검증하는 파일 I/O 없는 데이터 계약.

RGB 해시는 소문자 SHA-256이다. legacy_labels 등 추가 메타데이터는 보존하지만
그 안의 bbox를 문짝·출입구 라벨로 자동 승격하지 않는다. draft의 빠진 문 라벨만
반환 사본에 unknown/null로 채우며 reviewed에는 모든 라벨 필드를 요구한다.
"""

from copy import deepcopy
import math
import re


ENVIRONMENTS = {'gazebo', 'real_rgb', 'real_rgbd'}
SPLITS = {'development', 'train', 'validation', 'test', 'unassigned'}
STATES = {'open', 'closed', 'unknown'}
VISIBILITIES = {'visible', 'occluded', 'unknown'}
DOOR_FIELDS = {'id', 'state', 'leaf_bbox', 'doorway_bbox', 'leaf_visibility', 'doorway_visibility'}


def _object(value, name):
    if not isinstance(value, dict):
        raise ValueError(f'{name}은 객체여야 합니다')
    return value


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{name}은 비어 있지 않은 문자열이어야 합니다')
    return value


def _required(value, names, name):
    missing = names - set(value)
    if missing:
        raise ValueError(f'{name} 필수 항목 누락: {", ".join(sorted(missing))}')


def _box(value, width, height, name):
    def finite_number(number):
        if isinstance(number, bool) or not isinstance(number, (int, float)):
            return False
        try:
            return math.isfinite(number)
        except OverflowError:
            return False

    if (not isinstance(value, list) or len(value) != 4
            or not all(finite_number(v) for v in value)
            or not 0 <= value[0] < value[2] <= width
            or not 0 <= value[1] < value[3] <= height):
        raise ValueError(f'{name}은 영상 안의 유한한 양수 면적 xyxy여야 합니다')


def validate_dataset(document):
    """계약과 중복·분할 누출을 검증하고 독립된 사본을 반환한다.

    파일 존재·원본 해시 일치·사람 검수의 진위·실제 장소의 동일성은 확인하지 않는다.
    group_id는 작성자가 동일 문/장소/촬영 세션에 일관되게 부여해야 한다.
    """
    _object(document, 'dataset')
    _required(document, {'schema_version', 'dataset_id', 'frames'}, 'dataset')
    if type(document['schema_version']) is not int or document['schema_version'] != 1:
        raise ValueError('schema_version은 정수 1이어야 합니다')
    _text(document['dataset_id'], 'dataset_id')
    if not isinstance(document['frames'], list):
        raise ValueError('frames는 목록이어야 합니다')
    result = deepcopy(document)
    identifiers, hashes, groups = set(), set(), {}
    for frame in result['frames']:
        _object(frame, 'frame')
        _required(frame, {'id', 'environment', 'group_id', 'split', 'rgb', 'capabilities',
                          'review_status', 'doors'}, 'frame')
        identifier = _text(frame['id'], 'frame.id')
        if identifier in identifiers:
            raise ValueError('중복 frame ID')
        identifiers.add(identifier)
        environment = frame['environment']
        split = frame['split']
        if not isinstance(environment, str) or environment not in ENVIRONMENTS:
            raise ValueError('environment 값 오류')
        if not isinstance(split, str) or split not in SPLITS:
            raise ValueError('split 값 오류')
        group = _text(frame['group_id'], 'group_id')
        category = 'gazebo' if environment == 'gazebo' else 'real'
        if group in groups and groups[group] != (split, category):
            raise ValueError('같은 group_id의 분할 누출 또는 Gazebo/실제 자료 혼합')
        groups[group] = (split, category)
        rgb = _object(frame['rgb'], 'rgb')
        _required(rgb, {'path', 'sha256', 'width', 'height'}, 'rgb')
        _text(rgb['path'], 'rgb.path')
        checksum = rgb['sha256']
        if not isinstance(checksum, str) or not re.fullmatch(r'[0-9a-f]{64}', checksum):
            raise ValueError('rgb.sha256은 소문자 64자리 SHA-256이어야 합니다')
        if checksum in hashes:
            raise ValueError('중복 RGB 해시: 같은 영상을 여러 프레임이나 분할에 넣을 수 없습니다')
        hashes.add(checksum)
        width, height = rgb['width'], rgb['height']
        if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
            raise ValueError('RGB 가로·세로는 양의 정수여야 합니다')
        capabilities = _object(frame['capabilities'], 'capabilities')
        if set(capabilities) != {'depth', 'camera_info', 'tf'} or any(type(v) is not bool for v in capabilities.values()):
            raise ValueError('capabilities는 depth/camera_info/tf의 명시적 bool 객체여야 합니다')
        if environment == 'real_rgb' and capabilities['depth']:
            raise ValueError('real_rgb에는 depth를 표시할 수 없습니다')
        if environment == 'real_rgbd' and not capabilities['depth']:
            raise ValueError('real_rgbd에는 depth가 필요합니다')
        reviewed = frame['review_status']
        if not isinstance(reviewed, str) or reviewed not in {'draft', 'reviewed'}:
            raise ValueError('review_status 값 오류')
        if not isinstance(frame['doors'], list):
            raise ValueError('doors는 목록이어야 합니다')
        door_ids = set()
        for door in frame['doors']:
            _object(door, 'door')
            _required(door, DOOR_FIELDS if reviewed == 'reviewed' else {'id'}, 'door')
            door_id = _text(door['id'], 'door.id')
            if door_id in door_ids:
                raise ValueError('같은 프레임의 중복 문 ID')
            door_ids.add(door_id)
            for key, default in [('state', 'unknown'), ('leaf_bbox', None), ('doorway_bbox', None),
                                 ('leaf_visibility', 'unknown'), ('doorway_visibility', 'unknown')]:
                door.setdefault(key, default)
            if not isinstance(door['state'], str) or door['state'] not in STATES:
                raise ValueError('문 state 값 오류')
            for target in ('leaf', 'doorway'):
                visibility, bbox = door[f'{target}_visibility'], door[f'{target}_bbox']
                if not isinstance(visibility, str) or visibility not in VISIBILITIES:
                    raise ValueError(f'{target}_visibility 값 오류')
                if visibility == 'visible':
                    _box(bbox, width, height, f'{target}_bbox')
                elif bbox is not None:
                    raise ValueError(f'{target}가 비가시·불명이면 bbox는 null이어야 합니다')
    return result
