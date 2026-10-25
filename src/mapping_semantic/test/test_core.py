"""실제 RGB-D 계약·좌표 변환·오검출 거부·지도 수명과 파일 보호 검증."""

import copy
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest

from mapping_semantic.core import (
    ObjectMap, bbox_mask, correction_delta, detect_color_fixtures, load_json,
    project_mask, save_exclusive, transform_point, validate_camera_inputs,
    external_masks, validate_static_classes, normalize_depth, validate_sync_policy,
)


def camera_messages():
    def header():
        return NS(stamp=NS(sec=1, nanosec=0), frame_id='camera_color_optical_frame')
    rgb = NS(header=header(), encoding='rgb8', width=20, height=10)
    depth = NS(header=header(), encoding='32FC1', width=20, height=10)
    info = NS(header=header(), width=20, height=10, k=[10, 0, 10, 0, 10, 5, 0, 0, 1],
              d=[0, 0, 0, 0, 0], distortion_model='plumb_bob', binning_x=0, binning_y=0,
              roi=NS(x_offset=0, y_offset=0, width=0, height=0))
    return rgb, depth, info


def test_camera_exact_contract():
    timestamp, k = validate_camera_inputs(*camera_messages(), 'camera_color_optical_frame')
    assert timestamp == 1_000_000_000
    assert k[0, 0] == 10


@pytest.mark.parametrize('encoding', ['rgb8', 'bgr8'])
def test_color_encoding_contract_preserves_original_header_and_intrinsics(encoding):
    rgb, depth, info = camera_messages()
    rgb.encoding = encoding
    timestamp, k = validate_camera_inputs(rgb, depth, info, rgb.header.frame_id)
    assert timestamp == 1_000_000_000 and rgb.encoding == encoding
    assert k == pytest.approx(np.array(info.k).reshape(3, 3))


@pytest.mark.parametrize('encoding', ['mono8', 'rgba8', 'bgra8', '16UC3', '8UC3', ''])
def test_unsupported_color_encoding_rejected(encoding):
    rgb, depth, info = camera_messages()
    rgb.encoding = encoding
    with pytest.raises(ValueError):
        validate_camera_inputs(rgb, depth, info, rgb.header.frame_id)


def test_bgr_to_rgb_channel_conversion_preserves_fixture_pixel_masks():
    import cv2
    rgb = np.zeros((50, 60, 3), np.uint8)
    rgb[5:25, 5:25] = [100, 0, 0]
    rgb[25:45, 35:55] = [0, 0, 100]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    converted = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    expected = dict(detect_color_fixtures(rgb))
    actual = dict(detect_color_fixtures(converted))
    assert set(actual) == {'red_fixture', 'blue_fixture'}
    assert all(np.array_equal(expected[label], actual[label]) for label in expected)
    # 이 순수 검사는 채널 순서·픽셀 위치 계약이며 실제 CvBridge 실행은 ROS probe로 별도 검증한다.


@pytest.mark.parametrize('kind', ['frame', 'time', 'zero_time', 'encoding', 'size',
                                 'distortion', 'focal', 'binning', 'roi'])
def test_misaligned_or_invalid_camera_is_rejected(kind):
    rgb, depth, info = camera_messages()
    if kind == 'frame':
        depth.header.frame_id = 'depth_optical_frame'
    elif kind == 'time':
        depth.header.stamp.nanosec = 1
    elif kind == 'zero_time':
        rgb.header.stamp.sec = 0
    elif kind == 'encoding':
        depth.encoding = 'mono16'
    elif kind == 'size':
        depth.width = 10
    elif kind == 'distortion':
        info.d[0] = 0.1
    elif kind == 'focal':
        info.k[0] = float('nan')
    elif kind == 'binning':
        info.binning_x = 2
    else:
        info.roi.width = 10
    with pytest.raises(ValueError):
        validate_camera_inputs(rgb, depth, info, 'camera_color_optical_frame')


def test_fixture_detection_in_dim_rgb_and_absence():
    rgb = np.full((80, 100, 3), 40, np.uint8)
    assert detect_color_fixtures(rgb) == []
    rgb[10:30, 10:30] = [100, 0, 0]
    rgb[40:60, 60:80] = [0, 0, 100]
    detections = detect_color_fixtures(rgb)
    assert [d[0] for d in detections] == ['red_fixture', 'blue_fixture']
    assert all(mask.sum() == 18 * 18 for _, mask in detections)


def test_project_mask_ignores_background_and_invalid_pixels():
    depth = np.full((20, 20), 9.0, np.float32)
    mask = bbox_mask(depth.shape, [10, 10, 10, 10])
    depth[mask] = 2
    depth[5, 5] = float('inf')
    depth[5, 6] = 0
    k = np.array([[100, 0, 9.5], [0, 100, 9.5], [0, 0, 1]])
    point = project_mask(depth, k, mask)
    assert point == pytest.approx([0, 0, 2], abs=0.02)


@pytest.mark.parametrize('value', [0, -1, float('inf'), float('-inf'), float('nan'), 20])
def test_invalid_depth_does_not_create_position(value):
    with pytest.raises(ValueError):
        project_mask(np.full((10, 10), value), np.eye(3), np.ones((10, 10), bool))


def test_optical_axes_and_translation_match_measured_tf():
    translation = [1.082373, 1.000079091, 0.119864]
    quaternion = [-0.5, 0.5, -0.5, 0.5]
    assert transform_point([0, 0, 2], translation, quaternion) == pytest.approx([3.082373, 1.000079091, 0.119864])
    assert transform_point([1, 1, 2], translation, quaternion) == pytest.approx([3.082373, 0.000079091, -0.880136])


def test_invalid_quaternion_is_rejected():
    with pytest.raises(ValueError):
        transform_point([1, 0, 0], [0, 0, 0], [0, 0, 0, 0])


def confirmed_map():
    store = ObjectMap()
    store.observe_correction([0, 0, 0])
    for i in range(1, 4):
        store.update([('red_fixture', [2, 0, 0.3])], i)
    return store


def test_fusion_stable_ids_and_minimum_observations():
    store = ObjectMap()
    first = store.update([('red_fixture', [2, 0, 0])], 1)[0].id
    assert not store.confirmed()
    store.update([('red_fixture', [2.1, 0, 0])], 2)
    store.update([('red_fixture', [1.9, 0, 0])], 3)
    assert store.confirmed()[0].id == first
    assert store.confirmed()[0].position == pytest.approx([2, 0, 0])
    assert store.confirmed()[0].observations == 3


def test_one_frame_cannot_confirm_one_object_three_times():
    store = ObjectMap()
    store.update([('red_fixture', [2, 0, 0])] * 3, 1)
    assert not store.confirmed()
    assert len(store.objects) == 3
    with pytest.raises(ValueError):
        store.update([('red_fixture', [2, 0, 0])], 1)


def test_classes_and_distant_instances_are_not_merged():
    store = ObjectMap()
    store.update([('red_fixture', [2, 0, 0]), ('blue_fixture', [2, 0, 0]),
                  ('red_fixture', [3, 0, 0])], 1)
    assert len({o.id for o in store.objects}) == 3


def test_jump_and_accumulated_correction_clear_old_objects():
    store = confirmed_map()
    old_id = store.confirmed()[0].id
    assert not store.observe_correction([0.12, 0, 0])
    assert store.observe_correction([0.24, 0, 0])
    assert store.objects == [] and store.epoch == 1
    assert store.update([('red_fixture', [2, 0, 0])], 4)[0].id != old_id
    assert store.observe_correction([0.24, 0, 0.11])
    assert correction_delta([0, 0, math.pi - 0.01], [0, 0, -math.pi + 0.01])[1] == pytest.approx(0.02)


def test_save_load_roundtrip_and_overwrite_protection(tmp_path):
    store = confirmed_map()
    path = tmp_path / 'objects.json'
    save_exclusive(path, store.to_document('fixture_test'))
    restored = ObjectMap()
    restored.load_document(load_json(path), 'fixture_test', [0, 0, 0])
    assert restored.confirmed()[0].id == store.confirmed()[0].id
    assert restored.confirmed()[0].position == pytest.approx([2, 0, 0.3])
    with pytest.raises(FileExistsError):
        save_exclusive(path, store.to_document('fixture_test'))


@pytest.mark.parametrize('kind', ['map_id', 'frame', 'nonfinite', 'duplicate', 'counter', 'count'])
def test_invalid_load_is_atomic(kind):
    store = confirmed_map()
    document = copy.deepcopy(store.to_document('fixture_test'))
    if kind == 'map_id':
        document['map_id'] = 'other'
    elif kind == 'frame':
        document['frame'] = 'odom'
    elif kind == 'nonfinite':
        document['objects'][0]['position'][0] = float('nan')
    elif kind == 'duplicate':
        document['objects'].append(copy.deepcopy(document['objects'][0]))
    elif kind == 'counter':
        document['next_id'] = 1
    else:
        document['objects'][0]['observations'] = 0
    with pytest.raises(ValueError):
        store.load_document(document, 'fixture_test', [0, 0, 0])
    assert len(store.confirmed()) == 1


def test_wrong_correction_or_unidentified_map_rejected():
    store = confirmed_map()
    with pytest.raises(ValueError):
        store.load_document(store.to_document('a'), 'a', [0.3, 0, 0])
    with pytest.raises(ValueError):
        store.to_document('')


def test_nonfinite_json_is_rejected(tmp_path):
    path = tmp_path / 'bad.json'
    path.write_text('{"value": NaN}')
    with pytest.raises(ValueError):
        load_json(path)


@pytest.mark.parametrize('identifier', [
    'red_fixture_1', 'red_fixture_0000001', 'red_fixture_+00001',
    'red_fixture_2147483648', 'red_fixture_000000',
])
def test_loaded_marker_id_must_be_canonical_int32(identifier):
    store = confirmed_map()
    document = store.to_document('a')
    document['objects'][0]['id'] = identifier
    document['next_id'] = 2147483648
    with pytest.raises(ValueError):
        store.load_document(document, 'a', [0, 0, 0])


def test_loaded_suffix_is_globally_unique():
    store = confirmed_map()
    document = store.to_document('a')
    duplicate = copy.deepcopy(document['objects'][0])
    duplicate.update(id='blue_fixture_000001', class_id='blue_fixture')
    document['objects'].append(duplicate)
    with pytest.raises(ValueError):
        store.load_document(document, 'a', [0, 0, 0])


@pytest.mark.parametrize('next_id', [0, -1, True, 1.5, 2147483649])
def test_loaded_next_id_is_bounded(next_id):
    store = confirmed_map()
    document = store.to_document('a')
    document['next_id'] = next_id
    with pytest.raises(ValueError):
        store.load_document(document, 'a', [0, 0, 0])


def test_maximum_marker_id_can_load_but_cannot_overflow():
    store = confirmed_map()
    document = store.to_document('a')
    document['objects'][0]['id'] = 'red_fixture_2147483647'
    document['next_id'] = 2147483648
    store.load_document(document, 'a', [0, 0, 0])
    assert store.update([('red_fixture', [2, 0, 0.3])], 4)[0].observations == 4
    with pytest.raises(ValueError, match='소진'):
        store.update([('blue_fixture', [3, 0, 0.3])], 5)
    assert len(store.objects) == 1


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_save_nonfinite_does_not_create_file(tmp_path, value):
    path = tmp_path / 'invalid.json'
    with pytest.raises(ValueError):
        save_exclusive(path, {'value': value})
    assert not path.exists()


def external_message(label='chair', score=0.9):
    header = NS(stamp=NS(sec=1, nanosec=0), frame_id='optical')
    detection = NS(header=copy.deepcopy(header),
                   bbox=NS(center=NS(position=NS(x=20., y=20.), theta=0.), size_x=30., size_y=30.),
                   results=[NS(hypothesis=NS(class_id=label, score=score))])
    return NS(header=header, detections=[detection]), copy.deepcopy(header)


def test_external_roi_projection_and_model_confidence():
    message, header = external_message()
    label, mask, score = external_masks(message, header, (40, 40), ['chair'])[0]
    assert label == 'chair' and score == 0.9 and mask.sum() == 225
    assert project_mask(np.full((40, 40), 2.), np.array([[10, 0, 20], [0, 10, 20], [0, 0, 1]]), mask) == pytest.approx([0, 0, 2])


@pytest.mark.parametrize('kind', ['stamp', 'frame', 'childstamp', 'nan', 'negative', 'theta', 'outside', 'score'])
def test_external_bad_contract_rejected(kind):
    message, header = external_message()
    box = message.detections[0].bbox
    if kind == 'stamp': message.header.stamp.nanosec = 1
    elif kind == 'frame': message.header.frame_id = 'other'
    elif kind == 'childstamp': message.detections[0].header.stamp.sec = 2
    elif kind == 'nan': box.center.position.x = math.nan
    elif kind == 'negative': box.size_x = -1
    elif kind == 'theta': box.center.theta = 0.1
    elif kind == 'outside': box.center.position.x = -100
    else: message.detections[0].results[0].hypothesis.score = math.inf
    with pytest.raises(ValueError): external_masks(message, header, (40, 40), ['chair'])


@pytest.mark.parametrize('classes', [['person'], ['chair', 'car'], ['traffic light'], ['A'], ['a'*65], ['chair', 'chair']])
def test_unsafe_static_classes_rejected(classes):
    with pytest.raises(ValueError): validate_static_classes(classes)


def test_external_filter_and_person_never_accumulates():
    message, header = external_message('person')
    assert external_masks(message, header, (40, 40), ['chair'])[0][0] == 'person'
    store = ObjectMap(allowed_classes=['chair'], detector_mode='external_detections')
    with pytest.raises(ValueError): store.update([('person', [1, 2, 3])], 1)
    assert not store.objects
    message.detections[0].results[0].hypothesis.class_id = 'cup'
    assert external_masks(message, header, (40, 40), ['chair']) == []
    message.detections[0].results[0].hypothesis.class_id = 'chair'
    message.detections[0].results[0].hypothesis.score = 0.49
    assert external_masks(message, header, (40, 40), ['chair']) == []


def test_external_serialization_preserves_contract_and_score():
    store = ObjectMap(allowed_classes=['chair'], detector_mode='external_detections')
    store.observe_correction([0, 0, 0])
    for stamp in range(1, 4): store.update([('chair', [1, 2, 3])], stamp)
    store.objects[0].score = 0.8
    document = store.to_document('a')
    assert document['schema_version'] == 2
    restored = ObjectMap(allowed_classes=['chair'], detector_mode='external_detections')
    restored.load_document(document, 'a', [0, 0, 0])
    assert restored.objects[0].score == 0.8
    with pytest.raises(ValueError): ObjectMap().load_document(document, 'a', [0, 0, 0])
    document['static_classes'] = ['cup']
    with pytest.raises(ValueError): restored.load_document(document, 'a', [0, 0, 0])


@pytest.mark.parametrize('offset,accepted', [(0, True), (29_999_999, True),
                                            (30_000_000, True), (30_000_001, False)])
def test_approximate_skew_boundary_uses_rgb_timestamp(offset, accepted):
    rgb, depth, info = camera_messages()
    depth.header.stamp.nanosec = offset
    if accepted:
        timestamp, _ = validate_camera_inputs(rgb, depth, info, rgb.header.frame_id, 'approximate', .03)
        assert timestamp == 1_000_000_000
    else:
        with pytest.raises(ValueError):
            validate_camera_inputs(rgb, depth, info, rgb.header.frame_id, 'approximate', .03)


def test_approximate_limits_total_span_not_individual_rgb_offsets():
    rgb, depth, info = camera_messages()
    depth.header.stamp.nanosec = 20_000_000
    info.header.stamp.sec, info.header.stamp.nanosec = 0, 980_000_000
    with pytest.raises(ValueError):
        validate_camera_inputs(rgb, depth, info, rgb.header.frame_id, 'approximate', .03)


@pytest.mark.parametrize('kind', ['zero_info', 'stale_info', 'depth_frame', 'distortion'])
def test_approximate_does_not_relax_calibration_or_latched_info(kind):
    rgb, depth, info = camera_messages()
    if kind == 'zero_info': info.header.stamp.sec = 0
    elif kind == 'stale_info': info.header.stamp.sec = 2
    elif kind == 'depth_frame': depth.header.frame_id = 'unregistered_depth'
    else: info.d[0] = .01
    with pytest.raises(ValueError):
        validate_camera_inputs(rgb, depth, info, rgb.header.frame_id, 'approximate', .03)


@pytest.mark.parametrize('policy,skew', [('other', .03), ('exact', 0), ('approximate', -.1),
                                       ('approximate', .100001), ('approximate', math.nan),
                                       ('approximate', math.inf), ('exact', True)])
def test_invalid_sync_configuration(policy, skew):
    with pytest.raises(ValueError): validate_sync_policy(policy, skew)


def test_16uc1_mm_to_m_preserves_invalid_zero_and_input():
    depth = np.array([[0, 200, 1000, 2345, 65535]], dtype=np.uint16)
    original = depth.copy()
    result = normalize_depth(depth, '16UC1')
    assert result.dtype == np.float32
    assert result[0] == pytest.approx([0, .2, 1., 2.345, 65.535], abs=1e-5)
    assert np.array_equal(depth, original)


def test_32fc1_units_and_nonfinite_values_are_preserved_for_rejection():
    depth = np.array([[0., math.nan, math.inf, -math.inf, 2.]], np.float32)
    result = normalize_depth(depth, '32FC1')
    assert result.dtype == np.float32 and result[0,4] == 2
    assert result[0,0] == 0 and np.isnan(result[0,1]) and np.isposinf(result[0,2])
    with pytest.raises(ValueError):
        project_mask(result, np.eye(3), np.ones(result.shape, bool), minimum_valid=1)


@pytest.mark.parametrize('dtype,encoding', [(np.float32,'16UC1'),(np.int16,'16UC1'),
                                           (np.uint8,'16UC1'),(np.float64,'32FC1'),
                                           (np.uint16,'32FC1'),(np.uint16,'mono16')])
def test_encoding_dtype_mismatch_rejected(dtype, encoding):
    with pytest.raises(ValueError): normalize_depth(np.ones((2,2),dtype),encoding)


def test_depth_dimensions_and_big_endian_uint16():
    with pytest.raises(ValueError): normalize_depth(np.ones((2,2,1),np.uint16),'16UC1')
    assert normalize_depth(np.array([[2000]],dtype='>u2'),'16UC1')[0,0] == 2.


def test_approximate_camera_never_relaxes_external_detection_header():
    rgb, depth, info = camera_messages()
    depth.header.stamp.nanosec = 10_000_000
    validate_camera_inputs(rgb, depth, info, rgb.header.frame_id, 'approximate', .03)
    message, _ = external_message()
    message.header = copy.deepcopy(rgb.header)
    message.detections[0].header = copy.deepcopy(rgb.header)
    assert external_masks(message,rgb.header,(40,40),['chair'])
    # 외부 검출을 depth 시각으로 바꾸면 skew 안쪽이어도 RGB와 다르므로 거부.
    message.header = copy.deepcopy(depth.header)
    message.detections[0].header = copy.deepcopy(depth.header)
    with pytest.raises(ValueError): external_masks(message,rgb.header,(40,40),['chair'])


def test_depth_encoding_changes_do_not_change_fusion_or_meter_scale():
    rgb, depth, info = camera_messages()
    depth.encoding = '16UC1'
    _, k = validate_camera_inputs(rgb, depth, info, rgb.header.frame_id)
    store = ObjectMap()
    original = None
    for timestamp, encoding, array in [
        (1,'16UC1',np.full((10,20),2000,np.uint16)),
        (2,'32FC1',np.full((10,20),2.,np.float32)),
        (3,'16UC1',np.full((10,20),2000,np.uint16)),
    ]:
        xyz = project_mask(normalize_depth(array,encoding),k,np.ones((10,20),bool))
        record = store.update([('red_fixture',xyz)],timestamp)[0]
        original = original or record.id
        assert record.id == original and xyz[2] == 2.
    assert len(store.confirmed()) == 1 and store.confirmed()[0].observations == 3


@pytest.mark.parametrize('value', [None, {'red_fixture': 1, 'blue_fixture': 2}])
def test_external_schema_requires_explicit_class_list(value):
    store = ObjectMap(allowed_classes=['red_fixture', 'blue_fixture'], detector_mode='external_detections')
    store.observe_correction([0, 0, 0])
    document = store.to_document('test-map')
    if value is None:
        document.pop('static_classes')
    else:
        document['static_classes'] = value
    with pytest.raises(ValueError):
        store.load_document(document, 'test-map', [0, 0, 0])
