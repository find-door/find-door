"""ROS와 분리한 입력 검증·투영·정적 객체 병합·직렬화."""

from dataclasses import dataclass
import json
import math
import re
from pathlib import Path

import cv2
import numpy as np

DYNAMIC_CLASSES = frozenset('person bicycle car motorcycle bus train truck bird cat dog horse sheep cow elephant bear zebra giraffe'.split())


def validate_static_classes(classes):
    if (not isinstance(classes, (list, tuple)) or not classes
            or any(not isinstance(c, str) or re.fullmatch(r'[a-z][a-z0-9_]{0,63}', c) is None for c in classes)
            or len(set(classes)) != len(classes) or set(classes) & DYNAMIC_CLASSES):
        raise ValueError('static_classes는 중복 없는 안전한 class ID이며 동적 클래스와 겹치지 않아야 함')
    return tuple(classes)


def external_masks(message, image_header, shape, static_classes):
    """동일 시각 외부 검출의 중앙 50% ROI와 모델 점수를 반환한다."""
    expected = stamp_ns(image_header.stamp)
    def check_header(header):
        if header.frame_id != image_header.frame_id or stamp_ns(header.stamp) != expected:
            raise ValueError('Detection2D frame/timestamp 불일치')
    check_header(message.header)
    result = []
    allowed = set(validate_static_classes(static_classes)) | DYNAMIC_CLASSES
    for detection in message.detections:
        check_header(detection.header)
        hypotheses = []
        for item in detection.results:
            score = item.hypothesis.score
            if not math.isfinite(score) or not 0 <= score <= 1:
                raise ValueError('모델 confidence 범위 오류')
            hypotheses.append((score, item.hypothesis.class_id))
        if not hypotheses:
            continue
        score, label = max(hypotheses, key=lambda pair: pair[0])
        if score < 0.5 or label not in allowed:
            continue
        box = detection.bbox
        cx, cy, width, height, theta = finite_vector(
            [box.center.position.x, box.center.position.y, box.size_x, box.size_y, box.center.theta], 5, 'bbox')
        if width <= 0 or height <= 0 or theta != 0:
            raise ValueError('양수 크기의 비회전 bbox 필요')
        if cx + width / 2 <= 0 or cy + height / 2 <= 0 or cx - width / 2 >= shape[1] or cy - height / 2 >= shape[0]:
            raise ValueError('bbox가 영상 밖에 있음')
        # 영상 내부로 자른 bbox의 중앙 영역: 잘린 물체에서도 밖의 픽셀을 쓰지 않는다.
        x0, x1 = max(0, cx - width / 2), min(shape[1], cx + width / 2)
        y0, y1 = max(0, cy - height / 2), min(shape[0], cy + height / 2)
        mask = bbox_mask(shape, [(x0+x1)/2, (y0+y1)/2, (x1-x0)/2, (y1-y0)/2])
        if mask.sum() < 20:
            raise ValueError('중앙 ROI가 너무 작음')
        result.append((label, mask, float(score)))
    return result


def finite_vector(values, length, name):
    array = np.asarray(values, dtype=float)
    if array.shape != (length,) or not np.isfinite(array).all():
        raise ValueError(f'{name}: 유한한 {length}개 값 필요')
    return array


def stamp_ns(stamp):
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('유효하지 않은 timestamp')
    result = stamp.sec * 1_000_000_000 + stamp.nanosec
    if result <= 0:
        raise ValueError('timestamp는 0보다 커야 함')
    return result


def validate_sync_policy(sync_policy='exact', max_sync_skew_sec=0.03):
    if sync_policy not in ('exact', 'approximate'):
        raise ValueError('sync_policy는 exact 또는 approximate이어야 함')
    if (isinstance(max_sync_skew_sec, bool) or not isinstance(max_sync_skew_sec, (int, float))
            or not math.isfinite(max_sync_skew_sec) or not 0 < max_sync_skew_sec <= 0.1):
        raise ValueError('max_sync_skew_sec는 유한한 양수이며 0.1초 이하여야 함')
    return 0 if sync_policy == 'exact' else round(max_sync_skew_sec * 1e9)


def normalize_depth(depth, encoding):
    """정렬된 depth를 float32 미터로 변환한다. 0·NaN·inf는 투영 단계에서 거부한다."""
    if not isinstance(depth, np.ndarray) or depth.ndim != 2:
        raise ValueError('depth는 2차원 배열이어야 함')
    if encoding == '16UC1' and depth.dtype.kind == 'u' and depth.dtype.itemsize == 2:
        return depth.astype(np.float32) * np.float32(0.001)
    if encoding == '32FC1' and depth.dtype.kind == 'f' and depth.dtype.itemsize == 4:
        return depth.astype(np.float32, copy=False)
    raise ValueError('depth 인코딩/dtype 불일치: 16UC1 uint16(mm) 또는 32FC1 float32(m) 필요')


def validate_camera_inputs(rgb, depth, info, optical_frame,
                           sync_policy='exact', max_sync_skew_sec=0.03):
    """정렬·무왜곡 입력과 시각 편차를 검증하며 항상 RGB timestamp를 반환한다."""
    allowed_skew_ns = validate_sync_policy(sync_policy, max_sync_skew_sec)
    stamps = [stamp_ns(message.header.stamp) for message in (rgb, depth, info)]
    if max(stamps) - min(stamps) > allowed_skew_ns:
        raise ValueError('RGB/depth/CameraInfo timestamp 편차가 동기화 허용치를 넘음')
    if any(message.header.frame_id != optical_frame for message in (rgb, depth, info)):
        raise ValueError('RGB/depth/CameraInfo optical frame 불일치')
    if rgb.encoding not in ('rgb8', 'bgr8') or depth.encoding not in ('32FC1', '16UC1'):
        raise ValueError('rgb8/bgr8 및 32FC1(m)/16UC1(mm)만 지원')
    if rgb.width <= 0 or rgb.height <= 0:
        raise ValueError('영상 크기는 양수여야 함')
    if any((m.width, m.height) != (rgb.width, rgb.height) for m in (depth, info)):
        raise ValueError('RGB/depth/CameraInfo 해상도 불일치')
    k = finite_vector(info.k, 9, 'CameraInfo.K').reshape(3, 3)
    if (k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1])
            or abs(k[0, 1]) > 1e-9 or abs(k[1, 0]) > 1e-9):
        raise ValueError('지원하지 않는 pinhole K')
    distortion = np.asarray(info.d, dtype=float)
    if not np.isfinite(distortion).all() or np.any(np.abs(distortion) > 1e-9):
        raise ValueError('왜곡 보정되지 않은 CameraInfo')
    if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
        raise ValueError('지원하지 않는 distortion_model')
    if info.binning_x not in (0, 1) or info.binning_y not in (0, 1):
        raise ValueError('binning된 CameraInfo 미지원')
    if any((info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height)):
        raise ValueError('ROI CameraInfo 미지원')
    return stamps[0], k


def bbox_mask(shape, bbox):
    """향후 외부 2D detector 연결용 중심/크기 bbox → bool mask 어댑터."""
    cx, cy, width, height = finite_vector(bbox, 4, 'bbox')
    if width <= 0 or height <= 0:
        raise ValueError('bbox 크기는 양수여야 함')
    mask = np.zeros(shape, dtype=bool)
    x0 = max(0, int(math.ceil(cx - width / 2)))
    x1 = min(shape[1], int(math.ceil(cx + width / 2)))
    y0 = max(0, int(math.ceil(cy - height / 2)))
    y1 = min(shape[0], int(math.ceil(cy + height / 2)))
    if x1 > x0 and y1 > y0:
        mask[y0:y1, x0:x1] = True
    return mask


def detect_color_fixtures(rgb, minimum_pixels=80):
    """합성 red_fixture/blue_fixture만 검출한다. 학습된 실제 물체 인식기가 아니다."""
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError('RGB uint8 H×W×3 영상 필요')
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    saturated = (hsv[:, :, 1] >= 130) & (hsv[:, :, 2] >= 60)
    masks = {
        'red_fixture': saturated & ((hsv[:, :, 0] <= 10) | (hsv[:, :, 0] >= 170)),
        'blue_fixture': saturated & (hsv[:, :, 0] >= 100) & (hsv[:, :, 0] <= 130),
    }
    detections = []
    for label, mask in masks.items():
        count, components, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
        for index in range(1, count):
            if stats[index, cv2.CC_STAT_AREA] < minimum_pixels:
                continue
            interior = cv2.erode((components == index).astype(np.uint8),
                                 np.ones((3, 3), np.uint8), iterations=1).astype(bool)
            if interior.sum() >= 20:
                detections.append((label, interior))
    return detections


def project_mask(depth, k, mask, minimum_valid=20, minimum_fraction=0.5,
                 minimum_depth=0.2, maximum_depth=10.0):
    """마스크 내부 유효 픽셀의 XYZ 중앙값: 보이는 표면의 대표점."""
    if depth.ndim != 2 or mask.shape != depth.shape or mask.dtype != np.bool_:
        raise ValueError('depth와 bool mask 크기 불일치')
    k = np.asarray(k, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('유효하지 않은 K')
    valid = mask & np.isfinite(depth) & (depth >= minimum_depth) & (depth <= maximum_depth)
    valid_count = int(valid.sum())
    if valid_count < minimum_valid or valid_count < int(mask.sum()) * minimum_fraction:
        raise ValueError('마스크 안 유효 깊이 부족')
    rows, cols = np.nonzero(valid)
    z = depth[valid].astype(float)
    points = np.column_stack(((cols - k[0, 2]) * z / k[0, 0],
                              (rows - k[1, 2]) * z / k[1, 1], z))
    return np.median(points, axis=0)


def rotation_matrix(quaternion):
    q = finite_vector(quaternion, 4, 'quaternion')
    norm = np.linalg.norm(q)
    if norm < 1e-9:
        raise ValueError('영 quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def transform_point(point, translation, quaternion):
    return (rotation_matrix(quaternion) @ finite_vector(point, 3, 'point')
            + finite_vector(translation, 3, 'translation'))


def correction_delta(reference, current):
    a = finite_vector(reference, 3, 'reference correction')
    b = finite_vector(current, 3, 'current correction')
    yaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
    return math.hypot(b[0] - a[0], b[1] - a[1]), abs(yaw)


@dataclass
class ObjectRecord:
    id: str
    class_id: str
    position: np.ndarray
    observations: int
    last_stamp_ns: int
    score: float = 1.0


class ObjectMap:
    """epoch 기준 보정 변화가 임계값을 넘으면 전체 초기화하는 정적 fixture 지도."""

    def __init__(self, association_distance=0.25, minimum_observations=3,
                 reset_translation=0.2, reset_rotation=0.1,
                 allowed_classes=('red_fixture', 'blue_fixture'), detector_mode='color_fixture'):
        self.allowed_classes = validate_static_classes(allowed_classes)
        if detector_mode not in ('color_fixture', 'external_detections'):
            raise ValueError('지원하지 않는 detector_mode')
        self.detector_mode = detector_mode
        if not all(math.isfinite(v) and v > 0 for v in
                   (association_distance, reset_translation, reset_rotation)):
            raise ValueError('거리·각도 임계값은 유한한 양수여야 함')
        if (isinstance(minimum_observations, bool)
                or not isinstance(minimum_observations, int) or minimum_observations < 1):
            raise ValueError('minimum_observations는 양의 정수여야 함')
        self.association_distance = association_distance
        self.minimum_observations = minimum_observations
        self.reset_translation = reset_translation
        self.reset_rotation = reset_rotation
        self.objects = []
        self.reference = None
        self.epoch = 0
        self.next_id = 1
        self.last_stamp_ns = 0

    def clear(self, correction=None):
        self.objects = []
        self.reference = None if correction is None else finite_vector(correction, 3, 'correction').copy()
        self.epoch += 1
        self.last_stamp_ns = 0

    def observe_correction(self, correction):
        correction = finite_vector(correction, 3, 'correction')
        if self.reference is None:
            self.reference = correction.copy()
            return False
        distance, yaw = correction_delta(self.reference, correction)
        if distance > self.reset_translation or yaw > self.reset_rotation:
            self.clear(correction)
            return True
        return False

    def update(self, observations, timestamp):
        if isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp <= self.last_stamp_ns:
            raise ValueError('관측 timestamp가 양수·단조 증가해야 함')
        validated = []
        for label, point in observations:
            if label not in self.allowed_classes:
                raise ValueError('지도 누적이 허용되지 않은 class_id')
            validated.append((label, finite_vector(point, 3, 'position')))
        self.last_stamp_ns = timestamp
        matched, result = set(), []
        for label, point in validated:
            candidates = [(float(np.linalg.norm(obj.position - point)), obj)
                          for obj in self.objects if obj.class_id == label and obj.id not in matched]
            distance, nearest = min(candidates, key=lambda item: item[0]) if candidates else (math.inf, None)
            if distance <= self.association_distance:
                nearest.position += (point - nearest.position) / (nearest.observations + 1)
                nearest.observations += 1
                nearest.last_stamp_ns = timestamp
            else:
                if self.next_id > 2147483647:
                    raise ValueError('Marker int32 객체 ID 공간이 소진됨')
                identifier = f'{label}_{self.next_id:06d}'
                self.next_id += 1
                nearest = ObjectRecord(identifier, label, point.copy(), 1, timestamp)
                self.objects.append(nearest)
            matched.add(nearest.id)
            result.append(nearest)
        return result

    def confirmed(self):
        return [obj for obj in self.objects if obj.observations >= self.minimum_observations]

    def to_document(self, map_id, frame='map'):
        if not isinstance(map_id, str) or not map_id.strip() or self.reference is None:
            raise ValueError('명시적인 map_id 및 관측된 map→odom 보정이 필요함')
        return {
            'schema_version': 1 if self.detector_mode == 'color_fixture' else 2,
            'map_id': map_id, 'frame': frame,
            'detector_mode': self.detector_mode, 'static_classes': list(self.allowed_classes),
            'point_definition': 'visible_surface_median_xyz',
            'correction_reference': self.reference.tolist(), 'epoch': self.epoch,
            'next_id': self.next_id,
            'objects': [{'id': obj.id, 'class_id': obj.class_id,
                         'position': obj.position.tolist(), 'observations': obj.observations,
                         'last_stamp_ns': obj.last_stamp_ns, 'score': obj.score} for obj in self.confirmed()],
        }

    def load_document(self, document, map_id, correction, frame='map'):
        schema = 1 if self.detector_mode == 'color_fixture' else 2
        if not isinstance(document, dict) or document.get('schema_version') != schema:
            raise ValueError('지원하지 않는 객체 지도 schema')
        if not map_id or document.get('map_id') != map_id or document.get('frame') != frame:
            raise ValueError('객체 지도의 map_id/frame 불일치')
        if document.get('detector_mode') != self.detector_mode:
            raise ValueError('객체 지도 detector_mode 불일치')
        classes = document.get('static_classes', ['red_fixture', 'blue_fixture'] if schema == 1 else None)
        if not isinstance(classes, list) or tuple(classes) != self.allowed_classes:
            raise ValueError('저장 static_classes 계약 불일치')
        reference = finite_vector(document.get('correction_reference'), 3, 'saved correction')
        distance, angle = correction_delta(reference, correction)
        if distance > self.reset_translation or angle > self.reset_rotation:
            raise ValueError('저장 지도와 현재 map→odom 보정 불일치')
        records = document.get('objects')
        if not isinstance(records, list):
            raise ValueError('objects는 목록이어야 함')
        loaded, ids, suffixes, largest_suffix = [], set(), set(), 0
        for record in records:
            if not isinstance(record, dict):
                raise ValueError('object는 사전이어야 함')
            identifier, label = record.get('id'), record.get('class_id')
            count, timestamp = record.get('observations'), record.get('last_stamp_ns')
            if (not isinstance(identifier, str) or not identifier or identifier in ids
                    or label not in self.allowed_classes
                    or isinstance(count, bool) or not isinstance(count, int) or count < self.minimum_observations
                    or isinstance(timestamp, bool) or not isinstance(timestamp, int) or timestamp <= 0):
                raise ValueError('유효하지 않은 저장 object')
            try:
                prefix, suffix = identifier.rsplit('_', 1)
                suffix = int(suffix)
            except (ValueError, AttributeError) as exc:
                raise ValueError('유효하지 않은 object ID') from exc
            if (prefix != label or not 1 <= suffix <= 2147483647
                    or identifier != f'{label}_{suffix:06d}' or suffix in suffixes):
                raise ValueError('유효하지 않은 object ID')
            largest_suffix = max(largest_suffix, suffix)
            ids.add(identifier)
            suffixes.add(suffix)
            score = record.get('score', 1.0 if schema == 1 else None)
            if not isinstance(score, (int, float)) or isinstance(score, bool) or not math.isfinite(score) or not 0 <= score <= 1:
                raise ValueError('저장 confidence 오류')
            loaded.append(ObjectRecord(identifier, label,
                                       finite_vector(record.get('position'), 3, 'saved position'),
                                       count, timestamp, float(score)))
        next_id = document.get('next_id')
        # int32 최대값 다음 번호는 신규 할당을 거부하는 소진 sentinel이다.
        if (isinstance(next_id, bool) or not isinstance(next_id, int)
                or not largest_suffix < next_id <= 2147483648):
            raise ValueError('유효하지 않은 next_id')
        self.clear(reference)
        self.objects = loaded
        self.next_id = max(self.next_id, next_id)


def save_exclusive(path, document):
    if not path:
        raise ValueError('output_path가 비어 있음')
    payload = json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    with Path(path).open('x', encoding='utf-8') as stream:
        stream.write(payload)


def load_json(path):
    if not path:
        raise ValueError('input_path가 비어 있음')
    def reject_constant(value):
        raise ValueError(f'비유한 JSON 수치: {value}')
    with Path(path).open(encoding='utf-8') as stream:
        return json.load(stream, parse_constant=reject_constant)
