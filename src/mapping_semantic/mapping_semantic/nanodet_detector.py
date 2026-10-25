"""OpenCV Zoo NanoDet CPU 추론과 원본 영상 좌표 복원. ROS 의존성 없음."""
import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import time

import cv2
import numpy as np

from mapping_semantic.third_party.nanodet import NanoDet

COCO_CLASSES = (
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck',
    'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
    'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
    'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
    'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
    'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
    'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse',
    'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
    'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
    'toothbrush',
)


def check_rgb(rgb):
    if (not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8
            or rgb.ndim != 3 or rgb.shape[2] != 3 or min(rgb.shape[:2]) <= 0):
        raise ValueError('RGB uint8 H×W×3 영상이 필요합니다')


@dataclass(frozen=True)
class Letterbox:
    width: int
    height: int
    scale_x: float
    scale_y: float
    left: int
    top: int


def letterbox(rgb, size=416):
    check_rgb(rgb)
    if isinstance(size, bool) or not isinstance(size, int) or size < 1:
        raise ValueError('letterbox size는 양의 정수여야 합니다')
    height, width = rgb.shape[:2]
    ratio = size / max(height, width)
    resized_w, resized_h = max(1, round(width * ratio)), max(1, round(height * ratio))
    left, top = (size - resized_w) // 2, (size - resized_h) // 2
    result = np.full((size, size, 3), 114, dtype=np.uint8)
    result[top:top + resized_h, left:left + resized_w] = cv2.resize(rgb, (resized_w, resized_h))
    return result, Letterbox(width, height, resized_w / width, resized_h / height, left, top)


def reverse_boxes(predictions, geometry):
    """Nx6 [x1,y1,x2,y2,score,class] → 원본 좌표. padding-only 박스는 제거."""
    data = np.asarray(predictions, dtype=float)
    if data.size == 0:
        if data.shape not in ((0,), (0, 6)):
            raise ValueError('검출 결과는 N×6이어야 합니다')
        return np.empty((0, 6), dtype=float)
    if data.ndim != 2 or data.shape[1] != 6 or not np.isfinite(data).all():
        raise ValueError('검출 결과는 유한한 N×6이어야 합니다')
    if (np.any((data[:, 4] < 0) | (data[:, 4] > 1))
            or np.any(data[:, 5] != np.floor(data[:, 5]))
            or np.any((data[:, 5] < 0) | (data[:, 5] >= len(COCO_CLASSES)))):
        raise ValueError('검출 confidence/class ID가 유효하지 않습니다')
    if np.any(data[:, 2:4] < data[:, :2]):
        raise ValueError('박스 좌표 순서가 유효하지 않습니다')
    data = data.copy()
    data[:, [0, 2]] = np.clip((data[:, [0, 2]] - geometry.left) / geometry.scale_x, 0, geometry.width)
    data[:, [1, 3]] = np.clip((data[:, [1, 3]] - geometry.top) / geometry.scale_y, 0, geometry.height)
    return data[(data[:, 2] > data[:, 0]) & (data[:, 3] > data[:, 1])]


def class_aware_nms(boxes, scores, classes, score_threshold, iou_threshold):
    """OpenCV 4.5.x의 Nx1/1D NMS index 반환 형식을 모두 처리한다."""
    kept = []
    for label in np.unique(classes):
        candidates = np.flatnonzero(classes == label)
        xywh = boxes[candidates].copy()
        xywh[:, 2:4] -= xywh[:, 0:2]
        indices = cv2.dnn.NMSBoxes(xywh.tolist(), scores[candidates].tolist(),
                                   score_threshold, iou_threshold)
        if indices is not None and len(indices):
            kept.extend(candidates[np.asarray(indices, dtype=int).reshape(-1)].tolist())
    return np.asarray(sorted(kept, key=lambda index: (-scores[index], index)), dtype=int)


class CpuNanoDet(NanoDet):
    """상류 전처리는 유지하고 출력 검증·안정적 softmax·클래스별 NMS를 추가."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # 2022nov FP32 모델은 8/16/32 세 레벨(출력 6개)이다.
        # 상류 클래스는 여분의 stride=64 anchors를 만들지만 zip에서 쓰지 않는다.
        self.strides = (8, 16, 32)
        self.anchors_mlvl = self.anchors_mlvl[:3]

    def post_process(self, predictions):
        if len(predictions) != 2 * len(self.strides):
            raise ValueError('NanoDet의 stride별 score/bbox 출력 tensor 쌍이 필요합니다')
        boxes, scores = [], []
        for stride, anchors, cls, distance in zip(
                self.strides, self.anchors_mlvl, predictions[::2], predictions[1::2]):
            cls, distance = np.asarray(cls), np.asarray(distance)
            if cls.ndim == 3 and cls.shape[0] == 1:
                cls = cls[0]
            if distance.ndim == 3 and distance.shape[0] == 1:
                distance = distance[0]
            if cls.shape != (len(anchors), 80) or distance.shape != (len(anchors), 32):
                raise ValueError('NanoDet 출력 tensor shape 불일치')
            if (not np.isfinite(cls).all() or not np.isfinite(distance).all()
                    or np.any((cls < 0) | (cls > 1))):
                raise ValueError('NanoDet 출력 값 불량')
            logits = distance.reshape(-1, 8).astype(np.float64)
            exponent = np.exp(logits - logits.max(axis=1, keepdims=True))
            distances = ((exponent / exponent.sum(axis=1, keepdims=True)) @ self.project).reshape(-1, 4) * stride
            best = np.argsort(cls.max(axis=1))[::-1][:1000]
            points, distances, cls = anchors[best], distances[best], cls[best]
            boxes.append(np.clip(np.column_stack((points - distances[:, :2],
                                                   points + distances[:, 2:])), 0, 416))
            scores.append(cls)
        boxes, all_scores = np.concatenate(boxes), np.concatenate(scores)
        classes = all_scores.argmax(axis=1)
        confidence = all_scores.max(axis=1)
        keep = class_aware_nms(boxes, confidence, classes, self.prob_threshold, self.iou_threshold)
        return np.column_stack((boxes[keep], confidence[keep], classes[keep]))


class NanoDetDetector:
    def __init__(self, model_path, expected_sha256='', confidence=0.35, iou=0.6):
        if not model_path:
            raise ValueError('model_path가 필요합니다. 색상 검출로 대체하지 않습니다')
        self.model_path = Path(model_path).expanduser().resolve()
        if not self.model_path.is_file():
            raise ValueError('NanoDet ONNX 파일이 없습니다: ' + str(self.model_path))
        if not all(math.isfinite(v) and 0 < v < 1 for v in (confidence, iou)):
            raise ValueError('confidence와 iou는 0~1 사이여야 합니다')
        if expected_sha256 and not re.fullmatch(r'[0-9a-fA-F]{64}', expected_sha256):
            raise ValueError('expected_sha256은 64자리 hex여야 합니다')
        digest = hashlib.sha256()
        with self.model_path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
        self.sha256 = digest.hexdigest()
        if expected_sha256 and self.sha256 != expected_sha256.lower():
            raise ValueError('NanoDet 모델 SHA-256 불일치')
        self.confidence, self.iou = confidence, iou
        self.backend = CpuNanoDet(str(self.model_path), confidence, iou,
                                  cv2.dnn.DNN_BACKEND_OPENCV, cv2.dnn.DNN_TARGET_CPU)

    def infer(self, rgb):
        padded, geometry = letterbox(rgb)
        # OpenCV Zoo 평균/표준편차는 BGR 순서이다.
        predictions = self.backend.infer(cv2.cvtColor(padded, cv2.COLOR_RGB2BGR))
        return reverse_boxes(predictions, geometry)

    def metadata(self):
        return {'model_path': str(self.model_path), 'model_sha256': self.sha256,
                'backend': 'opencv_dnn_cpu', 'opencv_version': cv2.__version__,
                'input_size': [416, 416], 'confidence_threshold': self.confidence,
                'iou_threshold': self.iou, 'classes': 'COCO80', 'nms': 'class_aware'}


def detections_json(predictions):
    return [{'class_id': COCO_CLASSES[int(row[5])], 'class_index': int(row[5]),
             'score': float(row[4]), 'bbox_xyxy': row[:4].tolist()} for row in predictions]


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', required=True, type=Path)
    parser.add_argument('--model', required=True, type=Path)
    parser.add_argument('--expected-sha256', default='')
    parser.add_argument('--output', required=True, type=Path, help='새 JSON 파일; 기존 파일 거부')
    options = parser.parse_args(args)
    if options.output.exists():
        parser.error('기존 출력 파일을 덮어쓰지 않습니다')
    try:
        bgr = cv2.imread(str(options.image), cv2.IMREAD_COLOR)
        if bgr is None:
            raise ValueError('입력 영상을 읽을 수 없습니다')
        detector = NanoDetDetector(options.model, options.expected_sha256)
        start = time.perf_counter()
        predictions = detector.infer(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
        report = {**detector.metadata(), 'image': str(options.image.resolve()),
                  'image_size': [bgr.shape[1], bgr.shape[0]],
                  'inference_wall_sec': time.perf_counter() - start,
                  'detections': detections_json(predictions)}
        with options.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
            stream.write('\n')
    except (OSError, ValueError, cv2.error) as exc:
        parser.exit(1, f'NanoDet 추론 실패: {exc}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
