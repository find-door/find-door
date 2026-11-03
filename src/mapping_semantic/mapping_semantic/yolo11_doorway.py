"""별도 ML 환경에서 로컬 YOLO11 출입구 후보를 유지하는 제한적 검출기.

ROS 의존성과 ML 의존성을 import 시점에 적재하지 않는다. 기본 YOLOE를 바꾸지
않으며 단일 doorway 클래스의 원본 픽셀 박스만 기존 door seed 경계에 연결한다.
"""

import gc
import hashlib
from importlib import metadata
import io
import json
import math
import os
from pathlib import Path
import re
import time

from PIL import Image


FIELDS = {'schema_version', 'backend', 'checkpoint', 'checkpoint_sha256',
          'training_target', 'ultralytics_version', 'device', 'imgsz',
          'confidence', 'iou', 'max_det'}
MAX_PIXELS = 40_000_000


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('중복 설정 키를 허용하지 않습니다: ' + key)
        result[key] = value
    return result


def load_config(path):
    """설정·로컬 자산을 검사하고 독립 설정 사본과 원본 설정 해시를 반환한다."""
    path = Path(path).expanduser().resolve()
    raw = path.read_bytes()
    config = json.loads(raw, object_pairs_hook=_unique_object)
    if not isinstance(config, dict) or set(config) != FIELDS:
        raise ValueError('YOLO11 설정 필수 키·추가 키 오류')
    if (type(config['schema_version']) is not int or config['schema_version'] != 1
            or config['backend'] != 'yolo11' or config['training_target'] != 'doorway'):
        raise ValueError('schema_version=1·backend=yolo11·training_target=doorway가 필요합니다')
    if not isinstance(config['checkpoint'], str) or not config['checkpoint'].strip():
        raise ValueError('로컬 checkpoint 경로가 필요합니다')
    checksum = config['checkpoint_sha256']
    if not isinstance(checksum, str) or re.fullmatch('[0-9a-f]{64}', checksum) is None:
        raise ValueError('checkpoint_sha256은 소문자 SHA-256이어야 합니다')
    if not isinstance(config['ultralytics_version'], str) or not config['ultralytics_version'].strip():
        raise ValueError('고정 ultralytics_version이 필요합니다')
    if config['device'] not in ('0', 'cpu') or not isinstance(config['device'], str):
        raise ValueError('device는 문자열 0 또는 cpu여야 합니다')
    if (type(config['imgsz']) is not int or not 32 <= config['imgsz'] <= 1280
            or config['imgsz'] % 32):
        raise ValueError('imgsz는 32~1280의 32배수 정수여야 합니다')
    if type(config['max_det']) is not int or not 1 <= config['max_det'] <= 300:
        raise ValueError('max_det는 1~300 정수여야 합니다')
    if not _finite(config['confidence']) or not 0 <= config['confidence'] <= 1:
        raise ValueError('confidence는 유한한 [0,1] 값이어야 합니다')
    if not _finite(config['iou']) or not 0 < config['iou'] <= 1:
        raise ValueError('iou는 유한한 (0,1] 값이어야 합니다')
    checkpoint = Path(config['checkpoint']).expanduser()
    if not checkpoint.is_absolute():
        checkpoint = path.parent / checkpoint
    checkpoint = checkpoint.resolve()
    if checkpoint.suffix.lower() != '.pt' or not checkpoint.is_file():
        raise ValueError('로컬 PyTorch .pt checkpoint 파일이 필요합니다')
    if _sha(checkpoint) != checksum:
        raise ValueError('checkpoint 해시 불일치')
    config['checkpoint'] = str(checkpoint)
    return config, hashlib.sha256(raw).hexdigest()


def _check_names(names):
    if (not isinstance(names, dict) or len(names) != 1 or
            any(type(key) is not int or key != 0 or value != 'doorway'
                for key, value in names.items())):
        raise ValueError('checkpoint 클래스는 {0: doorway}여야 합니다')


def _image(data):
    if not isinstance(data, bytes) or not data:
        raise ValueError('비어 있지 않은 원본 RGB PNG 바이트가 필요합니다')
    with Image.open(io.BytesIO(data)) as source:
        if (source.format != 'PNG' or source.mode != 'RGB'
                or getattr(source, 'n_frames', 1) != 1
                or source.getexif().get(274, 1) != 1
                or source.width * source.height > MAX_PIXELS):
            raise ValueError('단일 RGB PNG·EXIF 방향 1·4천만 픽셀 이하 입력이 필요합니다')
        source.load()
        return source.copy()


def extract_detections(results, width, height, confidence, max_det):
    """학습 모델의 잘못된 출력을 빈 성공이나 좌표 보정으로 숨기지 않는다."""
    if not isinstance(results, (list, tuple)) or len(results) != 1:
        raise ValueError('한 영상당 하나의 검출 결과가 필요합니다')
    result = results[0]
    _check_names(result.names)
    boxes = result.boxes.xyxy.cpu().tolist()
    scores = result.boxes.conf.cpu().tolist()
    classes = result.boxes.cls.cpu().tolist()
    if len(boxes) != len(scores) or len(boxes) != len(classes) or len(boxes) > max_det:
        raise ValueError('검출 목록 길이·max_det 계약 오류')
    detections = []
    for box, score, category in zip(boxes, scores, classes):
        if not _finite(category) or category != 0:
            raise ValueError('학습한 단일 doorway 클래스 이외 검출')
        if not _finite(score) or not 0 <= score <= 1:
            raise ValueError('검출 confidence 오류')
        if (not isinstance(box, list) or len(box) != 4 or not all(_finite(x) for x in box)
                or not 0 <= box[0] < box[2] <= width or not 0 <= box[1] < box[3] <= height):
            raise ValueError('원본 영상 내부의 유한한 양수 면적 bbox가 필요합니다')
        if score < confidence:
            continue
        detections.append(dict(detection_id=f'det_{len(detections)+1}',
            **{'class': 'door'}, report_class='door', source_label='doorway', source_class_id=0,
            confidence=float(score), bbox=[float(x) for x in box]))
    return detections


class YOLO11DoorwayDetector:
    """명시한 로컬 checkpoint 한 개를 worker 생명주기 동안 유지한다."""

    def __init__(self, config_path, runtime_directory):
        self.config_path = Path(config_path).expanduser().resolve()
        self.config, self.source_config_sha256 = load_config(self.config_path)
        if metadata.version('ultralytics') != self.config['ultralytics_version']:
            raise ValueError('실제 Ultralytics 설치 버전과 설정이 다릅니다')
        self.runtime_directory = Path(runtime_directory).resolve()
        self.cache_directory = self.runtime_directory / 'ultralytics-config'
        self.cache_directory.mkdir(parents=True, exist_ok=True)
        os.environ['YOLO_CONFIG_DIR'] = str(self.cache_directory)
        os.environ['YOLO_AUTOINSTALL'] = 'false'
        os.environ['YOLO_OFFLINE'] = 'true'
        self.closed = False
        self._model = self._torch = None
        self.last_run = self._identity()
        try:
            self._load_model()
            _check_names(self._model.names)
            self._verify_identity()
        except BaseException:
            try:
                self.close()
            except Exception:
                pass
            raise

    def _load_model(self):
        import torch
        from ultralytics import YOLO, settings
        self._torch = torch
        settings.update({key: False for key in
                         ('sync', 'clearml', 'comet', 'dvc', 'mlflow', 'raytune', 'tensorboard', 'wandb')})
        if self.config['device'] == '0':
            if not torch.cuda.is_available():
                raise RuntimeError('요청한 CUDA 장치 0을 사용할 수 없습니다')
            torch.cuda.set_device(0)
            torch.cuda.init()
        self._model = YOLO(self.config['checkpoint'])

    def _identity(self):
        return dict(model_backend='yolo11', training_target='doorway',
            checkpoint_sha256=self.config['checkpoint_sha256'],
            source_config_sha256=self.source_config_sha256,
            ultralytics_version=self.config['ultralytics_version'],
            torch_version=getattr(self._torch, '__version__', None),
            device=self.config['device'], release_policy='keep',
            cache_override=str(self.cache_directory))

    def _verify_identity(self):
        if _sha(self.config_path) != self.source_config_sha256:
            raise ValueError('실행 중 원본 검출 설정 변경')
        if _sha(self.config['checkpoint']) != self.config['checkpoint_sha256']:
            raise ValueError('실행 중 checkpoint 해시 변경')

    def _synchronize(self):
        if self.config['device'] == '0' and self._torch is not None:
            self._torch.cuda.synchronize(0)

    def detect(self, image_bytes):
        started = time.monotonic()
        self.last_run = self._identity()
        self.last_run['success'] = False
        try:
            if self.closed:
                raise RuntimeError('종료한 검출기는 재사용할 수 없습니다')
            self._verify_identity()
            image = _image(image_bytes)
            self.last_run.update(input_sha256=hashlib.sha256(image_bytes).hexdigest(), input_size=list(image.size),
                                 imgsz=self.config['imgsz'], confidence=self.config['confidence'],
                                 iou=self.config['iou'], max_det=self.config['max_det'])
            self._synchronize()
            result = self._model.predict(source=image, device=self.config['device'],
                imgsz=self.config['imgsz'], conf=self.config['confidence'], iou=self.config['iou'],
                max_det=self.config['max_det'], agnostic_nms=False, half=False, augment=False,
                save=False, show=False, verbose=False, stream=False)
            self._synchronize()
            detections = extract_detections(result, image.width, image.height,
                                             self.config['confidence'], self.config['max_det'])
            self.last_run.update(success=True, detections=len(detections))
            return detections
        except Exception as exc:
            self.last_run['error'] = f'{type(exc).__name__}: {exc}'
            raise
        finally:
            self.last_run['inference_wall_sec'] = time.monotonic() - started

    def close(self):
        if self.closed:
            return
        self.closed = True
        model, self._model = self._model, None
        del model
        gc.collect()
        if self.config['device'] == '0' and self._torch is not None:
            self._torch.cuda.synchronize(0)
            self._torch.cuda.empty_cache()
