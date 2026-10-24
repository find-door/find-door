"""저장된 trinary 점유 지도와 독립적인 world 선분 정답을 비교한다."""

import hashlib
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import yaml


def _finite_number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name}: 유한한 숫자가 필요합니다')
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{name}: 유한한 숫자가 필요합니다')
    return result


def _pose(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f'{name}: [x, y, yaw] 세 값이 필요합니다')
    return [_finite_number(number, name) for number in value]


def _load_yaml(path):
    try:
        with path.open(encoding='utf-8') as stream:
            value = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f'YAML을 해석할 수 없습니다: {path}') from exc
    if not isinstance(value, dict):
        raise ValueError(f'YAML 최상위는 mapping이어야 합니다: {path}')
    return value


def transform_points(points, pose):
    """점에 source→target 평면 강체 변환을 적용한다. 스케일 보정 없음."""
    x, y, yaw = _pose(pose, '변환')
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return np.asarray(points, dtype=float) @ rotation.T + [x, y]


def load_occupied_map(map_path):
    """PGM 위→아래 행을 뒤집고 cell center를 map 프레임으로 변환한다."""
    map_path = Path(map_path).resolve()
    metadata = _load_yaml(map_path)
    if metadata.get('mode', 'trinary') != 'trinary':
        raise ValueError('지도 mode는 trinary만 지원합니다')
    required = ('image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh')
    if any(key not in metadata for key in required):
        raise ValueError(f'지도 YAML에 필요한 필드가 없습니다: {required}')
    resolution = _finite_number(metadata['resolution'], 'resolution')
    if resolution <= 0:
        raise ValueError('resolution은 양수여야 합니다')
    origin = _pose(metadata['origin'], 'origin')
    negate = metadata['negate']
    if type(negate) is not int or negate not in (0, 1):
        raise ValueError('negate는 정수 0 또는 1이어야 합니다')
    occupied = _finite_number(metadata['occupied_thresh'], 'occupied_thresh')
    free = _finite_number(metadata['free_thresh'], 'free_thresh')
    if not 0 <= free < occupied <= 1:
        raise ValueError('0 <= free_thresh < occupied_thresh <= 1이어야 합니다')
    if not isinstance(metadata['image'], str) or not metadata['image']:
        raise ValueError('image에는 PGM 경로가 필요합니다')
    image_path = (map_path.parent / metadata['image']).resolve()
    if image_path.suffix.lower() != '.pgm':
        raise ValueError('지도 이미지는 8비트 PGM(.pgm)만 지원합니다')
    with Image.open(image_path) as image:
        if image.format != 'PPM' or image.mode != 'L':
            raise ValueError('지도 이미지는 8비트 회색조 PGM이어야 합니다')
        pixels = np.asarray(image, dtype=np.uint8)
    probability = pixels.astype(float) / 255.0
    if not negate:
        probability = 1.0 - probability
    occupied_mask = probability > occupied
    free_mask = probability < free
    rows, columns = np.nonzero(occupied_mask)
    if not len(rows):
        raise ValueError('지도에 occupied 셀이 없습니다 (빈 지도 또는 전부 unknown/free)')
    height, width = pixels.shape
    local = np.column_stack(((columns + 0.5) * resolution,
                             (height - rows - 0.5) * resolution))
    points = transform_points(local, origin)
    details = {
        'yaml_path': str(map_path), 'image_path': str(image_path),
        'width': width, 'height': height, 'resolution_m': resolution,
        'origin': origin, 'mode': 'trinary', 'negate': negate,
        'occupied_thresh': occupied, 'free_thresh': free,
        'occupied_cells': int(occupied_mask.sum()),
        'free_cells': int(free_mask.sum()),
        'unknown_cells': int((~(occupied_mask | free_mask)).sum()),
    }
    return points, details


def load_truth(truth_path):
    truth = _load_yaml(Path(truth_path))
    if truth.get('frame') != 'world':
        raise ValueError('정답 frame은 world여야 합니다')
    raw = truth.get('segments')
    if not isinstance(raw, list) or not raw:
        raise ValueError('정답 segments는 비어 있지 않은 선분 목록이어야 합니다')
    segments = []
    for index, segment in enumerate(raw):
        if not isinstance(segment, list) or len(segment) != 4:
            raise ValueError(f'segments[{index}]는 [x1,y1,x2,y2]여야 합니다')
        values = [_finite_number(value, f'segments[{index}]') for value in segment]
        if values[:2] == values[2:]:
            raise ValueError(f'segments[{index}]의 길이가 0입니다')
        segments.append(values)
    return np.asarray(segments, dtype=float)


def distances_to_segments(points, segments):
    """각 점에서 가장 가까운 유한 선분까지 거리. 끝점 밖의 투영은 clamp."""
    starts = segments[:, :2]
    vectors = segments[:, 2:] - starts
    squared_lengths = np.sum(vectors ** 2, axis=1)
    distances = []
    for offset in range(0, len(points), 2048):
        displacement = points[offset:offset + 2048, None, :] - starts[None, :, :]
        fractions = np.sum(displacement * vectors, axis=2) / squared_lengths
        fractions = np.clip(fractions, 0.0, 1.0)
        residuals = displacement - fractions[:, :, None] * vectors
        distances.append(np.sqrt(np.min(np.sum(residuals ** 2, axis=2), axis=1)))
    return np.concatenate(distances)


def sample_segments(segments, maximum_spacing):
    """각 선분 양끝을 포함해 최대 간격 이하로 균등 샘플링한다."""
    samples = []
    for segment in segments:
        start, end = segment[:2], segment[2:]
        intervals = max(1, int(math.ceil(np.linalg.norm(end - start) / maximum_spacing)))
        samples.append(np.linspace(start, end, intervals + 1))
    return np.concatenate(samples)


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def evaluate_map(map_path, truth_path, map_to_world, tolerance=0.1, sample_spacing=0.025):
    """명시된 강체 변환만 사용해 지도 정확도와 정답 관측 범위를 계산한다."""
    transform = _pose(map_to_world, 'map_to_world')
    tolerance = _finite_number(tolerance, 'tolerance')
    sample_spacing = _finite_number(sample_spacing, 'sample_spacing')
    if tolerance <= 0 or sample_spacing <= 0:
        raise ValueError('tolerance와 sample_spacing은 양수여야 합니다')
    points, metadata = load_occupied_map(map_path)
    segments = load_truth(truth_path)
    world_points = transform_points(points, transform)
    forward = distances_to_segments(world_points, segments)
    truth_samples = sample_segments(segments, sample_spacing)
    backward, _ = cKDTree(world_points).query(truth_samples)
    return {
        'schema_version': 1,
        'method': 'occupied_cell_centers_to_finite_segments',
        'alignment': 'explicit_rigid_transform_no_fitting',
        'distance_unit': 'm',
        'map_to_world': transform,
        'tolerance_m': tolerance,
        'truth_maximum_sample_spacing_m': sample_spacing,
        'inputs': {
            'map_yaml': {'path': metadata['yaml_path'], 'sha256': _sha256(map_path)},
            'map_image': {'path': metadata['image_path'],
                          'sha256': _sha256(metadata['image_path'])},
            'truth_yaml': {'path': str(Path(truth_path).resolve()),
                           'sha256': _sha256(truth_path)},
        },
        'map': metadata,
        'truth_frame': 'world',
        'truth_segment_count': int(len(segments)),
        'truth_sample_count': int(len(truth_samples)),
        'occupied_to_truth': {
            'mean_m': float(np.mean(forward)),
            'rmse_m': float(np.sqrt(np.mean(forward ** 2))),
            'p95_m': float(np.percentile(forward, 95)),
            'max_m': float(np.max(forward)),
            'precision_at_tolerance': float(np.mean(forward <= tolerance)),
        },
        'truth_to_occupied': {
            'coverage_at_tolerance': float(np.mean(backward <= tolerance)),
            'mean_m': float(np.mean(backward)),
            'max_m': float(np.max(backward)),
        },
    }
