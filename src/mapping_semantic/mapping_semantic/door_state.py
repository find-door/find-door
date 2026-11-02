"""출입구 ROI의 깊이를 주변 벽 평면과 비교하는 미검증 상태 후보.

ROI는 문짝 박스가 아닌 출입구 전체이다. 정답 pose나 문 배치 각도를 받지 않는다.
기본 임계값은 최초 후보 설정이며 실제 자료 검증으로 채택된 값이 아니다.
"""

import math
import numpy as np


DEFAULT_CONFIG = {
    'minimum_depth_m': .2, 'maximum_depth_m': 10.,
    'closed_tolerance_m': .08, 'open_distance_m': .2,
    'strip_width_fraction': .3, 'upper_height_fraction': .65,
    'interior_inset_fraction': .15, 'minimum_samples': 60,
    'minimum_valid_fraction': .5, 'plane_tolerance_m': .025,
    'minimum_plane_inlier_fraction': .75, 'minimum_normal_z': .5,
    'maximum_normal_y': .35, 'minimum_open_fraction': .2,
    'minimum_closed_fraction': .85, 'maximum_foreground_fraction': .1,
    'maximum_plane_points': 1500, 'ransac_iterations': 64,
}


def _config(value):
    if value is not None and not isinstance(value, dict):
        raise ValueError('config는 dict여야 합니다')
    if value and set(value) - set(DEFAULT_CONFIG):
        raise ValueError('알 수 없는 config 키')
    result = {**DEFAULT_CONFIG, **(value or {})}
    for key, number in result.items():
        if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or number <= 0:
            raise ValueError(f'유한한 양수 설정 필요: {key}')
        if key in ('minimum_samples', 'maximum_plane_points', 'ransac_iterations'):
            if not isinstance(number, int) or number < 3:
                raise ValueError(f'3 이상 정수 설정 필요: {key}')
        elif 'fraction' in key or key in ('minimum_normal_z', 'maximum_normal_y'):
            if number >= 1:
                raise ValueError(f'0과 1 사이 설정 필요: {key}')
    if (result['minimum_depth_m'] >= result['maximum_depth_m']
            or result['closed_tolerance_m'] >= result['open_distance_m']
            or result['interior_inset_fraction'] >= .5
            or result['maximum_plane_points'] < result['minimum_samples']
            or result['minimum_open_fraction'] + result['minimum_closed_fraction'] <= 1):
        raise ValueError('서로 모순되는 설정 경계')
    return result


def _points(depth, k, mask):
    rows, cols = np.nonzero(mask)
    z = depth[rows, cols].astype(float)
    return np.column_stack(((cols-k[0, 2])*z/k[0, 0],
                            (rows-k[1, 2])*z/k[1, 1], z))


def _fit_plane(points, config):
    """고정 seed RANSAC 후 inlier SVD. 수직 벽 방향 후보만 허용한다."""
    if len(points) > config['maximum_plane_points']:
        points = points[np.linspace(0, len(points)-1, config['maximum_plane_points'], dtype=int)]
    rng = np.random.default_rng(0)
    best = None
    best_count = 0
    for _ in range(config['ransac_iterations']):
        a, b, c = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(b-a, c-a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        if normal[2] < 0:
            normal = -normal
        if normal[2] < config['minimum_normal_z'] or abs(normal[1]) > config['maximum_normal_y']:
            continue
        inliers = np.abs((points-a) @ normal) <= config['plane_tolerance_m']
        count = int(inliers.sum())
        if count > best_count:
            best, best_count = inliers, count
    if best is None or best_count < config['minimum_samples']:
        return None
    center = points[best].mean(axis=0)
    _, singular, vectors = np.linalg.svd(points[best]-center, full_matrices=False)
    if singular[1] < 1e-6:
        return None
    normal = vectors[-1]
    if normal[2] < 0:
        normal = -normal
    offset = -float(center @ normal)
    residual = np.abs(points @ normal + offset)
    fraction = float(np.mean(residual <= config['plane_tolerance_m']))
    if (fraction < config['minimum_plane_inlier_fraction']
            or normal[2] < config['minimum_normal_z'] or abs(normal[1]) > config['maximum_normal_y']):
        return None
    return normal, offset, fraction


def estimate_state(depth_m, K, doorway_bbox, config=None):
    """미터 depth·무왜곡 K·영상 내부 xyxy 출입구 ROI로 상태와 근거를 반환한다."""
    cfg = _config(config)
    if (not isinstance(depth_m, np.ndarray) or depth_m.ndim != 2
            or depth_m.dtype.kind != 'f' or min(depth_m.shape) < 2):
        raise ValueError('depth_m은 2차원 부동소수 미터 배열이어야 합니다')
    try:
        k = np.asarray(K, dtype=float)
        box = np.asarray(doorway_bbox, dtype=float)
    except (ValueError, TypeError) as exc:
        raise ValueError('K 또는 bbox 수치 형식 오류') from exc
    if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1])+abs(k[1, 0]) > 1e-9):
        raise ValueError('무왜곡 pinhole K 필요')
    h, w = depth_m.shape
    if (box.shape != (4,) or not np.isfinite(box).all()
            or not 0 <= box[0] < box[2] <= w or not 0 <= box[1] < box[3] <= h):
        raise ValueError('영상 안의 양수 면적 xyxy 출입구 bbox 필요')
    x0, y0, x1, y1 = box
    width, height = x1-x0, y1-y0
    ys, xs = np.indices((h, w))
    strip = max(3., width*cfg['strip_width_fraction'])
    upper = (ys >= y0) & (ys < y0+height*cfg['upper_height_fraction'])
    wall = upper & (((xs >= x0-strip) & (xs < x0)) | ((xs >= x1) & (xs < x1+strip)))
    # 영상 상단에 여유가 있으면 인방 위 벽도 사용한다.
    wall |= ((ys >= y0-min(strip, height*.2)) & (ys < y0)
             & (xs >= x0) & (xs < x1))
    inset = width*cfg['interior_inset_fraction']
    interior = upper & (xs >= x0+inset) & (xs < x1-inset)
    valid = np.isfinite(depth_m) & (depth_m >= cfg['minimum_depth_m']) & (depth_m <= cfg['maximum_depth_m'])
    evidence = {'wall_pixels': int(wall.sum()), 'interior_pixels': int(interior.sum()),
                'wall_valid_pixels': int((wall & valid).sum()),
                'interior_valid_pixels': int((interior & valid).sum())}

    def result(state, reason):
        return {'state': state, 'reason': reason, 'evidence': evidence, 'config': cfg.copy()}

    for name, mask in (('wall', wall), ('interior', interior)):
        count = int(mask.sum())
        fraction = float((mask & valid).sum()/count) if count else 0.
        evidence[f'{name}_valid_fraction'] = fraction
        if count < cfg['minimum_samples'] or (mask & valid).sum() < cfg['minimum_samples'] or fraction < cfg['minimum_valid_fraction']:
            return result('unknown', f'insufficient_{name}_depth')
    plane = _fit_plane(_points(depth_m, k, wall & valid), cfg)
    if plane is None:
        return result('unknown', 'wall_plane_unreliable')
    normal, offset, fraction = plane
    residual = _points(depth_m, k, interior & valid) @ normal + offset
    near = float(np.mean(np.abs(residual) <= cfg['closed_tolerance_m']))
    behind = float(np.mean(residual >= cfg['open_distance_m']))
    foreground = float(np.mean(residual < -cfg['closed_tolerance_m']))
    evidence.update(wall_plane_normal=normal.tolist(), wall_plane_offset_m=offset,
                    wall_plane_inlier_fraction=fraction, near_wall_fraction=near,
                    behind_wall_fraction=behind, foreground_fraction=foreground,
                    residual_quantiles_m=np.quantile(residual, [.1, .5, .9]).tolist())
    if foreground > cfg['maximum_foreground_fraction']:
        return result('unknown', 'foreground_occlusion')
    if behind >= cfg['minimum_open_fraction']:
        return result('open', 'depth_behind_wall')
    if near >= cfg['minimum_closed_fraction']:
        return result('closed', 'depth_near_wall')
    return result('unknown', 'ambiguous_depth')
