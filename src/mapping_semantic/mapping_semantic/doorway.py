"""벽보다 돌출된 양쪽 문틀을 색 대비와 깊이로 찾는 제한적 출입구 후보.

수평 카메라·균일한 주변 벽·양쪽 수직 문틀 가정이다. 문틀 없는 문, 평평한
문틀, 가림은 지원하지 않는다. 특정 색이나 world 치수·위치를 사용하지 않는다.
"""

import numpy as np

from mapping_semantic.door_state import DEFAULT_CONFIG, _fit_plane, _points


CONFIG = {
    'border_fraction': .08, 'contrast_rgb_distance': 30.,
    'vertical_support_fraction': .65, 'jamb_protrusion_m': .02,
    'maximum_protrusion_m': .4, 'minimum_jamb_pixels': 3,
    'maximum_jamb_width_fraction': .25, 'minimum_width_pixels': 24,
    'minimum_height_pixels': 30,
}


def _runs(mask):
    boundaries = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.flatnonzero(boundaries == 1), np.flatnonzero(boundaries == -1)))


def estimate_doorway(rgb_uint8, depth_m, K, detection_bbox_xyxy):
    """RGB·정렬 depth·K·문 검출 seed에서 출입구를 추정하거나 근거 부족을 반환한다."""
    rgb, depth = rgb_uint8, depth_m
    if (not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.ndim != 3
            or rgb.shape[2] != 3 or min(rgb.shape[:2]) < 30):
        raise ValueError('RGB uint8 H×W×3 영상 필요')
    if (not isinstance(depth, np.ndarray) or depth.shape != rgb.shape[:2]
            or depth.dtype.kind != 'f'):
        raise ValueError('RGB에 정렬된 부동소수 미터 깊이 필요')
    k = np.asarray(K, dtype=float)
    box = np.asarray(detection_bbox_xyxy, dtype=float)
    h, w = depth.shape
    if (k.shape != (3, 3) or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1])+abs(k[1, 0]) > 1e-9):
        raise ValueError('무왜곡 pinhole K 필요')
    if (box.shape != (4,) or not np.isfinite(box).all()
            or not 0 <= box[0] < box[2] <= w or not 0 <= box[1] < box[3] <= h):
        raise ValueError('영상 내부 양수 면적 검출 xyxy 필요')
    x0, y0, x1, y1 = box
    evidence = {'config': CONFIG.copy(), 'assumptions': [
        'upright_camera', 'uniform_border_wall', 'two_visible_protruding_jambs'],
        'seed_bbox': box.tolist()}

    def unavailable(reason):
        return {'status': 'unavailable', 'reason': reason, 'bbox': None, 'evidence': evidence}

    if y1-y0 < CONFIG['minimum_height_pixels']:
        return unavailable('seed_too_short')
    rows, cols = np.indices((h, w))
    top = int(y0)
    bottom = max(top+1, int(y0 + .65*(y1-y0)))
    upper = (rows >= top) & (rows < bottom)
    valid = np.isfinite(depth) & (depth >= .2) & (depth <= 10.)
    border = upper & ((cols < max(3, w*CONFIG['border_fraction']))
                      | (cols >= w-max(3, w*CONFIG['border_fraction']))) & valid
    if border.sum() < 60:
        return unavailable('insufficient_border_wall')
    wall_color = np.median(rgb[border], axis=0)
    color_distance = np.linalg.norm(rgb.astype(float)-wall_color, axis=2)
    contrast = color_distance >= CONFIG['contrast_rgb_distance']
    wall = border & ~contrast
    if wall.sum() < 60:
        return unavailable('wall_color_unreliable')
    plane = _fit_plane(_points(depth, k, wall), DEFAULT_CONFIG)
    if plane is None:
        return unavailable('wall_plane_unreliable')
    normal, offset, inlier = plane
    evidence.update(wall_color_rgb=wall_color.tolist(), wall_plane_normal=normal.tolist(),
                    wall_plane_offset_m=float(offset), wall_plane_inlier_fraction=float(inlier))
    sample = valid & contrast
    residual = np.full((h, w), np.nan)
    residual[sample] = _points(depth, k, sample) @ normal + offset
    front = ((residual <= -CONFIG['jamb_protrusion_m'])
             & (residual >= -CONFIG['maximum_protrusion_m']))
    support = front[top:bottom].mean(axis=0)
    jambs = [(int(a), int(b)) for a, b in _runs(support >= CONFIG['vertical_support_fraction'])
             if b-a >= CONFIG['minimum_jamb_pixels']]
    evidence['jamb_bands'] = [list(band) for band in jambs]
    # 여러 출입구를 합치지 않고 양쪽 두 문틀만 확인되는 제한적 후보다.
    if len(jambs) != 2:
        return unavailable('two_unambiguous_jambs_required')
    (left, left_end), (right_start, right) = jambs
    width = right-left
    if (width < CONFIG['minimum_width_pixels']
            or max(left_end-left, right-right_start) > width*CONFIG['maximum_jamb_width_fraction']
            or min(x1, right) <= max(x0, left)):
        return unavailable('seed_or_jamb_geometry_inconsistent')
    # 문턱은 두 문틀의 세로 색 대비가 동시에 끝나는 행으로 찾는다.
    band_left = front[:, left:left_end].mean(axis=1)
    band_right = front[:, right_start:right].mean(axis=1)
    both = (band_left >= .65) & (band_right >= .65)
    active = [run for run in _runs(both) if run[0] <= top < run[1]]
    if len(active) != 1:
        return unavailable('jamb_vertical_extent_unreliable')
    y_top, y_bottom = map(int, active[0])
    if y_bottom-y_top < CONFIG['minimum_height_pixels'] or y_bottom <= bottom:
        return unavailable('jamb_vertical_extent_too_short')
    if y_top > 0:
        header_height = max(2, min(left_end-left, right-right_start))
        if contrast[y_top:min(y_top+header_height, h), left:right].mean() < .65:
            return unavailable('visible_header_missing')
    evidence.update(top_clipped=y_top == 0, bottom_clipped=y_bottom == h,
                    boundary_source='rgb_contrast_and_protruding_depth_jambs')
    if y_bottom == h:
        return unavailable('door_bottom_not_visible')
    return {'status': 'estimated', 'reason': 'two_protruding_jambs',
            'bbox': [float(left), float(y_top), float(right), float(y_bottom)], 'evidence': evidence}
