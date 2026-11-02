"""추정 출입구 하단 두 광선과 추정 벽 평면의 교점으로 문턱 위치 후보를 구한다."""

import numpy as np


def threshold_point(K, bbox, normal, offset):
    """카메라 좌표 후보. bbox 하단이 문턱에 대응한다는 가정은 호출자가 검증해야 한다."""
    k, box, n = np.asarray(K, float), np.asarray(bbox, float), np.asarray(normal, float)
    if (k.shape != (3, 3) or box.shape != (4,) or n.shape != (3,)
            or not all(np.isfinite(x).all() for x in (k, box, n))
            or not np.isfinite(offset) or k[0, 0] <= 0 or k[1, 1] <= 0
            or not np.allclose(k[2], [0, 0, 1]) or abs(k[0, 1])+abs(k[1, 0]) > 1e-9
            or not 0 <= box[0] < box[2] or not 0 <= box[1] < box[3]
            or not np.isclose(np.linalg.norm(n), 1.)):
        raise ValueError('유효한 K·bbox·단위 벽 평면이 필요합니다')
    inverse = np.linalg.inv(k)
    points = []
    for x in (box[0], box[2]):
        ray = inverse @ [x, box[3], 1.]
        denominator = float(n @ ray)
        if abs(denominator) < 1e-6:
            raise ValueError('광선과 벽 평면이 평행합니다')
        point = ray * (-float(offset) / denominator)
        if not .2 <= point[2] <= 10.:
            raise ValueError('문턱 교점이 카메라 깊이 범위 밖입니다')
        points.append(point)
    return np.mean(points, axis=0).tolist()
