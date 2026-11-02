"""검출로 확인한 출입구 평면 영역을 재투영하고 새 RGB-D 기하와 대조한다.

새 문을 생성하거나 이전 상태를 복제하지 않는다. 지도 epoch 안의 임시 기하다.
"""
import math
import numpy as np

from mapping_semantic.core import finite_vector, rotation_matrix
from mapping_semantic.door_position import threshold_point

CONFIG = dict(minimum_visible_fraction=.5, maximum_anchor_distance_m=.1,
              maximum_plane_distance_m=.08, maximum_normal_angle_rad=math.radians(10),
              maximum_width_relative_error=.2)


def doorway_quad(K, bbox, normal, offset, translation, quaternion):
    """현재 추정 bbox 네 광선과 벽 평면의 교점을 map으로 변환한다."""
    # 문턱 함수와 동일한 pinhole·단위 평면·bbox 계약을 사용한다.
    threshold_point(K, bbox, normal, offset)
    k, box, n = np.asarray(K, float), np.asarray(bbox, float), np.asarray(normal, float)
    t = finite_vector(translation, 3, 'camera translation')
    r = rotation_matrix(quaternion)
    points=[]
    for x,y in [(box[0],box[1]),(box[2],box[1]),(box[2],box[3]),(box[0],box[3])]:
        ray=np.linalg.solve(k,[x,y,1.])
        denominator=float(n @ ray)
        if abs(denominator)<1e-6:
            raise ValueError('출입구 광선과 벽 평면이 평행합니다')
        point=ray*(-float(offset)/denominator)
        if not .2<=point[2]<=10.:
            raise ValueError('출입구 평면 교점이 깊이 범위 밖입니다')
        points.append((r @ point+t).tolist())
    return points


def _quad(points):
    p=np.asarray(points,float)
    if p.shape!=(4,3) or not np.isfinite(p).all():
        raise ValueError('유한한 출입구 네 점이 필요합니다')
    normal=np.cross(p[1]-p[0],p[3]-p[0])
    length=np.linalg.norm(normal)
    if length<1e-6 or np.linalg.norm(p[2]-p[3])<1e-6:
        raise ValueError('퇴화한 출입구 평면입니다')
    normal=normal/length
    if abs(float((p[2]-p[0]) @ normal))>1e-5:
        raise ValueError('출입구 네 점은 같은 평면에 있어야 합니다')
    return p,normal


def project_quad(points, K, translation, quaternion, width, height):
    """map 기하를 현재 영상에 투영한다. 충분히 보이지 않으면 None이다."""
    p,_=_quad(points)
    k=np.asarray(K,float)
    if (k.shape!=(3,3) or not np.isfinite(k).all() or k[0,0]<=0 or k[1,1]<=0
            or not np.allclose(k[2],[0,0,1]) or type(width)is not int or type(height)is not int
            or width<=0 or height<=0):
        raise ValueError('유효한 K와 해상도가 필요합니다')
    t=finite_vector(translation,3,'camera translation')
    camera=(p-t) @ rotation_matrix(quaternion)
    if np.any(camera[:,2]<.2) or np.any(camera[:,2]>10.):return None
    pixels=camera @ k.T
    pixels=pixels[:,:2]/pixels[:,2:]
    raw=[float(pixels[:,0].min()),float(pixels[:,1].min()),float(pixels[:,0].max()),float(pixels[:,1].max())]
    box=[max(0.,raw[0]),max(0.,raw[1]),min(float(width),raw[2]),min(float(height),raw[3])]
    area=(raw[2]-raw[0])*(raw[3]-raw[1])
    if box[2]<=box[0] or box[3]<=box[1] or area<=0:return None
    if (box[2]-box[0])*(box[3]-box[1])/area<CONFIG['minimum_visible_fraction']:return None
    return box


def compatible_quad(cached, current, anchor, position):
    """새 출입구의 위치·평면·폭이 기존 문과 일치할 때만 True."""
    old,n0=_quad(cached);new,n1=_quad(current)
    a=finite_vector(anchor,3,'door anchor');p=finite_vector(position,3,'current door position')
    if np.linalg.norm(a-p)>CONFIG['maximum_anchor_distance_m']:return False
    if abs(float(n0 @ n1))<math.cos(CONFIG['maximum_normal_angle_rad']):return False
    if np.abs((new-old[0]) @ n0).max()>CONFIG['maximum_plane_distance_m']:return False
    old_width=np.linalg.norm(old[2]-old[3]);new_width=np.linalg.norm(new[2]-new[3])
    return bool(abs(new_width/old_width-1)<=CONFIG['maximum_width_relative_error'])
