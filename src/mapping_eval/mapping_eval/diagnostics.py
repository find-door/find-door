"""ROS 없이 검사하는 TF 변화·시각화 그래프 진단. 원인 판정은 하지 않는다."""

import math


def stamp_ns(stamp):
    if (not isinstance(stamp.sec, int) or not isinstance(stamp.nanosec, int)
            or stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
        raise ValueError('유효하지 않은 timestamp')
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def finite(values):
    result = [float(value) for value in values]
    if not all(math.isfinite(value) for value in result):
        raise ValueError('비유한 좌표')
    return result


def quaternion_yaw(values):
    q = finite(values)
    if len(q) != 4:
        raise ValueError('quaternion은 4개 값이어야 함')
    norm = math.hypot(*q)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError('quaternion norm이 유효하지 않음')
    x, y, z, w = [value / norm for value in q]
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def transform_record(message, previous=None, transform_timeout=0.2):
    """map→odom 보정 원본과 변화량. 같으면 None, 첫 값은 jump=False."""
    if not math.isfinite(transform_timeout) or transform_timeout < 0:
        raise ValueError('transform_timeout은 유한한 0 이상의 값이어야 함')
    t, q = message.transform.translation, message.transform.rotation
    translation = finite([t.x, t.y, t.z])
    quaternion = finite([q.x, q.y, q.z, q.w])
    yaw = quaternion_yaw(quaternion)
    timestamp = stamp_ns(message.header.stamp)
    distance, angle = None, None
    if previous is not None:
        distance = math.dist(translation, previous['translation'])
        angle = math.atan2(math.sin(yaw - previous['yaw']), math.cos(yaw - previous['yaw']))
        # q와 -q는 같은 자세다. 헤더만 증가하는 반복 TF는 기록하지 않는다.
        norm = math.hypot(*quaternion)
        old = previous['quaternion']
        old_norm = math.hypot(*old)
        same_rotation = min(
            math.dist([v / norm for v in quaternion], [v / old_norm for v in old]),
            math.dist([v / norm for v in quaternion], [-v / old_norm for v in old])) <= 1e-12
        if distance <= 1e-12 and same_rotation:
            return None
    return {
        'header_stamp_ns': timestamp, 'parent_frame': message.header.frame_id,
        'child_frame': message.child_frame_id, 'translation': translation,
        'quaternion': quaternion, 'yaw': yaw, 'translation_delta_m': distance,
        'yaw_delta_rad': angle, 'jump': distance is not None and (distance > 0.5 or abs(angle) > 0.2),
        'source_scan_stamp_estimate_ns': timestamp - round(transform_timeout * 1e9),
        'source_scan_stamp_is_estimate': True,
    }


def parse_graph(message, tolerance=1e-8):
    """노드 좌표와 edge geometry 보존. 좌표 후보는 추론이며 실제 edge ID가 아니다."""
    nodes, edges, markers = [], [], []
    warnings = []
    for marker in message.markers:
        header = {'header_stamp_ns': stamp_ns(marker.header.stamp),
                  'frame_id': marker.header.frame_id, 'ns': marker.ns,
                  'id': marker.id, 'type': marker.type, 'action': marker.action}
        markers.append(header)
        if marker.action != 0:  # ADD만 해석. DELETEALL도 위 메타데이터에 보존.
            continue
        if marker.ns == 'slam_toolbox' and marker.type == 2:  # SPHERE
            p = marker.pose.position
            nodes.append({'marker_id': marker.id, 'position': finite([p.x, p.y, p.z]),
                          'frame_id': marker.header.frame_id})
        elif marker.ns == 'slam_toolbox_edges' and marker.type == 5:  # LINE_LIST
            points = [finite([p.x, p.y, p.z]) for p in marker.points]
            if len(points) % 2:
                warnings.append('odd_edge_point_count')
            pose = marker.pose
            # 실제 2.6.10는 identity pose. 달라지면 geometry 보존하되 ID매칭 금지.
            translation = finite([pose.position.x, pose.position.y, pose.position.z])
            rotation = finite([pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w])
            identity = translation == [0, 0, 0] and rotation in ([0, 0, 0, 1], [0, 0, 0, -1])
            if not identity:
                warnings.append('nonidentity_edge_marker_pose')
            for index in range(0, len(points), 2):
                edges.append({'marker_id': marker.id, 'pair_index': index // 2,
                              'frame_id': marker.header.frame_id, 'points': points[index:index + 2],
                              'marker_translation': translation, 'marker_quaternion': rotation,
                              'can_match': identity})
    for edge in edges:
        edge['endpoint_candidate_ids'] = [
            ([node['marker_id'] for node in nodes
              if node['frame_id'] == edge['frame_id'] and math.dist(point, node['position']) <= tolerance] or None)
            if edge['can_match'] else None for point in edge['points']]
        del edge['can_match']
    return {'nodes': nodes, 'edges': edges, 'markers': markers, 'warnings': warnings,
            'node_count': len(nodes), 'edge_count': sum(len(e['points']) == 2 for e in edges),
            'edge_ids_are_inferred': True, 'node_orientation_is_not_observed': True}
