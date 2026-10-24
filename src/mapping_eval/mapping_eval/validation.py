"""ROS 런타임 없이 시험할 수 있는 메시지 계약 검사."""

import math


def stamp_ns(stamp):
    """builtin_interfaces/Time 형식의 시각을 나노초로 변환한다."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def validate_scan(message):
    errors = []
    if message.header.frame_id != 'lidar_sim_frame':
        errors.append('/scan frame_id는 lidar_sim_frame이어야 합니다')
    if stamp_ns(message.header.stamp) <= 0:
        errors.append('/scan 타임스탬프가 양수가 아닙니다')
    if not message.ranges:
        errors.append('/scan ranges가 비어 있습니다')
    if not (math.isfinite(message.range_min)
            and math.isfinite(message.range_max)
            and 0 <= message.range_min < message.range_max):
        errors.append('/scan 측정 범위가 유효하지 않습니다')
    elif not any(math.isfinite(value)
                 and message.range_min <= value <= message.range_max
                 for value in message.ranges):
        errors.append('/scan에 유효한 실내 거리 측정값이 없습니다')
    if not math.isfinite(message.angle_increment) or message.angle_increment <= 0:
        errors.append('/scan angle_increment가 유효하지 않습니다')
    return errors


def validate_odom(message):
    errors = []
    if message.header.frame_id != 'odom':
        errors.append('/odom header.frame_id는 odom이어야 합니다')
    if message.child_frame_id != 'base_footprint':
        errors.append('/odom child_frame_id는 base_footprint이어야 합니다')
    if stamp_ns(message.header.stamp) <= 0:
        errors.append('/odom 타임스탬프가 양수가 아닙니다')
    return errors


def validate_odom_age(message, latest_clock_ns):
    """반복 발행된 과거 odom과 clock보다 미래인 odom을 거부한다."""
    if latest_clock_ns is None:
        return ['/odom 시각 비교에 필요한 /clock을 아직 받지 못했습니다']
    age_ns = latest_clock_ns - stamp_ns(message.header.stamp)
    if age_ns < 0:
        return ['/odom 타임스탬프가 최신 /clock보다 미래입니다']
    if age_ns >= 1_000_000_000:
        return ['/odom 타임스탬프가 최신 /clock보다 1초 이상 오래되었습니다']
    return []


def validate_map(message):
    errors = []
    if message.header.frame_id != 'map':
        errors.append('/map frame_id는 map이어야 합니다')
    info = message.info
    if not math.isfinite(info.resolution) or info.resolution <= 0:
        errors.append('/map resolution이 유효하지 않습니다')
    if info.width <= 0 or info.height <= 0:
        errors.append('/map 크기가 비어 있습니다')
    if len(message.data) != info.width * info.height:
        errors.append('/map data 길이가 width * height와 다릅니다')
    if any(value < -1 or value > 100 for value in message.data):
        errors.append('/map 점유 값은 -1 또는 0~100이어야 합니다')
    if not any(0 <= value <= 100 for value in message.data):
        errors.append('/map에 관측된 셀이 없습니다')
    return errors
