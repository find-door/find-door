"""실물 설정 계약. 미확인 template은 실행할 수 없다."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re

import yaml

TOPIC_TYPES = {'scan': 'sensor_msgs/msg/LaserScan', 'odom': 'nav_msgs/msg/Odometry',
               'rgb': 'sensor_msgs/msg/Image', 'depth': 'sensor_msgs/msg/Image',
               'camera_info': 'sensor_msgs/msg/CameraInfo',
               'detections': 'vision_msgs/msg/Detection2DArray'}
KEYS = {'schema_version', 'robot_id', 'hardware_confirmed', 'frames', 'topics',
        'thresholds', 'slam', 'semantic'}


def finite(value, positive=True):
    return (isinstance(value, (float, int)) and not isinstance(value, bool)
            and math.isfinite(value) and (value > 0 if positive else value >= 0))


def validate_profile(profile, allow_unconfirmed=False, check_files=True, require_camera=False):
    """반환값은 오류 목록. allow_unconfirmed도 미정 값을 실행 값으로 보충하지 않는다."""
    errors = []
    if not isinstance(profile, dict):
        return ['profile은 사전이어야 합니다']
    if set(profile) != KEYS:
        return ['profile 필수/허용 key 불일치']
    if type(profile.get('schema_version')) is not int or profile['schema_version'] != 1:
        errors.append('schema_version은 1이어야 합니다')
    if profile.get('hardware_confirmed') is not True and not allow_unconfirmed:
        errors.append('hardware_confirmed=true로 실제 확인해야 합니다')
    if type(profile.get('hardware_confirmed')) is not bool:
        errors.append('hardware_confirmed는 boolean이어야 합니다')
    def name(value):
        return (isinstance(value, str) and 'REPLACE_ME' not in value
                and bool(re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', value)))
    if not name(profile.get('robot_id')):
        errors.append('robot_id는 확인된 식별자여야 합니다')
    groups = {'frames': {'map', 'odom', 'base', 'laser', 'optical'},
              'topics': set(TOPIC_TYPES),
              'thresholds': {'min_scan_hz', 'min_odom_hz', 'max_age_sec'},
              'slam': {'resolution', 'min_laser_range', 'max_laser_range', 'do_loop_closing'},
              'semantic': {'enabled', 'map_id', 'model_path', 'model_sha256', 'output_path',
                           'sync_policy', 'max_sync_skew_sec', 'aligned_depth_confirmed',
                           'calibration_id', 'registration_note', 'static_classes'}}
    structure_invalid = False
    for group, keys in groups.items():
        if not isinstance(profile.get(group), dict) or set(profile[group]) != keys:
            errors.append(group + ' 필수/허용 key 불일치')
            structure_invalid = True
    if structure_invalid:
        return errors
    camera_required = require_camera or profile['semantic'].get('enabled') is True
    frames = profile['frames']
    for key, value in frames.items():
        if key == 'optical' and value is None and not camera_required:
            continue
        if (not isinstance(value, str) or 'REPLACE_ME' in value or not
                re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*', value)):
            errors.append('유효한 상대 frame 필요: ' + key)
    if len(set(str(v) for v in frames.values() if v is not None)) != sum(v is not None for v in frames.values()):
        errors.append('frame 중복')
    topics = profile['topics']
    for key, value in topics.items():
        if key not in ('scan', 'odom') and value is None and not camera_required:
            continue
        if (not isinstance(value, str) or 'REPLACE_ME' in value or not
                re.fullmatch(r'/[A-Za-z][A-Za-z0-9_]*(/[A-Za-z][A-Za-z0-9_]*)*', value)):
            errors.append('유효한 절대 topic 필요: ' + key)
        elif (any(part in {'cmd_vel', 'cmd_vel_raw', 'cmd_vel_smoothed', 'command', 'commands'}
                  or part.startswith('cmd_') for part in value.split('/'))
              or value in {'/tf', '/tf_static', '/clock', '/map'}):
            errors.append('명령/예약 topic을 센서 입력으로 사용할 수 없습니다: ' + value)
    if len(set(str(v) for v in topics.values() if v is not None)) != sum(v is not None for v in topics.values()):
        errors.append('topic 중복')
    for key, value in profile['thresholds'].items():
        if not finite(value):
            errors.append('유한한 양수 threshold 필요: ' + key)
    slam = profile['slam']
    for key in ('resolution', 'max_laser_range'):
        if not finite(slam.get(key)):
            errors.append('유한한 양수 slam 값 필요: ' + key)
    if not finite(slam.get('min_laser_range'), False):
        errors.append('min_laser_range는 유한한 0 이상 값이어야 합니다')
    elif finite(slam.get('max_laser_range')) and slam['min_laser_range'] >= slam['max_laser_range']:
        errors.append('laser min/max 순서 오류')
    if slam.get('do_loop_closing') is not False:
        errors.append('첫 실물 profile은 do_loop_closing=false만 허용합니다')
    semantic = profile['semantic']
    if type(semantic.get('enabled')) is not bool:
        errors.append('semantic.enabled는 boolean이어야 합니다')
    if semantic.get('sync_policy') not in ('exact', 'approximate'):
        errors.append('semantic sync_policy는 exact/approximate여야 합니다')
    if not finite(semantic.get('max_sync_skew_sec')) or semantic['max_sync_skew_sec'] > 0.1:
        errors.append('max_sync_skew_sec는 0초 초과 0.1초 이하여야 합니다')
    if type(semantic.get('aligned_depth_confirmed')) is not bool:
        errors.append('aligned_depth_confirmed는 boolean이어야 합니다')
    if semantic.get('aligned_depth_confirmed') is True:
        for key in ('calibration_id', 'registration_note'):
            value = semantic.get(key)
            if not isinstance(value, str) or not value.strip() or 'REPLACE_ME' in value:
                errors.append('정렬 확인 근거가 필요합니다: ' + key)
    classes = semantic.get('static_classes')
    if (not isinstance(classes, list) or not classes or any(
            not isinstance(c, str) or c not in ('chair', 'cup', 'bottle') for c in classes)
            or len(set(classes)) != len(classes)):
        errors.append('static_classes는 chair/cup/bottle의 중복 없는 부분집합이어야 합니다')
    if semantic.get('enabled') is True:
        if semantic.get('aligned_depth_confirmed') is not True:
            errors.append('semantic 실행은 RGB-depth 정렬 실측 확인이 필요합니다')
        if not name(semantic.get('map_id')):
            errors.append('semantic map_id를 명시해야 합니다')
        sha = semantic.get('model_sha256')
        if not isinstance(sha, str) or not re.fullmatch('[0-9a-f]{64}', sha):
            errors.append('모델 SHA-256 필요')
        for key in ('model_path', 'output_path'):
            value = semantic.get(key)
            if not isinstance(value, str) or 'REPLACE_ME' in value or not Path(value).is_absolute():
                errors.append('semantic 절대 경로 필요: ' + key)
        if check_files and isinstance(semantic.get('model_path'), str):
            model = Path(semantic['model_path'])
            if not model.is_file():
                errors.append('모델 파일이 없습니다')
            elif hashlib.sha256(model.read_bytes()).hexdigest() != sha:
                errors.append('모델 SHA-256 불일치')
        if check_files and isinstance(semantic.get('output_path'), str):
            output = Path(semantic['output_path'])
            if output.exists() or not output.parent.is_dir():
                errors.append('semantic 출력은 존재하는 부모 폴더의 새 파일이어야 합니다')
    return errors


def load_profile(path, allow_unconfirmed=False, require_camera=False):
    path = Path(path).expanduser().resolve()
    raw = path.read_bytes()
    profile = yaml.safe_load(raw)
    issues = validate_profile(profile, allow_unconfirmed, require_camera=require_camera)
    if issues:
        raise ValueError('; '.join(issues))
    return profile, hashlib.sha256(raw).hexdigest()


def slam_parameters(profile):
    return {'use_sim_time': False, 'odom_frame': profile['frames']['odom'],
            'map_frame': profile['frames']['map'], 'base_frame': profile['frames']['base'],
            'scan_topic': profile['topics']['scan'], 'mode': 'mapping',
            **profile['slam'], 'transform_publish_period': 0.05,
            'map_update_interval': 1.0, 'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0, 'use_scan_matching': True,
            'enable_interactive_mode': False}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', required=True)
    parser.add_argument('--allow-unconfirmed', action='store_true',
                        help='확인 flag만 생략하는 오프라인 검사; 미정 값은 계속 거부')
    options = parser.parse_args(args)
    try:
        _, sha = load_profile(options.profile, options.allow_unconfirmed)
        print(json.dumps({'valid': True, 'profile_sha256': sha,
                          'hardware_confirmation_required': options.allow_unconfirmed}, ensure_ascii=False))
        return 0
    except (ValueError, OSError, yaml.YAMLError) as exc:
        print('프로파일 오류: ' + str(exc))
        return 1
