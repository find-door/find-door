"""주행 CSV의 평면 위치 잔차와 인접 변화량을 분석한다.

Gazebo odom은 SLAM 입력에도 쓰이므로 독립적인 ATE가 아니다.
각도 오차나 루프 클로저 원인 판정은 수행하지 않는다.
"""

import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import re
import sys


COLUMNS = (
    'stamp_ns', 'odom_x', 'odom_y', 'map_x', 'map_y',
    'estimated_world_x', 'estimated_world_y', 'error_m',
)


def _positive_finite(value, name):
    if isinstance(value, bool):
        raise ValueError(f'{name}은 유한한 양수여야 합니다')
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name}은 유한한 양수여야 합니다') from exc
    if not math.isfinite(number) or number <= 0:
        raise ValueError(f'{name}은 유한한 양수여야 합니다')
    return number


def _read_samples(content):
    reader = csv.DictReader(io.StringIO(content))
    headers = reader.fieldnames or []
    if len(headers) != len(set(headers)):
        raise ValueError('CSV에 중복된 열 이름이 있습니다')
    missing = [column for column in COLUMNS if column not in headers]
    if missing:
        raise ValueError('CSV 필수 열 누락: ' + ', '.join(missing))
    samples = []
    previous_stamp = None
    for line, row in enumerate(reader, start=2):
        if None in row or any(value is None for value in row.values()):
            raise ValueError(f'CSV {line}행의 열 개수가 헤더와 다릅니다')
        raw_stamp = row['stamp_ns'].strip()
        if not re.fullmatch(r'[0-9]+', raw_stamp):
            raise ValueError(f'CSV {line}행 stamp_ns는 음수가 아닌 정수여야 합니다')
        stamp = int(raw_stamp)
        # driver의 ROS builtin_interfaces/Time에서 표현 가능한 양수 범위.
        if stamp > 2_147_483_647_999_999_999:
            raise ValueError(f'CSV {line}행 stamp_ns가 ROS 시각 범위를 넘습니다')
        if previous_stamp is not None and stamp <= previous_stamp:
            raise ValueError(f'CSV {line}행 stamp_ns가 관측 순서대로 증가하지 않습니다')
        previous_stamp = stamp
        values = {}
        for column in COLUMNS[1:]:
            try:
                number = float(row[column])
            except ValueError as exc:
                raise ValueError(f'CSV {line}행 {column}은 숫자여야 합니다') from exc
            if not math.isfinite(number):
                raise ValueError(f'CSV {line}행 {column}이 유한하지 않습니다')
            values[column] = number
        if values['error_m'] < 0:
            raise ValueError(f'CSV {line}행 error_m은 음수가 될 수 없습니다')
        residual = (values['estimated_world_x'] - values['odom_x'],
                    values['estimated_world_y'] - values['odom_y'])
        error = math.hypot(*residual)
        if not math.isfinite(error):
            raise ValueError(f'CSV {line}행 위치 잔차 계산이 유한하지 않습니다')
        samples.append((stamp, residual, error))
    if not samples:
        raise ValueError('CSV에 궤적 표본이 없습니다')
    return samples


def analyze_trajectory(path, jump_threshold=0.5, max_gap=0.2):
    """기록 순서를 유지해 위치 잔차와 점프 후보를 계산한다.

    CSV error_m 대신 estimated_world_xy - odom_xy로 잔차를 재계산한다.
    max_gap보다 큰 시간 간격은 점프 후보 판정에서 제외한다.
    """
    jump_threshold = _positive_finite(jump_threshold, 'jump_threshold')
    max_gap = _positive_finite(max_gap, 'max_gap')
    path = Path(path).resolve()
    raw = path.read_bytes()
    samples = _read_samples(raw.decode('utf-8-sig'))
    errors = [sample[2] for sample in samples]
    greatest_gap = 0.0
    gap_count = 0
    adjacent_max = None
    eligible_max = None
    eligible_pairs = 0
    events = []
    for previous, current in zip(samples, samples[1:]):
        gap = (current[0] - previous[0]) / 1_000_000_000
        greatest_gap = max(greatest_gap, gap)
        delta = (current[1][0] - previous[1][0], current[1][1] - previous[1][1])
        step = math.hypot(*delta)
        if not math.isfinite(step):
            raise ValueError('인접 잔차 변화량 계산이 유한하지 않습니다')
        detail = {
            'step_m': step, 'stamp_ns': current[0], 'previous_stamp_ns': previous[0],
            'gap_sec': gap, 'delta_residual_x_m': delta[0], 'delta_residual_y_m': delta[1],
        }
        if adjacent_max is None or step > adjacent_max['step_m']:
            adjacent_max = detail
        if gap > max_gap:
            gap_count += 1
            continue
        eligible_pairs += 1
        if eligible_max is None or step > eligible_max['step_m']:
            eligible_max = detail
        if step >= jump_threshold:
            events.append(detail)
    # hypot은 합산 제곱의 중간 overflow를 피한다.
    rmse = math.hypot(*errors) / math.sqrt(len(errors))
    if not math.isfinite(rmse):
        raise ValueError('위치 잔차 RMSE 계산이 유한하지 않습니다')
    return {
        'schema_version': 1,
        'input': {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()},
        'residual_definition': 'estimated_world_xy - odom_xy; CSV error_m is not used',
        'limitations': 'Gazebo odom is also a SLAM input; not independent ATE; '
                      'no angular error or loop-closure cause attribution',
        'sample_count': len(samples),
        'start_stamp_ns': samples[0][0], 'end_stamp_ns': samples[-1][0],
        'duration_sec': (samples[-1][0] - samples[0][0]) / 1_000_000_000,
        'residual_position': {'rmse_m': rmse, 'max_m': max(errors)},
        'adjacent_residual_step_max': adjacent_max,
        'eligible_residual_step_max': eligible_max,
        'max_gap_sec': greatest_gap, 'gap_count': gap_count,
        'eligible_pair_count': eligible_pairs,
        'jump_threshold_m': jump_threshold,
        'maximum_event_gap_sec': max_gap,
        'jump_event_count': len(events), 'jump_events': events,
    }


def main(args=None):
    parser = argparse.ArgumentParser(description='궤적 CSV의 위치 잔차와 점프 후보 분석')
    parser.add_argument('--trajectory', required=True, help='driver trajectory.csv 경로')
    parser.add_argument('--output', required=True, help='새 JSON 결과 파일')
    parser.add_argument('--jump-threshold', type=float, default=0.5,
                        help='점프 후보 최소 잔차 변화량(m), 기본값 0.5')
    parser.add_argument('--max-gap', type=float, default=0.2,
                        help='이 간격을 넘는 표본 쌍은 점프 판정에서 제외(초)')
    options = parser.parse_args(args)
    try:
        report = analyze_trajectory(options.trajectory, options.jump_threshold, options.max_gap)
        output = Path(options.output).resolve()
        if output == Path(options.trajectory).resolve():
            raise ValueError('출력 파일은 입력 CSV와 달라야 합니다')
        # 기존 결과도 덮어쓰지 않는다. 재실행은 새 출력 경로를 지정한다.
        payload = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            stream.write(payload)
    except (OSError, ValueError) as exc:
        print(f'궤적 분석 실패: {exc}', file=sys.stderr)
        return 1
    print(f"궤적 분석 완료: {report['sample_count']}개 표본, "
          f"위치 잔차 RMSE {report['residual_position']['rmse_m']:.4f} m, "
          f"점프 후보 {report['jump_event_count']}개, "
          f"긴 간격 {report['gap_count']}개 → {output}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
