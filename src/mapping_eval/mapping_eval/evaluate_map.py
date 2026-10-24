"""저장된 점유 지도 평가 CLI. ROS 실행 환경 없이 동작한다."""

import argparse
import json
from pathlib import Path
import sys

from mapping_eval.map_metrics import evaluate_map


def main(args=None):
    parser = argparse.ArgumentParser(description='PGM 점유 지도와 world 선분 정답 비교')
    parser.add_argument('--map', required=True, dest='map_path', help='지도 YAML 파일')
    parser.add_argument('--truth', required=True, help='world 선분 정답 YAML 파일')
    parser.add_argument('--map-to-world', nargs=3, type=float, required=True,
                        metavar=('X', 'Y', 'YAW'), help='명시적 map→world 변환(m, m, rad)')
    parser.add_argument('--output', required=True, help='평가 JSON 출력 파일')
    parser.add_argument('--tolerance', type=float, default=0.1, help='허용 거리(m)')
    parser.add_argument('--sample-spacing', type=float, default=0.025,
                        help='정답 선분 샘플의 최대 간격(m)')
    options = parser.parse_args(args)
    try:
        report = evaluate_map(options.map_path, options.truth, options.map_to_world,
                              options.tolerance, options.sample_spacing)
        output = Path(options.output).resolve()
        inputs = {Path(value['path']).resolve() for value in report['inputs'].values()}
        if output in inputs:
            raise ValueError('출력 파일은 입력 YAML·PGM과 달라야 합니다')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
                          + '\n', encoding='utf-8')
    except (ValueError, OSError) as exc:
        print(f'평가 실패: {exc}', file=sys.stderr)
        return 1
    errors = report['occupied_to_truth']
    coverage = report['truth_to_occupied']['coverage_at_tolerance']
    print(f"평가 완료: RMSE {errors['rmse_m']:.4f} m, "
          f"정밀도 {errors['precision_at_tolerance']:.1%}, "
          f'정답 관측 범위 {coverage:.1%} → {output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
