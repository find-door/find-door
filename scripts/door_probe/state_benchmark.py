"""정답 출입구 ROI를 제공한 깊이 상태 후보의 개발 진단. 전체 검출 성능과 구분한다."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/mapping_semantic'))
from mapping_semantic.door_state import estimate_state  # noqa: E402
from evaluate import load_captures  # noqa: E402


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captures', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('기존 결과는 덮어쓰지 않습니다')
    manifest, frames = load_captures(args.captures)
    args.output.mkdir(parents=True)
    result = {
        'schema_version': 1, 'mode': 'oracle_doorway_roi_development', 'environment': 'gazebo',
        'status': 'running', 'captures_sha256': digest(args.captures / 'captures.json'),
        'algorithm_sha256': digest(ROOT / 'src/mapping_semantic/mapping_semantic/door_state.py'),
        'source_capture_commit': manifest['git_commit'],
        'split_group': 'door_probe_v1_single_geometry_development_only',
        'frames': [], 'excluded_negative_frames': 0,
    }
    result['limitations'] = ['정답 출입구 ROI를 제공한 개발 진단', '검출·문틀 위치 추정·지도 통합 성능 아님',
                             '같은 장면 반복 RGB이며 독립 시험셋 아님', '실물 평가 아님']
    result_path = args.output / 'results.json'

    def save():
        result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')

    save()
    for frame in frames:
        if frame['state'] == 'negative':
            result['excluded_negative_frames'] += 1
            continue
        metadata = json.loads(Path(frame['metadata']).read_text())
        depth_path = (Path(frame['metadata']).parent / metadata['depth']).resolve()
        if not depth_path.is_relative_to(args.captures.resolve()):
            raise ValueError('깊이 경로가 수집 폴더 밖에 있음')
        if digest(depth_path) != metadata['depth_sha256']:
            raise ValueError('깊이 원본 해시 불일치')
        truth = 'closed' if frame['state'] == 'closed' else 'open'
        row = {'frame_id': frame['frame_id'], 'condition_id': frame['condition_id'],
               'truth': truth, 'depth_sha256': metadata['depth_sha256'],
               'roi_source': 'projected_geometric_ground_truth'}
        # 분류 함수에는 배치 각도·상태·모델 이름·TF·world 정답 위치를 전달하지 않는다.
        try:
            prediction = estimate_state(np.load(depth_path, allow_pickle=False),
                                        np.asarray(metadata['camera_info']['k']).reshape(3, 3),
                                        frame['truth_bbox_xyxy'])
            row.update(prediction=prediction, success=prediction['state'] == truth)
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as exc:
            row.update(error=f'{type(exc).__name__}: {exc}', success=False)
        result['frames'].append(row)
        save()
    confusion = {truth: {pred: 0 for pred in ['closed', 'open', 'unknown', 'error']}
                 for truth in ['closed', 'open']}
    for row in result['frames']:
        confusion[row['truth']][row.get('prediction', {}).get('state', 'error')] += 1
    result.update(status='complete', confusion=confusion,
                  correct=sum(row['success'] for row in result['frames']), total=len(result['frames']))
    save()
    lines = ['# 정답 출입구 ROI 기반 개발 진단', '', *result['limitations'], '',
             f"정확한 상태 {result['correct']}/{result['total']}, 평벽 제외 {result['excluded_negative_frames']}프레임.", '',
             '| 정답 | closed | open | unknown | 실행 오류 |', '|---|---:|---:|---:|---:|']
    for truth, counts in confusion.items():
        lines.append(f"| {truth} | {counts['closed']} | {counts['open']} | {counts['unknown']} | {counts['error']} |")
    (args.output / 'report.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
