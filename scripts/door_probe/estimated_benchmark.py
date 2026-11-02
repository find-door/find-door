"""실제 C 검출→출입구 추정→깊이 상태의 기존 개발 자료 평가. 정답은 채점에만 사용한다."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/mapping_semantic'))
from mapping_semantic.doorway import estimate_doorway  # noqa: E402
from mapping_semantic.door_state import estimate_state  # noqa: E402
from mapping_semantic.door_position import threshold_point  # noqa: E402
from mapping_semantic.core import transform_point  # noqa: E402
from evaluate import iou  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--detections', type=Path, required=True, help='기존 eval results.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('기존 출력은 덮어쓰지 않습니다')
    source = json.loads(args.detections.read_text())
    if source['status'] != 'complete' or len(source['frames']) != source['expected_frames']:
        parser.error('완료된 검출 기록이 필요합니다')
    if source.get('environment') != 'gazebo':
        parser.error('이 평가는 door_probe Gazebo 기록 전용입니다')
    for condition in source['captures_manifest']['conditions']:
        world = ROOT / f"src/mapping_gazebo/worlds/door_probe_{condition['state']}.sdf"
        if not world.is_file() or hashlib.sha256(world.read_bytes()).hexdigest() != condition['world_sha256']:
            parser.error('문턱 원점 정답 계약에 맞는 door_probe world 기록이 아닙니다')
    args.output.mkdir(parents=True)
    report = {'mode': 'detected_doorway_development', 'status': 'running',
              'environment': 'gazebo', 'source_results_sha256': hashlib.sha256(args.detections.read_bytes()).hexdigest(),
              'evaluator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'algorithm_sha256': {name: hashlib.sha256((ROOT / 'src/mapping_semantic/mapping_semantic' / name).read_bytes()).hexdigest()
                                    for name in ['doorway.py', 'door_state.py', 'door_position.py', 'core.py']},
              'limitations': ['기존 단일 문 모델 개발 자료', '검출은 과거 C 기록 재사용',
                              '독립 시험·실제 영상·SLAM 지도 평가 아님'], 'frames': []}
    out = args.output / 'results.json'
    for frame in source['frames']:
        row = {'frame_id': frame['frame_id'], 'condition_id': frame['condition_id'],
               'truth': None if frame['state'] == 'negative' else 'closed' if frame['state'] == 'closed' else 'open',
               'predicted_state': 'unknown', 'doorway_iou': None}
        try:
            if frame.get('status') == 'runtime_error':
                raise RuntimeError(f"상류 C 실행 오류: {frame.get('error', '미기록')}")
            doors = frame.get('door_detections', [])
            if not doors:
                row['reason'] = 'no_door_detection'
            else:
                # 사전 고정: 최고 confidence 문짝 하나를 seed로 사용한다.
                seed = max(doors, key=lambda x: x['confidence'])['bbox']
                meta_path = Path(frame['metadata'])
                metadata = json.loads(meta_path.read_text())
                rgb_path = Path(frame['rgb'])
                depth_path = meta_path.parent / metadata['depth']
                for path, expected in [(rgb_path, metadata['rgb_sha256']), (depth_path, metadata['depth_sha256'])]:
                    if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
                        raise ValueError('입력 해시 불일치')
                rgb = np.asarray(Image.open(rgb_path).convert('RGB'))
                depth = np.load(depth_path, allow_pickle=False)
                k = np.asarray(metadata['camera_info']['k']).reshape(3, 3)
                proposal = estimate_doorway(rgb, depth, k, seed)
                row['doorway'] = proposal
                row['reason'] = proposal['reason']
                if proposal['status'] == 'estimated':
                    state = estimate_state(depth, k, proposal['bbox'])
                    row.update(predicted_state=state['state'], state_evidence=state)
                    evidence = state['evidence']
                    if 'wall_plane_normal' in evidence:
                        try:
                            point = threshold_point(k, proposal['bbox'], evidence['wall_plane_normal'],
                                                    evidence['wall_plane_offset_m'])
                            tf = metadata['odom_from_camera']
                            observed = transform_point(point, tf['translation'], tf['quaternion']).tolist()
                            row.update(estimated_position=observed, position_frame='odom',
                                       position_assumption='estimated bbox bottom corresponds to threshold')
                            # 이 자료의 문턱 정답 (0,0,0)과의 오차는 추론 후 채점한다.
                            if frame['state'] != 'negative':
                                row['position_error_m'] = float(np.linalg.norm(observed))
                        except ValueError as exc:
                            row['position_unavailable'] = str(exc)
                    # 함수 실행이 끝난 다음 별도 정답 bbox로 채점한다.
                    if frame['truth_bbox_xyxy'] is not None:
                        row['doorway_iou'] = iou(proposal['bbox'], frame['truth_bbox_xyxy'])
        except (ValueError, RuntimeError, KeyError, OSError, np.linalg.LinAlgError) as exc:
            row['error'] = f'{type(exc).__name__}: {exc}'
        report['frames'].append(row)
        out.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    positive = [row for row in report['frames'] if row['truth'] is not None]
    report.update(status='complete', total_positive=len(positive),
                  correct_states=sum(row['predicted_state'] == row['truth'] for row in positive),
                  unknown_positive=sum(row['predicted_state'] == 'unknown' for row in positive),
                  doorway_iou_success=sum(row['doorway_iou'] is not None and row['doorway_iou'] >= .5 for row in positive),
                  execution_errors=sum('error' in row for row in report['frames']))
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'frames'}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
