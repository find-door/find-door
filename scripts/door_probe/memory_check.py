"""실제 저장 깊이의 닫힘→열림→닫힘을 인위적 시각으로 연결한 메모리 통합 진단."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/mapping_semantic'))
from mapping_semantic.door_map import DoorMap  # noqa: E402
from mapping_semantic.door_state import estimate_state  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captures', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('기존 출력은 덮어쓰지 않습니다')
    args.output.mkdir(parents=True)
    memory = DoorMap('door-probe-memory-diagnostic')
    report = {'mode': 'oracle_position_roi_stitched_recording_diagnostic',
              'limitations': ['출입구 위치와 ROI는 정답 입력', '인위적 관측 시각 연결',
                              '실제 문 움직임·전이 지연·SLAM·지도 위치 정확도 시험 아님'], 'timeline': []}
    report['source_sha256'] = {name: hashlib.sha256(
        (ROOT / 'src/mapping_semantic/mapping_semantic' / name).read_bytes()).hexdigest()
        for name in ['door_map.py', 'door_state.py']}
    stamp = 1_000_000_000
    for state in ['closed', 'open', 'closed']:
        for index in range(3):
            folder = args.captures / f'{state}_d2_a0'
            metadata = json.loads((folder / f'{index:02d}.json').read_text())
            checksum = hashlib.sha256((folder / metadata['depth']).read_bytes()).hexdigest()
            if checksum != metadata['depth_sha256']:
                raise ValueError('원본 깊이 해시 불일치')
            depth = np.load(folder / metadata['depth'], allow_pickle=False)
            prediction = estimate_state(depth, np.asarray(metadata['camera_info']['k']).reshape(3, 3),
                                        metadata['truth_bbox_xyxy'])
            record = memory.observe([0., 0., 0.], prediction['state'], stamp)
            report['timeline'].append({'stamp_ns': stamp, 'source_state': state,
                                       'depth_sha256': checksum,
                                       'prediction': prediction['state'], 'record': record})
            stamp += 500_000_000
    saved = memory.dump(stamp - 500_000_000)
    (args.output / 'doors.json').write_text(json.dumps(saved, ensure_ascii=False, indent=2) + '\n')
    restored = DoorMap('door-probe-memory-diagnostic')
    restored.load(json.loads((args.output / 'doors.json').read_text()), stamp - 500_000_000)
    report['restored'] = restored.snapshot(stamp - 500_000_000)
    report['expired'] = restored.snapshot(stamp + 3_000_000_000)
    rows = report['timeline']
    checks = {
        'single_id': len({row['record']['id'] for row in rows}) == 1,
        'fixed_position': all(row['record']['position'] == [0., 0., 0.] for row in rows),
        'confirmed_transitions': [rows[index]['record']['state'] for index in [2, 5, 8]] == ['closed', 'open', 'closed'],
        'pending_unknown': all(row['record']['state'] == 'unknown' for index, row in enumerate(rows) if index in [0, 1, 3, 4, 6, 7]),
        'restored_closed': len(report['restored']) == 1 and report['restored'][0]['state'] == 'closed',
        'expired_unknown': len(report['expired']) == 1 and report['expired'][0]['state'] == 'unknown',
    }
    if not all(checks.values()):
        raise RuntimeError(f'통합 진단 실패: {checks}')
    report['checks_passed'] = True
    (args.output / 'check.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
