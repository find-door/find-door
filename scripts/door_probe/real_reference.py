"""기존 실제 사진 C 원문의 문 개수 결과를 재감사한다. 새 추론·상태 평가는 하지 않는다."""

import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit(root):
    manifest_path = root / 'experiments/configs/validation-v1-reviewed.json'
    batch_path = root / 'experiments/results/validation-v1-runs/20261101-200920-900749/batch.json'
    manifest, batch = json.loads(manifest_path.read_text()), json.loads(batch_path.read_text())
    if batch['status'] != 'complete' or manifest['human_review_status'] != 'reviewed':
        raise ValueError('완료된 원문과 기존 검수 자료가 필요합니다')
    index = {item['frame_id']:item for item in batch['completed']}
    if len({f['frame_id'] for f in manifest['frames']}) != len(manifest['frames']) or len({f['sha256'] for f in manifest['frames']}) != len(manifest['frames']):
        raise ValueError('검수 자료의 중복 ID·RGB 해시')
    if len(index) != len(batch['completed']) or set(index) != {f['frame_id'] for f in manifest['frames']}:
        raise ValueError('원문·검수 프레임 목록 불일치')
    groups, rows = {}, []
    for frame in manifest['frames']:
        image_path, label_path = root / frame['path'], root / frame['label']
        raw_path = root / index[frame['frame_id']]['path']
        if digest(image_path)!=frame['sha256'] or digest(label_path)!=frame['label_sha256'] or digest(raw_path)!=index[frame['frame_id']]['sha256']:
            raise ValueError('사진·라벨·추론 원문 해시 불일치')
        label, comparison = json.loads(label_path.read_text()), json.loads(raw_path.read_text())
        if comparison['input']['sha256'] != frame['sha256'] or batch['source_hashes'].get(frame['path']) != frame['sha256'] or label['review_status'] != 'reviewed':
            raise ValueError('원문 입력·검수 이력 불일치')
        group = frame['evaluation_group']
        totals = groups.setdefault(group, dict(frames=0, eligible=0, correct_counts=0, excluded=0, runtime_errors=0))
        totals['frames'] += 1
        eligible = label['classes']['door']['evaluation_eligible']
        row = dict(frame_id=frame['frame_id'], group=group, eligible=eligible, raw_sha256=digest(raw_path))
        if not eligible:
            totals['excluded'] += 1
        else:
            totals['eligible'] += 1
            method = comparison['methods']['C']
            result = method['result']
            try:
                if method['exit_code'] or not result['schema_valid'] or result['error'] is not None:
                    raise ValueError('C 실행 실패')
                counts = [x['count'] for x in result['report']['objects'] if x['class']=='door']
                if len(counts)!=1 or type(counts[0]) is not int or counts[0]<0:
                    raise ValueError('문 개수 계약 오류')
                row.update(truth_count=label['classes']['door']['count'], predicted_count=counts[0],
                           correct=counts[0]==label['classes']['door']['count'])
                totals['correct_counts'] += int(row['correct'])
            except (ValueError, KeyError, TypeError) as exc:
                row['error'] = str(exc)
                totals['runtime_errors'] += 1
        rows.append(row)
    return dict(mode='existing_real_rgb_C_count_reference_no_new_inference', groups=groups, frames=rows,
                auditor_sha256=digest(Path(__file__)),
                manifest_sha256=digest(manifest_path), batch_sha256=digest(batch_path),
                original_code_commit=batch['code_commit'], detector_assets=batch['detector_assets'],
                limitations=['과거 추론 원문 재사용, 현재 시스템의 새 평가 아님', '문짝/출입구 bbox·깊이 상태·지도 위치 평가는 아님',
                             '실내·실외 분모 분리, 새 독립 시험셋 아님'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recon-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('기존 참고 결과는 덮어쓰지 않습니다')
    result = audit(args.recon_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(result['groups'], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
