"""완료된 실제 RGB 50장 C 박스를 재추론 없이 이중 bbox 평가 입력으로 연결한다."""
import argparse
from copy import deepcopy
import hashlib
import io
import json
from pathlib import Path

from PIL import Image
from label_evaluate import validate_predictions, verify_rgb
from label_review import read_manifest
from real_reference import audit, digest


def prepare_real(dataset_path, recon_root, summary_path):
    dataset, checksum = read_manifest(dataset_path)
    summary = json.loads(summary_path.read_text())
    audited = audit(recon_root)
    for key in ('batch_sha256', 'manifest_sha256', 'original_code_commit', 'detector_assets'):
        if audited[key] != summary[key]:
            raise ValueError('추적 요약과 실제 원문 출처 불일치: ' + key)
    manifest_path = recon_root / 'experiments/configs/validation-v1-reviewed.json'
    batch_path = recon_root / 'experiments/results/validation-v1-runs/20261101-200920-900749/batch.json'
    manifest = json.loads(manifest_path.read_text())
    batch = json.loads(batch_path.read_text())
    by_hash = {f['rgb']['sha256']: f for f in dataset['frames'] if f['environment'] == 'real_rgb'}
    if set(by_hash) != {f['sha256'] for f in manifest['frames']}:
        raise ValueError('실제 RGB 자료와 과거 추론 영상 목록 불일치')
    completed = {f['frame_id']: f for f in batch['completed']}
    rows = []
    for original in manifest['frames']:
        frame = by_hash[original['sha256']]
        verify_rgb(dataset_path, frame)
        image_path = Path(frame['rgb']['path'])
        if not image_path.is_absolute():
            image_path = dataset_path.resolve().parent / image_path
        # verify_rgb가 EXIF 방향 1을 확인한다. 당시 RGB PNG 인코딩을 재현해 입력 해시를 검사한다.
        with Image.open(image_path) as image:
            buffer = io.BytesIO()
            image.convert('RGB').save(buffer, format='PNG')
        prepared_sha256 = hashlib.sha256(buffer.getvalue()).hexdigest()
        item = completed[original['frame_id']]
        path = recon_root / item['path']
        raw = json.loads(path.read_text())
        size = [frame['rgb']['width'], frame['rgb']['height']]
        info = raw['input']
        if (any(info.get(key) != size for key in ('source_size', 'oriented_size', 'processed_size'))
                or info.get('processed_sha256') != prepared_sha256
                or info.get('processed_format') != 'PNG' or info.get('max_side') != 0):
            raise ValueError('원본 픽셀 좌표계를 보장할 수 없는 추론 입력')
        method = raw['methods']['C']
        result = method['result']
        failed = method['exit_code'] != 0 or result.get('schema_valid') is not True or result.get('error') is not None
        detections = None if failed else deepcopy(result['detections'])
        reused = result.get('detection_reused', False)
        if not failed and reused:
            if raw.get('detection_shared') is not True or detections != raw['methods']['B']['result']['detections']:
                raise ValueError('B/C 공유 검출 이력 불일치')
        rows.append(dict(id=frame['id'], rgb_sha256=original['sha256'],
                         status='runtime_error' if failed else 'complete',
                         error=str(result.get('error') or '기존 C 실행·스키마 오류') if failed else None,
                         detections=detections,
                         source=dict(frame_id=original['frame_id'], comparison_path=str(path.resolve()),
                                     comparison_sha256=item['sha256'], processed_sha256=prepared_sha256,
                                     detection_reused_from_B=reused,
                                     original_evaluation_group=original['evaluation_group'])))
    output = dict(schema_version=1, frames=rows, provenance=dict(
        mode='existing_real_rgb_C_boxes_no_new_inference',
        importer_sha256=digest(Path(__file__)), dataset_sha256_at_preparation=checksum,
        reference_summary_sha256=digest(summary_path),
        manifest_path=str(manifest_path.resolve()), manifest_sha256=audited['manifest_sha256'],
        batch_path=str(batch_path.resolve()), batch_sha256=audited['batch_sha256'],
        original_code_commit=audited['original_code_commit'], detector_assets=audited['detector_assets']))
    output['provenance']['limitations'] = [
        '과거 실제 사진 추론 재사용, 현재 ROS 시스템의 실물 평가 아님',
        '기존 개수 평가의 제외 여부는 새 bbox 평가에 승계하지 않음',
        '새 이중 bbox 검수·가시성으로 채점, 장면 독립성·분할은 별도 확인',
        '깊이·문 상태·지도 위치·추론 시간 평가 아님']
    validate_predictions(output, dataset)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--recon-root', type=Path, required=True)
    parser.add_argument('--summary', type=Path, default=Path('docs/data/door-real-reference-summary.json'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('기존 예측은 덮어쓰지 않습니다')
    result = prepare_real(args.dataset, args.recon_root, args.summary)
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as handle:
        handle.write(encoded)
    print(json.dumps(dict(output=str(args.output), frames=len(result['frames']),
                          runtime_errors=sum(f['status'] == 'runtime_error' for f in result['frames'])), ensure_ascii=False))


if __name__ == '__main__':
    main()
