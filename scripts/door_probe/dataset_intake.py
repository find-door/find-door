"""기존 Gazebo·실제 RGB 자료를 복사 없이 점검하고 이중 라벨 초안을 만든다."""

import argparse
import hashlib
import json
from pathlib import Path
import sys

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/mapping_semantic'))
from mapping_semantic.door_dataset import validate_dataset  # noqa: E402


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rgb(path, expected):
    if digest(path) != expected:
        raise ValueError(f'원본 영상 해시 불일치: {path}')
    with Image.open(path) as image:
        width, height = image.size
        if image.getexif().get(274, 1) != 1:
            raise ValueError('방향 보정 전 영상은 별도 준비가 필요합니다')
    return dict(path=str(path.resolve()), sha256=expected, width=width, height=height)


def door(identifier, state):
    return dict(id=identifier, state=state, leaf_bbox=None, doorway_bbox=None,
                leaf_visibility='unknown', doorway_visibility='unknown')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captures', type=Path, required=True)
    parser.add_argument('--recon-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('기존 라벨 초안은 덮어쓰지 않습니다')
    captures = json.loads((args.captures / 'captures.json').read_text())
    if captures['status'] != 'complete':
        parser.error('완료된 수집 기록이 필요합니다')
    external_manifest = args.recon_root / 'experiments/configs/validation-v1-reviewed.json'
    external = json.loads(external_manifest.read_text())
    if external.get('human_review_status') != 'reviewed':
        raise ValueError('기존 실제 사진 자료의 검수 이력이 필요합니다')
    result = dict(schema_version=1, dataset_id='door-dual-target-intake-v1', frames=[],
                  provenance={'captures_sha256': digest(args.captures / 'captures.json'),
                              'external_manifest_sha256': digest(external_manifest)},
                  limitations=['문짝·출입구 박스는 재검수 전이므로 null', '원본 영상·라벨을 재배포하지 않음',
                               '새 독립 시험셋 또는 학습 준비 완료 자료가 아님'])
    seen = set()
    omitted = 0
    for condition in captures['conditions']:
        for frame in condition['frames']:
            meta_path = args.captures / frame['metadata']
            meta = json.loads(meta_path.read_text())
            info = rgb(args.captures / frame['rgb'], meta['rgb_sha256'])
            if info['sha256'] in seen:
                omitted += 1
                continue
            seen.add(info['sha256'])
            depth_path = meta_path.parent / meta['depth']
            if digest(depth_path) != meta['depth_sha256']:
                raise ValueError('Gazebo 깊이 원본 해시 불일치')
            if len(meta['camera_info']['k']) != 9 or len(meta['odom_from_camera']['translation']) != 3 or len(meta['odom_from_camera']['quaternion']) != 4:
                raise ValueError('Gazebo 보정·TF 기록 불완전')
            result['frames'].append(dict(id=f'gz_{condition["id"]}', environment='gazebo',
                group_id='gazebo_door_probe_v1_single_geometry', split='development', rgb=info,
                capabilities=dict(depth=True, camera_info=True, tf=True), review_status='draft',
                doors=[] if condition['state']=='negative' else [door('door_1', 'closed' if condition['state']=='closed' else 'open')],
                legacy_labels=dict(metadata_path=str(meta_path.resolve()), metadata_sha256=digest(meta_path),
                                   projected_bbox=meta['truth_bbox_xyxy'],
                                   note='기존 기하 bbox는 가림·문짝/출입구 이중 라벨 검수 전')))
    for frame in external['frames']:
        label_path = args.recon_root / frame['label']
        if digest(label_path) != frame['label_sha256']:
            raise ValueError(f'기존 라벨 해시 불일치: {label_path}')
        label = json.loads(label_path.read_text())
        if label.get('review_status') != 'reviewed' or frame.get('human_review_status') != 'reviewed':
            raise ValueError('기존 실제 사진 라벨이 검수본이 아닙니다')
        image_info = rgb(args.recon_root / frame['path'], frame['sha256'])
        if [image_info['width'], image_info['height']] != frame['oriented_size']:
            raise ValueError('기존 검수 영상 크기 불일치')
        result['frames'].append(dict(id=f'real_{frame["frame_id"]}', environment='real_rgb',
            group_id=f'legacy_collection_{frame["dataset"]}', split='unassigned', rgb=image_info,
            capabilities=dict(depth=False, camera_info=False, tf=False), review_status='draft',
            doors=[door(item['instance_id'], item.get('state', 'unknown')) for item in label['instances'] if item['class']=='door'],
            legacy_labels=dict(path=str(label_path.resolve()), sha256=frame['label_sha256'],
                               original_review_status=label['review_status'],
                               evaluation_group=frame['evaluation_group'],
                               source_dataset=frame['dataset'], source_license_status=frame.get('license_status'),
                               evaluation_eligible=label['classes']['door']['evaluation_eligible'],
                               note='출입문 bbox의 문짝/문틀 의미가 분리되지 않아 새 박스는 미확정')))
    validated = validate_dataset(result)
    args.output.mkdir(parents=True)
    (args.output / 'dataset.json').write_text(json.dumps(validated, ensure_ascii=False, indent=2) + '\n')
    summary = dict(total_frames=len(validated['frames']), omitted_duplicate_rgb=omitted,
                   validator_sha256=digest(ROOT / 'src/mapping_semantic/mapping_semantic/door_dataset.py'),
                   intake_sha256=digest(__file__),
                   environments={name:sum(frame['environment']==name for frame in validated['frames'])
                                 for name in ['gazebo', 'real_rgb', 'real_rgbd']},
                   reviewed_dual_target_frames=sum(frame['review_status']=='reviewed' for frame in validated['frames']),
                   dataset_sha256=digest(args.output / 'dataset.json'), provenance=result['provenance'],
                   limitations=result['limitations'])
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
