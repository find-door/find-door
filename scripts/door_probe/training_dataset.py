"""AI 라벨의 출처 그룹을 보존해 문짝/출입구별 YOLO 개발 학습 입력을 만든다."""
import argparse
from collections import Counter, defaultdict
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from urllib.parse import urlsplit

import yaml

from dataset_intake import digest
from label_review import read_manifest
from label_evaluate import verify_rgb
from mapping_semantic.door_dataset import validate_dataset


def rank(group, seed):
    return hashlib.sha256(f'{seed}:{group}'.encode()).hexdigest()


def make_plan(dataset, source, seed=20261103):
    dataset = validate_dataset(dataset)
    real = [f for f in dataset['frames'] if f['environment'] == 'real_rgb']
    upstream = {f['sha256']: f for f in source['frames']}
    if len(upstream) != len(source['frames']) or set(upstream) != {f['rgb']['sha256'] for f in real}:
        raise ValueError('출처와 실제 RGB 목록·중복 해시 불일치')
    groups = defaultdict(list)
    metadata = {}
    identities = {}
    for frame in real:
        review = frame.get('annotation_review', {})
        if (review.get('method') != 'ai_visual' or review.get('human_reviewed') is not False
                or type(frame.get('annotation_uncertain')) is not bool
                or frame['review_status'] != ('draft' if frame['annotation_uncertain'] else 'reviewed')):
            raise ValueError('AI 검토 출처·불확실성과 reviewed 선언 불일치')
        item = upstream[frame['rgb']['sha256']]
        profile = None
        if item['dataset'] == 'DeepDoors2':
            group = 'legacy_collection_DeepDoors2'
            identity = 'DeepDoors2_collection'
        elif item['dataset'] == 'Open Images V7':
            profile = item['source']['AuthorProfileURL']
            url = urlsplit(profile)
            if (url.scheme != 'https' or url.netloc.lower() not in ('www.flickr.com', 'flickr.com')
                    or not url.path.startswith('/people/') or not url.path[len('/people/'):].strip('/')
                    or url.query or url.fragment):
                raise ValueError('확인 가능한 Flickr 작성자 프로필이 필요합니다')
            identity = url.path.rstrip('/').casefold()
            group = 'oi_author_' + hashlib.sha256(identity.encode()).hexdigest()[:16]
        else:
            raise ValueError('현재 감사한 두 source collection만 지원합니다')
        if group in identities and identities[group] != identity:
            raise ValueError('작성자 그룹 해시 충돌')
        identities[group] = identity
        groups[group].append(frame)
        metadata[frame['id']] = dict(id=frame['id'], rgb_sha256=frame['rgb']['sha256'], group_id=group,
            source_dataset=item['dataset'], source_split=item['split'], source_frame_id=item['frame_id'],
            source_author_profile=profile, original_group_id=frame['group_id'], original_split=frame['split'])
    candidates = defaultdict(list)
    for group, frames in groups.items():
        if not group.startswith('oi_author_'):
            continue
        # 검증 후보는 두 target 모두 채점 가능하고 프레임 완전성이 확인된 그룹에 한정한다.
        if any(f['review_status'] != 'reviewed' or any(d[t+'_visibility'] != 'visible'
                   for d in f['doors'] for t in ('leaf', 'doorway')) for f in frames):
            continue
        doors = [d for f in frames for d in f['doors']]
        states = {d['state'] for d in doors}
        category = 'negative' if not doors else next(iter(states)) if len(states) == 1 else 'mixed'
        candidates[category].append(group)
    validation = set()
    quotas = dict(closed=1, open=2, negative=4)
    for category, count in quotas.items():
        available = sorted(candidates[category], key=lambda g: (rank(g, seed), g))
        if len(available) < count:
            raise ValueError('고정 개발 검증 그룹 수 부족: ' + category)
        validation.update(available[:count])
    rows = []
    for frame in real:
        row = metadata[frame['id']]
        row['proposed_split'] = 'validation' if row['group_id'] in validation else 'train'
        rows.append(row)
    return dict(schema_version=1, purpose='development_training_pilot_not_independent_test', seed=seed,
        reference_kind='ai_visual_not_human_ground_truth', validation_quotas=quotas,
        validation_groups=sorted(validation), frames=rows,
        policy=['DeepDoors2 전체 collection을 한 그룹으로 유지', 'Open Images 작성자별 그룹, 장소 독립성 보장 아님',
                '성능·예측을 입력으로 사용하지 않는 해시 순서와 라벨 층화',
                '상류 test/validation 출처 보존, 이번 개발 학습 이후 독립 benchmark로 사용 금지',
                '별도 test 분할 없음, Gazebo 개발 자료는 학습/검증에서 제외'])


def select_target(dataset, plan, target):
    if target not in ('leaf', 'doorway'):
        raise ValueError('학습 target은 leaf 또는 doorway를 명시해야 합니다')
    dataset = validate_dataset(dataset)
    by_id = {r['id']: r for r in plan['frames']}
    real = [f for f in dataset['frames'] if f['environment'] == 'real_rgb']
    if len(by_id) != len(plan['frames']) or set(by_id) != {f['id'] for f in real}:
        raise ValueError('분할 계획과 실제 사진 ID 불일치')
    frames, excluded = [], []
    for frame in real:
        row = by_id[frame['id']]
        if row['rgb_sha256'] != frame['rgb']['sha256']:
            raise ValueError('분할 계획 RGB 해시 불일치')
        reason = ('uncertain_or_draft' if frame['review_status'] != 'reviewed' or frame['annotation_uncertain']
                  else 'target_visibility_unknown' if any(d[target+'_visibility'] != 'visible' for d in frame['doors']) else None)
        if reason:
            excluded.append(dict(id=frame['id'], reason=reason))
            continue
        item = deepcopy(frame)
        item['training_source'] = deepcopy(row)
        item['group_id'] = row['group_id']
        item['split'] = row['proposed_split']
        if item['split'] not in ('train', 'validation'):
            raise ValueError('개발 학습/검증만 허용합니다')
        frames.append(item)
    result = validate_dataset(dict(schema_version=1, dataset_id=dataset['dataset_id']+'-pilot-'+target,
        frames=frames, reference_kind='ai_visual_not_human_ground_truth', training_target=target,
        independence_status='not_verified_development_only'))
    for split in ('train', 'validation'):
        selected = [f for f in frames if f['split'] == split]
        if not selected or not any(f['doors'] for f in selected) or not any(not f['doors'] for f in selected):
            raise ValueError('학습과 검증 각각 양성·음성 사진이 필요합니다')
    return result, excluded


def export_dataset(manifest_path, source_path, reference_path, target, output, seed=20261103):
    if output.exists():
        raise FileExistsError('기존 학습 입력은 덮어쓰지 않습니다')
    dataset, checksum = read_manifest(manifest_path)
    reference = json.loads(reference_path.read_text())
    if digest(source_path) != reference['manifest_sha256']:
        raise ValueError('감사한 상류 manifest와 해시 불일치')
    source = json.loads(source_path.read_text())
    plan = make_plan(dataset, source, seed)
    selected, excluded = select_target(dataset, plan, target)
    # 출력 생성 전에 원본을 모두 확인한다. 원본 이미지·라벨은 변경하지 않는다.
    for frame in dataset['frames']:
        if frame['environment'] == 'real_rgb':
            verify_rgb(manifest_path, frame)
    root = Path(manifest_path).resolve().parent
    output = output.resolve()
    payloads = []
    for frame in selected['frames']:
        rgb = frame['rgb']
        image = Path(rgb['path'])
        if not image.is_absolute():
            image = root / image
        rgb['path'] = str(image.resolve())
        suffix = image.suffix.lower()
        if suffix not in ('.png', '.jpg', '.jpeg', '.webp'):
            raise ValueError('지원하지 않는 RGB 확장자')
        split = 'train' if frame['split'] == 'train' else 'val'
        stem = 'rgb_' + rgb['sha256']
        lines = []
        for door in frame['doors']:
            x0, y0, x1, y1 = door[target+'_bbox']
            xywh = [(x0+x1)/(2*rgb['width']), (y0+y1)/(2*rgb['height']),
                    (x1-x0)/rgb['width'], (y1-y0)/rgb['height']]
            lines.append('0 ' + ' '.join(format(x, '.17g') for x in xywh))
        payloads.append((frame, image.resolve(), split, stem, suffix, '\n'.join(lines)+ ('\n' if lines else '')))
    output.mkdir(parents=True, exist_ok=False)
    records = []
    for frame, image, split, stem, suffix, text in payloads:
        images, labels = output/'images'/split, output/'labels'/split
        images.mkdir(parents=True, exist_ok=True)
        labels.mkdir(parents=True, exist_ok=True)
        image_link, label_path = images/(stem+suffix), labels/(stem+'.txt')
        image_link.symlink_to(image)
        label_path.write_text(text)
        records.append(dict(id=frame['id'], split=frame['split'], group_id=frame['group_id'],
            rgb_sha256=frame['rgb']['sha256'], image=str(image_link), label=str(label_path), label_sha256=digest(label_path), objects=len(frame['doors'])))
    config = dict(path=str(output), train='images/train', val='images/val', names={0:'door_leaf' if target=='leaf' else 'doorway'})
    (output/'dataset.yaml').write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    for name, doc in [('manifest', selected), ('split-plan', plan)]:
        (output/(name+'.json')).write_text(json.dumps(doc, ensure_ascii=False, indent=2)+'\n')
    report = dict(target=target, reference_kind=plan['reference_kind'], purpose=plan['purpose'],
        manifest_source_sha256=checksum, source_manifest_sha256=digest(source_path), reference_sha256=digest(reference_path),
        exporter_sha256=digest(__file__), exported_manifest_sha256=digest(output/'manifest.json'),
        yaml_sha256=digest(output/'dataset.yaml'), plan_sha256=digest(output/'split-plan.json'),
        split_counts=dict(Counter(r['split'] for r in records)),
        negative_split_counts=dict(Counter(r['split'] for r in records if r['objects']==0)),
        object_split_counts={s:sum(r['objects'] for r in records if r['split']==s) for s in ('train','validation')},
        excluded=excluded, frames=records, model_training_run=False)
    (output/'export-report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--source-manifest', type=Path, required=True)
    parser.add_argument('--reference-summary', type=Path, default=Path('docs/data/door-real-reference-summary.json'))
    parser.add_argument('--target', choices=['leaf','doorway'], required=True)
    parser.add_argument('--seed', type=int, default=20261103)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = export_dataset(args.manifest, args.source_manifest, args.reference_summary, args.target, args.output, args.seed)
    print(json.dumps(dict(target=result['target'], split_counts=result['split_counts'],
                         excluded=len(result['excluded'])), ensure_ascii=False))


if __name__ == '__main__':
    main()
