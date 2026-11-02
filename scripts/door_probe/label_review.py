"""문짝·출입구 이중 라벨의 오프라인 검수 화면 생성과 수정본 적용."""

import argparse
import base64
from datetime import datetime, timezone
import hashlib
import io
import json
from pathlib import Path
import sys

from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/mapping_semantic'))
from mapping_semantic.door_dataset import validate_dataset  # noqa: E402


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read_manifest(path):
    raw = Path(path).read_bytes()
    return validate_dataset(json.loads(raw)), sha(raw)


def image_bytes(frame, root):
    path = Path(frame['rgb']['path'])
    if not path.is_absolute():
        path = root / path
    raw = path.read_bytes()
    if sha(raw) != frame['rgb']['sha256']:
        raise ValueError(f"원본 영상 해시 불일치: {frame['id']}")
    with Image.open(io.BytesIO(raw)) as image:
        if image.size != (frame['rgb']['width'], frame['rgb']['height']):
            raise ValueError('원본 영상 크기 불일치')
        if image.getexif().get(274, 1) != 1:
            raise ValueError('방향 보정 전 영상은 지원하지 않습니다')
        # 브라우저가 지원하는 PNG로 재인코딩하며 원본은 변경하지 않는다.
        output = io.BytesIO()
        image.convert('RGB').save(output, format='PNG')
    return output.getvalue()


def export_html(manifest_path, output_path):
    document, checksum = read_manifest(manifest_path)
    frames = []
    for frame in document['frames']:
        data = image_bytes(frame, Path(manifest_path).resolve().parent)
        frames.append(dict(id=frame['id'], environment=frame['environment'],
                           group_id=frame['group_id'], split=frame['split'],
                           doors=frame['doors'], reviewed=False,
                           image='data:image/png;base64,' + base64.b64encode(data).decode()))
    payload = json.dumps(dict(schema_version=1, base_sha256=checksum, frames=frames),
                         ensure_ascii=False).replace('<', chr(92) + 'u003c')
    template = Path(__file__).with_name('label_review.html').read_text()
    html = template.replace('/*__DATA__*/', payload)
    with Path(output_path).open('x', encoding='utf-8') as stream:
        stream.write(html)
    return len(frames)


def apply_patch(manifest_path, patch_path, output_path, reviewer=None):
    document, checksum = read_manifest(manifest_path)
    patch_raw = Path(patch_path).read_bytes()
    patch = json.loads(patch_raw)
    if (not isinstance(patch, dict) or set(patch) != {'schema_version', 'base_sha256', 'frames'}
            or type(patch['schema_version']) is not int or patch['schema_version'] != 1
            or patch['base_sha256'] != checksum):
        raise ValueError('원본 manifest 해시 또는 수정본 형식 불일치')
    if not isinstance(patch['frames'], list):
        raise ValueError('frames는 목록이어야 합니다')
    changes = {}
    for item in patch['frames']:
        if (not isinstance(item, dict) or set(item) != {'id', 'doors', 'reviewed'}
                or not isinstance(item['id'], str) or type(item['reviewed']) is not bool):
            raise ValueError('프레임 수정 형식 오류')
        if item['id'] in changes:
            raise ValueError('중복 프레임 수정')
        changes[item['id']] = item
    if set(changes) != {frame['id'] for frame in document['frames']}:
        raise ValueError('원본과 수정본 프레임 목록이 다릅니다')
    reviewed_count = sum(item['reviewed'] for item in changes.values())
    if reviewed_count and (not isinstance(reviewer, str) or not reviewer.strip()):
        raise ValueError('검수 완료 프레임에는 --reviewer 검수자 이름이 필요합니다')
    now = datetime.now(timezone.utc).isoformat()
    for frame in document['frames']:
        image_bytes(frame, Path(manifest_path).resolve().parent)
        item = changes[frame['id']]
        frame['doors'] = item['doors']
        frame['review_status'] = 'reviewed' if item['reviewed'] else 'draft'
        frame.pop('annotation_review', None)
        if item['reviewed']:
            frame['annotation_review'] = dict(reviewer=reviewer.strip(), reviewed_at=now)
    document = validate_dataset(document)
    history = document.setdefault('annotation_history', [])
    if not isinstance(history, list):
        raise ValueError('annotation_history는 목록이어야 합니다')
    history.append(dict(base_sha256=checksum, patch_sha256=sha(patch_raw), applied_at=now,
                        tool_sha256=sha(Path(__file__).read_bytes()),
                        reviewer=reviewer.strip() if reviewer else None,
                        reviewed_frames=reviewed_count))
    # 다른 위치에서도 원본을 찾도록 상대 경로를 절대 경로로 고정한다.
    for frame in document['frames']:
        path = Path(frame['rgb']['path'])
        if not path.is_absolute():
            frame['rgb']['path'] = str((Path(manifest_path).resolve().parent / path).resolve())
    with Path(output_path).open('x', encoding='utf-8') as stream:
        json.dump(document, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    return dict(frames=len(document['frames']), reviewed=reviewed_count,
                draft=len(document['frames']) - reviewed_count)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    export = sub.add_parser('export', help='원본 영상을 포함한 로컬 HTML 생성')
    export.add_argument('--manifest', type=Path, required=True)
    export.add_argument('--output', type=Path, required=True)
    apply = sub.add_parser('apply', help='브라우저가 저장한 수정본 검증·적용')
    apply.add_argument('--manifest', type=Path, required=True)
    apply.add_argument('--patch', type=Path, required=True)
    apply.add_argument('--reviewer')
    apply.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.command == 'export':
        print(json.dumps(dict(frames=export_html(args.manifest, args.output)), ensure_ascii=False))
    else:
        print(json.dumps(apply_patch(args.manifest, args.patch, args.output, args.reviewer), ensure_ascii=False))


if __name__ == '__main__':
    main()
