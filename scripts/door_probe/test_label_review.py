"""검수 수정본의 출처·원본 보존·명시적 검수 경계 검사."""
import copy
import hashlib
import json
from pathlib import Path
import sys

from PIL import Image
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from label_review import apply_patch, export_html


@pytest.fixture
def case(tmp_path):
    image = tmp_path / 'image.png'
    Image.new('RGB', (40, 30)).save(image)
    frame = dict(id='f1', environment='real_rgb', group_id='g', split='unassigned',
                 rgb=dict(path='image.png', sha256=hashlib.sha256(image.read_bytes()).hexdigest(), width=40, height=30),
                 capabilities=dict(depth=False, camera_info=False, tf=False),
                 review_status='draft', doors=[dict(id='door_1')], legacy_labels={'note': '보존'})
    manifest = tmp_path / 'dataset.json'
    manifest.write_text(json.dumps(dict(schema_version=1, dataset_id='test', frames=[frame])))
    patch = dict(schema_version=1, base_sha256=hashlib.sha256(manifest.read_bytes()).hexdigest(),
                 frames=[dict(id='f1', reviewed=False, doors=[dict(id='door_1', state='closed',
                     leaf_bbox=[1, 2, 20, 25], leaf_visibility='visible',
                     doorway_bbox=None, doorway_visibility='unknown')])])
    return manifest, patch, tmp_path


def apply(case, patch=None, reviewer=None):
    manifest, original, root = case
    path = root / 'patch.json'
    path.write_text(json.dumps(original if patch is None else patch))
    output = root / 'new.json'
    result = apply_patch(manifest, path, output, reviewer)
    return result, json.loads(output.read_text())


def test_apply_draft_preserves_source(case):
    original = case[0].read_bytes()
    result, doc = apply(case)
    assert result == dict(frames=1, reviewed=0, draft=1)
    frame = doc['frames'][0]
    assert frame['doors'][0]['leaf_bbox'] == [1, 2, 20, 25]
    assert frame['legacy_labels'] == {'note': '보존'}
    assert frame['split'] == 'unassigned'
    assert Path(frame['rgb']['path']).is_absolute()
    assert 'annotation_review' not in frame
    assert case[0].read_bytes() == original


def test_review_requires_name(case):
    case[1]['frames'][0]['reviewed'] = True
    with pytest.raises(ValueError, match='reviewer'):
        apply(case)
    assert not (case[2] / 'new.json').exists()
    result, doc = apply(case, reviewer='검수자')
    assert result['reviewed'] == 1
    assert doc['frames'][0]['annotation_review']['reviewer'] == '검수자'
    assert doc['frames'][0]['review_status'] == 'reviewed'


@pytest.mark.parametrize('mutation', [
    lambda p: p.update(base_sha256='0'*64),
    lambda p: p['frames'].append(copy.deepcopy(p['frames'][0])),
    lambda p: p['frames'][0].update(id='other'),
    lambda p: p['frames'][0].update(split='test'),
    lambda p: p['frames'][0].update(review_context={'note':'변경 시도'}),
    lambda p: p['frames'][0].update(reviewed=1),
    lambda p: p['frames'][0]['doors'][0].update(leaf_bbox=[0, 0, 100, 10]),
    lambda p: p['frames'][0]['doors'][0].update(doorway_visibility='visible'),
])
def test_invalid_patch_not_written(case, mutation):
    patch = copy.deepcopy(case[1]); mutation(patch)
    with pytest.raises(ValueError):
        apply(case, patch)
    assert not (case[2] / 'new.json').exists()


def test_source_image_changed(case):
    (case[2] / 'image.png').write_bytes(b'changed')
    with pytest.raises(ValueError, match='해시'):
        apply(case)


def test_no_overwrite(case):
    output = case[2] / 'new.json'
    output.write_text('keep')
    with pytest.raises(FileExistsError):
        apply(case)
    assert output.read_text() == 'keep'


def test_export_offline_and_escape(case):
    document = json.loads(case[0].read_text())
    document['frames'][0]['id'] = '</script><script>alert(1)</script>'
    document['frames'][0]['review_context'] = {'note': '</script> 참고 정보'}
    document['frames'][0]['review_status'] = 'reviewed'
    document['frames'][0]['doors'] = []
    case[0].write_text(json.dumps(document))
    output = case[2] / 'review.html'
    assert export_html(case[0], output) == 1
    text = output.read_text()
    assert 'data:image/png;base64,' in text
    assert '</script><script>alert(1)</script>' not in text
    assert '"reviewed": false' in text
    assert '/*__DATA__*/' not in text
    payload = text.split('const data = ', 1)[1].split(';\nconst $', 1)[0]
    assert json.loads(payload)['frames'][0]['id'] == document['frames'][0]['id']
    assert json.loads(payload)['frames'][0]['review_context'] == document['frames'][0]['review_context']
    with pytest.raises(FileExistsError):
        export_html(case[0], output)
