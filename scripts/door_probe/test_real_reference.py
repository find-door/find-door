"""기존 원문 참고 감사의 실패 분모와 해시 검증을 검사한다."""

import json
import pytest
from real_reference import audit, digest


def fixture(root, failed=False):
    image = root / 'photo.png'; image.write_bytes(b'original image')
    label = root / 'label.json'
    label.write_text(json.dumps({'review_status':'reviewed','classes':{'door':{'evaluation_eligible':True,'count':1}}}))
    raw = root / 'raw.json'
    raw.write_text(json.dumps({'input':{'sha256':digest(image)},'methods':{'C':{'exit_code':1 if failed else 0,
        'result':{'schema_valid':not failed,'error':'failed' if failed else None,'report':{'objects':[{'class':'door','count':1}]}}}}}))
    manifest = root / 'experiments/configs/validation-v1-reviewed.json';manifest.parent.mkdir(parents=True)
    manifest.write_text(json.dumps({'human_review_status':'reviewed','frames':[{'frame_id':'one','path':'photo.png',
        'label':'label.json','sha256':digest(image),'label_sha256':digest(label),'evaluation_group':'indoor_primary'}]}))
    batch = root / 'experiments/results/validation-v1-runs/20261101-200920-900749/batch.json';batch.parent.mkdir(parents=True)
    batch.write_text(json.dumps({'status':'complete','completed':[{'frame_id':'one','path':'raw.json','sha256':digest(raw)}],
        'source_hashes':{'photo.png':digest(image)},'code_commit':'reference','detector_assets':{}}))


@pytest.mark.parametrize('failed', [False, True])
def test_failure_stays_in_denominator(tmp_path, failed):
    fixture(tmp_path, failed)
    result = audit(tmp_path)['groups']['indoor_primary']
    assert result['eligible'] == 1
    assert result['runtime_errors'] == int(failed)
    assert result['correct_counts'] == int(not failed)


def test_changed_source_is_rejected(tmp_path):
    fixture(tmp_path)
    (tmp_path/'photo.png').write_bytes(b'changed')
    with pytest.raises(ValueError, match='해시'):
        audit(tmp_path)
