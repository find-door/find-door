"""과거 실제 검출의 해시·좌표·공유 이력·새 검수 분리를 확인한다."""
from copy import deepcopy
import hashlib
import io
import json

from PIL import Image
import pytest

from real_predictions import prepare_real
from real_reference import audit, digest
from test_real_reference import fixture
from test_label_evaluate import frame


def write(path, value):
    path.write_text(json.dumps(value))


def sources(root, *, failed=False, eligible=True, jpeg=False):
    fixture(root, failed)
    image = root / 'photo.png'
    Image.new('RGB', (100, 80), (37, 42, 93)).save(image, format='JPEG' if jpeg else 'PNG')
    with Image.open(image) as original:
        buffer = io.BytesIO()
        original.convert('RGB').save(buffer, format='PNG')
    prepared_sha = hashlib.sha256(buffer.getvalue()).hexdigest()
    label_path = root / 'label.json'
    label = json.loads(label_path.read_text())
    label['classes']['door']['evaluation_eligible'] = eligible
    write(label_path, label)
    raw_path = root / 'raw.json'
    raw = json.loads(raw_path.read_text())
    raw['input'].update(sha256=digest(image), processed_sha256=prepared_sha,
                        source_size=[100, 80], oriented_size=[100, 80], processed_size=[100, 80],
                        processed_format='PNG', max_side=0)
    detections = [dict(**{'class': 'door'}, confidence=.4, bbox=[10, 10, 30, 60])]
    raw['methods']['C']['result'].update(detections=detections, detection_reused=True)
    raw['methods']['B'] = dict(result=dict(detections=deepcopy(detections)))
    raw['detection_shared'] = True
    write(raw_path, raw)
    manifest_path = root / 'experiments/configs/validation-v1-reviewed.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['frames'][0].update(sha256=digest(image), label_sha256=digest(label_path))
    write(manifest_path, manifest)
    batch_path = root / 'experiments/results/validation-v1-runs/20261101-200920-900749/batch.json'
    batch = json.loads(batch_path.read_text())
    batch['completed'][0]['sha256'] = digest(raw_path)
    batch['source_hashes']['photo.png'] = digest(image)
    write(batch_path, batch)
    summary_path = root / 'summary.json'
    write(summary_path, audit(root))
    f = frame('real_one', 'real_rgb', reviewed=False)
    f['split'] = 'unassigned'
    f['rgb'].update(path=str(image), sha256=digest(image))
    dataset_path = root / 'dataset.json'
    write(dataset_path, dict(schema_version=1, dataset_id='test', frames=[f]))
    return dataset_path, summary_path, raw_path, batch_path


def update_raw(path, batch_path, root, summary_path, modify):
    raw = json.loads(path.read_text())
    modify(raw)
    write(path, raw)
    batch = json.loads(batch_path.read_text())
    batch['completed'][0]['sha256'] = digest(path)
    write(batch_path, batch)
    write(summary_path, audit(root))


@pytest.mark.parametrize('eligible', [False, True])
def test_preserves_boxes_without_promoting_labels_or_legacy_eligibility(tmp_path, eligible):
    dataset, summary, _, _ = sources(tmp_path, eligible=eligible)
    before = dataset.read_bytes()
    result = prepare_real(dataset, tmp_path, summary)
    assert dataset.read_bytes() == before
    assert result['frames'][0]['detections'][0]['bbox'] == [10, 10, 30, 60]
    assert result['frames'][0]['source']['detection_reused_from_B'] is True
    assert result['frames'][0]['status'] == 'complete'


def test_failed_C_is_not_empty_success_even_if_old_count_excluded(tmp_path):
    dataset, summary, _, _ = sources(tmp_path, failed=True, eligible=False)
    row = prepare_real(dataset, tmp_path, summary)['frames'][0]
    assert row['status'] == 'runtime_error' and row['detections'] is None and row['error']


def test_jpeg_to_png_hash_is_checked_without_changing_coordinates(tmp_path):
    dataset, summary, _, _ = sources(tmp_path, jpeg=True)
    row = prepare_real(dataset, tmp_path, summary)['frames'][0]
    assert row['rgb_sha256'] != row['source']['processed_sha256']
    assert row['detections'][0]['bbox'] == [10, 10, 30, 60]


@pytest.mark.parametrize('key', ['processed_size', 'oriented_size', 'source_size', 'processed_sha256'])
def test_changed_coordinate_system_rejected(tmp_path, key):
    dataset, summary, raw, batch = sources(tmp_path)
    update_raw(raw, batch, tmp_path, summary, lambda j: j['input'].update({key: None}))
    with pytest.raises(ValueError, match='좌표계'):
        prepare_real(dataset, tmp_path, summary)


def test_shared_boxes_must_equal_B(tmp_path):
    dataset, summary, raw, batch = sources(tmp_path)
    update_raw(raw, batch, tmp_path, summary,
               lambda j: j['methods']['B']['result'].update(detections=[]))
    with pytest.raises(ValueError, match='공유'):
        prepare_real(dataset, tmp_path, summary)


def test_source_mutation_rejected(tmp_path):
    dataset, summary, raw, _ = sources(tmp_path)
    raw.write_text('{}')
    with pytest.raises(ValueError, match='해시'):
        prepare_real(dataset, tmp_path, summary)


def test_pinned_reference_change_rejected(tmp_path):
    dataset, summary, _, _ = sources(tmp_path)
    document = json.loads(summary.read_text())
    document['batch_sha256'] = '0' * 64
    write(summary, document)
    with pytest.raises(ValueError, match='출처'):
        prepare_real(dataset, tmp_path, summary)


def test_missing_actual_image_rejected(tmp_path):
    dataset, summary, _, _ = sources(tmp_path)
    document = json.loads(dataset.read_text())
    document['frames'][0]['rgb']['sha256'] = '0' * 64
    write(dataset, document)
    with pytest.raises(ValueError, match='목록'):
        prepare_real(dataset, tmp_path, summary)
