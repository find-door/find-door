"""worker 입력 자산의 불변성과 잘못된 검출 응답 거부를 검증한다."""

from copy import deepcopy
import hashlib
import json
import os
import subprocess
import sys
import pytest

from mapping_semantic.detector_worker import runtime_config, backend_name, serialize_detection
from mapping_semantic.live_detector import validate_reply
from mapping_semantic.live_detector import Worker


def test_runtime_assets_are_absolute_and_external_config_unchanged(tmp_path):
    source = tmp_path / 'source'; source.mkdir()
    runtime = tmp_path / 'runtime'; runtime.mkdir()
    config = {'release_policy': 'unload'}
    for name in ['checkpoint', 'embeddings', 'manifest', 'prompts_config']:
        (source / name).write_text('asset')
        config[name] = name
    path = source / 'config.json'; path.write_text(json.dumps(config))
    original = path.read_bytes()
    generated, source_hash = runtime_config(path, runtime)
    result = json.loads(generated.read_text())
    assert path.read_bytes() == original
    assert source_hash == hashlib.sha256(original).hexdigest()
    assert result['release_policy'] == 'keep'
    assert all(result[name] == str(source / name) for name in ['checkpoint', 'embeddings', 'manifest', 'prompts_config'])
    (source / 'checkpoint').unlink()
    with pytest.raises(ValueError, match='자산 없음'):
        runtime_config(path, runtime)


@pytest.mark.parametrize('value,expected',[({},'yoloe'),({'backend':'yoloe'},'yoloe'),({'backend':'yolo11'},'yolo11')])
def test_backend_selection_keeps_existing_default(tmp_path,value,expected):
    path=tmp_path/'config.json';path.write_text(json.dumps(value))
    assert backend_name(path)==expected


@pytest.mark.parametrize('value',[[],None,{'backend':'unknown'},{'backend':None}])
def test_unknown_backend_is_rejected(tmp_path,value):
    path=tmp_path/'config.json';path.write_text(json.dumps(value))
    with pytest.raises(ValueError,match='backend'):
        backend_name(path)


def test_backend_selector_is_not_forwarded_to_foreign_yoloe_config(tmp_path):
    source=tmp_path/'source';source.mkdir();runtime=tmp_path/'runtime';runtime.mkdir()
    config=dict(backend='yoloe')
    for name in ['checkpoint','embeddings','manifest','prompts_config']:
        (source/name).write_text('asset');config[name]=name
    path=source/'config.json';path.write_text(json.dumps(config));original=path.read_bytes()
    generated,_=runtime_config(path,runtime)
    assert 'backend'not in json.loads(generated.read_text())
    assert path.read_bytes()==original


def test_plain_and_existing_model_records_share_the_worker_contract():
    record={'class':'door','report_class':'door','confidence':.2,'bbox':[1,2,30,40]}
    class Existing:
        def model_dump(self,by_alias):
            assert by_alias
            return record
    assert serialize_detection(record)==serialize_detection(Existing())==record


def good():
    return {'id': 1, 'detections': [{'class': 'door', 'report_class': 'door', 'confidence': .2, 'bbox': [1., 2., 30., 40.]}]}


def test_result_and_empty_detection():
    assert validate_reply(good(), 1, 640, 480)[0]['class'] == 'door'
    assert validate_reply({'id': 1, 'detections': []}, 1, 640, 480) == []


@pytest.mark.parametrize('kind', ['id', 'boolean_id', 'error', 'nan', 'outside', 'reversed', 'label', 'missing', 'nonrecord'])
def test_invalid_worker_result(kind):
    reply = deepcopy(good())
    item = reply['detections'][0]
    if kind == 'id': reply['id'] = 2
    elif kind == 'boolean_id': reply['id'] = True
    elif kind == 'error': reply['error'] = 'failed'
    elif kind == 'nan': item['confidence'] = float('nan')
    elif kind == 'outside': item['bbox'][2] = 700
    elif kind == 'reversed': item['bbox'][2] = 0
    elif kind == 'label': item['report_class'] = 'person'
    elif kind == 'missing': del reply['detections']
    else: reply['detections'] = [None]
    with pytest.raises(ValueError):
        validate_reply(reply, 1, 640, 480)


def test_worker_timeout_and_owned_process_cleanup(tmp_path, monkeypatch):
    root = tmp_path / 'recon'; (root / '.venv/bin').mkdir(parents=True)
    (root / '.venv/bin/python').symlink_to(sys.executable)
    config = tmp_path / 'config.json'; config.write_text('{}')
    fake = tmp_path / 'fake.py'
    fake.write_text('import time\ntime.sleep(30)\n')
    real_popen = subprocess.Popen
    observed = {}

    def fake_popen(command, **kwargs):
        observed.update(kwargs)
        return real_popen([sys.executable, '-u', str(fake)], **kwargs)

    monkeypatch.setattr('mapping_semantic.live_detector.subprocess.Popen', fake_popen)
    monkeypatch.setenv('PYTHONPATH', 'should-not-pass')
    monkeypatch.setenv('PYTHONHOME', 'should-not-pass')
    worker = Worker(root, config, tmp_path, .05)
    try:
        with pytest.raises(TimeoutError):
            worker.request(1, tmp_path / 'frame.png')
    finally:
        worker.close()
        worker.close()
    assert worker.process.poll() is not None
    assert 'PYTHONPATH' not in observed['env'] and 'PYTHONHOME' not in observed['env']
    assert observed['start_new_session'] is True
    with pytest.raises(ProcessLookupError):
        os.killpg(worker.process.pid, 0)
