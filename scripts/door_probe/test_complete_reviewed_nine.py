"""고정9장 진단이 기존예측을 보존하고 누락3장만 실행하는지 확인한다."""
from copy import deepcopy
import json
import os
import signal
from types import SimpleNamespace as N
import pytest
from PIL import Image
import complete_reviewed_nine as C
from dataset_intake import digest,door


@pytest.fixture
def setup(tmp_path,monkeypatch):
    frames=[]
    for n,identifier in enumerate(sorted(C.EXPECTED)):
        path=tmp_path/f'{n}.png';Image.new('RGB',(100,80),(n,20,30)).save(path)
        item=door('door_1','open');item.update(leaf_bbox=[10,10,30,60],leaf_visibility='visible',doorway_bbox=[5,5,50,70],doorway_visibility='visible')
        frames.append(dict(id=identifier,environment='gazebo',group_id='family',split='development',review_status='reviewed',
                           rgb=dict(path=str(path),sha256=digest(path),width=100,height=80),
                           capabilities=dict(depth=False,camera_info=False,tf=False),doors=[item]))
    dataset=tmp_path/'dataset.json';dataset.write_text(json.dumps(dict(schema_version=1,dataset_id='test',frames=frames)))
    identity=dict(config_sha256='c'*64,config_path=str(tmp_path/'config.json'),sources_sha256={'backend':'d'*64},assets={'checkpoint':{'sha256':'e'*64}})
    monkeypatch.setattr(C,'provenance',lambda root:deepcopy(identity))
    predictions=tmp_path/'predictions.json'
    original=dict(schema_version=1,provenance={'recon':deepcopy(identity)},frames=[
        dict(id=f['id'],rgb_sha256=f['rgb']['sha256'],status='complete',error=None,detections=[])for f in frames if f['id']not in C.MISSING])
    predictions.write_text(json.dumps(original))
    workers=[]
    class FakeWorker:
        fail=False
        interrupt_on_close=False
        def __init__(self,*args):self.calls=[];self.closed=False;self.process=N(pid=987654321);self.start_ticks=1;workers.append(self)
        def request(self,identifier,image_path):
            self.calls.append((identifier,image_path))
            if self.fail:raise RuntimeError('backend failed')
            return dict(id=identifier,detections=[],metadata={'source_config_sha256':identity['config_sha256']},inference_wall_sec=.01)
        def close(self):
            if self.interrupt_on_close:
                assert signal.getsignal(signal.SIGINT)==signal.SIG_IGN
                os.kill(os.getpid(),signal.SIGINT)
            self.closed=True
    monkeypatch.setattr(C,'Worker',FakeWorker)
    def gone(pid,signal):
        assert pid==987654321 and signal==0
        raise ProcessLookupError
    monkeypatch.setattr(C.os,'killpg',gone)
    return dataset,predictions,tmp_path,original,identity,workers,FakeWorker


def test_only_missing_three_are_inferred_and_existing_six_unchanged(setup):
    dataset,predictions,root,original,identity,workers,_=setup;before=predictions.read_bytes()
    r=C.complete(dataset,predictions,root,root/'new')
    output=json.loads((root/'new/predictions.json').read_text())
    assert output['frames'][:6]==original['frames']
    assert predictions.read_bytes()==before
    by_path={f['rgb']['path']:f['id']for f in json.loads(dataset.read_text())['frames']}
    assert {by_path[path]for _,path in workers[0].calls}==C.MISSING
    assert len(workers[0].calls)==3 and workers[0].closed
    assert r['worker_group_gone'] and r['status']=='complete' and len(output['frames'])==9


def test_changed_model_is_rejected_before_worker_start(setup,monkeypatch):
    dataset,predictions,root,original,identity,workers,_=setup
    identity['config_sha256']='f'*64
    monkeypatch.setattr(C,'provenance',lambda root:identity)
    with pytest.raises(ValueError,match='달라'):C.complete(dataset,predictions,root,root/'new')
    assert not workers and not (root/'new').exists()


def test_incomplete_review_is_rejected_before_inference(setup):
    dataset,predictions,root,_,_,workers,_=setup
    d=json.loads(dataset.read_text());d['frames'][0]['review_status']='draft';dataset.write_text(json.dumps(d))
    with pytest.raises(ValueError,match='검수'):C.complete(dataset,predictions,root,root/'new')
    assert not workers


def test_backend_failure_is_preserved_as_incomplete_not_empty_predictions(setup):
    dataset,predictions,root,original,_,workers,FakeWorker=setup;FakeWorker.fail=True
    with pytest.raises(RuntimeError,match='backend'):C.complete(dataset,predictions,root,root/'new')
    r=json.loads((root/'new/completion.json').read_text());out=json.loads((root/'new/predictions.json').read_text())
    assert r['status']=='failed' and r['not_completed_frames']==3
    assert out['frames']==original['frames'] and workers[0].closed


def test_existing_output_is_never_overwritten(setup):
    dataset,predictions,root,_,_,workers,_=setup
    out=root/'existing';out.mkdir();(out/'keep').write_text('보존')
    with pytest.raises(ValueError,match='덮어'):C.complete(dataset,predictions,root,out)
    assert not workers and (out/'keep').read_text()=='보존'



@pytest.mark.parametrize('fail',[False,True])
def test_second_sigint_during_cleanup_does_not_lose_records(setup,fail):
    dataset,predictions,root,_,_,workers,FakeWorker=setup
    FakeWorker.fail=fail;FakeWorker.interrupt_on_close=True
    previous=signal.getsignal(signal.SIGINT)
    if fail:
        with pytest.raises(RuntimeError,match='backend'):C.complete(dataset,predictions,root,root/'new')
    else:C.complete(dataset,predictions,root,root/'new')
    assert workers[0].closed and signal.getsignal(signal.SIGINT)==previous
    record=json.loads((root/'new/completion.json').read_text())
    assert record['status']==('failed'if fail else'complete')
