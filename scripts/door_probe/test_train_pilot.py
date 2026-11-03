"""학습 하네스의 입력 보호와 추론 오류·출처 계약을 GPU 없이 확인한다."""
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import pytest

from train_pilot import infer, run, verify_inputs, verify_image_inventory


class Array:
    def __init__(self, data):
        self.data=data
    def cpu(self):
        return self
    def tolist(self):
        return self.data


def frame(tmp_path, identifier):
    path=tmp_path/(identifier+'.png')
    Image.new('RGB',(100,80)).save(path)
    return dict(id=identifier,rgb=dict(path=str(path),sha256='a'*64))


def result(boxes=(),scores=(),classes=(),name='doorway'):
    return SimpleNamespace(boxes=SimpleNamespace(xyxy=Array(list(boxes)),conf=Array(list(scores)),cls=Array(list(classes))),names={0:name,1:'other'})


class Model:
    def __init__(self, outputs):
        self.outputs=iter(outputs)
    def predict(self, **kwargs):
        item=next(self.outputs)
        if isinstance(item,Exception):
            raise item
        return item


def test_valid_empty_inference_differs_from_runtime_error(tmp_path):
    frames=[frame(tmp_path,'empty'),frame(tmp_path,'failed')]
    rows=infer(Model([[result()],RuntimeError('CUDA failure')]),frames,{},dict(training_target='doorway'))['frames']
    assert rows[0]['status']=='complete' and rows[0]['detections']==[] and rows[0]['error']is None
    assert rows[1]['status']=='runtime_error' and rows[1]['detections']is None and 'CUDA failure'in rows[1]['error']


def test_source_class_is_preserved_when_adapting_to_evaluator(tmp_path):
    rows=infer(Model([[result([[1,2,40,70]],[.6],[0])]]),[frame(tmp_path,'door')],{},dict(training_target='doorway'))['frames']
    detection=rows[0]['detections'][0]
    assert detection==dict(**{'class':'door'},confidence=.6,bbox=[1,2,40,70],source_class_id=0,source_label='doorway')


@pytest.mark.parametrize('prediction',[[result([[1,2,40,70]],[.6],[1])],[]])
def test_invalid_model_result_is_not_silently_empty(tmp_path,prediction):
    row=infer(Model([prediction]),[frame(tmp_path,'invalid')],{}, {})['frames'][0]
    assert row['status']=='runtime_error' and row['detections']is None


def test_checkpoint_hash_guard_precedes_training(tmp_path):
    checkpoint=tmp_path/'checkpoint.pt';checkpoint.write_bytes(b'changed')
    with pytest.raises(ValueError,match='checkpoint 해시'):
        verify_inputs(dict(datasets=dict(leaf={}),checkpoint=dict(path=str(checkpoint),sha256='0'*64)),'leaf')


def test_existing_run_cannot_be_overwritten(tmp_path):
    output=tmp_path/'run';output.mkdir();marker=output/'keep';marker.write_text('original')
    with pytest.raises(FileExistsError):
        run(Path('nonexistent-protocol.json'),'leaf',output)
    assert marker.read_text()=='original'


def inventory(tmp_path):
    rows=[]
    for split,folder in [('train','train'),('validation','val')]:
        directory=tmp_path/'images'/folder
        directory.mkdir(parents=True)
        path=directory/'known.png'
        Image.new('RGB',(10,10)).save(path)
        rows.append(dict(split=split,image=str(path)))
    return rows


def test_exact_image_inventory_is_accepted(tmp_path):
    rows=inventory(tmp_path)
    verify_image_inventory(tmp_path,rows)


@pytest.mark.parametrize('suffix',['.png','.bmp','.tif','.json'])
def test_unlisted_files_are_rejected_regardless_of_image_extension(tmp_path,suffix):
    rows=inventory(tmp_path)
    (tmp_path/'images/train'/('unexpected'+suffix)).write_bytes(b'extra')
    with pytest.raises(ValueError,match='이미지 추가'):
        verify_image_inventory(tmp_path,rows)


def test_unlisted_broken_symlink_is_also_rejected(tmp_path):
    rows=inventory(tmp_path)
    (tmp_path/'images/val/unexpected.heic').symlink_to(tmp_path/'missing.heic')
    with pytest.raises(ValueError,match='이미지 추가'):
        verify_image_inventory(tmp_path,rows)
