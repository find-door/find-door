"""학습 출입구 후보의 자산·입력·응답·종료 경계를 ML/GPU 없이 검사한다."""

import hashlib
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace

from PIL import Image
import pytest

from mapping_semantic import yolo11_doorway as backend


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def config_fixture(tmp_path, **changes):
    checkpoint = tmp_path/'weights.pt'
    checkpoint.write_bytes(b'local test checkpoint, not a model')
    config = dict(schema_version=1, backend='yolo11', checkpoint='weights.pt',
        checkpoint_sha256=sha(checkpoint), training_target='doorway', ultralytics_version='8.4.191',
        device='cpu', imgsz=640, confidence=.1, iou=.45, max_det=300)
    config.update(changes)
    path = tmp_path/'detector.json'
    path.write_text(json.dumps(config))
    return path, checkpoint, config


def png(mode='RGB', size=(100, 80), orientation=None, format='PNG'):
    output = io.BytesIO()
    image = Image.new(mode, size)
    options = {}
    if orientation is not None:
        exif = image.getexif()
        exif[274] = orientation
        options['exif'] = exif
    image.save(output, format=format, **options)
    return output.getvalue()


class Array:
    def __init__(self, values):
        self.values = values
    def cpu(self):
        return self
    def tolist(self):
        return self.values


def result(boxes=None, scores=None, classes=None, names=None):
    return SimpleNamespace(names={0: 'doorway'} if names is None else names,
        boxes=SimpleNamespace(xyxy=Array([[1, 2, 30, 60]] if boxes is None else boxes),
            conf=Array([.5] if scores is None else scores), cls=Array([0.] if classes is None else classes)))


class Model:
    def __init__(self, results=None, names=None):
        self.names = {0:'doorway'} if names is None else names
        self.results = [result()] if results is None else results
        self.calls = []
        self.error = None
    def predict(self, **kwargs):
        self.calls.append(kwargs)
        if self.error:
            raise self.error
        return self.results


@pytest.fixture(autouse=True)
def isolated_environment(monkeypatch):
    # 검출기가 쓰는 환경은 원래 worker 전용이며 단위 검사에서는 각 검사 뒤 복원한다.
    for name in ('YOLO_CONFIG_DIR','YOLO_AUTOINSTALL','YOLO_OFFLINE'):
        monkeypatch.setenv(name, 'before-test')


def detector_fixture(tmp_path, monkeypatch, *, device='cpu', model=None):
    source, checkpoint, config = config_fixture(tmp_path, device=device)
    model = model or Model()
    events = []
    torch = SimpleNamespace(__version__='fake-torch', cuda=SimpleNamespace(
        synchronize=lambda index: events.append(('synchronize',index)),
        empty_cache=lambda: events.append(('empty_cache',))))
    loads = []
    def load(detector):
        loads.append(detector.config['checkpoint'])
        detector._model, detector._torch = model, torch
    monkeypatch.setattr(backend.metadata, 'version', lambda name: '8.4.191')
    monkeypatch.setattr(backend.YOLO11DoorwayDetector, '_load_model', load)
    detector = backend.YOLO11DoorwayDetector(source, tmp_path/'runtime')
    return detector, model, source, checkpoint, config, events, loads


def test_relative_assets_become_absolute_without_changing_source(tmp_path):
    source, checkpoint, _ = config_fixture(tmp_path)
    before = source.read_bytes(), checkpoint.read_bytes()
    config, checksum = backend.load_config(source)
    assert config['checkpoint'] == str(checkpoint.resolve())
    assert checksum == sha(source)
    assert (source.read_bytes(), checkpoint.read_bytes()) == before


@pytest.mark.parametrize('key,value', [
    ('schema_version', True), ('schema_version', 2), ('backend','yoloe'), ('training_target','leaf'),
    ('checkpoint',''), ('checkpoint_sha256','A'*64), ('ultralytics_version',''),
    ('device',0), ('device',True), ('device','1'), ('imgsz',True), ('imgsz',33),
    ('imgsz',0), ('imgsz',1312), ('max_det',True), ('max_det',0), ('max_det',301),
    ('confidence',True), ('confidence',float('nan')), ('confidence',float('inf')),
    ('confidence',-.1), ('confidence',1.1), ('iou',0), ('iou',False), ('iou',1.1)])
def test_invalid_config_rejected_before_loading(tmp_path,key,value):
    source,_,_=config_fixture(tmp_path, **{key:value})
    with pytest.raises(ValueError):
        backend.load_config(source)


def test_missing_extra_and_duplicate_keys_rejected(tmp_path):
    source,_,config=config_fixture(tmp_path)
    del config['max_det']
    source.write_text(json.dumps(config))
    with pytest.raises(ValueError,match='키'):
        backend.load_config(source)
    config['max_det']=300;config['download']='never'
    source.write_text(json.dumps(config))
    with pytest.raises(ValueError,match='키'):
        backend.load_config(source)
    source.write_text('{"backend":"yolo11","backend":"yoloe"}')
    with pytest.raises(ValueError,match='중복'):
        backend.load_config(source)


def test_checkpoint_mismatch_and_missing_asset_are_not_downloaded(tmp_path):
    source,checkpoint,_=config_fixture(tmp_path)
    checkpoint.write_bytes(b'changed')
    with pytest.raises(ValueError,match='해시'):
        backend.load_config(source)
    checkpoint.unlink()
    with pytest.raises(ValueError,match='로컬 PyTorch'):
        backend.load_config(source)


def test_other_backend_file_is_not_loaded_as_pytorch_checkpoint(tmp_path):
    source,checkpoint,config=config_fixture(tmp_path)
    alternate=tmp_path/'model.onnx';checkpoint.rename(alternate)
    config['checkpoint']=alternate.name
    source.write_text(json.dumps(config))
    with pytest.raises(ValueError,match='PyTorch .pt'):
        backend.load_config(source)


def test_installed_version_mismatch_precedes_model_loading(tmp_path,monkeypatch):
    source,_,_=config_fixture(tmp_path)
    monkeypatch.setattr(backend.metadata,'version',lambda name:'8.4.0')
    monkeypatch.setattr(backend.YOLO11DoorwayDetector,'_load_model',lambda self:pytest.fail('모델 적재 금지'))
    with pytest.raises(ValueError,match='설치 버전'):
        backend.YOLO11DoorwayDetector(source,tmp_path/'runtime')


def test_model_is_kept_and_original_pixel_geometry_and_source_class_are_preserved(tmp_path,monkeypatch):
    detector,model,source,checkpoint,_,events,loads=detector_fixture(tmp_path,monkeypatch)
    original=source.read_bytes(),checkpoint.read_bytes()
    image=png()
    try:
        a=detector.detect(image);b=detector.detect(image)
        assert a==b and len(loads)==1 and len(model.calls)==2
        assert a[0]['bbox']==[1.,2.,30.,60.]
        assert a[0]['class']==a[0]['report_class']=='door'
        assert a[0]['source_label']=='doorway' and a[0]['source_class_id']==0
        assert model.calls[0]['source'].size==(100,80)
        assert model.calls[0]['conf']==.1 and model.calls[0]['save']is False
        assert model.calls[0]['half']is False and model.calls[0]['augment']is False
        assert os.environ['YOLO_AUTOINSTALL']=='false' and os.environ['YOLO_OFFLINE']=='true'
        assert Path(os.environ['YOLO_CONFIG_DIR'])==tmp_path/'runtime/ultralytics-config'
        assert detector.last_run['input_sha256']==hashlib.sha256(image).hexdigest()
        assert detector.last_run['checkpoint_sha256']==sha(checkpoint)
        assert detector.last_run['training_target']=='doorway' and detector.last_run['success']is True
        assert detector.last_run['torch_version']=='fake-torch'
        assert (source.read_bytes(),checkpoint.read_bytes())==original
    finally:
        detector.close()
    assert not events  # CPU 경로에서 CUDA를 건드리지 않는다.


def test_empty_success_is_not_runtime_failure(tmp_path,monkeypatch):
    model=Model([result(boxes=[],scores=[],classes=[])])
    detector,*_=detector_fixture(tmp_path,monkeypatch,model=model)
    try:
        assert detector.detect(png())==[]
        assert detector.last_run['success']is True
        model.error=RuntimeError('prediction failed')
        with pytest.raises(RuntimeError,match='prediction failed'):
            detector.detect(png())
        assert detector.last_run['success']is False and 'prediction failed'in detector.last_run['error']
    finally:
        detector.close()


@pytest.mark.parametrize('changed',['config','checkpoint'])
def test_runtime_identity_change_is_rejected_before_predict(tmp_path,monkeypatch,changed):
    detector,model,source,checkpoint,*_=detector_fixture(tmp_path,monkeypatch)
    try:
        detector.detect(png())
        (source if changed=='config'else checkpoint).write_bytes(b'changed')
        with pytest.raises(ValueError,match='변경'):
            detector.detect(png())
        assert len(model.calls)==1 and detector.last_run['success']is False
    finally:
        detector.close()


@pytest.mark.parametrize('image',[b'', b'invalid', png('L'), png(format='JPEG'), png(orientation=6)])
def test_bad_images_do_not_reach_model(tmp_path,monkeypatch,image):
    detector,model,*_=detector_fixture(tmp_path,monkeypatch)
    try:
        with pytest.raises((ValueError,OSError)):
            detector.detect(image)
        assert not model.calls and detector.last_run['success']is False
    finally:
        detector.close()


def test_multiframe_and_oversized_png_are_rejected(tmp_path,monkeypatch):
    detector,model,*_=detector_fixture(tmp_path,monkeypatch)
    try:
        output=io.BytesIO();a=Image.new('RGB',(10,10),'red');b=Image.new('RGB',(10,10),'blue')
        a.save(output,format='PNG',save_all=True,append_images=[b])
        with pytest.raises(ValueError,match='단일 RGB'):
            detector.detect(output.getvalue())
        monkeypatch.setattr(backend,'MAX_PIXELS',20)
        with pytest.raises(ValueError,match='단일 RGB'):
            detector.detect(png())
        assert not model.calls
    finally:
        detector.close()


@pytest.mark.parametrize('names',[{0:'person'},{0:'doorway',1:'person'},{False:'doorway'},['doorway']])
def test_checkpoint_class_contract_is_checked_at_load(tmp_path,monkeypatch,names):
    with pytest.raises(ValueError,match='클래스'):
        detector_fixture(tmp_path,monkeypatch,model=Model(names=names))


@pytest.mark.parametrize('bad',[
    result(classes=[1]),result(classes=[.5]),result(scores=[float('nan')]),result(scores=[True]),
    result(scores=[1.1]),result(boxes=[[0,0,101,40]]),result(boxes=[[3,2,1,40]]),
    result(boxes=[[0,0,float('inf'),20]]),result(boxes=[[True,0,30,40]]),
    result(scores=[]),result(names={0:'person'})])
def test_invalid_predictions_are_not_clamped_or_silently_empty(tmp_path,monkeypatch,bad):
    detector,model,*_=detector_fixture(tmp_path,monkeypatch,model=Model([bad]))
    try:
        with pytest.raises(ValueError):
            detector.detect(png())
        assert len(model.calls)==1 and detector.last_run['success']is False
    finally:
        detector.close()


def test_missing_or_multiple_results_and_excess_detections_rejected():
    for results in ([],[result(),result()]):
        with pytest.raises(ValueError,match='하나'):
            backend.extract_detections(results,100,80,.1,300)
    with pytest.raises(ValueError,match='max_det'):
        backend.extract_detections([result(boxes=[[1,2,30,60]]*2,scores=[.3,.2],classes=[0,0])],100,80,.1,1)


def test_confidence_boundary_and_low_score_filter():
    outputs=backend.extract_detections([result(boxes=[[1,2,30,60]]*2,scores=[.1,.099],classes=[0,0])],100,80,.1,300)
    assert len(outputs)==1 and outputs[0]['confidence']==.1


def test_close_is_idempotent_and_only_own_cuda_context_is_released(tmp_path,monkeypatch):
    detector,model,_,_,_,events,_=detector_fixture(tmp_path,monkeypatch,device='0')
    detector.detect(png())
    detector.close();detector.close()
    assert events==[('synchronize',0),('synchronize',0),('synchronize',0),('empty_cache',)]
    assert detector._model is None and detector.closed is True
    with pytest.raises(RuntimeError,match='종료'):
        detector.detect(png())
    assert len(model.calls)==1
