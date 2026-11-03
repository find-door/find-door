"""고정 AI 개발 입력으로 YOLO11을 학습하고 같은 RGB의 검출을 기록한다."""
import argparse
import gc
from datetime import datetime, timezone
import importlib.metadata
import json
import math
import os
import signal
from pathlib import Path
import sys
import time
import traceback

from PIL import Image
import yaml

from dataset_intake import digest
from label_review import read_manifest
from label_evaluate import validate_predictions, verify_rgb


ROOT = Path(__file__).resolve().parents[2]


def write(path, value):
    with Path(path).open('x') as handle:
        handle.write(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def verify_image_inventory(folder, rows):
    """확장자와 무관하게 고정 images 목록 밖 파일·symlink를 거부한다."""
    for split, directory in [('train','train'),('validation','val')]:
        expected = {Path(r['image']).absolute() for r in rows if r['split']==split}
        actual = {p.absolute() for p in (folder/'images'/directory).rglob('*') if p.is_file() or p.is_symlink()}
        if expected != actual:
            raise ValueError('고정 입력 밖의 이미지 추가·누락을 허용하지 않습니다')


def verify_inputs(protocol, target):
    if target not in ('leaf', 'doorway') or target not in protocol['datasets']:
        raise ValueError('학습 target을 명시해야 합니다')
    checkpoint = Path(protocol['checkpoint']['path'])
    if digest(checkpoint) != protocol['checkpoint']['sha256']:
        raise ValueError('초기 checkpoint 해시 불일치')
    config = protocol['datasets'][target]
    folder = Path(config['path'])
    for name, key in [('manifest.json','manifest_sha256'),('dataset.yaml','yaml_sha256'),('export-report.json','report_sha256')]:
        if digest(folder/name) != config[key]:
            raise ValueError('고정 학습 입력 해시 불일치: '+name)
    dataset, _ = read_manifest(folder/'manifest.json')
    report = json.loads((folder/'export-report.json').read_text())
    actual = yaml.safe_load((folder/'dataset.yaml').read_text())
    if ('test' in actual or 'download' in actual or actual.get('names') != {0:'door_leaf' if target=='leaf' else 'doorway'}
            or actual.get('path') != str(folder) or actual.get('train') != 'images/train' or actual.get('val') != 'images/val'):
        raise ValueError('고정 단일 클래스·로컬 학습/검증 YAML 필요')
    if report['target'] != target or len(dataset['frames']) != 39 or len(report['frames']) != 39:
        raise ValueError('학습 target·프레임 수 불일치')
    rows = {r['id']:r for r in report['frames']}
    if len(rows) != 39 or set(rows) != {f['id'] for f in dataset['frames']}:
        raise ValueError('학습 manifest·라벨 목록 불일치')
    verify_image_inventory(folder, report['frames'])
    train, val = set(), set()
    for frame in dataset['frames']:
        if frame['environment'] != 'real_rgb' or frame['annotation_review']['human_reviewed'] is not False:
            raise ValueError('현재 파일은 AI 실제 RGB 개발 자료만 사용합니다')
        if frame['review_status'] != 'reviewed' or frame['annotation_uncertain'] or any(d[target+'_visibility'] != 'visible' for d in frame['doors']):
            raise ValueError('불확실 라벨·target 가시성 불명은 학습에서 제외해야 합니다')
        verify_rgb(folder/'manifest.json',frame)
        row = rows[frame['id']]
        if digest(row['label']) != row['label_sha256'] or digest(row['image']) != frame['rgb']['sha256']:
            raise ValueError('학습 라벨·이미지 해시 불일치')
        (train if frame['split']=='train' else val).add(frame['group_id'])
    if train & val or sorted(f['id'] for f in dataset['frames'] if f['split']=='validation') != sorted(protocol['validation_frame_ids']):
        raise ValueError('고정 그룹·검증 RGB 불일치')
    gz_path = Path(protocol['gazebo']['manifest_path'])
    if digest(gz_path) != protocol['gazebo']['manifest_sha256']:
        raise ValueError('고정 Gazebo 개발 입력 해시 불일치')
    gazebo, _ = read_manifest(gz_path)
    gz = [f for f in gazebo['frames'] if f['environment']=='gazebo' and f['review_status']=='reviewed']
    if len(gz) != 9:
        raise ValueError('사용자 검수 Gazebo 개발 자료는 9장이어야 합니다')
    for frame in gz:
        verify_rgb(gz_path,frame)
    return dataset, gazebo


def infer(model, frames, config, provenance):
    result = dict(schema_version=1,provenance=provenance,frames=[])
    for frame in frames:
        row = dict(id=frame['id'],rgb_sha256=frame['rgb']['sha256'],status='complete',error=None,detections=[])
        try:
            with Image.open(frame['rgb']['path']) as image:
                predictions = model.predict(source=image.convert('RGB'),verbose=False,save=False,device=0,**config)
            if len(predictions) != 1:
                raise ValueError('한 영상당 하나의 검출 결과 필요')
            prediction = predictions[0]
            for box, score, cls in zip(prediction.boxes.xyxy.cpu().tolist(),prediction.boxes.conf.cpu().tolist(),prediction.boxes.cls.cpu().tolist()):
                if int(cls) != 0 or not math.isfinite(score):
                    raise ValueError('단일 클래스 검출 계약 오류')
                row['detections'].append(dict(**{'class':'door'},confidence=score,bbox=box,
                    source_class_id=int(cls),source_label=prediction.names[int(cls)]))
        except Exception as exc:
            row.update(status='runtime_error',error=f'{type(exc).__name__}: {exc}',detections=None)
        result['frames'].append(row)
    return result


def run(protocol_path, target, output):
    output = output.resolve()
    if output.exists():
        raise FileExistsError('기존 학습 실행은 덮어쓰지 않습니다')
    protocol = json.loads(protocol_path.read_text())
    dataset, gazebo = verify_inputs(protocol,target)
    if os.environ.get('YOLO_AUTOINSTALL') != 'false' or os.environ.get('YOLO_OFFLINE') != 'true':
        raise ValueError('자동 설치·다운로드를 끈 전용 환경이 필요합니다')
    for name, version in protocol['versions'].items():
        if importlib.metadata.version(name) != version:
            raise ValueError('고정 라이브러리 버전 불일치: '+name)
    output.mkdir(parents=True,exist_ok=False)
    config_dir = output/'ultralytics-config'
    config_dir.mkdir()
    os.environ['YOLO_CONFIG_DIR'] = str(config_dir)
    # 기존 시스템 폰트를 사용해 라이브러리의 부수적 폰트 다운로드를 방지한다.
    font = Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font_dir = config_dir/'Ultralytics'
    font_dir.mkdir()
    (font_dir/'Arial.ttf').symlink_to(font)
    started = time.monotonic()
    record = dict(status='running',target=target,started_at=datetime.now(timezone.utc).isoformat(),
        pid=os.getpid(),pgid=os.getpgrp(),protocol_sha256=digest(protocol_path),harness_sha256=digest(__file__),
        font_alias=dict(path=str(font),sha256=digest(font)),completed_epochs=[],batch_events=0)
    write(output/'started.json',record)
    try:
        import torch
        import ultralytics
        from ultralytics import YOLO, settings
        settings.update({k:False for k in ('sync','clearml','comet','dvc','mlflow','raytune','tensorboard','wandb')})
        if not torch.cuda.is_available():
            raise RuntimeError('명시한 CUDA 학습 장치를 사용할 수 없습니다')
        torch.set_num_threads(4)
        torch.cuda.set_device(0)
        torch.cuda.init()
        torch.cuda.reset_peak_memory_stats(0)
        record['gpu_name'] = torch.cuda.get_device_name(0)
        record['versions'] = dict(torch=torch.__version__,ultralytics=ultralytics.__version__)
        model = YOLO(protocol['checkpoint']['path'])
        def epoch_end(trainer):
            epoch = int(trainer.epoch)+1
            record['completed_epochs'].append(epoch)
            event = dict(epoch=epoch,elapsed_seconds=time.monotonic()-started,batch_size=trainer.batch_size,
                         allocated_bytes=torch.cuda.max_memory_allocated(0),reserved_bytes=torch.cuda.max_memory_reserved(0))
            with (output/'epochs.jsonl').open('a') as handle:
                handle.write(json.dumps(event)+'\n')
        def batch_end(trainer):
            record['batch_events'] += 1
        model.add_callback('on_train_epoch_end',epoch_end)
        model.add_callback('on_train_batch_end',batch_end)
        args = dict(protocol['training'])
        model.train(data=str(Path(protocol['datasets'][target]['path'])/'dataset.yaml'),
                    project=str(output/'runs'),name=target,exist_ok=False,**args)
        record['training_seconds'] = time.monotonic()-started
        record['training_peak_allocated_bytes'] = torch.cuda.max_memory_allocated(0)
        record['training_peak_reserved_bytes'] = torch.cuda.max_memory_reserved(0)
        if record['completed_epochs'] != list(range(1,args['epochs']+1)):
            raise RuntimeError('계획한 전체 epoch가 완료되지 않았습니다')
        best = Path(model.trainer.best)
        record['best_checkpoint'] = str(best)
        record['best_checkpoint_sha256'] = digest(best)
        record['last_checkpoint_sha256'] = digest(model.trainer.last)
        record['args_sha256'] = digest(best.parent.parent/'args.yaml')
        record['results_csv_sha256'] = digest(best.parent.parent/'results.csv')
        record['best_fitness'] = float(model.trainer.best_fitness)
        del model
        gc.collect()
        torch.cuda.empty_cache()
        model = YOLO(str(best))
        provenance = dict(model='YOLO11n_finetuned',training_target=target,checkpoint_sha256=digest(best),
                          protocol_sha256=digest(protocol_path),reference_kind='development_only',inference=protocol['inference'])
        real = infer(model,[f for f in dataset['frames'] if f['split']=='validation'],protocol['inference'],provenance)
        gz = infer(model,[f for f in gazebo['frames'] if f['environment']=='gazebo' and f['review_status']=='reviewed'],protocol['inference'],provenance)
        validate_predictions(real,dataset)
        validate_predictions(gz,gazebo)
        write(output/'validation-predictions.json',real)
        write(output/'gazebo-predictions.json',gz)
        record.update(status='complete',validation_predictions_sha256=digest(output/'validation-predictions.json'),
                      gazebo_predictions_sha256=digest(output/'gazebo-predictions.json'),
                      inference_errors=sum(f['status']=='runtime_error'for f in real['frames']+gz['frames']))
    except BaseException as exc:
        record.update(status='interrupted'if isinstance(exc,KeyboardInterrupt)else'failed',
                      error=f'{type(exc).__name__}: {exc}',traceback=traceback.format_exc())
        raise
    finally:
        handler = signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            record.update(elapsed_seconds=time.monotonic()-started,finished_at=datetime.now(timezone.utc).isoformat())
            write(output/'completion.json',record)
        finally:
            signal.signal(signal.SIGINT,handler)
    return record


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--protocol',type=Path,required=True)
    parser.add_argument('--target',choices=['leaf','doorway'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    record=run(args.protocol,args.target,args.output)
    print(json.dumps(dict(status=record['status'],target=args.target,epochs=len(record['completed_epochs']),
                          seconds=record['elapsed_seconds'],inference_errors=record['inference_errors']),ensure_ascii=False))


if __name__=='__main__':
    main()
