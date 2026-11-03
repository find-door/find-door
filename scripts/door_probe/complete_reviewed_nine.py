"""검수한 금속문 9장 중 기존 6장 예측을 보존하고 빠진 2m 3장만 추론한다."""
import argparse
from copy import deepcopy
import json
import os
import signal
from pathlib import Path
import sys

from dataset_intake import digest
from evaluate import provenance
from label_review import read_manifest
from label_evaluate import validate_predictions,verify_rgb
from mapping_semantic.live_detector import Worker,validate_reply

EXPECTED={f'metal_{state}_2m'for state in ['closed','half','open']}|{
    f'metal_{state}_3m_{view}'for state in ['closed','half','open']for view in ['low','high']}
MISSING={f'metal_{state}_2m'for state in ['closed','half','open']}


def complete(dataset_path,prediction_path,recon_root,output):
    output=Path(output)
    if output.exists():raise ValueError('기존 추론 결과를 덮어쓰지 않습니다')
    dataset,dataset_sha=read_manifest(dataset_path)
    source=json.loads(Path(prediction_path).read_text())
    indexed=validate_predictions(source,dataset)
    reviewed={f['id']:f for f in dataset['frames']if f['review_status']=='reviewed'}
    if set(reviewed)!=EXPECTED or set(indexed)!=(EXPECTED-MISSING):
        raise ValueError('검수 9장과 기존 3m 6장만 허용하는 고정 개발 진단입니다')
    if any(f['environment']!='gazebo' or f['split']!='development'for f in reviewed.values()):
        raise ValueError('Gazebo development 자료만 허용합니다')
    if any(row['status']!='complete'for row in indexed.values()):
        raise ValueError('기존 추론 오류를 자동 재시도하지 않습니다')
    for frame in reviewed.values():verify_rgb(dataset_path,frame)
    identity=provenance(Path(recon_root).resolve())
    prior=source['provenance']['recon']
    if (identity['config_sha256']!=prior['config_sha256']
            or identity['sources_sha256']!=prior['sources_sha256']
            or {k:v['sha256']for k,v in identity['assets'].items()}!={k:v['sha256']for k,v in prior['assets'].items()}):
        raise ValueError('기존 6장과 검출 설정·자산·소스가 달라 결과를 합치지 않습니다')
    output.mkdir(parents=True);runtime=output/'worker';runtime.mkdir()
    result=deepcopy(source)
    result['provenance']=dict(parent_path=str(Path(prediction_path).resolve()),parent_sha256=digest(prediction_path),
                              parent=deepcopy(source['provenance']),dataset_sha256=dataset_sha,recon=identity,
                              completion_script_sha256=digest(__file__))
    record=dict(status='running',new_frames=[],reused_frames=len(indexed),source_sha256=digest(__file__),
                worker_source_sha256=digest(Path(__file__).resolve().parents[2]/'src/mapping_semantic/mapping_semantic/detector_worker.py'))
    worker=None
    try:
        worker=Worker(recon_root,identity['config_path'],runtime,60)
        record.update(worker_pgid=worker.process.pid,worker_start_ticks=worker.start_ticks)
        for request_id,identifier in enumerate(sorted(MISSING)):
            frame=reviewed[identifier];verify_rgb(dataset_path,frame)
            reply=worker.request(request_id,frame['rgb']['path'])
            raw=output/f'raw-{request_id:02d}.json'
            raw.write_text(json.dumps(reply,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
            detections=validate_reply(reply,request_id,frame['rgb']['width'],frame['rgb']['height'])
            if reply['metadata']['source_config_sha256']!=identity['config_sha256']:
                raise ValueError('실행 중 검출 설정 변경')
            result['frames'].append(dict(id=identifier,rgb_sha256=frame['rgb']['sha256'],status='complete',error=None,detections=detections))
            record['new_frames'].append(dict(id=identifier,rgb_sha256=frame['rgb']['sha256'],raw_path=str(raw),raw_sha256=digest(raw),
                                             inference_wall_sec=reply['inference_wall_sec'],metadata=reply['metadata']))
        record['status']='complete'
    except BaseException as exc:
        record.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        previous=signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            if worker is not None:
                try:
                    worker.close()
                    try:os.killpg(worker.process.pid,0)
                    except ProcessLookupError:record['worker_group_gone']=True
                    else:raise RuntimeError('worker 그룹 정리 지연')
                except Exception as exc:record.update(status='failed',cleanup_error=str(exc))
            validate_predictions(result,dataset)
            (output/'predictions.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
            record['not_completed_frames']=len(EXPECTED)-len(result['frames'])
            (output/'completion.json').write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        finally:
            signal.signal(signal.SIGINT,previous)
    if record['status']!='complete':raise RuntimeError('추론·정리가 끝나지 않았습니다')
    return record


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset',type=Path,required=True);p.add_argument('--predictions',type=Path,required=True)
    p.add_argument('--recon-root',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();record=complete(args.dataset,args.predictions,args.recon_root,args.output)
    print(json.dumps(dict(status=record['status'],reused=record['reused_frames'],inferred=len(record['new_frames'])),ensure_ascii=False))


if __name__=='__main__':main()
