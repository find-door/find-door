"""낮은/보조 시점의 새 RGB 쌍을 같은 기존 YOLOE worker로 비교한다."""
import argparse
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace as N

import numpy as np
from PIL import Image
from view_capture import validate_views

from capture import digest, write_json
from evaluate import provenance

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'src/mapping_semantic'))
from mapping_semantic.live_detector import Worker, validate_reply  # noqa: E402


def verify_capture(document):
    """저장된 두 시점 메타데이터와 원본을 최종 비교 계약으로 다시 검사한다."""
    if document['status'] != 'complete' or len(document['frames']) != 2:
        raise ValueError('완료된 두 시점 촬영이 필요합니다')
    frames = {frame['view']: frame for frame in document['frames']}
    if set(frames) != {'low', 'high'}:
        raise ValueError('낮은/높은 시점 쌍이 필요합니다')
    messages=[]
    for name in ['low', 'high']:
        frame=frames[name]
        if digest(frame['rgb']) != frame['sha256']:
            raise ValueError('원본 RGB 해시 불일치')
        with Image.open(frame['rgb']) as image:
            if image.size != (frame['width'], frame['height']):
                raise ValueError('원본 RGB 크기 불일치')
        ns=frame['stamp_ns']
        header=N(frame_id=frame['frame_id'],stamp=N(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000))
        rgb=N(header=header,width=frame['width'],height=frame['height'])
        info=N(header=header,width=frame['width'],height=frame['height'],k=frame['k'],d=frame['d'])
        messages.append((rgb,info))
    bounds=validate_views(*messages,document['robot_camera'])
    for name,bound in zip(['low','high'],bounds):
        if not np.allclose(bound,frames[name]['frame_bounds_unclipped'],rtol=0,atol=1e-6):
            raise ValueError('문틀 기하 투영 기록 불일치')
    if document['pair_stamp_delta_ns'] != abs(frames['low']['stamp_ns']-frames['high']['stamp_ns']):
        raise ValueError('시각 차이 기록 불일치')
    return True


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--captures',type=Path,nargs=3,required=True)
    parser.add_argument('--recon-root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('새 출력 폴더가 필요합니다')
    frames=[];seen=set();sources=[]
    for directory in args.captures:
        path=directory.resolve()/'capture.json';d=json.loads(path.read_text())
        if d['status']!='complete' or d['state'] in seen or d['state'] not in ['closed','half','open']:
            parser.error('서로 다른 세 상태의 완료된 촬영 결과가 필요합니다')
        verify_capture(d)
        seen.add(d['state']);sources.append(dict(path=str(path),sha256=digest(path)))
        if len(d['frames'])!=2 or {f['view']for f in d['frames']}!={'low','high'}:parser.error('시점 쌍이 필요합니다')
        for frame in d['frames']:
            if digest(frame['rgb'])!=frame['sha256']:parser.error('입력 RGB 해시 불일치')
            frames.append(dict(frame,state=d['state']))
    args.output=args.output.resolve();args.output.mkdir(parents=True)
    runtime=args.output/'worker';runtime.mkdir()
    identity=provenance(args.recon_root.resolve())
    result=dict(status='running',expected_frames=6,frames=[],captures=sources,recon=identity,
                source_sha256=digest(__file__),view_validator_sha256=digest(Path(__file__).with_name('view_capture.py')),worker_source_sha256=digest(ROOT/'src/mapping_semantic/mapping_semantic/detector_worker.py'))
    worker=None
    try:
        worker=Worker(args.recon_root,identity['config_path'],runtime,60)
        result['worker_pgid']=worker.process.pid
        result['worker_start_ticks']=worker.start_ticks
        for identifier,frame in enumerate(frames):
            if digest(frame['rgb'])!=frame['sha256']:raise ValueError('추론 전 입력 RGB 변경')
            raw=worker.request(identifier,frame['rgb'])
            write_json(args.output/f'{frame["state"]}-{frame["view"]}-raw.json',raw)
            detections=validate_reply(raw,identifier,frame['width'],frame['height'])
            if raw['metadata']['source_config_sha256']!=identity['config_sha256']:raise ValueError('추론 설정 해시 불일치')
            doors=[d for d in detections if d['class']=='door' and d['confidence']>=.1]
            result['frames'].append(dict(frame,status='complete',doors=doors,door_count=len(doors),door_present=bool(doors),metadata=raw['metadata']))
        result['status']='complete'
    except BaseException as exc:
        result.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        if worker is not None:
            try:
                worker.close()
                try:os.killpg(worker.process.pid,0)
                except ProcessLookupError:result['worker_group_gone']=True
                else:raise RuntimeError('검출 worker 그룹 정리 지연')
            except Exception as exc:
                result.update(status='failed',cleanup_error=str(exc))
        result['not_completed_frames']=6-len(result['frames'])
        write_json(args.output/'results.json',result)


if __name__=='__main__':main()
