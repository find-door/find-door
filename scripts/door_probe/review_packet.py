"""기존 라벨 초안에 검출 실패·보조 시점의 실제 RGB를 검수용으로 추가한다."""
import argparse
from collections import Counter
from copy import deepcopy
import json
from pathlib import Path

from dataset_intake import digest, rgb, door
from label_review import export_html, read_manifest
from view_detect import verify_capture
from mapping_semantic.door_dataset import validate_dataset

FAMILY_GROUP='gazebo_door_probe_v1_family'
OLD_GROUP='gazebo_door_probe_v1_single_geometry'


def _verified_json(path, expected):
    if digest(path)!=expected:
        raise ValueError(f'부모 기록 해시 불일치: {path}')
    return json.loads(Path(path).read_text())


def build_packet(base_path, scene_path, view_path):
    document, base_sha=read_manifest(base_path)
    original=deepcopy(document)
    scene=json.loads(Path(scene_path).read_text())
    view=json.loads(Path(view_path).read_text())
    output=[];indexed={f['rgb']['sha256']:f for f in document['frames']};omitted=0
    for frame in document['frames']:
        info=rgb(Path(frame['rgb']['path']) if Path(frame['rgb']['path']).is_absolute()
                 else Path(base_path).resolve().parent/frame['rgb']['path'],frame['rgb']['sha256'])
        if (info['width'],info['height'])!=(frame['rgb']['width'],frame['rgb']['height']):
            raise ValueError('기존 인벤토리 RGB 크기 불일치')
        frame['rgb']=info
        if frame['environment']=='gazebo' and frame['group_id'] in (OLD_GROUP,FAMILY_GROUP):
            if frame['split']!='development':
                raise ValueError('기존 개발 장면을 학습·독립 시험 분할과 혼합할 수 없습니다')
            frame['group_id']=FAMILY_GROUP
    rows=[]
    if len(scene['runs'])!=3 or {r['condition']for r in scene['runs']}!={'closed','half','open'}:
        raise ValueError('세 금속문 각도의 기록이 필요합니다')
    for run in scene['runs']:
        source=_verified_json(run['path'],run['sha256'])
        if source['status']!=run['status'] or source['state']!=run['expected_state']:
            raise ValueError('장면 요약과 원문 상태 불일치')
        rows.append(dict(id=f'metal_{run["condition"]}_2m',info=run['rgb'],state=run['expected_state'],
                         capabilities=dict(depth=False,camera_info=False,tf=False),
                         context=dict(condition=run['condition'],distance_m=2,
                                      trial_status=run['status'],source_path=run['path'],source_sha256=run['sha256'],
                                      note='실행 실패 여부는 문 상태 정답이 아닙니다. 실제 depth/CameraInfo/TF는 이 RGB 항목에 보존되지 않았습니다.')))
    results=_verified_json(view['results_path'],view['results_sha256'])
    if results['status']!='complete' or len(results['frames'])!=6:
        raise ValueError('완료된 여섯 시점의 검출 원문이 필요합니다')
    expected={(state,camera)for state in ['closed','half','open']for camera in ['low','high']}
    if {(f['state'],f['view'])for f in results['frames']}!=expected:
        raise ValueError('촬영 시점 중복 또는 누락')
    captures={}
    for capture in view['captures']:
        source=_verified_json(capture['path'],capture['sha256'])
        verify_capture(source)
        if source['state'] in captures:raise ValueError('중복 촬영 상태')
        captures[source['state']]=source
    if set(captures)!={'closed','half','open'}:raise ValueError('촬영 상태 누락')
    for frame in results['frames']:
        source=captures[frame['state']]
        recorded=[f for f in source['frames']if f['view']==frame['view']]
        if len(recorded)!=1 or any(recorded[0][key]!=frame[key]for key in ['sha256','k','d','width','height','stamp_ns','frame_id']):
            raise ValueError('검출 입력과 촬영 원본 불일치')
        low=frame['view']=='low'
        rows.append(dict(id=f'metal_{frame["state"]}_3m_{frame["view"]}',info=dict(path=frame['rgb'],sha256=frame['sha256']),
                         state='closed'if frame['state']=='closed'else'open',
                         capabilities=dict(depth=False,camera_info=True,tf=low),
                         context=dict(condition=frame['state'],view=frame['view'],distance_m=3,
                                      camera_height_m=source['robot_camera']['translation'][2]if low else source['aux_position'][2],
                                      k=frame['k'],d=frame['d'],predictions=frame['doors'],
                                      source_path=view['results_path'],source_sha256=view['results_sha256'],
                                      pose_source='관측 시각 TF 기록'if low else'보조 정적 SDF 설정',
                                      note='예측 bbox는 검수 정답이 아닙니다. 새 문짝·출입구 bbox는 직접 작성합니다.')))
    for row in rows:
        info=rgb(Path(row['info']['path']),row['info']['sha256'])
        if info['sha256']in indexed:
            existing=indexed[info['sha256']]
            if (existing['environment']!='gazebo' or existing['group_id']!=FAMILY_GROUP
                    or existing['split']!='development' or len(existing['doors'])!=1
                    or existing['doors'][0]['state']!=row['state']):
                raise ValueError('중복 RGB의 환경·분할·문 상태가 새 개발 자료와 다릅니다')
            omitted+=1;continue
        output.append(dict(id=row['id'],environment='gazebo',group_id=FAMILY_GROUP,split='development',rgb=info,
                           capabilities=row['capabilities'],review_status='draft',doors=[door('door_1',row['state'])],
                           review_context=row['context']))
        indexed[info['sha256']]=output[-1]
    # 실패·시점 비교 영상을 먼저 보여주되 기존 라벨·ID·검수 기록은 보존한다.
    document['frames']=output+document['frames']
    document['dataset_id']='door-review-packet-v1'
    history=document.setdefault('review_packet_history',[])
    if not isinstance(history,list):raise ValueError('review_packet_history는 목록이어야 합니다')
    history.append(dict(base_sha256=base_sha,scene_summary_sha256=digest(scene_path),view_summary_sha256=digest(view_path),
                        original_group=OLD_GROUP,family_group=FAMILY_GROUP,added_frames=len(output),omitted_duplicate_rgb=omitted))
    result=validate_dataset(document)
    return result,dict(total_frames=len(result['frames']),added_frames=len(output),omitted_duplicate_rgb=omitted,
                       reviewed_dual_target_frames=sum(f['review_status']=='reviewed'for f in result['frames']),
                       environments=dict(Counter(f['environment']for f in result['frames'])),
                       base_frames=len(original['frames']),provenance=history[-1])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base',type=Path,required=True)
    parser.add_argument('--scene-summary',type=Path,default=Path('docs/data/door-scene-variation-summary.json'))
    parser.add_argument('--view-summary',type=Path,default=Path('docs/data/door-view-diagnostic-summary.json'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('새 출력 폴더가 필요합니다')
    document,summary=build_packet(args.base,args.scene_summary,args.view_summary)
    args.output.mkdir(parents=True)
    manifest=args.output/'dataset.json';manifest.write_text(json.dumps(document,ensure_ascii=False,indent=2)+'\n')
    export_html(manifest,args.output/'review.html')
    summary.update(dataset_path=str(manifest.resolve()),dataset_sha256=digest(manifest),html_path=str((args.output/'review.html').resolve()),
                   html_sha256=digest(args.output/'review.html'),packet_tool_sha256=digest(__file__))
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
