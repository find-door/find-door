"""검수한 문짝·출입구 bbox를 기존 검출 원문과 환경별로 따로 평가한다. 추론은 하지 않는다."""
import argparse
from collections import Counter
from copy import deepcopy
import json
import math
from pathlib import Path

from dataset_intake import digest, rgb
from label_review import read_manifest
from mapping_semantic.door_dataset import validate_dataset
from evaluate import iou, read_json


def _finite(value):
    try:return not isinstance(value,bool) and isinstance(value,(int,float)) and math.isfinite(value)
    except OverflowError:return False


def validate_predictions(document, dataset):
    if (not isinstance(document,dict) or type(document.get('schema_version'))is not int
            or document['schema_version']!=1 or not isinstance(document.get('frames'),list)
            or not isinstance(document.get('provenance'),dict)):
        raise ValueError('prediction schema_version=1, frames, provenance가 필요합니다')
    try:json.dumps(document['provenance'],allow_nan=False)
    except (ValueError,TypeError,OverflowError)as exc:raise ValueError('provenance는 유한한 JSON이어야 합니다')from exc
    frames={f['id']:f for f in dataset['frames']};result={}
    for row in document['frames']:
        if not isinstance(row,dict) or not isinstance(row.get('id'),str) or row['id']not in frames:
            raise ValueError('예측 프레임 ID가 자료와 다릅니다')
        if not {'id','rgb_sha256','status','error','detections'}<=set(row):
            raise ValueError('예측 프레임 필수 항목 누락')
        identifier=row['id'];frame=frames[identifier]
        if identifier in result:raise ValueError('중복 예측 ID')
        if row.get('rgb_sha256')!=frame['rgb']['sha256']:raise ValueError('예측 RGB 해시가 자료와 다릅니다')
        if row.get('status')=='runtime_error':
            if not isinstance(row.get('error'),str) or not row['error'].strip() or row.get('detections')is not None:
                raise ValueError('실행 오류에는 이유와 detections=null이 필요합니다')
            result[identifier]=row;continue
        if row.get('status')!='complete' or row.get('error')is not None or not isinstance(row.get('detections'),list):
            raise ValueError('완료된 검출 목록 또는 실행 오류가 필요합니다')
        for detection in row['detections']:
            if not isinstance(detection,dict) or detection.get('class')not in ('door','person'):
                raise ValueError('door/person 검출만 지원합니다')
            box=detection.get('bbox');score=detection.get('confidence')
            if (not _finite(score) or not 0<=score<=1 or not isinstance(box,list) or len(box)!=4
                    or not all(_finite(x)for x in box)
                    or not 0<=box[0]<box[2]<=frame['rgb']['width']
                    or not 0<=box[1]<box[3]<=frame['rgb']['height']):
                raise ValueError('검출 점수·원본 픽셀 bbox 범위 오류')
        result[identifier]=row
    return result


def maximum_matching(overlaps, threshold):
    """IoU 문턱을 넘는 간선의 최대 개수 일대일 대응. 점수 합 최적화나 AP가 아니다."""
    assigned={}
    def augment(truth,visited):
        order=sorted(range(len(overlaps[truth])),key=lambda p:(-overlaps[truth][p],p))
        for prediction in order:
            if overlaps[truth][prediction]<threshold or prediction in visited:continue
            visited.add(prediction)
            if prediction not in assigned or augment(assigned[prediction],visited):
                assigned[prediction]=truth;return True
        return False
    for truth in range(len(overlaps)):augment(truth,set())
    return sorted((truth,prediction)for prediction,truth in assigned.items())


def evaluate_dataset(dataset, predictions, environment, split, threshold=.5):
    if environment not in ('gazebo','real_rgb','real_rgbd') or split not in ('development','train','validation','test','unassigned'):
        raise ValueError('환경과 분할을 각각 명시해야 합니다')
    if not _finite(threshold) or not 0<threshold<=1:raise ValueError('IoU 문턱 범위 오류')
    dataset=validate_dataset(dataset)
    indexed=validate_predictions(predictions,dataset)
    selected=[f for f in dataset['frames']if f['environment']==environment and f['split']==split]
    result=dict(environment=environment,declared_split=split,iou_threshold=threshold,
                dataset_total_frames=len(dataset['frames']),selected_frames=len(selected),
                selected_prediction_frames=sum(f['id']in indexed for f in selected),
                reviewed_selected_frames=sum(f['review_status']=='reviewed'for f in selected),
                selected_frames_without_prediction=sum(f['id']not in indexed for f in selected),
                prediction_provenance=deepcopy(predictions['provenance']),
                independence_status='not_verified_by_this_tool',
                prediction_role='same_raw_door_boxes_compared_to_both_references',
                matching_policy='maximum_cardinality_iou_threshold',targets={})
    for target in ('leaf','doorway'):
        counts=Counter();rows=[];matched_ious=[]
        for frame in selected:
            row=dict(id=frame['id'],group_id=frame['group_id'],matches=[])
            if frame['review_status']!='reviewed':
                row['status']='excluded_draft';counts['draft_frames']+=1;rows.append(row);continue
            if any(d[f'{target}_visibility']!='visible'for d in frame['doors']):
                row['status']='excluded_visibility';counts['excluded_visibility_frames']+=1;rows.append(row);continue
            truth=[d[f'{target}_bbox']for d in frame['doors']]
            counts['eligible_frames']+=1;counts['eligible_gt_objects']+=len(truth)
            predicted=indexed.get(frame['id'])
            if predicted is None:
                row['status']='missing_prediction';counts['missing_predictions']+=1;rows.append(row);continue
            if predicted['status']=='runtime_error':
                row.update(status='runtime_error',error=predicted['error']);counts['runtime_errors']+=1;rows.append(row);continue
            boxes=[(index,d['bbox'])for index,d in enumerate(predicted['detections'])if d['class']=='door']
            overlaps=[[iou(a,b)for _,b in boxes]for a in truth]
            matches=maximum_matching(overlaps,threshold)
            tp=len(matches);fp=len(boxes)-tp;fn=len(truth)-tp
            row.update(status='scored',tp=tp,fp=fp,fn=fn,
                       matches=[dict(door_id=frame['doors'][a]['id'],detection_index=boxes[b][0],iou=overlaps[a][b])for a,b in matches])
            matched_ious.extend(overlaps[a][b]for a,b in matches)
            counts.update(valid_inference_frames=1,valid_inference_gt_objects=len(truth),tp=tp,fp=fp,fn_valid_inference=fn,
                          frame_exact_successes=int(fp==0 and fn==0))
            rows.append(row)
        def ratio(a,b):return a/b if b else None
        c={key:counts[key]for key in ['draft_frames','excluded_visibility_frames','eligible_frames','eligible_gt_objects',
             'valid_inference_frames','valid_inference_gt_objects','missing_predictions','runtime_errors','tp','fp','fn_valid_inference','frame_exact_successes']}
        precision=ratio(c['tp'],c['tp']+c['fp']);recall=ratio(c['tp'],c['tp']+c['fn_valid_inference'])
        c.update(frame_exact_success_rate=ratio(c['frame_exact_successes'],c['eligible_frames']),
                 valid_inference_precision=precision,valid_inference_recall=recall,
                 valid_inference_f1=ratio(2*c['tp'],2*c['tp']+c['fp']+c['fn_valid_inference']),
                 matched_mean_iou=sum(matched_ious)/len(matched_ious)if matched_ious else None)
        result['targets'][target]=dict(summary=c,frames=rows)
    summaries=[t['summary']for t in result['targets'].values()]
    result['status']=('no_selected_frames'if not selected else
                      ('no_evaluable_targets'if result['reviewed_selected_frames']==len(selected)else'awaiting_review')if not any(c['eligible_frames']for c in summaries)else
                      'incomplete_predictions'if any(c['missing_predictions']for c in summaries)else
                      'evaluated_with_runtime_errors'if any(c['runtime_errors']for c in summaries)else'evaluated')
    return result


def verify_rgb(dataset_path, frame):
    path=Path(frame['rgb']['path'])
    if not path.is_absolute():path=Path(dataset_path).resolve().parent/path
    info=rgb(path,frame['rgb']['sha256'])
    if (info['width'],info['height'])!=(frame['rgb']['width'],frame['rgb']['height']):
        raise ValueError('원본 RGB 크기가 manifest와 다릅니다')


def prepare_view(dataset_path, summary_path):
    dataset,checksum=read_manifest(dataset_path)
    source=read_json(summary_path);raw_path=Path(source['results_path'])
    if digest(raw_path)!=source['results_sha256']:raise ValueError('기존 검출 원문 해시 불일치')
    raw=read_json(raw_path)
    if (raw['status']!='complete' or raw['not_completed_frames']!=0 or raw['expected_frames']!=6
            or len(raw['frames'])!=6 or any(f['status']!='complete'for f in raw['frames'])):raise ValueError('완료된 기존 검출 원문이 필요합니다')
    by_hash={f['rgb']['sha256']:f for f in dataset['frames']};frames=[]
    for item in raw['frames']:
        frame=by_hash.get(item['sha256'])
        if frame is None:raise ValueError('검출 영상이 현재 자료에 없습니다')
        verify_rgb(dataset_path,frame)
        frames.append(dict(id=frame['id'],rgb_sha256=item['sha256'],status='complete',error=None,detections=item['doors']))
    result=dict(schema_version=1,provenance=dict(source_path=str(raw_path),source_sha256=digest(raw_path),
                 summary_sha256=digest(summary_path),dataset_sha256_at_preparation=checksum,recon=raw['recon']),frames=frames)
    validate_predictions(result,dataset)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    prepare=sub.add_parser('prepare-view',help='이미 완료된 여섯 장 원문을 재사용, 추론 없음')
    prepare.add_argument('--dataset',type=Path,required=True)
    prepare.add_argument('--summary',type=Path,default=Path('docs/data/door-view-diagnostic-summary.json'))
    prepare.add_argument('--output',type=Path,required=True)
    score=sub.add_parser('score',help='한 환경·한 분할의 검수 bbox만 평가')
    score.add_argument('--dataset',type=Path,required=True);score.add_argument('--predictions',type=Path,required=True)
    score.add_argument('--environment',choices=['gazebo','real_rgb','real_rgbd'],required=True)
    score.add_argument('--split',choices=['development','train','validation','test','unassigned'],required=True)
    score.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('기존 결과를 덮어쓰지 않습니다')
    if args.command=='prepare-view':result=prepare_view(args.dataset,args.summary)
    else:
        dataset,checksum=read_manifest(args.dataset)
        for frame in dataset['frames']:
            if frame['environment']==args.environment and frame['split']==args.split:verify_rgb(args.dataset,frame)
        result=evaluate_dataset(dataset,read_json(args.predictions),args.environment,args.split)
        result.update(dataset_sha256=checksum,predictions_sha256=digest(args.predictions),evaluator_sha256=digest(__file__))
    encoded=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x')as handle:handle.write(encoded)
    print(json.dumps(dict(output=str(args.output),status=result.get('status','prepared'),
        selected_frames=result.get('selected_frames'),reviewed_frames=result.get('reviewed_selected_frames'),
        prediction_frames=result.get('selected_prediction_frames',len(result.get('frames',[])))),ensure_ascii=False))


if __name__=='__main__':main()
