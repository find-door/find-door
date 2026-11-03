"""검수·환경·가시성·오류 분모와 다중 문 일대일 대응을 확인한다."""
from copy import deepcopy
import json
import pytest
from label_evaluate import evaluate_dataset,maximum_matching,prepare_view,verify_rgb
from dataset_intake import digest


def frame(identifier='f1',environment='gazebo',reviewed=True,doors=None):
    box=dict(id='door1',state='closed',leaf_bbox=[10,10,30,60],leaf_visibility='visible',
             doorway_bbox=[5,5,50,70],doorway_visibility='visible')
    return dict(id=identifier,environment=environment,group_id='g'if environment=='gazebo'else'real',
                split='development',review_status='reviewed'if reviewed else'draft',doors=[box]if doors is None else doors,
                rgb=dict(path='rgb.png',sha256=('a'if identifier=='f1'else'b')*64,width=100,height=80),
                capabilities=dict(depth=False,camera_info=False,tf=False))


def prediction(f,boxes=None,status='complete'):
    return dict(id=f['id'],rgb_sha256=f['rgb']['sha256'],status=status,
                error='backend failed'if status=='runtime_error'else None,
                detections=None if status=='runtime_error'else[
                    dict(**{'class':'door'},confidence=.4,bbox=b)for b in (boxes or [])])


def evaluate(frames,rows,**kwargs):
    dataset=dict(schema_version=1,dataset_id='test',frames=frames)
    return evaluate_dataset(dataset,dict(schema_version=1,provenance={},frames=rows),
                            kwargs.pop('environment','gazebo'),kwargs.pop('split','development'),**kwargs)


def test_leaf_and_doorway_are_separate_references():
    f=frame();r=evaluate([f],[prediction(f,[[10,10,30,60]])])
    assert r['targets']['leaf']['summary']['tp']==1
    assert r['targets']['doorway']['summary']['tp']==0
    assert r['targets']['doorway']['summary']['fn_valid_inference']==1


def test_maximum_matching_avoids_greedy_trap():
    assert maximum_matching([[.9,.8],[.85,0]],.5)==[(0,1),(1,0)]
    assert maximum_matching([], .5)==[]
    assert maximum_matching([[]],.5)==[]


def test_duplicate_detections_are_false_positives():
    f=frame();r=evaluate([f],[prediction(f,[[10,10,30,60]]*2)])['targets']['leaf']['summary']
    assert (r['tp'],r['fp'],r['fn_valid_inference'])==(1,1,0)
    assert r['frame_exact_success_rate']==0 and r['valid_inference_precision']==.5


@pytest.mark.parametrize('boxes,success', [([],1),([[10,10,30,60]],0)])
def test_reviewed_negative_frame(boxes,success):
    f=frame(doors=[]);s=evaluate([f],[prediction(f,boxes)])['targets']['leaf']['summary']
    assert s['eligible_frames']==1 and s['frame_exact_successes']==success
    assert s['valid_inference_recall']is None


def test_draft_labels_produce_no_performance():
    f=frame(reviewed=False);r=evaluate([f],[prediction(f,[[10,10,30,60]])])
    assert r['status']=='awaiting_review'
    for target in r['targets'].values():
        assert target['summary']['draft_frames']==1
        assert target['summary']['frame_exact_success_rate']is None
        assert target['summary']['valid_inference_precision']is None


def test_unknown_visibility_excludes_whole_frame_for_that_target():
    f=frame();f['doors'][0].update(doorway_visibility='unknown',doorway_bbox=None)
    r=evaluate([f],[prediction(f,[[10,10,30,60]])])
    assert r['targets']['leaf']['summary']['eligible_frames']==1
    assert r['targets']['doorway']['summary']['excluded_visibility_frames']==1


@pytest.mark.parametrize('status',['runtime_error','missing'])
def test_failed_or_missing_inference_remains_in_success_denominator(status):
    a,b=frame(),frame('f2');rows=[prediction(a,[[10,10,30,60]])]
    if status=='runtime_error':rows.append(prediction(b,status=status))
    r=evaluate([a,b],rows);s=r['targets']['leaf']['summary']
    assert s['eligible_frames']==2 and s['frame_exact_success_rate']==.5
    assert s['eligible_gt_objects']==2 and s['valid_inference_gt_objects']==1
    assert s['valid_inference_precision']==1 and s['valid_inference_recall']==1
    assert r['status']==('evaluated_with_runtime_errors'if status=='runtime_error'else'incomplete_predictions')
    assert s['runtime_errors'if status=='runtime_error'else'missing_predictions']==1


def test_real_and_gazebo_are_never_aggregated():
    a,b=frame(),frame('f2','real_rgb');r=evaluate([a,b],[prediction(a),prediction(b)])
    assert r['selected_frames']==1 and r['dataset_total_frames']==2
    real=evaluate([a,b],[prediction(a),prediction(b)],environment='real_rgb')
    assert real['targets']['leaf']['frames'][0]['id']=='f2'


@pytest.mark.parametrize('kind',['hash','duplicate','unknown_id','score','box','class','missing_key','failed_boxes'])
def test_invalid_prediction_rejected(kind):
    f=frame();p=prediction(f,[[10,10,30,60]]);rows=[p]
    if kind=='hash':p['rgb_sha256']='0'*64
    elif kind=='duplicate':rows.append(deepcopy(p))
    elif kind=='unknown_id':p['id']='unrelated'
    elif kind=='score':p['detections'][0]['confidence']=True
    elif kind=='box':p['detections'][0]['bbox'][2]=101
    elif kind=='class':p['detections'][0]['class']='wall'
    elif kind=='missing_key':del p['error']
    else:p.update(status='runtime_error',error='failed')
    with pytest.raises(ValueError):evaluate([f],rows)


def test_source_documents_not_mutated():
    f=frame();p=prediction(f,[[10,10,30,60]]);before=deepcopy((f,p));evaluate([f],[p]);assert (f,p)==before


def test_actual_image_shape_checked_and_relative_paths_resolved(tmp_path):
    from PIL import Image
    image=tmp_path/'rgb.png';Image.new('RGB',(100,80)).save(image)
    f=frame();f['rgb']['sha256']=digest(image)
    verify_rgb(tmp_path/'dataset.json',f)
    f['rgb']['width']=200
    with pytest.raises(ValueError,match='크기'):verify_rgb(tmp_path/'dataset.json',f)


@pytest.mark.parametrize('threshold',[0,1.1,True,float('nan')])
def test_invalid_iou_threshold(threshold):
    with pytest.raises(ValueError):evaluate([frame()],[],threshold=threshold)



def test_no_selected_environment_is_not_pending_review():
    r=evaluate([frame()],[],environment='real_rgb')
    assert r['status']=='no_selected_frames' and r['selected_frames']==0


def test_reviewed_but_unobservable_targets_have_no_metrics():
    f=frame()
    f['doors'][0].update(leaf_visibility='occluded',leaf_bbox=None,doorway_visibility='unknown',doorway_bbox=None)
    r=evaluate([f],[prediction(f)])
    assert r['status']=='no_evaluable_targets'


def test_report_keeps_provenance_independent_from_input():
    f=frame();preds=dict(schema_version=1,provenance={'model':{'name':'test'}},frames=[prediction(f)])
    from label_evaluate import evaluate_dataset
    r=evaluate_dataset(dict(schema_version=1,dataset_id='test',frames=[f]),preds,'gazebo','development')
    r['prediction_provenance']['model']['name']='changed'
    assert preds['provenance']['model']['name']=='test'


@pytest.fixture
def prepared_fixture(tmp_path):
    from PIL import Image
    frames=[];raw=[]
    for n in range(6):
        image=tmp_path/f'{n}.png';Image.new('RGB',(100,80),(n,20,30)).save(image)
        f=frame(f'frame{n}',reviewed=n==0);f['rgb'].update(path=image.name,sha256=digest(image))
        frames.append(f);raw.append(dict(status='complete',sha256=digest(image),doors=[dict(**{'class':'door'},confidence=.4,bbox=[10,10,30,60])]))
    dataset=tmp_path/'dataset.json';dataset.write_text(json.dumps(dict(schema_version=1,dataset_id='test',frames=frames)))
    source=tmp_path/'raw.json';source.write_text(json.dumps(dict(status='complete',not_completed_frames=0,expected_frames=6,frames=raw,recon={'model':'fake'})))
    summary=tmp_path/'summary.json';summary.write_text(json.dumps(dict(results_path=str(source),results_sha256=digest(source))))
    return dataset,source,summary


def test_prepare_view_reuses_only_hashed_source_and_original_images(prepared_fixture):
    dataset,source,summary=prepared_fixture;p=prepare_view(dataset,summary)
    assert len(p['frames'])==6 and p['provenance']['source_sha256']==digest(source)
    source.write_text('{}')
    with pytest.raises(ValueError,match='해시'):prepare_view(dataset,summary)


def test_cli_reuses_predictions_after_label_review_and_refuses_overwrite(prepared_fixture,tmp_path,monkeypatch):
    import sys
    from label_evaluate import main
    dataset,source,summary=prepared_fixture;predictions=tmp_path/'predictions.json';report=tmp_path/'report.json'
    monkeypatch.setattr(sys,'argv',['label_evaluate','prepare-view','--dataset',str(dataset),'--summary',str(summary),'--output',str(predictions)])
    main();prepared_sha=digest(predictions)
    document=json.loads(dataset.read_text());document['frames'][1]['review_status']='reviewed';dataset.write_text(json.dumps(document))
    monkeypatch.setattr(sys,'argv',['label_evaluate','score','--dataset',str(dataset),'--predictions',str(predictions),'--environment','gazebo','--split','development','--output',str(report)])
    main();result=json.loads(report.read_text())
    assert result['targets']['leaf']['summary']['eligible_frames']==2
    assert result['targets']['leaf']['summary']['tp']==2
    assert result['prediction_provenance']['dataset_sha256_at_preparation']!=result['dataset_sha256']
    assert digest(predictions)==prepared_sha
    before=report.read_bytes()
    with pytest.raises(SystemExit):main()
    assert report.read_bytes()==before



@pytest.mark.parametrize('negative,boxes,expected',[(False,[],0.),(True,[[10,10,30,60]],0.),(True,[],None)])
def test_f1_uses_count_denominator_when_precision_or_recall_undefined(negative,boxes,expected):
    f=frame(doors=[]if negative else None)
    result=evaluate([f],[prediction(f,boxes)])['targets']['leaf']['summary']
    assert result['valid_inference_f1']==expected
