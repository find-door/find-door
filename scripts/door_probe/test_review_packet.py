"""실패 영상 추가 시 출처·개발 그룹·초안 라벨·원본 보존을 확인한다."""
import copy
import json
from pathlib import Path
import pytest
from PIL import Image
from dataset_intake import digest,door
from review_packet import build_packet,FAMILY_GROUP,OLD_GROUP
from view_capture import frame_bounds,AUX_POSITION,AUX_OPTICAL_QUATERNION


def write(path,doc):path.write_text(json.dumps(doc));return path


@pytest.fixture
def packet(tmp_path):
    number=0
    def image():
        nonlocal number
        number+=1;path=tmp_path/f'{number}.png';Image.new('RGB',(640,480),(number,20,30)).save(path)
        return dict(path=str(path),sha256=digest(path),width=640,height=480)
    def frame(identifier,environment,group,split):
        return dict(id=identifier,environment=environment,group_id=group,split=split,rgb=image(),
                    capabilities=dict(depth=False,camera_info=False,tf=False),review_status='draft',doors=[door('door_1','closed')])
    base=write(tmp_path/'base.json',dict(schema_version=1,dataset_id='base',frames=[
        frame('gz','gazebo',OLD_GROUP,'development'),frame('real','real_rgb','legacy','unassigned')]))
    runs=[]
    for state in ['closed','half','open']:
        expected='closed'if state=='closed'else'open'
        status='complete'if state=='half'else'failed'
        path=write(tmp_path/f'{state}-trial.json',dict(status=status,state=expected))
        runs.append(dict(condition=state,expected_state=expected,status=status,path=str(path),sha256=digest(path),rgb=image()))
    scene=write(tmp_path/'scene.json',dict(runs=runs))
    rows=[];captures=[]
    k=[500.,0,320.,0,500.,240.,0,0,1.]
    camera=dict(translation=[-3,0,.12],quaternion=AUX_OPTICAL_QUATERNION)
    for state in ['closed','half','open']:
        recorded=[]
        for view in ['low','high']:
            info=image()
            bound=frame_bounds(camera['translation']if view=='low'else AUX_POSITION,AUX_OPTICAL_QUATERNION,k)
            rgb=dict(view=view,rgb=info['path'],sha256=info['sha256'],width=640,height=480,k=list(k),d=[0.]*5,
                     frame_id='camera_color_optical_frame'if view=='low'else'door_aux_optical_frame',stamp_ns=4_000_000_000,
                     frame_bounds_unclipped=bound)
            recorded.append(rgb)
            rows.append(dict(rgb,state=state,doors=[dict(bbox=[10,20,30,40],confidence=.4)]))
        path=write(tmp_path/f'{state}-capture.json',dict(state=state,status='complete',frames=recorded,robot_camera=camera,
                                                       aux_position=AUX_POSITION,pair_stamp_delta_ns=0))
        captures.append(dict(path=str(path),sha256=digest(path)))
    raw=write(tmp_path/'results.json',dict(status='complete',frames=rows))
    view=write(tmp_path/'view.json',dict(results_path=str(raw),results_sha256=digest(raw),captures=captures))
    return base,scene,view,tmp_path


def test_new_frames_are_draft_and_sources_unchanged(packet):
    base,scene,view,root=packet;before=base.read_bytes()
    result,summary=build_packet(base,scene,view)
    assert base.read_bytes()==before
    assert summary['total_frames']==11 and summary['added_frames']==9
    assert summary['reviewed_dual_target_frames']==0
    assert all(f['doors'][0]['leaf_bbox']is None and f['doors'][0]['doorway_bbox']is None for f in result['frames'])
    assert all(f['group_id']==FAMILY_GROUP and f['split']=='development' for f in result['frames']if f['environment']=='gazebo')
    assert result['frames'][-1]['split']=='unassigned'
    assert not any(f['capabilities']['depth']for f in result['frames'][:9])
    high=next(f for f in result['frames']if f['id']=='metal_closed_3m_high')
    assert high['capabilities']==dict(depth=False,camera_info=True,tf=False)
    assert high['review_context']['predictions'][0]['bbox']==[10,20,30,40]


def test_rebuilding_deduplicates_without_losing_review(packet):
    base,scene,view,root=packet;first,_=build_packet(base,scene,view)
    first['frames'][0]['review_status']='reviewed'
    first['frames'][0]['doors'][0].update(leaf_bbox=[10,20,30,40],leaf_visibility='visible')
    path=write(root/'previous.json',first)
    second,summary=build_packet(path,scene,view)
    assert summary['added_frames']==0 and summary['omitted_duplicate_rgb']==9
    assert summary['reviewed_dual_target_frames']==1
    assert second['frames'][0]['doors'][0]['leaf_bbox']==[10,20,30,40]


def test_original_group_cannot_be_in_test_split(packet):
    base,scene,view,root=packet;d=json.loads(base.read_text());d['frames'][0]['split']='test';write(base,d)
    with pytest.raises(ValueError,match='분할'):build_packet(base,scene,view)


def test_duplicate_rgb_cannot_hide_split_leakage(packet):
    base,scene,view,root=packet;result,_=build_packet(base,scene,view)
    candidate=copy.deepcopy(result['frames'][0]);candidate.update(group_id='claimed_independent',split='test')
    d=json.loads(base.read_text());d['frames'].append(candidate);write(base,d)
    with pytest.raises(ValueError,match='분할'):build_packet(base,scene,view)


def test_parent_hash_tamper_rejected(packet):
    base,scene,view,root=packet
    source=json.loads(scene.read_text())['runs'][0]['path'];Path(source).write_text('{}')
    with pytest.raises(ValueError,match='해시'):build_packet(base,scene,view)


def test_changed_image_rejected(packet):
    base,scene,view,root=packet
    source=json.loads(scene.read_text())['runs'][0]['rgb']['path'];Path(source).write_bytes(b'changed')
    with pytest.raises(ValueError,match='해시'):build_packet(base,scene,view)


def test_predictions_do_not_replace_labels(packet):
    base,scene,view,root=packet;result,_=build_packet(base,scene,view)
    for f in result['frames'][:9]:
        assert f['review_status']=='draft'
        assert all(d['leaf_visibility']=='unknown' and d['doorway_visibility']=='unknown'for d in f['doors'])
