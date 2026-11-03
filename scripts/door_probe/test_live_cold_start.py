"""빈 지도 최초 등록과 음성 대조군이 재관측·추가 문을 숨기지 않는지 확인한다."""
from copy import deepcopy
import pytest

from live_check import cold_start_evidence, stationary_ready, validate_map_identity, validate_save_result, transition_control, final_observation_evidence


def history():
    return [dict(stamp_ns=1_000_000_000,objects=[],counters={}),
            dict(stamp_ns=1_500_000_000,objects=[dict(id='door-1',position=[1.,0.,0.],state='open',
                observations=3,last_stamp_ns=1_400_000_000)],counters=dict(processed=3))]


def test_first_registration_preserves_empty_start_and_fresh_votes():
    result=cold_start_evidence(history(),'open')
    assert result['initial_objects']==0 and result['final_objects']==1 and result['observations']==3
    assert result['source_age_sec']==pytest.approx(.1) and result['reobserved']==0


@pytest.mark.parametrize('kind',['nonempty_start','reobserved','duplicate','few_votes','wrong_state','old','future'])
def test_warm_or_invalid_observations_are_not_cold_success(kind):
    data=history();item=data[-1]['objects'][0]
    if kind=='nonempty_start':data[0]['objects']=[deepcopy(item)]
    elif kind=='reobserved':data[0]['counters']['reobserved']=1
    elif kind=='duplicate':data[-1]['objects'].append(deepcopy(item))
    elif kind=='few_votes':item['observations']=2
    elif kind=='wrong_state':item['state']='closed'
    elif kind=='old':data[-1]['stamp_ns']=4_000_000_000
    elif kind=='future':item['last_stamp_ns']=2_000_000_000
    with pytest.raises(RuntimeError):cold_start_evidence(data,'open')


def negative():
    return dict(maps=[1],scan_count=6,detector=[dict(counters=dict(completed=10))],
                doors=[dict(stamp_ns=1_000_000_000,objects=[],counters={}),
                       dict(stamp_ns=4_000_000_000,objects=[],counters={})])


def test_negative_requires_both_time_and_processed_bundles():
    data=negative();assert stationary_ready(data,'negative')
    data['detector'][0]['counters']['completed']=9;assert not stationary_ready(data,'negative')
    data['detector'][0]['counters']['completed']=10;data['doors'][-1]['stamp_ns']-=1
    assert not stationary_ready(data,'negative')


def test_disappeared_false_door_is_still_a_negative_failure():
    data=negative();data['doors'][0]['objects']=[dict(state='unknown')]
    with pytest.raises(RuntimeError,match='대조군'):
        stationary_ready(data,'negative')
    with pytest.raises(RuntimeError):cold_start_evidence(data['doors'],'negative')


def test_empty_negative_cold_start_and_positive_gate():
    data=negative();assert cold_start_evidence(data['doors'],'negative')['final_objects']==0
    assert not stationary_ready(data,'open')
    data['doors'][-1]['objects']=[dict(state='open')]
    assert stationary_ready(data,'open')


def test_foreign_map_identity_cannot_contaminate_the_experiment():
    validate_map_identity([], 'owned')
    validate_map_identity([dict(map_id='owned')], 'owned')
    with pytest.raises(RuntimeError,match='map_id'):
        validate_map_identity([dict(map_id='foreign'),dict(map_id='owned')], 'owned')


def test_negative_requires_save_rejection_and_no_fake_output():
    validate_save_result('negative',False,False)
    validate_save_result('open',True,True)
    with pytest.raises(RuntimeError):validate_save_result('negative',True,False)
    with pytest.raises(RuntimeError):validate_save_result('negative',False,True)
    with pytest.raises(RuntimeError):validate_save_result('open',False,False)


def test_object_arriving_during_save_wait_invalidates_negative_completion():
    data=negative()
    assert final_observation_evidence(data,'negative',True)['negative_observation']['final_objects']==0
    data['doors'].append(dict(stamp_ns=4_100_000_000,objects=[dict(state='unknown')],counters={}))
    with pytest.raises(RuntimeError):final_observation_evidence(data,'negative',True)
    with pytest.raises(RuntimeError):final_observation_evidence(data,'negative',False)


def test_state_expiring_during_save_wait_invalidates_cold_completion():
    data=dict(doors=history())
    assert final_observation_evidence(data,'open',True)['cold_start']['observations']==3
    data['doors'][-1]['objects'][0]['state']='unknown'
    with pytest.raises(RuntimeError):final_observation_evidence(data,'open',True)


def test_transition_uses_actual_world_and_hinge_instead_of_original_dimensions(tmp_path):
    path=tmp_path/'world.sdf';path.write_text('<sdf><world name="custom"><model name="door_leaf"><pose>1 -0.6 0.2 0 0 0</pose></model></world></sdf>')
    assert transition_control(path)==('custom',[1.,-.6,.2])


@pytest.mark.parametrize('pose',['0 0 0 0 0 -1.57','0 0 0 0 0 nan','0 0 0','0 0 0 1 0 0'])
def test_invalid_transition_layout_is_not_silently_moved_as_a_closed_door(tmp_path,pose):
    path=tmp_path/'world.sdf';path.write_text(f'<sdf><world name="custom"><model name="door_leaf"><pose>{pose}</pose></model></world></sdf>')
    with pytest.raises(ValueError):transition_control(path)
