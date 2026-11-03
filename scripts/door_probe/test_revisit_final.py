"""스캔 이후 떠나는 구간과 저장 대기의 잘못된 관측도 최종 검사한다."""
from copy import deepcopy
import pytest
from revisit_final import validate_complete_history


def fixture():
    objects=[dict(id=f'door_{i:06d}',position=[i*2.,0.,0.],state='unknown',observation_status='stale',
                  last_stamp_ns=1_000_000_000,observations=10,last_confirmed_state='closed',
                  last_confirmed_stamp_ns=1_000_000_000)for i in range(4)]
    history=[];events=[];current=deepcopy(objects)
    def add(now):history.append(dict(door_map=dict(stamp_ns=now,epoch=0,objects=deepcopy(current))))
    for phase,(expected,changed)in enumerate([('open',4_000_000_000),('closed',14_000_000_000)]):
        before=deepcopy(current[0]);start=len(history)
        for offset in [0,1_000_000_000,2_000_000_000]:add(changed+offset)
        away_end=len(history)
        for vote in range(3):
            stamp=changed+3_000_000_000+vote*100_000_000
            current[0].update(state=expected,observation_status='confirmed',last_stamp_ns=stamp,
                              last_confirmed_state=expected,last_confirmed_stamp_ns=stamp,
                              observations=before['observations']+3+vote)
            add(stamp+10_000_000)
        scan_end=len(history)
        current[0].update(state='unknown',observation_status='stale')
        add(changed+6_000_000_000)
        events.append(dict(expected_state=expected,before=before,changed_sim_ns=changed,
            history_start=start,away_history_end=away_end,history_end=scan_end,
            camera_out_of_view=dict(optical_point=[0.,0.,-6.]),returncode=0,stdout='data: true\n'))
    return objects,events,history


def test_whole_history_includes_travel_and_save_wait_after_scan():
    objects,events,history=fixture();result=validate_complete_history(objects,events,history)
    assert result['passed']
    assert result['events'][0]['history_end']==events[1]['history_start']
    assert result['events'][1]['history_end']==len(history)


@pytest.mark.parametrize('phase',[0,1])
def test_wrong_known_after_scan_cannot_hide_outside_the_evaluation_window(phase):
    objects,events,history=fixture();index=events[phase]['history_end']
    obj=history[index]['door_map']['objects'][0];stamp=history[index]['door_map']['stamp_ns']
    opposite='closed'if phase==0 else'open'
    obj.update(state=opposite,observation_status='confirmed',last_stamp_ns=stamp,
               last_confirmed_state=opposite,last_confirmed_stamp_ns=stamp,observations=obj['observations']+1)
    with pytest.raises(RuntimeError):validate_complete_history(objects,events,history)


@pytest.mark.parametrize('problem',['duplicate_event','bad_boundary','forward_camera','failed_control','fake_away'])
def test_missing_control_or_away_proof_rejects_success(problem):
    objects,events,history=fixture()
    if problem=='duplicate_event':events[1]['expected_state']='open'
    elif problem=='bad_boundary':events[0]['history_start']=-1
    elif problem=='forward_camera':events[0]['camera_out_of_view']['optical_point'][2]=1.
    elif problem=='failed_control':events[0]['stdout']='data: false'
    else:events[0]['away_history_end']=events[0]['history_start']+1
    with pytest.raises(RuntimeError):validate_complete_history(objects,events,history)
