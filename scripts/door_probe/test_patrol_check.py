"""순찰 몸체 제어·이동 방향 장애물·사후 문 대응의 실패 조건을 검사한다."""
from copy import deepcopy
import math
from types import SimpleNamespace as N
import pytest
from patrol_check import command_for,directional_clearance,evaluate_observations,stages,validate_saved_snapshot


def test_northbound_route_is_body_forward_and_stops_within_tolerance():
    stage=dict(kind='move',x=0.,y=3.,yaw=math.pi/2)
    command,done=command_for(stage,[0.,-3.,math.pi/2])
    assert not done and command[0]==pytest.approx(.35) and abs(command[1])<1e-10
    assert command_for(stage,[0.,2.97,math.pi/2])==((0.,0.,0.),True)


def test_large_heading_error_rotates_without_translating_and_wraps_pi():
    stage=dict(kind='move',x=0.,y=3.,yaw=math.pi/2)
    command,done=command_for(stage,[0.,-3.,0.])
    assert command[:2]==(0.,0.) and command[2]==.5 and not done
    command,done=command_for(dict(kind='turn',yaw=math.pi),[0.,0.,-math.pi+.01])
    assert done and abs(command[2])<.02


def scan():
    return N(angle_min=-math.pi,angle_increment=math.pi/180,range_min=.1,range_max=10.,ranges=[2.]*361)


def test_clearance_uses_requested_direction_and_rejects_missing_measurements():
    message=scan();message.ranges[180]=.4
    assert directional_clearance(message,(.1,0.,0.))==.4
    assert directional_clearance(message,(-.1,0.,0.))==2.
    assert directional_clearance(message,(0.,0.,.3))is None
    message.ranges=[float('nan')]*361
    with pytest.raises(RuntimeError):directional_clearance(message,(.1,0.,0.))


def truth_and_history():
    truth=[dict(id=str(i),center_world_xyz=[float(i*2),0.,0.],state='open'if i%2 else 'closed')for i in range(4)]
    history=[]
    for frame in range(3):
        stamp=1_000_000_000+frame*100_000_000
        objects=[dict(id=f'door_{i:06d}',position=item['center_world_xyz'],state=item['state'],last_stamp_ns=stamp)for i,item in enumerate(truth)]
        history.append(dict(door_map=dict(stamp_ns=stamp+50_000_000,objects=objects)))
    return truth,history


def test_fixed_alignment_needs_three_distinct_fresh_observations_per_door():
    truth,history=truth_and_history()
    assert evaluate_observations(history,truth,[0.,0.,0.])['passed']
    assert not evaluate_observations([history[0]]*10,truth,[0.,0.,0.])['passed']
    for row in history:row['door_map']['stamp_ns']+=3_000_000_000
    assert not evaluate_observations(history,truth,[0.,0.,0.])['passed']


@pytest.mark.parametrize('problem',['missing','wrong_state','duplicate_id','extra','same_id'])
def test_missing_wrong_duplicated_or_unmatched_door_is_a_failure(problem):
    truth,history=truth_and_history()
    for row in history:
        objects=row['door_map']['objects']
        if problem=='missing':objects.pop()
        elif problem=='wrong_state':objects[0]['state']='open'
        elif problem=='duplicate_id':
            duplicate=deepcopy(objects[0]);duplicate['id']='second_id';objects.append(duplicate)
        elif problem=='extra':
            extra=deepcopy(objects[0]);extra.update(id='extra',position=[50.,50.,0.]);objects.append(extra)
        else:
            for obj in objects:obj['id']='same'
    assert not evaluate_observations(history,truth,[0.,0.,0.])['passed']


def test_plan_visits_four_views_and_does_not_use_expected_door_states():
    plan=stages()
    scans=[s for s in plan if s['kind']=='scan' and s['name']!='save']
    assert len(scans)==4 and all(s['duration_sim_sec']==7 for s in scans)
    assert all('state'not in s and 'door_id'not in s for s in plan)


def test_correct_later_observations_cannot_hide_an_earlier_wrong_known_state():
    truth,history=truth_and_history()
    wrong=deepcopy(history[0]);wrong['door_map']['objects'][0]['state']='open'
    wrong['door_map']['objects'][0]['last_stamp_ns']-=100_000_000
    assert not evaluate_observations([wrong]+history,truth,[0.,0.,0.])['passed']


@pytest.mark.parametrize('problem',['late_extra','duplicate','missing','anchor','state','map_id'])
def test_saved_snapshot_must_match_latest_map_even_after_save_wait(problem):
    truth,history=truth_and_history();objects=history[-1]['door_map']['objects']
    saved=dict(door_session_schema_version=1,memory=dict(map_id='run',objects=deepcopy(objects)))
    assert validate_saved_snapshot(saved,'run',objects)['matched']
    if problem=='late_extra':
        extra=deepcopy(objects[0]);extra['id']='late';objects.append(extra)
    elif problem=='duplicate':saved['memory']['objects'].append(deepcopy(objects[0]))
    elif problem=='missing':saved['memory']['objects'].pop()
    elif problem=='anchor':saved['memory']['objects'][0]['position']=[.1,0.,0.]
    elif problem=='state':saved['memory']['objects'][0]['state']='unknown'
    else:saved['memory']['map_id']='other'
    with pytest.raises(RuntimeError):validate_saved_snapshot(saved,'run',objects)
