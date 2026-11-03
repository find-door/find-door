"""각 제어 경계로 전체 이력을 나눠 이동·주차·저장 대기까지 재검사한다."""
import math
from revisit import assert_identity,validate_away,validate_revisit


def validate_complete_history(initial_objects,events,history):
    if not isinstance(events,list) or len(events)!=2:
        raise RuntimeError('열림·닫힘 두 재방문 제어가 필요합니다')
    if [event.get('expected_state')for event in events]!=['open','closed']:
        raise RuntimeError('재방문 제어 순서가 다릅니다')
    if not isinstance(history,list) or not history:
        raise RuntimeError('전체 관측 이력이 없습니다')
    starts=[event.get('history_start')for event in events]
    if any(type(index)is not int for index in starts) or not 0<=starts[0]<starts[1]<len(history):
        raise RuntimeError('재방문 이력 경계가 잘못됐습니다')
    if events[0]['before']['id']!=events[1]['before']['id']:
        raise RuntimeError('두 번의 재방문 대상 ID가 다릅니다')
    if events[0]['changed_sim_ns']>=events[1]['changed_sim_ns']:
        raise RuntimeError('재방문 제어 시각이 역행했습니다')
    all_maps=[row['door_map']for row in history[starts[0]:]]
    identity=assert_identity(initial_objects,all_maps)
    results=[]
    for index,event in enumerate(events):
        point=event.get('camera_out_of_view',{}).get('optical_point')
        if (not isinstance(point,list)or len(point)!=3 or
                any(type(value)not in(int,float)or not math.isfinite(value)for value in point) or point[2]>=0):
            raise RuntimeError('카메라 뒤의 문을 제어한 증거가 없습니다')
        if event.get('returncode')!=0 or event.get('stdout','').strip()!='data: true':
            raise RuntimeError('Gazebo 상태 변경 응답이 정상적이지 않습니다')
        start=starts[index];end=starts[index+1]if index+1<len(events)else len(history)
        away_end=event.get('away_history_end')
        if type(away_end)is not int or not start<away_end<end:
            raise RuntimeError('미관측 대기 이력 경계가 잘못됐습니다')
        away=validate_away(event['before'],[row['door_map']for row in history[start:away_end]],initial_objects)
        evaluation=validate_revisit(event['before'],[row['door_map']for row in history[start:end]],
                                    event['expected_state'],event['changed_sim_ns'],initial_objects)
        results.append(dict(expected_state=event['expected_state'],history_start=start,history_end=end,
                            away_check=away,evaluation=evaluation))
    return dict(passed=True,identity=identity,events=results,
                coverage='첫 변경부터 다음 변경 직전까지·둘째 변경부터 저장 응답 후 마지막 수신 관측까지')
