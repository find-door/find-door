"""문을 보지 않는 구간과 재방문 구간의 실제 지도 이력을 사후 검사한다."""

import math


def _fail(message):
    raise RuntimeError(message)


def _stamp(value, label):
    if type(value) is not int or value <= 0:
        _fail(f'{label}: 양의 정수 시각 필요')
    return value


def _position(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(type(x) not in (int, float) or not math.isfinite(x) for x in value)):
        _fail('문 anchor 좌표가 유효하지 않음')
    return list(value)


def assert_identity(initial_objects, maps):
    """초기 네 ID와 anchor가 모든 직접 지도 snapshot에서 그대로인지 확인한다."""
    if not isinstance(initial_objects, list) or len(initial_objects) != 4:
        _fail('초기 문은 정확히 네 개여야 함')
    initial = {}
    for obj in initial_objects:
        if not isinstance(obj, dict) or not isinstance(obj.get('id'), str) or not obj['id'] or obj['id'] in initial:
            _fail('초기 문 ID 누락·중복')
        initial[obj['id']] = _position(obj.get('position'))
    if not isinstance(maps, list) or not maps:
        _fail('지도 수신 이력이 없음')
    previous_time = 0
    for mapping in maps:
        if not isinstance(mapping, dict):
            _fail('지도 snapshot 형식 오류')
        stamp = _stamp(mapping.get('stamp_ns'), '지도')
        if stamp < previous_time:
            _fail('지도 시각 역행')
        previous_time = stamp
        if 'epoch' in mapping and (type(mapping['epoch']) is not int or mapping['epoch'] != 0):
            _fail('지도 epoch가 변경됨')
        objects = mapping.get('objects')
        if not isinstance(objects, list) or len(objects) != 4:
            _fail('문 ID 누락 또는 추가')
        seen = set()
        for obj in objects:
            if not isinstance(obj, dict) or not isinstance(obj.get('id'), str) or obj['id'] in seen:
                _fail('문 ID 중복 또는 형식 오류')
            identifier = obj['id']
            if identifier not in initial:
                _fail('초기 목록과 대응되지 않는 문 ID')
            seen.add(identifier)
            if _position(obj.get('position')) != initial[identifier]:
                _fail('고정 문 anchor가 변경됨')
    return dict(passed=True, ids=sorted(initial), snapshots=len(maps))


def _baseline(before, initial_objects):
    if not isinstance(before, dict) or before.get('id') not in {obj.get('id') for obj in initial_objects}:
        _fail('재방문 대상 ID가 초기 문과 대응되지 않음')
    anchor = next(obj['position'] for obj in initial_objects if obj['id'] == before['id'])
    if _position(before.get('position')) != _position(anchor):
        _fail('재방문 전 대상 anchor가 초기 위치와 다름')
    _record(before, _stamp(before.get('last_stamp_ns'), '재방문 전 관측'))
    if (before.get('last_confirmed_state') not in ('open', 'closed')
            or before.get('state') != 'unknown' or before.get('observation_status') != 'stale'):
        _fail('재방문 전 대상은 과거 확정 상태를 가진 stale·unknown이어야 함')
    return before['id']


def _record(obj, now):
    stamp = _stamp(obj.get('last_stamp_ns'), '관측')
    history_stamp = _stamp(obj.get('last_confirmed_stamp_ns'), '마지막 확정')
    if stamp > now or history_stamp > stamp:
        _fail('관측 또는 마지막 확정 시각이 미래임')
    if type(obj.get('observations')) is not int or obj['observations'] < 1:
        _fail('관측 횟수 오류')
    if obj.get('state') not in ('open', 'closed', 'unknown') or obj.get('last_confirmed_state') not in ('open', 'closed'):
        _fail('대상 문 상태 또는 확정 이력 오류')
    if obj.get('observation_status') not in ('confirmed', 'uncertain', 'stale'):
        _fail('현재 관측 여부 오류')
    if obj['state'] != 'unknown' and (obj['observation_status'] != 'confirmed'
            or obj['state'] != obj['last_confirmed_state'] or history_stamp != stamp):
        _fail('현재 known 상태와 확정 관측 이력 불일치')
    if obj['state'] != 'unknown' and now-stamp > 2_000_000_000:
        _fail('오래된 관측을 현재 known 상태로 표시함')
    if obj['state'] == 'unknown' and obj['observation_status'] == 'confirmed':
        _fail('unknown을 현재 확정 관측으로 표시함')
    return stamp


def _target(mapping, identifier):
    return next(obj for obj in mapping['objects'] if obj['id'] == identifier)


def validate_away(before, maps, initial_objects):
    """최소 2초 이력에서 미관측 대상의 관측·확정 이력이 갱신되지 않아야 한다."""
    assert_identity(initial_objects, maps)
    identifier = _baseline(before, initial_objects)
    span = maps[-1]['stamp_ns'] - maps[0]['stamp_ns']
    if span < 2_000_000_000:
        _fail('미관측 지도 수신 구간이 2초보다 짧음')
    keys = ('last_stamp_ns', 'observations', 'last_confirmed_state', 'last_confirmed_stamp_ns')
    for mapping in maps:
        obj = _target(mapping, identifier)
        _record(obj, mapping['stamp_ns'])
        if any(obj[key] != before[key] for key in keys):
            _fail('보지 않는 대상의 관측 또는 마지막 확정 이력이 갱신됨')
        if obj['state'] != 'unknown' or obj['observation_status'] != 'stale':
            _fail('보지 않는 과거 상태를 현재 상태처럼 표시함')
    return dict(passed=True, id=identifier, duration_sim_sec=span/1e9,
                snapshots=len(maps), last_confirmed_state=before['last_confirmed_state'],
                last_confirmed_stamp_ns=before['last_confirmed_stamp_ns'])


def validate_revisit(before, maps, expected_state, changed_sim_ns, initial_objects, minimum_observations=3):
    """실제 제어 뒤 새 관측으로 확정한 상태만 재방문 갱신으로 인정한다."""
    assert_identity(initial_objects, maps)
    identifier = _baseline(before, initial_objects)
    changed = _stamp(changed_sim_ns, '문 제어')
    if expected_state not in ('open', 'closed') or expected_state == before['last_confirmed_state']:
        _fail('이전 마지막 상태와 다른 알려진 상태로 변경하는 시험이어야 함')
    if type(minimum_observations) is not int or minimum_observations < 1:
        _fail('최소 확인 관측 수 오류')
    previous_stamp, previous_count = before['last_stamp_ns'], before['observations']
    old_history = (before['last_confirmed_state'], before['last_confirmed_stamp_ns'])
    previous_history = old_history
    updated = False
    confirmed = set()
    first = None
    for mapping in maps:
        obj = _target(mapping, identifier)
        now = mapping['stamp_ns']
        stamp = _record(obj, now)
        if stamp < previous_stamp or obj['observations'] < previous_count:
            _fail('대상 관측 시각 또는 횟수가 역행함')
        history = (obj['last_confirmed_state'], obj['last_confirmed_stamp_ns'])
        if stamp == previous_stamp and (obj['observations'] != previous_count or history != previous_history):
            _fail('동일 관측 시각에서 횟수 또는 확정 이력이 갱신됨')
        if stamp > previous_stamp and obj['observations'] <= previous_count:
            _fail('새 관측 시각에 관측 횟수가 증가하지 않음')
        fresh_expected = (obj['state'] == expected_state and obj['last_confirmed_state'] == expected_state
                          and obj['observation_status'] == 'confirmed'
                          and obj['last_confirmed_stamp_ns'] == stamp
                          and stamp > changed and 0 <= now-stamp <= 2_000_000_000)
        if now >= changed and obj['state'] not in ('unknown', expected_state):
            _fail('문 제어 이후 반대의 현재 known 상태가 나타남')
        if not updated and history != old_history:
            if not fresh_expected or obj['observations'] < before['observations'] + minimum_observations:
                _fail('새 관측 근거 없이 마지막 상태가 변경됨')
            updated = True
            first = stamp
        if updated:
            if obj['last_confirmed_state'] != expected_state or history[1] < previous_history[1]:
                _fail('재방문 뒤 마지막 확정 이력이 되돌아감')
            if history != previous_history and not fresh_expected:
                _fail('stale·unknown 관측에서 마지막 확정 이력을 갱신함')
            if fresh_expected:
                confirmed.add(stamp)
        previous_stamp, previous_count, previous_history = stamp, obj['observations'], history
    if not updated or len(confirmed) < minimum_observations:
        _fail('서로 다른 시각의 새로운 확정 관측이 부족함')
    return dict(id=identifier, position=_position(before['position']),
                previous_state=before['last_confirmed_state'], updated_state=expected_state,
                changed_sim_ns=changed, first_confirmation_stamp_ns=first,
                final_confirmed_stamp_ns=max(confirmed), confirmed_observations=len(confirmed),
                confirmation_after_command_sec=(first-changed)/1e9, passed=True)
