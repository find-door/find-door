"""재방문 채점이 ID 변경·미관측 갱신·과거 상태 조작으로 통과하지 않는지 검사한다."""

from copy import deepcopy

import pytest

from revisit import assert_identity, validate_away, validate_revisit


S = 1_000_000_000


def initial():
    return [dict(id=f'door_{i:06d}', position=[float(i*2), 0., 0.], state='unknown',
                 observation_status='stale', last_stamp_ns=S, observations=3,
                 last_confirmed_state='closed', last_confirmed_stamp_ns=S) for i in range(4)]


def snapshot(stamp, objects):
    return dict(stamp_ns=stamp, epoch=0, objects=deepcopy(objects))


def successful_revisit():
    objects = initial()
    before = deepcopy(objects[0])
    maps = [snapshot(6*S, objects)]
    for index in range(5):
        stamp = 8*S + index*200_000_000
        objects[0].update(last_stamp_ns=stamp, observations=4+index)
        if index >= 2:
            objects[0].update(state='open', observation_status='confirmed',
                              last_confirmed_state='open', last_confirmed_stamp_ns=stamp)
        else:
            objects[0].update(state='unknown', observation_status='uncertain')
        maps.append(snapshot(stamp+50_000_000, objects))
    objects[0].update(state='unknown', observation_status='stale')
    maps.append(snapshot(12*S, objects))
    return before, maps


def test_valid_revisit_requires_new_confirmations_and_allows_later_stale():
    before, maps = successful_revisit()
    result = validate_revisit(before, maps, 'open', 7*S, initial())
    assert result['passed'] and result['id'] == before['id']
    assert result['previous_state'] == 'closed' and result['updated_state'] == 'open'
    assert result['first_confirmation_stamp_ns'] == 8_400_000_000
    assert result['confirmed_observations'] == 3
    assert result['confirmation_after_command_sec'] == pytest.approx(1.4)
    assert result['final_confirmed_stamp_ns'] == 8_800_000_000
    assert before == initial()[0]


def test_away_requires_span_and_unchanged_history():
    before = initial()[0]
    maps = [snapshot(4*S, initial()), snapshot(6*S, initial())]
    assert validate_away(before, maps, initial())['passed']
    with pytest.raises(RuntimeError, match='2초'):
        validate_away(before, maps[:1], initial())


@pytest.mark.parametrize('field,value', [
    ('last_stamp_ns', 2*S), ('observations', 4),
    ('last_confirmed_state', 'open'), ('last_confirmed_stamp_ns', 2*S),
    ('state', 'closed'), ('observation_status', 'uncertain'),
])
def test_out_of_view_updates_are_rejected(field, value):
    maps = [snapshot(4*S, initial()), snapshot(6*S, initial())]
    maps[-1]['objects'][0][field] = value
    with pytest.raises(RuntimeError):
        validate_away(initial()[0], maps, initial())


@pytest.mark.parametrize('problem', ['missing', 'extra', 'duplicate', 'anchor', 'epoch', 'time'])
def test_identity_and_epoch_are_invariants(problem):
    before, maps = successful_revisit()
    if problem == 'missing':
        maps[-1]['objects'].pop()
    elif problem == 'extra':
        maps[-1]['objects'].append(dict(id='extra', position=[10., 0., 0.]))
    elif problem == 'duplicate':
        maps[-1]['objects'][1]['id'] = maps[-1]['objects'][0]['id']
    elif problem == 'anchor':
        maps[-1]['objects'][0]['position'][0] += .001
    elif problem == 'epoch':
        maps[-1]['epoch'] = 1
    else:
        maps[-1]['stamp_ns'] = 1
    with pytest.raises(RuntimeError):
        validate_revisit(before, maps, 'open', 7*S, initial())


@pytest.mark.parametrize('problem', ['unknown_update', 'stale_update', 'past_command', 'fake_time',
                                     'duplicate_vote', 'opposite_known', 'history_reverts', 'too_few_votes',
                                     'same_source_history_change', 'observation_regression'])
def test_forged_or_insufficient_revisit_evidence_cannot_pass(problem):
    before, maps = successful_revisit()
    first = maps[3]['objects'][0]
    command = 7*S
    if problem == 'unknown_update':
        first.update(state='unknown', observation_status='uncertain')
    elif problem == 'stale_update':
        first.update(state='unknown', observation_status='stale')
    elif problem == 'past_command':
        command = 9*S
    elif problem == 'fake_time':
        first['last_confirmed_stamp_ns'] = first['last_stamp_ns']-1
    elif problem == 'duplicate_vote':
        maps = maps[:4] + [deepcopy(maps[3]) for _ in range(10)]
    elif problem == 'opposite_known':
        maps[1]['objects'][0].update(state='closed', observation_status='confirmed',
                                    last_confirmed_stamp_ns=8*S)
    elif problem == 'history_reverts':
        maps[-1]['objects'][0].update(last_confirmed_state='closed', last_confirmed_stamp_ns=S)
    elif problem == 'too_few_votes':
        first['observations'] = 5
    elif problem == 'same_source_history_change':
        first.update(last_stamp_ns=maps[2]['objects'][0]['last_stamp_ns'],
                     last_confirmed_stamp_ns=maps[2]['objects'][0]['last_stamp_ns'])
    else:
        maps[-1]['objects'][0]['last_stamp_ns'] = S
    with pytest.raises(RuntimeError):
        validate_revisit(before, maps, 'open', command, initial())


def test_same_state_is_not_a_change_test_and_empty_history_fails():
    before, maps = successful_revisit()
    with pytest.raises(RuntimeError):
        validate_revisit(before, maps, 'closed', 7*S, initial())
    with pytest.raises(RuntimeError):
        assert_identity(initial(), [])


def test_reverse_change_open_to_closed_uses_same_contract():
    before, maps = successful_revisit()
    before['last_confirmed_state'] = 'open'
    for mapping in maps:
        obj = mapping['objects'][0]
        obj['last_confirmed_state'] = 'closed' if obj['last_confirmed_state'] == 'open' else 'open'
        if obj['state'] == 'open':
            obj['state'] = 'closed'
    assert validate_revisit(before, maps, 'closed', 7*S, initial())['passed']


def test_stale_current_known_and_changed_before_anchor_are_rejected():
    before, maps = successful_revisit()
    maps[-1]['objects'][0].update(state='open', observation_status='confirmed')
    with pytest.raises(RuntimeError, match='오래된'):
        validate_revisit(before, maps, 'open', 7*S, initial())
    before, maps = successful_revisit()
    before['position'][0] = .1
    with pytest.raises(RuntimeError, match='anchor'):
        validate_revisit(before, maps, 'open', 7*S, initial())
