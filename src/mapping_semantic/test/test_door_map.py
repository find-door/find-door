"""고정 위치·상태 전이·시각 만료·원자적 복원을 확인한다."""

from copy import deepcopy
import json

import pytest

from mapping_semantic.door_map import DoorMap


def test_transition_preserves_fixed_anchor_and_id():
    m = DoorMap('room')
    for i in range(1, 4):
        record = m.observe([.01 * i, 0, 0], 'closed', i)
    assert record['state'] == 'closed'
    for i in range(4, 7):
        record = m.observe([.1, 0, 0], 'open', i)
        assert record['state'] == ('open' if i == 6 else 'unknown')
    assert record['position'] == [.01, 0, 0]
    assert record['id'] == 'door_000001'
    assert len(m.snapshot(7)) == 1


def test_duplicate_and_unknown_do_not_extend_confirmed_state():
    m = DoorMap('room')
    for i in range(1, 4):
        m.observe([0, 0, 0], 'closed', i)
    duplicate = m.observe([0, 0, 0], 'open', 3)
    assert duplicate['state'] == 'closed' and duplicate['observations'] == 3
    assert m.observe([0, 0, 0], 'unknown', 4)['state'] == 'unknown'
    assert m.observe([0, 0, 0], 'closed', 5)['state'] == 'unknown'


def test_expiry_discards_votes_even_without_snapshot():
    m = DoorMap('room', state_ttl_sec=1)
    for i in range(1, 4):
        m.observe([0, 0, 0], 'closed', i)
    assert m.observe([0, 0, 0], 'closed', 2_000_000_004)['state'] == 'unknown'
    assert m.snapshot(4_000_000_005)[0]['state'] == 'unknown'
    with pytest.raises(ValueError, match='역행'):
        m.observe([0, 0, 0], 'open', 4)


def test_ambiguity_rejected_without_mutation_and_far_position_new_id():
    m = DoorMap('room')
    a = m.observe([0, 0, 0], 'closed', 1)
    b = m.observe([.4, 0, 0], 'closed', 1)
    assert a['id'] != b['id']
    before = m.dump(2)
    with pytest.raises(ValueError, match='모호'):
        m.observe([.2, 0, 0], 'open', 3)
    assert m.dump(2) == before


def test_roundtrip_restarts_votes_and_keeps_last_timestamp():
    m = DoorMap('room')
    for i in range(1, 4):
        m.observe([0, 0, 0], 'closed', i)
    document = json.loads(json.dumps(m.dump(4)))
    other = DoorMap('room')
    other.load(document, 4)
    assert other.snapshot(4)[0]['state'] == 'closed'
    assert other.observe([0, 0, 0], 'closed', 5)['state'] == 'unknown'
    with pytest.raises(ValueError):
        other.observe([0, 0, 0], 'open', 3)


def test_expired_restore_and_reset_never_reuses_id():
    m = DoorMap('room', minimum_observations=1)
    m.observe([0, 0, 0], 'open', 1)
    document = m.dump(2)
    other = DoorMap('room', minimum_observations=1)
    other.load(document, 3_000_000_000)
    assert other.snapshot(3_000_000_000)[0]['state'] == 'unknown'
    other.reset()
    assert other.observe([0, 0, 0], 'closed', 1)['id'] == 'door_000002'


@pytest.mark.parametrize('change', [
    lambda d: d.update(map_id='other'),
    lambda d: d.update(frame='odom'),
    lambda d: d.update(next_id=1),
    lambda d: d.update(watermark_ns=100),
    lambda d: d['objects'][0].update(position=[float('nan'), 0, 0]),
    lambda d: d['objects'][0].update(last_stamp_ns=100),
    lambda d: d['objects'].append(deepcopy(d['objects'][0])),
    lambda d: d['objects'][0].update(id='door_0000001'),
])
def test_bad_load_is_atomic(change):
    m = DoorMap('room')
    m.observe([0, 0, 0], 'open', 1)
    before = m.dump(2)
    bad = deepcopy(before)
    change(bad)
    with pytest.raises(ValueError):
        m.load(bad, 3)
    assert m.dump(2) == before


@pytest.mark.parametrize('position,state,stamp', [
    ([float('inf'), 0, 0], 'open', 1), ([True, 0, 0], 'open', 1),
    ([0, 0], 'open', 1), ([0, 0, 0], 'half', 1),
    ([0, 0, 0], 'open', True), ([0, 0, 0], 'open', 0),
])
def test_invalid_observation(position, state, stamp):
    m = DoorMap('room')
    with pytest.raises(ValueError):
        m.observe(position, state, stamp)
    assert m.snapshot(1) == []


def test_config_cannot_be_changed_through_public_value():
    m = DoorMap('room')
    m.config['minimum_observations'] = 1
    assert m.observe([0, 0, 0], 'closed', 1)['state'] == 'unknown'


@pytest.mark.parametrize('kwargs', [
    {'map_id': ''}, {'map_id': 'x', 'state_ttl_sec': 1e300},
    {'map_id': 'x', 'minimum_observations': True},
    {'map_id': 'x', 'association_distance': float('nan')},
])
def test_invalid_configuration(kwargs):
    with pytest.raises(ValueError):
        DoorMap(**kwargs)


def test_boolean_loaded_config_and_unconfirmed_state_are_rejected():
    m = DoorMap('room', minimum_observations=1)
    m.observe([0, 0, 0], 'closed', 1)
    document = m.dump(2)
    document['config']['minimum_observations'] = True
    with pytest.raises(ValueError):
        m.load(document, 2)
    other = DoorMap('room')
    other.observe([0, 0, 0], 'closed', 1)
    document = other.dump(2)
    document['objects'][0]['state'] = 'closed'
    with pytest.raises(ValueError):
        other.load(document, 2)


def test_loading_old_document_cannot_reuse_allocated_id():
    m = DoorMap('room')
    m.observe([0, 0, 0], 'closed', 1)
    old = m.dump(2)
    assert m.observe([1, 0, 0], 'open', 3)['id'] == 'door_000002'
    m.reset()
    m.load(old, 4)
    assert m.observe([1, 0, 0], 'open', 5)['id'] == 'door_000003'


def test_ttl_boundary_and_id_exhaustion():
    m = DoorMap('room', minimum_observations=1, state_ttl_sec=1)
    m.observe([0, 0, 0], 'closed', 1)
    assert m.snapshot(1_000_000_001)[0]['state'] == 'closed'
    assert m.snapshot(1_000_000_002)[0]['state'] == 'unknown'
    m.next_id = 2147483648
    before = m.dump(1_000_000_002)
    with pytest.raises(ValueError, match='소진'):
        m.observe([1, 0, 0], 'open', 1_000_000_003)
    assert m.dump(1_000_000_002) == before
