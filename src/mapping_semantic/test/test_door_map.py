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


def _confirmed_map():
    m = DoorMap('room')
    for stamp in (1, 2, 3):
        m.observe([0, 0, 0], 'closed', stamp)
    return m


def test_last_confirmation_survives_unknown_transition_and_ttl():
    m = _confirmed_map()
    current = m.observe([0, 0, 0], 'unknown', 4)
    assert current['state'] == 'unknown'
    assert (current['last_confirmed_state'], current['last_confirmed_stamp_ns']) == ('closed', 3)
    for stamp in (5, 6):
        current = m.observe([0, 0, 0], 'open', stamp)
        assert current['state'] == 'unknown'
        assert (current['last_confirmed_state'], current['last_confirmed_stamp_ns']) == ('closed', 3)
    current = m.observe([0, 0, 0], 'open', 7)
    assert (current['state'], current['last_confirmed_state'], current['last_confirmed_stamp_ns']) == ('open', 'open', 7)
    current = m.snapshot(3_000_000_000)[0]
    assert current['state'] == 'unknown'
    assert (current['last_confirmed_state'], current['last_confirmed_stamp_ns']) == ('open', 7)


def test_first_uncertain_has_no_invented_history_and_reset_erases_history():
    m = DoorMap('room')
    for stamp, state in ((1, 'closed'), (2, 'open'), (3, 'unknown')):
        row = m.observe([0, 0, 0], state, stamp)
        assert row['last_confirmed_state'] == 'unknown'
        assert row['last_confirmed_stamp_ns'] is None
    m = _confirmed_map()
    m.reset()
    row = m.observe([0, 0, 0], 'closed', 1)
    assert row['id'] == 'door_000002'
    assert row['last_confirmed_state'] == 'unknown'
    assert row['last_confirmed_stamp_ns'] is None


def test_same_confirmed_observation_advances_history_but_duplicate_does_not():
    m = _confirmed_map()
    row = m.observe([0, 0, 0], 'closed', 4)
    assert row['last_confirmed_stamp_ns'] == 4
    duplicate = m.observe([0, 0, 0], 'open', 4)
    assert duplicate == row
    unknown = m.observe([0, 0, 0], 'unknown', 5)
    assert unknown['last_confirmed_stamp_ns'] == 4


def test_schema2_roundtrip_preserves_expired_history_without_votes():
    m = _confirmed_map()
    document = json.loads(json.dumps(m.dump(4)))
    assert document['door_map_schema_version'] == 2
    other = DoorMap('room')
    other.load(document, 3_000_000_000)
    row = other.snapshot(3_000_000_000)[0]
    assert row['state'] == 'unknown'
    assert (row['last_confirmed_state'], row['last_confirmed_stamp_ns']) == ('closed', 3)
    row = other.observe([0, 0, 0], 'closed', 3_000_000_001)
    assert row['state'] == 'unknown'
    assert row['last_confirmed_stamp_ns'] == 3
    assert other.dump(3_000_000_002)['objects'][0]['last_confirmed_stamp_ns'] == 3


@pytest.mark.parametrize('known', [True, False])
def test_schema1_migration_only_infers_known_state(known):
    m = _confirmed_map()
    if not known:
        m.observe([0, 0, 0], 'unknown', 4)
    document = m.dump(5)
    document['door_map_schema_version'] = 1
    for row in document['objects']:
        row.pop('last_confirmed_state')
        row.pop('last_confirmed_stamp_ns')
    other = DoorMap('room')
    other.load(document, 3_000_000_000)
    row = other.snapshot(3_000_000_000)[0]
    assert row['state'] == 'unknown'
    assert row['last_confirmed_state'] == ('closed' if known else 'unknown')
    assert row['last_confirmed_stamp_ns'] == (3 if known else None)
    assert other.dump(3_000_000_000)['door_map_schema_version'] == 2


@pytest.mark.parametrize('change', [
    lambda row: row.pop('last_confirmed_state'),
    lambda row: row.update(extra_history='open'),
    lambda row: row.update(last_confirmed_state='half'),
    lambda row: row.update(last_confirmed_state='unknown'),
    lambda row: row.update(last_confirmed_stamp_ns=None),
    lambda row: row.update(last_confirmed_stamp_ns=True),
    lambda row: row.update(last_confirmed_stamp_ns=3.0),
    lambda row: row.update(last_confirmed_stamp_ns=0),
    lambda row: row.update(last_confirmed_stamp_ns=5),
    lambda row: row.update(last_confirmed_stamp_ns=2),
    lambda row: row.update(last_confirmed_state='open'),
    lambda row: row.update(observations=2),
])
def test_bad_schema2_history_is_atomic(change):
    m = _confirmed_map()
    before = m.dump(4)
    document = deepcopy(before)
    change(document['objects'][0])
    with pytest.raises(ValueError):
        m.load(document, 5)
    assert m.dump(4) == before


def test_unknown_history_requires_null_and_historical_stamp_not_after_observation():
    m = DoorMap('room')
    m.observe([0, 0, 0], 'unknown', 1)
    before = m.dump(2)
    document = deepcopy(before)
    document['objects'][0]['last_confirmed_stamp_ns'] = 1
    with pytest.raises(ValueError):
        m.load(document, 3)
    assert m.dump(2) == before
    m = _confirmed_map()
    m.observe([0, 0, 0], 'unknown', 4)
    document = m.dump(6)
    document['objects'][0]['last_confirmed_stamp_ns'] = 5
    with pytest.raises(ValueError):
        m.load(document, 6)


def test_schema1_rejects_history_fields_instead_of_silently_accepting_them():
    m = _confirmed_map()
    document = m.dump(4)
    document['door_map_schema_version'] = 1
    with pytest.raises(ValueError):
        m.load(document, 4)
