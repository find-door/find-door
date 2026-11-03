"""고정 출입구 위치와 시간에 따른 상태를 관리하는 ROS 독립 지도.

position은 호출자가 추정한 고정 문턱 위치다. 회전하는 문짝 표면 대표점으로
대체하면 안 된다. 시간은 관측 기준 나노초이며 reset만 시각 epoch를 바꾼다.
"""

from copy import deepcopy
import math
import re


MAX_ID = 2147483647
STATES = ('open', 'closed', 'unknown')


def _stamp(value):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError('시각은 양의 정수 나노초여야 합니다')
    return value


def _position(value):
    if (not isinstance(value, (list, tuple)) or len(value) != 3
            or any(isinstance(x, bool) or not isinstance(x, (int, float))
                   or not math.isfinite(x) for x in value)):
        raise ValueError('고정 문 위치는 유한한 세 좌표여야 합니다')
    return list(map(float, value))


class DoorMap:
    """세 번의 연속 상태 근거를 확인하며 위치 anchor와 ID는 고정한다.

    snapshot은 시각 watermark를 진행시킨다. 현재 state는 TTL과 판단 불가에서
    unknown이 되지만 last_confirmed_*는 마지막 실제 확정 관측을 보존한다.
    load는 투표 근거를 복원하지 않아 새 관측에서 다시 연속 근거를 모아야 한다.
    """

    def __init__(self, map_id, frame='map', association_distance=.25,
                 minimum_observations=3, state_ttl_sec=2.):
        if any(not isinstance(x, str) or not x.strip() for x in (map_id, frame)):
            raise ValueError('map_id와 frame을 명시해야 합니다')
        for value in (association_distance, state_ttl_sec):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError('거리와 TTL은 유한한 양수여야 합니다')
        if (isinstance(minimum_observations, bool) or not isinstance(minimum_observations, int)
                or minimum_observations < 1):
            raise ValueError('최소 관측 횟수는 양의 정수여야 합니다')
        self.map_id, self.frame = map_id, frame
        self._config = dict(association_distance=float(association_distance),
                           minimum_observations=minimum_observations,
                           state_ttl_sec=float(state_ttl_sec), ambiguity_distance_m=.05)
        scaled_ttl = state_ttl_sec * 1_000_000_000
        if not math.isfinite(scaled_ttl) or not 1 <= scaled_ttl <= 2**63 - 1:
            raise ValueError('TTL은 1나노초 이상 int64 범위여야 합니다')
        self._ttl_ns = int(scaled_ttl)
        self._objects = []
        self.next_id = 1
        self._watermark = 0

    @property
    def config(self):
        return self._config.copy()

    def _time(self, stamp):
        stamp = _stamp(stamp)
        if stamp < self._watermark:
            raise ValueError('시각이 역행했습니다. 새 epoch에는 reset을 사용하세요')
        return stamp

    @staticmethod
    def _forget(record):
        record.update(state='unknown', _candidate=None, _votes=0)

    @staticmethod
    def _public(record):
        return deepcopy({key: value for key, value in record.items() if not key.startswith('_')})

    def _expire(self, now):
        for record in self._objects:
            if now - record['last_stamp_ns'] > self._ttl_ns:
                self._forget(record)

    def observe(self, position, state, stamp_ns, expected_id=None):
        position, stamp = _position(position), self._time(stamp_ns)
        if state not in STATES:
            raise ValueError('state는 open/closed/unknown이어야 합니다')
        candidates = sorted((math.dist(position, item['position']), index)
                            for index, item in enumerate(self._objects)
                            if math.dist(position, item['position']) <= self.config['association_distance'])
        if len(candidates) > 1 and candidates[1][0] - candidates[0][0] < self.config['ambiguity_distance_m']:
            raise ValueError('출입구 연결이 모호합니다')
        if expected_id is not None and (not candidates or self._objects[candidates[0][1]]['id'] != expected_id):
            raise ValueError('기존 문 ID와 연결되지 않아 재관측을 거부합니다')
        if candidates:
            record = self._objects[candidates[0][1]]
            if stamp == record['last_stamp_ns']:
                return self._public(record)
        else:
            if self.next_id > MAX_ID:
                raise ValueError('문 ID 공간이 소진됐습니다')
            record = dict(id=f'door_{self.next_id:06d}', position=position, state='unknown',
                          last_stamp_ns=stamp, observations=0,
                          last_confirmed_state='unknown', last_confirmed_stamp_ns=None,
                          _candidate=None, _votes=0)
            self.next_id += 1
            self._objects.append(record)
        self._expire(stamp)
        if state == 'unknown':
            self._forget(record)
        else:
            if record['_candidate'] != state:
                self._forget(record)
                record['_candidate'] = state
            record['_votes'] += 1
            record['state'] = state if record['_votes'] >= self.config['minimum_observations'] else 'unknown'
            if record['state'] != 'unknown':
                record['last_confirmed_state'] = state
                record['last_confirmed_stamp_ns'] = stamp
        record['last_stamp_ns'] = stamp
        record['observations'] += 1
        self._watermark = stamp
        return self._public(record)

    def snapshot(self, now_ns):
        now = self._time(now_ns)
        self._expire(now)
        self._watermark = now
        return [self._public(record) for record in self._objects]

    def reset(self):
        self._objects.clear()
        self._watermark = 0

    def dump(self, now_ns):
        return dict(door_map_schema_version=2, map_id=self.map_id, frame=self.frame,
                    config=self.config.copy(), next_id=self.next_id, watermark_ns=now_ns,
                    objects=self.snapshot(now_ns))

    def load(self, document, now_ns):
        """지도 계약·수치·시각을 모두 확인한 다음 한 번에 교체한다."""
        now = self._time(now_ns)
        keys = {'door_map_schema_version', 'map_id', 'frame', 'config', 'next_id', 'watermark_ns', 'objects'}
        if (not isinstance(document, dict) or set(document) != keys
                or type(document['door_map_schema_version']) is not int or document['door_map_schema_version'] not in (1, 2)
                or document['map_id'] != self.map_id or document['frame'] != self.frame
                or document['config'] != self.config
                or not isinstance(document['config'], dict)
                or any(type(document['config'].get(key)) is not type(value)
                       for key, value in self.config.items())):
            raise ValueError('문 지도 저장 계약이 다릅니다')
        watermark = _stamp(document['watermark_ns'])
        if watermark > now or not isinstance(document['objects'], list):
            raise ValueError('저장 시각 또는 객체 목록이 잘못됐습니다')
        next_id = document['next_id']
        if type(next_id) is not int or not 1 <= next_id <= MAX_ID + 1:
            raise ValueError('next_id 범위 오류')
        loaded, ids, numbers = [], set(), []
        for source in document['objects']:
            fields = {'id', 'position', 'state', 'last_stamp_ns', 'observations'}
            if document['door_map_schema_version'] == 2:
                fields |= {'last_confirmed_state', 'last_confirmed_stamp_ns'}
            if not isinstance(source, dict) or set(source) != fields:
                raise ValueError('문 객체 필드 오류')
            identifier = source['id']
            if not isinstance(identifier, str) or not re.fullmatch(r'door_[0-9]{6,10}', identifier):
                raise ValueError('문 ID 형식 오류')
            number = int(identifier[5:])
            if not 1 <= number <= MAX_ID or identifier != f'door_{number:06d}' or identifier in ids:
                raise ValueError('문 ID 범위·중복 오류')
            stamp = _stamp(source['last_stamp_ns'])
            if stamp > watermark or source['state'] not in STATES:
                raise ValueError('문 상태 또는 관측 시각 오류')
            if type(source['observations']) is not int or source['observations'] < 1:
                raise ValueError('관측 횟수 오류')
            if source['state'] != 'unknown' and source['observations'] < self.config['minimum_observations']:
                raise ValueError('확정 상태의 관측 횟수가 부족합니다')
            if document['door_map_schema_version'] == 1:
                historical_state = source['state']
                historical_stamp = stamp if historical_state != 'unknown' else None
            else:
                historical_state = source['last_confirmed_state']
                historical_stamp = source['last_confirmed_stamp_ns']
                if historical_state not in STATES:
                    raise ValueError('마지막 확정 상태 오류')
                if historical_state == 'unknown':
                    if historical_stamp is not None:
                        raise ValueError('확정 이력 없는 상태에 시각이 있습니다')
                elif (_stamp(historical_stamp) > stamp
                      or source['observations'] < self.config['minimum_observations']):
                    raise ValueError('마지막 확정 시각 또는 관측 횟수 오류')
                if source['state'] != 'unknown' and (historical_state != source['state'] or historical_stamp != stamp):
                    raise ValueError('현재 확정 상태와 마지막 확정 이력이 다릅니다')
            item = dict(source, position=_position(source['position']),
                        last_confirmed_state=historical_state,
                        last_confirmed_stamp_ns=historical_stamp,
                        _candidate=None, _votes=0)
            if now - stamp > self._ttl_ns:
                self._forget(item)
            loaded.append(item)
            ids.add(identifier)
            numbers.append(number)
        if numbers and next_id <= max(numbers):
            raise ValueError('next_id가 기존 ID보다 커야 합니다')
        self._objects, self.next_id, self._watermark = loaded, max(self.next_id, next_id), now
