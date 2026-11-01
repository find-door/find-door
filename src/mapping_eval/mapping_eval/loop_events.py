"""LoggingSolver 기록의 루프 수락을 정답 odom의 상대 변환과 비교한다.

연결의 생성 경로는 기록에 없으므로 최적화 직전 마지막 새 연결을 루프 연결로 추정한다.
새 연결이 없거나 마지막 연결이 연속 노드의 순차 연결이면 루프가 새 연결을 만들지 않은 것이므로
unknown으로 세고 정상·오연결 어느 쪽에도 넣지 않는다.
시뮬레이션 odom이 정답인 경우에만 의미가 있다.
"""

import argparse
import json
import math
from pathlib import Path
import sys


def _relative(source, target):
    dx, dy = target[0] - source[0], target[1] - source[1]
    c, s = math.cos(source[2]), math.sin(source[2])
    yaw = target[2] - source[2]
    return (c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(yaw), math.cos(yaw)))


def analyze_loops(path, translation_limit=0.5, yaw_limit_deg=5.0):
    if translation_limit <= 0 or yaw_limit_deg <= 0:
        raise ValueError('허용 오차는 양수여야 합니다')
    events = []
    for number, line in enumerate(Path(path).read_text(encoding='utf-8').splitlines(), 1):
        try:
            record = json.loads(line)
            pending = record['pending']
            event = {'seq': record['seq'], 'correction_count': record['correction_count']}
            if not pending:
                event['classification'] = 'unknown'
                events.append(event)
                continue
            edge = pending[-1]
            if edge['target']['id'] - edge['source']['id'] == 1:
                event['classification'] = 'unknown'
                events.append(event)
                continue
            truth = _relative(edge['source']['odom'], edge['target']['odom'])
            link = edge['link']
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise ValueError(f'{number}번째 줄을 해석할 수 없습니다: {exc}') from exc
        error_m = math.hypot(link[0] - truth[0], link[1] - truth[1])
        yaw = link[2] - truth[2]
        error_deg = abs(math.degrees(math.atan2(math.sin(yaw), math.cos(yaw))))
        consistent = error_m < translation_limit and error_deg < yaw_limit_deg
        event.update({
            'source_id': edge['source']['id'], 'target_id': edge['target']['id'],
            'target_time_s': edge['target']['time'],
            'source_odom': edge['source']['odom'], 'target_odom': edge['target']['odom'],
            'true_distance_m': math.hypot(truth[0], truth[1]),
            'translation_error_m': error_m, 'yaw_error_deg': error_deg,
            'pending_edge_count': len(pending),
            'classification': 'consistent' if consistent else 'contradicts_truth',
        })
        events.append(event)
    counts = {name: sum(e['classification'] == name for e in events)
              for name in ('consistent', 'contradicts_truth', 'unknown')}
    return {
        'schema_version': 1, 'accepted_count': len(events), **counts,
        'translation_limit_m': translation_limit, 'yaw_limit_deg': yaw_limit_deg,
        'events': events,
        'limits': '마지막 새 연결을 루프 연결로 추정; 거절된 시도는 기록되지 않음; 허용 오차는 잠정값',
    }


def main(args=None):
    parser = argparse.ArgumentParser(description='수락된 루프 연결을 정답 odom과 비교')
    parser.add_argument('--log', required=True, help='LoggingSolver JSONL 경로')
    parser.add_argument('--output', required=True, help='새 JSON 결과 파일')
    options = parser.parse_args(args)
    try:
        report = analyze_loops(options.log)
        output = Path(options.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    except (OSError, ValueError) as exc:
        print(f'루프 분석 실패: {exc}', file=sys.stderr)
        return 1
    print(f"루프 분석 완료: 수락 {report['accepted_count']}회, "
          f"정답과 모순 {report['contradicts_truth']}회, 미상 {report['unknown']}회 → {output}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
