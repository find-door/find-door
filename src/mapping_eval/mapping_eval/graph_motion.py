"""SLAM 그래프 노드 위치 변화로 지도 이상 후보를 찾는다. 정답 벽 없이 동작한다.

현재 위치가 정상이어도 과거 노드가 크게 밀리면 지도가 망가진다.
큰 이동은 잘못된 루프 연결일 수도, 정당한 드리프트 보정일 수도 있으므로
이상 후보로만 보고하고 원인이나 지도 정확도를 판정하지 않는다.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys


def _read_snapshots(path):
    data = Path(path).read_bytes()
    snapshots = []
    for number, line in enumerate(data.decode('utf-8').splitlines(), 1):
        try:
            record = json.loads(line)
            nodes = {}
            for node in record['nodes']:
                if node['frame_id'] != 'map':
                    raise ValueError('map 프레임이 아닌 노드입니다')
                x, y = float(node['position'][0]), float(node['position'][1])
                if not (math.isfinite(x) and math.isfinite(y)):
                    raise ValueError('노드 좌표가 유한하지 않습니다')
                if node['marker_id'] in nodes:
                    raise ValueError('노드 ID가 중복되었습니다')
                nodes[node['marker_id']] = (x, y)
            stamp = record['receive_ros_ns']
        except (KeyError, IndexError, TypeError, ValueError) as exc:
            raise ValueError(f'{number}번째 줄을 해석할 수 없습니다: {exc}') from exc
        if snapshots and stamp < snapshots[-1][0]:
            raise ValueError(f'{number}번째 줄의 시각이 역행했습니다')
        if nodes:
            snapshots.append((stamp, nodes))
    return snapshots, hashlib.sha256(data).hexdigest()


def analyze_graph(path, threshold=0.5):
    if isinstance(threshold, bool) or not math.isfinite(threshold) or threshold <= 0:
        raise ValueError('이동량 기준은 유한한 양수여야 합니다')
    snapshots, digest = _read_snapshots(path)
    if len(snapshots) < 2:
        raise ValueError('비교할 그래프 스냅샷이 2개 미만입니다')
    events = []
    max_step = 0.0
    first_seen = {}
    previous = None
    for stamp, nodes in snapshots:
        for marker_id, position in nodes.items():
            first_seen.setdefault(marker_id, position)
        if previous is not None:
            steps = {marker_id: math.dist(position, previous[marker_id])
                     for marker_id, position in nodes.items() if marker_id in previous}
            step = max(steps.values(), default=0.0)
            max_step = max(max_step, step)
            moved = sorted(marker_id for marker_id, value in steps.items() if value >= threshold)
            if moved:
                events.append({'receive_ros_ns': stamp, 'max_step_m': step,
                               'moved_node_count': len(moved), 'compared_node_count': len(steps),
                               'moved_node_ids': moved})
        previous = nodes
    final = snapshots[-1][1]
    drift = {marker_id: math.dist(position, first_seen[marker_id])
             for marker_id, position in final.items()}
    drifted = sorted(marker_id for marker_id, value in drift.items() if value >= threshold)
    return {
        'schema_version': 1, 'input_sha256': digest, 'threshold_m': threshold,
        'snapshot_count': len(snapshots), 'final_node_count': len(final),
        'max_step_m': max_step, 'max_final_drift_m': max(drift.values()),
        'step_event_count': len(events), 'step_events': events,
        'drifted_node_count': len(drifted), 'drifted_node_ids': drifted,
        'anomaly_candidate': bool(events or drifted),
        'limits': '이상 후보일 뿐 원인·지도 정확도 판정이 아님; 기준값은 시뮬레이션 잠정값',
    }


def main(args=None):
    parser = argparse.ArgumentParser(description='SLAM 그래프 노드 이동으로 지도 이상 후보 분석')
    parser.add_argument('--graph', required=True, help='graph.jsonl 경로')
    parser.add_argument('--output', required=True, help='새 JSON 결과 파일')
    parser.add_argument('--threshold', type=float, default=0.5,
                        help='이상 후보 최소 노드 이동량(m), 기본값 0.5')
    options = parser.parse_args(args)
    try:
        report = analyze_graph(options.graph, options.threshold)
        output = Path(options.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    except (OSError, ValueError) as exc:
        print(f'그래프 분석 실패: {exc}', file=sys.stderr)
        return 1
    print(f"그래프 분석 완료: 최대 이동 {report['max_step_m']:.3f} m, "
          f"이상 후보 {'있음' if report['anomaly_candidate'] else '없음'} → {output}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
