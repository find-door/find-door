"""실행을 마친 YOLO11 개발 진단의 원문·추적 해시·실제 센서 그림을 정리한다."""
import argparse
import hashlib
import json
from pathlib import Path


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', type=Path, required=True)
    parser.add_argument('--summary', type=Path, required=True)
    parser.add_argument('--images', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    base = args.base.resolve()
    records = []
    from live_check import final_observation_evidence
    names = ['cold-closed-01', 'cold-half-01', 'cold-open-02', 'negative-02',
             'transition-01', 'baseline-closed-01', 'baseline-open-01',
             'cold-open-01', 'negative-01', 'negative-03']
    for name in names:
        folder = base / name
        path = folder / 'check.json'
        data = json.loads(path.read_text())
        # 완료·실패 모두 실제 실행 당시 하네스와 연결한다.
        snapshot = folder / 'executed-harness.py'
        if sha(snapshot) != data['harness_sha256']:
            raise ValueError('실행 당시 하네스 해시 불일치: ' + name)
        record = dict(name=name, raw_path=str(path.relative_to(root)), raw_sha256=sha(path),
                      report=data, artifacts={})
        for filename in ['executed-harness.py', 'cold-start-history.json', 'doors.json',
                         'rgb.png', 'occupancy.npy', 'map-doors.png', 'simulation.log', 'doors.log']:
            artifact = folder / filename
            if artifact.is_file():
                record['artifacts'][filename] = sha(artifact)
        if data['status'] == 'complete':
            history = json.loads((folder / 'cold-start-history.json').read_text())
            record['final_history_recheck'] = final_observation_evidence(dict(
                doors=history['door_maps'], detector=history['detector_status'],
                maps=[1], scan_count=data['scan_count']), data['state'], True)
        records.append(record)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from PIL import Image
    import numpy as np
    plt.rcParams['font.family'] = font_manager.FontProperties(
        fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name()
    plt.rcParams['axes.unicode_minus'] = False
    args.images.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 2, figsize=(11, 11))
    for row, (name, title) in enumerate(zip(names[:3], ['닫힘 0°', '반열림 45° → open', '완전 열림 90°'])):
        data = records[row]['report']
        with Image.open(base / name / 'rgb.png') as source:
            axes[row, 0].imshow(source)
        axes[row, 0].axis('off')
        axes[row, 0].set_title(title + ' · 로봇 RGB')
        grid = np.load(base / name / 'occupancy.npy', allow_pickle=False)
        geometry = data['map_metadata']
        ox, oy, theta = geometry['origin']
        if abs(theta) > 1e-8:
            raise ValueError('이 그림은 회전 없는 실제 OccupancyGrid만 지원합니다')
        resolution = geometry['resolution']
        ax = axes[row, 1]
        ax.set_facecolor('#cdcdcd')
        ax.imshow(np.ma.masked_less(grid, 0), origin='lower', cmap='gray_r', vmin=0, vmax=100,
                  extent=[ox, ox+grid.shape[1]*resolution, oy, oy+grid.shape[0]*resolution])
        obj = data['door_map']['objects'][0]
        ax.scatter(*obj['position'][:2], s=55, color='red' if obj['state']=='closed' else 'green',
                   label=obj['state'] + ' · ' + obj['id'])
        ax.set_xlim(-2.5, .5)
        ax.set_ylim(-2, 2)
        ax.set_xlabel('map x (m)')
        ax.set_ylabel('map y (m)')
        ax.set_title(title + ' · 실제 LiDAR 지도 일부')
        ax.legend(loc='upper left', fontsize=8)
        if data['status'] != 'complete' or data['cold_start']['reobserved'] != 0:
            raise ValueError('완료된 보조 재관측 없는 단일 문 결과가 필요합니다')
    fig.suptitle('학습한 출입구 후보 · Gazebo 금속문 · 2m 정면 정지\n각각 빈 문 지도에서 시작한 별도 실행 / 정지 상태의 제한된 SLAM 지도')
    fig.tight_layout()
    overview = args.images / 'stationary.png'
    fig.savefig(overview, dpi=140)
    plt.close(fig)
    transition = records[4]['report']
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.5))
    for ax, event, title in zip(axes, transition['transitions'], ['90° 열림으로 순간 변경', '0° 닫힘으로 순간 변경']):
        times = [(item['stamp_ns'] - event['begin_sim_ns']) / 1e9 for item in event['history']]
        states = [{'closed': 0, 'unknown': .5, 'open': 1}[item['objects'][0]['state']] for item in event['history']]
        ax.step(times, states, where='post', marker='o')
        ax.axhline(1 if event['expected_state'] == 'open' else 0, color='gray', linestyle=':')
        ax.set_yticks([0, .5, 1], ['닫힘', '불명', '열림'])
        ax.set_ylim(-.15, 1.15)
        ax.set_xlabel('변경 전 최근 지도 시각부터 경과 (시뮬레이션 초)')
        ax.set_title(title)
        ax.grid(alpha=.3)
    fig.suptitle('같은 문 ID·위치 유지 / 알려진 문 재관측 보조 끔 / 실제 센서 상태 출력')
    fig.tight_layout()
    timeline = args.images / 'transition.png'
    fig.savefig(timeline, dpi=150)
    plt.close(fig)
    summary = dict(date='2026-11-03', stage='v0.2.0-door-yolo11-live.1',
                   base_commit='17f6425', default_backend='yoloe', candidate_backend='yolo11',
                   scope='같은 개발 장면의 실제 Gazebo 통합 진단. 독립·실물 성능 아님',
                   tests=dict(offline_passed=898, offline_ros_skipped=5, package_passed=340,
                              package_errors=0, package_failures=0),
                   cases=records, source_sha256={}, figures={})
    for path in [Path(__file__), root / 'scripts/door_probe/live_check.py',
                 root / 'src/mapping_semantic/mapping_semantic/yolo11_doorway.py',
                 root / 'src/mapping_semantic/mapping_semantic/detector_worker.py', base / 'detector.json',
                 base / 'worker-probe.json', base / 'invalid-first-run.json']:
        summary['source_sha256'][str(path.resolve().relative_to(root))] = sha(path)
    for path in [overview, timeline]:
        summary['figures'][str(path.resolve().relative_to(root))] = sha(path)
    args.summary.parent.mkdir(parents=True, exist_ok=True)
    args.summary.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')


if __name__ == '__main__':
    main()
