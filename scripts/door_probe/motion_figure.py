"""직선·회전 진단 기록의 odom 변화와 상태 출력을 그림으로 보존한다."""

import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports', nargs='+', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('기존 그림은 덮어쓰지 않습니다')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font = font_manager.FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
    plt.rcParams.update({'font.family': font.get_name(), 'axes.unicode_minus': False})
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    rotation = None
    for path in args.reports:
        report = json.loads(path.read_text())
        modes = {'live_gazebo_yoloe_lidar_slam_motion', 'live_gazebo_yoloe_lidar_slam_rotate'}
        if report['status'] != 'complete' or report['mode'] not in modes:
            parser.error('완료된 직선·회전 진단만 그립니다')
        is_rotation = report['mode'].endswith('_rotate')
        if rotation is not None and is_rotation != rotation:
            parser.error('직선과 회전 진단은 별도 그림으로 기록합니다')
        rotation = is_rotation
        stages = report['rotation' if rotation else 'motion']
        samples = [sample for stage in stages for sample in stage['history']]
        t0 = stages[0]['begin_sim_ns']
        x0 = stages[0]['begin_odom_yaw' if rotation else 'begin_odom_x']
        times = [(sample['map']['stamp_ns']-t0)/1e9 for sample in samples]
        if rotation:
            xs = [math.degrees(math.atan2(math.sin(sample['odom_yaw']-x0), math.cos(sample['odom_yaw']-x0))) for sample in samples]
        else:
            xs = [sample['odom_xy'][0]-x0 for sample in samples]
        states = [{'closed': 0, 'unknown': .5, 'open': 1}[sample['map']['objects'][0]['state']] for sample in samples]
        label = '닫힌 문 관측' if report['state'] == 'closed' else '열린 문 관측'
        color = 'tab:red' if report['state'] == 'closed' else 'tab:green'
        axes[0].plot(times, xs, label=label, color=color)
        axes[1].step(times, states, where='post', label=label, color=color)
    axes[0].set_ylabel('odom yaw 변화 (°)' if rotation else 'odom x 변화 (m)')
    axes[0].set_title('Gazebo 제자리 회전·복귀 개발 진단 — odom' if rotation else 'Gazebo 후진·복귀 개발 진단 — 좌표 프레임 odom')
    axes[1].set_ylabel('문 상태 출력')
    axes[1].set_yticks([0, .5, 1], ['닫힘', '불명', '열림'])
    axes[1].set_ylim(-.15, 1.15)
    axes[1].set_xlabel('지도 출력 시각 기준 경과 시간 (시뮬레이션 초)')
    for axis in axes:
        axis.grid(alpha=.3)
        axis.legend()
    fig.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
