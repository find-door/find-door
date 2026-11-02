"""주행 진단 기록의 odom 변화와 상태 출력을 그림으로 보존한다."""

import argparse
import json
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
    for path in args.reports:
        report = json.loads(path.read_text())
        if report['status'] != 'complete' or report['mode'] != 'live_gazebo_yoloe_lidar_slam_motion':
            parser.error('완료된 주행 진단만 그립니다')
        stages = report['motion']
        samples = [sample for stage in stages for sample in stage['history']]
        t0, x0 = stages[0]['begin_sim_ns'], stages[0]['begin_odom_x']
        times = [(sample['map']['stamp_ns']-t0)/1e9 for sample in samples]
        xs = [sample['odom_xy'][0]-x0 for sample in samples]
        states = [{'closed': 0, 'unknown': .5, 'open': 1}[sample['map']['objects'][0]['state']] for sample in samples]
        label = '닫힌 문 관측' if report['state'] == 'closed' else '열린 문 관측'
        color = 'tab:red' if report['state'] == 'closed' else 'tab:green'
        axes[0].plot(times, xs, label=label, color=color)
        axes[1].step(times, states, where='post', label=label, color=color)
    axes[0].set_ylabel('odom x 변화 (m)')
    axes[0].set_title('Gazebo 후진·복귀 개발 진단 — 좌표 프레임 odom')
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
