"""실제 Gazebo의 알려진 문 재관측 ON/OFF 지도 상태를 비교한다."""
import argparse
import json
from pathlib import Path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reports',nargs=2,type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('새 출력 경로가 필요합니다')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    font=font_manager.FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
    plt.rcParams['font.family']=font.get_name()
    fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
    seen=set()
    for path in args.reports:
        report=json.loads(path.read_text())
        if report['status']!='complete' or report['mode']!='known_door_rgbd_reobservation':
            parser.error('완료된 기존 문 재관측 기록이 필요합니다')
        enabled=report['reobserve_known_doors']
        if enabled in seen:parser.error('ON/OFF 두 기록이 필요합니다')
        seen.add(enabled)
        for ax,event in zip(axes,report['known_door_cycle']):
            stamps=[(row['stamp_ns']-event['begin_sim_ns'])/1e9 for row in event['history']]
            states=[{'closed':0,'unknown':.5,'open':1}[row['objects'][0]['state']] for row in event['history']]
            ax.step(stamps,states,where='post',color='tab:green' if enabled else 'tab:orange',
                    label='현재 RGB-D 재관측 켬' if enabled else '현재 RGB-D 재관측 끔')
    for ax,expected,title in zip(axes,[1,0],['90° 완전 열림으로 변경','0° 닫힘으로 변경']):
        ax.axhline(expected,color='gray',linestyle=':',linewidth=1,label='배치 정답 상태')
        ax.set_yticks([0,.5,1],['닫힘','불명','열림']);ax.set_ylim(-.15,1.15)
        ax.set_title(title);ax.set_ylabel('지도 상태 출력');ax.grid(alpha=.3);ax.legend(loc='best')
    axes[-1].set_xlabel('각도 변경 전 최근 지도 시각부터 경과 (시뮬레이션 초)')
    fig.suptitle('반열림에서 먼저 확인한 같은 문 · 재관측 ON/OFF 개발 진단')
    fig.tight_layout()
    args.output.parent.mkdir(parents=True,exist_ok=True);fig.savefig(args.output,dpi=160);plt.close(fig)


if __name__=='__main__':main()
