"""보조 시점 진단의 실제 RGB 여섯 장과 예측 박스만 비교 그림으로 보존한다."""
import argparse
import json
from pathlib import Path
from PIL import Image
from capture import digest


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('새 그림 경로가 필요합니다')
    d=json.loads(args.results.read_text())
    if d['status']!='complete' or len(d['frames'])!=6:parser.error('완료된 여섯 장 결과가 필요합니다')
    frames={(f['state'],f['view']):f for f in d['frames']}
    if len(frames)!=6:parser.error('중복 결과가 있습니다')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.patches import Rectangle
    font=font_manager.FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
    plt.rcParams['font.family']=font.get_name()
    fig,axes=plt.subplots(3,2,figsize=(9,10))
    for row,(state,label) in enumerate([('closed','닫힘 0°'),('half','반열림 45°'),('open','완전 열림 90°')]):
        for col,(view,title) in enumerate([('low','로봇 낮은 시점'),('high','전체 문 보조 시점')]):
            frame=frames[(state,view)]
            if digest(frame['rgb'])!=frame['sha256']:raise ValueError('RGB 해시 불일치')
            ax=axes[row,col]
            with Image.open(frame['rgb'])as image:ax.imshow(image)
            for door in frame['doors']:
                x1,y1,x2,y2=door['bbox']
                ax.add_patch(Rectangle((x1,y1),x2-x1,y2-y1,fill=False,edgecolor='#00aaff',linewidth=1.5))
                ax.text(x1+2,y1+10,f"door {door['confidence']:.3f}",color='white',fontsize=8,
                        bbox=dict(facecolor='#005577',alpha=.85,pad=1))
            ax.set_title(f"{label} · {title}\ndoor 검출 {frame['door_count']}개",fontsize=11)
            ax.set_xlim(0,640);ax.set_ylim(480,0);ax.axis('off')
    fig.suptitle('평판 금속문 3m · 같은 YOLOE · 높이 비교 개발 진단',fontsize=14)
    fig.tight_layout(rect=(0,0,1,.96))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=150)
    plt.close(fig)


if __name__=='__main__':main()
