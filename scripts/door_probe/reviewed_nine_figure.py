"""사용자 검수9장의 원본 RGB·두 정답 bbox·실제 YOLOE 예측을 비교한다."""
import argparse
import json
from pathlib import Path
from label_review import read_manifest
from label_evaluate import evaluate_dataset,verify_rgb
from complete_reviewed_nine import EXPECTED


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--labels',type=Path,required=True);p.add_argument('--predictions',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    if args.output.exists():p.error('새 출력 파일이 필요합니다')
    dataset,_=read_manifest(args.labels);frames={f['id']:f for f in dataset['frames']if f['review_status']=='reviewed'}
    if set(frames)!=EXPECTED:p.error('검수한 금속문9장이 필요합니다')
    predictions=json.loads(args.predictions.read_text());by_id={f['id']:f for f in predictions['frames']}
    report=evaluate_dataset(dataset,predictions,'gazebo','development')
    if report['status']!='evaluated':p.error('누락·오류 없는 검출 결과가 필요합니다')
    metrics={target:{f['id']:f for f in body['frames']if f['status']=='scored'}for target,body in report['targets'].items()}
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    from matplotlib.patches import Rectangle,Patch
    from PIL import Image
    font=font_manager.FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf')
    plt.rcParams['font.family']=font.get_name()
    fig,axes=plt.subplots(3,3,figsize=(12,12))
    for row,(state,label)in enumerate([('closed','닫힘 0°'),('half','반열림 45°'),('open','완전 열림 90°')]):
        for col,(suffix,view)in enumerate([('2m','2m 낮은 시점'),('3m_low','3m 낮은 시점'),('3m_high','3m 높은 시점')]):
            identifier=f'metal_{state}_{suffix}';f=frames[identifier];ax=axes[row,col]
            verify_rgb(args.labels,f)
            path=Path(f['rgb']['path']);path=path if path.is_absolute()else args.labels.resolve().parent/path
            with Image.open(path)as image:ax.imshow(image)
            for d in f['doors']:
                for target,color,style in [('leaf','#22cc44','-'),('doorway','#ff8800','--')]:
                    b=d[f'{target}_bbox']
                    if b is not None:ax.add_patch(Rectangle((b[0],b[1]),b[2]-b[0],b[3]-b[1],fill=False,edgecolor=color,linestyle=style,linewidth=1.2))
            for d in by_id[identifier]['detections']:
                if d['class']!='door':continue
                b=d['bbox'];ax.add_patch(Rectangle((b[0],b[1]),b[2]-b[0],b[3]-b[1],fill=False,edgecolor='#00aaff',linewidth=1.3))
            match=metrics['leaf'][identifier]['matches'];match2=metrics['doorway'][identifier]['matches']
            detected=any(d['class']=='door'for d in by_id[identifier]['detections'])
            detail=(f"문짝 IoU {match[0]['iou']:.3f} / 출입구 {match2[0]['iou']:.3f}"if match and match2 else
                    (f"문짝 TP {metrics['leaf'][identifier]['tp']} / 출입구 TP {metrics['doorway'][identifier]['tp']}"if detected else'유효 door 검출 없음'))
            ax.set_title(f'{label} · {view}\n{detail}',fontsize=10)
            ax.set_xlim(0,f['rgb']['width']);ax.set_ylim(f['rgb']['height'],0);ax.axis('off')
    fig.suptitle('사용자 검수 양성 9장 · 같은 개발 장면 · IoU 기준 0.5',fontsize=14)
    fig.legend(handles=[Patch(facecolor='none',edgecolor='#22cc44',label='검수 문짝'),
                        Patch(facecolor='none',edgecolor='#ff8800',linestyle='--',label='검수 출입구'),
                        Patch(facecolor='none',edgecolor='#00aaff',label='YOLOE 예측')],loc='lower center',ncol=3)
    fig.subplots_adjust(left=.02,right=.98,bottom=.07,top=.92,wspace=.05,hspace=.24)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.output,dpi=150);plt.close(fig)


if __name__=='__main__':main()
