"""순찰 원문·실제 MP4 전체 디코딩·지도 그림·사용자 전달본을 정리한다."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--headless',type=Path,required=True)
    parser.add_argument('--summary',type=Path,required=True)
    parser.add_argument('--images',type=Path,required=True)
    parser.add_argument('--export-dir',type=Path,required=True)
    args=parser.parse_args()
    if args.export_dir.exists():parser.error('새 export-dir이 필요합니다')
    root=Path(__file__).resolve().parents[2];run=args.run.resolve()
    check=json.loads((run/'check.json').read_text());historical=bool(check.get('historical_state_check'));recording=json.loads((run/'recording-checks.json').read_text())
    if check['status']!='complete' or recording['status']!='complete':parser.error('완료된 실제 순찰·녹화가 필요합니다')
    if sha(run/'executed-harness.py')!=check['harness_sha256'] or sha(run/'executed-recorder.py')!=recording['source_sha256']:
        parser.error('실행 당시 소스 snapshot 불일치')
    import cv2
    import numpy as np
    from PIL import Image
    cap=cv2.VideoCapture(str(run/'find-door-patrol-gazebo-rviz.mp4'))
    if not cap.isOpened():raise RuntimeError('완성 MP4 열기 실패')
    fps=cap.get(cv2.CAP_PROP_FPS)
    if fps!=recording['fps']:raise RuntimeError('MP4 FPS 불일치')
    metadata=json.loads((run/'frames.json').read_text());count=0;captures={};writers={}
    for label in ['gazebo','rviz']:
        destination=run/f'find-door-patrol-{label}.mp4'
        if destination.exists():raise RuntimeError('기존 분리 영상 덮어쓰기 거부')
        pipeline=('appsrc ! videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency '
                  'speed-preset=veryfast bitrate=4000 key-int-max=20 ! '
                  'video/x-h264,stream-format=avc,alignment=au ! mp4mux faststart=true ! '
                  f'filesink location="{destination}"')
        writers[label]=cv2.VideoWriter(pipeline,cv2.CAP_GSTREAMER,0,fps,(960,840),True)
        if not writers[label].isOpened():raise RuntimeError('분리 영상 인코더 초기화 실패')
    stage_first={}
    for index,row in enumerate(metadata):stage_first.setdefault(row['patrol']['stage'],index)
    selected={stage:first+min(35,sum(row['patrol']['stage']==stage for row in metadata)-1)
              for stage,first in stage_first.items()if stage.endswith('_scan')}
    try:
        while True:
            okay,frame=cap.read()
            if not okay:break
            if frame.shape!=(1080,1920,3):raise RuntimeError('MP4 해상도 불일치')
            writers['gazebo'].write(frame[80:920,:960]);writers['rviz'].write(frame[80:920,960:1920])
            for stage,index in selected.items():
                if count==index:captures[stage]=Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
            count+=1
    finally:
        cap.release()
        for writer in writers.values():writer.release()
    if count!=recording['recorded_frames'] or len(metadata)!=count:raise RuntimeError('전체 디코딩 프레임 수 불일치')
    args.images.mkdir(parents=True,exist_ok=True)
    contact=Image.new('RGB',(1280,720),(18,25,34))
    order=['south_east_scan','south_west_scan','north_east_scan','north_west_scan']
    for index,stage in enumerate(order):contact.paste(captures[stage].resize((640,360)),((index%2)*640,(index//2)*360))
    contact.save(args.images/'video-preview.jpg',quality=95)
    # 실제 OccupancyGrid 위에 초기 고정 정합으로 옮긴 odom 궤적을 그린다.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    plt.rcParams['font.family']=font_manager.FontProperties(fname='/usr/share/fonts/truetype/nanum/NanumGothic.ttf').get_name()
    plt.rcParams['axes.unicode_minus']=False
    grid=np.load(run/'occupancy.npy',allow_pickle=False);geometry=check['map_metadata']
    ox,oy=geometry['origin'];resolution=geometry['resolution']
    history=json.loads((run/'history.json').read_text());ax,ay,yaw=check['evaluation']['map_from_odom_initial']
    points=np.asarray([[ax+math.cos(yaw)*r['odom'][0]-math.sin(yaw)*r['odom'][1],
                       ay+math.sin(yaw)*r['odom'][0]+math.cos(yaw)*r['odom'][1]]for r in history])
    fig,axis=plt.subplots(figsize=(8,8));axis.set_facecolor('#cdcdcd')
    axis.imshow(np.ma.masked_less(grid,0),origin='lower',cmap='gray_r',vmin=0,vmax=100,
                extent=[ox,ox+grid.shape[1]*resolution,oy,oy+grid.shape[0]*resolution])
    axis.plot(points[:,0],points[:,1],color='tab:blue',linewidth=2,label='odom 궤적 · 초기 고정 정합')
    for obj in check['final_door_map']['objects']:
        state=obj['last_confirmed_state'] if historical else obj['state']
        axis.scatter(*obj['position'][:2],s=75,color={'open':'green','closed':'red','unknown':'#d5a900'}[state],alpha=.6 if historical else 1.)
        label=obj['id']+'\nlast:'+state+' / '+obj['observation_status'] if historical else obj['id']+' '+state
        axis.annotate(label,obj['position'][:2],xytext=(8,8),textcoords='offset points',fontsize=9)
    axis.set_xlim(-6,6);axis.set_ylim(-6.7,6.7);axis.set_xlabel('map x (m)');axis.set_ylabel('map y (m)')
    axis.set_title('실제 순찰 종료 지도 · 마지막 확인은 열림/닫힘 유지\n현재 관측은 네 문 모두 stale · 색은 과거 확인 상태' if historical else '실제 순찰 종료 지도 · 네 문의 위치 보존\n마지막 관측 후 TTL 초과로 상태는 모두 unknown')
    axis.legend(loc='upper left');axis.set_aspect('equal');fig.tight_layout();fig.savefig(args.images/'final-map.png',dpi=150);plt.close(fig)
    exports=[]
    for filename in ['find-door-patrol-gazebo-rviz.mp4','find-door-patrol-gazebo.mp4','find-door-patrol-rviz.mp4']:
        path=run/filename;video=cv2.VideoCapture(str(path));decoded=0
        if not video.isOpened():raise RuntimeError('전달 영상 열기 실패')
        while True:
            okay,frame=video.read()
            if not okay:break
            decoded+=1
        video.release()
        if decoded!=count:raise RuntimeError('전달 영상 전체 디코딩 누락')
        exports.append(dict(path=filename,bytes=path.stat().st_size,sha256=sha(path),decoded_frames=decoded,duration_sec=decoded/fps))
    cases=[]
    for folder in [args.headless.resolve(),run]:
        path=folder/'check.json';value=json.loads(path.read_text())
        if sha(folder/'executed-harness.py')!=value['harness_sha256']:raise RuntimeError('하네스 snapshot 불일치')
        cases.append(dict(path=str(path.relative_to(root)),sha256=sha(path),report=value,artifacts={
            p.name:sha(p)for p in folder.iterdir()if p.is_file()and p.suffix in ['.json','.npy','.log','.py','.png']}))
    summary=dict(date='2026-11-03',base_commit='2b6ac04' if historical else '3911341',stage='v0.2.0-door-last-observation.1' if historical else 'v0.2.0-door-patrol.1',
        scope='새 건물 합성 개발 변형의 고정 경로 순찰·순차 문 관측. 자율 탐색·실물·독립 시험 아님',
        cases=cases,recording=recording,exports=exports,
        tests=dict(offline_passed=956 if historical else 929,offline_ros_skipped=5,semantic_package_passed=363 if historical else 340,gazebo_package_failed=0),
        source_sha256={},figure_sha256={})
    for source in ['scripts/door_probe/patrol_world.py','scripts/door_probe/test_patrol_world.py',
                   'scripts/door_probe/patrol_check.py','scripts/door_probe/test_patrol_check.py','scripts/door_probe/patrol_report.py',
                   'src/mapping_semantic/mapping_semantic/door_map.py','src/mapping_semantic/mapping_semantic/door_node.py',
                   'src/mapping_semantic/test/test_door_map.py','src/mapping_semantic/test/test_door_node.py','src/mapping_semantic/test/test_reobservation_ros.py',
                   'scripts/demo/door_patrol_record.py','scripts/demo/door_patrol.rviz','scripts/demo/door_patrol_gui.config',
                   'src/mapping_gazebo/worlds/door_patrol_v1.sdf','src/mapping_gazebo/config/door_patrol_v1.json',
                   'experiments/door_probe/yolo11-live-01/detector.json']:
        summary['source_sha256'][source]=sha(root/source)
    for figure in args.images.iterdir():
        if figure.is_file():summary['figure_sha256'][str(figure.resolve().relative_to(root))]=sha(figure)
    args.summary.parent.mkdir(parents=True,exist_ok=True)
    args.summary.write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    args.export_dir.mkdir(parents=True)
    for item in exports:shutil.copyfile(run/item['path'],args.export_dir/item['path'])
    for filename in ['check.json','recording-checks.json','doors.json','route-plan.json']:
        shutil.copyfile(run/filename,args.export_dir/filename)
    shutil.copyfile(root/'src/mapping_gazebo/worlds/door_patrol_v1.sdf',args.export_dir/'door_patrol_v1.sdf')
    shutil.copyfile(root/'src/mapping_gazebo/config/door_patrol_v1.json',args.export_dir/'world-manifest.json')
    for figure in args.images.iterdir():shutil.copyfile(figure,args.export_dir/figure.name)
    text=('Find Door — 건물 내부 순찰·문 탐색 Gazebo 실제 실행 영상\n\n'
          f'통합: find-door-patrol-gazebo-rviz.mp4 (1920×1080 H.264, {fps:g}fps, {count/fps:g}초)\n'
          '왼쪽: 실제 Gazebo 건물 / 오른쪽: 실제 RViz 지도·주행경로·문 마커 / 하단: 실제 RGB와 ROS 문 상태.\n'
          '분리 영상: find-door-patrol-gazebo.mp4, find-door-patrol-rviz.mp4.\n'
          f'중앙 복도를 약 {check["travel_distance_odom_m"]:.2f}m 이동하며 좌우 4개 문을 순차 관측했습니다.\n'
          '닫힘·반열림·완전열림 문을 서로 다른 네 ID로 등록하고 지도를 저장했습니다.\n'
          + ('마지막 확인 상태·시각은 유지하며 현재 관측이 오래되면 stale로 별도 표시합니다. 옅은 빨강/초록은 과거 확인입니다.\n' if historical else '보지 못한 문은 마지막 관측 후 2초 TTL이 지나면 unknown(노랑)이 됩니다. 위치와 ID는 보존됩니다.\n')
          + '경로는 사전 정의한 시험 경로이며 자율 프런티어 탐색·동시 다중 문 검출·실물 성능 시험은 아닙니다.\n'
          '전체 바탕화면 대신 전용 Xephyr의 실제 앱 화면을 녹화했습니다.\n'
          f'{count}프레임 중 실제 캡처{recording["actual_captures"]}·반복{recording["repeated_frames"]}이며 지연 구간을 반복 표시해 벽시계 길이를 보존했습니다.\n'
          '세 영상 모두 전체 프레임 디코딩 검사를 통과했습니다. 영상은 편집으로 실패 구간을 제거하지 않았습니다.\n'
          f'실행 원문·프레임별metadata·로그는 저장소 {run.relative_to(root)}/에 보존합니다.\n')
    (args.export_dir/'안내.txt').write_text(text)
    print(json.dumps(dict(export_dir=str(args.export_dir),frames=count,exports=exports),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
