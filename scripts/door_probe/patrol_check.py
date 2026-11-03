"""별도 건물에서 실제 cmd_vel 순찰·문 관측·LiDAR 지도·선택 녹화를 검사한다.

경로는 사전 정의한 개발 경로다. 문 정답은 종료 후 대응 평가에만 사용하며
검출·상태 추론·주행 정지 성공 조건으로 주입하지 않는다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[2]


def angle_error(target, current):
    return math.atan2(math.sin(target-current), math.cos(target-current))


def stages():
    """문 정답 상태나 검출 결과에 따라 수정하지 않는 고정 복도 순찰 계획."""
    result = []
    for station, y in [('south', -3.), ('north', 3.)]:
        result.append(dict(name=station+'_arrive', label='복도 '+('남쪽' if y<0 else '북쪽')+' 지점 이동',
                           kind='move', x=0., y=y, yaw=math.pi/2))
        for side, yaw in [('east', 0.), ('west', math.pi)]:
            label=('남쪽' if y<0 else '북쪽')+' · '+('우측' if side=='east' else '좌측')
            result.append(dict(name=station+'_'+side+'_turn', label=label+' 방향 탐색', kind='turn', yaw=yaw))
            result.append(dict(name=station+'_'+side+'_scan', label=label+' 문 관측', kind='scan', duration_sim_sec=7.))
        result.append(dict(name=station+'_face_corridor', label='복도 진행 방향 복귀', kind='turn', yaw=math.pi/2))
    result.append(dict(name='finish',label='복도 끝 지점 이동',kind='move',x=0.,y=4.5,yaw=math.pi/2))
    result.append(dict(name='save',label='전체 지도·문 위치 저장',kind='scan',duration_sim_sec=3.))
    return result


def command_for(stage, pose, speed=.35):
    """odom 좌표 오차를 몸체 Twist로 바꾼다. yaw 오차가 크면 먼저 회전한다."""
    x,y,yaw=pose
    error=angle_error(stage.get('yaw',yaw),yaw)
    if stage['kind']=='scan':return (0.,0.,0.),False
    angular=max(-.5,min(.5,1.5*error))
    if stage['kind']=='turn':return (0.,0.,angular),abs(error)<.025
    dx,dy=stage['x']-x,stage['y']-y
    distance=math.hypot(dx,dy)
    if distance<.06 and abs(error)<.04:return (0.,0.,0.),True
    if abs(error)>.15:return (0.,0.,angular),False
    gain=min(speed,.9*distance)/max(distance,1e-9)
    return ((math.cos(yaw)*dx+math.sin(yaw)*dy)*gain,
            (-math.sin(yaw)*dx+math.cos(yaw)*dy)*gain,angular),False


def directional_clearance(scan, velocity):
    """요청한 실제 몸체 이동 방향 ±25°의 유효 scan 최솟값. 미관측이면 거부."""
    vx,vy,_=velocity
    if math.hypot(vx,vy)<.005:return None
    direction=math.atan2(vy,vx)
    values=[distance for i,distance in enumerate(scan.ranges)
            if math.isfinite(distance) and scan.range_min<=distance<=scan.range_max
            and abs(angle_error(scan.angle_min+i*scan.angle_increment,direction))<=math.radians(25)]
    if len(values)<3:raise RuntimeError('이동 방향의 유효 LiDAR 표본 부족')
    return min(values)


def evaluate_observations(history, truth, alignment, maximum_distance=.45):
    """고정 초기 map←odom 정합으로 문별 최초/상태/중복을 사후 평가한다."""
    ax,ay,yaw=alignment
    expected={item['id']:(ax+math.cos(yaw)*item['center_world_xyz'][0]-math.sin(yaw)*item['center_world_xyz'][1],
                          ay+math.sin(yaw)*item['center_world_xyz'][0]+math.cos(yaw)*item['center_world_xyz'][1])
              for item in truth}
    matches={key:dict(ids=set(),fresh_states=Counter(),first_stamp_ns=None,minimum_distance_m=None)for key in expected}
    unmatched=set();seen=set()
    for row in history:
        mapping=row['door_map']
        for obj in mapping['objects']:
            key=(obj['id'],obj['last_stamp_ns'])
            if key in seen:continue
            seen.add(key)
            distance,name=min((math.dist(obj['position'][:2],xy),name)for name,xy in expected.items())
            if distance>maximum_distance:
                unmatched.add(obj['id']);continue
            record=matches[name];record['ids'].add(obj['id'])
            record['minimum_distance_m']=distance if record['minimum_distance_m']is None else min(distance,record['minimum_distance_m'])
            if record['first_stamp_ns']is None:record['first_stamp_ns']=obj['last_stamp_ns']
            age=(mapping['stamp_ns']-obj['last_stamp_ns'])/1e9
            if 0<=age<=2:record['fresh_states'][obj['state']]+=1
    results=[]
    for item in truth:
        record=matches[item['id']]
        results.append(dict(truth_id=item['id'],expected_state=item['state'],observed_ids=sorted(record['ids']),
                            fresh_state_observations=dict(record['fresh_states']),first_stamp_ns=record['first_stamp_ns'],
                            minimum_correspondence_distance_m=record['minimum_distance_m'],
                            passed=len(record['ids'])==1 and record['fresh_states'][item['state']]>=3
                            and not record['fresh_states']['closed' if item['state']=='open' else 'open']))
    all_ids=[identifier for result in results for identifier in result['observed_ids']]
    passed=all(r['passed']for r in results) and len(set(all_ids))==len(truth) and not unmatched
    return dict(passed=passed,doors=results,unmatched_ids=sorted(unmatched),
                alignment_policy='최초 map←odom 고정·Gazebo pose odom 사용, 독립 SLAM 정확도 평가 아님',
                map_from_odom_initial=alignment,maximum_correspondence_distance_m=maximum_distance)


def validate_saved_snapshot(document, map_id, current_objects):
    """저장 대기 뒤 수신 지도와 실제 파일의 ID·위치·상태를 함께 확인한다."""
    version=document.get('door_session_schema_version')
    if type(version)is not int or version not in (1,2) or document.get('memory',{}).get('map_id')!=map_id:
        raise RuntimeError('저장 문 지도의 schema/map_id 불일치')
    saved=document['memory']['objects']
    for records in [saved,current_objects]:
        if len({item['id'] for item in records})!=len(records):raise RuntimeError('저장/현재 문 ID 중복')
    saved_by_id={item['id']:item for item in saved}
    if set(saved_by_id)!={item['id'] for item in current_objects}:raise RuntimeError('저장/현재 문 목록 불일치')
    for current in current_objects:
        stored=saved_by_id[current['id']]
        if stored['position']!=current['position'] or stored['state']!=current['state']:
            raise RuntimeError('저장/현재 문 위치·상태 불일치')
        for key in ('last_confirmed_state','last_confirmed_stamp_ns'):
            if version==2 and (key not in stored or key not in current or stored[key]!=current[key]):
                raise RuntimeError('저장/현재 마지막 확인 이력 불일치')
    return dict(objects=len(saved),ids=sorted(saved_by_id),matched=True)


def validate_historical_states(mapping, evaluation):
    """이전 상태가 현재 판단처럼 채점되지 않고 마지막 확인으로 보존됐는지 검사한다."""
    objects={record['id']:record for record in mapping['objects']}
    checks=[]
    for door in evaluation['doors']:
        if len(door['observed_ids'])!=1:raise RuntimeError('마지막 확인 이력과 대응할 문 ID 부족')
        record=objects[door['observed_ids'][0]]
        if (record.get('last_confirmed_state')!=door['expected_state']
                or type(record.get('last_confirmed_stamp_ns'))is not int
                or record['last_confirmed_stamp_ns']>record['last_stamp_ns']):
            raise RuntimeError('마지막 확인 상태·관측 시각 보존 실패')
        if record.get('observation_status')!='stale' or record['state']!='unknown':
            raise RuntimeError('종료 주차 후 현재 미관측 표시 실패')
        checks.append(dict(id=record['id'],last_confirmed_state=record['last_confirmed_state'],
                           last_confirmed_stamp_ns=record['last_confirmed_stamp_ns'],
                           current_state=record['state'],observation_status=record['observation_status']))
    return dict(passed=True,doors=checks)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--world',type=Path,default=ROOT/'src/mapping_gazebo/worlds/door_patrol_v1.sdf')
    parser.add_argument('--manifest',type=Path,default=ROOT/'src/mapping_gazebo/config/door_patrol_v1.json')
    parser.add_argument('--detector-config',type=Path,default=ROOT/'experiments/door_probe/yolo11-live-01/detector.json')
    parser.add_argument('--recon-root',type=Path,default=Path('/home/blind/cv-middle/CV-VLM/recon-report'))
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--timeout',type=float,default=300.)
    args=parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID'),os.environ.get('GZ_PARTITION'))!=('12','codex'):
        parser.error('ROS_DOMAIN_ID=12 GZ_PARTITION=codex 필요')
    if args.output.exists():parser.error('새 출력 폴더가 필요합니다')
    args.output=args.output.resolve();args.output.mkdir(parents=True)
    truth=json.loads(args.manifest.read_text());plan=stages()
    (args.output/'route-plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
    from rclpy.time import Time
    from nav_msgs.msg import Odometry,OccupancyGrid,Path as PathMsg
    from geometry_msgs.msg import Twist,PoseStamped
    from sensor_msgs.msg import Image,LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer,TransformListener,TransformException
    from cv_bridge import CvBridge
    from PIL import Image as PILImage
    from capture import stop_group,process_identity
    from mapping_semantic.door_node import correction_vector
    from mapping_semantic.core import rotation_matrix
    rclpy.init();node=Node('indoor_door_patrol',parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node);bridge=CvBridge()
    data=dict(odom=None,scan=None,grid=None,rgb=None,doors=None,detector=None,odom_at=0.,scan_at=0.,scan_count=0,map_updates=0)
    observation_history=[]
    map_id='door-patrol-'+args.output.name
    def on_odom(message):data.update(odom=message,odom_at=time.monotonic())
    def on_scan(message):data.update(scan=message,scan_at=time.monotonic(),scan_count=data['scan_count']+1)
    def on_grid(message):data.update(grid=message,map_updates=data['map_updates']+1)
    def on_doors(message):
        value=json.loads(message.data)
        if value['map_id']!=map_id:raise RuntimeError('다른 map_id 혼입')
        if value.get('epoch')!=0:raise RuntimeError('지도/시각 epoch 초기화')
        data['doors']=value
        observation_history.append(dict(wall_ns=time.time_ns(),door_map=value))
    latched=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Odometry,'/odom',on_odom,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',on_scan,qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid,'/map',on_grid,latched)
    node.create_subscription(Image,'/camera/color/image_raw',lambda m:data.update(rgb=m),qos_profile_sensor_data)
    node.create_subscription(String,'/semantic/door_map',on_doors,latched)
    node.create_subscription(String,'/doors/detector_status',lambda m:data.update(detector=json.loads(m.data)),10)
    velocity_pub=node.create_publisher(Twist,'/cmd_vel',10)
    status_pub=node.create_publisher(String,'/demo/patrol_status',10)
    path_pub=node.create_publisher(PathMsg,'/demo/patrol_path',10)
    path=PathMsg();path.header.frame_id='map'
    history=[];children=[];handles=[];report=dict(status='running',base_commit='3911341',map_id=map_id,
        world_path=str(args.world.resolve()),world_sha256=hashlib.sha256(args.world.read_bytes()).hexdigest(),
        manifest_sha256=hashlib.sha256(args.manifest.read_bytes()).hexdigest(),stages=[],recording_requested=args.record,
        limitations=['사전 정의 경로, 자율 프런티어 탐색 아님','Gazebo pose odom, 실물·독립 장소 성능 아님','TTL 초과 문은 unknown 표시'])
    started=time.monotonic();initial_alignment=None;traveled=0.;last_xy=None;minimum_clearance=None
    def start(name,command,environment=None):
        handle=(args.output/(name+'.log')).open('w');handles.append(handle)
        child=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True,env=environment)
        children.append((name,child));report.setdefault('processes',[]).append(dict(name=name,pid=child.pid,identity=process_identity(child.pid),command=command))
        return child
    def pump(seconds=.05):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=min(.02,max(0,end-time.monotonic())))
        if time.monotonic()-started>args.timeout:raise TimeoutError('전체 순찰 시간 초과')
    def publish_command(command):
        msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=command;velocity_pub.publish(msg)
    def pose():
        message=data['odom'];p=message.pose.pose.position
        q=message.pose.pose.orientation
        rotation=rotation_matrix([q.x,q.y,q.z,q.w])
        return [p.x,p.y,math.atan2(rotation[1,0],rotation[0,0])]
    def save_rgb(name):
        if data['rgb']is not None:PILImage.fromarray(bridge.imgmsg_to_cv2(data['rgb'],desired_encoding='rgb8')).save(args.output/name)
    def sample(stage,index):
        nonlocal traveled,last_xy
        now=node.get_clock().now().nanoseconds;p=pose()
        if last_xy is not None:traveled+=math.dist(last_xy,p[:2])
        last_xy=p[:2]
        current=dict(wall_ns=time.time_ns(),elapsed_sec=time.monotonic()-started,stage=stage['name'],odom=p,door_map=data['doors'])
        history.append(current)
        status=String();status.data=json.dumps(dict(stage=stage['name'],label=stage['label'],elapsed_sec=current['elapsed_sec'],robot_odom=p,
            route_index=index,route_count=len(plan),status='running'),ensure_ascii=False);status_pub.publish(status)
        try:
            msg=data['odom'];when=Time.from_msg(msg.header.stamp)
            tf=buffer.lookup_transform('map','base_footprint',when).transform
            position=PoseStamped();position.header.frame_id='map';position.header.stamp=msg.header.stamp
            position.pose.position.x=tf.translation.x;position.pose.position.y=tf.translation.y
            position.pose.orientation=tf.rotation
            if not path.poses or math.dist([path.poses[-1].pose.position.x,path.poses[-1].pose.position.y],[tf.translation.x,tf.translation.y])>.03:
                path.poses.append(position)
            path.header.stamp=msg.header.stamp;path_pub.publish(path)
        except TransformException:report['path_tf_waits']=report.get('path_tf_waits',0)+1
        return now
    try:
        pump(.8)
        if node.count_publishers('/clock'):raise RuntimeError('기존 /clock 발행자 있음')
        spawn=truth['spawn']
        start('simulation',['ros2','launch','mapping_slam','simulation.launch.py','gui:=false','world:='+str(args.world.resolve()),
            f'x:={spawn["x"]}',f'y:={spawn["y"]}',f'yaw:={spawn["yaw"]}'])
        start('doors',['ros2','launch','mapping_semantic','live_doors.launch.py','recon_root:='+str(args.recon_root.resolve()),
            'reobserve_known_doors:=false','map_id:='+map_id,'detector_config:='+str(args.detector_config.resolve()),'output_path:='+str(args.output/'doors.json')])
        deadline=time.monotonic()+80
        while time.monotonic()<deadline:
            pump(.1)
            if all(data.get(key)is not None for key in ['odom','scan','grid','rgb','doors','detector']):
                try:initial_alignment=correction_vector(buffer.lookup_transform('map','odom',Time()).transform)
                except TransformException:continue
                break
        else:raise TimeoutError('SLAM·센서·검출 초기화 시간 초과')
        if math.dist(pose()[:2],[spawn['x'],spawn['y']])>.08:raise RuntimeError('스폰과 초기 Gazebo pose odom 불일치')
        report['initial_odom']=pose();report['initial_door_map']=data['doors']
        if data['doors']['objects']:raise RuntimeError('빈 문 지도에서 시작하지 못함')
        if args.record:
            env=os.environ.copy();env['DISPLAY']=':92'
            start('xephyr',['Xephyr',':92','-screen','1920x840','-ac','-noreset','-nolisten','tcp'])
            pump(1.)
            env['GZ_SIM_RESOURCE_PATH']='/home/blind/worker_following_ws/install/wf_gazebo/share'
            start('gazebo-gui',['gz','sim','-g','--gui-config',str(ROOT/'scripts/demo/door_patrol_gui.config')],env)
            start('rviz',['rviz2','-d',str(ROOT/'scripts/demo/door_patrol.rviz'),'--ros-args','-p','use_sim_time:=true'],env)
            pump(8.)
            start('recorder',['/usr/bin/python3',str(ROOT/'scripts/demo/door_patrol_record.py'),'--output-dir',str(args.output),
                '--expected-map-id',map_id,'--display',':92','--fps','10','--timeout',str(args.timeout),
                '--stop-file',str(args.output/'recording-stop')],env)
            ready=time.monotonic()+30
            while not (args.output/'recording-ready.json').exists() and time.monotonic()<ready:pump(.1)
            if not (args.output/'recording-ready.json').exists():raise RuntimeError('녹화 준비 실패')
        for index,stage in enumerate(plan):
            print(f'{index+1}/{len(plan)} {stage["label"]} pose={pose()}',flush=True)
            begin=node.get_clock().now().nanoseconds;stage_start=time.monotonic()
            result=dict(stage,begin_sim_ns=begin,begin_odom=pose(),begin_objects=data['doors']['objects'])
            report['stages'].append(result)
            while True:
                pump(.05)
                if time.monotonic()-data['odom_at']>2 or time.monotonic()-data['scan_at']>2:raise RuntimeError('odom/scan 수신 정지')
                now=sample(stage,index)
                if data['doors'].get('epoch')!=0:raise RuntimeError('주행 중 지도/시각 epoch 초기화')
                if stage['kind']=='scan':
                    command=(0.,0.,0.);done=(now-begin)/1e9>=stage['duration_sim_sec']
                else:command,done=command_for(stage,pose())
                clearance=directional_clearance(data['scan'],command)
                if clearance is not None:
                    minimum_clearance=clearance if minimum_clearance is None else min(minimum_clearance,clearance)
                    if clearance<.5:raise RuntimeError('주행 방향 LiDAR 장애물 거리 0.5m 미만')
                publish_command((0.,0.,0.)if done else command)
                if done:break
                if time.monotonic()-stage_start>65:raise TimeoutError('개별 경로 단계 시간 초과')
            result.update(end_sim_ns=node.get_clock().now().nanoseconds,end_odom=pose(),end_objects=data['doors']['objects'])
            if stage['kind']=='scan':save_rgb(stage['name']+'-rgb.png')
            pump(.15)
        publish_command((0.,0.,0.));pump(.3)
        report.update(travel_distance_odom_m=traveled,minimum_directional_clearance_m=minimum_clearance,
            scan_count=data['scan_count'],map_updates=data['map_updates'],clock_publishers=node.count_publishers('/clock'),
            map_publishers=[v.node_name for v in node.get_publishers_info_by_topic('/map')],
            final_door_map=data['doors'],final_detector=data['detector'],final_odom=pose())
        client=node.create_client(Trigger,'/semantic/doors/save')
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('저장 서비스 없음')
        future=client.call_async(Trigger.Request());deadline=time.monotonic()+5
        while not future.done() and time.monotonic()<deadline:pump(.05)
        saved=future.result()if future.done()else None
        report['save_response']=dict(success=saved.success,message=saved.message)if saved else None
        if not saved or not saved.success:raise RuntimeError('문 지도 저장 실패')
        # 저장 응답을 기다리며 수신한 관측도 최종 평가에 반드시 포함한다.
        report['final_door_map']=data['doors']
        report['final_detector']=data['detector']
        report['evaluation']=evaluate_observations(observation_history,truth['doors'],initial_alignment)
        report['saved_snapshot_check']=validate_saved_snapshot(json.loads((args.output/'doors.json').read_text()),map_id,data['doors']['objects'])
        report['historical_state_check']=validate_historical_states(data['doors'],report['evaluation'])
        grid=data['grid']
        import numpy as np
        np.save(args.output/'occupancy.npy',np.asarray(grid.data,dtype=np.int8).reshape(grid.info.height,grid.info.width),allow_pickle=False)
        report['map_metadata']=dict(width=grid.info.width,height=grid.info.height,resolution=grid.info.resolution,
            origin=[grid.info.origin.position.x,grid.info.origin.position.y])
        if not report['evaluation']['passed']:raise RuntimeError('문별 사후 평가 실패: 누락·중복·상태 근거 부족')
        if traveled<8.8:raise RuntimeError('실제 주행 거리 부족')
        if report['clock_publishers']!=1 or report['map_publishers']!=['slam_toolbox']:raise RuntimeError('지도·clock 발행자 수 오류')
        report['status']='complete'
    except BaseException as exc:
        report.update(status='failed',error=f'{type(exc).__name__}: {exc}',last_odom=pose()if data['odom']else None,
            final_door_map=data['doors'],final_detector=data['detector'])
        if initial_alignment is not None:report['evaluation']=evaluate_observations(observation_history,truth['doors'],initial_alignment)
        save_rgb('failure-rgb.png')
        raise
    finally:
        previous_sigint=signal.signal(signal.SIGINT,signal.SIG_IGN)
        publish_command((0.,0.,0.))
        (args.output/'recording-stop').touch()
        for name,child in children:
            if name=='recorder':
                try:child.wait(timeout=15)
                except subprocess.TimeoutExpired:report.setdefault('cleanup_errors',[]).append('녹화 종료 지연')
                report['recorder_returncode']=child.poll()
        for name,child in reversed(children):
            try:stop_group(child)
            except Exception as exc:report.setdefault('cleanup_errors',[]).append(name+': '+str(exc))
        for handle in handles:handle.close()
        report['process_returncodes']={name:child.poll()for name,child in children}
        if data['detector']:
            pgid=data['detector'].get('worker_pgid')
            if isinstance(pgid,int):
                try:os.killpg(pgid,0)
                except ProcessLookupError:report['worker_group_gone']=True
                else:report.setdefault('cleanup_errors',[]).append('검출 worker 그룹 남음')
        if report.get('cleanup_errors') or (args.record and report.get('recorder_returncode')!=0):report['status']='failed'
        report['elapsed_wall_sec']=time.monotonic()-started
        report['harness_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
        report['source_sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in [
            'src/mapping_semantic/mapping_semantic/door_map.py',
            'src/mapping_semantic/mapping_semantic/door_node.py',
            'scripts/demo/door_patrol_record.py']}
        (args.output/'check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
        (args.output/'history.json').write_text(json.dumps(history,ensure_ascii=False,indent=2)+'\n')
        (args.output/'observation-history.json').write_text(json.dumps(observation_history,ensure_ascii=False,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
        signal.signal(signal.SIGINT,previous_sigint)
    if report['status']!='complete':raise RuntimeError('순찰/녹화 정리 검증 실패')


if __name__=='__main__':main()
