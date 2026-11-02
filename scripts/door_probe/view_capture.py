"""같은 Gazebo 장면을 3m의 낮은 로봇/높은 보조 RGB 시점에서 한 쌍 촬영한다."""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image as PILImage

from capture import digest, optical_heading_error, rotation, spawn_pose, stamp_ns, stop_group, tf_record, write_json
from generate_worlds import element, model
from scene_variant import variant, STATES

AUX_POSITION = [-3., 0., 1.16]
AUX_OPTICAL_QUATERNION = [-.5, .5, -.5, .5]
# 단위는 m. 금속문 변형의 문틀 외곽 기하이며 가림 정답은 아니다.
FRAME_CORNERS = np.array([[-.105, y, z] for y in [-.696, .696] for z in [0., 2.31]])


def with_camera(state):
    root = ET.fromstring(variant(state))
    world = root.find('world')
    link = model(world, 'diagnostic_camera', (*AUX_POSITION, 0, 0, 0))
    sensor = element(link, 'sensor', name='diagnostic_rgb', type='camera')
    element(sensor, 'always_on', 'true')
    element(sensor, 'update_rate', '15')
    element(sensor, 'topic', '/door_aux/image')
    element(sensor, 'gz_frame_id', 'door_aux_optical_frame')
    camera = element(sensor, 'camera')
    element(camera, 'camera_info_topic', '/door_aux/camera_info')
    element(camera, 'horizontal_fov', '1.1397')
    image = element(camera, 'image')
    for key, value in [('width', 640), ('height', 480), ('format', 'R8G8B8')]:
        element(image, key, value)
    clip = element(camera, 'clip')
    element(clip, 'near', '.2')
    element(clip, 'far', '10')
    ET.indent(root, space='  ')
    return '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def frame_bounds(translation, quaternion, k):
    points = (FRAME_CORNERS-np.array(translation)) @ rotation(quaternion)
    if np.any(points[:, 2] <= 0):
        raise ValueError('문틀이 카메라 뒤에 있습니다')
    pixels = points @ np.asarray(k).reshape(3, 3).T
    pixels = pixels[:, :2]/pixels[:, 2:]
    return [float(pixels[:, 0].min()), float(pixels[:, 1].min()),
            float(pixels[:, 0].max()), float(pixels[:, 1].max())]


def validate_views(low, high, camera):
    rgb, info = low
    aux, aux_info = high
    if abs(stamp_ns(rgb)-stamp_ns(aux)) > 50_000_000:
        raise ValueError('두 시점의 RGB 시각 차이 초과')
    if stamp_ns(rgb) != stamp_ns(info) or stamp_ns(aux) != stamp_ns(aux_info):
        raise ValueError('각 시점의 RGB/CameraInfo 시각 불일치')
    if any((m.width, m.height) != (640, 480) for m in [rgb, info, aux, aux_info]):
        raise ValueError('해상도 불일치')
    for image, calibration, expected_frame in [(rgb, info, 'camera_color_optical_frame'),
                                                (aux, aux_info, 'door_aux_optical_frame')]:
        if image.header.frame_id != expected_frame or calibration.header.frame_id != expected_frame:
            raise ValueError('카메라 프레임 불일치')
        k = np.asarray(calibration.k).reshape(3, 3)
        if not (np.isfinite(k).all() and k[0, 0]>0 and k[1, 1]>0 and np.allclose(k[2], [0, 0, 1])):
            raise ValueError('CameraInfo K 오류')
        if not np.isfinite(calibration.d).all() or not np.allclose(calibration.d, 0):
            raise ValueError('왜곡 카메라는 지원하지 않습니다')
    if not np.allclose(info.k, aux_info.k, rtol=0, atol=.01):
        raise ValueError('두 시점의 내부 파라미터 불일치')
    if len(camera['translation']) != 3 or not np.isfinite(camera['translation']).all():
        raise ValueError('로봇 카메라 위치 수치 오류')
    if not np.allclose(rotation(camera['quaternion']), rotation(AUX_OPTICAL_QUATERNION), atol=.02, rtol=0):
        raise ValueError('두 시점 광학 축 방향 불일치')
    if np.linalg.norm(np.array(camera['translation'][:2])-AUX_POSITION[:2]) > .02:
        raise ValueError('두 시점 수평 위치 불일치')
    if optical_heading_error(camera['quaternion'], 0) > 1:
        raise ValueError('로봇 카메라 광축 방향 불일치')
    bounds = [frame_bounds(camera['translation'], camera['quaternion'], info.k),
              frame_bounds(AUX_POSITION, AUX_OPTICAL_QUATERNION, aux_info.k)]
    inside = [0<=b[0]<b[2]<=640 and 0<=b[1]<b[3]<=480 for b in bounds]
    if inside != [False, True]:
        raise ValueError('낮은 시점 잘림·높은 시점 전체 문틀 구도의 조건 불일치')
    return bounds


def main():
    import message_filters
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import Image, CameraInfo
    from tf2_ros import Buffer, TransformListener, TransformException
    from cv_bridge import CvBridge

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--state', choices=STATES, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if (os.environ.get('ROS_DOMAIN_ID'),os.environ.get('GZ_PARTITION')) != ('12','codex'):
        parser.error('ROS_DOMAIN_ID=12 GZ_PARTITION=codex 필요')
    args.output=args.output.resolve()
    if args.output.exists():
        parser.error('새 출력 경로가 필요합니다')
    args.output.mkdir(parents=True)
    world=args.output/'world.sdf';world.write_text(with_camera(args.state))
    rclpy.init()
    node=Node('door_view_capture', parameter_overrides=[Parameter('use_sim_time',value=True)])
    buffer=Buffer();listener=TransformListener(buffer,node);bridge=CvBridge()
    queues=[deque(maxlen=20),deque(maxlen=20)];syncs=[];subscribers=[]
    for index,topics in enumerate([('/camera/color/image_raw','/camera/camera_info'),('/door_aux/image','/door_aux/camera_info')]):
        subs=[message_filters.Subscriber(node,typ,topic,qos_profile=qos_profile_sensor_data) for typ,topic in zip([Image,CameraInfo],topics)]
        sync=message_filters.TimeSynchronizer(subs,20)
        sync.registerCallback(lambda *m, i=index:queues[i].append(m))
        subscribers.extend(subs);syncs.append(sync)
    children=[];logs=[]
    record=dict(state=args.state,status='running',world_path=str(world),world_sha256=digest(world),
                source_sha256=digest(__file__),variant_source_sha256=digest(Path(__file__).with_name('scene_variant.py')),
                distance_m=3,aux_position=AUX_POSITION,aux_optical_quaternion=AUX_OPTICAL_QUATERNION,
                notes=['보조 RGB 검출 진단이며 깊이 상태·지도·실물 평가가 아님'])

    def pump():
        rclpy.spin_once(node,timeout_sec=.05)
        if any(child.poll() is not None for child in children):
            raise RuntimeError('생성 프로세스 조기 종료')

    def start(label,command):
        log=(args.output/f'{label}.log').open('w');logs.append(log)
        env=os.environ.copy();env['ROS_LOG_DIR']=str(args.output/'roslogs')
        child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
        children.append(child);record[label]=dict(command=command,pgid=child.pid)

    try:
        until=time.monotonic()+2
        while time.monotonic()<until:pump()
        if node.count_publishers('/clock'):raise RuntimeError('기존 /clock 발행자 존재')
        x,y,yaw=spawn_pose(3,0)
        start('simulation',['ros2','launch','mapping_gazebo','indoor.launch.py','gui:=false',
                            f'world:={world}',f'x:={x}',f'y:={y}',f'yaw:={yaw}'])
        start('aux_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
                            '/door_aux/image@sensor_msgs/msg/Image[gz.msgs.Image',
                            '/door_aux/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'])
        deadline=time.monotonic()+45
        while time.monotonic()<deadline:
            pump()
            if not queues[0] or not queues[1]:continue
            pairs=[(low,high) for low in queues[0] for high in queues[1]
                   if min(stamp_ns(low[0]),stamp_ns(high[0]))>3_000_000_000
                   and abs(stamp_ns(low[0])-stamp_ns(high[0]))<=50_000_000]
            if not pairs:continue
            low,high=min(pairs,key=lambda pair:abs(stamp_ns(pair[0][0])-stamp_ns(pair[1][0])))
            try:camera=tf_record(buffer.lookup_transform('odom','camera_color_optical_frame',Time.from_msg(low[0].header.stamp)).transform)
            except TransformException:continue
            bounds=validate_views(low,high,camera)
            if node.count_publishers('/clock')!=1:raise RuntimeError('/clock 발행자 수 오류')
            record['frames']=[]
            for name,messages,bound in zip(['low','high'],[low,high],bounds):
                rgb,info=messages;path=args.output/f'{name}.png'
                PILImage.fromarray(bridge.imgmsg_to_cv2(rgb,desired_encoding='rgb8')).save(path)
                record['frames'].append(dict(view=name,rgb=str(path),sha256=digest(path),stamp_ns=stamp_ns(rgb),
                                             frame_id=rgb.header.frame_id,k=list(info.k),d=list(info.d),
                                             width=rgb.width,height=rgb.height,frame_bounds_unclipped=bound))
            record.update(status='complete',robot_camera=camera,pair_stamp_delta_ns=abs(stamp_ns(low[0])-stamp_ns(high[0])),clock_publishers=1)
            break
        else:raise TimeoutError('두 시점 수집 시간 초과')
    except BaseException as exc:
        record.update(status='failed',error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        errors=[]
        for child in reversed(children):
            try:stop_group(child)
            except Exception as exc:errors.append(str(exc))
        for log in logs:log.close()
        if errors:record.update(status='failed',cleanup_errors=errors)
        write_json(args.output/'capture.json',record)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
