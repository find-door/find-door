"""실제 ROS Node에서 무검출 RGB-D 재관측·원본 검출 필요·캐시 초기화를 검사한다."""
import os
import json
import pytest


@pytest.mark.parametrize('enabled,fail_quad',[(True,False),(False,True),(True,True)])
def test_fresh_rgbd_without_detection(monkeypatch,tmp_path,enabled,fail_quad):
    if (os.environ.get('ROS_DOMAIN_ID'),os.environ.get('GZ_PARTITION'))!=('12','codex'):
        pytest.skip('명시적인 Codex ROS 격리 환경 필요')
    rclpy=pytest.importorskip('rclpy')
    from mapping_semantic import door_node
    main=door_node.main
    if fail_quad:
        def unavailable(*args):raise ValueError('일부 코너를 투영할 수 없음')
        monkeypatch.setattr(door_node,'doorway_quad',unavailable)
    from sensor_msgs.msg import CameraInfo
    from vision_msgs.msg import Detection2DArray,Detection2D,ObjectHypothesisWithPose
    from geometry_msgs.msg import TransformStamped
    from types import SimpleNamespace as N
    from test_doorway import scene

    def exercise(node):
        for child in ['odom','camera_color_optical_frame']:
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id=child
            if child=='odom':tf.transform.rotation.w=1.
            else:
                tf.transform.translation.z=.6
                tf.transform.rotation.x=-.5;tf.transform.rotation.y=.5
                tf.transform.rotation.z=-.5;tf.transform.rotation.w=.5
            node.buffer.set_transform_static(tf,'test')

        def send(detected,opened=False,missing_jamb=False):
            color,depth,k=scene(leaf=not opened)
            if opened:depth[28:150,68:152]=3.
            if missing_jamb:depth[:,152:160]=float('nan')
            rgb=node.bridge.cv2_to_imgmsg(color,encoding='rgb8')
            metric=node.bridge.cv2_to_imgmsg(depth,encoding='32FC1')
            info=CameraInfo(width=240,height=180,k=k.flatten().tolist())
            detections=Detection2DArray()
            stamp=node.get_clock().now().to_msg()
            for msg in [rgb,metric,info,detections]:
                msg.header.stamp=stamp;msg.header.frame_id='camera_color_optical_frame'
            if detected:
                box=Detection2D();box.header=detections.header
                box.bbox.center.position.x=102.5;box.bbox.center.position.y=90.
                box.bbox.size_x=55.;box.bbox.size_y=100.
                hypothesis=ObjectHypothesisWithPose();hypothesis.hypothesis.class_id='door';hypothesis.hypothesis.score=.5
                box.results=[hypothesis];detections.detections=[box]
            node.images(rgb,metric,info,detections);node.tick()

        send(False,True)
        assert not node.cached_doors and node.last_source==0
        for _ in range(3):send(True)
        initial=node.visible(node.get_clock().now().nanoseconds)[0]
        assert initial['state']=='closed' and len(node.cached_doors)==int(enabled and not fail_quad)
        if enabled and fail_quad:assert node.counters['cache_geometry_unavailable']>0
        for _ in range(3):send(False,True)
        current=node.visible(node.get_clock().now().nanoseconds)[0]
        assert current['id']==initial['id'] and current['position']==initial['position']
        if enabled and not fail_quad:
            assert current['state']=='open' and current['last_stamp_ns']>initial['last_stamp_ns']
            assert node.counters['reobserved']==3
            previous=node.last_source
            send(False,True,missing_jamb=True)
            assert node.last_source==previous
        else:
            assert current['state']=='closed' and current['last_stamp_ns']==initial['last_stamp_ns']
            assert not node.counters['reobserved']
        expired_at=node.last_source+3_000_000_000
        stale=node.visible(expired_at)[0]
        assert stale['state']=='unknown' and stale['last_confirmed_state']==current['state']
        assert stale['observation_status']=='stale'
        assert stale['last_confirmed_stamp_ns']==current['last_confirmed_stamp_ns']
        markers=[]
        monkeypatch.setattr(node.marker_pub,'publish',markers.append)
        node.publish(expired_at)
        labels=[m for m in markers[-1].markers if m.ns=='door_text']
        spheres=[m for m in markers[-1].markers if m.ns=='doors']
        assert 'last:'+current['state'] in labels[0].text and 'stale' in labels[0].text
        assert spheres[0].color.a<1
        monkeypatch.setattr(node,'clock_now',lambda:expired_at)
        response=node.save(None,N())
        assert response.success
        document=json.loads(path.read_text())
        assert document['door_session_schema_version']==2 and document['memory']['door_map_schema_version']==2
        saved=document['memory']['objects'][0]
        assert saved['state']=='unknown' and saved['last_confirmed_state']==current['state']
        assert 'observation_status' not in saved
        response=node.load(None,N())
        assert response.success and not node.cached_doors
        loaded=node.visible(expired_at)[0]
        assert loaded['observation_status']=='stale' and loaded['last_confirmed_stamp_ns']==saved['last_confirmed_stamp_ns']
        # 이전 schema1의 unknown에는 유실된 확인 이력을 만들어 넣지 않는다.
        document['door_session_schema_version']=document['memory']['door_map_schema_version']=1
        for item in document['memory']['objects']:
            del item['last_confirmed_state'];del item['last_confirmed_stamp_ns']
        path.write_text(json.dumps(document))
        assert node.load(None,N()).success
        assert node.visible(expired_at)[0]['last_confirmed_state']=='unknown'
        node.cached_doors['test']={'quad':[]}
        node.reset(node.get_clock().now().nanoseconds,'correction_resets')
        assert not node.cached_doors

    monkeypatch.setattr(rclpy,'spin',exercise)
    path=tmp_path/'doors.json'
    main(['--ros-args','-p','map_id:=reobservation-test','-p',f'output_path:={path}',
          '-p',f'input_path:={path}','-p',f'reobserve_known_doors:={str(enabled).lower()}'])
