"""미확인 설정과 stale/잘못된 센서 입력을 실행 허용하지 않는지 확인한다."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
import yaml

from mapping_robot.profile import validate_profile, slam_parameters
from mapping_robot.preflight import fixture_guard, message_issues, stream_summary


def profile():
    return {'schema_version': 1, 'robot_id': 'test_robot', 'hardware_confirmed': True,
            'frames': dict(map='map', odom='odom', base='base', laser='laser', optical='optical'),
            'topics': dict(scan='/scan', odom='/odom', rgb='/rgb', depth='/depth',
                           camera_info='/camera_info', detections='/detections'),
            'thresholds': dict(min_scan_hz=5.0, min_odom_hz=10.0, max_age_sec=0.5),
            'slam': dict(resolution=0.05, min_laser_range=0.2, max_laser_range=10., do_loop_closing=False),
            'semantic': dict(enabled=False, map_id=None, model_path=None, model_sha256=None,
                             output_path=None, sync_policy='exact', max_sync_skew_sec=.03,
                             aligned_depth_confirmed=False, calibration_id=None,
                             registration_note=None, static_classes=['chair', 'cup', 'bottle'])}


def test_complete_profile_and_hardware_time_mapping():
    p = profile()
    assert validate_profile(p) == []
    parameters = slam_parameters(p)
    assert parameters['use_sim_time'] is False
    assert parameters['base_frame'] == 'base'
    assert parameters['scan_topic'] == '/scan'
    assert parameters['do_loop_closing'] is False


def test_template_fails_even_offline_override():
    p = yaml.safe_load((Path(__file__).parents[1] / 'config/robot.template.yaml').read_text())
    assert validate_profile(p)
    assert validate_profile(p, allow_unconfirmed=True)
    p = profile()
    p['hardware_confirmed'] = False
    assert validate_profile(p)
    assert not validate_profile(p, allow_unconfirmed=True)


@pytest.mark.parametrize('group,key,value', [
    ('frames','base','/base'), ('frames','base','odom'),
    ('topics','scan','/robot/cmd_vel'), ('topics','scan','/cmd_drive'),
    ('topics','scan','/clock'), ('topics','scan','/odom'),
    ('thresholds','min_scan_hz',float('nan')), ('thresholds','max_age_sec',False),
    ('slam','min_laser_range',11), ('slam','do_loop_closing',True),
    ('semantic','max_sync_skew_sec',.2), ('semantic','static_classes',['person']),
    ('semantic','aligned_depth_confirmed',True),
])
def test_invalid_contract_rejected(group,key,value):
    p=profile(); p[group][key]=value
    assert validate_profile(p)


def test_semantic_requires_alignment_model_and_hash(tmp_path):
    import hashlib
    p=profile(); s=p['semantic']; s['enabled']=True
    assert validate_profile(p)
    model=tmp_path/'model.onnx'; model.write_bytes(b'opaque test bytes, not executable model')
    s.update(map_id='test_map', model_path=str(model), model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
             output_path=str(tmp_path/'new.json'), aligned_depth_confirmed=True,
             calibration_id='bench_001', registration_note='실측 확인 기록 위치')
    assert not validate_profile(p)
    s['model_sha256']='0'*64
    assert validate_profile(p)


def scan():
    return NS(header=NS(frame_id='laser',stamp=NS(sec=1,nanosec=0)),
              angle_min=-1.,angle_max=1.,angle_increment=1.,range_min=.2,range_max=10.,
              time_increment=0.,scan_time=.2,ranges=[1.,2.,float('inf')])


def test_scan_distance_frame_geometry():
    msg=scan(); assert not message_issues('scan',msg,profile())
    msg.ranges=[float('inf')]*3; assert message_issues('scan',msg,profile())
    msg=scan();msg.header.frame_id='wrong'; assert message_issues('scan',msg,profile())
    msg=scan();msg.ranges=[1.,-1.,2.]; assert message_issues('scan',msg,profile())


def test_odom_child_and_nonunit_quaternion_rejected():
    msg=NS(header=NS(frame_id='odom',stamp=NS(sec=1,nanosec=0)),child_frame_id='base',
           pose=NS(pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
    assert not message_issues('odom',msg,profile())
    msg.pose.pose.orientation.w=0.
    assert message_issues('odom',msg,profile())
    msg.pose.pose.orientation.w=1.;msg.child_frame_id='other'
    assert message_issues('odom',msg,profile())


def test_frozen_stamp_and_stale_burst_fail():
    samples=[(1_000_000_000,0.),(1_100_000_000,.1),(1_200_000_000,.2)]
    assert not stream_summary(samples,1_200_000_000,.2,5,.5)['issues']
    assert stream_summary(samples,3_000_000_000,2.,5,.5)['issues']
    assert stream_summary([(1_000_000_000,t) for t in (0.,.1,.2)],1_000_000_000,.2,5,.5)['issues']
    assert stream_summary(samples,100,.2,5,.5)['issues']


def test_fixture_cannot_run_in_hardware_domain():
    with pytest.raises(ValueError): fixture_guard(True,{'ROS_DOMAIN_ID':'0'})
    fixture_guard(True, {'ROS_DOMAIN_ID':'12','GZ_PARTITION':'codex'})


def test_slam_only_allows_unknown_camera_but_explicit_camera_requires_it():
    p=profile()
    p['frames']['optical']=None
    for key in ('rgb','depth','camera_info','detections'):
        p['topics'][key]=None
    assert not validate_profile(p)
    assert validate_profile(p, require_camera=True)
    p['semantic']['enabled']=True
    assert validate_profile(p)


def test_float32_scan_limits_accept_rounding_but_reject_real_difference():
    import struct
    f32=lambda value: struct.unpack('f',struct.pack('f',value))[0]
    p=profile();msg=scan();msg.range_min=f32(.2)
    p['slam']['max_laser_range']=9.9;msg.range_max=f32(9.9)
    assert not message_issues('scan',msg,p)
    p['slam']['min_laser_range']=.19
    assert message_issues('scan',msg,p)
    p['slam']['min_laser_range']=.2;p['slam']['max_laser_range']=10.
    assert message_issues('scan',msg,p)


def test_image_storage_and_decoded_depth_reject_empty_invalid_inputs():
    import numpy as np
    from mapping_robot.preflight import validate_image_storage, validate_camera_arrays
    msg=NS(width=5,height=4,encoding='rgb8',step=15,data=bytes(60),is_bigendian=0)
    validate_image_storage(msg)
    msg.data=b''
    with pytest.raises(ValueError):validate_image_storage(msg)
    msg.data=bytes(60);msg.step=0
    with pytest.raises(ValueError):validate_image_storage(msg)
    rgb=np.zeros((4,5,3),dtype=np.uint8)
    assert validate_camera_arrays(rgb,np.full((4,5),1000,dtype=np.uint16),'16UC1')==20
    assert validate_camera_arrays(rgb,np.ones((4,5),dtype=np.float32),'32FC1')==20
    for depth in (np.zeros((4,5),dtype=np.float32),np.full((4,5),np.nan,dtype=np.float32)):
        with pytest.raises(ValueError):validate_camera_arrays(rgb,depth,'32FC1')
    with pytest.raises(ValueError):validate_camera_arrays(rgb,np.ones((4,5),dtype=np.float64),'32FC1')


def test_pose_tf_consistency_sign_invariance_and_real_mismatch():
    import math
    from mapping_robot.preflight import pose_tf_error
    pose=NS(position=NS(x=0.,y=0.,z=0.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
    tf=NS(translation=NS(x=0.,y=0.,z=0.),rotation=NS(x=0.,y=0.,z=0.,w=-1.))
    assert pose_tf_error(pose,tf)['valid']
    tf.translation.x=.021
    assert not pose_tf_error(pose,tf)['valid']
    tf.translation.x=0.;tf.rotation.z=math.sin(.03/2);tf.rotation.w=math.cos(.03/2)
    assert not pose_tf_error(pose,tf)['valid']
    tf.rotation.z=float('nan')
    with pytest.raises(ValueError):pose_tf_error(pose,tf)


@pytest.mark.parametrize('group', ['frames', 'topics', 'thresholds', 'slam', 'semantic'])
@pytest.mark.parametrize('mutation', ['missing', 'unknown', 'non_string'])
def test_nested_structure_errors_return_issues_without_traceback(group, mutation):
    p = profile()
    if mutation == 'missing':
        p[group].pop(next(iter(p[group])))
    else:
        p[group][1 if mutation == 'non_string' else 'unknown_key'] = None
    issues = validate_profile(p)
    assert issues and any('key 불일치' in issue for issue in issues)


def test_top_level_non_string_key_and_cli_malformed_nested_yaml(tmp_path, capsys):
    from mapping_robot.profile import main
    p = profile()
    p[1] = None
    assert validate_profile(p)
    p = profile()
    p['frames'][1] = None
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(p))
    assert main(['--profile', str(path)]) == 1
    assert '프로파일 오류' in capsys.readouterr().out
