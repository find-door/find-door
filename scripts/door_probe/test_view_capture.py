"""보조 시점이 내부 파라미터를 유지하고 전체 문틀 구도를 갖추는지 확인한다."""
from types import SimpleNamespace as N
import copy
import xml.etree.ElementTree as ET
import pytest
from view_capture import with_camera, validate_views, AUX_POSITION, AUX_OPTICAL_QUATERNION
from scene_variant import variant


def messages(frame):
    stamp=N(sec=4,nanosec=0);header=N(frame_id=frame,stamp=stamp)
    rgb=N(width=640,height=480,header=header)
    info=N(width=640,height=480,header=header,k=[500.,0.,320.,0.,500.,240.,0.,0.,1.],d=[0.]*5)
    return rgb,info


def case():
    return (messages('camera_color_optical_frame'),messages('door_aux_optical_frame'),
            dict(translation=[-3.,0.,.25],quaternion=AUX_OPTICAL_QUATERNION))


def test_equal_intrinsics_and_different_framing():
    bounds=validate_views(*case())
    assert bounds[0][1]<0
    assert 0<bounds[1][1]<bounds[1][3]<480


@pytest.mark.parametrize('kind',['intrinsics','stamp','position','heading','frame','distortion','nan','nan_position'])
def test_rejects_invalid_pair(kind):
    low,high,camera=case()
    if kind=='intrinsics':high[1].k[0]=450.
    elif kind=='stamp':high[0].header.stamp.sec=5
    elif kind=='position':camera['translation'][0]=-2.
    elif kind=='heading':camera['quaternion']=[0.,0.,0.,1.]
    elif kind=='frame':high[0].header.frame_id='wrong'
    elif kind=='distortion':high[1].d[0]=.1
    elif kind=='nan_position':camera['translation'][0]=float('nan')
    else:high[1].k[0]=float('nan')
    with pytest.raises(ValueError):validate_views(low,high,camera)


@pytest.mark.parametrize('state',['closed','half','open'])
def test_aux_camera_addition_preserves_scene(state):
    root=ET.fromstring(with_camera(state));original=ET.fromstring(variant(state))
    world=root.find('world');camera=world.find("model[@name='diagnostic_camera']")
    assert list(map(float,camera.findtext('pose').split()))[:3]==AUX_POSITION
    assert camera.find('link/sensor').get('type')=='camera'
    world.remove(camera)
    # 인덴트 공백을 제외하고 기존 기하·플러그인·재질은 동일하다.
    def shape(e):return (e.tag,e.attrib,(e.text or '').strip(),[shape(c)for c in e])
    assert shape(world)==shape(original.find('world'))


@pytest.fixture
def saved_pair(tmp_path):
    from PIL import Image
    from capture import digest
    low,high,camera=case()
    bounds=validate_views(low,high,camera)
    frames=[]
    for name,messages,bound in zip(['low','high'],[low,high],bounds):
        rgb,info=messages
        path=tmp_path/f'{name}.png';Image.new('RGB',(640,480)).save(path)
        frames.append(dict(view=name,rgb=str(path),sha256=digest(path),stamp_ns=4_000_000_000,
                           frame_id=rgb.header.frame_id,width=640,height=480,k=list(info.k),d=list(info.d),frame_bounds_unclipped=bound))
    return dict(status='complete',frames=frames,robot_camera=camera,pair_stamp_delta_ns=0)


def test_saved_pair_revalidation(saved_pair):
    from view_detect import verify_capture
    assert verify_capture(saved_pair)


@pytest.mark.parametrize('kind',['hash','size','k','bounds','delta'])
def test_rejects_tampered_saved_pair(saved_pair,kind):
    from view_detect import verify_capture
    if kind=='hash':saved_pair['frames'][0]['sha256']='0'*64
    elif kind=='size':saved_pair['frames'][0]['width']=320
    elif kind=='k':saved_pair['frames'][1]['k'][0]=450.
    elif kind=='bounds':saved_pair['frames'][1]['frame_bounds_unclipped'][0]=1.
    else:saved_pair['pair_stamp_delta_ns']=10
    with pytest.raises(ValueError):verify_capture(saved_pair)
