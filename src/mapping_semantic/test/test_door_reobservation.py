"""출입구 재투영·새 기하 대조와 기존 ID 전용 갱신의 거부 경계."""
import copy
import math
import numpy as np
import pytest
from mapping_semantic.door_reobservation import doorway_quad, project_quad, compatible_quad
from mapping_semantic.door_map import DoorMap

K=np.array([[200.,0,120],[0,200.,90],[0,0,1.]])
BOX=[60.,20.,160.,150.]


def quad():return doorway_quad(K,BOX,[0,0,-1],2.,[0,0,0],[0,0,0,1])


def test_roundtrip_and_translation():
    points=quad()
    assert project_quad(points,K,[0,0,0],[0,0,0,1],240,180)==pytest.approx(BOX)
    assert project_quad(points,K,[.1,0,0],[0,0,0,1],240,180)==pytest.approx([50,20,150,150])


@pytest.mark.parametrize('camera',[[0,0,3],[10,0,0],[0,0,-20]])
def test_behind_outside_or_far(camera):
    assert project_quad(quad(),K,camera,[0,0,0,1],240,180) is None


def test_rotated_camera_projection():
    angle=.1;q=[0,0,math.sin(angle/2),math.cos(angle/2)]
    result=project_quad(quad(),K,[0,0,0],q,240,180)
    assert result is not None and result!=pytest.approx(BOX)


def test_same_geometry_accepts():
    assert compatible_quad(quad(),quad(),[0,0,2],[0,0,2])


@pytest.mark.parametrize('kind',['anchor','plane','width','normal'])
def test_changed_geometry_rejects(kind):
    old=np.array(quad());new=old.copy();anchor=[0,0,2];position=list(anchor)
    if kind=='anchor':position[0]=.11
    elif kind=='plane':new[:,2]+=.09
    elif kind=='width':new[:,0]*=1.3
    else:
        a=.3;new[:,2]+=new[:,0]*math.tan(a)
    assert not compatible_quad(old,new,anchor,position)


@pytest.mark.parametrize('kind',['nan','shape','degenerate','nonplanar'])
def test_invalid_quad(kind):
    points=np.array(quad())
    if kind=='nan':points[0,0]=np.nan
    elif kind=='shape':points=points[:3]
    elif kind=='degenerate':points[:]=0
    else:points[2,2]+=.2
    with pytest.raises(ValueError):project_quad(points,K,[0,0,0],[0,0,0,1],240,180)


def test_reobservation_cannot_create_or_switch_id():
    memory=DoorMap('test')
    with pytest.raises(ValueError):memory.observe([0,0,0],'open',1,expected_id='door_000001')
    assert memory.next_id==1 and memory.snapshot(1)==[]
    record=memory.observe([0,0,0],'closed',2)
    before=copy.deepcopy(memory.dump(2))
    for identifier,position in [('door_000002',[0,0,0]),(record['id'],[1,0,0])]:
        with pytest.raises(ValueError):memory.observe(position,'open',3,expected_id=identifier)
        assert memory.dump(2)==before
    assert memory.observe([.01,0,0],'open',3,expected_id=record['id'])['id']==record['id']


def test_upper_corner_can_fail_while_threshold_remains_valid():
    from mapping_semantic.door_position import threshold_point
    k=[[100,0,50],[0,100,50],[0,0,1]]
    normal=[math.sqrt(.66),.3,.5]
    point=threshold_point(k,[0,0,100,100],normal,-1.2)
    assert .2<point[2]<10
    with pytest.raises(ValueError):
        doorway_quad(k,[0,0,100,100],normal,-1.2,[0,0,0],[0,0,0,1])
