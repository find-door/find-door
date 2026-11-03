"""네 문 실제 기하·복도 통로·정답 분리·재생성 계약을 확인한다."""
import json
import math
import xml.etree.ElementTree as ET

import pytest

from patrol_world import (DOOR_HEIGHT, DOOR_WIDTH, MANIFEST_PATH, WORLD_PATH,
                          main, make_patrol_world, manifest)


def parsed():
    return ET.fromstring(make_patrol_world()).find('world')


def values(element):
    return list(map(float, element.text.split()))


def inside(model, collision, point):
    """yaw만 있는 자체 box 충돌체 내부인지 world 좌표에서 계산한다."""
    p = values(model.find('pose'))
    shape = values(collision.find('pose'))
    delta = [point[i] - p[i] for i in range(3)]
    c, s = math.cos(p[5]), math.sin(p[5])
    local = [c*delta[0]+s*delta[1], -s*delta[0]+c*delta[1], delta[2]]
    sizes = values(collision.find('geometry/box/size'))
    return all(abs(local[i]-shape[i]) < sizes[i]/2-1e-8 for i in range(3))


def collisions(world, names=None):
    return [(model, collision) for model in world.findall('model')
            if names is None or model.get('name') in names
            for collision in model.findall('link/collision')]


def test_four_rooms_with_clear_ground_truth_boundary():
    world, document = parsed(), manifest()
    assert world.get('name') == document['world_name'] == 'door_patrol_v1'
    assert document['schema_version'] == 1 and document['room_count'] == 4
    assert not document['roof'] and not document['independent_test']
    assert '인식 입력' in document['ground_truth_policy']
    assert document['spawn'] == dict(x=0., y=-4.8, yaw=math.pi/2)
    assert document['bounds']['corridor'] == dict(x=[-2., 2.], y=[-6., 6.])
    assert {tuple(d['center_world_xyz'][:2]) for d in document['doors']} == {
        (2., -3.), (2., 3.), (-2., -3.), (-2., 3.)}
    assert sorted(d['angle_deg'] for d in document['doors']) == [0, 0, 45, 90]
    assert [d['state'] for d in document['doors']] == ['closed', 'open', 'open', 'closed']


def test_all_shapes_have_identical_visible_and_collision_geometry():
    world = parsed()
    names = [model.get('name') for model in world.findall('model')]
    assert len(names) == len(set(names))
    assert len([name for name in names if name.endswith('_frame')]) == 4
    assert len([name for name in names if name.endswith('_leaf')]) == 4
    for model in world.findall('model'):
        assert model.findtext('static') == 'true'
        for link in model.findall('link'):
            collision_map = {item.get('name').removesuffix('_collision'): item
                             for item in link.findall('collision')}
            visual_map = {item.get('name').removesuffix('_visual'): item
                          for item in link.findall('visual')}
            assert collision_map.keys() == visual_map.keys()
            for name, collision in collision_map.items():
                visual = visual_map[name]
                assert collision.findtext('pose') == visual.findtext('pose')
                assert collision.findtext('geometry/box/size') == visual.findtext('geometry/box/size')
                assert all(number > 0 and math.isfinite(number)
                           for number in values(collision.find('geometry/box/size')))


def test_no_external_assets_robot_or_sensors_are_copied():
    root = ET.fromstring(make_patrol_world())
    for tag in ['include', 'uri', 'mesh', 'sensor']:
        assert root.find('.//' + tag) is None
    assert 'wf_gazebo' not in ET.tostring(root, encoding='unicode')
    assert root.find("world/plugin[@name='gz::sim::systems::Sensors']") is not None


@pytest.mark.parametrize('index', range(4))
def test_each_door_has_clear_wall_opening_and_matching_lintel(index):
    world, spec = parsed(), manifest()['doors'][index]
    x, y, _ = spec['center_world_xyz']
    walls = collisions(world, {'building_walls'})
    for dy in [-.5, 0., .5]:
        for z in [.15, 1., 2.1]:
            assert not any(inside(model, box, [x, y+dy, z]) for model, box in walls)
    for point in [[x, y-.8, 1.], [x, y+.8, 1.], [x, y, 2.7]]:
        assert any(inside(model, box, point) for model, box in walls)
    frame = world.find(f"model[@name='{spec['frame_model']}']")
    assert frame is not None
    jambs = frame.findall("link/collision")
    assert len(jambs) == 3
    # 문틀은 복도 쪽으로 돌출하고 문 정면의 유효 폭은 비운다.
    inward_x = x + (.105 if x < 0 else -.105)
    assert not any(inside(frame, jamb, [inward_x, y, 1.]) for jamb in jambs)


@pytest.mark.parametrize('index', range(4))
def test_real_leaf_pose_matches_hinge_angle_and_opens_inside_room(index):
    world, spec = parsed(), manifest()['doors'][index]
    leaf = world.find(f"model[@name='{spec['leaf_model']}']")
    pose = values(leaf.find('pose'))
    assert pose[:3] == pytest.approx(spec['hinge_world_xyz'], abs=1e-10)
    assert pose[5] == pytest.approx(spec['wall_yaw_rad']-math.radians(spec['angle_deg']), abs=1e-10)
    slab = leaf.find("link/collision[@name='slab_collision']")
    dimensions = values(slab.find('geometry/box/size'))
    assert dimensions == pytest.approx([.04, DOOR_WIDTH, DOOR_HEIGHT])
    local = values(slab.find('pose'))
    center = [pose[0]+math.cos(pose[5])*local[0]-math.sin(pose[5])*local[1],
              pose[1]+math.sin(pose[5])*local[0]+math.cos(pose[5])*local[1], pose[2]+local[2]]
    if spec['angle_deg']:
        assert abs(center[0]) > abs(spec['center_world_xyz'][0])
    else:
        assert center[0] == pytest.approx(spec['center_world_xyz'][0])
    opening_center = [*spec['center_world_xyz'][:2], 1.]
    assert inside(leaf, slab, opening_center) == (spec['angle_deg'] == 0)


def test_robot_center_patrol_strip_is_clear_and_bounded_by_end_walls():
    world = parsed()
    solids = [(model, collision) for model, collision in collisions(world)
              if model.get('name') not in ['building_foundation', 'room_floors']]
    for x in [-.4, 0., .4]:
        for y in [-5.5, -4.8, -3., 0., 3., 5.5]:
            assert not any(inside(model, box, [x, y, .2]) for model, box in solids)
    for y in [-6., 6.]:
        assert any(inside(model, box, [0., y, .2]) for model, box in solids)
    # 어떤 가구도 문틀과 회전 문짝이 사용하는 벽에서 1.2m까지의 영역에 없다.
    furniture = world.find("model[@name='room_furniture']")
    for collision in furniture.findall('link/collision'):
        p, size = values(collision.find('pose')), values(collision.find('geometry/box/size'))
        assert abs(p[0])-size[0]/2 > 3.2


def test_committed_world_and_manifest_are_exactly_reproducible():
    assert WORLD_PATH.read_text() == make_patrol_world()
    assert json.loads(MANIFEST_PATH.read_text()) == manifest()
    main(['--check'])


def test_cli_writes_new_pair_and_refuses_overwriting_or_aliases(tmp_path):
    world, document = tmp_path/'new.sdf', tmp_path/'new.json'
    args = ['--output-world', str(world), '--output-manifest', str(document)]
    main(args)
    assert world.read_text() == make_patrol_world()
    assert json.loads(document.read_text()) == manifest()
    main(args + ['--check'])
    with pytest.raises(SystemExit):
        main(args)
    with pytest.raises(SystemExit):
        main(['--output-world', str(tmp_path/'same'), '--output-manifest', str(tmp_path/'same')])
    document.write_text('{}\n')
    with pytest.raises(SystemExit):
        main(args + ['--check'])
