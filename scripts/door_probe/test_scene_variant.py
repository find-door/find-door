"""금속문 변형의 별도 출력·기하·경첩과 실제 깊이 경로를 확인한다."""
import math
import xml.etree.ElementTree as ET
import pytest
from generate_worlds import make_world
from scene_variant import variant, STATES
from test_worlds import values, center_ray_hits


@pytest.mark.parametrize('state', STATES)
def test_variant_geometry_and_material(state):
    world = ET.fromstring(variant(state)).find('world')
    assert world.get('name') == 'door_metal_v1'
    assert not world.findall('.//include')
    leaf = world.find("model[@name='door_leaf']")
    assert values(leaf, 'pose')[1] == pytest.approx(-.6)
    assert values(leaf, "link/visual[@name='slab_visual']/geometry/box/size") == pytest.approx((.04, 1.2, 2.2))
    assert not any('panel' in v.get('name') or 'moulding' in v.get('name') for v in leaf.findall('link/visual'))
    assert len(leaf.findall('link/collision')) == len(leaf.findall('link/visual'))
    for link in world.findall('model/link'):
        for visual in link.findall('visual'):
            collision = link.find(f"collision[@name='{visual.get('name').removesuffix('_visual')}_collision']")
            assert visual.findtext('pose') == collision.findtext('pose')
            assert visual.findtext('geometry/box/size') == collision.findtext('geometry/box/size')


def test_new_center_depth_and_fixed_hinge():
    worlds = {state: ET.fromstring(variant(state)).find('world') for state in STATES}
    hits = {state: center_ray_hits(world)[0] for state, world in worlds.items()}
    assert hits['closed'][1] == 'door_leaf'
    assert hits['half'][1] == 'door_leaf'
    assert hits['open'][1:] == ('walls', 'background_collision')
    assert hits['open'][0] == pytest.approx(2.95)
    assert values(worlds['open'].find("model[@name='door_leaf']"), 'pose') == pytest.approx((0, -.6, 0, 0, 0, -math.pi/2))


def test_original_generator_unchanged():
    from pathlib import Path
    root = Path(__file__).resolve().parents[2]
    for state in STATES:
        assert make_world(state) == (root / f'src/mapping_gazebo/worlds/door_probe_{state}.sdf').read_text()
