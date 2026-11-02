"""문 실험 world의 경첩·개구부·렌더링/충돌 계약을 검증한다."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from generate_worlds import make_world, STATES, WORLD_DIR


def values(element, path, default='0 0 0 0 0 0'):
    return tuple(map(float, element.findtext(path, default).split()))


def center_ray_hits(world):
    """(-3, 0, 0.5)에서 +x로 쏜 광선과 실제 box 충돌체의 교점을 구한다."""
    hits = []
    for model in world.findall('model'):
        mx, my, mz, _, _, yaw = values(model, 'pose')
        cosine, sine = math.cos(yaw), math.sin(yaw)
        # world 광선을 model 좌표로 옮긴다. 생성 world의 link는 원점에 있다.
        origin = (cosine * (-3 - mx) + sine * -my,
                  -sine * (-3 - mx) + cosine * -my, 0.5 - mz)
        direction = (cosine, -sine, 0)
        for collision in model.findall('link/collision'):
            center = values(collision, 'pose')[:3]
            size = values(collision, 'geometry/box/size')
            start, end = 0.0, math.inf
            for point, vector, midpoint, extent in zip(origin, direction, center, size):
                low, high = midpoint - extent / 2, midpoint + extent / 2
                if abs(vector) < 1e-10:
                    if not low <= point <= high:
                        end = -math.inf
                        break
                else:
                    interval = sorted(((low - point) / vector, (high - point) / vector))
                    start, end = max(start, interval[0]), min(end, interval[1])
            if start <= end:
                hits.append((start - 3, model.get('name'), collision.get('name')))
    return sorted(hits)


@pytest.mark.parametrize('state', STATES)
def test_committed_world_matches_generator_and_sensor_plugins(state):
    path = WORLD_DIR / f'door_probe_{state}.sdf'
    assert path.read_text(encoding='utf-8') == make_world(state)
    root = ET.parse(path).getroot()
    world = root.find('world')
    assert world.get('name') == 'door_probe_v1'
    assert len(world.findall('plugin')) == 5
    assert world.find("plugin[@name='gz::sim::systems::Sensors']/render_engine").text == 'ogre2'
    assert not root.findall('.//include') and not root.findall('.//mesh')
    assert all(model.findtext('static') == 'true' for model in world.findall('model'))


@pytest.mark.parametrize('state', STATES)
def test_visuals_have_matching_collision_geometry(state):
    root = ET.fromstring(make_world(state))
    for link in root.findall('.//link'):
        collisions = {item.get('name').removesuffix('_collision'): item
                      for item in link.findall('collision')}
        visuals = {item.get('name').removesuffix('_visual'): item for item in link.findall('visual')}
        assert collisions.keys() == visuals.keys()
        for name, collision in collisions.items():
            visual = visuals[name]
            assert collision.findtext('pose') == visual.findtext('pose')
            assert collision.findtext('geometry/box/size') == visual.findtext('geometry/box/size')
            assert all(size > 0 for size in values(collision, 'geometry/box/size'))


@pytest.mark.parametrize('state, angle', [('closed', 0), ('half', 45), ('open', 90)])
def test_leaf_rotates_about_fixed_hinge_toward_room(state, angle):
    root = ET.fromstring(make_world(state))
    leaf = root.find(".//model[@name='door_leaf']")
    assert values(leaf, 'pose') == pytest.approx((0, -0.5, 0, 0, 0, -math.radians(angle)))
    yaw = values(leaf, 'pose')[-1]
    # 문짝 자유단 (local 0,1,0)의 world 위치. 90°에서 (1,-0.5,0).
    tip = (-math.sin(yaw), -0.5 + math.cos(yaw))
    assert tip == pytest.approx((math.sin(math.radians(angle)), -0.5 + math.cos(math.radians(angle))))
    assert leaf.find("link/visual[@name='front_handle_lever_visual']") is not None


def test_center_depth_changes_with_door_state_and_negative_is_solid_wall():
    nearest = {state: center_ray_hits(ET.fromstring(make_world(state)).find('world'))[0]
               for state in STATES}
    assert nearest['closed'][1] == 'door_leaf'
    assert -0.05 < nearest['closed'][0] < 0
    assert nearest['half'][1] == 'door_leaf'
    assert 0.4 < nearest['half'][0] < 0.6
    assert nearest['open'][1:] == ('walls', 'background_collision')
    assert nearest['open'][0] == pytest.approx(3.95)
    assert nearest['negative'][1:] == ('walls', 'front_wall_collision')
    assert nearest['negative'][0] == pytest.approx(-0.05)
    root = ET.fromstring(make_world('negative'))
    assert not any('door' in model.get('name') for model in root.findall('.//model'))


def test_generator_leaves_unrelated_worlds_untouched(tmp_path):
    import subprocess
    import sys

    unrelated = tmp_path / 'indoor_v1.sdf'
    unrelated.write_text('사용자가 보존한 기존 world', encoding='utf-8')
    subprocess.run([sys.executable, str(Path(__file__).with_name('generate_worlds.py')),
                    '--output-dir', str(tmp_path)], check=True, capture_output=True)
    assert unrelated.read_text(encoding='utf-8') == '사용자가 보존한 기존 world'
    assert sorted(path.name for path in tmp_path.glob('door_probe_*.sdf')) == [
        'door_probe_closed.sdf', 'door_probe_half.sdf', 'door_probe_negative.sdf', 'door_probe_open.sdf']
