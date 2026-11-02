"""문짝 각도만 다른 Gazebo 실험 world와 문 없는 평벽 대조군을 생성한다.

닫힌 문 평면은 x=0, 개구부는 y=[-0.5, 0.5], z=[0, 2] m이다.
경첩 (0, -0.5, 0)에서 음의 yaw로 회전하여 문짝은 +x 방 안으로 열린다.
로봇·센서는 외부 wf_gazebo launch에서 스폰하며 이 파일에 복사하지 않는다.
"""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
WORLD_DIR = ROOT / 'src/mapping_gazebo/worlds'
STATES = {'closed': 0, 'half': 45, 'open': 90, 'negative': None}
WALL = (0.78, 0.76, 0.70, 1)
WOOD = (0.38, 0.19, 0.085, 1)
PANEL = (0.45, 0.245, 0.12, 1)
TRIM = (0.27, 0.13, 0.055, 1)
METAL = (0.5, 0.52, 0.54, 1)


def numbers(values):
    return ' '.join(f'{v:.12g}' for v in values)


def element(parent, tag, text=None, **attributes):
    node = ET.SubElement(parent, tag, attributes)
    if text is not None:
        node.text = str(text)
    return node


def model(world, name, pose=(0, 0, 0, 0, 0, 0)):
    item = element(world, 'model', name=name)
    element(item, 'static', 'true')
    element(item, 'pose', numbers(pose))
    return element(item, 'link', name='body')


def box(link, name, center, size, color):
    """렌더링과 깊이·충돌이 같은 형상을 사용하도록 쌍으로 생성한다."""
    for kind in ('collision', 'visual'):
        shape = element(link, kind, name=f'{name}_{kind}')
        element(shape, 'pose', numbers((*center, 0, 0, 0)))
        geometry = element(shape, 'geometry')
        element(element(geometry, 'box'), 'size', numbers(size))
        if kind == 'visual':
            material = element(shape, 'material')
            element(material, 'ambient', numbers(color))
            element(material, 'diffuse', numbers(color))
            element(material, 'specular', '0.08 0.08 0.08 1')


def make_world(state):
    """한 번에 하나만 실행하는 고정 각도 world의 XML 문자열을 반환한다."""
    angle = STATES[state]
    root = ET.Element('sdf', version='1.9')
    root.append(ET.Comment(
        ' scripts/door_probe/generate_worlds.py로 생성. '
        '실제 Gazebo 기하 실험용이며 실물 문 인식 성능을 나타내지 않음. '
        f'조건={state}, 문짝 각도={angle}도. '
    ))
    world = element(root, 'world', name='door_probe_v1')
    physics = element(world, 'physics', name='1ms', type='ignored')
    element(physics, 'max_step_size', '0.001')
    element(physics, 'real_time_factor', '1.0')
    for filename, name in (
        ('physics', 'Physics'), ('user-commands', 'UserCommands'),
        ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors'), ('imu', 'Imu'),
    ):
        plugin = element(world, 'plugin', filename=f'gz-sim-{filename}-system',
                         name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            element(plugin, 'render_engine', 'ogre2')
    scene = element(world, 'scene')
    element(scene, 'ambient', '0.5 0.5 0.5 1')
    element(scene, 'background', '0.7 0.75 0.8 1')
    light = element(world, 'light', type='directional', name='sun')
    element(light, 'cast_shadows', 'true')
    element(light, 'pose', '-3 -3 8 0 0 0')
    element(light, 'diffuse', '0.8 0.8 0.8 1')
    element(light, 'specular', '0.1 0.1 0.1 1')
    element(light, 'direction', '0.4 0.2 -0.9')

    floor = model(world, 'floor')
    box(floor, 'floor', (0, 0, -0.05), (20, 20, 0.1), (0.55, 0.55, 0.52, 1))
    walls = model(world, 'walls')
    # 정면 관측 범위를 비우고 개구부 뒤에 거리를 아는 배경 벽을 둔다.
    if state == 'negative':
        box(walls, 'front_wall', (0, 0, 1.5), (0.1, 12, 3), WALL)
    else:
        box(walls, 'front_left', (0, -3.25, 1.5), (0.1, 5.5, 3), WALL)
        box(walls, 'front_right', (0, 3.25, 1.5), (0.1, 5.5, 3), WALL)
        box(walls, 'above_opening', (0, 0, 2.5), (0.1, 1, 1), WALL)
    box(walls, 'background', (4, 0, 1.5), (0.1, 12, 3), WALL)
    box(walls, 'room_left', (2, -6, 1.5), (4, 0.1, 3), WALL)
    box(walls, 'room_right', (2, 6, 1.5), (4, 0.1, 3), WALL)

    if angle is not None:
        frame = model(world, 'door_frame')
        box(frame, 'left_jamb', (-0.075, -0.54, 1), (0.06, 0.08, 2), TRIM)
        box(frame, 'right_jamb', (-0.075, 0.54, 1), (0.06, 0.08, 2), TRIM)
        box(frame, 'lintel', (-0.075, 0, 2.05), (0.06, 1.16, 0.1), TRIM)
        leaf = model(world, 'door_leaf', (0, -0.5, 0, 0, 0, -math.radians(angle)))
        box(leaf, 'slab', (0, 0.5, 1), (0.04, 1, 2), WOOD)
        # 양면 패널·손잡이는 문짝과 같은 link에 두어 상태별로 함께 회전한다.
        for side, sign in (('front', -1), ('back', 1)):
            for label, z, height in (('lower', 0.48, 0.62), ('upper', 1.45, 0.78)):
                box(leaf, f'{side}_{label}_panel', (sign * 0.023, 0.5, z),
                    (0.006, 0.72, height), PANEL)
                for edge, y in (('left', 0.13), ('right', 0.87)):
                    box(leaf, f'{side}_{label}_{edge}_moulding', (sign * 0.028, y, z),
                        (0.016, 0.025, height + 0.05), TRIM)
                for edge, edge_z in (('bottom', z - height / 2 - 0.0125),
                                     ('top', z + height / 2 + 0.0125)):
                    box(leaf, f'{side}_{label}_{edge}_moulding', (sign * 0.028, 0.5, edge_z),
                        (0.016, 0.765, 0.025), TRIM)
            box(leaf, f'{side}_handle_plate', (sign * 0.027, 0.9, 1),
                (0.014, 0.045, 0.18), METAL)
            box(leaf, f'{side}_handle_spindle', (sign * 0.055, 0.9, 1.02),
                (0.07, 0.025, 0.025), METAL)
            box(leaf, f'{side}_handle_lever', (sign * 0.085, 0.845, 1.02),
                (0.025, 0.135, 0.025), METAL)
    ET.indent(root, space='  ')
    return '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=WORLD_DIR)
    options = parser.parse_args()
    options.output_dir.mkdir(parents=True, exist_ok=True)
    for state in STATES:
        path = options.output_dir / f'door_probe_{state}.sdf'
        path.write_text(make_world(state), encoding='utf-8')
        print(path)


if __name__ == '__main__':
    main()
