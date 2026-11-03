"""복도 양쪽 네 방과 실제 문짝이 있는 별도 순찰용 건물 world를 생성한다.

기존 실험의 금속문 기하를 사용한 개발 장소 변형이다. 로봇·센서는 world에
포함하지 않으며 외부 wf_gazebo가 스폰한다. manifest의 정답은 경로·평가·
시각화 전용으로, 문 검출·깊이 상태 추론의 입력에 넣지 않는다.
"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_worlds import box, make_world, model, numbers
from scene_variant import variant


ROOT = Path(__file__).resolve().parents[2]
WORLD_PATH = ROOT / 'src/mapping_gazebo/worlds/door_patrol_v1.sdf'
MANIFEST_PATH = ROOT / 'src/mapping_gazebo/config/door_patrol_v1.json'
WORLD_NAME = 'door_patrol_v1'
WALL_HEIGHT = 3.3
WALL_THICKNESS = .12
DOOR_WIDTH = 1.2
DOOR_HEIGHT = 2.2
WALL_COLOR = (.88, .88, .86, 1.)
DOORS = (
    ('right_lower', 2., -3., 0., 0),
    ('right_upper', 2., 3., 0., 90),
    ('left_lower', -2., -3., math.pi, 45),
    ('left_upper', -2., 3., math.pi, 0),
)


def manifest():
    """world와 같은 상수에서 평가용 정답과 스폰 정보를 생성한다."""
    doors = []
    for identifier, x, y, yaw, angle in DOORS:
        doors.append(dict(
            id=identifier, center_world_xyz=[x, y, 0.], wall_yaw_rad=yaw,
            angle_deg=angle, state='closed' if angle == 0 else 'open',
            frame_model=f'door_{identifier}_frame', leaf_model=f'door_{identifier}_leaf',
            hinge_world_xyz=[x + math.sin(yaw)*DOOR_WIDTH/2,
                             y - math.cos(yaw)*DOOR_WIDTH/2, 0.],
            clear_width_m=DOOR_WIDTH, clear_height_m=DOOR_HEIGHT))
    return dict(
        schema_version=1, world_name=WORLD_NAME,
        purpose='복도 주행·좌우 문 관측을 위한 Gazebo 개발 장소 변형',
        ground_truth_policy='문 위치·각도·상태는 경로·평가·시각화 전용이며 인식 입력으로 사용하지 않음',
        independent_test=False,
        spawn=dict(x=0., y=-4.8, yaw=math.pi/2),
        bounds=dict(building=dict(x=[-5.5, 5.5], y=[-6., 6.]),
                    corridor=dict(x=[-2., 2.], y=[-6., 6.]),
                    wall_height_m=WALL_HEIGHT, wall_thickness_m=WALL_THICKNESS,
                    convention='bounds는 경계 벽 중심선 기준이며 외벽 반두께는 바깥으로 돌출'),
        room_count=4, roof=False, doors=doors,
        limitations=['사전 설계한 합성 개발 건물이며 독립·실물 성능 정답이 아님',
                     '문짝은 정적 각도로 고정되며 연속 경첩 물리 모델이 아님',
                     '로봇·카메라·LiDAR 모델은 포함하지 않음'])


def _add_doors(world):
    for spec in manifest()['doors']:
        state = {0: 'closed', 45: 'half', 90: 'open'}[spec['angle_deg']]
        source = ET.fromstring(variant(state)).find('world')
        x, y, _ = spec['center_world_xyz']
        yaw = spec['wall_yaw_rad']
        for part in ('frame', 'leaf'):
            item = deepcopy(source.find(f"model[@name='door_{part}']"))
            item.set('name', spec[f'{part}_model'])
            values = list(map(float, item.findtext('pose').split()))
            px, py = values[:2]
            values[0] = x + math.cos(yaw)*px - math.sin(yaw)*py
            values[1] = y + math.sin(yaw)*px + math.cos(yaw)*py
            values[5] += yaw
            item.find('pose').text = numbers(values)
            world.append(item)


def _add_floors(world):
    foundation = model(world, 'building_foundation')
    box(foundation, 'foundation', (0., 0., -.07), (13., 14., .1), (.33, .36, .39, 1.))
    tiles = model(world, 'room_floors')
    box(tiles, 'corridor', (0., 0., -.01), (4., 12., .02), (.64, .65, .63, 1.))
    palettes = [(.68, .73, .76, 1.), (.73, .72, .65, 1.),
                (.68, .75, .69, 1.), (.75, .68, .67, 1.)]
    for (identifier, x, y, _, _), color in zip(DOORS, palettes):
        room_x = 3.75 if x > 0 else -3.75
        box(tiles, identifier, (room_x, y, -.01), (3.5, 6., .02), color)


def _add_walls(world):
    walls = model(world, 'building_walls')
    for x, side in ((-5.5, 'west'), (5.5, 'east')):
        box(walls, side, (x, 0., WALL_HEIGHT/2),
            (WALL_THICKNESS, 12., WALL_HEIGHT), WALL_COLOR)
    for y, end in ((-6., 'south'), (6., 'north')):
        box(walls, end, (0., y, WALL_HEIGHT/2),
            (11.+WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT), WALL_COLOR)
    for x, side in ((-3.75, 'left'), (3.75, 'right')):
        box(walls, f'{side}_room_divider', (x, 0., WALL_HEIGHT/2),
            (3.5, WALL_THICKNESS, WALL_HEIGHT), WALL_COLOR)
    # 복도 경계의 문 개구부 y=[-3.6,-2.4],[2.4,3.6]를 양쪽 모두 비운다.
    for x, side in ((-2., 'left'), (2., 'right')):
        for index, (start, end) in enumerate(((-6., -3.6), (-2.4, 2.4), (3.6, 6.))):
            box(walls, f'{side}_corridor_{index}', (x, (start+end)/2, WALL_HEIGHT/2),
                (.1, end-start, WALL_HEIGHT), WALL_COLOR)
        for y, label in ((-3., 'lower'), (3., 'upper')):
            box(walls, f'{side}_{label}_lintel_wall',
                (x, y, (DOOR_HEIGHT+WALL_HEIGHT)/2),
                (.1, DOOR_WIDTH, WALL_HEIGHT-DOOR_HEIGHT), WALL_COLOR)


def _add_furniture(world):
    """문과 복도 앞을 비우고 방의 바깥쪽에 책상·선반을 배치한다."""
    furniture = model(world, 'room_furniture')
    for identifier, x, y, _, _ in DOORS:
        side = 1 if x > 0 else -1
        desk_x = side*4.65
        desk_y = y + (1.7 if y < 0 else -1.7)
        wood = (.43, .32, .22, 1.)
        dark = (.25, .29, .32, 1.)
        box(furniture, f'{identifier}_desk_top', (desk_x, desk_y, .76), (1.15, 1.4, .08), wood)
        for dx in (-.46, .46):
            for dy in (-.58, .58):
                box(furniture, f'{identifier}_desk_leg_{dx}_{dy}',
                    (desk_x+dx, desk_y+dy, .36), (.07, .07, .72), dark)
        # 낮은 선반은 문 중심선과 경첩의 1.2m 회전 범위에서 떨어져 있다.
        shelf_x = side*4.9
        shelf_y = y + (-1.8 if y < 0 else 1.8)
        box(furniture, f'{identifier}_shelf', (shelf_x, shelf_y, .5),
            (.55, 1.1, 1.), (.45, .48, .50, 1.))
        for z in (.15, .5, .85):
            box(furniture, f'{identifier}_shelf_front_{z}',
                (shelf_x-side*.29, shelf_y, z), (.035, 1., .04), dark)


def make_patrol_world():
    """외부 모델 없이 자체 box 기하만 가진 SDF 문자열을 반환한다."""
    root = ET.fromstring(make_world('negative'))
    root.insert(0, ET.Comment(' patrol_world.py 생성: 지붕을 생략한 네 방·복도 개발 건물. '
                             '문 정답 manifest는 추론 입력으로 사용하지 않음. '))
    world = root.find('world')
    world.set('name', WORLD_NAME)
    for item in list(world.findall('model')):
        world.remove(item)
    _add_floors(world)
    _add_walls(world)
    _add_doors(world)
    _add_furniture(world)
    ET.indent(root, space='  ')
    return '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def serialized_outputs():
    return make_patrol_world(), json.dumps(manifest(), ensure_ascii=False, indent=2, allow_nan=False) + '\n'


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-world', type=Path, default=WORLD_PATH)
    parser.add_argument('--output-manifest', type=Path, default=MANIFEST_PATH)
    parser.add_argument('--check', action='store_true', help='기존 두 파일과 재생성 내용의 일치만 검사')
    args = parser.parse_args(argv)
    paths = [args.output_world, args.output_manifest]
    texts = serialized_outputs()
    if args.check:
        for path, expected in zip(paths, texts):
            if not path.is_file() or path.read_text() != expected:
                parser.error(f'재생성 내용과 다릅니다: {path}')
        print('world·manifest 재생성 일치')
        return
    if paths[0].resolve() == paths[1].resolve():
        parser.error('world와 manifest는 서로 다른 경로여야 합니다')
    if any(path.exists() for path in paths):
        parser.error('기존 파일을 덮어쓰지 않습니다. 새 경로 또는 --check를 사용하세요')
    for path, contents in zip(paths, texts):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(contents)
        print(path)


if __name__ == '__main__':
    main()
