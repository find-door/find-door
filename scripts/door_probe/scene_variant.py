"""기존 world를 보존하고 넓고 높은 평판 금속문 개발 변형을 생성한다."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

from generate_worlds import make_world, numbers


STATES = ('closed', 'half', 'open')


def variant(state):
    root = ET.fromstring(make_world(state))
    world = root.find('world')
    world.set('name', 'door_metal_v1')
    root.insert(0, ET.Comment(' scene_variant.py: 기존 패널 목재문의 개발 변형. 독립 시험셋 아님. '))
    # 문의 y폭과 z높이를 각각 확대한다. 경첩과 충돌·시각 기하 모두 함께 변환한다.
    for pose in world.findall('.//model/pose') + world.findall('.//link/*/pose'):
        values = list(map(float, pose.text.split()))
        values[1] *= 1.2
        values[2] *= 1.1
        pose.text = numbers(values)
    for size in world.findall('.//geometry/box/size'):
        values = list(map(float, size.text.split()))
        values[1] *= 1.2
        values[2] *= 1.1
        size.text = numbers(values)
    walls = world.find("model[@name='walls']/link")
    for kind in ('collision', 'visual'):
        # 뒤벽은 4m에서 3m로 이동한다. 측벽도 새 방 길이에 맞춘다.
        walls.find(f"{kind}[@name='background_{kind}']/pose").text = '3 0 1.65 0 0 0'
        for side in ('left', 'right'):
            wall = walls.find(f"{kind}[@name='room_{side}_{kind}']")
            values = list(map(float, wall.findtext('pose').split()))
            values[0] = 1.5
            wall.find('pose').text = numbers(values)
            wall.find('geometry/box/size').text = '3 0.12 3.3'
    leaf = world.find("model[@name='door_leaf']/link")
    for item in list(leaf):
        if 'panel' in item.get('name', '') or 'moulding' in item.get('name', ''):
            leaf.remove(item)
    colors = {'door_leaf': (.25, .36, .45, 1), 'door_frame': (.48, .50, .52, 1),
              'walls': (.88, .88, .86, 1)}
    for model in world.findall('model'):
        if model.get('name') not in colors:
            continue
        for material in model.findall('link/visual/material'):
            for name in ('ambient', 'diffuse'):
                material.find(name).text = numbers(colors[model.get('name')])
    ET.indent(root, space='  ')
    return '<?xml version="1.0" encoding="utf-8"?>\n' + ET.tostring(root, encoding='unicode') + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error('새 출력 폴더가 필요합니다')
    args.output_dir.mkdir(parents=True)
    for state in STATES:
        path = args.output_dir / f'door_metal_{state}.sdf'
        path.write_text(variant(state))
        print(path)


if __name__ == '__main__':
    main()
