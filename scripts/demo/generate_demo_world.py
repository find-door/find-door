"""indoor_v1의 벽 충돌체를 그대로 쓰고 텍스처·가구를 입힌 데모 world를 만든다.

소개 영상 전용이다. 가구가 LiDAR에 보이므로 indoor_v1_truth.yaml로 평가하지 않는다.
"""
import math
import random
import re
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

WORLDS = Path(__file__).resolve().parents[2] / 'src/mapping_gazebo/worlds'
MEDIA = WORLDS / 'demo_media'


def noisy(image, amount, seed):
    rng = np.random.default_rng(seed)
    a = np.asarray(image.convert('RGB'), dtype=np.int16)
    a = a + rng.integers(-amount, amount + 1, a.shape[:2])[..., None]
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8))


def brick():
    rnd = random.Random(1)
    im = Image.new('RGB', (256, 512), (190, 182, 170))
    d = ImageDraw.Draw(im)
    for row in range(16):
        off = 0 if row % 2 else -32
        for col in range(5):
            r = rnd.randint(135, 172)
            x = off + col * 64
            d.rectangle([x + 2, row * 32 + 2, x + 62, row * 32 + 30],
                        fill=(r, r // 2 + 8, r // 3 + 6))
    return noisy(im, 8, 1)


def plaster(color, trim):
    im = Image.new('RGB', (256, 512), color)
    d = ImageDraw.Draw(im)
    d.rectangle([0, 440, 256, 512], fill=trim)
    d.rectangle([0, 434, 256, 440], fill=(240, 240, 235))
    return noisy(im, 4, 2)


def wood():
    rnd = random.Random(3)
    im = Image.new('RGB', (1024, 1024))
    d = ImageDraw.Draw(im)
    for row in range(8):
        for col in range(3):
            b = rnd.randint(-18, 18)
            x = col * 384 - (row % 3) * 128
            d.rectangle([x, row * 128, x + 382, row * 128 + 126],
                        fill=(150 + b, 104 + b, 64 + b))
    im = im.filter(ImageFilter.GaussianBlur(0.6))
    return noisy(im, 6, 3)


def tile():
    im = Image.new('RGB', (512, 512), (120, 122, 125))
    d = ImageDraw.Draw(im)
    for r in range(4):
        for c in range(4):
            fill = (222, 222, 216) if (r + c) % 2 else (168, 176, 186)
            d.rectangle([c * 128 + 2, r * 128 + 2, c * 128 + 126, r * 128 + 126], fill=fill)
    return noisy(im, 3, 4)


def poster(seed, colors):
    rnd = random.Random(seed)
    im = Image.new('RGB', (320, 240), colors[0])
    d = ImageDraw.Draw(im)
    for i in range(7):
        x, y, s = rnd.randint(0, 320), rnd.randint(0, 240), rnd.randint(30, 110)
        d.ellipse([x - s, y - s, x + s, y + s], fill=colors[1 + i % (len(colors) - 1)])
    d.rectangle([0, 0, 319, 239], outline=(30, 30, 30), width=6)
    return im


TEXTURES = {
    'brick': brick,
    'plaster_blue': lambda: plaster((150, 178, 205), (70, 92, 120)),
    'plaster_cream': lambda: plaster((226, 214, 186), (128, 100, 72)),
    'plaster_green': lambda: plaster((168, 196, 164), (74, 104, 82)),
    'wood': wood,
    'tile': tile,
    'poster_a': lambda: poster(5, [(242, 196, 70), (216, 80, 60), (40, 110, 150), (245, 240, 225)]),
    'poster_b': lambda: poster(6, [(30, 60, 90), (240, 150, 60), (120, 200, 200), (235, 235, 235)]),
    'poster_c': lambda: poster(7, [(235, 235, 225), (190, 60, 90), (60, 140, 90), (50, 50, 70)]),
}

WALL_TEXTURE = {
    'outer_south': 'brick', 'corridor_wall_1': 'brick', 'corridor_wall_2': 'brick',
    'corridor_wall_3': 'brick', 'outer_north': 'plaster_blue', 'outer_west': 'plaster_cream',
    'outer_east': 'plaster_green', 'room_divider_1': 'plaster_cream',
    'room_divider_2': 'plaster_cream',
}

# (이름, x, y, z 중심, sx, sy, sz, 색). 주행 경로에서 0.6 m 이상 떨어뜨린다.
FURNITURE = [
    ('table_top', 1.0, 4.9, 0.72, 1.2, 0.8, 0.06, (0.45, 0.28, 0.16)),
    ('table_body', 1.0, 4.9, 0.35, 0.9, 0.5, 0.70, (0.30, 0.19, 0.11)),
    ('sofa_seat', 0.45, 3.1, 0.22, 0.8, 1.6, 0.44, (0.20, 0.36, 0.55)),
    ('sofa_back', 0.15, 3.1, 0.50, 0.2, 1.6, 1.0, (0.16, 0.29, 0.46)),
    ('shelf_a', 3.6, 5.78, 0.8, 1.4, 0.4, 1.6, (0.55, 0.36, 0.20)),
    ('shelf_b', 7.0, 5.78, 0.8, 1.6, 0.4, 1.6, (0.55, 0.36, 0.20)),
    ('desk', 9.3, 3.2, 0.40, 1.2, 1.4, 0.80, (0.82, 0.80, 0.74)),
    ('crate_a', 9.4, 5.4, 0.3, 0.7, 0.7, 0.6, (0.72, 0.52, 0.26)),
    ('crate_b', 8.6, 5.5, 0.2, 0.5, 0.5, 0.4, (0.80, 0.30, 0.20)),
    ('crate_c', 4.6, 0.3, 0.25, 0.6, 0.5, 0.5, (0.72, 0.52, 0.26)),
    ('locker', 9.7, 0.8, 0.8, 0.5, 1.2, 1.6, (0.30, 0.45, 0.42)),
]
PLANTS = [(4.5, 2.6), (5.6, 2.6), (0.4, 0.4)]
POSTERS = [
    ('poster_a', 2.8, 0.01, 0), ('poster_b', 6.2, 0.01, 0), ('poster_c', 9.0, 1.99, math.pi),
    ('poster_a', 1.2, 5.99, math.pi), ('poster_b', 8.2, 5.99, math.pi),
    ('poster_c', 9.99, 4.3, -math.pi / 2),
]


def material(name):
    return (f'<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
            f'<pbr><metal><albedo_map>demo_media/{name}.jpg</albedo_map>'
            f'<metalness>0</metalness><roughness>0.9</roughness></metal></pbr></material>')


def color(rgb):
    c = ' '.join(f'{v:.2f}' for v in rgb)
    return f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material>'


def box_visual(name, pose, size, mat):
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size>'
            f'</box></geometry>{mat}</visual>')


def wall_links(source):
    pattern = r'<collision name="(\w+)_collision">\s*<pose>([^<]+)</pose>\s*<geometry><box><size>([^<]+)'
    parts = []
    for name, pose, size in re.findall(pattern, source):
        x, y, z = [float(v) for v in pose.split()[:3]]
        sx, sy, sz = [float(v) for v in size.split()]
        parts.append(f'<collision name="{name}_collision"><pose>{pose}</pose><geometry><box>'
                     f'<size>{size}</size></box></geometry></collision>')
        # 박스 면마다 텍스처 전체가 입혀지므로 약 1 m 조각으로 나눠 늘어남을 막는다.
        along_x = sx > sy
        length = sx if along_x else sy
        count = max(1, round(length))
        for i in range(count):
            step = length / count
            offset = -length / 2 + step * (i + 0.5)
            px, py = (x + offset, y) if along_x else (x, y + offset)
            tile_size = f'{step} {sy} {sz}' if along_x else f'{sx} {step} {sz}'
            parts.append(box_visual(f'{name}_{i}', f'{px} {py} {z} 0 0 0', tile_size,
                                    material(WALL_TEXTURE[name])))
    return '\n'.join(parts)


def floor_visuals():
    parts = []
    for ix in range(5):
        for iy in range(3):
            x, y = 1 + ix * 2, 1 + iy * 2
            texture = 'tile' if y < 2 else 'wood'
            parts.append(f'<visual name="floor_{ix}_{iy}"><pose>{x} {y} 0.002 0 0 0</pose><geometry>'
                         f'<plane><normal>0 0 1</normal><size>2 2</size></plane></geometry>'
                         f'{material(texture)}</visual>')
    return '\n'.join(parts)


def furniture_links():
    parts = []
    for name, x, y, z, sx, sy, sz, rgb in FURNITURE:
        pose, size = f'{x} {y} {z} 0 0 0', f'{sx} {sy} {sz}'
        parts.append(f'<collision name="{name}_collision"><pose>{pose}</pose><geometry><box>'
                     f'<size>{size}</size></box></geometry></collision>')
        parts.append(box_visual(name, pose, size, color(rgb)))
    for i, (x, y) in enumerate(PLANTS):
        parts.append(f'<collision name="plant_{i}_collision"><pose>{x} {y} 0.15 0 0 0</pose><geometry>'
                     f'<cylinder><radius>0.15</radius><length>0.3</length></cylinder></geometry></collision>')
        parts.append(f'<visual name="plant_{i}_pot"><pose>{x} {y} 0.15 0 0 0</pose><geometry>'
                     f'<cylinder><radius>0.15</radius><length>0.3</length></cylinder></geometry>'
                     f'{color((0.65, 0.35, 0.22))}</visual>')
        parts.append(f'<visual name="plant_{i}_leaf"><pose>{x} {y} 0.62 0 0 0</pose><geometry>'
                     f'<sphere><radius>0.32</radius></sphere></geometry>'
                     f'{color((0.20, 0.50, 0.24))}</visual>')
    # 포스터는 LiDAR 높이(약 0.2 m)보다 위에 있고 벽에 붙어 있어 지도에 영향을 주지 않는다.
    for i, (name, x, y, yaw) in enumerate(POSTERS):
        parts.append(box_visual(f'poster_{i}', f'{x} {y} 1.0 0 0 {yaw}', '0.9 0.02 0.68', material(name)))
    return '\n'.join(parts)


def main():
    MEDIA.mkdir(exist_ok=True)
    for name, make in TEXTURES.items():
        make().save(MEDIA / f'{name}.jpg', quality=85)
    source = (WORLDS / 'indoor_v1.sdf').read_text()
    head = source[source.index('<plugin'):source.index('<model name="ground_plane">')]
    head = head.replace('<direction>-0.5 0.1 -0.9</direction>', '<direction>-0.4 0.2 -0.9</direction>')
    world = f'''<?xml version="1.0" ?>
<!-- scripts/demo/generate_demo_world.py로 생성. 벽 충돌체는 indoor_v1과 같고 가구가 추가된 소개 영상 전용 world. -->
<sdf version="1.9">
  <world name="indoor_demo">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <scene><ambient>0.55 0.55 0.55 1</ambient><background>0.82 0.88 0.95 1</background><grid>false</grid></scene>
    {head}
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
{floor_visuals()}
      </link>
    </model>
    <model name="walls">
      <static>true</static>
      <link name="link">
{wall_links(source)}
      </link>
    </model>
    <model name="furniture">
      <static>true</static>
      <link name="link">
{furniture_links()}
      </link>
    </model>
  </world>
</sdf>
'''
    (WORLDS / 'indoor_demo.sdf').write_text(world)


if __name__ == '__main__':
    main()
