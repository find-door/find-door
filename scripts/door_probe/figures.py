"""완료된 실험의 실제 카메라 영상과 검출 overlay를 조건표 이미지로 보존한다."""

import argparse
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True, help='results.json이 있는 평가 폴더')
    parser.add_argument('--output', type=Path, required=True, help='새 이미지 폴더')
    parser.add_argument('--font', type=Path, default=Path('/usr/share/fonts/truetype/nanum/NanumGothic.ttf'),
                        help='한글 표시가 가능한 기존 글꼴 파일')
    args = parser.parse_args()
    report = json.loads((args.results / 'results.json').read_text())
    if report['status'] != 'complete' or len(report['frames']) != report['expected_frames']:
        parser.error('완료된 평가만 그림으로 만듭니다')
    if args.output.exists():
        parser.error('기존 이미지 폴더는 덮어쓰지 않습니다')
    if not args.font.is_file():
        parser.error('--font로 한글 글꼴 파일을 지정하세요')
    selected = {}
    for frame in report['frames']:
        key = (frame['distance_m'], frame['view_angle_deg'], frame['state'])
        selected.setdefault(key, frame)
    states = ['closed', 'half', 'open', 'negative']
    names = {'closed': '닫힘', 'half': '반열림', 'open': '완전 열림', 'negative': '문 없는 벽'}
    positions = [(d, a) for d in [1, 2, 3] for a in [0, -30, 30]]
    if set(selected) != {(d, a, s) for d, a in positions for s in states}:
        parser.error('36조건 전체 행렬이 필요합니다')
    args.output.mkdir(parents=True)
    font = ImageFont.truetype(str(args.font), 14)
    for kind in ('rgb', 'detections'):
        canvas = Image.new('RGB', (4 * 320, 9 * 266 + 26), '#eeeeee')
        draw = ImageDraw.Draw(canvas)
        draw.text((8, 4), 'Gazebo 로봇 카메라 영상 | 조건별 첫 프레임 | 시뮬레이션 자료', fill='black', font=font)
        for row, (distance, angle) in enumerate(positions):
            for col, state in enumerate(states):
                record = selected[distance, angle, state]
                path = Path(record['rgb']) if kind == 'rgb' else args.results / record['overlay']
                with Image.open(path) as source:
                    thumb = source.convert('RGB').resize((320, 240), Image.LANCZOS)
                x, y = col * 320, row * 266 + 26
                canvas.paste(thumb, (x, y + 26))
                label = f'{names[state]} | 거리 {distance}m | 관측각 {angle:+}°'
                draw.text((x + 6, y + 4), label, fill='black', font=font)
        canvas.save(args.output / f'{kind}-grid.png')
    # 2m 정면 대표만 별도로 보존해 본문에서 읽기 쉽게 표시한다.
    strip = Image.new('RGB', (1280, 266), '#eeeeee')
    draw = ImageDraw.Draw(strip)
    for col, state in enumerate(states):
        record = selected[2, 0, state]
        with Image.open(record['rgb']) as source:
            strip.paste(source.convert('RGB').resize((320, 240), Image.LANCZOS), (col * 320, 26))
        draw.text((col * 320 + 6, 4), f'{names[state]} | 거리 2m | 정면', fill='black', font=font)
    strip.save(args.output / 'rgb-states-2m.png')


if __name__ == '__main__':
    main()
