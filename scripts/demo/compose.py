"""카메라 JPEG(벽시계 이름)와 RViz 캡처 PNG(mtime)를 시각으로 맞춰 좌우 합성 프레임을 만든다."""
import bisect, os, sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
run, out, step = Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3])
out.mkdir(parents=True, exist_ok=True)
cams = sorted((int(p.stem) / 1e9, p) for p in (run / 'cam').glob('*.jpg'))
ct = [t for t, _ in cams]
rv = sorted((os.stat(p).st_mtime, p) for p in (run / 'rviz').glob('*.png'))
rv = [r for r in rv if ct[0] <= r[0] <= ct[-1]]
H = 480
font = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf', 18)
small = ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', 15)
def label(d, xy, text, f):
    x, y = xy; w = d.textlength(text, font=f)
    d.rectangle([x - 6, y - 4, x + w + 6, y + f.size + 6], fill=(0, 0, 0))
    d.text((x, y), text, font=f, fill=(255, 255, 255))
t0 = rv[0][0]
for n, (t, p) in enumerate(rv[::step]):
    i = min(bisect.bisect_left(ct, t), len(ct) - 1)
    cam = Image.open(cams[i][1]).convert('RGB').resize((640, H))
    m = Image.open(p).convert('RGB').crop((24, 68, 796, 564))
    m = m.resize((round(m.width * H / m.height), H), Image.LANCZOS)
    f = Image.new('RGB', (640 + 4 + m.width, H), (20, 20, 20))
    f.paste(cam, (0, 0)); f.paste(m, (644, 0))
    d = ImageDraw.Draw(f)
    label(d, (12, 10), 'Robot camera (RGB)', font)
    label(d, (656, 10), 'SLAM map + estimated path (RViz2)', font)
    label(d, (f.width - 150, H - 30), f'{t - t0:5.1f} s  ({step}x)', small)
    f.save(out / f'{n:05d}.png')
print(len(rv[::step]), 'frames', f.size)
