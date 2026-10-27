"""색상 표적 정답은 SDF 박스의 부피 중심이 아닌 관측 앞면이다."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def test_semantic_reference_matches_visible_front_surface():
    source = Path(__file__).resolve().parents[2]
    world = ET.parse(source / 'mapping_gazebo/worlds/semantic_v1.sdf')
    truth = yaml.safe_load((source / 'mapping_eval/config/semantic_v1_truth.yaml').read_text())
    assert truth['frame'] == 'world'
    for label, point in truth['objects'].items():
        model = world.find(f'.//model[@name="{label}"]')
        assert model is not None and model.findtext('static') == 'true'
        pose = list(map(float, model.findtext('pose').split()))
        assert pose[3:] == [0, 0, 0]
        box = list(map(float, model.findtext('link/collision/geometry/box/size').split()))
        np.testing.assert_allclose(point, [pose[0] - box[0] / 2, pose[1], pose[2]])
        assert 1.2 < point[0]  # 두 검증 관측 위치 모두 앞면 쪽이다.
