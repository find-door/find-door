"""좌표 변환과 점유 확률 해석이 지도 평가를 왜곡하지 않는지 확인한다."""

import json
import math

import numpy as np
from PIL import Image
import pytest
import yaml

from mapping_eval.evaluate_map import main
from mapping_eval.map_metrics import (
    distances_to_segments, evaluate_map, load_occupied_map, load_truth,
)


def write_map(tmp_path, pixels=None, **overrides):
    if pixels is None:
        pixels = [[0, 255, 255], [0, 255, 255], [0, 255, 255]]
    image = tmp_path / 'map.pgm'
    Image.fromarray(np.asarray(pixels, dtype=np.uint8)).save(image)
    metadata = {'image': 'map.pgm', 'resolution': 1.0, 'origin': [0.0, 0.0, 0.0],
                'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
                'mode': 'trinary'}
    metadata.update(overrides)
    path = tmp_path / 'map.yaml'
    path.write_text(yaml.safe_dump(metadata), encoding='utf-8')
    return path


def write_truth(tmp_path, segments=None, frame='world'):
    path = tmp_path / 'truth.yaml'
    path.write_text(yaml.safe_dump({
        'frame': frame,
        'segments': segments if segments is not None else [[0.5, 0.5, 0.5, 2.5]],
    }), encoding='utf-8')
    return path


def test_exact_map_has_zero_error_and_full_sampled_coverage(tmp_path):
    path = write_map(tmp_path)
    truth = write_truth(tmp_path)
    result = evaluate_map(path, truth, [0, 0, 0], sample_spacing=1.0)
    assert result['occupied_to_truth']['rmse_m'] == 0
    assert result['occupied_to_truth']['precision_at_tolerance'] == 1
    assert result['truth_to_occupied']['coverage_at_tolerance'] == 1
    assert result['map']['occupied_cells'] == 3
    assert len(result['inputs']['map_image']['sha256']) == 64


def test_translation_is_applied_without_automatic_alignment(tmp_path):
    path = write_map(tmp_path)
    truth = write_truth(tmp_path)
    result = evaluate_map(path, truth, [0.4, 0, 0], tolerance=0.1)
    assert result['occupied_to_truth']['mean_m'] == pytest.approx(0.4)
    assert result['occupied_to_truth']['rmse_m'] == pytest.approx(0.4)
    assert result['occupied_to_truth']['p95_m'] == pytest.approx(0.4)
    assert result['occupied_to_truth']['max_m'] == pytest.approx(0.4)
    assert result['occupied_to_truth']['precision_at_tolerance'] == 0
    assert result['truth_to_occupied']['coverage_at_tolerance'] == 0


def test_image_row_flip_and_cell_center(tmp_path):
    path = write_map(tmp_path, [[0, 255], [255, 255], [255, 0]])
    points, _ = load_occupied_map(path)
    np.testing.assert_allclose(points, [[0.5, 2.5], [1.5, 0.5]])


def test_origin_rotation_then_explicit_transform(tmp_path):
    path = write_map(tmp_path, [[0]], origin=[1, 2, math.pi / 2])
    points, _ = load_occupied_map(path)
    np.testing.assert_allclose(points, [[0.5, 2.5]])
    # map (0.5, 2.5) rotates to (-2.5, 0.5), then translates to (-0.5, 3.5).
    truth = write_truth(tmp_path, [[-0.5, 3.0, -0.5, 4.0]])
    report = evaluate_map(path, truth, [2, 3, math.pi / 2])
    assert report['occupied_to_truth']['max_m'] == pytest.approx(0, abs=1e-12)


def test_negate_and_unknown_are_respected(tmp_path):
    path = write_map(tmp_path, [[255, 127, 0]], negate=1)
    points, metadata = load_occupied_map(path)
    np.testing.assert_allclose(points, [[0.5, 0.5]])
    assert metadata['occupied_cells'] == 1
    assert metadata['unknown_cells'] == 1
    assert metadata['free_cells'] == 1


def test_non_negated_unknown_not_counted_as_wall(tmp_path):
    path = write_map(tmp_path, [[0, 205, 255]])
    points, metadata = load_occupied_map(path)
    np.testing.assert_allclose(points, [[0.5, 0.5]])
    assert metadata['unknown_cells'] == 1


def test_threshold_equality_is_unknown_in_trinary_mode(tmp_path):
    path = write_map(tmp_path, [[0, 127, 200, 255]],
                     occupied_thresh=1.0 - 127 / 255.0,
                     free_thresh=1.0 - 200 / 255.0)
    _, metadata = load_occupied_map(path)
    assert metadata['occupied_cells'] == 1
    assert metadata['unknown_cells'] == 2
    assert metadata['free_cells'] == 1


@pytest.mark.parametrize('pixels', [[[255]], [[205]]])
def test_maps_without_occupied_cells_are_rejected(tmp_path, pixels):
    with pytest.raises(ValueError, match='occupied'):
        load_occupied_map(write_map(tmp_path, pixels))


def test_distance_uses_segment_endpoints_not_infinite_wall():
    distances = distances_to_segments(np.asarray([[2., 1.], [0.5, 1.]]),
                                      np.asarray([[0., 0., 1., 0.]]))
    np.testing.assert_allclose(distances, [math.sqrt(2), 1])


def test_unmapped_truth_lowers_coverage_without_lowering_precision(tmp_path):
    path = write_map(tmp_path)
    truth = write_truth(tmp_path, [[0.5, 0.5, 0.5, 2.5], [10., 0.5, 10., 2.5]])
    report = evaluate_map(path, truth, [0, 0, 0], sample_spacing=1.0)
    assert report['occupied_to_truth']['precision_at_tolerance'] == 1
    assert report['truth_to_occupied']['coverage_at_tolerance'] == 0.5


@pytest.mark.parametrize('overrides', [
    {'mode': 'scale'}, {'mode': 'raw'}, {'resolution': 0},
    {'resolution': float('nan')}, {'origin': [0, 0]}, {'origin': [0, float('inf'), 0]},
    {'negate': 2}, {'negate': True}, {'occupied_thresh': 0.1, 'free_thresh': 0.2},
    {'occupied_thresh': 1.1}, {'free_thresh': -0.1}, {'free_thresh': float('nan')},
])
def test_invalid_map_metadata_is_rejected(tmp_path, overrides):
    with pytest.raises(ValueError):
        load_occupied_map(write_map(tmp_path, **overrides))


@pytest.mark.parametrize('segments', [[], [[0, 0, 0, 0]], [[0, 1, 2]],
                                     [[0, 0, float('inf'), 1]]])
def test_invalid_truth_is_rejected(tmp_path, segments):
    with pytest.raises(ValueError):
        load_truth(write_truth(tmp_path, segments))


def test_truth_frame_is_not_assumed(tmp_path):
    with pytest.raises(ValueError, match='world'):
        load_truth(write_truth(tmp_path, frame='map'))


@pytest.mark.parametrize('transform,tolerance,spacing', [
    ([0, 0], 0.1, 0.025), ([0, 0, float('nan')], 0.1, 0.025),
    ([0, 0, 0], 0, 0.025), ([0, 0, 0], float('inf'), 0.025),
    ([0, 0, 0], 0.1, 0),
])
def test_invalid_evaluation_parameters_are_rejected(tmp_path, transform, tolerance, spacing):
    with pytest.raises(ValueError):
        evaluate_map(write_map(tmp_path), write_truth(tmp_path), transform, tolerance, spacing)


def test_cli_requires_explicit_map_to_world(tmp_path):
    with pytest.raises(SystemExit) as error:
        main(['--map', str(write_map(tmp_path)), '--truth', str(write_truth(tmp_path)),
              '--output', str(tmp_path / 'report.json')])
    assert error.value.code == 2


def test_cli_writes_reproducible_json(tmp_path):
    output = tmp_path / 'report.json'
    assert main(['--map', str(write_map(tmp_path)), '--truth', str(write_truth(tmp_path)),
                 '--map-to-world', '0', '0', '0', '--output', str(output)]) == 0
    report = json.loads(output.read_text())
    assert report['alignment'] == 'explicit_rigid_transform_no_fitting'
    assert report['map_to_world'] == [0, 0, 0]
    assert report['tolerance_m'] == 0.1


def test_cli_does_not_overwrite_inputs(tmp_path):
    path = write_map(tmp_path)
    original = path.read_bytes()
    assert main(['--map', str(path), '--truth', str(write_truth(tmp_path)),
                 '--map-to-world', '0', '0', '0', '--output', str(path)]) == 1
    assert path.read_bytes() == original
