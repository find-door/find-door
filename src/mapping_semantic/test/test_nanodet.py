"""모델 다운로드·ROS 없이 letterbox/출력 decode/입력 보호를 검사한다."""
from types import SimpleNamespace
import hashlib

import numpy as np
import pytest

from mapping_semantic.nanodet_detector import (
    COCO_CLASSES, CpuNanoDet, NanoDetDetector, class_aware_nms, letterbox, reverse_boxes,
)


def test_letterbox_non_square_and_reverse_roundtrip():
    rgb = np.zeros((231, 640, 3), np.uint8)
    padded, geometry = letterbox(rgb)
    assert padded.shape == (416, 416, 3)
    assert padded[0, 0].tolist() == [114, 114, 114]
    expected = np.array([33.0, 27.0, 600.0, 200.0])
    encoded = expected.copy()
    encoded[[0, 2]] = encoded[[0, 2]] * geometry.scale_x + geometry.left
    encoded[[1, 3]] = encoded[[1, 3]] * geometry.scale_y + geometry.top
    result = reverse_boxes(np.array([[*encoded, .8, 0]]), geometry)
    np.testing.assert_allclose(result[0, :4], expected)


def test_portrait_clipping_and_padding_only_boxes():
    _, geometry = letterbox(np.zeros((640, 200, 3), np.uint8))
    result = reverse_boxes(np.array([[-10, -10, 420, 420, .9, 56],
                                     [0, 10, 20, 30, .9, 0]]), geometry)
    assert len(result) == 1
    np.testing.assert_allclose(result[0, :4], [0, 0, 200, 640])
    assert COCO_CLASSES[int(result[0, 5])] == 'chair'


@pytest.mark.parametrize('image', [np.zeros((2, 2), np.uint8), np.zeros((2, 2, 3)),
                                  np.zeros((0, 2, 3), np.uint8), None])
def test_invalid_rgb(image):
    with pytest.raises(ValueError):
        letterbox(image)


@pytest.mark.parametrize('prediction', [np.zeros((2, 5)), [[0, 0, 1, 1, .5, 80]],
                                        [[0, 0, 1, 1, .5, .5]], [[0, 0, 1, 1, 2, 1]],
                                        [[2, 0, 1, 1, .5, 1]], [[0, 0, 1, 1, np.nan, 0]]])
def test_invalid_outputs(prediction):
    _, geometry = letterbox(np.zeros((20, 20, 3), np.uint8))
    with pytest.raises(ValueError):
        reverse_boxes(prediction, geometry)


@pytest.mark.parametrize('indices', [np.array([[0]]), np.array([0]), (0,)])
def test_class_aware_nms_and_index_shapes(monkeypatch, indices):
    monkeypatch.setattr('cv2.dnn.NMSBoxes', lambda *args: indices)
    boxes = np.array([[0., 0, 10, 10], [0, 0, 10, 10]])
    # 동일 위치라도 서로 다른 클래스는 유지된다.
    assert class_aware_nms(boxes, np.array([.8, .9]), np.array([0, 1]), .35, .6).tolist() == [1, 0]


def decoder():
    model = CpuNanoDet.__new__(CpuNanoDet)
    model.strides = (8,)
    model.anchors_mlvl = [np.array([[3.5, 3.5]])]
    model.project = np.arange(8)
    model.prob_threshold = .35
    model.iou_threshold = .6
    return model


def test_decode_mocked_logits_stable_and_batch_shape():
    model = decoder()
    scores = np.zeros((1, 1, 80), np.float32)
    scores[0, 0, 56] = .9
    # 큰 logits도 overflow 없이 softmax가 계산돼야 한다.
    logits = np.full((1, 1, 32), 10000., np.float32)
    result = model.post_process([scores, logits])
    assert result.shape == (1, 6)
    assert np.isfinite(result).all()
    assert result[0, 5] == 56


def test_decode_empty_detections_consistent_shape():
    result = decoder().post_process([np.zeros((1, 80)), np.zeros((1, 32))])
    assert result.shape == (0, 6)


@pytest.mark.parametrize('outputs', [[], [np.zeros((2, 80)), np.zeros((1, 32))],
                                     [np.ones((1, 80)) * np.nan, np.zeros((1, 32))]])
def test_wrong_model_output_rejected(outputs):
    with pytest.raises(ValueError):
        decoder().post_process(outputs)


def test_model_missing_checksum_and_mocked_rgb_to_bgr(tmp_path, monkeypatch):
    with pytest.raises(ValueError, match='없습니다'):
        NanoDetDetector(tmp_path / 'missing.onnx')
    model = tmp_path / 'fixture.onnx'
    model.write_bytes(b'mocked model')
    with pytest.raises(ValueError, match='불일치'):
        NanoDetDetector(model, '0' * 64)
    seen = []
    backend = SimpleNamespace(infer=lambda image: seen.append(image) or np.empty((0, 6)))
    monkeypatch.setattr('mapping_semantic.nanodet_detector.CpuNanoDet', lambda *args: backend)
    detector = NanoDetDetector(model, hashlib.sha256(model.read_bytes()).hexdigest())
    rgb = np.zeros((416, 416, 3), np.uint8)
    rgb[:, :, 0] = 255
    assert detector.infer(rgb).shape == (0, 6)
    assert seen[0][0, 0].tolist() == [0, 0, 255]
    assert detector.metadata()['backend'] == 'opencv_dnn_cpu'


def test_real_2022nov_six_output_shapes(monkeypatch):
    class FakeNet:
        def setPreferableBackend(self, value):
            pass

        def setPreferableTarget(self, value):
            pass

    monkeypatch.setattr('cv2.dnn.readNet', lambda path: FakeNet())
    model = CpuNanoDet('mock.onnx')
    assert model.strides == (8, 16, 32)
    assert [len(a) for a in model.anchors_mlvl] == [2704, 676, 169]
    outputs = []
    for count in (2704, 676, 169):
        scores = np.zeros((1, count, 80), np.float32)
        scores[0, 0, 0] = .9
        outputs.extend([scores, np.zeros((1, count, 32), np.float32)])
    result = model.post_process(outputs)
    assert result.shape[1] == 6 and len(result) >= 1
    assert np.isfinite(result).all()
    with pytest.raises(ValueError):
        model.post_process(outputs + [np.zeros((1, 36, 80)), np.zeros((1, 36, 32))])
