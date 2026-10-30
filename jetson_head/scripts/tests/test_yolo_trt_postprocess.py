"""Pre/post-proceso YOLOv8 de yolo_trt.py (sin GPU ni TensorRT)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from yolo_trt import letterbox, nms, postprocess  # noqa: E402


def raw_output(preds, n=8400, num_classes=80):
    """preds: lista de (cx, cy, w, h, class_id, score) en coordenadas 640x640."""
    out = np.zeros((1, 4 + num_classes, n), np.float32)
    for k, (cx, cy, w, h, c, s) in enumerate(preds):
        out[0, :4, k] = (cx, cy, w, h)
        out[0, 4 + c, k] = s
    return out


def test_letterbox_640x480_pads_vertically():
    img = np.zeros((480, 640, 3), np.uint8)
    t, scale, pad = letterbox(img)
    assert t.shape == (1, 3, 640, 640) and t.dtype == np.float32
    assert scale == 1.0 and pad == (0, 80)
    assert abs(t[0, 0, 0, 0] - 114 / 255) < 1e-6 and t[0, 0, 100, 0] == 0.0


def test_postprocess_maps_back_to_image_and_filters_low_scores():
    out = raw_output([(320, 320, 100, 50, 39, 0.9), (100, 200, 20, 20, 0, 0.1)])
    dets = postprocess(out, 1.0, (0, 80), (480, 640), conf_thr=0.4)
    assert len(dets) == 1
    x1, y1, x2, y2, score, cls = dets[0]
    assert (x1, y1, x2, y2) == (270.0, 215.0, 370.0, 265.0)
    assert cls == 39 and abs(score - 0.9) < 1e-6


def test_nms_suppresses_same_class_overlap_but_keeps_other_class():
    out = raw_output([(320, 320, 100, 100, 0, 0.9), (325, 322, 100, 100, 0, 0.8), (322, 320, 100, 100, 41, 0.7)])
    dets = postprocess(out, 1.0, (0, 0), (640, 640))
    assert sorted(d[5] for d in dets) == [0, 41]


def test_no_detections_returns_empty_list():
    assert postprocess(raw_output([]), 1.0, (0, 0), (640, 640)) == []


def test_nms_basic_order():
    boxes = np.array([[0, 0, 10, 10], [1, 1, 10, 10], [50, 50, 60, 60]], np.float32)
    assert nms(boxes, np.array([0.5, 0.9, 0.3]), 0.5) == [1, 2]
