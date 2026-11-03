"""Tests offline de bench_yolo (sin GPU ni onnxruntime): formato y estadísticos."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import bench_yolo  # noqa: E402


class FakeEngine:
    """Salida YOLOv8 (1, 84, N) con una sola caja 'person' centrada."""

    def infer(self, tensor):
        out = np.zeros((1, 84, 10), dtype=np.float32)
        out[0, :4, 0] = [320, 320, 100, 200]
        out[0, 4, 0] = 0.9
        return out

    def close(self):
        pass


def test_run_uses_same_pipeline_and_measures():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    infer_ms, total_ms, dets = bench_yolo.run(FakeEngine(), img, n=5, warmup=1, conf=0.25, iou=0.45)
    assert len(infer_ms) == 5 and len(total_ms) == 5
    assert all(t >= i for t, i in zip(total_ms, infer_ms))
    assert [d[5] for d in dets] == [0]


def test_fmt_and_summary():
    dets = [(1.2, 2.6, 30.4, 40.5, 0.87654, 0)]
    assert bench_yolo.fmt(dets) == [('person', 0.877, [1, 3, 30, 40])]
    s = bench_yolo.summary([1.0, 2.0, 3.0, 4.0])
    assert s['mean'] == 2.5 and s['p50'] == 3.0
