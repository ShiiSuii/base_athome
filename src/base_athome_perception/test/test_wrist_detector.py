"""Tracker del objeto de la camara de muneca."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception.wrist_detector import TargetTracker, iou  # noqa: E402

CAN = (360.0, 45.0, 486.0, 363.0)


def test_median_width_smooths_jitter():
    t = TargetTracker(window=5)
    out = None
    for i, w in enumerate([126, 126, 160, 126, 127]):  # un cuadro con caja inflada
        out = t.update([((360, 45, 360 + w, 363), 0.6, 'bottle')], now=i * 0.1)
    assert out[1] == 126


def test_keeps_same_object_when_another_appears():
    t = TargetTracker()
    t.update([(CAN, 0.6, 'bottle')], now=0.0)
    other = (50.0, 50.0, 150.0, 300.0)
    box, _ = t.update([(other, 0.65, 'cup'), ((362, 46, 488, 364), 0.5, 'bottle')], now=0.1)
    assert iou(box, CAN) > 0.8


def test_holds_briefly_then_forgets():
    t = TargetTracker(hold_s=0.5)
    t.update([(CAN, 0.6, 'bottle')], now=0.0)
    assert t.update([], now=0.3) is not None
    assert t.update([], now=0.8) is None


def test_ignores_low_confidence_and_tiny_boxes():
    t = TargetTracker(min_conf=0.25)
    assert t.update([(CAN, 0.1, 'bottle'), ((0, 0, 10, 10), 0.9, 'cup')], now=0.0) is None
