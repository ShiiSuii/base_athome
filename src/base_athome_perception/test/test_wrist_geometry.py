"""Geometria de la camara de muneca: calibracion de distancia y ubicacion del objeto."""
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception import wrist_geometry as wg  # noqa: E402


def test_measurement_of_2026_10_30_gives_correct_advance():
    # Caja YOLO de 143 px -> Z_est 0.254 m; con cinta: 24 cm del centro de la pinza a la cara de la lata
    c = wg.WristCalib(b=0.0)
    z = wg.size_distance(143, c)
    assert z == pytest.approx(0.254, abs=1e-3)
    wg.add_measurement(c, z, 0.24)
    assert wg.advance_needed(z, c) == pytest.approx(0.24 + 0.0375, abs=1e-6)
    # el valor por defecto incorporado reproduce esa calibracion
    assert wg.advance_needed(z, wg.WristCalib()) == pytest.approx(0.2775, abs=2e-3)


def test_old_constant_placed_object_inside_gripper():
    z = wg.size_distance(143, wg.WristCalib())
    assert max(0.0, z - 0.27) == 0.0  # el bug: la resta fija dejaba 0 cm de avance


def test_two_measurements_fit_scale_and_offset():
    c = wg.WristCalib()
    c.samples = []
    true = lambda z: 1.2 * z - 0.02  # noqa: E731
    for z in (0.20, 0.35):
        wg.add_measurement(c, z, true(z) - c.object_diameter_m / 2)
    assert c.a == pytest.approx(1.2, abs=1e-6) and c.b == pytest.approx(-0.02, abs=1e-6)


def test_lateral_offsets_sign_and_camera_height():
    c = wg.WristCalib()
    left, up = wg.lateral_offsets(c.cx_px, c.cy_px, 0.3, c)
    assert left == pytest.approx(0.0) and up == pytest.approx(c.camera_above_axis_m)
    left, _ = wg.lateral_offsets(c.cx_px + 100, c.cy_px, 0.3, c)
    assert left < 0  # objeto a la derecha de la imagen -> a la derecha del robot


def test_object_center_along_horizontal_approach():
    c = wg.WristCalib()
    R = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float).T  # z herramienta = +x mundo
    R = np.column_stack([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
    p = wg.object_center([0.3, 0.0, 0.8], R, advance_m=0.2775, left_m=0.0, up_m=0.0, calib=c)
    assert p == pytest.approx([0.3 - 0.02 + 0.2775, 0.0, 0.8])
    p = wg.object_center([0.3, 0.0, 0.8], R, 0.2, left_m=0.05, up_m=0.01, calib=c)
    assert p[1] == pytest.approx(0.05) and p[2] == pytest.approx(0.81)


def test_calib_roundtrip_dict():
    c = wg.WristCalib(a=1.1, b=0.01)
    c.samples = [(0.25, 0.27)]
    d = wg.WristCalib.from_dict(c.to_dict())
    assert d.a == pytest.approx(1.1) and d.samples == [(0.25, 0.27)]


def test_alignment_offset_is_perpendicular_to_approach():
    R = np.column_stack([[0, 0, -1], [0, 1, 0], [1, 0, 0]])  # avance = +x
    d = wg.alignment_offset(R, left_m=0.03, up_m=-0.01)
    assert d == pytest.approx([0.0, 0.03, -0.01])
    assert float(np.dot(d, R[:, 2])) == pytest.approx(0.0)
    assert wg.is_aligned(0.005, 0.01) and not wg.is_aligned(0.02, 0.0)
