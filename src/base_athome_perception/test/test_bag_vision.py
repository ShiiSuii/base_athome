"""Medir la bolsa: con la escena dibujada de verdad y los casos que fallaron en el robot."""
import math

import numpy as np
import pytest

from base_athome_perception import bag_vision as bv
from base_athome_perception.wrist_detector import SimColorDetector, is_own_robot_box
from base_athome_sim import sim_scene as sc

K_HEAD = (615.0, 615.0, 320.0, 240.0)
K_WRIST = (485.0, 485.0, 320.0, 240.0)


def look_at(eye, target):
    """Ejes opticos (columnas) en el marco fijo de una camara en `eye` mirando a `target`."""
    z = np.asarray(target, float) - np.asarray(eye, float)
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], axis=1)


def view(eye, target, K, faces):
    R = look_at(eye, target)
    rgb, depth = sc.render(R.T, -R.T @ np.asarray(eye, float), K, 640, 480, faces)
    return R, rgb, depth


def test_rayo_casi_horizontal_se_descarta_regresion_1557_cm():
    # Medido en el robot: muneca con eje optico [0.95, -0.29, 0.10]. Las filas altas ni cortan y las otras dan metros.
    eye = np.array([0.5, 0.0, 0.9])
    R = look_at(eye, eye + np.array([0.95, -0.29, -0.02]))
    for box in ((280, 200, 360, 260), (280, 240, 360, 300), (280, 300, 360, 340)):
        m = bv.wrist_measure(box, eye, R, K_WRIST, 0.30)
        assert not m.ok and m.reason


def test_muneca_mirando_abajo_mide_a_centimetros():
    bag = (1.0, 0.1)
    eye = np.array([0.72, 0.1, 0.72])
    R, rgb, _ = view(eye, (1.0, 0.1, 0.1), K_WRIST, sc.bag_faces(bag, 0.30))
    box = SimColorDetector().detect(rgb)[0][0]
    m = bv.wrist_measure(box, eye, R, K_WRIST, 0.30)
    assert m.ok and math.hypot(m.xy[0] - bag[0], m.xy[1] - bag[1]) < 0.06


@pytest.mark.parametrize("bag,top", [((2.0, 0.3), 0.30), ((1.2, -0.4), 0.30), ((3.0, 1.0), 0.45)])
def test_cabeza_mide_centro_y_alto_no_la_cara_de_adelante(bag, top):
    eye = np.array([0.0, 0.0, 1.35])
    R, rgb, depth = view(eye, (bag[0], bag[1], 0.0), K_HEAD, sc.bag_faces(bag, top))
    box = SimColorDetector().detect(rgb)[0][0]
    m = bv.head_measure(box, depth, eye, R, K_HEAD)
    assert m.ok
    assert math.hypot(m.xy[0] - bag[0], m.xy[1] - bag[1]) < 0.04
    assert abs(m.top_z - top) < 0.03


def test_bolsa_cortada_por_el_borde_no_se_mide():
    eye, R = np.array([0.72, 0.1, 0.72]), look_at((0.72, 0.1, 0.72), (1.0, 0.1, 0.1))
    m = bv.wrist_measure((0, 100, 300, 300), eye, R, K_WRIST, 0.30, (640, 480))
    assert not m.ok and "borde" in m.reason


def test_cabeza_rechaza_lo_que_no_sirve():
    eye = np.array([0.0, 0.0, 1.35])
    R, _, depth = view(eye, (2.0, 0.0, 0.0), K_HEAD, [])
    assert not bv.head_measure((300, 220, 340, 260), depth, eye, R, K_HEAD).ok          # solo piso
    assert not bv.head_measure((300, 220, 340, 260), np.zeros_like(depth), eye, R, K_HEAD).ok   # sin profundidad
    R, _, depth = view(eye, (2.0, 0.0, 1.8), K_HEAD, sc.box_faces((2.0, 0.0, 1.8), (0.4, 0.4, 0.4), sc.BAG_BGR))
    assert not bv.head_measure((280, 200, 360, 280), depth, eye, R, K_HEAD).ok          # en el aire


def test_cabeza_con_bolsa_al_costado_solo_sirve_si_se_acepta_explicitamente():
    eye = np.array([0.0, 0.0, 1.35])
    bag = (1.2, 0.0)
    R, _, depth = view(eye, (1.2, 0.55, 0.0), K_HEAD, sc.bag_faces(bag, 0.30))
    box = (500, 150, 639, 330)
    assert "borde" in bv.head_measure(box, depth, eye, R, K_HEAD).reason
    assert bv.head_measure(box, depth, eye, R, K_HEAD, allow_clipped=True).ok


def test_elegir_la_cercana_a_lo_esperado_no_la_de_mayor_confianza():
    far, near = bv.Measurement(xy=(5.0, 5.0), top_z=0.3), bv.Measurement(xy=(1.0, 0.1), top_z=0.3)
    bad = bv.Measurement(reason="rayo casi horizontal")
    assert bv.choose_nearest([far, bad, near], (1.0, 0.0))[0] == 2
    best, why = bv.choose_nearest([far], (1.0, 0.0))
    assert best is None and "cm" in why
    best, why = bv.choose_nearest([bad], (1.0, 0.0))
    assert best is None and why == "rayo casi horizontal"


def test_creencia_ignora_una_medicion_loca():
    b = bv.BagBelief()
    for xy in ((1.0, 0.0), (1.02, 0.01), (0.99, -0.01)):
        assert b.update(bv.Measurement(xy=xy, top_z=0.3))
    assert not b.update(bv.Measurement(xy=(15.57, 0.0), top_z=0.3))
    assert not b.update(bv.Measurement(reason="x"))
    assert b.count == 3 and b.rejected == 1 and abs(b.xy[0] - 1.0) < 0.02 and b.spread() < 0.03


def test_creencia_no_queda_anclada_a_una_primera_medicion_mala():
    b = bv.BagBelief()
    b.update(bv.Measurement(xy=(4.0, 4.0), top_z=0.3))
    for i in range(4):
        b.update(bv.Measurement(xy=(1.0 + i * 0.01, 0.0), top_z=0.3))
    assert b.xy[0] < 1.1 and b.count == 4


def test_varias_bolsas_sin_referencia_no_se_adivina():
    a, c = bv.Measurement(xy=(1.0, 0.0)), bv.Measurement(xy=(2.0, 0.0))
    assert bv.choose_nearest([a], None)[0] == 0
    best, why = bv.choose_nearest([a, c], None)
    assert best is None and "varias" in why


def test_gesto_simulado_se_lee_como_el_de_una_persona():
    bag, person_xy = (1.5, 1.2), (2.6, 0.3)
    eye = np.array([0.5, 0.0, 1.35])
    person = sc.make_person(person_xy, eye[:2], pointing_to=bag)
    R, _, depth = view(eye, (person_xy[0], person_xy[1], 1.1), K_HEAD, sc.person_faces(person))
    people = bv.project_people([person], eye, R, K_HEAD, 640, 480)
    assert people
    target, why = bv.pointed_target(people, depth, eye, R, K_HEAD)
    assert target is not None, why
    assert math.hypot(target[0] - bag[0], target[1] - bag[1]) < 0.5


def test_sin_gesto_dice_el_motivo():
    eye = np.array([0.0, 0.0, 1.35])
    person = sc.make_person((2.0, 0.0), eye[:2])
    R, _, depth = view(eye, (2.0, 0.0, 1.1), K_HEAD, sc.person_faces(person))
    target, why = bv.pointed_target(bv.project_people([person], eye, R, K_HEAD, 640, 480), depth, eye, R, K_HEAD)
    assert target is None and "gesto" in why
    assert bv.pointed_target([], depth, eye, R, K_HEAD) == (None, "no ve personas")


def test_keypoints_fuera_de_imagen_no_inventan_persona():
    eye = np.array([0.0, 0.0, 1.35])
    R = look_at(eye, (2.0, 0.0, 0.0))
    assert bv.project_people([sc.make_person((0.0, 3.0), eye[:2])], eye, R, K_HEAD, 640, 480) == []


def test_propio_robot_abajo_se_filtra():
    assert is_own_robot_box((30, 400, 90, 470), 480) and not is_own_robot_box((250, 120, 390, 300), 480)
