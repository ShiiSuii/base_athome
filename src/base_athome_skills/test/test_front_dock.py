"""Tests unitarios para el docking frontal genérico con LiDAR (HOME-101).

Valida lógica pura sin dependencias de ROS ni actuadores:
1. pared a 1.2 m de frente y girada 15° (alineación 'turn', avance 'go', parada 'done')
2. puerta cerrada entre jambas (frena a distancia segura y devuelve 'door_closed')
3. puerta abierta (hueco entre jambas: corredor libre, nunca 'door_closed')
4. mesa por patas (haz a 38 cm pasa bajo la tabla, delega y envuelve table_dock)
5. persona en el corredor (cilindro o piernas bloqueando el avance: 'blocked')
6. nada adelante (escaneo vacío sin retornos: 'nothing_ahead')
7. scan con NaN/inf (resistencia a valores no finitos)
8. objetivo demasiado lejos ('too_far') y sin escaneo ('no_scan')
9. validación de corredor trasero para undock (propuesta para node.py:459)
10. desempaquetado homogéneo de tuplas y compatibilidad de interfaz
"""

import math
import random
from typing import List, Optional, Tuple, Union

import pytest

from base_athome_skills.mission import front_dock as fd
from test_table_dock import SyntheticCylinder, SyntheticSegment, generate_scan


# ==============================================================================
# TESTS 1: PARED A 1.2 M DE FRENTE Y GIRADA 15°
# ==============================================================================

def test_pared_girada_15_grados_alinea():
    """Pared a 1.2 m girada 15° (+ y -) debe corregir orientación antes de avanzar."""
    alpha = math.radians(15.0)
    d = 1.20
    half_w = 0.80

    # Pared girada +15° (inclinada hacia la izquierda -> giro CCW positivo)
    p1 = (d * math.cos(alpha) - (-half_w) * math.sin(alpha), d * math.sin(alpha) + (-half_w) * math.cos(alpha))
    p2 = (d * math.cos(alpha) - half_w * math.sin(alpha), d * math.sin(alpha) + half_w * math.cos(alpha))
    wall_pos = SyntheticSegment(p1, p2)
    angles, ranges = generate_scan([wall_pos])

    decision = fd.front_dock_step(angles, ranges)
    assert decision.ok
    assert decision.action == "turn"
    assert decision.value == pytest.approx(alpha, abs=math.radians(2.0))

    # Pared girada -15° (inclinada hacia la derecha -> giro CW negativo)
    alpha_neg = math.radians(-15.0)
    p1_n = (d * math.cos(alpha_neg) - (-half_w) * math.sin(alpha_neg), d * math.sin(alpha_neg) + (-half_w) * math.cos(alpha_neg))
    p2_n = (d * math.cos(alpha_neg) - half_w * math.sin(alpha_neg), d * math.sin(alpha_neg) + half_w * math.cos(alpha_neg))
    wall_neg = SyntheticSegment(p1_n, p2_n)
    angles_n, ranges_n = generate_scan([wall_neg])

    decision_neg = fd.front_dock_step(angles_n, ranges_n)
    assert decision_neg.ok
    assert decision_neg.action == "turn"
    assert decision_neg.value == pytest.approx(alpha_neg, abs=math.radians(2.0))


def test_pared_1_2m_avanza_recto():
    """Pared perpendicular a 1.2 m de frente debe avanzar la distancia restante.

    d_laser = 1.20 m
    d_chassis = 1.20 - 0.02 = 1.18 m
    target_standoff = 0.25 m
    remaining = 1.18 - 0.25 = 0.93 m
    """
    wall = SyntheticSegment((1.20, -0.80), (1.20, 0.80))
    angles, ranges = generate_scan([wall])

    decision = fd.front_dock_step(angles, ranges)
    assert decision.ok
    assert decision.action == "go"
    assert decision.value == pytest.approx(0.93, abs=0.02)


def test_pared_a_distancia_segura_termina():
    """Pared a distancia segura de standoff (0.25 m del chasis) debe devolver 'done'."""
    # d_laser = 0.25 + 0.02 = 0.27 m
    wall = SyntheticSegment((0.27, -0.60), (0.27, 0.60))
    angles, ranges = generate_scan([wall])

    decision = fd.front_dock_step(angles, ranges)
    assert decision.ok
    assert decision.action == "done"
    assert decision.value == "surface"
    assert decision.info == "surface"


# ==============================================================================
# TESTS 2: PUERTA CERRADA ENTRE JAMBAS
# ==============================================================================

def test_puerta_cerrada_entre_jambas_frena_a_distancia_segura():
    """Puerta cerrada entre jambas a distancia segura (0.27 m del láser) frena con 'door_closed'."""
    # Vano de puerta de 0.84 m entre jambas (-0.42 m a +0.42 m)
    jamba_izq = SyntheticSegment((0.27, 0.42), (0.27, 1.00))
    puerta = SyntheticSegment((0.27, -0.42), (0.27, 0.42))
    jamba_der = SyntheticSegment((0.27, -1.00), (0.27, -0.42))
    angles, ranges = generate_scan([jamba_izq, puerta, jamba_der])

    decision = fd.front_dock_step(angles, ranges, target="door")
    assert decision.failed
    assert decision.action == "fail"
    assert decision.value == "door_closed"
    assert decision.reason == "door_closed"


def test_puerta_cerrada_a_distancia_avanza_o_frena_segun_config():
    """Puerta cerrada a 1.0 m: avanza si approach_closed_door=True, frena si False."""
    jamba_izq = SyntheticSegment((1.00, 0.42), (1.00, 1.00))
    puerta = SyntheticSegment((1.00, -0.42), (1.00, 0.42))
    jamba_der = SyntheticSegment((1.00, -1.00), (1.00, -0.42))
    angles, ranges = generate_scan([jamba_izq, puerta, jamba_der])

    # 1. Por defecto se aproxima hacia la distancia segura (standoff 0.25 m)
    dec_approach = fd.front_dock_step(angles, ranges, target="door")
    assert dec_approach.ok
    assert dec_approach.action == "go"
    # remaining = (1.00 - 0.02) - 0.25 = 0.73 m
    assert dec_approach.value == pytest.approx(0.73, abs=0.02)
    assert dec_approach.feature == "door_closed"

    # 2. Con approach_closed_door=False frena de inmediato reportando 'door_closed'
    cfg_stop = fd.FrontDockConfig(approach_closed_door=False)
    dec_stop = fd.front_dock_step(angles, ranges, target="door", cfg=cfg_stop)
    assert dec_stop.failed
    assert dec_stop.action == "fail"
    assert dec_stop.value == "door_closed"
    assert dec_stop.reason == "door_closed"


# ==============================================================================
# TESTS 3: PUERTA ABIERTA (HUECO: NUNCA 'door_closed')
# ==============================================================================

def test_puerta_abierta_hueco_nunca_door_closed():
    """Vano abierto de 0.90 m con corredor libre nunca debe devolver 'door_closed'."""
    # Jambas a los costados pero vano de -0.45 a +0.45 m totalmente libre
    jamba_izq = SyntheticSegment((1.00, 0.45), (1.00, 1.20))
    jamba_der = SyntheticSegment((1.00, -1.20), (1.00, -0.45))
    angles, ranges = generate_scan([jamba_izq, jamba_der])

    decision = fd.front_dock_step(angles, ranges, target="door")

    # Invariante crítico: NUNCA 'door_closed'
    assert decision.action != "fail" or decision.reason != "door_closed"
    assert decision.value != "door_closed"
    assert decision.reason != "door_closed"
    # Debe confirmar que la puerta está abierta
    assert decision.ok
    assert decision.action == "done"
    assert decision.value == "door_open"
    assert decision.info == "door_open"


# ==============================================================================
# TESTS 4: MESA POR PATAS (DELEGA EN table_dock)
# ==============================================================================

def test_mesa_por_patas_delega_en_table_dock():
    """Haz a 38 cm pasa bajo la tabla y detecta patas; delega y envuelve table_dock."""
    pata_izq = SyntheticCylinder((0.80, 0.35), 0.04)
    pata_der = SyntheticCylinder((0.80, -0.35), 0.04)
    angles, ranges = generate_scan([pata_izq, pata_der])

    # 1. Modo automático reconoce patas de mesa y delega
    dec_auto = fd.front_dock_step(angles, ranges, target="auto")
    assert dec_auto.ok
    assert dec_auto.action == "go"
    assert dec_auto.feature == "table"

    # 2. Modo explícito target="table"
    dec_table = fd.front_dock_step(angles, ranges, target="table")
    assert dec_table.ok
    assert dec_table.action == "go"
    assert dec_table.feature == "table"


def test_mesa_alcanzada_standoff_done():
    """Mesa alcanzada en standoff devuelve ('done', 'table')."""
    # Patas en posición de docking completado (standoff 0.12 m)
    pata_izq = SyntheticCylinder((0.19, 0.35), 0.04)
    pata_der = SyntheticCylinder((0.19, -0.35), 0.04)
    angles, ranges = generate_scan([pata_izq, pata_der])

    dec = fd.front_dock_step(angles, ranges, target="table")
    assert dec.ok
    assert dec.action == "done"
    assert dec.value == "table"
    assert dec.info == "table"


# ==============================================================================
# TESTS 5: PERSONA EN EL CORREDOR ('blocked')
# ==============================================================================

def test_persona_sola_en_el_corredor_bloquea():
    """Persona (cilindro estrecho de 30 cm) en el corredor frontal devuelve 'blocked'."""
    persona = SyntheticCylinder((0.70, 0.00), 0.15)
    angles, ranges = generate_scan([persona])

    decision = fd.front_dock_step(angles, ranges)
    assert decision.failed
    assert decision.action == "fail"
    assert decision.value == "blocked"
    assert decision.reason == "blocked"


def test_persona_delante_de_pared_bloquea():
    """Persona interpuesta entre el robot y una pared devuelve 'blocked'."""
    persona = SyntheticCylinder((0.70, 0.05), 0.15)
    pared = SyntheticSegment((1.40, -1.00), (1.40, 1.00))
    angles, ranges = generate_scan([persona, pared])

    decision = fd.front_dock_step(angles, ranges)
    assert decision.failed
    assert decision.action == "fail"
    assert decision.value == "blocked"
    assert decision.reason == "blocked"


@pytest.mark.parametrize("side", [-1, 1])
@pytest.mark.parametrize("with_wall", [False, True])
@pytest.mark.parametrize("target", ["auto", "wall"])
def test_borde_en_corredor_con_centro_afuera_bloquea(side, with_wall, target):
    """El borde entra 2 cm en el corredor aunque el centro esté a 40 cm del eje."""
    obstacle = SyntheticSegment((0.70, side * 0.28), (0.70, side * 0.52))
    objects = [obstacle]
    if with_wall:
        objects.append(SyntheticSegment((1.40, -1.00), (1.40, 1.00)))
    angles, ranges = generate_scan(objects)

    decision = fd.front_dock_step(angles, ranges, target=target)

    assert decision == ("fail", "blocked")
    assert decision.reason == "blocked"


@pytest.mark.parametrize("min_points", [2, 3, 4])
@pytest.mark.parametrize("target", ["auto", "wall"])
def test_cluster_minimo_delante_de_pared_bloquea(min_points, target):
    """Un cluster válido, incluidos tres retornos, basta para impedir el avance."""
    wall = SyntheticSegment((1.40, -1.00), (1.40, 1.00))
    angles, ranges = generate_scan([wall])
    center = min(range(len(angles)), key=lambda i: abs(angles[i]))
    for i in range(center, center + min_points):
        ranges[i] = 0.70 / math.cos(angles[i])

    decision = fd.front_dock_step(
        angles, ranges, target=target,
        cfg=fd.FrontDockConfig(min_cluster_points=min_points),
    )

    assert decision == ("fail", "blocked")
    assert decision.reason == "blocked"


@pytest.mark.parametrize("side", [-1, 1])
def test_obstaculo_fuera_del_corredor_no_impide_avance(side):
    obstacle = SyntheticSegment((0.70, side * 0.32), (0.70, side * 0.56))
    wall = SyntheticSegment((1.40, -1.00), (1.40, 1.00))
    angles, ranges = generate_scan([obstacle, wall])

    decision = fd.front_dock_step(angles, ranges, target="wall")

    assert decision.action == "go"
    assert decision.value == pytest.approx(1.13, abs=0.02)


def test_piernas_humanas_en_el_corredor_bloquea():
    """Dos piernas humanas con separación estrecha devuelven 'blocked'."""
    pierna_izq = SyntheticCylinder((0.70, 0.11), 0.05)
    pierna_der = SyntheticCylinder((0.70, -0.11), 0.05)
    angles, ranges = generate_scan([pierna_izq, pierna_der])

    decision = fd.front_dock_step(angles, ranges)
    assert decision.failed
    assert decision.action == "fail"
    assert decision.value == "blocked"
    assert decision.reason == "blocked"


# ==============================================================================
# TESTS 6: NADA ADELANTE ('nothing_ahead')
# ==============================================================================

def test_nada_adelante():
    """Escaneo vacío sin obstáculos dentro del rango devuelve 'nothing_ahead'."""
    angles, ranges = generate_scan([])

    decision = fd.front_dock_step(angles, ranges)
    assert decision.failed
    assert decision.action == "fail"
    assert decision.value == "nothing_ahead"
    assert decision.reason == "nothing_ahead"


# ==============================================================================
# TESTS 7: SCAN CON NAN / INF Y RESISTENCIA A RUIDO
# ==============================================================================

def test_scan_con_nan_e_inf_tolerado_y_funcional():
    """Escaneo de pared con 30% de rayos con NaN o Inf no colapsa y avanza correctamente."""
    rng = random.Random(77)
    wall = SyntheticSegment((1.00, -0.60), (1.00, 0.60))
    angles, ranges = generate_scan([wall])

    corrupted_ranges: List[float] = []
    for r in ranges:
        dice = rng.random()
        if dice < 0.15:
            corrupted_ranges.append(float("nan"))
        elif dice < 0.30:
            corrupted_ranges.append(float("inf"))
        else:
            corrupted_ranges.append(r)

    decision = fd.front_dock_step(angles, corrupted_ranges)
    assert decision.ok
    assert decision.action == "go"
    assert decision.value == pytest.approx(0.73, abs=0.03)


def test_scan_totalmente_nan_o_inf_falla_limpio():
    """Escaneo con 100% de NaN o Inf falla limpiamente con 'nothing_ahead'."""
    angles = [math.radians(-135.0 + i * 0.25) for i in range(1081)]
    all_nans = [float("nan") for _ in angles]
    all_infs = [float("inf") for _ in angles]

    dec_nans = fd.front_dock_step(angles, all_nans)
    assert dec_nans.failed
    assert dec_nans.action == "fail"
    assert dec_nans.reason == "nothing_ahead"

    dec_infs = fd.front_dock_step(angles, all_infs)
    assert dec_infs.failed
    assert dec_infs.action == "fail"
    assert dec_infs.reason == "nothing_ahead"


# ==============================================================================
# TESTS 8: OBJETIVO DEMASIADO LEJOS Y ENTRADAS NULAS
# ==============================================================================

def test_pared_demasiado_lejos():
    """Pared más allá de max_dock_distance_m (2.5 m) devuelve 'too_far'."""
    wall_far = SyntheticSegment((3.20, -1.00), (3.20, 1.00))
    angles, ranges = generate_scan([wall_far])

    decision = fd.front_dock_step(angles, ranges)
    assert decision.failed
    assert decision.action == "fail"
    assert decision.value == "too_far"
    assert decision.reason == "too_far"


def test_escaneo_invalido_devuelve_no_scan():
    """Entradas vacías o incompatibles devuelven 'no_scan'."""
    assert fd.front_dock_step([], []).reason == "no_scan"
    assert fd.front_dock_step(None, None).reason == "no_scan"
    assert fd.front_dock_step([0.1, 0.2], [1.0]).reason == "no_scan"


# ==============================================================================
# TESTS 9: VALIDACIÓN DE CORREDOR TRASERO PARA UNDOCK (node.py:459)
# ==============================================================================

def test_validacion_corredor_trasero():
    """Verifica detección de obstáculos en los cuadrantes traseros del LiDAR de 270°."""
    # 1. Corredor trasero libre
    angles_clear, ranges_clear = generate_scan([])
    ok, err = fd.check_rear_corridor_clear(angles_clear, ranges_clear)
    assert ok is True
    assert err is None

    # 2. Obstáculo trasero lateral en el cono visible (x = -0.20 m, y = 0.25 m -> ángulo ~128°)
    rear_obs = SyntheticCylinder((-0.20, 0.25), 0.05)
    angles_obs, ranges_obs = generate_scan([rear_obs])
    ok_b, err_b = fd.check_rear_corridor_clear(angles_obs, ranges_obs, check_dist_m=0.30)
    assert ok_b is False
    assert "obstáculo" in str(err_b).lower()


# ==============================================================================
# TESTS 10: INTERFAZ Y COMPATIBILIDAD CON TUPLA
# ==============================================================================

def test_interfaz_tupla_y_propiedades():
    """Garantiza compatibilidad de desempaquetado de 2-tupla (action, value)."""
    wall = SyntheticSegment((1.00, -0.60), (1.00, 0.60))
    angles, ranges = generate_scan([wall])

    dec = fd.front_dock_step(angles, ranges)

    # Desempaquetado nativo
    action, amount = dec
    assert action == "go"
    assert isinstance(amount, float)

    # Propiedades
    assert dec.action == "go"
    assert dec.value == amount
    assert dec.ok is True
    assert dec.failed is False
    assert dec.reason is None
    assert dec.feature == "surface"

    # Constantes por defecto de configuración
    cfg = fd.FrontDockConfig()
    assert cfg.target_standoff_m == 0.25
    assert cfg.corridor_half_width_m == 0.30
    assert cfg.max_speed_mps == 0.08
    assert cfg.min_door_width_m == 0.65
    assert cfg.max_door_width_m == 1.10
