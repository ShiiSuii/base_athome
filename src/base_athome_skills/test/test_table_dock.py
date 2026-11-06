"""Tests unitarios para la lógica pura de table_dock (HOME-99 / HOME-58).

Valida ajuste de borde de mesa y acercamiento fino sin ROS ni actuadores:
1. borde recto: detección, avance con 'go', y llegada a destino con 'done'
2. patas: mesa abierta con haz a 38 cm (pasa bajo la tapa), 2 patas y 4 patas
3. mesa torcida: alineación angular ('turn', error) con giro a izquierda (+) y derecha (-)
4. nada adelante: escaneos vacíos o fuera de rango ('fail')
5. persona delante: obstáculo o piernas bloqueando la mesa o el corredor ('fail')
6. ruido: tolerancia a ruido gaussiano y outliers, y rechazo de ruido puro
7. límites físicos y plausibilidad: orientación excesiva, distancia excesiva
8. desempaquetado de tupla e interfaces homogéneas
"""

import math
import random
from typing import List, Optional, Tuple, Union

import pytest

from base_athome_skills.mission import table_dock as td


# ==============================================================================
# GENERADOR SINTÉTICO DE ESCANEOS HOKUYO UST-05LX (LÓGICA PURA)
# ==============================================================================

# Geometría estándar de Hokuyo UST-05LX: 270° de FOV (-135° a +135°), resolución 0.25°
HOKUYO_ANGLE_MIN = math.radians(-135.0)
HOKUYO_ANGLE_MAX = math.radians(135.0)
HOKUYO_ANGLE_INC = math.radians(0.25)
HOKUYO_NUM_RAYS = 1081


class SyntheticSegment:
    """Segmento de recta 2D definido por dos puntos extremos (x1, y1) -> (x2, y2)."""

    def __init__(self, p1: Tuple[float, float], p2: Tuple[float, float]):
        self.p1 = p1
        self.p2 = p2
        self.vx = p2[0] - p1[0]
        self.vy = p2[1] - p1[1]

    def intersect(self, angle: float) -> Optional[float]:
        ca = math.cos(angle)
        sa = math.sin(angle)
        denom = self.vx * sa - self.vy * ca
        if abs(denom) < 1e-9:
            return None
        u = (self.p1[1] * ca - self.p1[0] * sa) / denom
        if 0.0 <= u <= 1.0:
            t = (self.p1[0] + u * self.vx) / ca if abs(ca) > 1e-4 else (self.p1[1] + u * self.vy) / sa
            if t > 0.0:
                return t
        return None


class SyntheticCylinder:
    """Cilindro 2D (pata de mesa o pierna/cuerpo humano) definido por centro y radio."""

    def __init__(self, center: Tuple[float, float], radius: float):
        self.cx, self.cy = center
        self.radius = radius

    def intersect(self, angle: float) -> Optional[float]:
        ca = math.cos(angle)
        sa = math.sin(angle)
        b = self.cx * ca + self.cy * sa
        c = self.cx ** 2 + self.cy ** 2 - self.radius ** 2
        disc = b ** 2 - c
        if disc >= 0.0:
            t = b - math.sqrt(disc)
            if t > 0.0:
                return t
        return None


def generate_scan(
    objects: Optional[List[Union[SyntheticSegment, SyntheticCylinder]]] = None,
    noise_std_m: float = 0.0,
    outlier_prob: float = 0.0,
    seed: int = 42,
) -> Tuple[List[float], List[float]]:
    """Genera listas (angles, ranges) exactas para un conjunto de objetos sintéticos."""
    rng = random.Random(seed)
    angles: List[float] = [HOKUYO_ANGLE_MIN + i * HOKUYO_ANGLE_INC for i in range(HOKUYO_NUM_RAYS)]
    ranges: List[float] = []

    for a in angles:
        min_r = float("inf")
        if objects:
            for obj in objects:
                t = obj.intersect(a)
                if t is not None and t < min_r:
                    min_r = t

        # Rango máximo del Hokuyo (5.0 m) para rayos sin retorno
        r = min_r if min_r < 5.0 else float("inf")

        # Inyección de ruido gaussiano sobre mediciones válidas
        if noise_std_m > 0.0 and math.isfinite(r):
            r += rng.gauss(0.0, noise_std_m)

        # Inyección de rayos atípicos (outliers aleatorios)
        if outlier_prob > 0.0 and rng.random() < outlier_prob:
            r = rng.uniform(0.10, 4.00)

        ranges.append(r)

    return angles, ranges


# ==============================================================================
# TESTS 1: BORDE RECTO CONTINUO (DETECCIÓN, AVANCE 'go', Y LLEGADA 'done')
# ==============================================================================

def test_borde_recto_avance():
    """Mesa frontal continua a 0.80 m del lidar.

    Chasis sobresale 2 cm (CHASSIS_FRONT_OFFSET_M = 0.02 m).
    Frente del chasis a 0.78 m de la mesa.
    Standoff deseado = 0.12 m.
    Restante para avanzar = 0.78 - 0.12 = 0.66 m.
    """
    # Mesa de 1.20 m de ancho a x = 0.80 m
    mesa = SyntheticSegment((0.80, -0.60), (0.80, 0.60))
    angles, ranges = generate_scan([mesa])

    decision = td.table_dock_step(angles, ranges)

    assert decision.ok
    assert decision.action == "go"
    # Avance recto de 0.66 m dentro de tolerancia estricta
    assert decision.value == pytest.approx(0.66, abs=0.015)
    assert decision.edge is not None
    assert decision.edge.kind == "continuous"
    assert decision.edge.yaw_error_rad == pytest.approx(0.0, abs=math.radians(0.5))
    assert decision.edge.distance_laser_m == pytest.approx(0.80, abs=0.01)


def test_borde_recto_llegada_done():
    """Mesa frontal exactamente en la posición de standoff objetivo (0.12 m del chasis).

    Lidar queda a 0.14 m de la mesa (0.12 m + 0.02 m offset de chasis).
    El comando debe ser ('done', 0).
    """
    mesa = SyntheticSegment((0.14, -0.50), (0.14, 0.50))
    angles, ranges = generate_scan([mesa])

    decision = td.table_dock_step(angles, ranges)

    assert decision.ok
    assert decision.action == "done"
    assert decision.value == 0
    assert decision == ("done", 0)


# ==============================================================================
# TESTS 2: PATAS DE MESA (HAZ A 38 CM PASA BAJO LA TABLA)
# ==============================================================================

def test_patas_mesa_dos_patas_frontales():
    """Mesa de comedor donde el haz a 38 cm pasa por debajo de la tapa y ve solo 2 patas.

    Patas cilíndricas de radio 2.5 cm (diámetro 5 cm) a x = 0.90 m, y = +/- 0.45 m (luz = 0.90 m).
    La cara frontal de las patas queda en x = 0.90 - 0.025 = 0.875 m.
    Con voladizo de la tapa de 0.05 m, la tabla queda en x = 0.875 - 0.05 = 0.825 m.
    Frente del chasis queda a 0.825 - 0.02 = 0.805 m de la tabla.
    Faltan para standoff (0.12 m): 0.805 - 0.12 = 0.685 m.
    """
    pata_izq = SyntheticCylinder((0.90, 0.45), 0.025)
    pata_der = SyntheticCylinder((0.90, -0.45), 0.025)
    angles, ranges = generate_scan([pata_izq, pata_der])

    decision = td.table_dock_step(angles, ranges)

    assert decision.ok
    assert decision.action == "go"
    assert decision.edge is not None
    assert decision.edge.kind == "legs"
    assert decision.value == pytest.approx(0.685, abs=0.02)
    assert decision.edge.yaw_error_rad == pytest.approx(0.0, abs=math.radians(1.0))
    assert decision.edge.span_m == pytest.approx(0.90, abs=0.03)


def test_patas_mesa_cuatro_patas_elige_frontales():
    """Mesa de 4 patas (2 frontales y 2 traseras).

    Frontales: x = 0.85 m, y = +/- 0.40 m.
    Traseras: x = 1.60 m, y = +/- 0.40 m.
    El algoritmo debe seleccionar el par frontal y calcular la distancia respecto al frente.
    """
    p_f_izq = SyntheticCylinder((0.85, 0.40), 0.03)
    p_f_der = SyntheticCylinder((0.85, -0.40), 0.03)
    p_t_izq = SyntheticCylinder((1.60, 0.40), 0.03)
    p_t_der = SyntheticCylinder((1.60, -0.40), 0.03)

    angles, ranges = generate_scan([p_f_izq, p_f_der, p_t_izq, p_t_der])

    decision = td.table_dock_step(angles, ranges)

    assert decision.ok
    assert decision.action == "go"
    assert decision.edge is not None
    assert decision.edge.kind == "legs"
    # La cara frontal de las patas frontales está en 0.85 - 0.03 = 0.82 m
    # Tapa en 0.82 - 0.05 = 0.77 m. Chasis en 0.77 - 0.02 = 0.75 m.
    # Restante: 0.75 - 0.12 = 0.63 m.
    assert decision.value == pytest.approx(0.63, abs=0.025)


# ==============================================================================
# TESTS 3: MESA TORCIDA (ALINEACIÓN ANGULAR CON 'turn', SIGNO Y TOLERANCIA)
# ==============================================================================

def test_mesa_torcida_giro_positivo_izquierda():
    """Mesa girada +15° en sentido antihorario (el lado izquierdo está más cerca del robot).

    El robot debe girar a la izquierda (yaw positivo, CCW) para encararla perpendicularmente.
    Comando: ('turn', error) con error > 0 y ~ +15° (+0.2618 rad).
    """
    alpha = math.radians(15.0)
    d = 0.90
    half_w = 0.60
    # Puntos rotados: x = d*cos(alpha) - s*sin(alpha), y = d*sin(alpha) + s*cos(alpha)
    p1 = (d * math.cos(alpha) - (-half_w) * math.sin(alpha), d * math.sin(alpha) + (-half_w) * math.cos(alpha))
    p2 = (d * math.cos(alpha) - half_w * math.sin(alpha), d * math.sin(alpha) + half_w * math.cos(alpha))

    mesa = SyntheticSegment(p1, p2)
    angles, ranges = generate_scan([mesa])

    decision = td.table_dock_step(angles, ranges)

    assert decision.ok
    assert decision.action == "turn"
    assert decision.value > 0.0  # Giro hacia la izquierda (positivo)
    assert decision.value == pytest.approx(alpha, abs=math.radians(1.5))


def test_mesa_torcida_giro_negativo_derecha():
    """Mesa girada -20° en sentido horario (el lado derecho está más cerca del robot).

    El robot debe girar a la derecha (yaw negativo, CW) para encararla perpendicularmente.
    Comando: ('turn', error) con error < 0 y ~ -20° (-0.349 rad).
    """
    alpha = math.radians(-20.0)
    d = 0.85
    half_w = 0.55
    p1 = (d * math.cos(alpha) - (-half_w) * math.sin(alpha), d * math.sin(alpha) + (-half_w) * math.cos(alpha))
    p2 = (d * math.cos(alpha) - half_w * math.sin(alpha), d * math.sin(alpha) + half_w * math.cos(alpha))

    mesa = SyntheticSegment(p1, p2)
    angles, ranges = generate_scan([mesa])

    decision = td.table_dock_step(angles, ranges)

    assert decision.ok
    assert decision.action == "turn"
    assert decision.value < 0.0  # Giro hacia la derecha (negativo)
    assert decision.value == pytest.approx(alpha, abs=math.radians(1.5))


def test_mesa_torcida_patas():
    """Patas de mesa dispuestas con una rotación de +12°.

    El ajuste sobre las patas debe inferir la normal rotada y mandar ('turn', error > 0).
    """
    alpha = math.radians(12.0)
    d = 0.95
    s = 0.45
    c1 = (d * math.cos(alpha) - s * math.sin(alpha), d * math.sin(alpha) + s * math.cos(alpha))
    c2 = (d * math.cos(alpha) - (-s) * math.sin(alpha), d * math.sin(alpha) + (-s) * math.cos(alpha))

    pata_izq = SyntheticCylinder(c1, 0.025)
    pata_der = SyntheticCylinder(c2, 0.025)

    angles, ranges = generate_scan([pata_izq, pata_der])

    decision = td.table_dock_step(angles, ranges)

    assert decision.ok
    assert decision.action == "turn"
    assert decision.value > 0.0
    assert decision.value == pytest.approx(alpha, abs=math.radians(1.5))


# ==============================================================================
# TESTS 4: NADA ADELANTE (ESCANEOS VACÍOS O FUERA DE RANGO)
# ==============================================================================

def test_nada_adelante_inf_falla():
    """Escaneo en espacio abierto donde ningún rayo retorna (todos inf)."""
    angles, ranges = generate_scan(objects=None)

    decision = td.table_dock_step(angles, ranges)

    assert decision.failed
    assert decision.action == "fail"
    assert "no ve borde" in decision.reason.lower()


def test_nada_adelante_mas_alla_del_rango_maximo():
    """Objetos a más de 3.5 m (fuera del rango de búsqueda de docking)."""
    pared_lejana = SyntheticSegment((3.80, -2.0), (3.80, 2.0))
    angles, ranges = generate_scan([pared_lejana])

    decision = td.table_dock_step(angles, ranges)

    assert decision.failed
    assert decision.action == "fail"
    assert "no ve borde" in decision.reason.lower()


def test_escaneo_invalido_o_vacio():
    """Listas vacías o dimensiones incompatibles."""
    assert td.table_dock_step([], []).failed
    assert td.table_dock_step([0.1], [1.0, 2.0]).failed


# ==============================================================================
# TESTS 5: PERSONA DELANTE (OBSTÁCULO Y PIERNAS HUMANAS)
# ==============================================================================

def test_persona_delante_bloquea_mesa():
    """Hay una mesa a 1.40 m, pero una persona parada a 0.60 m bloquea el camino."""
    mesa = SyntheticSegment((1.40, -0.60), (1.40, 0.60))
    persona = SyntheticCylinder((0.60, 0.0), 0.15)  # Cuerpo humano

    angles, ranges = generate_scan([persona, mesa])

    decision = td.table_dock_step(angles, ranges)

    assert decision.failed
    assert decision.action == "fail"
    assert "persona delante" in decision.reason.lower()


def test_persona_sola_sin_mesa():
    """Solo una persona parada delante del robot, sin ninguna mesa."""
    persona = SyntheticCylinder((0.75, 0.0), 0.16)
    angles, ranges = generate_scan([persona])

    decision = td.table_dock_step(angles, ranges)

    assert decision.failed
    assert decision.action == "fail"
    assert "persona delante" in decision.reason.lower()


def test_persona_piernas_humanas():
    """Dos piernas humanas con separación de 22 cm (menor a la separación de patas de mesa >= 55 cm)."""
    pierna_izq = SyntheticCylinder((0.70, 0.11), 0.05)
    pierna_der = SyntheticCylinder((0.70, -0.11), 0.05)

    angles, ranges = generate_scan([pierna_izq, pierna_der])

    decision = td.table_dock_step(angles, ranges)

    assert decision.failed
    assert decision.action == "fail"
    assert "persona delante" in decision.reason.lower()


# ==============================================================================
# TESTS 6: RUIDO (RUIDO GAUSSIANO, OUTLIERS Y RUIDO PURO)
# ==============================================================================

def test_ruido_gaussiano_y_outliers_ajusta_bien():
    """Mesa frontal a 0.80 m con ruido gaussiano (sigma = 1.5 cm) y 3% de rayos espurios.

    RANSAC + PCA debe filtrar el ruido y entregar una distancia de avance consistente.
    """
    mesa = SyntheticSegment((0.80, -0.60), (0.80, 0.60))
    angles, ranges = generate_scan([mesa], noise_std_m=0.015, outlier_prob=0.03, seed=123)

    decision = td.table_dock_step(angles, ranges)

    assert decision.ok
    assert decision.action == "go"
    # Debe seguir estando muy cerca de 0.66 m (dentro de 2 cm a pesar del ruido)
    assert decision.value == pytest.approx(0.66, abs=0.02)
    assert decision.edge.yaw_error_rad == pytest.approx(0.0, abs=math.radians(1.5))


def test_ruido_puro_sin_geometria_falla():
    """Ruido aleatorio uniforme en todo el campo (sin recta ni patas)."""
    rng = random.Random(999)
    angles = [HOKUYO_ANGLE_MIN + i * HOKUYO_ANGLE_INC for i in range(HOKUYO_NUM_RAYS)]
    ranges = [rng.uniform(0.30, 2.80) for _ in angles]

    decision = td.table_dock_step(angles, ranges)

    assert decision.failed
    assert decision.action == "fail"


# ==============================================================================
# TESTS 7: LÍMITES FÍSICOS Y CRITERIOS DE IMPLAUSIBILIDAD
# ==============================================================================

def test_mesa_inclinacion_excesiva_implausible():
    """Superficie inclinada a 60° (pared o pasillo lateral, no una mesa frontal)."""
    alpha = math.radians(60.0)
    d = 0.90
    half_w = 0.60
    p1 = (d * math.cos(alpha) - (-half_w) * math.sin(alpha), d * math.sin(alpha) + (-half_w) * math.cos(alpha))
    p2 = (d * math.cos(alpha) - half_w * math.sin(alpha), d * math.sin(alpha) + half_w * math.cos(alpha))

    mesa = SyntheticSegment(p1, p2)
    angles, ranges = generate_scan([mesa])

    decision = td.table_dock_step(angles, ranges)

    assert decision.failed
    assert decision.action == "fail"
    assert "implausible" in decision.reason.lower() or "inclinación" in decision.reason.lower()


def test_mesa_demasiado_cerca_riesgo_colision():
    """Mesa a 4 cm del chasis (por debajo del margen de seguridad de 5 cm)."""
    # Lidar a 0.06 m de la mesa -> chasis a 0.04 m (< MIN_SAFETY_CLEARANCE_M = 0.05 m)
    mesa = SyntheticSegment((0.06, -0.40), (0.06, 0.40))
    angles, ranges = generate_scan([mesa])

    decision = td.table_dock_step(angles, ranges)

    assert decision.failed
    assert decision.action == "fail"
    assert "seguridad" in decision.reason.lower() or "cerca" in decision.reason.lower() or "colisión" in decision.reason.lower()


# ==============================================================================
# TESTS 8: DESEMPAQUETADO, INTERFACES Y CONSTANTES DE CALIBRACIÓN
# ==============================================================================

def test_desempaquetado_tupla_e_interfaces():
    """Garantiza que la decisión se desempaqueta como 2-tupla tradicional."""
    mesa = SyntheticSegment((0.80, -0.50), (0.80, 0.50))
    scan = generate_scan([mesa])

    # Llamada con tupla (angles, ranges)
    res1 = td.table_dock_step(scan)
    action, val = res1
    assert action == "go"
    assert isinstance(val, float)

    # Aliases td.dock_step y td.table_dock
    res2 = td.dock_step(scan[0], scan[1])
    assert res2.action == "go"
    res3 = td.table_dock(scan[0], scan[1])
    assert res3.action == "go"

    # Verificación de propiedades
    assert res1.ok is True
    assert res1.failed is False
    assert res1.reason is None
    assert res1[0] == "go"
    assert res1[1] == val


def test_constantes_de_calibracion_existencia_y_rangos():
    """Verifica que las constantes críticas existan y tengan valores físicos razonables."""
    assert td.LASER_HEIGHT_M == 0.38
    assert td.LASER_X_M == 0.22
    assert td.CHASSIS_FRONT_M == 0.24
    assert 0.05 <= td.TARGET_STANDOFF_M <= 0.20
    assert 0.02 <= td.DOCK_SPEED_MPS <= 0.10
    assert 0.50 <= td.MIN_LEG_SPAN_M <= 0.70
    assert 1.50 <= td.MAX_LEG_SPAN_M <= 2.50
    assert 0.01 <= td.POSITION_TOL_M <= 0.03
    assert 0.01 <= td.ALIGN_TOL_RAD <= 0.05
