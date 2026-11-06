"""Ajuste de borde de mesa y acercamiento fino (docking) con LiDAR para Base@Home.

Reglamento RoboCup@Home 2026.2 (Pick and Place Challenge):
- El árbitro puede desplazar la mesa de comedor hasta 50 cm respecto a su posición nominal en el mapa.
- Nav2 deja la base a >= 0.44 m del obstáculo con hasta 0.18 m de tolerancia (hasta 0.62 m de base_link).
- El brazo Piper 6-DOF tiene un alcance útil en línea recta de solo 20-25 cm desde la pose de espera
  (medido en docs/ROBOT.md:204).
- Por tanto, se requiere un acercamiento fino (table docking) para posicionar el frente del chasis a 10-15 cm
  del borde de la mesa de forma perpendicular y con la orientación bloqueada.

CONSIDERACIÓN FÍSICA CRÍTICA DEL SENSOR (LIDAR A 38 CM DEL PISO):
- El telémetro láser Hokuyo UST-05LX está montado en la base a 38 cm de altura sobre el suelo (LASER_HEIGHT_M = 0.38 m).
- Una mesa estándar de comedor o escritorio tiene la tabla (tabletop) a 72-76 cm de altura, con un vano inferior libre > 65 cm.
- En consecuencia, para mesas abiertas de 4 patas, el haz planar del LiDAR pasa TOTALMENTE POR DEBAJO de la tabla
  y detecta EXCLUSIVAMENTE LAS PATAS de la mesa (clusters cilíndricos/estrechos aislados).
- Sin embargo, para mesas bajas (ratonas / coffee tables ~40 cm), escritorios con faldón frontal / cajoneras / apron,
  o frentes con manteles, el haz colisiona contra un BORDE CONTINUO.
- Este módulo maneja ambos casos de forma nativa:
    1) Caso Borde Continuo: ajuste por mínimos cuadrados totales / PCA sobre un segmento denso y alargado.
    2) Caso Patas de Mesa: agrupación de clusters estrechos (patas), identificación del par frontal con separación
       compatible (0.55 m a 2.00 m), ajuste de la recta frontal tangente y aplicación del voladizo nominal de la tabla
       (TABLE_LEG_OVERHANG_M = 0.05 m).
- Detección de personas y obstáculos:
    * Una persona de pie genera un cluster estrecho (0.15-0.40 m) o dos piernas con separación < 0.35 m
      (estrictamente menor que la separación mínima entre patas de mesa MIN_LEG_SPAN_M = 0.55 m).
    * Si se detecta un obstáculo en el corredor de avance hacia la mesa, falla indicando 'persona delante'.

LÓGICA PURA DESACOPLADA (SIN ROS NI ACTUADORES):
- Entrada: escaneo láser representado en listas (angles, ranges).
- Salida: tupla ('turn', error_rad), ('go', remaining_m), ('done', 0) o ('fail', motivo).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, List, Optional, Sequence, Tuple, Union

# ==============================================================================
# CONSTANTES DE CALIBRACIÓN - FÍSICA DEL ROBOT, SENSORES Y CONTROL
# Modificar aquí para calibrar con hardware real (Hokuyo UST-05LX, chasis hoverboard)
# ==============================================================================

# Geometría de la base del robot (properties.xacro / ROBOT.md)
CHASSIS_FRONT_M: float = 0.24         # Distancia de base_link al frente del chasis (chassis_length/2 = 0.48/2)
LASER_X_M: float = 0.22               # Offset frontal del Hokuyo UST-05LX respecto a base_link
LASER_HEIGHT_M: float = 0.38          # Altura del haz láser respecto al piso (38 cm)
CHASSIS_FRONT_OFFSET_M: float = CHASSIS_FRONT_M - LASER_X_M  # 0.02 m (chasis sobresale 2 cm delante del lidar)

# Objetivo de docking (distancia final deseada al borde de la mesa)
TARGET_STANDOFF_M: float = 0.12       # Standoff deseado entre frente del chasis y borde de la mesa (12 cm)
DEFAULT_TARGET_DISTANCE_M: float = TARGET_STANDOFF_M
TABLE_LEG_OVERHANG_M: float = 0.05    # Voladizo nominal de la tabla sobre las patas frontales (5 cm)

# Límites de seguridad y tolerancias
MIN_SAFETY_CLEARANCE_M: float = 0.05  # Margen mínimo de seguridad antes de colisión dura (5 cm)
POSITION_TOL_M: float = 0.015         # Tolerancia longitudinal de parada (1.5 cm)
ALIGN_TOL_RAD: float = math.radians(1.5)  # Tolerancia angular para avance recto (~1.5°)
MAX_DOCK_TRAVEL_M: float = 0.80       # Máximo avance total permitido en docking

# Velocidades de docking (para uso del controlador/ejecutor)
DOCK_SPEED_MPS: float = 0.06          # Velocidad de avance fino hacia la mesa (6 cm/s, máx 10 cm/s)
DOCK_MIN_SPEED_MPS: float = 0.02      # Velocidad mínima para vencer fricción estática
DOCK_TURN_RPS: float = 0.15           # Velocidad angular de alineación (rad/s)
DOCK_MIN_TURN_RPS: float = 0.03       # Velocidad angular mínima

# Filtrado del LaserScan
FOV_FILTER_DEG: float = 120.0         # Campo de visión frontal considerado (+/- 60°)
MIN_SCAN_RANGE_M: float = 0.06        # Rango mínimo medible por Hokuyo UST-05LX
MAX_SCAN_RANGE_M: float = 3.00        # Rango máximo de búsqueda para docking a mesa

# Criterios de plausibilidad de la mesa
MIN_TABLE_DIST_M: float = 0.05        # Distancia mínima plausible al borde del chasis
MAX_TABLE_DIST_M: float = 2.50        # Distancia máxima plausible tras aproximación Nav2
MIN_TABLE_WIDTH_M: float = 0.45       # Ancho mínimo de un borde continuo de mesa (45 cm)
MAX_TABLE_WIDTH_M: float = 2.50       # Ancho máximo de mesa
MAX_LINE_RMSE_M: float = 0.035        # Error cuadrático medio máximo para considerar borde recto (3.5 cm)
MAX_YAW_ERROR_RAD: float = math.radians(45.0)  # Inclinación máxima respecto al frente del robot (45°)

# Criterios de detección de patas y personas
MIN_LEG_SPAN_M: float = 0.55          # Separación mínima entre patas de mesa (55 cm)
MAX_LEG_SPAN_M: float = 2.00          # Separación máxima entre patas de mesa (200 cm)
MAX_LEG_DIAMETER_M: float = 0.15      # Diámetro máximo del cluster de una pata (15 cm)
HUMAN_LEGS_MAX_SPAN_M: float = 0.35   # Separación máxima entre piernas de una persona (35 cm)
PERSON_CLUSTER_MAX_WIDTH_M: float = 0.40  # Ancho máximo de cluster de cuerpo/persona (40 cm)
CORRIDOR_HALF_WIDTH_M: float = 0.32   # Semiancho del corredor frontal de avance del robot (32 cm)
CLUSTER_BREAK_DIST_M: float = 0.12    # Salto de distancia euclídea para separar clusters (12 cm)


# ==============================================================================
# ESTRUCTURAS DE DATOS
# ==============================================================================

@dataclass(frozen=True)
class TableDockConfig:
    """Configuración de parámetros para ajuste de borde y acercamiento a mesa."""
    target_standoff_m: float = TARGET_STANDOFF_M
    chassis_front_offset_m: float = CHASSIS_FRONT_OFFSET_M
    table_leg_overhang_m: float = TABLE_LEG_OVERHANG_M
    min_safety_clearance_m: float = MIN_SAFETY_CLEARANCE_M
    position_tol_m: float = POSITION_TOL_M
    align_tol_rad: float = ALIGN_TOL_RAD
    fov_filter_deg: float = FOV_FILTER_DEG
    min_scan_range_m: float = MIN_SCAN_RANGE_M
    max_scan_range_m: float = MAX_SCAN_RANGE_M
    min_table_dist_m: float = MIN_TABLE_DIST_M
    max_table_dist_m: float = MAX_TABLE_DIST_M
    min_table_width_m: float = MIN_TABLE_WIDTH_M
    max_table_width_m: float = MAX_TABLE_WIDTH_M
    max_line_rmse_m: float = MAX_LINE_RMSE_M
    max_yaw_error_rad: float = MAX_YAW_ERROR_RAD
    min_leg_span_m: float = MIN_LEG_SPAN_M
    max_leg_span_m: float = MAX_LEG_SPAN_M
    max_leg_diameter_m: float = MAX_LEG_DIAMETER_M
    human_legs_max_span_m: float = HUMAN_LEGS_MAX_SPAN_M
    person_cluster_max_width_m: float = PERSON_CLUSTER_MAX_WIDTH_M
    corridor_half_width_m: float = CORRIDOR_HALF_WIDTH_M
    cluster_break_dist_m: float = CLUSTER_BREAK_DIST_M
    min_cluster_points: int = 2


@dataclass(frozen=True)
class TableEdge:
    """Representación del borde frontal de una mesa ajustado a partir del LiDAR."""
    kind: str                         # 'continuous' (borde recto continuo) o 'legs' (inferido por patas)
    distance_laser_m: float           # Distancia perpendicular desde el lidar hasta la línea del borde (m)
    distance_chassis_m: float         # Distancia perpendicular desde el frente del chasis hasta el borde (m)
    yaw_error_rad: float              # Error de orientación: ángulo que debe girar el robot para quedar perpendicular (+ izquierda, - derecha)
    normal: Tuple[float, float]       # Vector normal unitario (nx, ny) apuntando hacia la mesa (nx > 0)
    center: Tuple[float, float]       # Punto central representativo del borde (xc, yc) en frame láser
    span_m: float                     # Ancho visible del borde o separación entre patas extremas (m)
    rmse_m: float                     # Error cuadrático medio del ajuste de línea (m)
    points_count: int                 # Número de puntos láser que sustentan la detección


class TableDockDecision(tuple):
    """Tupla inmutable de decisión para docking a mesa: (action, value).

    Garantiza compatibilidad bidireccional:
    - Se desempaqueta como 2-tupla nativa: `action, amount = step(...)`
    - Comparación de tuplas: `res == ('turn', 0.12)`, `res == ('go', 0.45)`, `res == ('done', 0)`
    - Acceso a propiedades orientadas a objetos: `.action`, `.value`, `.reason`, `.edge`, `.ok`, `.failed`

    Acciones posibles:
    - ('turn', error_rad): gira para alinear el frente perpendicular a la mesa.
    - ('go', remaining_m): avanza en línea recta lo que falta para el standoff.
    - ('done', 0): mesa alcanzada con alineación y distancia correcta.
    - ('fail', motivo_str): falla con explicación de por qué no se detecta o es implausible.
    """

    def __new__(cls, action: str, value: Any, reason: Optional[str] = None, edge: Optional[TableEdge] = None):
        return super().__new__(cls, (action, value))

    def __init__(self, action: str, value: Any, reason: Optional[str] = None, edge: Optional[TableEdge] = None):
        self._action = action
        self._value = value
        self._reason = reason if reason is not None else (str(value) if action == "fail" else None)
        self._edge = edge

    @property
    def action(self) -> str:
        return self[0]

    @property
    def value(self) -> Any:
        return self[1]

    @property
    def reason(self) -> Optional[str]:
        return self._reason

    @property
    def edge(self) -> Optional[TableEdge]:
        return self._edge

    @property
    def ok(self) -> bool:
        return self[0] in ("turn", "go", "done")

    @property
    def failed(self) -> bool:
        return self[0] == "fail"

    def __repr__(self) -> str:
        if self[0] == "fail":
            return f"TableDockDecision('fail', {self[1]!r})"
        return f"TableDockDecision({self[0]!r}, {self[1]!r})"


# ==============================================================================
# CLUSTERING Y AJUSTE GEOMÉTRICO (LÓGICA PURA)
# ==============================================================================

@dataclass
class _Cluster:
    pts: List[Tuple[float, float]]     # (x, y) en frame láser
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    span_x: float
    span_y: float
    diameter: float
    center_x: float
    center_y: float
    front_pt: Tuple[float, float]      # punto con menor x


def _make_cluster(pts: List[Tuple[float, float]]) -> _Cluster:
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max_x - min_x
    span_y = max_y - min_y
    diameter = math.hypot(span_x, span_y)
    center_x = sum(xs) / len(xs)
    center_y = sum(ys) / len(ys)
    front_pt = min(pts, key=lambda p: p[0])
    return _Cluster(pts, min_x, max_x, min_y, max_y, span_x, span_y, diameter, center_x, center_y, front_pt)


def _fit_line_pca(pts: Sequence[Tuple[float, float]]) -> Tuple[Tuple[float, float], float, float]:
    """Ajuste de recta 2D por Mínimos Cuadrados Totales (PCA).

    Retorna:
    - normal: (nx, ny) unitario con nx > 0 (apuntando hacia adelante/mesa)
    - d: distancia perpendicular desde el origen (0,0) a la recta
    - rmse: error cuadrático medio de las distancias ortogonales
    """
    n = len(pts)
    if n < 2:
        return (1.0, 0.0), 0.0, 0.0

    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n

    cxx = sum((p[0] - mx) ** 2 for p in pts)
    cyy = sum((p[1] - my) ** 2 for p in pts)
    cxy = sum((p[0] - mx) * (p[1] - my) for p in pts)

    disc = math.hypot(cxx - cyy, 2.0 * cxy)
    lam_max = (cxx + cyy + disc) / 2.0
    lam_min = max(0.0, (cxx + cyy - disc) / 2.0)
    rmse = math.sqrt(lam_min / n)

    if abs(cxy) > 1e-12:
        vx = cxy
        vy = lam_max - cxx
    else:
        if cxx >= cyy:
            vx, vy = 1.0, 0.0
        else:
            vx, vy = 0.0, 1.0

    norm_v = math.hypot(vx, vy)
    if norm_v > 1e-12:
        vx /= norm_v
        vy /= norm_v

    # Normal ortogonal a la recta:
    nx, ny = -vy, vx
    if nx < 0.0:
        nx, ny = -nx, -ny

    d = mx * nx + my * ny
    return (nx, ny), d, rmse


def _fit_line_ransac(pts: Sequence[Tuple[float, float]], thresh_m: float = 0.025,
                     max_iters: int = 40) -> Tuple[Tuple[float, float], float, float, List[Tuple[float, float]]]:
    """Ajuste de recta robusto RANSAC + refinamiento PCA sobre inliers."""
    n = len(pts)
    if n < 2:
        norm, d, rmse = _fit_line_pca(pts)
        return norm, d, rmse, list(pts)

    if n <= 4:
        norm, d, rmse = _fit_line_pca(pts)
        return norm, d, rmse, list(pts)

    best_inliers: List[Tuple[float, float]] = []

    # Muestreo determinista distribuido a lo largo del cluster para evitar aleatoriedad en tests
    step = max(1, n // max_iters)
    for i in range(0, n, step):
        p1 = pts[i]
        # Seleccionar segundo punto separado espacialmente
        for j in (n - 1 - i, (i + n // 2) % n):
            if i == j:
                continue
            p2 = pts[j]
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            dist = math.hypot(dx, dy)
            if dist < 0.10:
                continue
            # Normal provisional
            nx, ny = -dy / dist, dx / dist
            if nx < 0.0:
                nx, ny = -nx, -ny
            d = p1[0] * nx + p1[1] * ny

            inliers = [p for p in pts if abs(p[0] * nx + p[1] * ny - d) <= thresh_m]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                if len(best_inliers) >= int(0.90 * n):
                    break

    if len(best_inliers) < 2:
        best_inliers = list(pts)

    norm, d, rmse = _fit_line_pca(best_inliers)
    return norm, d, rmse, best_inliers


# ==============================================================================
# AJUSTE DE BORDE DE MESA (CONTINUO Y PATAS A 38 CM)
# ==============================================================================

def fit_table_edge(
    angles: Sequence[float],
    ranges: Sequence[float],
    cfg: Optional[TableDockConfig] = None,
) -> Tuple[Optional[TableEdge], Optional[str]]:
    """Ajusta el borde frontal de una mesa a partir de un escaneo LiDAR (lógica pura).

    Considera la física del haz a 38 cm de altura:
    - Borde continuo si la mesa tiene panel frontal, faldón o es baja.
    - Patas aisladas si el haz pasa por debajo de la tabla.
    - Detecta personas y obstáculos que obstruyan el paso.

    Retorna:
    - (TableEdge, None) si detecta un borde plausible.
    - (None, motivo_falla) si no ve borde o es implausible.
    """
    if cfg is None:
        cfg = TableDockConfig()

    if len(angles) != len(ranges) or len(ranges) == 0:
        return None, "escaneo vacío o dimensiones incompatibles de ángulos y rangos"

    fov_half_rad = math.radians(cfg.fov_filter_deg / 2.0)
    valid_pts: List[Tuple[float, float, float, float]] = []  # (x, y, angle, range)

    for a, r in zip(angles, ranges):
        if not math.isfinite(r):
            continue
        if r < cfg.min_scan_range_m or r > cfg.max_scan_range_m:
            continue
        if abs(a) > fov_half_rad:
            continue
        x = float(r * math.cos(a))
        y = float(r * math.sin(a))
        valid_pts.append((x, y, float(a), float(r)))

    if not valid_pts:
        return None, "no ve borde: sin retornos válidos en el campo frontal"

    # Agrupamiento por discontinuidad euclídea
    clusters_raw: List[List[Tuple[float, float]]] = []
    cur: List[Tuple[float, float]] = []

    for pt in valid_pts:
        p2d = (pt[0], pt[1])
        if not cur:
            cur.append(p2d)
            continue
        prev = cur[-1]
        dist = math.hypot(p2d[0] - prev[0], p2d[1] - prev[1])
        if dist <= cfg.cluster_break_dist_m:
            cur.append(p2d)
        else:
            if len(cur) >= cfg.min_cluster_points:
                clusters_raw.append(cur)
            cur = [p2d]
    if len(cur) >= cfg.min_cluster_points:
        clusters_raw.append(cur)

    if not clusters_raw:
        return None, "no ve borde: retornos aislados no forman ningún cluster coherente"

    clusters = [_make_cluster(c) for c in clusters_raw]

    # Clasificar clusters
    continuous_candidates: List[Tuple[_Cluster, Tuple[float, float], float, float, int]] = []
    leg_candidates: List[_Cluster] = []
    person_clusters: List[_Cluster] = []

    for c in clusters:
        # ¿Es pata? Cluster compacto y estrecho
        if c.diameter <= cfg.max_leg_diameter_m:
            leg_candidates.append(c)
        # ¿Es persona? Cluster de ancho humano en corredor frontal
        elif c.diameter <= cfg.person_cluster_max_width_m and abs(c.center_y) <= cfg.corridor_half_width_m:
            person_clusters.append(c)

        # ¿Es candidato a borde continuo de mesa?
        if c.span_y >= cfg.min_table_width_m or c.diameter >= cfg.min_table_width_m:
            norm, d, rmse, inliers = _fit_line_ransac(c.pts, thresh_m=0.025)
            inlier_ratio = len(inliers) / len(c.pts)
            # Debe ser recto y con orientación coherente con el frente
            yaw_err = math.atan2(norm[1], norm[0])
            if rmse <= cfg.max_line_rmse_m and inlier_ratio >= 0.70 and abs(yaw_err) <= cfg.max_yaw_error_rad:
                continuous_candidates.append((c, norm, d, rmse, len(inliers)))

    # -------------------------------------------------------------------------
    # CASO 1: Borde continuo detectado
    # -------------------------------------------------------------------------
    if continuous_candidates:
        # Seleccionar el más cercano al robot
        continuous_candidates.sort(key=lambda item: item[2])
        best_c, norm, d_laser, rmse, n_inliers = continuous_candidates[0]

        # Verificar si hay personas u obstáculos delante del borde en el corredor de avance
        # Solo clusters físicos significativos (al menos 5 puntos y ancho >= 6 cm) para evitar speckle / outliers
        for c in clusters:
            if c is best_c:
                continue
            if len(c.pts) < 5 or c.diameter < 0.06:
                continue
            # Distancia ortogonal del obstáculo al plano del robot
            d_obs = c.front_pt[0] * norm[0] + c.front_pt[1] * norm[1]
            if d_obs < d_laser - 0.08 and abs(c.center_y) <= cfg.corridor_half_width_m:
                return None, f"persona delante: detectado obstáculo a {c.front_pt[0]:.2f} m que bloquea la mesa"

        yaw_error = math.atan2(norm[1], norm[0])
        d_chassis = d_laser - cfg.chassis_front_offset_m
        edge = TableEdge(
            kind="continuous",
            distance_laser_m=round(d_laser, 4),
            distance_chassis_m=round(d_chassis, 4),
            yaw_error_rad=round(yaw_error, 4),
            normal=(round(norm[0], 4), round(norm[1], 4)),
            center=(round(best_c.center_x, 4), round(best_c.center_y, 4)),
            span_m=round(best_c.span_y, 4),
            rmse_m=round(rmse, 4),
            points_count=n_inliers,
        )
        return edge, None

    # -------------------------------------------------------------------------
    # CASO 2: Detección por patas de mesa (haz a 38 cm pasa bajo la tabla)
    # -------------------------------------------------------------------------
    # Primero comprobar si los clusters de patas en realidad corresponden a dos piernas humanas
    if len(leg_candidates) >= 2:
        for i in range(len(leg_candidates)):
            for j in range(i + 1, len(leg_candidates)):
                l1, l2 = leg_candidates[i], leg_candidates[j]
                span_legs = math.hypot(l1.center_x - l2.center_x, l1.center_y - l2.center_y)
                # Separación de piernas humanas: 10 a 35 cm, centradas en el frente
                if 0.10 <= span_legs <= cfg.human_legs_max_span_m:
                    mid_y = (l1.center_y + l2.center_y) / 2.0
                    mid_x = (l1.center_x + l2.center_x) / 2.0
                    if abs(mid_y) <= cfg.corridor_half_width_m:
                        return None, f"persona delante: detectadas piernas humanas a {mid_x:.2f} m"

    # Buscar pares de patas de mesa válidas (separación de 55 cm a 200 cm)
    table_leg_pairs: List[Tuple[_Cluster, _Cluster, float, Tuple[float, float], float, float]] = []

    if len(leg_candidates) >= 2:
        for i in range(len(leg_candidates)):
            for j in range(i + 1, len(leg_candidates)):
                l1, l2 = leg_candidates[i], leg_candidates[j]
                span = math.hypot(l1.front_pt[0] - l2.front_pt[0], l1.front_pt[1] - l2.front_pt[1])
                if not (cfg.min_leg_span_m <= span <= cfg.max_leg_span_m):
                    continue

                # Ordenar pata izquierda (mayor y) y pata derecha (menor y)
                left_leg = l1 if l1.front_pt[1] >= l2.front_pt[1] else l2
                right_leg = l2 if l1.front_pt[1] >= l2.front_pt[1] else l1

                # Recta a través de las caras frontales de las patas
                dx = left_leg.front_pt[0] - right_leg.front_pt[0]
                dy = left_leg.front_pt[1] - right_leg.front_pt[1]
                L = math.hypot(dx, dy)
                if L < 1e-4:
                    continue

                # Normal hacia adelante (nx > 0)
                nx, ny = dy / L, -dx / L
                if nx < 0.0:
                    nx, ny = -nx, -ny

                yaw = math.atan2(ny, nx)
                if abs(yaw) > cfg.max_yaw_error_rad:
                    continue

                d_legs = left_leg.front_pt[0] * nx + left_leg.front_pt[1] * ny
                # La tabla sobresale hacia el robot respecto a las patas (voladizo nominal)
                d_table = d_legs - cfg.table_leg_overhang_m
                avg_depth = (left_leg.front_pt[0] + right_leg.front_pt[0]) / 2.0

                table_leg_pairs.append((left_leg, right_leg, span, (nx, ny), d_table, avg_depth))

    if table_leg_pairs:
        # Seleccionar el par frontal más cercano al robot
        table_leg_pairs.sort(key=lambda item: item[5])
        left_leg, right_leg, span, norm, d_laser, _ = table_leg_pairs[0]

        # Verificar si hay obstáculos entre el robot y las patas
        for c in clusters:
            if c is left_leg or c is right_leg:
                continue
            if len(c.pts) < 5 or c.diameter < 0.06:
                continue
            d_obs = c.front_pt[0] * norm[0] + c.front_pt[1] * norm[1]
            if d_obs < d_laser - 0.08 and abs(c.center_y) <= cfg.corridor_half_width_m:
                return None, f"persona delante: detectado obstáculo a {c.front_pt[0]:.2f} m que bloquea la mesa"

        yaw_error = math.atan2(norm[1], norm[0])
        d_chassis = d_laser - cfg.chassis_front_offset_m
        cx = (left_leg.center_x + right_leg.center_x) / 2.0
        cy = (left_leg.center_y + right_leg.center_y) / 2.0
        total_pts = len(left_leg.pts) + len(right_leg.pts)

        edge = TableEdge(
            kind="legs",
            distance_laser_m=round(d_laser, 4),
            distance_chassis_m=round(d_chassis, 4),
            yaw_error_rad=round(yaw_error, 4),
            normal=(round(norm[0], 4), round(norm[1], 4)),
            center=(round(cx, 4), round(cy, 4)),
            span_m=round(span, 4),
            rmse_m=0.005,  # Línea directa de dos patas tiene residual nulo en el par
            points_count=total_pts,
        )
        return edge, None

    # -------------------------------------------------------------------------
    # CASO 3: Sin mesa detectada — diagnóstico de fallas
    # -------------------------------------------------------------------------
    # ¿Hay algo peligroso inmediatamente junto al chasis (< margen de seguridad)?
    for c in clusters:
        if c.front_pt[0] < cfg.chassis_front_offset_m + cfg.min_safety_clearance_m and abs(c.center_y) <= cfg.corridor_half_width_m:
            return None, f"demasiado cerca: riesgo de colisión a {c.front_pt[0]:.2f} m"

    # ¿Hay una persona o cluster aislado delante?
    significant_persons = [c for c in person_clusters if len(c.pts) >= 5 or c.diameter >= 0.06]
    if significant_persons:
        closest_person = min(significant_persons, key=lambda c: c.front_pt[0])
        return None, f"persona delante: detectada a {closest_person.front_pt[0]:.2f} m sin mesa detrás"

    significant_legs = [c for c in leg_candidates if len(c.pts) >= 4 and c.diameter >= 0.03]
    if significant_legs:
        closest_leg = min(significant_legs, key=lambda c: c.front_pt[0])
        if abs(closest_leg.center_y) <= cfg.corridor_half_width_m:
            return None, f"persona delante: detectado obstáculo a {closest_leg.front_pt[0]:.2f} m"

    # Revisar si hay clusters anchos pero con residual alto (mesa torcida en exceso o pared/curva)
    for c in clusters:
        if c.diameter >= cfg.min_table_width_m:
            norm, d, rmse, _ = _fit_line_ransac(c.pts, thresh_m=0.025)
            yaw_err = math.atan2(norm[1], norm[0])
            if abs(yaw_err) > cfg.max_yaw_error_rad:
                return None, f"borde implausible: inclinación excesiva ({math.degrees(abs(yaw_err)):.1f}° > 45°)"
            if rmse > cfg.max_line_rmse_m:
                return None, f"borde implausible: residual alto / puntos no colineales (RMSE {rmse:.3f} m)"

    return None, "no ve borde: puntos dispersos no corresponden a mesa ni patas de mesa"


# ==============================================================================
# FUNCIÓN PRINCIPAL DE CONTROL DE DOCKING (table_dock_step)
# ==============================================================================

def table_dock_step(
    angles_or_scan: Union[Sequence[float], Tuple[Sequence[float], Sequence[float]]],
    ranges: Optional[Sequence[float]] = None,
    *,
    target_distance_m: Optional[float] = None,
    cfg: Optional[TableDockConfig] = None,
) -> TableDockDecision:
    """Calcula la acción inmediata para acercamiento fino a la mesa con LiDAR (lógica pura).

    Parámetros:
    - angles_or_scan: lista de ángulos (rad) o tupla (angles, ranges).
    - ranges: lista de distancias (m) si angles_or_scan es solo la lista de ángulos.
    - target_distance_m: distancia final deseada (standoff) entre el frente del chasis y la mesa (m).
                         Por defecto TARGET_STANDOFF_M = 0.12 m.
    - cfg: configuración opcional de parámetros y tolerancias.

    Retorna:
    - ('turn', error_rad): gira para alinear el eje frontal del robot perpendicular a la mesa.
                           error > 0: mesa inclinada a la izquierda -> girar izquierda (CCW).
                           error < 0: mesa inclinada a la derecha -> girar derecha (CW).
    - ('go', remaining_m): avanza en línea recta la distancia restante hacia la mesa.
    - ('done', 0): alcanzado el objetivo de alineación y distancia dentro de tolerancias.
    - ('fail', motivo): falla descriptiva si no ve la mesa, hay una persona delante o el borde es implausible.
    """
    if cfg is None:
        cfg = TableDockConfig()

    # Soporte para paso como tupla (angles, ranges) o como dos argumentos independientes
    if ranges is None:
        if isinstance(angles_or_scan, (tuple, list)) and len(angles_or_scan) == 2:
            angles = angles_or_scan[0]
            ranges_val = angles_or_scan[1]
        else:
            return TableDockDecision("fail", "escaneo inválido: se requieren listas de ángulos y rangos")
    else:
        angles = angles_or_scan  # type: ignore[assignment]
        ranges_val = ranges

    target_standoff = float(target_distance_m) if target_distance_m is not None else cfg.target_standoff_m

    # Ajustar borde de la mesa
    edge, err = fit_table_edge(angles, ranges_val, cfg=cfg)
    if edge is None:
        return TableDockDecision("fail", err or "no ve borde de mesa")

    # -------------------------------------------------------------------------
    # Validación de plausibilidad física del borde
    # -------------------------------------------------------------------------
    if edge.distance_chassis_m < cfg.min_safety_clearance_m:
        return TableDockDecision(
            "fail",
            f"demasiado cerca de la mesa: riesgo de colisión ({edge.distance_chassis_m:.2f} m < {cfg.min_safety_clearance_m:.2f} m)",
            edge=edge,
        )

    if edge.distance_laser_m > cfg.max_table_dist_m:
        return TableDockDecision(
            "fail",
            f"borde implausible: mesa demasiado lejos ({edge.distance_laser_m:.2f} m > {cfg.max_table_dist_m:.2f} m)",
            edge=edge,
        )

    if abs(edge.yaw_error_rad) > cfg.max_yaw_error_rad:
        return TableDockDecision(
            "fail",
            f"borde implausible: inclinación excesiva ({math.degrees(abs(edge.yaw_error_rad)):.1f}° > 45°)",
            edge=edge,
        )

    if edge.kind == "continuous" and edge.rmse_m > cfg.max_line_rmse_m:
        return TableDockDecision(
            "fail",
            f"borde implausible: residual alto / no colineal (RMSE {edge.rmse_m:.3f} m > {cfg.max_line_rmse_m:.3f} m)",
            edge=edge,
        )

    # -------------------------------------------------------------------------
    # Cálculo de consignas ('turn', 'go', 'done')
    # -------------------------------------------------------------------------
    d_chassis = edge.distance_chassis_m
    remaining_m = d_chassis - target_standoff

    # 1. Prioridad: alineación angular con la normal de la mesa
    if abs(edge.yaw_error_rad) > cfg.align_tol_rad:
        return TableDockDecision("turn", edge.yaw_error_rad, edge=edge)

    # 2. Si está alineado, verificar si ya alcanzó la distancia deseada
    if remaining_m <= cfg.position_tol_m:
        return TableDockDecision("done", 0, edge=edge)

    # 3. Avanzar derecho lo que falta
    return TableDockDecision("go", remaining_m, edge=edge)


# Alias funcionales para máxima compatibilidad con el resto del workspace
dock_step = table_dock_step
table_dock = table_dock_step
