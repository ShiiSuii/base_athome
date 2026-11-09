"""Docking frontal genérico con LiDAR para Base@Home (lógica pura sin ROS).

Permite que el robot, al terminar una navegación de Nav2, se acerque con precisión
a lo que tiene adelante usando el Hokuyo UST-05LX (haz a 0.38 m del piso, FOV 270°):
- Superficie continua y plana (pared o puerta): acercamiento perpendicular hasta distancia segura configurable.
- Puerta cerrada: si target='door' y hay superficie del ancho de una puerta (0.7-1.0 m), frena y devuelve 'door_closed'.
- Puerta abierta: hueco en el corredor frontal; devuelve 'door_open' y nunca 'door_closed'.
- Mesa: delega y envuelve table_dock.py (HOME-99/HOME-58) sin duplicar código.
- Persona u obstáculo en el corredor: devuelve 'blocked'.

Salida por cada escaneo: tupla compatible (action, value):
('turn', error_rad) | ('go', remaining_m) | ('done', info) | ('fail', motivo)
con motivo in {'door_closed', 'blocked', 'nothing_ahead', 'too_far', 'no_scan'}.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, List, Optional, Sequence, Tuple, Union

from base_athome_skills.mission import table_dock as td

# ==============================================================================
# CONFIGURACIÓN POR DEFECTO
# ==============================================================================

@dataclass
class FrontDockConfig:
    """Parámetros de calibración y tolerancias para el docking frontal genérico."""
    # Distancia final deseada (standoff) entre el frente del chasis y la superficie/obstáculo (m)
    target_standoff_m: float = 0.25
    # Ancho del corredor frontal considerado para avance y detección de bloqueos (m)
    corridor_half_width_m: float = 0.30  # Ancho total = 0.60 m (radio robot = 0.44 m)
    # Velocidad máxima sugerida de avance (m/s)
    max_speed_mps: float = 0.08
    # Velocidad angular sugerida de giro (rad/s)
    max_turn_rps: float = 0.15
    # Tolerancias de convergencia
    align_tol_rad: float = math.radians(2.0)  # ~2.0 grados
    position_tol_m: float = 0.02              # 2.0 cm
    # Parámetros de puertas domésticas estándar (RoboCup@Home)
    min_door_width_m: float = 0.65
    max_door_width_m: float = 1.10
    # Geometría física del robot Base@Home (ROBOT.md / properties.xacro)
    chassis_front_m: float = 0.24             # base_link al frente del chasis
    laser_x_m: float = 0.22                   # base_link al origen del sensor láser
    chassis_front_offset_m: float = 0.02      # 0.24 - 0.22 (chasis sobresale 2 cm adelante del láser)
    # Rangos de búsqueda LiDAR
    max_dock_distance_m: float = 2.50         # Distancia máxima de inicio de docking
    min_scan_range_m: float = 0.06            # Rango mínimo medible
    max_scan_range_m: float = 4.00            # Rango máximo considerado
    min_safety_clearance_m: float = 0.05      # Distancia crítica de frenado de emergencia
    # Calidad de ajuste de línea
    max_line_rmse_m: float = 0.04             # RMSE máximo para considerar superficie plana
    max_yaw_error_rad: float = math.radians(45.0)  # Inclinación máxima corregible
    # Detección de personas y obstáculos
    person_cluster_max_width_m: float = 0.45  # Diámetro máximo de torso o pierna humana
    cluster_break_dist_m: float = 0.10        # Umbral euclídeo de segmentación de clusters
    min_cluster_points: int = 3
    # Comportamiento con puertas cerradas
    approach_closed_door: bool = True         # Si True, avanza hasta standoff seguro antes de devolver 'door_closed'


# ==============================================================================
# CLASE DE DECISIÓN (FrontDockDecision)
# ==============================================================================

class FrontDockDecision(tuple):
    """Tupla inmutable de decisión de docking frontal: (action, value).

    Desempaquetable como 2-tupla nativa:
        action, amount = front_dock_step(...)
    Comparación de tuplas:
        res == ('turn', 0.10)
        res == ('go', 0.50)
        res == ('done', 'surface')
        res == ('fail', 'door_closed')

    Propiedades:
    - action: 'turn' | 'go' | 'done' | 'fail'
    - value: float (rad o m), string informativo o motivo de fallo
    - reason: motivo normalizado en caso de fallo (in {'door_closed', 'blocked', 'nothing_ahead', 'too_far', 'no_scan'})
    - info: detalle informativo en caso de éxito ('surface', 'table', 'door_open')
    - feature: tipo de objetivo detectado ('surface', 'door_closed', 'door_open', 'table')
    """

    def __new__(
        cls,
        action: str,
        value: Any,
        reason: Optional[str] = None,
        info: Optional[str] = None,
        feature: Optional[str] = None,
    ):
        return super().__new__(cls, (action, value))

    def __init__(
        self,
        action: str,
        value: Any,
        reason: Optional[str] = None,
        info: Optional[str] = None,
        feature: Optional[str] = None,
    ):
        self._action = action
        self._value = value
        self._reason = reason if reason is not None else (str(value) if action == "fail" else None)
        self._info = info if info is not None else (str(value) if action == "done" else None)
        self._feature = feature

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
    def info(self) -> Optional[str]:
        return self._info

    @property
    def feature(self) -> Optional[str]:
        return self._feature

    @property
    def ok(self) -> bool:
        return self[0] in ("turn", "go", "done")

    @property
    def failed(self) -> bool:
        return self[0] == "fail"

    def __repr__(self) -> str:
        if self[0] == "fail":
            return f"FrontDockDecision('fail', {self[1]!r})"
        return f"FrontDockDecision({self[0]!r}, {self[1]!r})"


# ==============================================================================
# SEGMENTACIÓN Y CLUSTERING LIGERO (LÓGICA PURA)
# ==============================================================================

@dataclass
class _LaserCluster:
    pts: List[Tuple[float, float]]
    center_x: float
    center_y: float
    span_x: float
    span_y: float
    diameter: float
    min_x: float
    min_y: float
    max_y: float


def _make_cluster(pts: List[Tuple[float, float]]) -> _LaserCluster:
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    cx = sum(xs) / len(xs)
    cy = sum(ys) / len(ys)
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    span_x = max_x - min_x
    span_y = max_y - min_y
    diameter = math.hypot(span_x, span_y)
    return _LaserCluster(
        pts=pts,
        center_x=cx,
        center_y=cy,
        span_x=span_x,
        span_y=span_y,
        diameter=diameter,
        min_x=min_x,
        min_y=min_y,
        max_y=max_y,
    )


def _cluster_points(pts: Sequence[Tuple[float, float]], break_dist: float, min_pts: int = 2) -> List[_LaserCluster]:
    if not pts:
        return []
    clusters: List[List[Tuple[float, float]]] = []
    cur: List[Tuple[float, float]] = []
    for p in pts:
        if not cur:
            cur.append(p)
            continue
        prev = cur[-1]
        if math.hypot(p[0] - prev[0], p[1] - prev[1]) <= break_dist:
            cur.append(p)
        else:
            if len(cur) >= min_pts:
                clusters.append(cur)
            cur = [p]
    if len(cur) >= min_pts:
        clusters.append(cur)
    return [_make_cluster(c) for c in clusters]


# ==============================================================================
# FUNCIÓN PRINCIPAL DE CONTROL: front_dock_step
# ==============================================================================

def front_dock_step(
    angles_or_scan: Union[Sequence[float], Tuple[Sequence[float], Sequence[float]]],
    ranges: Optional[Sequence[float]] = None,
    *,
    target: str = "auto",
    target_distance_m: Optional[float] = None,
    cfg: Optional[FrontDockConfig] = None,
) -> FrontDockDecision:
    """Calcula la acción inmediata para acercamiento fino frontal con LiDAR (lógica pura).

    Parámetros:
    - angles_or_scan: lista de ángulos (rad) o tupla (angles, ranges).
    - ranges: lista de distancias (m) si angles_or_scan es solo la lista de ángulos.
    - target: tipo de objetivo esperado ('auto', 'door', 'table', 'wall', 'surface').
    - target_distance_m: distancia final deseada (standoff) entre el frente del chasis y el obstáculo (m).
    - cfg: configuración opcional de tolerancias y geometría.

    Retorna FrontDockDecision ((action, value)):
    - ('turn', error_rad): gira para alinear el frente perpendicular al objetivo.
    - ('go', remaining_m): avanza en línea recta la distancia restante.
    - ('done', info): objetivo alcanzado dentro de tolerancias ('surface', 'table', 'door_open').
    - ('fail', motivo): fallo con motivo en {'door_closed', 'blocked', 'nothing_ahead', 'too_far', 'no_scan'}.
    """
    if cfg is None:
        cfg = FrontDockConfig()

    target_standoff = float(target_distance_m) if target_distance_m is not None else cfg.target_standoff_m

    # 1. Normalización de entradas
    if ranges is None:
        if isinstance(angles_or_scan, (tuple, list)) and len(angles_or_scan) == 2:
            angles = angles_or_scan[0]
            ranges_val = angles_or_scan[1]
        else:
            return FrontDockDecision("fail", "no_scan", reason="no_scan")
    else:
        angles = angles_or_scan  # type: ignore[assignment]
        ranges_val = ranges

    if angles is None or ranges_val is None or len(angles) == 0 or len(ranges_val) == 0 or len(angles) != len(ranges_val):
        return FrontDockDecision("fail", "no_scan", reason="no_scan")

    # 2. Filtrado de puntos válidos en el FOV frontal (+/- 60°)
    fov_half_rad = math.radians(60.0)
    valid_pts: List[Tuple[float, float]] = []
    closest_r = float("inf")

    for a, r in zip(angles, ranges_val):
        if not math.isfinite(r):
            continue
        if r < cfg.min_scan_range_m or r > cfg.max_scan_range_m:
            continue
        if abs(a) > fov_half_rad:
            continue
        x = float(r * math.cos(a))
        y = float(r * math.sin(a))
        valid_pts.append((x, y))
        if r < closest_r:
            closest_r = r

    if not valid_pts:
        return FrontDockDecision("fail", "nothing_ahead", reason="nothing_ahead")

    # Si todo retorno está más allá del rango de búsqueda
    if closest_r > cfg.max_dock_distance_m:
        return FrontDockDecision("fail", "too_far", reason="too_far")

    # 3. Delegación a mesa si target=='table' o si se detectan patas de mesa en modo 'auto'
    if target == "table":
        return _delegate_table_dock(angles, ranges_val, target_standoff, cfg)

    if target == "auto":
        # Chequear si table_dock detecta una mesa abierta por patas (haz a 38 cm)
        t_edge, _ = td.fit_table_edge(angles, ranges_val)
        if t_edge is not None and t_edge.kind == "legs":
            return _delegate_table_dock(angles, ranges_val, target_standoff, cfg)

    # 4. Segmentación en clusters para análisis de corredor y superficies
    clusters = _cluster_points(valid_pts, break_dist=cfg.cluster_break_dist_m, min_pts=cfg.min_cluster_points)
    if not clusters:
        return FrontDockDecision("fail", "nothing_ahead", reason="nothing_ahead")

    # 5. Detección de personas u obstáculos estrechos en el corredor de avance (|y| <= corridor_half_width_m)
    # Una persona de pie tiene diámetro 0.15-0.45 m.
    corridor_clusters: List[_LaserCluster] = []
    for c in clusters:
        # Se cruza o reside en el corredor central
        if abs(c.center_y) <= cfg.corridor_half_width_m or (c.min_y <= 0.0 <= c.max_y):
            corridor_clusters.append(c)

    # 6. Caso especial: TARGET == "DOOR" (puerta cerrada vs puerta abierta)
    if target == "door":
        return _process_door_dock(valid_pts, clusters, corridor_clusters, target_standoff, cfg)

    # 7. Caso general: SUPERFICIE CONTINUA / PARED (o target=='surface' / target=='wall' / target=='auto')
    return _process_surface_dock(valid_pts, clusters, corridor_clusters, target_standoff, cfg)


# ==============================================================================
# SUB-RUTINAS DE PROCESAMIENTO (LÓGICA PURA)
# ==============================================================================

def _delegate_table_dock(
    angles: Sequence[float],
    ranges: Sequence[float],
    target_standoff: float,
    cfg: FrontDockConfig,
) -> FrontDockDecision:
    """Envuelve y mapea las decisiones de table_dock_step sin duplicar código."""
    table_cfg = td.TableDockConfig(
        target_standoff_m=target_standoff,
        corridor_half_width_m=cfg.corridor_half_width_m,
        align_tol_rad=cfg.align_tol_rad,
        position_tol_m=cfg.position_tol_m,
        max_table_dist_m=cfg.max_dock_distance_m,
    )
    t_res = td.table_dock_step(angles, ranges, target_distance_m=target_standoff, cfg=table_cfg)

    action, val = t_res
    if action == "done":
        return FrontDockDecision("done", "table", info="table", feature="table")
    if action == "go":
        return FrontDockDecision("go", float(val), feature="table")
    if action == "turn":
        return FrontDockDecision("turn", float(val), feature="table")

    # Falla: mapear a los motivos normalizados de front_dock
    err_str = str(val).lower()
    if "persona" in err_str or "obstáculo" in err_str or "colisión" in err_str or "cerca" in err_str:
        return FrontDockDecision("fail", "blocked", reason="blocked", feature="table")
    if "demasiado lejos" in err_str or "lejos" in err_str:
        return FrontDockDecision("fail", "too_far", reason="too_far", feature="table")
    if "no ve borde" in err_str or "sin retornos" in err_str:
        return FrontDockDecision("fail", "nothing_ahead", reason="nothing_ahead", feature="table")
    return FrontDockDecision("fail", "blocked", reason="blocked", feature="table")


def _process_door_dock(
    valid_pts: List[Tuple[float, float]],
    clusters: List[_LaserCluster],
    corridor_clusters: List[_LaserCluster],
    target_standoff: float,
    cfg: FrontDockConfig,
) -> FrontDockDecision:
    """Evalúa si la puerta está abierta (hueco) o cerrada (superficie continua de ancho puerta)."""
    # Verificar si el corredor frontal de ancho de puerta (|y| <= min_door_width_m / 2.0) está libre (hueco)
    doorway_half_w = cfg.min_door_width_m / 2.0
    pts_in_doorway = [p for p in valid_pts if abs(p[1]) <= doorway_half_w and p[0] <= cfg.max_dock_distance_m]

    # Si no hay puntos dentro del vano de la puerta, pero hay jambas/paredes a los costados: PUERTA ABIERTA
    if len(pts_in_doorway) < 4:
        # Hueco confirmado: el corredor está expedito
        return FrontDockDecision("done", "door_open", info="door_open", feature="door_open")

    # Si hay puntos en el vano, chequear si es un obstáculo/persona estrecha en la puerta
    # o si es una superficie continua que cubre el ancho de la puerta
    pts_corridor = [p for p in valid_pts if abs(p[1]) <= cfg.corridor_half_width_m]
    if pts_corridor:
        min_x_corridor = min(p[0] for p in pts_corridor)
        # Buscar el cluster que contiene estos puntos cercanos
        blocking_clusters = [c for c in corridor_clusters if c.min_x <= min_x_corridor + 0.15]
        for bc in blocking_clusters:
            # Si el cluster es estrecho (persona u obstáculo), bloquea la puerta
            if bc.diameter < 0.40 and bc.span_y < 0.40:
                return FrontDockDecision("fail", "blocked", reason="blocked", feature="obstacle")

    # Ajuste de recta sobre los puntos del vano y los clusters centrales
    door_pts = [p for p in valid_pts if abs(p[1]) <= cfg.max_door_width_m / 2.0 + 0.10]
    if len(door_pts) < 6:
        return FrontDockDecision("done", "door_open", info="door_open", feature="door_open")

    norm, d_laser, rmse, inliers = td._fit_line_ransac(door_pts, thresh_m=0.03)
    if rmse > cfg.max_line_rmse_m:
        # No es una superficie plana continua
        return FrontDockDecision("fail", "blocked", reason="blocked", feature="door")

    # Coordenadas geométricas
    yaw_error = math.atan2(norm[1], norm[0])
    d_chassis = d_laser - cfg.chassis_front_offset_m
    remaining_m = d_chassis - target_standoff

    # ¿Está demasiado lejos?
    if d_laser > cfg.max_dock_distance_m:
        return FrontDockDecision("fail", "too_far", reason="too_far")

    # Puerta cerrada detectada:
    # Si approach_closed_door es False, o si ya llegó a la distancia segura de frenado:
    if not cfg.approach_closed_door or remaining_m <= cfg.position_tol_m:
        # Frena a la distancia segura sin empujar y devuelve 'door_closed'
        return FrontDockDecision("fail", "door_closed", reason="door_closed", feature="door_closed")

    # Si todavía está lejos y approach_closed_door es True: orienta y avanza hacia la distancia segura
    if abs(yaw_error) > cfg.align_tol_rad:
        return FrontDockDecision("turn", yaw_error, feature="door_closed")
    return FrontDockDecision("go", remaining_m, feature="door_closed")


def _process_surface_dock(
    valid_pts: List[Tuple[float, float]],
    clusters: List[_LaserCluster],
    corridor_clusters: List[_LaserCluster],
    target_standoff: float,
    cfg: FrontDockConfig,
) -> FrontDockDecision:
    """Evalúa acercamiento a pared o superficie continua plana con protección de corredor."""
    # Buscar cluster candidato a pared continua (span_y >= 0.50 m o diámetro ancho)
    surface_candidates: List[Tuple[_LaserCluster, Tuple[float, float], float, float, List[Tuple[float, float]]]] = []

    for c in clusters:
        if c.span_y >= 0.50 or c.diameter >= 0.50 or (c.min_y <= -0.20 and c.max_y >= 0.20):
            norm, d, rmse, inliers = td._fit_line_ransac(c.pts, thresh_m=0.03)
            yaw_err = math.atan2(norm[1], norm[0])
            if rmse <= cfg.max_line_rmse_m and abs(yaw_err) <= cfg.max_yaw_error_rad:
                surface_candidates.append((c, norm, d, rmse, inliers))

    if not surface_candidates:
        # Si no hay superficie continua pero hay clusters en el corredor, es un obstáculo
        if corridor_clusters:
            return FrontDockDecision("fail", "blocked", reason="blocked")
        return FrontDockDecision("fail", "nothing_ahead", reason="nothing_ahead")

    # Seleccionar la superficie más cercana
    surface_candidates.sort(key=lambda item: item[2])
    best_c, norm, d_laser, rmse, inliers = surface_candidates[0]

    # Verificar si hay una persona u obstáculo entre el robot y la pared en el corredor
    for c in clusters:
        if c is best_c:
            continue
        if len(c.pts) >= 4 and abs(c.center_y) <= cfg.corridor_half_width_m:
            # Distancia ortogonal al plano de avance
            d_obs = c.min_x
            if d_obs < d_laser - 0.15:
                return FrontDockDecision("fail", "blocked", reason="blocked")

    yaw_error = math.atan2(norm[1], norm[0])
    d_chassis = d_laser - cfg.chassis_front_offset_m
    remaining_m = d_chassis - target_standoff

    if d_laser > cfg.max_dock_distance_m:
        return FrontDockDecision("fail", "too_far", reason="too_far")

    # Prioridad 1: Alineación perpendicular con la normal de la superficie
    if abs(yaw_error) > cfg.align_tol_rad:
        return FrontDockDecision("turn", yaw_error, feature="surface")

    # Prioridad 2: Llegada a la distancia de seguridad
    if remaining_m <= cfg.position_tol_m:
        return FrontDockDecision("done", "surface", info="surface", feature="surface")

    # Prioridad 3: Avanzar derecho lo que resta
    return FrontDockDecision("go", remaining_m, feature="surface")


# ==============================================================================
# VALIDACIÓN DE CORREDOR TRASERO PARA UNDOCK (PROPUESTA PARA node.py:459)
# ==============================================================================

def check_rear_corridor_clear(
    angles: Sequence[float],
    ranges: Sequence[float],
    *,
    check_dist_m: float = 0.40,
    cfg: Optional[FrontDockConfig] = None,
) -> Tuple[bool, Optional[str]]:
    """Evalúa los cuadrantes laterales traseros del LiDAR (270° FOV) para undock seguro.

    El Hokuyo UST-05LX tiene un campo de 270° (-135° a +135°).
    Los rayos con |ángulo| >= 90° (cuadrantes 2 y 3) apuntan hacia atrás (x < 0).
    Permite validar que no haya personas u obstáculos pegados a los costados traseros
    antes de que node.py:459 ejecute retroceso con cmd_vel negativo.

    Retorna:
    - (True, None) si el corredor trasero medible está libre.
    - (False, motivo) si se detecta un obstáculo a menos de check_dist_m.
    """
    if cfg is None:
        cfg = FrontDockConfig()

    if len(angles) != len(ranges) or len(ranges) == 0:
        return False, "sin escaneo válido para verificar corredor trasero"

    # Distancia crítica hacia atrás considerando la longitud del chasis
    x_limit_m = -(check_dist_m + cfg.chassis_front_offset_m)

    for a, r in zip(angles, ranges):
        if not math.isfinite(r) or r < cfg.min_scan_range_m or r > cfg.max_scan_range_m:
            continue
        # Cuadrantes traseros visibles: 90° a 135° y -135° a -90°
        if abs(a) >= (math.pi / 2.0):
            x = r * math.cos(a)
            y = r * math.sin(a)
            # Dentro del corredor de retroceso
            if x_limit_m <= x <= 0.0 and abs(y) <= cfg.corridor_half_width_m:
                return False, f"obstáculo en cuadrante trasero a {abs(x):.2f} m detrás del láser"

    return True, None
