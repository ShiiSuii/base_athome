"""Geometria de "senalar una bolsa y que el robot la agarre". Puro: lo usan el detector real, el simulador y el ejecutor.

Marco map, metros. Con la pose FLOOR_PREGRASP_DEG y el elevador arriba, el punto de agarre queda en base_footprint
(REACH_X, REACH_Y, REF_OBJECT_Z). Para un objeto bajo se baja el elevador lo que le falta al objeto.

Orden para no golpear nada: frenar lejos -> brazo a la pose de piso -> avanzar con el elevador arriba (la pinza pasa
por encima de la bolsa) -> bajar -> agarrar. El brazo nunca baja colgando sobre la base.
"""
import math
from typing import List, Optional, Sequence, Tuple

Point3 = Sequence[float]

# Buscada con piper_kinematics dentro de los limites del firmware: pinza 30 grados del vertical, inclinada hacia afuera.
FLOOR_PREGRASP_DEG = [-25.3, 173.0, -130.4, 77.7, 80.0, -135.1]
REACH_X, REACH_Y = 0.60, 0.064      # objeto respecto de la base en esa pose, 12 cm por delante de la pinza
REF_OBJECT_Z = 0.977                # altura del objeto para ese mismo agarre con torso_lift_joint = 0
CHASSIS_FRONT_M = 0.24              # xacro: chassis_length / 2
LASER_X_M = 0.22                    # xacro: laser_offset_x
MIN_BASE_CLEARANCE_M = 0.05         # margen minimo aceptado entre el chasis y la bolsa
DEFAULT_BAG_RADIUS_M = 0.20
STROKE_M = 0.60                     # carrera entre sensores (xacro: torso_switch_stroke)
STROKE_MARGIN_M = 0.02              # no llegar a tocar el sensor en uso normal
DOCK_DISTANCE_M = 0.40              # tramo final en linea recta y lento: Nav2 no se acerca mas (robot_radius 0.44)
MIN_POINT_ANGLE_DEG = 35.0          # brazo separado del cuerpo: si no, esta colgando y no es un gesto
MAX_POINT_RANGE_M = 6.0
MAX_BAG_FROM_RAY_M = 1.0            # bolsa aceptada alrededor del punto senalado
PLACE_TORSO_M = -0.45               # altura para soltar la bolsa en el piso


def _sub(a: Point3, b: Point3) -> List[float]:
    return [float(a[i]) - float(b[i]) for i in range(3)]


def is_pointing(shoulder: Point3, wrist: Point3, hip: Point3, min_angle_deg: float = MIN_POINT_ANGLE_DEG) -> bool:
    """El brazo se separa del torso lo suficiente como para ser un gesto."""
    arm, down = _sub(wrist, shoulder), _sub(hip, shoulder)
    na, nd = math.sqrt(sum(v * v for v in arm)), math.sqrt(sum(v * v for v in down))
    if na < 0.15 or nd < 0.10:
        return False
    cosang = sum(a * d for a, d in zip(arm, down)) / (na * nd)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosang)))) >= min_angle_deg


def pointed_floor_point(shoulder: Point3, wrist: Point3, floor_z: float = 0.0,
                        max_range_m: float = MAX_POINT_RANGE_M) -> Optional[Tuple[float, float]]:
    """Donde corta el piso la linea hombro -> muneca. None si apunta hacia arriba, horizontal o demasiado lejos."""
    d = _sub(wrist, shoulder)
    if d[2] >= -1e-3:
        return None
    t = (float(floor_z) - float(wrist[2])) / d[2]
    if t < 0:
        return None
    x, y = float(wrist[0]) + t * d[0], float(wrist[1]) + t * d[1]
    if math.hypot(x - float(wrist[0]), y - float(wrist[1])) > max_range_m:
        return None
    return x, y


def choose_bag(target_xy: Sequence[float], bags: Sequence[Point3],
               max_distance_m: float = MAX_BAG_FROM_RAY_M) -> Optional[Point3]:
    """La bolsa mas cercana al punto senalado, si hay una razonablemente cerca."""
    best, best_d = None, float(max_distance_m)
    for b in bags:
        d = math.hypot(float(b[0]) - float(target_xy[0]), float(b[1]) - float(target_xy[1]))
        if d <= best_d:
            best, best_d = b, d
    return best


def stand_pose(bag_xy: Sequence[float], from_xy: Sequence[float],
               extra_back_m: float = 0.0) -> Tuple[float, float, float]:
    """Pose de la base (x, y, yaw) que deja la bolsa en el punto de agarre, llegando desde `from_xy`.
    `extra_back_m` la retrasa sobre el eje de avance: desde ahi basta avanzar derecho."""
    heading = math.atan2(float(bag_xy[1]) - float(from_xy[1]), float(bag_xy[0]) - float(from_xy[0]))
    yaw = heading - math.atan2(REACH_Y, REACH_X)
    c, s = math.cos(yaw), math.sin(yaw)
    rx, ry = REACH_X + extra_back_m, REACH_Y
    return float(bag_xy[0]) - (c * rx - s * ry), float(bag_xy[1]) - (s * rx + c * ry), yaw


def torso_for_object(z_object: float, stroke_m: float = STROKE_M) -> float:
    """torso_lift_joint (0 arriba, negativo abajo) para agarrar a esa altura, dentro de la carrera."""
    low = -(float(stroke_m) - STROKE_MARGIN_M)
    return max(low, min(0.0, float(z_object) - REF_OBJECT_Z))


def object_in_base(point_map: Point3, robot_xy_yaw: Sequence[float]) -> Tuple[float, float, float]:
    """Un punto del mapa en el marco de la base, sabiendo la pose del robot."""
    dx, dy = float(point_map[0]) - float(robot_xy_yaw[0]), float(point_map[1]) - float(robot_xy_yaw[1])
    c, s = math.cos(float(robot_xy_yaw[2])), math.sin(float(robot_xy_yaw[2]))
    return c * dx + s * dy, -s * dx + c * dy, float(point_map[2])


def base_clearance(bag_radius_m: float = DEFAULT_BAG_RADIUS_M) -> float:
    """Espacio libre entre el frente del chasis y el borde de la bolsa, ya en la pose de agarre."""
    return REACH_X - float(bag_radius_m) - CHASSIS_FRONT_M


def dock_stop_range(margin_m: float = MIN_BASE_CLEARANCE_M) -> float:
    """Distancia del lidar a la que el frente del chasis queda a `margin_m` de lo que tenga adelante."""
    return CHASSIS_FRONT_M - LASER_X_M + margin_m


def dock_verdict(travelled_m: float, front_range_m: Optional[float], goal_m: float = DOCK_DISTANCE_M,
                 stop_range_m: Optional[float] = None) -> str:
    """Decision del tramo final: 'go', 'done' o 'blocked' (algo mas cerca de lo esperable frente al lidar)."""
    if float(travelled_m) >= float(goal_m):
        return "done"
    stop = dock_stop_range() if stop_range_m is None else float(stop_range_m)
    if front_range_m is not None and float(front_range_m) < stop:
        return "blocked"
    return "go"
