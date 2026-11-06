"""Geometria de "senalar una bolsa y que el robot la agarre". Puro: lo usan el detector real, el simulador y el ejecutor.

Marco map, metros. Con la pose FLOOR_PREGRASP_DEG y el elevador arriba, el punto de agarre queda en base_footprint
(REACH_X, REACH_Y, REF_OBJECT_Z). Para un objeto bajo se baja el elevador lo que le falta al objeto.

Orden para no golpear nada: frenar lejos -> brazo a la pose de piso -> avanzar con el elevador arriba (la pinza pasa
por encima de la bolsa) -> bajar -> agarrar. El brazo nunca baja colgando sobre la base.
"""
import dataclasses
import math
import os
import re
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

Point3 = Sequence[float]


def _robot_properties() -> dict:
    """Numeros de urdf/xacro/properties.xacro. Vacio si el paquete no esta instalado (quedan los valores por defecto)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("hoverboard_base"), "urdf", "xacro", "properties.xacro")
        with open(path, encoding="utf-8") as f:
            found = re.findall(r'<xacro:property\s+name="(\w+)"\s+value="(-?[0-9.]+)"', f.read())
        return {k: float(v) for k, v in found}
    except Exception:  # noqa: BLE001
        return {}


_XACRO = _robot_properties()

# Buscada con piper_kinematics dentro de los limites del firmware: pinza 24 grados del vertical, inclinada hacia afuera,
# y a 17 cm de una bolsa de 30 cm con el elevador abajo (tope de avance real: 26 cm).
FLOOR_PREGRASP_DEG = [-15.4, 161.1, -112.4, 83.9, 78.0, -50.9]
REACH_X, REACH_Y = 0.58, 0.12       # objeto respecto de la base en esa pose, 12 cm por delante de la pinza
GROUND_TO_BASE_LINK_M = 0.06        # base.xacro. Las alturas de objetos se miden desde el PISO; la cinematica, desde base_link
REF_OBJECT_Z = 0.933                # altura desde el piso del objeto para ese agarre con torso_lift_joint = 0
CHASSIS_FRONT_M = _XACRO.get("chassis_length", 0.48) / 2.0    # la base sale del modelo del robot: un solo lugar
LASER_X_M = _XACRO.get("laser_offset_x", 0.22)
MIN_BASE_CLEARANCE_M = 0.05         # margen minimo aceptado entre el chasis y la bolsa
DEFAULT_BAG_RADIUS_M = 0.20
GRASP_GAP_M = 0.12                  # el objeto, por delante de la pinza en la pose de piso
STROKE_M = _XACRO.get("torso_switch_stroke", 0.60)     # carrera entre sensores
STROKE_MARGIN_M = 0.02              # no llegar a tocar el sensor en uso normal
DOCK_DISTANCE_M = 0.40              # tramo final en linea recta y lento: Nav2 no se acerca mas (robot_radius 0.44)
MIN_POINT_ANGLE_DEG = 25.0          # brazo separado del cuerpo: si no, esta colgando y no es un gesto.
                                    # Una persona con el hombro a 1.4 m apunta asi a una bolsa desde 0.65 m; con 35 ya no
                                    # leia el gesto hacia una bolsa a menos de 1 m (simulacion, escenario "lejos")
MAX_POINT_RANGE_M = 6.0
BAG_GRASP_BELOW_TOP_M = 0.04        # los dedos toman el nudo o el asa, no el aire sobre la bolsa
BAG_GRASP_MIN_Z_M = 0.03            # nunca mas cerca del piso que esto
DOCK_POSITION_TOL_M = 0.01
DOCK_ALIGN_TOL_RAD = math.radians(1.0)
# La pose enseñada deja la bolsa justo en el borde de lo que alcanza el brazo con esa orientacion (con la pose del 05/11
# a z=0.20: x=0.55 da IK, x=0.57 casi no y x=0.60 nunca). Parar un poco mas cerca da margen para corregir.
REACH_MARGIN_M = 0.04
DOCK_MAX_TRAVEL_M = 0.80            # mas que esto en el tramo final: algo anda mal con la medicion
BAG_REFINE_BACK_M = 0.30            # mas atras de donde empieza el tramo final: la cabeza llega a ver la bolsa entera
MAX_BAG_FROM_RAY_M = 1.0            # bolsa aceptada alrededor del punto senalado
PLACE_TORSO_M = -0.45               # altura para soltar la bolsa en el piso
PERSON_NEAR_M = 2.2                 # mas cerca de esto ya se ve bien el gesto: no hace falta moverse
PERSON_STOP_M = 1.5                 # distancia para hablar y ver el cuerpo entero con la RealSense


@dataclass(frozen=True)
class FloorGrasp:
    """Donde queda el objeto respecto de la base con la pose de piso y el elevador arriba."""
    reach_x: float = REACH_X
    reach_y: float = REACH_Y
    ref_z: float = REF_OBJECT_Z


def floor_grasp_from_tcp(tcp_in_base: Sequence[float], approach: Sequence[float], gap_m: float = GRASP_GAP_M) -> FloorGrasp:
    """A partir de la pose ensenada: el objeto va `gap_m` por delante de la pinza."""
    return FloorGrasp(float(tcp_in_base[0]) + gap_m * float(approach[0]), float(tcp_in_base[1]) + gap_m * float(approach[1]),
                      float(tcp_in_base[2]) + gap_m * float(approach[2]) + GROUND_TO_BASE_LINK_M)


def grasp_advance(tcp_in_base: Sequence[float], approach: Sequence[float], z_object: float, grasp: FloorGrasp,
                  back_m: float = 0.02) -> float:
    """Cuanto tiene que avanzar la pinza desde la pose de piso, con el elevador ya abajo, hasta un objeto a esa altura."""
    torso = torso_for_object(z_object, grasp=grasp)
    center = [float(tcp_in_base[i]) - back_m * float(approach[i]) for i in range(3)]
    center[2] += torso
    obj = (grasp.reach_x, grasp.reach_y, float(z_object) - GROUND_TO_BASE_LINK_M)
    return math.sqrt(sum((obj[i] - center[i]) ** 2 for i in range(3)))


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


def grasp_height(top_z: float) -> float:
    """Altura de agarre (desde el piso) para una bolsa cuya parte mas alta esta en top_z."""
    return max(BAG_GRASP_MIN_Z_M, float(top_z) - BAG_GRASP_BELOW_TOP_M)


def stand_pose(bag_xy: Sequence[float], from_xy: Sequence[float], extra_back_m: float = 0.0,
               grasp: FloorGrasp = FloorGrasp()) -> Tuple[float, float, float]:
    """Pose de la base (x, y, yaw) que deja la bolsa en el punto de agarre, llegando desde `from_xy`.
    `extra_back_m` la retrasa sobre el eje de avance: desde ahi basta avanzar derecho."""
    heading = math.atan2(float(bag_xy[1]) - float(from_xy[1]), float(bag_xy[0]) - float(from_xy[0]))
    yaw = heading - math.atan2(grasp.reach_y, grasp.reach_x)
    c, s = math.cos(yaw), math.sin(yaw)
    rx, ry = grasp.reach_x + extra_back_m, grasp.reach_y
    return float(bag_xy[0]) - (c * rx - s * ry), float(bag_xy[1]) - (s * rx + c * ry), yaw


def torso_for_object(z_object: float, stroke_m: float = STROKE_M, grasp: FloorGrasp = FloorGrasp()) -> float:
    """torso_lift_joint (0 arriba, negativo abajo) para agarrar a esa altura, dentro de la carrera."""
    low = -(float(stroke_m) - STROKE_MARGIN_M)
    return max(low, min(0.0, float(z_object) - grasp.ref_z))


def object_in_base(point_map: Point3, robot_xy_yaw: Sequence[float]) -> Tuple[float, float, float]:
    """Un punto del mapa en el marco de la base, sabiendo la pose del robot."""
    dx, dy = float(point_map[0]) - float(robot_xy_yaw[0]), float(point_map[1]) - float(robot_xy_yaw[1])
    c, s = math.cos(float(robot_xy_yaw[2])), math.sin(float(robot_xy_yaw[2]))
    return c * dx + s * dy, -s * dx + c * dy, float(point_map[2])


def person_stand_pose(person_xy: Sequence[float], robot_xy: Sequence[float], near_m: float = PERSON_NEAR_M,
                      stop_m: float = PERSON_STOP_M) -> Optional[Tuple[float, float, float]]:
    """Donde pararse para ver bien el gesto: sobre la linea hacia la persona, mirandola. None si ya esta cerca."""
    dx, dy = float(person_xy[0]) - float(robot_xy[0]), float(person_xy[1]) - float(robot_xy[1])
    dist = math.hypot(dx, dy)
    if dist <= near_m:
        return None
    return float(person_xy[0]) - stop_m * dx / dist, float(person_xy[1]) - stop_m * dy / dist, math.atan2(dy, dx)


def base_clearance(bag_radius_m: float = DEFAULT_BAG_RADIUS_M, grasp: FloorGrasp = FloorGrasp()) -> float:
    """Espacio libre entre el frente del chasis y el borde de la bolsa, ya en la pose de agarre."""
    return grasp.reach_x - float(bag_radius_m) - CHASSIS_FRONT_M


def dock_stop_range(margin_m: float = MIN_BASE_CLEARANCE_M) -> float:
    """Distancia del lidar a la que el frente del chasis queda a `margin_m` de lo que tenga adelante."""
    return CHASSIS_FRONT_M - LASER_X_M + margin_m


def with_reach_margin(grasp: FloorGrasp, margin_m: float = REACH_MARGIN_M) -> FloorGrasp:
    """El punto donde conviene dejar la bolsa: mas cerca que el limite de alcance de la pose, sobre el mismo rayo."""
    k = (grasp.reach_x - float(margin_m)) / grasp.reach_x
    return dataclasses.replace(grasp, reach_x=grasp.reach_x * k, reach_y=grasp.reach_y * k)


def correction_phrase(reason: str) -> Optional[str]:
    """Que decirle a la persona segun por que no se pudo leer el gesto o ver la bolsa. None si no hay nada que pueda corregir."""
    text = reason.lower()
    for key, phrase in (("falta cuerpo", "Please step back so I can see your whole arm."),
                        ("sin gesto", "Please stretch your arm and point at the bag."),
                        ("no apunta al piso", "Please point down at the bag on the floor."),
                        ("no ve personas", "I cannot see you. Please stand in front of me."),
                        ("no vi a nadie", "I cannot see you. Please stand in front of me."),
                        ("bolsa", "I do not see a bag there. Please point again, closer to the bag.")):
        if key in text:
            return phrase
    return None


def dock_step(bag_in_base: Sequence[float], grasp: FloorGrasp = FloorGrasp()) -> Tuple[str, float]:
    """Que hacer en el tramo final segun donde esta la bolsa AHORA: ('turn', error en rad), ('go', metros que faltan) o
    ('done', 0). Nav2 llega con 18 cm de tolerancia: avanzar una distancia fija deja la bolsa fuera de alcance.
    Primero alinea, asi al avanzar derecho la bolsa cae en el punto de agarre."""
    x, y = float(bag_in_base[0]), float(bag_in_base[1])
    remaining = x - grasp.reach_x
    if remaining <= DOCK_POSITION_TOL_M:
        return "done", 0.0
    error = math.atan2(y - grasp.reach_y * x / grasp.reach_x, x)
    if abs(error) > DOCK_ALIGN_TOL_RAD:
        return "turn", error
    return "go", remaining


def dock_verdict(travelled_m: float, front_range_m: Optional[float], goal_m: float = DOCK_DISTANCE_M,
                 stop_range_m: Optional[float] = None) -> str:
    """Decision del tramo final: 'go', 'done' o 'blocked' (algo mas cerca de lo esperable frente al lidar).
    El haz esta a 38 cm del piso: frena ante lo alto, pero NO ve una bolsa mas baja. Con la bolsa, el margen lo da
    la distancia medida por la cabeza."""
    if float(travelled_m) >= float(goal_m):
        return "done"
    stop = dock_stop_range() if stop_range_m is None else float(stop_range_m)
    if front_range_m is not None and float(front_range_m) < stop:
        return "blocked"
    return "go"
