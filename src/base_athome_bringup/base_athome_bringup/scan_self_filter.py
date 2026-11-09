"""Autofiltro 2D del chasis en base_link, independiente de ROS.

El Hokuyo esta en (x=0.22, y=0) y sin giro respecto de base_link. La caja
incluye los retornos propios medidos en x=0.0..0.08, y=0.24..0.30, sin
extenderse por delante del chasis (x=0.26). No es un filtro por distancia:
un obstaculo cercano que quede fuera de la caja se conserva.
"""

import math
from collections.abc import Sequence


def filter_scan_ranges(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    *,
    laser_x: float = 0.22,
    laser_y: float = 0.0,
    laser_yaw: float = 0.0,
    box_min_x: float = -0.30,
    box_max_x: float = 0.26,
    box_min_y: float = -0.36,
    box_max_y: float = 0.36,
    max_abs_angle: float = math.pi,
) -> list[float]:
    """Devuelve una copia con +inf en los puntos dentro de la caja inclusiva.

    Angulos en radianes, distancias y traslacion en metros. La pose XY/yaw
    describe el laser en base_link; se supone un barrido horizontal. NaN,
    infinitos y rangos no positivos se conservan. No modifica la entrada.
    """
    geometry = (
        angle_min, angle_increment, laser_x, laser_y, laser_yaw,
        box_min_x, box_max_x, box_min_y, box_max_y,
    )
    if not all(math.isfinite(value) for value in geometry):
        raise ValueError("La geometria del laser y la caja debe ser finita")
    if box_min_x >= box_max_x or box_min_y >= box_max_y:
        raise ValueError("Cada minimo de la caja debe ser menor que su maximo")

    filtered = list(ranges)
    for index, distance in enumerate(ranges):
        if not math.isfinite(distance) or distance <= 0.0:
            continue
        beam = angle_min + index * angle_increment
        if abs(beam) > max_abs_angle:     # solo hacia adelante: los costados y atras ven el propio robot
            filtered[index] = math.inf
            continue
        angle = beam + laser_yaw
        x = laser_x + distance * math.cos(angle)
        y = laser_y + distance * math.sin(angle)
        if box_min_x <= x <= box_max_x and box_min_y <= y <= box_max_y:
            filtered[index] = math.inf
    return filtered
