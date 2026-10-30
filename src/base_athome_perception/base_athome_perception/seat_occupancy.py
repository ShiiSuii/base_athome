#!/usr/bin/env python3
"""Ocupacion de asientos para el HRI Challenge (ofrecer un asiento libre).

Enfoque usado por varios equipos @Home (mapa semantico + deteccion de personas):
  1. el robot mira cada asiento del mapa semantico con la cabeza,
  2. detecta personas (YOLO pose) y estima su posicion en el piso con la
     profundidad alineada de la RealSense, la pose de AMCL y el angulo de la cabeza,
  3. un asiento esta ocupado por las personas que caen dentro de su poligono
     (ampliado un margen); se ofrece el que tenga capacidad libre.
Si no hay profundidad o pose del robot se usa un respaldo en imagen: al mirar el
asiento, una persona sentada cerca del centro de la imagen lo ocupa.
Todo aca es geometria pura (sin ROS) para poder testearlo.
"""
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float]


def depth_at_bbox(depth_m, bbox: Sequence[int]) -> Optional[float]:
    """Mediana de profundidad (m) en la zona del torso de la caja (x1, y1, x2, y2)."""
    import numpy as np
    x1, y1, x2, y2 = (int(v) for v in bbox[:4])
    h, w = depth_m.shape[:2]
    cx1, cx2 = int(x1 + 0.35 * (x2 - x1)), int(x1 + 0.65 * (x2 - x1))
    cy1, cy2 = int(y1 + 0.25 * (y2 - y1)), int(y1 + 0.55 * (y2 - y1))
    patch = depth_m[max(0, cy1):min(h, cy2), max(0, cx1):min(w, cx2)]
    vals = patch[np.isfinite(patch) & (patch > 0.2) & (patch < 8.0)]
    if vals.size < 10:
        return None
    return float(np.median(vals))


def person_map_position(u: float, depth_z: float, fx: float, cx: float,
                        robot_pose: Tuple[float, float, float], head_pan_rad: float,
                        pan_sign: float = 1.0) -> Point:
    """Posicion (x, y) en el mapa de una persona vista en la columna u de la imagen.

    El eje x de la imagen crece a la derecha (yaw negativo). pan positivo = izquierda
    (REP-103) si pan_sign = 1. Se ignora el offset camara-base (~0.15 m) y el tilt.
    """
    alpha = math.atan2(u - cx, fx)
    rng = depth_z / max(math.cos(alpha), 1e-3)
    rx, ry, ryaw = robot_pose
    heading = ryaw + pan_sign * head_pan_rad - alpha
    return rx + rng * math.cos(heading), ry + rng * math.sin(heading)


def point_in_polygon(p: Point, poly: Sequence[Sequence[float]]) -> bool:
    x, y = p
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][0], poly[i][1]
        x2, y2 = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-9) + x1:
            inside = not inside
    return inside


def _dist_to_polygon(p: Point, poly: Sequence[Sequence[float]]) -> float:
    if point_in_polygon(p, poly):
        return 0.0
    best = float("inf")
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i][0], poly[i][1]
        bx, by = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / ((dx * dx + dy * dy) or 1e-9)))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


def seat_contains(seat: Dict, p: Point, margin_m: float = 0.35) -> bool:
    poly = seat.get("polygon")
    if poly and len(poly) >= 3:
        return _dist_to_polygon(p, poly) <= margin_m
    if "x" in seat and "y" in seat:
        return math.hypot(p[0] - float(seat["x"]), p[1] - float(seat["y"])) <= 0.5 + margin_m
    return False


def seat_occupancy(seats: Dict[str, Dict], persons_xy: Iterable[Point], margin_m: float = 0.35) -> Dict[str, int]:
    """Cantidad de personas dentro de cada asiento (cada persona cuenta en un solo asiento: el mas cercano)."""
    occ = {k: 0 for k in seats}
    for p in persons_xy:
        hits = [(k, _dist_to_polygon(p, s["polygon"]) if s.get("polygon") else
                 math.hypot(p[0] - float(s.get("x", 1e9)), p[1] - float(s.get("y", 1e9))))
                for k, s in seats.items() if seat_contains(s, p, margin_m)]
        if hits:
            occ[min(hits, key=lambda kv: kv[1])[0]] += 1
    return occ


def merge_nearby(points: Iterable[Point], radius_m: float = 0.4) -> List[Point]:
    """Une detecciones repetidas de la misma persona (vista desde varios angulos)."""
    merged: List[List[float]] = []
    for p in points:
        for m in merged:
            if math.hypot(p[0] - m[0] / m[2], p[1] - m[1] / m[2]) <= radius_m:
                m[0] += p[0]
                m[1] += p[1]
                m[2] += 1
                break
        else:
            merged.append([p[0], p[1], 1])
    return [(m[0] / m[2], m[1] / m[2]) for m in merged]


def choose_free_seat(seats: Dict[str, Dict], occupancy: Dict[str, int],
                     reserved: Optional[Dict[str, int]] = None) -> Optional[str]:
    """Asiento con lugar libre (capacidad - ocupantes - reservados). Prefiere el mas libre."""
    reserved = reserved or {}
    best, best_free = None, 0
    for k, s in seats.items():
        free = int(s.get("capacity", 1)) - occupancy.get(k, 0) - reserved.get(k, 0)
        if free > best_free:
            best, best_free = k, free
    return best


def image_fallback_occupied(person_bboxes: Iterable[Sequence[int]], postures: Iterable[str],
                            img_w: int, center_frac: float = 0.4) -> int:
    """Sin profundidad: personas sentadas cerca del centro de la imagen al mirar el asiento."""
    n = 0
    lo, hi = img_w * (0.5 - center_frac / 2), img_w * (0.5 + center_frac / 2)
    for bbox, posture in zip(person_bboxes, postures):
        cx = (bbox[0] + bbox[2]) / 2.0
        if lo <= cx <= hi and str(posture).lower() in ("sitting", "seated", "sentado"):
            n += 1
    return n
