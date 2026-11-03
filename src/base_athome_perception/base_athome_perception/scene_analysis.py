#!/usr/bin/env python3
"""Analisis de la sala barriendo con el pan-tilt (sin rutina pregrabada).

En cada angulo de la cabeza se guardan personas (YOLO pose + nombre si la cara se
reconoce) y muebles para sentarse (YOLO: chair, couch, bench). Todo se expresa en
"rumbo respecto del robot" (grados, + = izquierda) para poder unir lo visto desde
varios angulos y explicarle al invitado hacia donde ir. Geometria pura, testeable.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

SEAT_CLASSES = {"chair": 1, "couch": 2, "bench": 2}  # capacidad tipica


@dataclass
class SeenObject:
    kind: str                  # "person" | "chair" | "couch" | "bench"
    bearing_deg: float         # respecto del frente del robot, + = izquierda
    width_deg: float
    seated: bool = False
    name: Optional[str] = None
    distance_m: Optional[float] = None
    occupants: int = 0
    views: int = 1
    extra: Dict = field(default_factory=dict)

    @property
    def capacity(self) -> int:
        return SEAT_CLASSES.get(self.kind, 0)

    @property
    def free(self) -> int:
        return max(0, self.capacity - self.occupants)


def pixel_bearing(u: float, head_pan_deg: float, fx: float, cx: float, pan_sign: float = 1.0) -> float:
    """Rumbo (grados, + izquierda) de la columna u vista con la cabeza en head_pan_deg."""
    return pan_sign * head_pan_deg - math.degrees(math.atan2(u - cx, fx))


def bbox_bearing_width(bbox: Sequence[float], head_pan_deg: float, fx: float, cx: float,
                       pan_sign: float = 1.0):
    b1 = pixel_bearing(bbox[0], head_pan_deg, fx, cx, pan_sign)
    b2 = pixel_bearing(bbox[2], head_pan_deg, fx, cx, pan_sign)
    return (b1 + b2) / 2.0, abs(b1 - b2)


def overlap_frac(inner: Sequence[float], outer: Sequence[float]) -> float:
    """Fraccion del area de `inner` que cae dentro de `outer` (cajas x1, y1, x2, y2)."""
    ix1, iy1 = max(inner[0], outer[0]), max(inner[1], outer[1])
    ix2, iy2 = min(inner[2], outer[2]), min(inner[3], outer[3])
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area = max(1e-6, (inner[2] - inner[0]) * (inner[3] - inner[1]))
    return inter / area


def occupants_in_image(seat_bbox: Sequence[float], person_bboxes: Sequence[Sequence[float]],
                       min_overlap: float = 0.3) -> int:
    """Personas cuya parte inferior (cadera-piernas) se superpone con el mueble."""
    n = 0
    for p in person_bboxes:
        lower = (p[0], p[1] + 0.5 * (p[3] - p[1]), p[2], p[3])
        if overlap_frac(lower, seat_bbox) >= min_overlap:
            n += 1
    return n


def scan_plan(pans=(-65.0, -43.0, -22.0, 0.0, 22.0, 43.0, 65.0), tilts=(-12.0, -28.0)):
    """Vistas (pan, tilt) del barrido de la sala en zigzag: una fila por inclinacion, alternando el
    sentido para no volver la cabeza al principio de cada fila. tilt negativo = mirar hacia abajo:
    la fila baja ve a las personas SENTADAS (la de -10 grados las dejaba fuera de cuadro)."""
    out = []
    for i, tilt in enumerate(tilts):
        row = list(pans) if i % 2 == 0 else list(reversed(pans))
        out.extend((float(p), float(tilt)) for p in row)
    return out


def head_steps(start, goal, speed_dps: float, rate_hz: float = 20.0):
    """Puntos intermedios (pan, tilt) para mover la cabeza a velocidad limitada (lineal, ambos ejes juntos)."""
    dp, dt = goal[0] - start[0], goal[1] - start[1]
    dist = max(abs(dp), abs(dt))
    n = max(1, int(math.ceil(dist / max(1e-3, speed_dps) * rate_hz)))
    return [(start[0] + dp * k / n, start[1] + dt * k / n) for k in range(1, n + 1)]


def merge_views(objs: List[SeenObject], tol_deg: float = 12.0) -> List[SeenObject]:
    """Une el mismo objeto visto desde dos angulos de la cabeza (mismo tipo, rumbo cercano)."""
    merged: List[SeenObject] = []
    for o in sorted(objs, key=lambda x: x.bearing_deg):
        m = next((x for x in merged if x.kind == o.kind
                  and abs(x.bearing_deg - o.bearing_deg) <= max(tol_deg, (x.width_deg + o.width_deg) / 4)), None)
        if m is None:
            merged.append(o)
            continue
        k = m.views
        m.bearing_deg = (m.bearing_deg * k + o.bearing_deg) / (k + 1)
        m.width_deg = max(m.width_deg, o.width_deg)
        m.occupants = max(m.occupants, o.occupants)
        m.seated = m.seated or o.seated
        m.name = m.name or o.name
        m.distance_m = m.distance_m if m.distance_m is not None else o.distance_m
        m.views = k + 1
    return merged


def choose_seat(objects: List[SeenObject]) -> Optional[SeenObject]:
    """Mueble con lugar libre; prefiere el mas cercano al frente del robot."""
    seats = [o for o in objects if o.capacity and o.free > 0]
    return min(seats, key=lambda o: abs(o.bearing_deg)) if seats else None


def direction_words(bearing_deg: float) -> str:
    if bearing_deg > 45:
        return "to your far left"
    if bearing_deg > 12:
        return "on your left"
    if bearing_deg < -45:
        return "to your far right"
    if bearing_deg < -12:
        return "on your right"
    return "in front of you"


def describe_scene(objects: List[SeenObject], guest_name: str, seat: Optional[SeenObject]) -> str:
    people = [o for o in objects if o.kind == "person"]
    parts = []
    if people:
        named = [p for p in people if p.name]
        if named:
            parts.append(" ".join(f"{p.name} is {'sitting' if p.seated else 'standing'} {direction_words(p.bearing_deg)}."
                                  for p in named))
        others = len(people) - len(named)
        if others:
            parts.append(f"I can see {others} more {'person' if others == 1 else 'people'} here.")
    else:
        parts.append("I do not see anyone sitting here yet.")
    if seat is not None:
        label = {"couch": "sofa"}.get(seat.kind, seat.kind)
        parts.append(f"{guest_name}, the {label} {direction_words(seat.bearing_deg)} is free. Please take a seat there.")
    else:
        parts.append(f"{guest_name}, I could not find a free seat. Please feel free to sit wherever you like.")
    return " ".join(parts)


def person_seated_at(bearing_deg: float, poses_bearings: Sequence[float], postures: Sequence[str],
                     tol_deg: float = 15.0) -> bool:
    """Hay una persona SENTADA cerca del rumbo del asiento."""
    return any(abs(b - bearing_deg) <= tol_deg and str(p).upper().startswith("SENTAD")
               for b, p in zip(poses_bearings, postures))


class GentleFaceTracker:
    """Seguimiento LEVE del rostro durante el registro del invitado (p. ej. invitado mas alto que el robot).

    Filtra el centro de la cara (EMA), ignora errores chicos (zona muerta), da pasos pequenos y limitados en
    frecuencia, y mantiene la cabeza en una ventana acotada: tilt [tilt_min, tilt_max] (permite mirar hacia
    arriba) y pan +-pan_max alrededor del frente.
    """

    def __init__(self, gain: float = 0.25, deadband_deg: float = 5.0, max_step_deg: float = 2.0,
                 min_interval_s: float = 0.3, alpha: float = 0.4, tilt_min: float = -10.0, tilt_max: float = 25.0,
                 pan_max: float = 25.0):
        self.gain, self.deadband, self.max_step = gain, deadband_deg, max_step_deg
        self.min_interval, self.alpha = min_interval_s, alpha
        self.tilt_min, self.tilt_max, self.pan_max = tilt_min, tilt_max, pan_max
        self._c = None
        self._last_t = -1e9

    def reset(self):
        self._c = None

    def update(self, face_cx, face_cy, img_w, img_h, fx, fy, pan_deg, tilt_deg, now):
        """Nuevo (pan, tilt) objetivo o None si no hay que moverse."""
        c = (float(face_cx), float(face_cy))
        self._c = c if self._c is None else (self._c[0] + self.alpha * (c[0] - self._c[0]),
                                             self._c[1] + self.alpha * (c[1] - self._c[1]))
        if now - self._last_t < self.min_interval:
            return None
        dpan, dtilt = face_tracking_step(self._c[0], self._c[1], img_w, img_h, fx, fy, gain=self.gain,
                                         deadband_deg=self.deadband, max_step_deg=self.max_step)
        if not dpan and not dtilt:
            return None
        new_pan = max(-self.pan_max, min(self.pan_max, pan_deg + dpan))
        new_tilt = max(self.tilt_min, min(self.tilt_max, tilt_deg + dtilt))
        if abs(new_pan - pan_deg) < 1e-6 and abs(new_tilt - tilt_deg) < 1e-6:
            return None
        self._last_t = now
        return new_pan, new_tilt


def face_tracking_step(face_cx: float, face_cy: float, img_w: int, img_h: int, fx: float, fy: float,
                       gain: float = 0.6, deadband_deg: float = 2.0, max_step_deg: float = 8.0):
    """Correccion (dpan, dtilt) en grados para llevar la cara al centro de la imagen.

    dpan > 0 gira a la izquierda (cara a la izquierda del centro); dtilt > 0 sube.
    """
    ex = -math.degrees(math.atan2(face_cx - img_w / 2.0, fx))
    ey = -math.degrees(math.atan2(face_cy - img_h / 2.0, fy))
    def step(e):
        if abs(e) < deadband_deg:
            return 0.0
        return max(-max_step_deg, min(max_step_deg, gain * e))
    return step(ex), step(ey)
