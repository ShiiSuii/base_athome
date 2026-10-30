"""Deteccion de puerta abierta a partir de un LaserScan (logica pura, sin ROS).

Reglamento RoboCup@Home 2026.2 §3.6.8: con el robot fuera de la arena, la prueba
empieza cuando el referee abre la puerta. El robot espera frente a la puerta cerrada;
la puerta se considera abierta cuando la distancia frontal crece claramente respecto
de la referencia medida con la puerta cerrada, de forma sostenida.
"""
from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence


@dataclass
class DoorDetectorConfig:
    window_deg: float = 20.0          # ancho total de la ventana frontal
    center_deg: float = 0.0           # 0 = frente del laser
    min_valid_fraction: float = 0.3   # fraccion minima de rayos validos en la ventana
    calibration_scans: int = 10       # scans para medir la puerta cerrada
    max_closed_distance_m: float = 1.5  # si la "puerta cerrada" esta mas lejos, no hay puerta
    open_margin_m: float = 0.6        # aumento sobre la referencia para declarar abierta
    open_scans_required: int = 5      # scans consecutivos (40 Hz Hokuyo -> ~0.125 s)


def window_distance(ranges: Sequence[float], angle_min: float, angle_increment: float,
                    range_min: float, range_max: float, cfg: DoorDetectorConfig) -> Optional[float]:
    """Mediana de los rangos validos dentro de la ventana frontal; None si hay pocos validos."""
    if angle_increment == 0 or not ranges:
        return None
    half = math.radians(cfg.window_deg) / 2.0
    center = math.radians(cfg.center_deg)
    in_window = []
    valid = []
    for i, r in enumerate(ranges):
        a = angle_min + i * angle_increment
        if abs(math.atan2(math.sin(a - center), math.cos(a - center))) > half:
            continue
        in_window.append(r)
        # inf = sin retorno dentro del alcance: para una puerta abierta cuenta como "lejos"
        if math.isinf(r) and r > 0:
            valid.append(range_max)
        elif math.isfinite(r) and range_min <= r <= range_max:
            valid.append(r)
    if not in_window or len(valid) < cfg.min_valid_fraction * len(in_window):
        return None
    valid.sort()
    mid = len(valid) // 2
    return valid[mid] if len(valid) % 2 else 0.5 * (valid[mid - 1] + valid[mid])


@dataclass
class DoorOpenDetector:
    """Maquina de estados: CALIBRATING -> WAITING_CLOSED -> OPEN (o NO_DOOR)."""

    cfg: DoorDetectorConfig = field(default_factory=DoorDetectorConfig)
    state: str = 'CALIBRATING'
    reference_m: Optional[float] = None
    _calib: List[float] = field(default_factory=list)
    _open_count: int = 0

    def update(self, distance: Optional[float]) -> str:
        if self.state in ('OPEN', 'NO_DOOR') or distance is None:
            return self.state
        if self.state == 'CALIBRATING':
            self._calib.append(distance)
            if len(self._calib) >= self.cfg.calibration_scans:
                self._calib.sort()
                self.reference_m = self._calib[len(self._calib) // 2]
                self.state = ('WAITING_CLOSED' if self.reference_m <= self.cfg.max_closed_distance_m
                              else 'NO_DOOR')
            return self.state
        if distance >= self.reference_m + self.cfg.open_margin_m:
            self._open_count += 1
            if self._open_count >= self.cfg.open_scans_required:
                self.state = 'OPEN'
        else:
            self._open_count = 0
        return self.state
