#!/usr/bin/env python3
"""Geometria de la camara de muneca (DaBai, solo RGB) para la manipulacion.

Problema que resuelve (2026-10-30): la distancia al objeto se estimaba con el ancho de
la caja de YOLO (Z = f * D / w) y luego se restaba una constante fija de 27 cm
("lente -> centro de los dedos"). Con la camara actual la estimacion daba ~25 cm, la
resta dejaba 0 y el objeto aparecia DENTRO de la pinza en RViz, aunque con un metro
media 24 cm desde el centro de la pinza.

Modelo nuevo, calibrable con una cinta metrica:
    avance = a * Z_est + b
  - Z_est: distancia estimada por tamano (incluye el sesgo de la caja de YOLO).
  - avance: cuanto debe avanzar el CENTRO DE LA PINZA (entre las yemas) para que el
    centro del objeto quede entre los dedos.
  - 1 medicion -> se ajusta b (a = 1); 2 o mas -> ajuste lineal por minimos cuadrados.
Todo puro (sin ROS) y testeable.
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class WristCalib:
    a: float = 1.0
    b: float = 0.0235               # ajustado con la medicion del 2026-10-30 (24 cm a la cara del objeto)
    focal_px: float = 485.0
    cx_px: float = 320.0
    cy_px: float = 240.0
    object_diameter_m: float = 0.075
    grasp_center_back_m: float = 0.02   # centro de la pinza = TCP (yemas) - 2 cm sobre el eje de avance
    camera_above_axis_m: float = 0.0    # altura efectiva de la lente sobre el eje de la pinza (medir; la camara
                                        # esta arriba pero inclinada hacia abajo, el efecto neto ~0)
    samples: List[Tuple[float, float]] = field(default_factory=list)  # (Z_est, avance_real)

    @classmethod
    def from_dict(cls, d: Optional[Dict]) -> "WristCalib":
        c = cls()
        for k, v in (d or {}).items():
            if k == "samples":
                c.samples = [(float(z), float(r)) for z, r in v]
            elif hasattr(c, k):
                setattr(c, k, float(v))
        return c

    def to_dict(self) -> Dict:
        d = {k: round(float(getattr(self, k)), 5) for k in
             ("a", "b", "focal_px", "cx_px", "cy_px", "object_diameter_m", "grasp_center_back_m", "camera_above_axis_m")}
        d["samples"] = [[round(z, 4), round(r, 4)] for z, r in self.samples]
        return d


def size_distance(width_px: float, calib: WristCalib) -> float:
    """Z estimada lente-objeto por tamano aparente."""
    return calib.focal_px * calib.object_diameter_m / max(1.0, float(width_px))


def advance_needed(z_est: float, calib: WristCalib) -> float:
    """Cuanto debe avanzar el centro de la pinza hasta el centro del objeto (m)."""
    return max(0.0, calib.a * z_est + calib.b)


def add_measurement(calib: WristCalib, z_est: float, measured_to_face_m: float) -> WristCalib:
    """Agrega una medicion con cinta (centro de la pinza -> CARA del objeto) y re-ajusta a, b."""
    real_advance = measured_to_face_m + calib.object_diameter_m / 2.0
    calib.samples.append((float(z_est), float(real_advance)))
    zs = np.array([s[0] for s in calib.samples])
    rs = np.array([s[1] for s in calib.samples])
    if len(calib.samples) >= 2 and float(np.ptp(zs)) > 0.03:
        a, b = np.polyfit(zs, rs, 1)
        calib.a, calib.b = float(np.clip(a, 0.5, 2.0)), float(b)
    else:  # una sola distancia: solo desplazamiento
        calib.a = 1.0
        calib.b = float(np.mean(rs - zs))
    return calib


def lateral_offsets(cx: float, cy: float, z_est: float, calib: WristCalib) -> Tuple[float, float]:
    """(izquierda, arriba) del objeto respecto del eje de la pinza, en metros.

    Imagen: x a la derecha, y hacia abajo. La lente esta `camera_above_axis_m` por encima
    del eje de avance, asi que un objeto centrado en la imagen esta por debajo... de la lente
    y a la altura del eje cuando cy coincide con la proyeccion del eje.
    """
    left = -(cx - calib.cx_px) / calib.focal_px * z_est
    up = -(cy - calib.cy_px) / calib.focal_px * z_est + calib.camera_above_axis_m
    return float(left), float(up)


def object_center(tcp: Sequence[float], R_tool: np.ndarray, advance_m: float, left_m: float, up_m: float,
                  calib: WristCalib) -> np.ndarray:
    """Centro del objeto en el mismo marco que `tcp`/`R_tool` (columnas = ejes de la herramienta).

    Eje de avance = z de la herramienta. "Izquierda/arriba" se toman en el plano perpendicular
    usando la vertical del mundo cuando es posible (robusto a la rotacion de la muneca).
    """
    tcp = np.asarray(tcp, dtype=float)
    approach = np.asarray(R_tool, dtype=float)[:, 2]
    approach = approach / max(1e-9, np.linalg.norm(approach))
    world_up = np.array([0.0, 0.0, 1.0])
    left = np.cross(world_up, approach)
    if np.linalg.norm(left) < 1e-3:  # herramienta vertical: usar el eje y de la herramienta
        left = np.asarray(R_tool, dtype=float)[:, 1]
    left = left / np.linalg.norm(left)
    up = np.cross(approach, left)
    grasp_center = tcp - calib.grasp_center_back_m * approach
    return grasp_center + advance_m * approach + left_m * left + up_m * up


def alignment_offset(R_tool: np.ndarray, left_m: float, up_m: float) -> np.ndarray:
    """Desplazamiento del TCP (mismo marco que R_tool) para centrar el objeto en el eje de la pinza.

    Se mueve solo en el plano perpendicular al eje de avance (izquierda/arriba como en
    object_center), sin acercarse al objeto ni cambiar la orientacion.
    """
    approach = np.asarray(R_tool, dtype=float)[:, 2]
    approach = approach / max(1e-9, np.linalg.norm(approach))
    left = np.cross([0.0, 0.0, 1.0], approach)
    if np.linalg.norm(left) < 1e-3:
        left = np.asarray(R_tool, dtype=float)[:, 1]
    left = left / np.linalg.norm(left)
    up = np.cross(approach, left)
    return left_m * left + up_m * up


def is_aligned(left_m: float, up_m: float, tol_lateral_m: float = 0.008, tol_vertical_m: float = 0.012) -> bool:
    return abs(left_m) <= tol_lateral_m and abs(up_m) <= tol_vertical_m
