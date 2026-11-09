"""Dibuja lo que ve una camara en la simulacion: piso, bolsa y persona, con profundidad. Sin ROS.

Sirve para que la percepcion REAL corra en lazo cerrado: si la cabeza se mueve mal o una medicion esta corrida, se nota,
porque la imagen sale de la geometria y no de una respuesta armada. La camara: pc = Rcw @ pw + tcw.
"""
import math
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

BAG_BGR = (200, 30, 200)      # el color que segmenta SimColorDetector
BODY_BGR = (150, 90, 40)
ARM_BGR = (170, 120, 70)
HEAD_BGR = (160, 190, 225)
MAX_DEPTH_M = 6.0
SHOULDER_Z, HIP_Z, HEAD_Z, BODY_R, SHOULDER_HALF = 1.40, 0.95, 1.62, 0.15, 0.20
ARM_M = 0.6


def floor_view(Rcw: np.ndarray, tcw: np.ndarray, K: Sequence[float], width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
    """Imagen BGR y profundidad en mm (uint16; 0 = sin dato) con el piso y un fondo sin dato."""
    us, vs = np.meshgrid(np.arange(width), np.arange(height))
    rays = np.stack([(us - K[2]) / K[0], (vs - K[3]) / K[1], np.ones_like(us, dtype=float)], axis=-1)
    world = rays @ Rcw                  # R^T @ ray, por fila
    origin = -Rcw.T @ tcw
    with np.errstate(divide="ignore", invalid="ignore"):
        s = -origin[2] / world[..., 2]
    hit = np.isfinite(s) & (s > 0.0) & (s <= MAX_DEPTH_M)
    depth = np.zeros((height, width), np.uint16)
    depth[hit] = (s[hit] * 1000.0).astype(np.uint16)
    rgb = np.full((height, width, 3), (72, 76, 80), np.uint8)
    x, y = origin[0] + s[hit] * world[hit][:, 0], origin[1] + s[hit] * world[hit][:, 1]
    dark = (np.floor(x * 2).astype(int) + np.floor(y * 2).astype(int)) % 2 == 1
    rgb[hit] = np.where(dark[:, None], (118, 122, 122), (130, 135, 135))
    return rgb, depth


def _project(points: np.ndarray, Rcw, tcw, K):
    pc = (Rcw @ points.T).T + tcw
    return pc, np.stack([K[2] + K[0] * pc[:, 0] / pc[:, 2], K[3] + K[1] * pc[:, 1] / pc[:, 2]], axis=1)


def _draw_faces(rgb, depth, faces: List[Tuple[np.ndarray, Tuple[int, int, int]]], Rcw, tcw, K) -> None:
    """Caras planas con z-buffer: la profundidad de cada pixel sale del plano de la cara."""
    h, w = depth.shape
    for pts, color in faces:
        pc, uv = _project(pts, Rcw, tcw, K)
        if np.any(pc[:, 2] < 0.03) or np.max(np.abs(uv)) > 1e5:
            continue
        x0, y0 = np.floor(uv.min(axis=0)).astype(int)
        x1, y1 = np.ceil(uv.max(axis=0)).astype(int)
        x0, y0, x1, y1 = max(0, x0), max(0, y0), min(w, x1 + 1), min(h, y1 + 1)
        if x0 >= x1 or y0 >= y1:
            continue
        n = np.cross(pc[1] - pc[0], pc[2] - pc[0])
        mask = np.zeros((y1 - y0, x1 - x0), np.uint8)
        cv2.fillConvexPoly(mask, np.round(uv - [x0, y0]).astype(np.int32), 1)
        vs, us = np.nonzero(mask)
        if us.size == 0:
            continue
        vs, us = vs + y0, us + x0
        denom = n[0] * (us - K[2]) / K[0] + n[1] * (vs - K[3]) / K[1] + n[2]
        with np.errstate(divide="ignore", invalid="ignore"):
            z = np.where(np.abs(denom) > 1e-9, float(n @ pc[0]) / denom, 0.0) * 1000.0
        z = np.clip(np.nan_to_num(z), 1.0, 65534.0)
        now = depth[vs, us]
        front = (now == 0) | (z < now)
        depth[vs[front], us[front]] = z[front].astype(np.uint16)
        rgb[vs[front], us[front]] = color


def box_faces(center: Sequence[float], size: Sequence[float], color) -> List:
    cx, cy, cz = center
    hx, hy, hz = size[0] / 2.0, size[1] / 2.0, size[2] / 2.0
    c = np.array([[cx + a * hx, cy + b * hy, cz + d * hz] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)])
    idx = ((0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5))
    return [(c[list(i)], color) for i in idx]


def prism_faces(xy: Sequence[float], z0: float, z1: float, radius: float, color, sides: int = 10) -> List:
    a = [2.0 * math.pi * i / sides for i in range(sides)]
    ring = [(xy[0] + radius * math.cos(t), xy[1] + radius * math.sin(t)) for t in a]
    faces = [(np.array([[*ring[i], z0], [*ring[(i + 1) % sides], z0], [*ring[(i + 1) % sides], z1], [*ring[i], z1]]), color)
             for i in range(sides)]
    faces.append((np.array([[*p, z1] for p in ring]), color))
    return faces


def segment_faces(p0: Sequence[float], p1: Sequence[float], half: float, color) -> List:
    """Brazo: una barra de seccion cuadrada entre dos puntos."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    d = d / max(1e-6, np.linalg.norm(d))
    a = np.cross(d, [0.0, 0.0, 1.0])
    if np.linalg.norm(a) < 1e-3:
        a = np.array([1.0, 0.0, 0.0])
    a = a / np.linalg.norm(a) * half
    b = np.cross(d, a)
    corners = [a + b, a - b, -a - b, -a + b]
    return [(np.array([p0 + corners[i], p1 + corners[i], p1 + corners[(i + 1) % 4], p0 + corners[(i + 1) % 4]]), color)
            for i in range(4)]


def make_person(xy: Sequence[float], facing_xy: Sequence[float], pointing_to: Optional[Sequence[float]] = None) -> Dict:
    """Persona de pie mirando a `facing_xy`; con `pointing_to` extiende el brazo derecho hacia ese punto del piso.
    Keypoints COCO: 0 nariz, 5/6 hombros, 7/8 codos, 9/10 munecas, 11/12 caderas."""
    yaw = math.atan2(facing_xy[1] - xy[1], facing_xy[0] - xy[0])
    left = np.array([-math.sin(yaw), math.cos(yaw)])
    side = lambda s, z: [xy[0] + s * left[0], xy[1] + s * left[1], z]
    r_sh, l_sh = np.array(side(-SHOULDER_HALF, SHOULDER_Z)), np.array(side(SHOULDER_HALF, SHOULDER_Z))
    if pointing_to is not None:
        d = np.array([pointing_to[0], pointing_to[1], 0.0]) - r_sh
        r_wr = r_sh + ARM_M * d / np.linalg.norm(d)
    else:
        r_wr = r_sh + np.array([0.0, 0.0, -ARM_M])
    l_wr = l_sh + np.array([0.0, 0.0, -ARM_M])
    keypoints = {0: [xy[0], xy[1], HEAD_Z], 5: l_sh.tolist(), 6: r_sh.tolist(), 7: ((l_sh + l_wr) / 2).tolist(),
                 8: ((r_sh + r_wr) / 2).tolist(), 9: l_wr.tolist(), 10: r_wr.tolist(),
                 11: side(0.12, HIP_Z), 12: side(-0.12, HIP_Z)}
    return dict(xy=list(xy), keypoints=keypoints)


def make_customer(
    xy: Sequence[float],
    facing_xy: Optional[Sequence[float]] = None,
    hand: str = "right",
    seated: bool = False
) -> Dict:
    """Cliente en la simulacion con la mano levantada (Restaurant).
    Keypoints COCO con la muneca sobre la cabeza.
    """
    if facing_xy is None:
        yaw = 0.0
    else:
        yaw = math.atan2(facing_xy[1] - xy[1], facing_xy[0] - xy[0])
    left = np.array([-math.sin(yaw), math.cos(yaw)])

    head_z = 1.25 if seated else HEAD_Z
    shoulder_z = 1.05 if seated else SHOULDER_Z
    hip_z = 0.55 if seated else HIP_Z

    side = lambda s, z: [xy[0] + s * left[0], xy[1] + s * left[1], z]
    r_sh = np.array(side(-SHOULDER_HALF, shoulder_z))
    l_sh = np.array(side(SHOULDER_HALF, shoulder_z))

    raised_z = head_z + 0.20

    if hand in ("right", "both"):
        r_wr = np.array([r_sh[0], r_sh[1], raised_z])
    else:
        r_wr = r_sh + np.array([0.0, 0.0, -ARM_M])

    if hand in ("left", "both"):
        l_wr = np.array([l_sh[0], l_sh[1], raised_z])
    else:
        l_wr = l_sh + np.array([0.0, 0.0, -ARM_M])

    r_el = (r_sh + r_wr) / 2.0
    l_el = (l_sh + l_wr) / 2.0

    keypoints = {
        0: [xy[0], xy[1], head_z],
        5: l_sh.tolist(),
        6: r_sh.tolist(),
        7: l_el.tolist(),
        8: r_el.tolist(),
        9: l_wr.tolist(),
        10: r_wr.tolist(),
        11: side(0.12, hip_z),
        12: side(-0.12, hip_z)
    }
    return dict(xy=list(xy), keypoints=keypoints, hand=hand, seated=seated)


def person_faces(person: Dict) -> List:
    kp = person["keypoints"]
    # Soporta tanto personas de pie como clientes sentados usando la altura de sus keypoints
    k5 = kp.get(5) or kp.get("5")
    k0 = kp.get(0) or kp.get("0")
    shoulder_z = float(k5[2]) if k5 is not None else SHOULDER_Z
    head_z = float(k0[2]) if k0 is not None else HEAD_Z
    k9 = kp.get(9) or kp.get("9")
    k10 = kp.get(10) or kp.get("10")
    faces = prism_faces(person["xy"], 0.0, shoulder_z + 0.05, BODY_R + 0.07, BODY_BGR)
    faces += prism_faces(person["xy"], shoulder_z + 0.05, head_z + 0.1, 0.10, HEAD_BGR)
    faces += segment_faces(k5, k9, 0.045, ARM_BGR) + segment_faces(k6 := (kp.get(6) or kp.get("6")), k10, 0.045, ARM_BGR)
    return faces


def bag_faces(xy: Sequence[float], top_z: float, width: float = 0.28, center_z: Optional[float] = None) -> List:
    cz = top_z / 2.0 if center_z is None else center_z
    return box_faces((xy[0], xy[1], cz), (width, width, top_z if center_z is None else 0.30), BAG_BGR)


def render(Rcw: np.ndarray, tcw: np.ndarray, K: Sequence[float], width: int, height: int, faces: List) -> Tuple[np.ndarray, np.ndarray]:
    rgb, depth = floor_view(Rcw, tcw, K, width, height)
    _draw_faces(rgb, depth, faces, Rcw, tcw, K)
    return rgb, depth
