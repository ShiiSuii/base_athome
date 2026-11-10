"""De una deteccion 2D a saber donde esta la bolsa. Un solo lugar para la mision, el debug y la simulacion. Sin ROS.

Todo en un marco fijo con z hacia arriba y z = 0 en el piso (map o base_footprint). Una camara se describe con
(T, R): su posicion y sus ejes opticos en ese marco (columnas: x derecha, y abajo, z adelante), y K = (fx, fy, cx, cy).

Por que hay validaciones: con la camara casi horizontal, un pixel apenas bajo el horizonte corta el plano a metros de
distancia (medido en el robot: 15 m). Una medicion que no pasa se descarta con su motivo, nunca se usa.
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from base_athome_skills.mission import bag_task

L_SHOULDER, R_SHOULDER, L_WRIST, R_WRIST, L_HIP, R_HIP = 5, 6, 9, 10, 11, 12
MIN_KP_CONF = 0.4
MIN_RAY_DOWN_DEG = 20.0        # muneca: un rayo mas plano convierte un pixel de error en decimetros
WRIST_MAX_RANGE_M = 1.6        # la muneca solo mide de cerca
HEAD_DEPTH_RANGE_M = (0.3, 6.0)
BAG_MAX_TOP_M = 1.3            # mas alto que esto no es una bolsa en el piso ni en una mesa
BAG_MAX_RADIUS_M = 0.30
BORDER_PX = 3
MIN_DEPTH_POINTS = 12
FLOOR_MARGIN_M = 0.04
BELIEF_GATE_M = 0.35           # una medicion nueva mas lejos que esto de lo que se cree es otro objeto
HEAD_STILL_DPS = 3.0           # con la cabeza moviendose, imagen y TF no son del mismo instante


@dataclass
class Measurement:
    """Resultado de medir: posicion, o el motivo por el que no sirve."""
    xy: Optional[Tuple[float, float]] = None
    top_z: Optional[float] = None
    radius: Optional[float] = None
    reason: str = ""

    @property
    def ok(self) -> bool:
        return self.xy is not None


def quat_to_R(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pixel_ray(u: float, v: float, R: np.ndarray, K: Sequence[float]) -> np.ndarray:
    """Direccion en el marco fijo del pixel, con componente optica 1 (al multiplicar por la profundidad da el punto)."""
    return np.asarray(R, dtype=float) @ np.array([(u - K[2]) / K[0], (v - K[3]) / K[1], 1.0])


def depth_at(depth: np.ndarray, u: float, v: float, radius: int = 4) -> Optional[float]:
    """Mediana de la profundidad valida alrededor del pixel, en metros."""
    h, w = depth.shape[:2]
    u, v = int(round(u)), int(round(v))
    if not (0 <= u < w and 0 <= v < h):
        return None
    patch = depth[max(0, v - radius):v + radius + 1, max(0, u - radius):u + radius + 1]
    valid = patch[patch > 0]
    return float(np.median(valid)) / 1000.0 if valid.size else None


def wrist_measure(box: Sequence[float], T: Sequence[float], R: np.ndarray, K: Sequence[float], top_z: float,
                  size: Optional[Tuple[int, int]] = None) -> Measurement:
    """Bolsa vista por la muneca, sin profundidad: el rayo del centro de la caja corta la altura media de la bolsa.

    El centro de la caja corresponde al centro del bulto, a media altura, tambien con la camara inclinada. Usar el
    borde superior contra la altura de arriba corre el resultado hacia atras tanto como mide la bolsa.
    """
    if size is not None and (box[0] <= BORDER_PX or box[1] <= BORDER_PX or box[2] >= size[0] - BORDER_PX or box[3] >= size[1] - BORDER_PX):
        return Measurement(reason="bolsa cortada por el borde de la imagen: el centro de la caja no es el suyo")
    u, v = (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0
    ray = pixel_ray(u, v, R, K)
    down = math.degrees(math.atan2(-ray[2], math.hypot(ray[0], ray[1])))
    if down < MIN_RAY_DOWN_DEG:
        return Measurement(reason=f"rayo casi horizontal ({down:.0f} grados): la camara no mira al piso")
    s = (float(top_z) / 2.0 - float(T[2])) / ray[2]
    if s <= 0.0:
        return Measurement(reason="la bolsa quedaria arriba de la camara")
    p = np.asarray(T, dtype=float) + s * ray
    dist = float(np.linalg.norm(p - np.asarray(T, dtype=float)))
    if dist > WRIST_MAX_RANGE_M:
        return Measurement(reason=f"demasiado lejos para la muneca ({dist:.1f} m)")
    return Measurement(xy=(float(p[0]), float(p[1])), top_z=float(top_z))


def _head_without_depth(box, T, R, K) -> Measurement:
    """Bolsa negra: la RealSense casi no devuelve profundidad sobre ella. El pie de la caja toca el piso: el rayo por ese
    punto da la distancia, y a esa distancia el borde superior da la altura. Vale solo mirando bien hacia abajo."""
    T = np.asarray(T, dtype=float)
    u = (box[0] + box[2]) / 2.0
    foot = pixel_ray(u, box[3], R, K)
    if math.degrees(math.atan2(-foot[2], math.hypot(foot[0], foot[1]))) < MIN_RAY_DOWN_DEG:
        return Measurement(reason="sin profundidad y el rayo al piso es casi horizontal")
    s = -float(T[2]) / foot[2]
    near = T + s * foot
    if not HEAD_DEPTH_RANGE_M[0] <= s <= HEAD_DEPTH_RANGE_M[1]:
        return Measurement(reason=f"sin profundidad y el piso queda fuera de rango ({s:.2f})")
    top_z = float(T[2] + s * pixel_ray(u, box[1], R, K)[2])
    if not 0.05 <= top_z <= BAG_MAX_TOP_M:
        return Measurement(reason=f"sin profundidad y la altura no es razonable ({top_z:.2f} m)")
    radius = min(BAG_MAX_RADIUS_M, abs(box[2] - box[0]) * s / K[0] / 2.0)
    d = np.array([near[0] - T[0], near[1] - T[1]])
    xy = near[:2] + radius * d / max(1e-3, float(np.linalg.norm(d)))
    return Measurement(xy=(float(xy[0]), float(xy[1])), top_z=top_z, radius=float(radius))


def head_measure(box: Sequence[float], depth: np.ndarray, T: Sequence[float], R: np.ndarray, K: Sequence[float],
                 allow_clipped: bool = False) -> Measurement:
    """Bolsa vista por la cabeza, con los puntos de profundidad que caen en la caja.

    Se quedan los puntos sobre el piso y a la distancia del bulto. La altura sale del punto mas alto. Solo se ve el lado
    que da a la camara: el centro esta un radio mas alla del punto mas cercano, y el radio se mide de costado.
    `allow_clipped`: acepta una bolsa cortada por el borde (el limite de la cabeza puede dejarla al costado): sirve para
    saber hacia donde ir, no para agarrar; hay que volver a medirla de cerca.
    """
    h, w = depth.shape[:2]
    if not allow_clipped and (box[0] <= BORDER_PX or box[1] <= BORDER_PX or box[2] >= w - BORDER_PX or box[3] >= h - BORDER_PX):
        return Measurement(reason="bolsa cortada por el borde de la imagen")
    x0, x1 = max(0, int(box[0])), min(w, int(box[2]) + 1)
    y0, y1 = max(0, int(box[1])), min(h, int(box[3]) + 1)
    if x1 - x0 < 2 or y1 - y0 < 2:
        return Measurement(reason="caja vacia")
    us, vs = np.meshgrid(np.arange(x0, x1, max(1, (x1 - x0) // 40)), np.arange(y0, y1, max(1, (y1 - y0) // 40)))
    z = depth[vs, us].astype(float) / 1000.0
    valid = (z >= HEAD_DEPTH_RANGE_M[0]) & (z <= HEAD_DEPTH_RANGE_M[1])
    if int(valid.sum()) < MIN_DEPTH_POINTS:
        return _head_without_depth(box, T, R, K)
    rays = np.stack([(us[valid] - K[2]) / K[0], (vs[valid] - K[3]) / K[1], np.ones(int(valid.sum()))])
    T = np.asarray(T, dtype=float)
    pts = T + (np.asarray(R, dtype=float) @ (rays * z[valid])).T
    pts = pts[pts[:, 2] > FLOOR_MARGIN_M]              # el piso que entra en la caja no es la bolsa
    if len(pts) < MIN_DEPTH_POINTS:
        return Measurement(reason="a la altura del piso: sin bulto")
    dist = np.linalg.norm(pts[:, :2] - T[:2], axis=1)
    pts = pts[dist < np.median(dist) + BAG_MAX_RADIUS_M * 2.0]     # lo que se ve atras (pared, otra cosa) tampoco
    top_z = float(np.percentile(pts[:, 2], 95))
    if top_z > BAG_MAX_TOP_M:
        return Measurement(reason=f"demasiado alto para ser una bolsa ({top_z:.2f} m)")
    center = np.median(pts[:, :2], axis=0)
    ahead = (center - T[:2]) / max(1e-3, float(np.linalg.norm(center - T[:2])))
    side = np.array([-ahead[1], ahead[0]])
    along, across = (pts[:, :2] - T[:2]) @ ahead, (pts[:, :2] - T[:2]) @ side
    radius = min(BAG_MAX_RADIUS_M, float(np.percentile(across, 95) - np.percentile(across, 5)) / 2.0)
    forward = float(np.percentile(along, 5)) + radius
    lateral = float(np.percentile(across, 95) + np.percentile(across, 5)) / 2.0
    xy = T[:2] + forward * ahead + lateral * side
    return Measurement(xy=(float(xy[0]), float(xy[1])), top_z=top_z, radius=radius)


def choose_nearest(measures: Sequence[Measurement], expected_xy: Optional[Sequence[float]],
                   gate_m: float = BELIEF_GATE_M) -> Tuple[Optional[int], str]:
    """Indice de la medicion valida mas cercana a lo esperado. Sin expectativa, solo si hay una: elegir entre varias
    sin referencia es adivinar. Asi se asocia entre camaras: la confianza del detector no dice cual es LA bolsa."""
    best, best_d = None, float("inf")
    rejected = ""
    for i, m in enumerate(measures):
        if not m.ok:
            rejected = rejected or m.reason
            continue
        if expected_xy is None:
            if best is not None:
                return None, "varias bolsas y ninguna referencia para elegir"
            best = i
            continue
        d = math.hypot(m.xy[0] - expected_xy[0], m.xy[1] - expected_xy[1])
        if d > gate_m:
            rejected = rejected or f"a {d * 100:.0f} cm de donde deberia estar"
        elif d < best_d:
            best, best_d = i, d
    return best, ("" if best is not None else rejected or "no ve una bolsa")


@dataclass
class BagBelief:
    """Donde se cree que esta la bolsa: mediana de las ultimas mediciones coherentes, en un marco fijo."""
    gate_m: float = BELIEF_GATE_M
    keep: int = 7
    samples: List[Tuple[float, float, float]] = field(default_factory=list)
    rejected: int = 0
    keep_outliers: int = 4
    outliers: List[Tuple[float, float, float]] = field(default_factory=list)

    def update(self, m: Measurement) -> bool:
        if not m.ok:
            return False
        if self.samples:
            cx, cy = self.xy
            if math.hypot(m.xy[0] - cx, m.xy[1] - cy) > self.gate_m:
                self.rejected += 1
                self.outliers = [o for o in self.outliers if math.hypot(m.xy[0] - o[0], m.xy[1] - o[1]) <= self.gate_m]
                self.outliers.append((m.xy[0], m.xy[1], m.top_z if m.top_z is not None else self.top_z))
                if len(self.outliers) >= self.keep_outliers:   # la primera medicion era la mala: no quedar anclado a ella
                    self.samples, self.outliers = self.outliers, []
                    return True
                return False
            self.outliers = []
        top = m.top_z if m.top_z is not None else (self.top_z or 0.0)
        self.samples = (self.samples + [(m.xy[0], m.xy[1], top)])[-self.keep:]
        return True

    @property
    def count(self) -> int:
        return len(self.samples)

    @property
    def xy(self) -> Optional[Tuple[float, float]]:
        if not self.samples:
            return None
        a = np.array(self.samples)
        return float(np.median(a[:, 0])), float(np.median(a[:, 1]))

    @property
    def top_z(self) -> Optional[float]:
        return float(np.median(np.array(self.samples)[:, 2])) if self.samples else None

    def spread(self) -> float:
        """Cuanto varian las mediciones (m): grande = todavia no confiar."""
        if len(self.samples) < 2:
            return float("inf")
        a = np.array(self.samples)[:, :2]
        return float(np.max(np.linalg.norm(a - np.median(a, axis=0), axis=1)))


# ---------------------------------------------------------------- personas
def torso_pixel(person) -> Tuple[float, float]:
    """Centro del torso: al centrarlo entran los brazos y las caderas que hacen falta para leer el gesto."""
    box, kp, conf = person
    pts = [kp[i] for i in (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP) if conf[i] >= MIN_KP_CONF]
    if pts:
        return float(np.mean([p[0] for p in pts])), float(np.mean([p[1] for p in pts]))
    return float((box[0] + box[2]) / 2.0), float((box[1] + box[3]) / 2.0)


def _keypoint_3d(kp, conf, i, depth, T, R, K):
    if conf[i] < MIN_KP_CONF:
        return None
    z = depth_at(depth, kp[i][0], kp[i][1])
    return None if z is None else np.asarray(T, dtype=float) + z * pixel_ray(kp[i][0], kp[i][1], R, K)


def pointed_target(people: Sequence, depth: np.ndarray, T, R, K) -> Tuple[Optional[Tuple[float, float]], str]:
    """Punto del piso que senala alguna persona, o el motivo por el que no. Lo usan la mision y el debug."""
    if not people:
        return None, "no ve personas"
    reason = "ve persona, sin gesto de senalar"
    for _, kp, conf in people:
        for sh, wr, hip in ((L_SHOULDER, L_WRIST, L_HIP), (R_SHOULDER, R_WRIST, R_HIP)):
            s, w, h = (_keypoint_3d(kp, conf, i, depth, T, R, K) for i in (sh, wr, hip))
            if s is None or w is None or h is None:
                if min(conf[sh], conf[wr], conf[hip]) < MIN_KP_CONF:
                    reason = "no ve el hombro, la muneca o la cadera: falta cuerpo en la imagen"
                continue
            if bag_task.is_pointing(s, w, h):
                target = bag_task.pointed_floor_point(s, w)
                if target:
                    return target, ""
                reason = "brazo separado, pero no apunta al piso"
    return None, reason


def _person_depth_region(depth, u, v, mask):
    """Measured surface only: enough pixels, dominant depth cluster, limited spread."""
    z = depth[v, u].astype(float) / 1000.0
    valid = mask & np.isfinite(z) & (z >= HEAD_DEPTH_RANGE_M[0]) & (z <= HEAD_DEPTH_RANGE_M[1])
    u, v, z = u[valid], v[valid], z[valid]
    if len(z) < 12:
        return None
    order = np.argsort(z)
    groups = np.split(order, np.where(np.diff(z[order]) > 0.20)[0] + 1)
    group = max(groups, key=len)
    if len(group) < max(12, 0.6 * len(z)):
        return None
    u, v, z = u[group], v[group], z[group]
    if (len(np.unique(u)) < 3 or len(np.unique(v)) < 3
            or np.percentile(z, 90) - np.percentile(z, 10) > 0.35):
        return None
    return float(np.median(u)), float(np.median(v)), float(np.median(z))


def person_depth_sample(person, depth):
    """Recover depth holes using the interior torso, then reliable facial landmarks.

    Never sample the full bounding box: it includes background and raised-arm gaps.
    Returns measured (pixel_u, pixel_v, depth_m), not an assumed human size/range.
    """
    if depth is None or depth.ndim != 2:
        return None
    _, kp, conf = person
    h, w = depth.shape
    def good(i):
        return (conf[i] >= MIN_KP_CONF and np.isfinite(kp[i]).all()
                and 0 <= kp[i][0] < w and 0 <= kp[i][1] < h)

    ids = (L_SHOULDER, R_SHOULDER, R_HIP, L_HIP)
    if all(good(i) for i in ids):
        polygon = np.array([kp[i] for i in ids], dtype=float)
        center = polygon.mean(axis=0)
        polygon = center + 0.65 * (polygon - center)  # keep away from silhouette/background
        area = abs(np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1)
                          - polygon[:, 1] * np.roll(polygon[:, 0], -1))) / 2
        if area >= 12:
            lo, hi = np.floor(polygon.min(axis=0)).astype(int), np.ceil(polygon.max(axis=0)).astype(int)
            v, u = np.mgrid[lo[1]:min(h, hi[1]+1), lo[0]:min(w, hi[0]+1)]
            cross = []
            for first, last in zip(polygon, np.roll(polygon, -1, axis=0)):
                cross.append((last[0]-first[0])*(v-first[1]) - (last[1]-first[1])*(u-first[0]))
            cross = np.array(cross)
            mask = np.all(cross >= 0, axis=0) | np.all(cross <= 0, axis=0)
            result = _person_depth_region(depth, u, v, mask)
            if result is not None:
                return result

    # Cropped/seated person: preserve the central chest measurement when hips
    # are not confidently visible, but require a supported surface (not one pixel).
    elif good(L_SHOULDER) and good(R_SHOULDER):
        center = (np.asarray(kp[L_SHOULDER]) + np.asarray(kp[R_SHOULDER])) / 2
        x, y = np.rint(center).astype(int)
        v, u = np.mgrid[max(0, y-4):min(h, y+5), max(0, x-4):min(w, x+5)]
        result = _person_depth_region(depth, u, v, np.ones(u.shape, dtype=bool))
        if result is not None:
            return result

    # Face depth can remain valid when dark clothing absorbs infrared.
    # Require nose and both eyes to constrain patch size to this person's face.
    if all(good(i) for i in (0, 1, 2)):
        eye_span = float(np.linalg.norm(kp[1] - kp[2]))
        if eye_span >= 4:
            radius = min(4, max(1, int(eye_span * 0.2)))
            pixels = set()
            for i in (0, 1, 2):
                x, y = np.rint(kp[i]).astype(int)
                for yy in range(max(0, y-radius), min(h, y+radius+1)):
                    for xx in range(max(0, x-radius), min(w, x+radius+1)):
                        pixels.add((xx, yy))
            u, v = np.array(sorted(pixels)).T
            return _person_depth_region(depth, u, v, np.ones(u.shape, dtype=bool))
    return None


def person_map_position(person, depth: np.ndarray, T, R, K) -> Optional[Tuple[float, float]]:
    sample = person_depth_sample(person, depth)
    if sample is None:
        return None
    u, v, z = sample
    p = np.asarray(T, dtype=float) + z * pixel_ray(u, v, R, K)
    return float(p[0]), float(p[1])


def _sparse_person_depth(person, depth: Optional[np.ndarray]) -> Optional[Tuple[float, float, float]]:
    """Recupera profundidad rala (>= 4 pixeles coherentes en torso/hombros) cuando la persona esta lejos."""
    if depth is None or getattr(depth, "ndim", 0) != 2:
        return None
    box, kp, conf = person
    h, w = depth.shape

    def good(i):
        return (i < len(conf) and conf[i] >= MIN_KP_CONF and np.isfinite(kp[i]).all()
                and 0 <= kp[i][0] < w and 0 <= kp[i][1] < h)

    ids = (L_SHOULDER, R_SHOULDER, R_HIP, L_HIP)
    if all(good(i) for i in ids):
        polygon = np.array([kp[i] for i in ids], dtype=float)
        center = polygon.mean(axis=0)
        polygon = center + 0.70 * (polygon - center)
        lo, hi = np.floor(polygon.min(axis=0)).astype(int), np.ceil(polygon.max(axis=0)).astype(int)
        v, u = np.mgrid[max(0, lo[1]):min(h, hi[1] + 1), max(0, lo[0]):min(w, hi[0] + 1)]
        if u.size > 0:
            z = depth[v, u].astype(float) / 1000.0
            valid = np.isfinite(z) & (z >= HEAD_DEPTH_RANGE_M[0]) & (z <= 6.5)
            zv = z[valid]
            if len(zv) >= 4 and np.percentile(zv, 90) - np.percentile(zv, 10) <= 0.45:
                return float(np.median(u[valid])), float(np.median(v[valid])), float(np.median(zv))

    if good(L_SHOULDER) and good(R_SHOULDER):
        center = (np.asarray(kp[L_SHOULDER]) + np.asarray(kp[R_SHOULDER])) / 2.0
        x, y = np.rint(center).astype(int)
        v, u = np.mgrid[max(0, y - 5):min(h, y + 6), max(0, x - 5):min(w, x + 6)]
        if u.size > 0:
            z = depth[v, u].astype(float) / 1000.0
            valid = np.isfinite(z) & (z >= HEAD_DEPTH_RANGE_M[0]) & (z <= 6.5)
            zv = z[valid]
            if len(zv) >= 4 and np.percentile(zv, 90) - np.percentile(zv, 10) <= 0.45:
                return float(np.median(u[valid])), float(np.median(v[valid])), float(np.median(zv))
    return None


def _pinhole_person_depth(person, K: Sequence[float]) -> Optional[float]:
    """Estima la distancia (m) por escala aparente de keypoints/caja con intrinsecos K = (fx, fy, cx, cy)."""
    if K is None or len(K) < 4 or K[0] <= 0 or K[1] <= 0:
        return None
    box, kp, conf = person
    fx, fy = float(K[0]), float(K[1])

    def kp_ok(i):
        return (i < len(conf) and conf[i] >= MIN_KP_CONF and np.isfinite(kp[i]).all()
                and (kp[i][0] != 0.0 or kp[i][1] != 0.0))

    estimates: List[float] = []
    shoulders = [kp[i] for i in (L_SHOULDER, R_SHOULDER) if kp_ok(i)]
    hips = [kp[i] for i in (L_HIP, R_HIP) if kp_ok(i)]
    if shoulders and hips:
        sh_y = float(np.mean([p[1] for p in shoulders]))
        hip_y = float(np.mean([p[1] for p in hips]))
        dy_torso = abs(hip_y - sh_y)
        if dy_torso >= 6.0:
            estimates.append(0.50 * fy / dy_torso)

    if shoulders and kp_ok(0):
        sh_y = float(np.mean([p[1] for p in shoulders]))
        dy_head = abs(sh_y - float(kp[0][1]))
        if dy_head >= 4.0:
            estimates.append(0.24 * fy / dy_head)

    if kp_ok(L_SHOULDER) and kp_ok(R_SHOULDER):
        dx_sh = float(np.linalg.norm(np.asarray(kp[L_SHOULDER]) - np.asarray(kp[R_SHOULDER])))
        if dx_sh >= 6.0:
            estimates.append(0.40 * fx / dx_sh)

    if not estimates and box is not None and len(box) >= 4:
        bh = float(box[3]) - float(box[1])
        if math.isfinite(bh) and bh >= 12.0:
            estimates.append(1.40 * fy / bh)

    if not estimates:
        return 3.2
    return float(np.clip(np.median(estimates), 1.8, 6.0))


def person_estimated_map_position(
    person, depth: Optional[np.ndarray], T, R, K
) -> Optional[Tuple[float, float]]:
    """Fallback cuando person_map_position no alcanza 12 pixeles de profundidad (persona muy lejos).

    Usa primero profundidad rala coherente (>= 4 pixeles en torso/hombros) y, si no hay profundidad,
    estima el rango por escala aparente (torso/cabeza/hombros/bbox) proyectado sobre el rayo medido (T, R, K).
    Requiere TF (T, R) e intrinsecos K validos; nunca inventa coordenadas sin geometria de camara.
    """
    if T is None or R is None or K is None or len(K) < 4 or K[0] <= 0 or K[1] <= 0:
        return None
    T_arr, R_arr = np.asarray(T, dtype=float), np.asarray(R, dtype=float)
    if T_arr.shape != (3,) or R_arr.shape != (3, 3) or not (np.isfinite(T_arr).all() and np.isfinite(R_arr).all()):
        return None

    exact = person_map_position(person, depth, T_arr, R_arr, K)
    if exact is not None:
        return exact

    sparse = _sparse_person_depth(person, depth)
    if sparse is not None:
        u, v, z = sparse
        p = T_arr + z * pixel_ray(u, v, R_arr, K)
        return float(p[0]), float(p[1])

    u, v = torso_pixel(person)
    if not (math.isfinite(u) and math.isfinite(v)):
        return None
    z_est = _pinhole_person_depth(person, K)
    if z_est is None or not math.isfinite(z_est):
        return None
    p = T_arr + z_est * pixel_ray(u, v, R_arr, K)
    return float(p[0]), float(p[1])


def yolo_people(result) -> List:
    """Salida de YOLO pose -> [(caja, keypoints, confianzas)], la persona mas grande primero."""
    if result.keypoints is None or result.keypoints.conf is None or result.boxes is None:
        return []
    out = list(zip(result.boxes.xyxy.cpu().numpy(), result.keypoints.xy.cpu().numpy(), result.keypoints.conf.cpu().numpy()))
    return sorted(out, key=lambda p: -(p[0][2] - p[0][0]) * (p[0][3] - p[0][1]))


def project_people(people_3d: Sequence[Dict], T, R, K, width: int, height: int) -> List:
    """Personas simuladas (keypoints 3D) a la misma forma que yolo_people, segun las ve esta camara."""
    out = []
    T, R = np.asarray(T, dtype=float), np.asarray(R, dtype=float)
    for person in people_3d:
        kp, conf = np.zeros((17, 2)), np.zeros(17)
        visible = []
        for name, p in person["keypoints"].items():
            c = R.T @ (np.asarray(p, dtype=float) - T)
            if c[2] <= 0.05:
                continue
            u, v = K[2] + K[0] * c[0] / c[2], K[3] + K[1] * c[1] / c[2]
            if 0 <= u < width and 0 <= v < height:
                kp[int(name)] = (u, v)
                conf[int(name)] = 0.9
                visible.append((u, v))
        if len(visible) >= 3:
            us, vs = zip(*visible)
            out.append((np.array([min(us), min(vs), max(us), max(vs)]), kp, conf))
    return sorted(out, key=lambda p: -(p[0][2] - p[0][0]) * (p[0][3] - p[0][1]))
