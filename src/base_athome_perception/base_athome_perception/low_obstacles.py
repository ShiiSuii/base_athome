"""Obstaculos bajos (4-38 cm sobre el piso) desde una imagen de profundidad (HOME-67).

Modulo puro: numpy solamente, sin ROS, sin camara, sin Nav2. El lidar esta a 38 cm del piso
y no ve patas bajas, cajas ni bolsas; la RealSense de la cabeza si puede verlas. Este modulo
convierte una imagen de profundidad + intrinsecos K + pose de la camara en un barrido tipo
``sensor_msgs/LaserScan`` (rangos por angulo) con solo los puntos cuya altura sobre el piso
esta entre ``z_min`` y ``z_max``.

PROTOTIPO: no esta conectado a ninguna parte. Integrarlo en la navegacion (ver
docs/ROADMAP.md HOME-67) es decision del usuario. La validacion es solo con escenas
sinteticas; la simulacion no demuestra seguridad fisica.

Convenciones
------------
* Imagen de profundidad: ``H x W``; ``uint16`` en milimetros (``depth_scale=0.001``, formato
  de la RealSense) o ``float`` en metros (``depth_scale=1.0``). 0, NaN, inf = sin dato.
* Frame optico de la camara: x derecha, y abajo, z adelante (``camera_color_optical_frame``;
  la profundidad alineada a color usa los K del color).
* Frame del piso/base (``base_footprint``): x adelante, y izquierda, z arriba, z=0 es el piso.
* ``T_base_cam`` (4x4): pose del frame optico de la camara expresada en el frame del piso. En
  el robot sale de TF (base_footprint <- frame optico); con la cabeza pan-tilt y el elevador
  cambia en cada cuadro, por eso es un argumento y no una constante.
* Barrido: angulo = atan2(y, x) en el plano horizontal del piso, rango = distancia horizontal
  desde el origen de ``base_footprint`` (no desde la camara).
* ``ranges`` (como LaserScan): valor finito = obstaculo; ``+inf`` = bin observado sin
  obstaculo bajo; ``NaN`` = bin sin datos de profundidad (la camara no lo vio). El que
  consuma el barrido NO debe tratar NaN como espacio libre.
"""
from collections import deque
from dataclasses import dataclass, field
import math
import time

import numpy as np

DEFAULT_Z_MIN_M = 0.04   # por debajo: piso, juntas, alfombra, ruido de profundidad
DEFAULT_Z_MAX_M = 0.38   # altura del haz del lidar: por encima ya lo ve el lidar


@dataclass
class LowObstacleConfig:
    """Parametros del detector. Todo en metros / radianes salvo que se indique."""

    z_min: float = DEFAULT_Z_MIN_M
    z_max: float = DEFAULT_Z_MAX_M
    # Margen extra sobre z_min que crece con la distancia horizontal (m de altura por m de
    # rango). Absorbe el error de pitch/altura de la camara, que levanta el piso lejano.
    floor_margin_per_m: float = 0.0
    # Profundidad valida de la camara (D435: ~0.28 m minimo a 640x480; el ruido crece con z^2).
    depth_min: float = 0.25
    depth_max: float = 3.0
    depth_scale: float = 0.001          # uint16 mm -> m. Usar 1.0 si la imagen ya esta en m
    stride: int = 2                     # submuestreo de pixeles (1 = todos): controla el costo
    # Barrido de salida
    angle_min: float = -math.pi
    angle_max: float = math.pi
    angle_increment: float = math.radians(1.0)
    range_min: float = 0.10
    range_max: float = 2.0
    # Filtros de ruido
    min_neighbors: int = 2              # vecinos (de 8) que tambien deben ser candidatos
    min_points_per_bin: int = 3         # puntos minimos para aceptar un bin
    persistence_window: int = 1         # cuadros que se recuerdan (1 = sin filtro temporal)
    persistence_min_hits: int = 1       # en cuantos de esos cuadros debe aparecer el bin

    def validate(self):
        if not 0.0 <= self.z_min < self.z_max:
            raise ValueError('se exige 0 <= z_min < z_max')
        if self.stride < 1 or int(self.stride) != self.stride:
            raise ValueError('stride debe ser un entero >= 1')
        if self.angle_increment <= 0.0 or self.angle_max <= self.angle_min:
            raise ValueError('angulos del barrido invalidos')
        if not 0.0 <= self.range_min < self.range_max:
            raise ValueError('se exige 0 <= range_min < range_max')
        if not 0.0 < self.depth_min < self.depth_max:
            raise ValueError('se exige 0 < depth_min < depth_max')
        if self.depth_scale <= 0.0:
            raise ValueError('depth_scale debe ser > 0')
        if not 0 <= self.min_neighbors <= 8:
            raise ValueError('min_neighbors debe estar en 0..8')
        if self.min_points_per_bin < 1:
            raise ValueError('min_points_per_bin debe ser >= 1')
        if not 1 <= self.persistence_min_hits <= self.persistence_window:
            raise ValueError('se exige 1 <= persistence_min_hits <= persistence_window')


@dataclass
class LowScan:
    """Barrido tipo LaserScan. ``ranges``: finito=obstaculo, +inf=libre observado, NaN=sin dato."""

    ranges: np.ndarray
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    num_candidate_points: int = 0       # puntos entre z_min y z_max antes de los filtros
    num_filtered_points: int = 0        # puntos que sobreviven al filtro de ruido
    stats: dict = field(default_factory=dict)

    @property
    def angles(self):
        return self.angle_min + (np.arange(self.ranges.size) + 0.5) * self.angle_increment

    @property
    def obstacle_mask(self):
        return np.isfinite(self.ranges)

    def nearest(self):
        """(rango, angulo) del obstaculo mas cercano, o None si no hay."""
        mask = self.obstacle_mask
        if not mask.any():
            return None
        idx = int(np.argmin(np.where(mask, self.ranges, np.inf)))
        return float(self.ranges[idx]), float(self.angles[idx])


def camera_pose_from_head(height, pitch_down=0.0, roll=0.0, yaw=0.0, x=0.0, y=0.0):
    """T_base_cam (frame optico en el piso) para una camara a ``height`` m con la mirada inclinada.

    ``pitch_down`` > 0 mira hacia el piso, ``yaw`` > 0 gira a la izquierda, ``roll`` sobre el eje
    optico. Util para pruebas y bancos; en el robot la pose sale de TF. Rotacion =
    Rz(yaw) Ry(pitch) Rx(roll) R0, con R0 = optico -> base sin inclinar.
    """
    r0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch_down), math.sin(pitch_down)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    t = np.eye(4)
    t[:3, :3] = rz @ ry @ rx @ r0
    t[:3, 3] = (x, y, height)
    return t


def _check_pose(t):
    t = np.asarray(t, dtype=np.float64)
    if t.shape != (4, 4) or not np.all(np.isfinite(t)):
        raise ValueError('T_base_cam debe ser una matriz 4x4 finita')
    r = t[:3, :3]
    if not np.allclose(r @ r.T, np.eye(3), atol=1e-3) or abs(np.linalg.det(r) - 1.0) > 1e-3:
        raise ValueError('la rotacion de T_base_cam no es ortonormal')
    if not np.allclose(t[3], [0, 0, 0, 1], atol=1e-9):
        raise ValueError('la ultima fila de T_base_cam debe ser [0, 0, 0, 1]')
    return t


def _check_k(k):
    k = np.asarray(k, dtype=np.float64)
    if k.shape == (9,):
        k = k.reshape(3, 3)  # CameraInfo.k
    if k.shape != (3, 3) or not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('K debe ser 3x3 (o 9 valores) finita con fx, fy > 0')
    return k


def _neighbor_count(mask):
    """Cuantos de los 8 vecinos de cada pixel son True (bordes: vecinos fuera = False)."""
    h, w = mask.shape
    padded = np.zeros((h + 2, w + 2), dtype=np.uint8)
    padded[1:-1, 1:-1] = mask
    count = np.zeros((h, w), dtype=np.uint8)
    for dy in range(3):
        for dx in range(3):
            if dy == 1 and dx == 1:
                continue
            count += padded[dy:dy + h, dx:dx + w]
    return count


class LowObstacleDetector:
    """Detector con cache de rayos por pixel y filtro temporal opcional (con estado)."""

    def __init__(self, k, config=None):
        self.config = config or LowObstacleConfig()
        self.config.validate()
        self.k = _check_k(k)
        self._grid_key = None
        self._xn = self._yn = None
        self._history = deque(maxlen=self.config.persistence_window)
        self.num_bins = int(math.ceil(
            (self.config.angle_max - self.config.angle_min) / self.config.angle_increment - 1e-9))

    def reset(self):
        """Olvida el historial del filtro temporal (llamar al perder la camara o cambiar de escena)."""
        self._history.clear()

    def _rays(self, shape):
        if self._grid_key != shape:
            s = self.config.stride
            v, u = np.mgrid[0:shape[0]:s, 0:shape[1]:s]
            self._xn = ((u - self.k[0, 2]) / self.k[0, 0]).astype(np.float32)
            self._yn = ((v - self.k[1, 2]) / self.k[1, 1]).astype(np.float32)
            self._grid_key = shape
        return self._xn, self._yn

    def detect(self, depth, t_base_cam):
        """Devuelve un ``LowScan`` para una imagen de profundidad y la pose de la camara."""
        cfg = self.config
        depth = np.asarray(depth)
        if depth.ndim != 2 or depth.size == 0:
            raise ValueError('la profundidad debe ser una imagen 2D no vacia')
        t = _check_pose(t_base_cam)
        r, tr = t[:3, :3], t[:3, 3]
        xn, yn = self._rays(depth.shape)
        z = depth[::cfg.stride, ::cfg.stride].astype(np.float32) * np.float32(cfg.depth_scale)

        with np.errstate(invalid='ignore'):
            valid = np.isfinite(z) & (z >= cfg.depth_min) & (z <= cfg.depth_max)
        z = np.where(valid, z, np.float32(0.0))

        # Coordenadas en el frame del piso: P = z * (R @ [xn, yn, 1]) + t  (z es la profundidad
        # optica). Primero la altura (barata), recien despues x e y de los candidatos.
        ax = r[0, 0] * xn + r[0, 1] * yn + r[0, 2]
        ay = r[1, 0] * xn + r[1, 1] * yn + r[1, 2]
        az = r[2, 0] * xn + r[2, 1] * yn + r[2, 2]
        h = z * az + tr[2]
        px = z * ax + tr[0]
        py = z * ay + tr[1]
        horiz = np.hypot(px, py)

        z_lo = cfg.z_min + cfg.floor_margin_per_m * horiz
        cand = valid & (h >= z_lo) & (h <= cfg.z_max) & (horiz >= cfg.range_min) \
            & (horiz <= cfg.range_max)
        n_candidates = int(cand.sum())

        # Filtro de ruido 1: pixeles aislados (flying pixels, speckle) no son obstaculos.
        if cfg.min_neighbors > 0 and n_candidates:
            cand &= _neighbor_count(cand) >= cfg.min_neighbors

        ang = np.arctan2(py, px)
        nb = self.num_bins
        ranges = np.full(nb, np.nan, dtype=np.float32)

        # Bins observados (con cualquier profundidad valida): inf = libre, NaN = sin dato.
        obs_idx = ((ang[valid] - cfg.angle_min) / cfg.angle_increment).astype(np.int64)
        obs_idx = obs_idx[(obs_idx >= 0) & (obs_idx < nb)]
        observed = np.zeros(nb, dtype=bool)
        observed[obs_idx] = True
        ranges[observed] = np.inf

        n_filtered = int(cand.sum())
        hit = np.zeros(nb, dtype=bool)
        best = np.full(nb, np.inf, dtype=np.float32)
        if n_filtered:
            idx = np.floor((ang[cand] - cfg.angle_min) / cfg.angle_increment).astype(np.int64)
            rng = horiz[cand]
            keep = (idx >= 0) & (idx < nb)
            idx, rng = idx[keep], rng[keep]
            counts = np.bincount(idx, minlength=nb)
            np.minimum.at(best, idx, rng)
            hit = counts >= cfg.min_points_per_bin  # Filtro de ruido 2: puntos por bin

        # Filtro de ruido 3 (opcional): persistencia en varios cuadros.
        self._history.append(hit)
        if cfg.persistence_window > 1:
            hits = np.sum(self._history, axis=0)
            accepted = hit & (hits >= cfg.persistence_min_hits)
        else:
            accepted = hit
        ranges[accepted] = best[accepted]
        # un bin con obstaculo aceptado cuenta como observado aunque no tuviera otro dato
        return LowScan(
            ranges=ranges, angle_min=cfg.angle_min, angle_max=cfg.angle_max,
            angle_increment=cfg.angle_increment, range_min=cfg.range_min,
            range_max=cfg.range_max, num_candidate_points=n_candidates,
            num_filtered_points=n_filtered,
            stats={'valid_fraction': float(valid.mean()),
                   'observed_bins': int(observed.sum()),
                   'obstacle_bins': int(accepted.sum()),
                   'num_bins': nb})


def depth_to_low_scan(depth, k, t_base_cam, config=None):
    """Version sin estado: una imagen de profundidad -> un ``LowScan``."""
    return LowObstacleDetector(k, config).detect(depth, t_base_cam)


def benchmark(depth, k, t_base_cam, config=None, frames=50, warmup=5):
    """Tiempo por cuadro en ms: {'mean','median','p95','max','frames'} (reloj monotono)."""
    det = LowObstacleDetector(k, config)
    for _ in range(warmup):
        det.detect(depth, t_base_cam)
    times = []
    for _ in range(frames):
        t0 = time.perf_counter()
        det.detect(depth, t_base_cam)
        times.append((time.perf_counter() - t0) * 1e3)
    arr = np.asarray(times)
    return {'mean': float(arr.mean()), 'median': float(np.median(arr)),
            'p95': float(np.percentile(arr, 95)), 'max': float(arr.max()), 'frames': frames}
