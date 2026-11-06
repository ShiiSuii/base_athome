"""Tests de deteccion de obstaculos bajos desde profundidad (HOME-67)."""
import math

import numpy as np
import pytest

from base_athome_perception.low_obstacles import (
    DEFAULT_Z_MAX_M,
    DEFAULT_Z_MIN_M,
    LowObstacleConfig,
    LowObstacleDetector,
    benchmark,
    camera_pose_from_head,
    depth_to_low_scan,
)

# Intrinsecos tipicos de la RealSense D435 a 640x480
K_D435_640 = np.array([
    [605.0, 0.0, 320.0],
    [0.0, 605.0, 240.0],
    [0.0, 0.0, 1.0],
], dtype=np.float64)


def _ray_directions_in_base(shape, k, t_base_cam):
    """Origen y direcciones unitarias de rayos por pixel en el frame base_footprint."""
    h, w = shape
    v, u = np.mgrid[0:h, 0:w]
    xn = (u - k[0, 2]) / k[0, 0]
    yn = (v - k[1, 2]) / k[1, 1]
    # En frame optico el punto con profundidad optica z es z * [xn, yn, 1].
    # Por comodidad devolvemos el vector d_cam = [xn, yn, 1] rotado al frame base:
    # P_base(z) = origin + z * d_base, donde z es directamente la profundidad optica!
    r = t_base_cam[:3, :3]
    origin = t_base_cam[:3, 3]
    dx = r[0, 0] * xn + r[0, 1] * yn + r[0, 2]
    dy = r[1, 0] * xn + r[1, 1] * yn + r[1, 2]
    dz = r[2, 0] * xn + r[2, 1] * yn + r[2, 2]
    return origin, np.stack([dx, dy, dz], axis=-1)


def render_floor(shape, k, t_base_cam, max_depth=3.5):
    """Profundidad optica (en m, float32) de un piso plano z=0."""
    origin, d = _ray_directions_in_base(shape, k, t_base_cam)
    dz = d[..., 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        z = -origin[2] / dz
    valid = (dz < -1e-6) & (z > 0.1) & (z <= max_depth)
    return np.where(valid, z, 0.0).astype(np.float32)


def add_aabb(depth_m, k, t_base_cam, x_range, y_range, z_range):
    """Agrega una caja alineada a los ejes en base_footprint (interseccion rayo-AABB)."""
    origin, d = _ray_directions_in_base(depth_m.shape, k, t_base_cam)
    bounds_min = np.array([x_range[0], y_range[0], z_range[0]], dtype=np.float64)
    bounds_max = np.array([x_range[1], y_range[1], z_range[1]], dtype=np.float64)

    tmin = np.full(depth_m.shape, -np.inf, dtype=np.float64)
    tmax = np.full(depth_m.shape, np.inf, dtype=np.float64)

    for axis in range(3):
        da = d[..., axis]
        oa = origin[axis]
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (bounds_min[axis] - oa) / da
            t2 = (bounds_max[axis] - oa) / da
        t_near = np.minimum(t1, t2)
        t_far = np.maximum(t1, t2)
        parallel_out = (np.abs(da) < 1e-9) & ((oa < bounds_min[axis]) | (oa > bounds_max[axis]))
        tmin = np.where(parallel_out, np.inf, np.maximum(tmin, t_near))
        tmax = np.where(parallel_out, -np.inf, np.minimum(tmax, t_far))

    hit = (tmax >= np.maximum(tmin, 0.05)) & (tmin > 0.05)
    out = depth_m.copy()
    cur = np.where(out > 0.0, out, np.inf)
    best = np.where(hit & (tmin < cur), tmin, cur)
    return np.where(np.isfinite(best), best, 0.0).astype(np.float32)


def add_sphere(depth_m, k, t_base_cam, center, radius):
    """Agrega una esfera/bulto (ej. bolsa en el piso) por interseccion rayo-esfera."""
    origin, d = _ray_directions_in_base(depth_m.shape, k, t_base_cam)
    oc = origin - np.asarray(center, dtype=np.float64)
    a = np.sum(d * d, axis=-1)
    b = 2.0 * np.sum(d * oc, axis=-1)
    c = float(np.dot(oc, oc) - radius * radius)
    disc = b * b - 4.0 * a * c
    hit = disc >= 0.0
    sqrt_disc = np.sqrt(np.maximum(disc, 0.0))
    t = (-b - sqrt_disc) / (2.0 * a)
    hit &= t > 0.05
    out = depth_m.copy()
    cur = np.where(out > 0.0, out, np.inf)
    best = np.where(hit & (t < cur), t, cur)
    return np.where(np.isfinite(best), best, 0.0).astype(np.float32)


def to_uint16_mm(depth_m):
    return np.clip(np.round(depth_m * 1000.0), 0, 65535).astype(np.uint16)


def test_floor_only_produces_zero_obstacles():
    """Escena 1: piso plano solo -> 0 bins con obstaculo, bins centrales libres (+inf)."""
    t_cam = camera_pose_from_head(height=1.15, pitch_down=math.radians(32.0), x=0.20)
    depth_mm = to_uint16_mm(render_floor((480, 640), K_D435_640, t_cam))
    scan = depth_to_low_scan(depth_mm, K_D435_640, t_cam)

    assert scan.stats['obstacle_bins'] == 0
    assert scan.nearest() is None
    assert scan.stats['observed_bins'] > 20
    # Los bins dentro del FOV frontal deben ser +inf (observados libres) y fuera del FOV NaN
    center_idx = int((0.0 - scan.angle_min) / scan.angle_increment)
    assert math.isinf(scan.ranges[center_idx])
    back_idx = 0  # -pi rad (atras del robot)
    assert math.isnan(scan.ranges[back_idx])


def test_low_box_detected_at_correct_range_and_angle():
    """Escena 2: caja baja (18 cm de alto, bajo el lidar de 38 cm) a x=1.0 m frente al robot."""
    t_cam = camera_pose_from_head(height=1.15, pitch_down=math.radians(30.0), x=0.20)
    depth_m = render_floor((480, 640), K_D435_640, t_cam)
    # Caja de x=0.95..1.15 m, y=-0.12..0.12 m, z=0.0..0.18 m
    depth_m = add_aabb(depth_m, K_D435_640, t_cam, (0.95, 1.15), (-0.12, 0.12), (0.0, 0.18))
    scan = depth_to_low_scan(to_uint16_mm(depth_m), K_D435_640, t_cam)

    nearest = scan.nearest()
    assert nearest is not None
    r_min, ang_min = nearest
    assert 0.93 <= r_min <= 1.02
    assert abs(ang_min) <= math.radians(8.0)
    assert scan.stats['obstacle_bins'] >= 5


def test_tall_obstacle_above_lidar_band_ignored():
    """Puntos por encima de 38 cm (que ya ve el lidar) no contaminan el barrido bajo."""
    t_cam = camera_pose_from_head(height=1.15, pitch_down=math.radians(30.0), x=0.20)
    depth_m = render_floor((480, 640), K_D435_640, t_cam)
    # Tablero flotante entre z=0.45 y z=0.80 m (ej. tapa de mesa sin patas en esa zona)
    depth_m = add_aabb(depth_m, K_D435_640, t_cam, (0.90, 1.10), (-0.20, 0.20), (0.45, 0.80))
    scan = depth_to_low_scan(to_uint16_mm(depth_m), K_D435_640, t_cam)

    assert scan.stats['obstacle_bins'] == 0
    assert scan.nearest() is None


def test_bag_on_floor_off_center():
    """Escena 3: bolsa en el piso desplazada a la izquierda (x=1.20 m, y=+0.30 m, alto ~22 cm)."""
    t_cam = camera_pose_from_head(height=1.15, pitch_down=math.radians(32.0), x=0.20)
    depth_m = render_floor((480, 640), K_D435_640, t_cam)
    # Bolsa modelada como bulto esferico de radio 0.11 m apoyado en el piso (centro z=0.11 m)
    depth_m = add_sphere(depth_m, K_D435_640, t_cam, center=(1.20, 0.30, 0.11), radius=0.11)
    scan = depth_to_low_scan(to_uint16_mm(depth_m), K_D435_640, t_cam)

    nearest = scan.nearest()
    assert nearest is not None
    r_min, ang_min = nearest
    expected_r = math.hypot(1.20, 0.30) - 0.11
    expected_ang = math.atan2(0.30, 1.20)
    assert abs(r_min - expected_r) < 0.05
    assert abs(ang_min - expected_ang) < math.radians(5.0)
    assert scan.stats['obstacle_bins'] >= 4


def test_tilted_head_pan_tilt_and_roll_compensation():
    """Escena 4: cabeza inclinada con pitch, yaw y roll -> T_base_cam compensa y ubica igual la caja."""
    t_cam = camera_pose_from_head(
        height=1.10,
        pitch_down=math.radians(36.0),
        roll=math.radians(7.0),
        yaw=math.radians(-12.0),
        x=0.18,
        y=-0.02,
    )
    depth_m = render_floor((480, 640), K_D435_640, t_cam)
    # Caja baja frente a la base en (x=0.90..1.05, y=-0.25..-0.05, z=0..0.16)
    depth_m = add_aabb(depth_m, K_D435_640, t_cam, (0.90, 1.05), (-0.25, -0.05), (0.0, 0.16))
    scan = depth_to_low_scan(to_uint16_mm(depth_m), K_D435_640, t_cam)

    nearest = scan.nearest()
    assert nearest is not None
    r_min, ang_min = nearest
    expected_r = math.hypot(0.90, -0.15)
    expected_ang = math.atan2(-0.15, 0.90)
    assert abs(r_min - expected_r) < 0.06
    assert abs(ang_min - expected_ang) < math.radians(6.0)


def test_depth_with_holes_nans_infs_and_speckle_noise():
    """Escena 5: profundidad con huecos (0, NaN, inf) y pixeles aislados de ruido (speckle)."""
    rng = np.random.default_rng(42)
    t_cam = camera_pose_from_head(height=1.15, pitch_down=math.radians(30.0), x=0.20)
    depth_m = render_floor((480, 640), K_D435_640, t_cam)
    depth_m = add_aabb(depth_m, K_D435_640, t_cam, (1.00, 1.15), (-0.10, 0.10), (0.0, 0.20))

    # Huecos grandes sin dato (0, NaN, +inf) + banda ciega a la derecha
    holes = rng.random(depth_m.shape) < 0.25
    depth_m[holes] = 0.0
    depth_m[100:140, 50:120] = np.nan
    depth_m[140:180, 50:120] = np.inf
    depth_m[:, 520:] = 0.0  # sector derecho ciego -> debe quedar NaN, no libre

    # Puntos aislados espurios ("flying pixels") a altura de obstaculo pero espaciados cada 6 px
    for v in range(60, 420, 12):
        for u in range(60, 500, 12):
            depth_m[v, u] = 0.85

    cfg = LowObstacleConfig(depth_scale=1.0, min_neighbors=2, min_points_per_bin=3)
    scan = depth_to_low_scan(depth_m, K_D435_640, t_cam, config=cfg)

    # La caja real sigue detectandose pese al 25 % de huecos
    nearest = scan.nearest()
    assert nearest is not None
    assert 0.98 <= nearest[0] <= 1.08

    # Ningun bin fuera de la caja central debe dispararse por los flying pixels aislados
    obs_angles = scan.angles[scan.obstacle_mask]
    assert np.all(np.abs(obs_angles) < math.radians(10.0))


def test_temporal_persistence_filter_rejects_single_frame_glitches():
    """El filtro de persistencia temporal exige ver el bin en M de N cuadros."""
    t_cam = camera_pose_from_head(height=1.15, pitch_down=math.radians(30.0), x=0.20)
    floor_mm = to_uint16_mm(render_floor((480, 640), K_D435_640, t_cam))
    box_m = add_aabb(
        render_floor((480, 640), K_D435_640, t_cam),
        K_D435_640, t_cam, (0.95, 1.10), (-0.10, 0.10), (0.0, 0.18),
    )
    box_mm = to_uint16_mm(box_m)

    cfg = LowObstacleConfig(persistence_window=3, persistence_min_hits=2)
    det = LowObstacleDetector(K_D435_640, cfg)

    # Cuadro 1 con caja: aun no alcanza 2 de 3 -> 0 obstaculos aceptados
    s1 = det.detect(box_mm, t_cam)
    assert s1.stats['obstacle_bins'] == 0

    # Cuadro 2 solo piso: sigue con 1 hit en la ventana -> 0 obstaculos
    s2 = det.detect(floor_mm, t_cam)
    assert s2.stats['obstacle_bins'] == 0

    # Cuadro 3 con caja: ahora tiene 2 hits en los ultimos 3 cuadros -> se acepta
    s3 = det.detect(box_mm, t_cam)
    assert s3.stats['obstacle_bins'] > 0

    det.reset()
    s4 = det.detect(box_mm, t_cam)
    assert s4.stats['obstacle_bins'] == 0


def test_floor_margin_per_m_compensates_pitch_calibration_error():
    """floor_margin_per_m absorbe un error pequeno de pitch para que el piso lejano no dispare falsos positivos."""
    t_real = camera_pose_from_head(height=1.15, pitch_down=math.radians(28.0), x=0.20)
    # La camara mide un piso plano con pitch real de 28 deg, pero TF cree que esta a 26.2 deg (+1.8 deg de error):
    # a 1.8 m de distancia el piso parece levantarse ~5.6 cm (> z_min=4 cm).
    t_wrong = camera_pose_from_head(height=1.15, pitch_down=math.radians(26.2), x=0.20)
    depth_mm = to_uint16_mm(render_floor((480, 640), K_D435_640, t_real))

    scan_no_margin = depth_to_low_scan(
        depth_mm, K_D435_640, t_wrong, LowObstacleConfig(floor_margin_per_m=0.0)
    )
    assert scan_no_margin.stats['obstacle_bins'] > 0

    scan_with_margin = depth_to_low_scan(
        depth_mm, K_D435_640, t_wrong, LowObstacleConfig(floor_margin_per_m=0.025)
    )
    assert scan_with_margin.stats['obstacle_bins'] == 0


def test_validation_and_benchmark_budget():
    """Valida chequeos de entrada y mide el tiempo por cuadro en 640x480."""
    with pytest.raises(ValueError):
        LowObstacleConfig(z_min=0.40, z_max=0.20).validate()
    with pytest.raises(ValueError):
        LowObstacleDetector(np.eye(3), LowObstacleConfig(stride=0))

    t_cam = camera_pose_from_head(height=1.15, pitch_down=math.radians(30.0), x=0.20)
    with pytest.raises(ValueError):
        depth_to_low_scan(np.zeros((10, 10), dtype=np.uint16), K_D435_640, np.ones((4, 4)))

    depth_m = render_floor((480, 640), K_D435_640, t_cam)
    depth_m = add_aabb(depth_m, K_D435_640, t_cam, (0.95, 1.15), (-0.12, 0.12), (0.0, 0.18))
    depth_mm = to_uint16_mm(depth_m)

    stats = benchmark(depth_mm, K_D435_640, t_cam, frames=50, warmup=5)
    stats_s1 = benchmark(
        depth_mm, K_D435_640, t_cam, config=LowObstacleConfig(stride=1), frames=30, warmup=5
    )
    stats_s4 = benchmark(
        depth_mm, K_D435_640, t_cam, config=LowObstacleConfig(stride=4), frames=50, warmup=5
    )
    print(
        f'\n[BENCHMARK 640x480 NUC] '
        f'stride=1: mean={stats_s1["mean"]:.2f}ms median={stats_s1["median"]:.2f}ms '
        f'p95={stats_s1["p95"]:.2f}ms max={stats_s1["max"]:.2f}ms | '
        f'stride=2 (default): mean={stats["mean"]:.2f}ms median={stats["median"]:.2f}ms '
        f'p95={stats["p95"]:.2f}ms max={stats["max"]:.2f}ms | '
        f'stride=4: mean={stats_s4["mean"]:.2f}ms median={stats_s4["median"]:.2f}ms '
        f'p95={stats_s4["p95"]:.2f}ms max={stats_s4["max"]:.2f}ms'
    )
    assert DEFAULT_Z_MIN_M == pytest.approx(0.04)
    assert DEFAULT_Z_MAX_M == pytest.approx(0.38)
    assert stats['frames'] == 50
    assert stats['mean'] < 50.0  # en la NUC con stride=2 toma < 10 ms (~15 Hz son 66.7 ms)
