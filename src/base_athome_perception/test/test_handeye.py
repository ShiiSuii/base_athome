"""Hand-eye eye-in-hand con datos sinteticos de solucion conocida."""
import numpy as np
import pytest
import cv2

from base_athome_perception import handeye as he


def rand_T(rng, rot_deg=40.0, trans=0.3):
    rv = rng.normal(size=3)
    rv = rv / np.linalg.norm(rv) * np.radians(rng.uniform(0, rot_deg))
    return he.make_T(cv2.Rodrigues(rv)[0], rng.uniform(-trans, trans, 3))


# Camara aproximada como la del URDF: 2,5 cm en x, 8 cm detras del TCP, girada 90 grados en z
X_TRUE = he.make_T(cv2.Rodrigues(np.array([0.0, 0.0, np.pi / 2]))[0], [0.025, 0.0, -0.08])


def synth(n=15, noise_mm=0.0, noise_deg=0.0, seed=1):
    rng = np.random.default_rng(seed)
    T_base_target = he.make_T(np.eye(3), [0.5, 0.0, 0.8])
    Tb, Tc = [], []
    for _ in range(n):
        T_base_tcp = he.make_T(np.eye(3), [0.4, 0.0, 1.0]) @ rand_T(rng, 35, 0.1)
        T_cam_target = np.linalg.inv(T_base_tcp @ X_TRUE) @ T_base_target
        if noise_mm or noise_deg:
            T_cam_target = T_cam_target @ he.make_T(
                cv2.Rodrigues(rng.normal(scale=np.radians(noise_deg), size=3))[0],
                rng.normal(scale=noise_mm / 1000.0, size=3))
        Tb.append(T_base_tcp)
        Tc.append(T_cam_target)
    return Tb, Tc, T_base_target


def test_recovers_known_transform_without_noise():
    Tb, Tc, _ = synth()
    best = he.solve(Tb, Tc)[0]
    assert np.allclose(best.T_tcp_cam, X_TRUE, atol=1e-6)
    assert best.residual_mm < 1e-3


def test_with_noise_residual_reflects_noise_and_estimate_is_close():
    Tb, Tc, _ = synth(n=20, noise_mm=1.0, noise_deg=0.3)
    best = he.solve(Tb, Tc)[0]
    err_mm = np.linalg.norm(best.T_tcp_cam[:3, 3] - X_TRUE[:3, 3]) * 1000
    assert err_mm < 5.0
    assert 0.3 < best.residual_mm < 6.0     # el ruido aparece en el residual, no se oculta


def test_rejects_too_few_or_degenerate_samples():
    Tb, Tc, _ = synth(n=5)
    with pytest.raises(ValueError, match="pocas muestras"):
        he.solve(Tb, Tc)
    same = [Tb[0] @ he.make_T(np.eye(3), [0.01 * i, 0, 0]) for i in range(10)]
    assert any("rotaciones poco variadas" in s for s in he.check_samples(same))


def test_accept_requires_independent_validation():
    Tb, Tc, T_bt = synth()
    best = he.solve(Tb, Tc)[0]
    ok, why = he.accept(best, None)
    assert not ok and "validacion independiente" in why
    p_cam = (np.linalg.inv(Tb[0] @ best.T_tcp_cam) @ T_bt)[:3, 3]
    e = he.validate_points(Tb[0], best.T_tcp_cam, p_cam, T_bt[:3, 3])
    assert e < 0.01
    assert he.accept(best, [e])[0]
    assert not he.accept(best, [25.0])[0]


def test_marker_pose_roundtrip():
    K = np.array([[485.0, 0, 320], [0, 485.0, 240], [0, 0, 1]])
    dist = np.zeros(5)
    T = he.make_T(cv2.Rodrigues(np.array([0.2, -0.1, 0.05]))[0], [0.02, -0.01, 0.4])
    h = 0.025
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]])
    img, _ = cv2.projectPoints(obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, dist)
    T_est, rms = he.marker_pose(img.reshape(4, 2), 0.05, K, dist)
    assert np.allclose(T_est, T, atol=1e-4) and rms < 1e-3
