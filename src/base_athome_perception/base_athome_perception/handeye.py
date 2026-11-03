"""Calibracion hand-eye eye-in-hand (camara de muneca DaBai en la pinza del Piper). Nucleo puro, sin ROS.

Cadena:  T_base_target = T_base_tcp(q_i) @ X @ T_cam_target_i      (X = T_tcp_cam, lo que se busca)
  - T_base_tcp(q_i): FK con las articulaciones MEDIDAS por CAN (nunca las comandadas).
  - T_cam_target_i: pose del ArUco por solvePnP (intrinsecos calibrados de la camara).
Un X correcto deja T_base_target IGUAL en todas las muestras (el marcador no se mueve): la dispersion
de esa pose es el residual que se reporta (mm / grados). Una calibracion solo se acepta si el residual
y la validacion independiente (posiciones medidas con cinta/regla) estan dentro de tolerancia.
Referencias: OpenCV calibrateHandEye (Tsai, Park, Horaud, Andreff, Daniilidis).
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

METHODS = {
    "tsai": cv2.CALIB_HAND_EYE_TSAI, "park": cv2.CALIB_HAND_EYE_PARK, "horaud": cv2.CALIB_HAND_EYE_HORAUD,
    "andreff": cv2.CALIB_HAND_EYE_ANDREFF, "daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
}
MIN_SAMPLES = 8
MIN_ROTATION_SPREAD_DEG = 20.0   # sin rotaciones variadas el problema queda mal condicionado


def make_T(R, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, float)
    T[:3, 3] = np.asarray(t, float).reshape(3)
    return T


def rot_angle_deg(R) -> float:
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))))


def marker_pose(corners_px: np.ndarray, marker_len_m: float, K: np.ndarray, dist: np.ndarray
                ) -> Tuple[np.ndarray, float]:
    """T_cam_marker y error de reproyeccion RMS (px) de un marcador cuadrado (esquinas en orden ArUco)."""
    h = marker_len_m / 2.0
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    img = np.asarray(corners_px, dtype=np.float64).reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        raise ValueError("solvePnP fallo")
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    rms = float(np.sqrt(np.mean(np.sum((proj.reshape(4, 2) - img) ** 2, axis=1))))
    return make_T(cv2.Rodrigues(rvec)[0], tvec), rms


@dataclass
class HandEyeResult:
    method: str
    T_tcp_cam: np.ndarray
    residual_mm: float           # dispersion (RMS) de la posicion del marcador en base
    residual_deg: float          # dispersion (RMS) de su orientacion
    target_in_base: np.ndarray   # pose media del marcador en base
    per_sample_mm: List[float] = field(default_factory=list)

    def to_dict(self) -> Dict:
        R, t = self.T_tcp_cam[:3, :3], self.T_tcp_cam[:3, 3]
        q = cv2.Rodrigues(R)[0].reshape(3)
        return dict(method=self.method, xyz_m=[round(float(v), 5) for v in t],
                    rotvec_rad=[round(float(v), 6) for v in q],
                    residual_mm=round(self.residual_mm, 2), residual_deg=round(self.residual_deg, 3),
                    n_samples=len(self.per_sample_mm))


def check_samples(T_base_tcp: Sequence[np.ndarray]) -> List[str]:
    """Problemas de la toma de muestras (vacia = suficiente para resolver)."""
    issues = []
    if len(T_base_tcp) < MIN_SAMPLES:
        issues.append(f"pocas muestras ({len(T_base_tcp)} < {MIN_SAMPLES})")
    if len(T_base_tcp) >= 2:
        R0 = T_base_tcp[0][:3, :3]
        spread = max(rot_angle_deg(R0.T @ T[:3, :3]) for T in T_base_tcp)
        if spread < MIN_ROTATION_SPREAD_DEG:
            issues.append(f"rotaciones poco variadas ({spread:.1f} < {MIN_ROTATION_SPREAD_DEG} grados)")
    return issues


def evaluate(T_base_tcp, T_cam_target, X) -> Tuple[float, float, np.ndarray, List[float]]:
    poses = [Tb @ X @ Tc for Tb, Tc in zip(T_base_tcp, T_cam_target)]
    P = np.array([p[:3, 3] for p in poses])
    mean_t = P.mean(axis=0)
    per = [float(np.linalg.norm(p - mean_t) * 1000.0) for p in P]
    U, _, Vt = np.linalg.svd(sum(p[:3, :3] for p in poses))
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:
        U[:, -1] *= -1
        R_mean = U @ Vt
    ang = [rot_angle_deg(R_mean.T @ p[:3, :3]) for p in poses]
    return (float(np.sqrt(np.mean(np.square(per)))), float(np.sqrt(np.mean(np.square(ang)))),
            make_T(R_mean, mean_t), per)


def solve(T_base_tcp: Sequence[np.ndarray], T_cam_target: Sequence[np.ndarray],
          methods: Sequence[str] = tuple(METHODS)) -> List[HandEyeResult]:
    """Resuelve con todos los metodos y los ordena por residual (el primero es el mejor)."""
    if len(T_base_tcp) != len(T_cam_target):
        raise ValueError("listas de distinto largo")
    issues = check_samples(T_base_tcp)
    if issues:
        raise ValueError("; ".join(issues))
    Rg = [T[:3, :3] for T in T_base_tcp]
    tg = [T[:3, 3] for T in T_base_tcp]
    Rt = [T[:3, :3] for T in T_cam_target]
    tt = [T[:3, 3] for T in T_cam_target]
    out = []
    for name in methods:
        R, t = cv2.calibrateHandEye(Rg, tg, Rt, tt, method=METHODS[name])
        X = make_T(R, t)
        mm, deg, target, per = evaluate(T_base_tcp, T_cam_target, X)
        out.append(HandEyeResult(name, X, mm, deg, target, per))
    return sorted(out, key=lambda r: (r.residual_mm, r.residual_deg))


def validate_points(T_base_tcp: np.ndarray, X: np.ndarray, p_cam: Sequence[float],
                    p_base_measured: Sequence[float]) -> float:
    """Validacion independiente: punto visto por la camara vs su posicion MEDIDA en base. Error en mm."""
    p = T_base_tcp @ X @ np.append(np.asarray(p_cam, float), 1.0)
    return float(np.linalg.norm(p[:3] - np.asarray(p_base_measured, float)) * 1000.0)


def accept(result: HandEyeResult, validation_mm: Optional[Sequence[float]],
           max_residual_mm: float = 5.0, max_residual_deg: float = 1.0, max_validation_mm: float = 10.0
           ) -> Tuple[bool, str]:
    """Una calibracion NO es valida sin validacion independiente."""
    if result.residual_mm > max_residual_mm or result.residual_deg > max_residual_deg:
        return False, f"residual {result.residual_mm:.1f} mm / {result.residual_deg:.2f} grados fuera de tolerancia"
    if not validation_mm:
        return False, "falta la validacion independiente con posiciones medidas"
    worst = max(validation_mm)
    if worst > max_validation_mm:
        return False, f"validacion: peor error {worst:.1f} mm > {max_validation_mm} mm"
    return True, f"OK: residual {result.residual_mm:.1f} mm, validacion peor {worst:.1f} mm"
