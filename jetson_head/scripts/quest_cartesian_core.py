"""Teleoperacion CARTESIANA del Piper con la pose 6D de un control del Meta Quest 2 (logica pura, numpy).

Mientras el grip esta apretado, la pinza (TCP) sigue el movimiento RELATIVO del control desde el momento en
que se apreto (embrague): T_tcp = T_tcp0 con delta de posicion (escalado) y delta de rotacion del control.
Ejes: WebXR 'local' (x derecha, y arriba, -z adelante) -> robot base_link (x adelante, y izquierda, z arriba),
suponiendo que el operador mira hacia donde mira el robot.
Cinematica: DH del SDK de AgileX (mismo modelo que piper_kinematics.py, validado contra el URDF: 0,15 mm),
expresada en el marco del hombro con los ejes de base_link (montaje lateral rpy=-pi/2).
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

JOINT_LIMITS_DEG = np.array([(-154.0, 154.0), (0.0, 195.0), (-175.0, 0.0), (-110.0, 110.0), (-80.0, 80.0), (-180.0, 180.0)])
_A = [0.0, 0.0, 285.03, -21.98, 0.0, 0.0]
_ALPHA = [0.0, -math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2]
_THETA0 = [0.0, -math.pi * 172.22 / 180.0, -102.78 / 180.0 * math.pi, 0.0, 0.0, 0.0]
_D = [123.0, 0.0, 0.0, 250.75, 0.0, 91.0 + 135.8]      # TCP = yemas (link6 + 135.8 mm)
R_MOUNT = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
M_XR_TO_ROBOT = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # v_robot = M @ v_xr


def fk(q_rad: Sequence[float]):
    """TCP (mm) y rotacion en el marco del hombro con ejes de base_link."""
    T = np.eye(4)
    for i in range(6):
        ca, sa = math.cos(_ALPHA[i]), math.sin(_ALPHA[i])
        th = q_rad[i] + _THETA0[i]
        ct, st = math.cos(th), math.sin(th)
        T = T @ np.array([[ct, -st, 0, _A[i]], [st * ca, ct * ca, -sa, -sa * _D[i]],
                          [st * sa, ct * sa, ca, ca * _D[i]], [0, 0, 0, 1.0]])
    return R_MOUNT @ T[:3, 3], R_MOUNT @ T[:3, :3]


def quat_to_R(q):   # (x, y, z, w)
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    """Matriz de rotacion -> cuaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        k = math.sqrt(t + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / k, (R[0, 2] - R[2, 0]) / k, (R[1, 0] - R[0, 1]) / k, 0.25 * k)
    i = int(np.argmax(np.diag(R)))
    j, m = (i + 1) % 3, (i + 2) % 3
    k = math.sqrt(1.0 + R[i, i] - R[j, j] - R[m, m]) * 2
    q = [0.0, 0.0, 0.0, (R[m, j] - R[j, m]) / k]
    q[i], q[j], q[m] = 0.25 * k, (R[j, i] + R[i, j]) / k, (R[m, i] + R[i, m]) / k
    return tuple(q)


def rot_err(R, Rt):
    E = R.T @ Rt
    ang = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1) / 2)))
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]]) / (2 * math.sin(ang))
    return R @ (v * ang)   # en el marco del mundo


def ik_step(q_deg, p_target_mm, R_target, use_rot=True, iters=6, rot_w=200.0, damping=8.0):
    """Mínimos cuadrados amortiguados desde q (grados). Devuelve (q_nuevo_grados, error_mm, error_grados)."""
    q = np.radians(np.asarray(q_deg, float))
    lo, hi = np.radians(JOINT_LIMITS_DEG[:, 0] + 1.0), np.radians(JOINT_LIMITS_DEG[:, 1] - 1.0)
    for _ in range(iters):
        p, R = fk(q)
        e = np.concatenate([p_target_mm - p, rot_w * rot_err(R, R_target) if use_rot else np.zeros(3)])
        J = np.zeros((6, 6))
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = 1e-4
            p2, R2 = fk(q + dq)
            J[:3, j] = (p2 - p) / 1e-4
            J[3:, j] = rot_w * rot_err(R, R2) / 1e-4 if use_rot else 0.0
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), e)
        q = np.clip(q + dq, lo, hi)
    p, R = fk(q)
    return np.degrees(q), float(np.linalg.norm(p_target_mm - p)), math.degrees(np.linalg.norm(rot_err(R, R_target)))


@dataclass
class CartesianState:
    engaged: bool = False
    use_rot: bool = True
    prev_x: bool = False
    c0_p: Optional[np.ndarray] = None
    c0_R: Optional[np.ndarray] = None
    t0_p: Optional[np.ndarray] = None
    t0_R: Optional[np.ndarray] = None
    q_cmd: Optional[List[float]] = None
    target_p: Optional[np.ndarray] = None
    target_R: Optional[np.ndarray] = None
    err_mm: float = 0.0
    err_deg: float = 0.0
    gripper_mm: float = 90.0
    reason: str = "sin datos"
    log: List[str] = field(default_factory=list)


def scale_rotation(R, k):
    """Misma rotacion (eje) con el angulo multiplicado por k (sensibilidad de giro)."""
    ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
    if ang < 1e-9 or abs(k - 1.0) < 1e-9:
        return R
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(ang))
    a = ang * k
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K


def cart_step(st: CartesianState, pose, buttons, measured_deg, now_age_s, dt,
              scale=1.0, max_joint_dps=45.0, timeout_s=0.3, grip_open=90.0, grip_closed=10.0,
              rot_scale=1.0, max_wrist_dps=None):
    """pose = (pos_xyz_m, quat_xyzw) del control en WebXR. Devuelve st con q_cmd (None = no mandar)."""
    if measured_deg is None:
        st.engaged, st.q_cmd, st.reason = False, None, "sin feedback del brazo"
        return st
    if pose is None or now_age_s > timeout_s:
        st.engaged, st.q_cmd, st.reason = False, None, "sin datos del visor (timeout)"
        return st
    b = list(buttons) + [0.0] * 6
    x_pressed = b[4] > 0.5
    if x_pressed and not st.prev_x:
        st.use_rot = not st.use_rot
        st.log.append("orientacion " + ("SEGUIDA" if st.use_rot else "FIJA (solo posicion)"))
    st.prev_x = x_pressed
    st.gripper_mm = grip_open + (grip_closed - grip_open) * max(0.0, min(1.0, b[0]))
    pos = np.asarray(pose[0], float) * 1000.0
    Rc = quat_to_R(pose[1])
    if b[1] < 0.5:
        st.engaged, st.q_cmd, st.reason = False, None, "grip suelto (embrague abierto)"
        return st
    if not st.engaged:                         # embragar: referencias desde lo MEDIDO
        st.engaged = True
        st.c0_p, st.c0_R = pos, Rc
        st.t0_p, st.t0_R = fk(np.radians(measured_deg))
        st.q_cmd = list(measured_deg)
    dp = M_XR_TO_ROBOT @ (pos - st.c0_p) * scale
    dR = scale_rotation(M_XR_TO_ROBOT @ (Rc @ st.c0_R.T) @ M_XR_TO_ROBOT.T, rot_scale)
    st.target_p = st.t0_p + dp
    st.target_R = dR @ st.t0_R if st.use_rot else st.t0_R
    q_new, st.err_mm, st.err_deg = ik_step(st.q_cmd, st.target_p, st.target_R, use_rot=True)
    # velocidad maxima por articulacion; la muneca (j4..j6) puede ir mas rapido para que el giro responda
    lim = np.array([max_joint_dps] * 3 + [max_wrist_dps or max_joint_dps] * 3) * dt
    q_prev = np.asarray(st.q_cmd, float)
    q_new = q_prev + np.clip(q_new - q_prev, -lim, lim)
    q_new = np.minimum(np.asarray(measured_deg) + 15.0, np.maximum(np.asarray(measured_deg) - 15.0, q_new))
    st.q_cmd = [float(v) for v in q_new]
    st.reason = (f"siguiendo el control (error {st.err_mm:.0f} mm / {st.err_deg:.0f} grados"
                 + (", FUERA DE ALCANCE" if st.err_mm > 15.0 else "") + ")")
    return st
