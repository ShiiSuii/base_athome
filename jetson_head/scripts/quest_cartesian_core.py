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

# Limites REALES para la IK: medidos el 2026-11-04 con el brazo real saturando en j2=180, j3=-170, j4=100, j5=-70
# (ficha del fabricante; el URDF es mas amplio). j1: ficha. j6: se midio -159, mas alla de los +-120 de la ficha.
JOINT_LIMITS_DEG = np.array([(-150.0, 150.0), (0.0, 180.0), (-170.0, 0.0), (-100.0, 100.0), (-70.0, 70.0), (-180.0, 180.0)])
_A = [0.0, 0.0, 285.03, -21.98, 0.0, 0.0]
_ALPHA = [0.0, -math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2]
_THETA0 = [0.0, -math.pi * 172.22 / 180.0, -102.78 / 180.0 * math.pi, 0.0, 0.0, 0.0]
_D = [123.0, 0.0, 0.0, 250.75, 0.0, 91.0 + 135.8]      # TCP = yemas (link6 + 135.8 mm)
R_MOUNT = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])
M_XR_TO_ROBOT = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])   # v_robot = M @ v_xr


def heading_yaw(R_xr) -> float:
    """Rumbo (rad, giro sobre +Y de WebXR) del eje 'adelante' del control (-Z), proyectado en el plano horizontal."""
    f = R_xr @ np.array([0.0, 0.0, -1.0])
    return math.atan2(-f[0], -f[2])


def yaw_matrix(yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


TCP_D6_MM = _D[5]      # d6 de las yemas; 0 = centro de la muneca (cruce de j4, j5, j6)


def fk(q_rad: Sequence[float], d6: Optional[float] = None):
    """Punto de control (mm; por defecto TCP, d6=0: centro de muneca) y rotacion, en el marco del hombro."""
    T = np.eye(4)
    for i in range(6):
        ca, sa = math.cos(_ALPHA[i]), math.sin(_ALPHA[i])
        th = q_rad[i] + _THETA0[i]
        ct, st = math.cos(th), math.sin(th)
        d = _D[i] if (i < 5 or d6 is None) else d6
        T = T @ np.array([[ct, -st, 0, _A[i]], [st * ca, ct * ca, -sa, -sa * d],
                          [st * sa, ct * sa, ca, ca * d], [0, 0, 0, 1.0]])
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


class OneEuro:
    """Filtro 1-euro (Casiez et al.): corte bajo con la mano lenta (sin temblor), alto con la mano rapida (sin retraso)."""

    def __init__(self, mincutoff=1.2, beta=0.03, dcutoff=1.0):
        self.mincutoff, self.beta, self.dcutoff = mincutoff, beta, dcutoff
        self.dx = 0.0

    @staticmethod
    def alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def gain(self, speed, dt):
        """Ganancia (0..1) para esta muestra segun la velocidad medida (unidades/s)."""
        self.dx += self.alpha(self.dcutoff, dt) * (speed - self.dx)
        return self.alpha(self.mincutoff + self.beta * abs(self.dx), dt)


def adaptive_damping(J, d_min, d_max, sigma0=40.0):
    """Amortiguacion segun el menor valor singular de J: minima lejos de singularidades, maxima cerca."""
    smin = float(np.linalg.svd(J, compute_uv=False)[-1])
    if smin >= sigma0:
        return d_min
    return d_min + (d_max - d_min) * (1.0 - (smin / sigma0) ** 2)


def ik_step(q_deg, p_target_mm, R_target, use_rot=True, iters=6, rot_w=200.0, damping=8.0, damping_min=None, d6=None):
    """Mínimos cuadrados amortiguados desde q (grados). Devuelve (q_nuevo_grados, error_mm, error_grados)."""
    q = np.radians(np.asarray(q_deg, float))
    lo, hi = np.radians(JOINT_LIMITS_DEG[:, 0] + 2.0), np.radians(JOINT_LIMITS_DEG[:, 1] - 2.0)
    for _ in range(iters):
        p, R = fk(q, d6)
        e = np.concatenate([p_target_mm - p, rot_w * rot_err(R, R_target) if use_rot else np.zeros(3)])
        J = np.zeros((6, 6))
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = 1e-4
            p2, R2 = fk(q + dq, d6)
            J[:3, j] = (p2 - p) / 1e-4
            J[3:, j] = rot_w * rot_err(R, R2) / 1e-4 if use_rot else 0.0
        lam = damping if damping_min is None else adaptive_damping(J, damping_min, damping)
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        q = np.clip(q + dq, lo, hi)
    p, R = fk(q, d6)
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
    gripper_half: bool = False      # gatillo: alterna abierta <-> 50 %
    prev_trig: bool = False
    f_p: Optional[np.ndarray] = None   # pose del control FILTRADA (quita el temblor de la mano)
    f_R: Optional[np.ndarray] = None
    euro_p: Optional[OneEuro] = None
    euro_r: Optional[OneEuro] = None
    forward_yaw: float = 0.0        # calibracion: rumbo del control que corresponde al +X (adelante) del robot
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
              rot_scale=1.0, max_wrist_dps=None, max_lead_deg=15.0, ik_damping=80.0,
              smooth=0.35, deadband_mm=1.5, deadband_deg=0.7, grip_toggle=False, grip_half=45.0,
              one_euro=False, euro_mincutoff=1.2, euro_beta=0.03, ik_iters=10, damping_min=None,
              max_excess_mm=30.0, max_excess_deg=20.0, pivot_d6=None):
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
    if grip_toggle:                          # gatillo: cada pulsacion alterna abierta <-> 50 %
        trig = b[0] > 0.5
        if trig and not st.prev_trig:
            st.gripper_half = not st.gripper_half
            st.log.append("pinza " + ("al 50 %" if st.gripper_half else "ABIERTA"))
        st.prev_trig = trig
        st.gripper_mm = grip_half if st.gripper_half else grip_open
    else:
        st.gripper_mm = grip_open + (grip_closed - grip_open) * max(0.0, min(1.0, b[0]))
    raw_p = np.asarray(pose[0], float) * 1000.0
    raw_R = quat_to_R(pose[1])
    # filtro exponencial de la pose (posicion lineal; rotacion por el angulo relativo) + zona muerta
    if st.f_p is None:
        st.f_p, st.f_R = raw_p, raw_R
        st.euro_p, st.euro_r = OneEuro(euro_mincutoff, euro_beta), OneEuro(euro_mincutoff, euro_beta * 20.0)
    elif one_euro:                           # mm/s y grados/s: la ganancia depende de la velocidad de la mano
        dpf = raw_p - st.f_p
        st.f_p = st.f_p + st.euro_p.gain(float(np.linalg.norm(dpf)) / dt, dt) * dpf
        dRf = raw_R @ st.f_R.T
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dRf) - 1) / 2))))
        st.f_R = scale_rotation(dRf, st.euro_r.gain(ang / dt, dt)) @ st.f_R
    else:
        dpf = raw_p - st.f_p
        st.f_p = st.f_p + (smooth if np.linalg.norm(dpf) > deadband_mm else 0.0) * dpf
        dRf = raw_R @ st.f_R.T
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dRf) - 1) / 2))))
        if ang > deadband_deg:
            st.f_R = scale_rotation(dRf, smooth) @ st.f_R
    pos, Rc = st.f_p, st.f_R
    if b[1] < 0.5:
        st.engaged, st.q_cmd, st.reason = False, None, "grip suelto (embrague abierto)"
        st.f_p = st.f_R = None                 # al volver a agarrar, el filtro arranca de la pose actual
        return st
    if not st.engaged:                         # embragar: referencias desde lo MEDIDO
        st.engaged = True
        st.c0_p, st.c0_R = pos, Rc
        st.t0_p, st.t0_R = fk(np.radians(measured_deg), pivot_d6)
        st.q_cmd = list(measured_deg)
    # Ejes: WebXR -> "frente calibrado" -> robot. Sin calibrar, el frente es hacia donde miraba el visor al entrar en VR.
    M = M_XR_TO_ROBOT @ yaw_matrix(-st.forward_yaw)
    dp = M @ (pos - st.c0_p) * scale
    dR = scale_rotation(M @ (Rc @ st.c0_R.T) @ M.T, rot_scale)
    st.target_p = st.t0_p + dp
    st.target_R = dR @ st.t0_R if st.use_rot else st.t0_R
    # amortiguacion alta: cerca de la singularidad de muneca (j5 ~ 0, p.ej. en HOME) evita giros grandes y
    # opuestos de j4/j6 (medido: 40 grados con 8 -> 7 grados con 80, mismo error de la pinza ~1 mm)
    q_new, st.err_mm, st.err_deg = ik_step(st.q_cmd, st.target_p, st.target_R, use_rot=True, iters=ik_iters,
                                           damping=ik_damping, damping_min=damping_min, d6=pivot_d6)
    # velocidad maxima por articulacion; la muneca (j4..j6) puede ir mas rapido para que el giro responda
    # Anti-acumulacion: si el objetivo esta fuera de alcance, las referencias se corren para que quede pegado al
    # borde alcanzable. Asi, al volver, la pinza responde enseguida (antes habia que recuperar todo el exceso).
    pa, Ra = fk(np.radians(q_new), pivot_d6)
    e = st.target_p - pa
    en = float(np.linalg.norm(e))
    if en > max_excess_mm:
        st.t0_p = st.t0_p - e * (1.0 - max_excess_mm / en)
    if st.err_deg > max_excess_deg and st.use_rot:
        st.t0_R = scale_rotation(Ra @ st.target_R.T, 1.0 - max_excess_deg / st.err_deg) @ st.t0_R
    lim = np.array([max_joint_dps] * 3 + [max_wrist_dps or max_joint_dps] * 3) * dt
    q_prev = np.asarray(st.q_cmd, float)
    # Limites que conservan la DIRECCION: se escala todo el paso por igual (recortar cada articulacion por
    # separado deforma la trayectoria de la pinza).
    dq = q_new - q_prev
    k = min(1.0, float(np.min(lim / np.maximum(np.abs(dq), 1e-9))))
    q_new = q_prev + k * dq
    lead = q_new - np.asarray(measured_deg)
    peak = float(np.max(np.abs(lead)))
    if peak > max_lead_deg:
        q_new = np.asarray(measured_deg) + lead * (max_lead_deg / peak)
    st.q_cmd = [float(v) for v in q_new]
    st.reason = (f"siguiendo el control (error {st.err_mm:.0f} mm / {st.err_deg:.0f} grados"
                 + (", FUERA DE ALCANCE" if st.err_mm > 15.0 else "") + ")")
    return st
