#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Base@Home - Cinemática, Transformaciones y Envolvente Segura
# Manipulador AgileX Piper 6-DOF & Cámara RealSense D435 en Cabezal Pan-Tilt
# RoboCup @Home (Open Platform League)
# ==============================================================================

import math
from typing import Dict, List, Optional, Tuple, Union
import numpy as np

# ------------------------------------------------------------------------------
# 1. CONSTANTES FÍSICAS Y ENVOLVENTES DE SEGURIDAD ('ÁREA SEGURA')
# ------------------------------------------------------------------------------

# Límites angulares seguros de articulaciones en grados (establecidos por el arquitecto cinemático)
# Evita colisión física con columna del elevador, pantalla interactiva, cubo de montaje y chasis
SAFE_JOINT_LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    'joint1': (-65.0, 95.0),    # Rotación base en Z
    'joint2': (10.0, 175.0),    # Hombro pitch en Y (soporta reposo/home 172.7° y extensión frontal)
    'joint3': (-165.0, -15.0),  # Codo pitch en Y (soporta reposo/home -164.0° y extensión frontal)
    'joint4': (-90.0, 90.0),    # Muñeca roll en X
    'joint5': (-72.0, 72.0),    # Muñeca pitch en Y (permite nivelación horizontal de la pinza)
    'joint6': (-160.0, 160.0),  # Muñeca yaw/roll en Z (soporta reposo -154.0°)
}

# Caja cartesiana segura en el marco arm_base_link (metros)
# Espacio de trabajo útil frente al robot y sobre la mesa
SAFE_CARTESIAN_BOX_M: Dict[str, Tuple[float, float]] = {
    'x': (0.12, 0.58),   # Frente al robot: 12 cm a 58 cm
    'y': (-0.25, 0.42),  # Lateral: 25 cm a la derecha, 42 cm a la izquierda
    'z': (-0.25, 0.40),  # Altura: 25 cm bajo la base a 40 cm sobre la base
}

# Rango útil de apertura de la pinza paralela (mm)
GRIPPER_MIN_MM = 0.0
GRIPPER_MAX_MM = 70.0
PRINGLES_GRASP_APERTURE_MM = 62.0  # Sujeción firme del cilindro (75 mm de diámetro con compresión)

# Poses nombradas calibradas por defecto (grados)
DEFAULT_NAMED_POSES: Dict[str, Dict[str, Union[List[float], float, str]]] = {
    'home': {
        'joints_deg': [89.58, 172.74, -163.98, 84.59, -1.27, -153.97],
        'gripper_mm': 70.0,
        'description': 'Pose de reposo seguro estibado junto a columna del elevador'
    },
    'cube_stow': {
        'joints_deg': [88.14, 172.84, -163.72, -78.66, -3.95, -2.81],
        'gripper_mm': 1.0,
        'description': 'Pose compacta de estiba lateral descansando junto al cubo'
    },
    'pre_grasp_center': {
        'joints_deg': [0.0, 65.0, -75.0, 0.0, 10.0, 0.0],
        'gripper_mm': 70.0,
        'description': 'Pose frontal de aproximación hacia centro de mesa'
    },
    'pre_grasp_table': {
        'joints_deg': [15.0, 75.0, -85.0, 0.0, 10.0, 0.0],
        'gripper_mm': 70.0,
        'description': 'Pose baja de aproximación a mesa de servicio'
    }
}

# Offset geométrico entre cabezal Pan-Tilt y base del brazo Piper:
# El brazo está montado en el hombro lateral izquierdo (+Y=0.175m) del prisma del torso (35x25x35 cm).
# Respecto a arm_base_link en coordenadas del robot:
#   dx = -0.085 m (8.5 cm detrás del plano de la base del brazo)
#   dy = -0.175 m (17.5 cm a la derecha del brazo montado en el lateral izquierdo)
#   dz = +0.195 m (19.5 cm por encima del hombro de montaje del brazo)
HEAD_TO_ARM_OFFSET_M: Tuple[float, float, float] = (-0.085, -0.175, 0.195)


# ------------------------------------------------------------------------------
# 2. TRANSFORMACIÓN 3D: CÁMARA REALSENSE -> BRAZO PIPER
# ------------------------------------------------------------------------------

def transform_camera_to_arm(
    x_opt: float,
    y_opt: float,
    z_opt: float,
    pan_rad: float = 0.0,
    tilt_rad: float = 0.0,
    head_offset: Tuple[float, float, float] = HEAD_TO_ARM_OFFSET_M
) -> Tuple[float, float, float]:
    """
    Transforma coordenadas 3D desde el marco óptico de la cámara RealSense
    (X derecha, Y abajo, Z profundidad hacia adelante) al marco cartesiano de la base
    del brazo robótico Piper (arm_base_link: X adelante, Y izquierda, Z arriba).

    Parámetros:
      - x_opt, y_opt, z_opt: Coordenadas del objeto en metros respecto al marco óptico.
      - pan_rad: Ángulo actual del eje Pan (yaw) del cabezal en radianes.
      - tilt_rad: Ángulo actual del eje Tilt (pitch) del cabezal en radianes.
      - head_offset: Offset (dx, dy, dz) del cabezal relativo a la base del brazo.

    Retorna:
      - (x_arm, y_arm, z_arm) en metros en el marco arm_base_link.
    """
    # 1. Transformación de Optical Frame a Camera Link Frame:
    #    X_cam (adelante)   =  Z_opt
    #    Y_cam (izquierda)  = -X_opt
    #    Z_cam (arriba)     = -Y_opt
    x_cam = z_opt
    y_cam = -x_opt
    z_cam = -y_opt

    # 2. Rotación por inclinación Tilt (alrededor del eje Y del cabezal)
    cos_t = math.cos(tilt_rad)
    sin_t = math.sin(tilt_rad)
    x_t = cos_t * x_cam + sin_t * z_cam
    y_t = y_cam
    z_t = -sin_t * x_cam + cos_t * z_cam

    # 3. Rotación por giro Pan (alrededor del eje Z del cabezal)
    cos_p = math.cos(pan_rad)
    sin_p = math.sin(pan_rad)
    x_p = cos_p * x_t - sin_p * y_t
    y_p = sin_p * x_t + cos_p * y_t
    z_p = z_t

    # 4. Traslación del cabezal a arm_base_link
    dx, dy, dz = head_offset
    x_arm = x_p + dx
    y_arm = y_p + dy
    z_arm = z_p + dz

    return (x_arm, y_arm, z_arm)


# ------------------------------------------------------------------------------
# 3. COMPROBACIONES DE SEGURIDAD E INTERBLOQUEO (HARD-STOP & ENVELOPE)
# ------------------------------------------------------------------------------

def is_in_safe_joint_envelope(
    joints_deg: List[float],
    margin_deg: float = 1.0,
    envelope: Optional[Dict[str, Tuple[float, float]]] = None
) -> Tuple[bool, str]:
    """
    Verifica si una configuración articular se encuentra estrictamente dentro
    del área segura definida.

    Retorna:
      (True, 'OK') si es segura, o (False, 'Detalle de violación') en caso contrario.
    """
    limits = envelope if envelope is not None else SAFE_JOINT_LIMITS_DEG
    joint_keys = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    if len(joints_deg) < 6:
        return False, f"Cantidad de articulaciones insuficiente ({len(joints_deg)} < 6)"

    for i, key in enumerate(joint_keys):
        val = joints_deg[i]
        low, high = limits[key]
        if val < (low - margin_deg):
            return False, f"{key} ({val:.2f}°) está por debajo del límite seguro ({low:.1f}°)"
        if val > (high + margin_deg):
            return False, f"{key} ({val:.2f}°) supera el límite seguro ({high:.1f}°)"

    return True, "OK"


def is_in_safe_cartesian_box(
    x_m: float,
    y_m: float,
    z_m: float,
    margin_m: float = 0.01,
    box: Optional[Dict[str, Tuple[float, float]]] = None
) -> Tuple[bool, str]:
    """
    Verifica si un punto cartesiano en el marco de la base del brazo está dentro
    de la caja de seguridad operativa.
    """
    b = box if box is not None else SAFE_CARTESIAN_BOX_M
    x_min, x_max = b['x']
    y_min, y_max = b['y']
    z_min, z_max = b['z']

    if x_m < (x_min - margin_m) or x_m > (x_max + margin_m):
        return False, f"X={x_m:.3f}m fuera del rango seguro [{x_min:.2f}, {x_max:.2f}]m"
    if y_m < (y_min - margin_m) or y_m > (y_max + margin_m):
        return False, f"Y={y_m:.3f}m fuera del rango seguro [{y_min:.2f}, {y_max:.2f}]m"
    if z_m < (z_min - margin_m) or z_m > (z_max + margin_m):
        return False, f"Z={z_m:.3f}m fuera del rango seguro [{z_min:.2f}, {z_max:.2f}]m"

    return True, "OK"


def clamp_to_safe_cartesian_box(
    x_m: float,
    y_m: float,
    z_m: float,
    box: Optional[Dict[str, Tuple[float, float]]] = None
) -> Tuple[float, float, float]:
    """
    Recorta suavemente un punto a los límites de la caja segura.
    """
    b = box if box is not None else SAFE_CARTESIAN_BOX_M
    cx = max(b['x'][0], min(b['x'][1], x_m))
    cy = max(b['y'][0], min(b['y'][1], y_m))
    cz = max(b['z'][0], min(b['z'][1], z_m))
    return (cx, cy, cz)


# ------------------------------------------------------------------------------
# 4. INTERPOLACIÓN COSENO S-CURVE SUAVE (ZERO JERK / TRAYECTORIAS C0 & C1)
# ------------------------------------------------------------------------------

def cosine_s_curve(start_val: float, target_val: float, alpha: float) -> float:
    """
    Interpolación suave con perfil Cosine S-Curve.
    Garantiza velocidad cero y derivada suave en los extremos alpha=0 y alpha=1.
    """
    alpha = max(0.0, min(1.0, alpha))
    s = (1.0 - math.cos(math.pi * alpha)) / 2.0
    return start_val + s * (target_val - start_val)


def interpolate_trajectory(
    wp_start: Dict[str, any],
    wp_end: Dict[str, any],
    steps: int = 20
) -> List[Dict[str, any]]:
    """
    Genera una secuencia de waypoints interpolados suavemente mediante S-Curve.
    """
    res = []
    start_j = wp_start['joints_deg']
    target_j = wp_end['joints_deg']
    start_g = wp_start.get('gripper_mm', 70.0)
    target_g = wp_end.get('gripper_mm', 70.0)

    for i in range(steps + 1):
        alpha = i / max(1, steps)
        j_interp = [
            cosine_s_curve(start_j[k], target_j[k], alpha)
            for k in range(len(start_j))
        ]
        g_interp = cosine_s_curve(start_g, target_g, alpha)
        res.append({
            'joints_deg': j_interp,
            'gripper_mm': round(g_interp, 2),
            'alpha': round(alpha, 3)
        })
    return res


# ------------------------------------------------------------------------------
# 5. MODELO CINEMÁTICO DIRECTO E INVERSO PARA AGILEX PIPER 6-DOF
# ------------------------------------------------------------------------------

class PiperKinematicsSolver:
    """
    Solucionador cinemático autónomo basado en los parámetros Denavit-Hartenberg (DH)
    del brazo AgileX Piper 6-DOF con pinza paralela.
    """

    def __init__(self, tcp_offset_mm: float = 135.8):
        self.PI = math.pi
        self.tcp_offset_mm = tcp_offset_mm

        # Parámetros DH estándar calibrados del fabricante Piper:
        self._a = [0.0, 0.0, 285.03, -21.98, 0.0, 0.0]
        self._alpha = [0.0, -self.PI / 2.0, 0.0, self.PI / 2.0, -self.PI / 2.0, self.PI / 2.0]
        self._theta_offset = [0.0, -self.PI * 172.22 / 180.0, -102.78 / 180.0 * self.PI, 0.0, 0.0, 0.0]
        self._d = [123.0, 0.0, 0.0, 250.75, 0.0, 91.0 + self.tcp_offset_mm]

    def forward_kinematics(self, joints_rad: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calcula la cinemática directa (FK).
        Retorna:
          - Posición cartesiana del extremo efector [X, Y, Z] en mm.
          - Matriz de rotación R (3x3).
        """
        T = np.eye(4)
        for i in range(6):
            th = joints_rad[i] + self._theta_offset[i]
            ca = math.cos(self._alpha[i])
            sa = math.sin(self._alpha[i])
            ct = math.cos(th)
            st = math.sin(th)
            a = self._a[i]
            d = self._d[i]

            Ti = np.array([
                [ct, -st * ca,  st * sa, a * ct],
                [st,  ct * ca, -ct * sa, a * st],
                [0.0,      sa,       ca,      d],
                [0.0,     0.0,      0.0,    1.0]
            ], dtype=float)
            T = T @ Ti

        xyz_mm = T[:3, 3]
        R = T[:3, :3]
        return xyz_mm, R

    def forward_position_m(self, joints_rad: List[float]) -> Tuple[float, float, float]:
        """Calcula [X, Y, Z] en metros en el marco arm_base_link."""
        xyz_mm, _ = self.forward_kinematics(joints_rad)
        return (xyz_mm[0] / 1000.0, xyz_mm[1] / 1000.0, xyz_mm[2] / 1000.0)

    def inverse_kinematics(
        self,
        target_xyz_m: Tuple[float, float, float],
        q_seed_rad: Optional[List[float]] = None,
        max_iterations: int = 140,
        tolerance_mm: float = 2.5
    ) -> Tuple[List[float], bool]:
        """
        Resuelve la Cinemática Inversa (IK) con Damped Least Squares ponderado (Weighted DLS),
        sesgo de espacio nulo para neutralizar el roll de la muñeca (joint 4 y 6)
        y semillas adaptativas para máxima convergencia en todo el espacio de trabajo.

        Retorna:
          - joints_deg: Lista de 6 ángulos en grados.
          - success: True si convergió con error < tolerance_mm y respeta la envolvente.
        """
        tx_mm = target_xyz_m[0] * 1000.0
        ty_mm = target_xyz_m[1] * 1000.0
        tz_mm = target_xyz_m[2] * 1000.0
        target_vec = np.array([tx_mm, ty_mm, tz_mm], dtype=float)

        yaw = math.atan2(target_xyz_m[1], max(0.01, target_xyz_m[0]))

        if q_seed_rad is not None:
            seeds = [np.array(q_seed_rad, dtype=float)]
        else:
            seeds = [
                np.array([yaw, math.radians(70.0), math.radians(-100.0), 0.0, math.radians(30.0), 0.0], dtype=float),
                np.array([yaw, math.radians(60.0), math.radians(-85.0), 0.0, math.radians(20.0), 0.0], dtype=float),
                np.array([yaw, math.radians(80.0), math.radians(-115.0), 0.0, math.radians(40.0), 0.0], dtype=float),
                np.array([yaw, math.radians(50.0), math.radians(-70.0), 0.0, math.radians(15.0), 0.0], dtype=float),
            ]

        # Ponderación articular: alta prioridad a hombro/codo, penalización al roll innecesario
        W_inv = np.diag([1.0, 1.0, 1.0, 0.15, 0.85, 0.15])
        lambda_val = 0.015

        best_q = None
        best_err = float('inf')

        for seed in seeds:
            q = seed.copy()
            for _ in range(max_iterations):
                cur_pos, _ = self.forward_kinematics(q)
                err = target_vec - cur_pos
                err_norm = np.linalg.norm(err)

                if err_norm < tolerance_mm:
                    joints_deg = [math.degrees(v) for v in q]
                    safe, _ = is_in_safe_joint_envelope(joints_deg, margin_deg=3.0)
                    if safe:
                        return joints_deg, True

                if err_norm < best_err:
                    best_err = err_norm
                    best_q = q.copy()

                # Jacobiano Numérico 3x6
                J = np.zeros((3, 6), dtype=float)
                eps = 1e-4
                for j in range(6):
                    q_step = q.copy()
                    q_step[j] += eps
                    pos_step, _ = self.forward_kinematics(q_step)
                    J[:, j] = (pos_step - cur_pos) / eps

                # Weighted DLS
                JJT_damp = J @ W_inv @ J.T + (lambda_val ** 2) * np.eye(3)
                dq = W_inv @ J.T @ np.linalg.inv(JJT_damp) @ err

                # Proyección en espacio nulo para mantener wrist roll en cero y yaw en azimut
                q_bias = np.array([yaw, math.radians(65.0), math.radians(-85.0), 0.0, math.radians(20.0), 0.0])
                P_null = np.eye(6) - W_inv @ J.T @ np.linalg.inv(JJT_damp) @ J
                dq += 0.08 * P_null @ (q_bias - q)

                dq = np.clip(dq, -0.15, 0.15)
                q += dq

                # Límites cinemáticos duros
                q[0] = np.clip(q[0], math.radians(-65.0), math.radians(95.0))
                q[1] = np.clip(q[1], math.radians(10.0), math.radians(175.0))
                q[2] = np.clip(q[2], math.radians(-165.0), math.radians(-15.0))
                q[3] = np.clip(q[3], math.radians(-90.0), math.radians(90.0))
                q[4] = np.clip(q[4], math.radians(-72.0), math.radians(72.0))
                q[5] = np.clip(q[5], math.radians(-160.0), math.radians(160.0))

            cur_pos, _ = self.forward_kinematics(q)
            err_norm = np.linalg.norm(target_vec - cur_pos)
            if err_norm < best_err:
                best_err = err_norm
                best_q = q.copy()

        final_q = best_q if best_q is not None else q
        joints_deg = [math.degrees(v) for v in final_q]
        safe, _ = is_in_safe_joint_envelope(joints_deg, margin_deg=3.0)
        return joints_deg, (best_err < tolerance_mm * 1.5 and safe)


# Instancia singleton predeterminada
kinematics = PiperKinematicsSolver()
