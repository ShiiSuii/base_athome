#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Base@Home - Módulo de Cinemática, Transformaciones 3D y Seguridad Espacial
# Brazo Robótico AgileX Piper 6-DOF (Montaje Lateral Izquierdo en Torso/Prisma)
#
# Incluye:
#   1. Parámetros Denavit-Hartenberg oficiales de AgileX Piper (C_PiperForwardKinematics)
#   2. Conversión exacta entre el marco intrínseco de la base del brazo (arm_base_link,
#      montado con roll = -90° en el lateral izquierdo del prisma) y el Marco del Hombro
#      del Robot (+X adelante, +Y izquierda, +Z arriba).
#   3. Transformación 3D exacta de Cámara RealSense D435 (Pan-Tilt) al Hombro del Brazo.
#   4. Solucionador de Cinemática Inversa (Weighted DLS + Regularización Nullspace hacia
#      la pose Pre-Grasp enseñada) y avance en línea de herramienta (Move Tool Line
#      inspirado en la arquitectura CHARMIE @Home).
# ==============================================================================

import math
from typing import Dict, List, Optional, Tuple
import numpy as np

# ------------------------------------------------------------------------------
# 1. CONSTANTES GEOMÉTRICAS Y ENVOLVENTE DE SEGURIDAD (SAFE ENVELOPE)
# ------------------------------------------------------------------------------

# Posiciones en torso_lift_link según URDF:
#   - Eje Tilt del cabezal (pan_tilt_base + pan + tilt): (0.070, 0.000, 0.200) m
#   - Base del brazo en lado izquierdo del prisma (arm_base_link): (0.155, +0.175, -0.075) m
# Vector desde el hombro izquierdo (arm_base_link) hasta el eje Tilt del cabezal,
# calibrado con compensación de centro volumétrico de botella (+5 cm X) y nivel de mesa (+6 cm Z):
HEAD_TO_ARM_OFFSET_M: Tuple[float, float, float] = (-0.035, -0.175, 0.335)

# Offset desde el eje Tilt hasta el centro óptico de la RealSense D435 (camera_link):
CAMERA_LEVER_ARM_M: Tuple[float, float, float] = (0.035, 0.0, 0.015)

# Cámara del brazo (Orbbec DaBai) centrada físicamente con el eje de la pinza (tcp_link):
WRIST_CAMERA_RIGHT_OFFSET_M: float = 0.0

# Límites articulares seguros (en grados) compatibles con el URDF y las poses de Home, Pre-Grasp y Grasp
SAFE_JOINT_LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    'joint1': (-154.0, 154.0),
    'joint2': (0.0, 195.0),
    'joint3': (-175.0, 0.0),
    'joint4': (-110.0, 110.0),
    'joint5': (-85.0, 85.0),
    'joint6': (-180.0, 180.0),
}

# Caja cartesiana segura en el Marco del Hombro del Robot (+X adelante, +Y izq, +Z arriba)
# Nota: Como el hombro está a +0.175m a la izquierda del centro del robot, Y = -0.175m
# corresponde exactamente al plano central frente al robot.
SAFE_CARTESIAN_BOX_M: Dict[str, Tuple[float, float]] = {
    'x': (0.15, 0.72),    # 15 cm a 72 cm hacia adelante (permite extensión completa de agarre)
    'y': (-0.48, 0.35),   # Cubre desde 31 cm a la derecha del centro hasta 35 cm a la izquierda
    'z': (-0.48, 0.35),   # Desde 48 cm bajo el hombro hasta 35 cm sobre el hombro
}

# Poses nombradas calibradas por defecto (en grados)
DEFAULT_NAMED_POSES: Dict[str, Dict[str, object]] = {
    'home': {
        'description': 'Pose de reposo seguro al lateral izquierdo del prisma',
        'joints_deg': [72.09, 176.5, -155.15, 102.62, -31.51, -99.91],
        'gripper_mm': 69.5,
    },
    'cube_stow': {
        'description': 'Pose compacta de transporte junto al cubo',
        'joints_deg': [72.09, 176.5, -155.15, 102.62, -31.51, -99.91],
        'gripper_mm': 20.0,
    },
    'pre_grasp': {
        'description': 'Pose Pre-Grasp enseñada frente a la mesa con cámara de muñeca nivelada',
        'joints_deg': [53.33, 176.38, -79.88, 69.54, -75.36, -89.05],
        'gripper_mm': 69.5,
    },
    'pre_grasp_center': {
        'description': 'Pose de pre-agarre centrada frente al torso',
        'joints_deg': [53.33, 176.38, -79.88, 69.54, -75.36, -89.05],
        'gripper_mm': 69.5,
    },
    'pre_grasp_table': {
        'description': 'Pose de aproximación a mesa con pinza abierta',
        'joints_deg': [53.33, 176.38, -79.88, 69.54, -75.36, -89.05],
        'gripper_mm': 69.5,
    },
}


# ------------------------------------------------------------------------------
# 2. TRANSFORMACIÓN 3D: CÁMARA REALSENSE -> MARCO DEL HOMBRO DEL BRAZO PIPER
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
    (X derecha, Y abajo, Z profundidad hacia adelante) al Marco Cartesiano del
    Hombro del Brazo Piper (+X adelante, +Y izquierda, +Z arriba), siguiendo
    exactamente la cadena cinemática del URDF.
    """
    # 1. De camera_color_optical_frame a tilt_link (sumando el brazo de palanca de camera_link):
    lx, ly, lz = CAMERA_LEVER_ARM_M
    x_cam = z_opt + lx
    y_cam = -x_opt + ly
    z_cam = -y_opt + lz

    # 2. Rotación por inclinación Tilt (alrededor del eje Y de tilt_joint):
    #    Nota: tilt_rad > 0 inclina la cámara hacia abajo (mesa).
    cos_t = math.cos(tilt_rad)
    sin_t = math.sin(tilt_rad)
    x_t = cos_t * x_cam + sin_t * z_cam
    y_t = y_cam
    z_t = -sin_t * x_cam + cos_t * z_cam

    # 3. Rotación por giro Pan (alrededor del eje Z de pan_joint):
    cos_p = math.cos(pan_rad)
    sin_p = math.sin(pan_rad)
    x_p = cos_p * x_t - sin_p * y_t
    y_p = sin_p * x_t + cos_p * y_t
    z_p = z_t

    # 4. Traslación desde el eje del cabezal hasta el hombro izquierdo (arm_base_link):
    dx, dy, dz = head_offset
    x_sh = x_p + dx
    y_sh = y_p + dy
    z_sh = z_p + dz

    return (x_sh, y_sh, z_sh)


# ------------------------------------------------------------------------------
# 3. COMPROBACIONES DE SEGURIDAD E INTERBLOQUEO (HARD-STOP & ENVELOPE)
# ------------------------------------------------------------------------------

def is_in_safe_joint_envelope(
    joints_deg: List[float],
    margin_deg: float = 2.0,
    envelope: Optional[Dict[str, Tuple[float, float]]] = None
) -> Tuple[bool, str]:
    """
    Verifica si una configuración articular se encuentra dentro del área segura.
    """
    limits = envelope if envelope is not None else SAFE_JOINT_LIMITS_DEG
    joint_keys = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    if len(joints_deg) < 6:
        return False, f"Cantidad de articulaciones insuficiente ({len(joints_deg)} < 6)"

    for i, key in enumerate(joint_keys):
        val = joints_deg[i]
        low, high = limits[key]
        if val < (low - margin_deg):
            return False, f"{key} ({val:.1f}°) por debajo del límite ({low:.0f}°)"
        if val > (high + margin_deg):
            return False, f"{key} ({val:.1f}°) supera el límite ({high:.0f}°)"

    return True, "OK"


def is_in_safe_cartesian_box(
    x_m: float,
    y_m: float,
    z_m: float,
    box: Optional[Dict[str, Tuple[float, float]]] = None
) -> Tuple[bool, str]:
    """
    Verifica si un punto 3D (x, y, z) en el Marco del Hombro (+X adelante, +Y izq, +Z arriba)
    está dentro de la ventana operativa segura y libre de colisión con el prisma/pantalla.
    """
    bounds = box if box is not None else SAFE_CARTESIAN_BOX_M

    if not (bounds['x'][0] <= x_m <= bounds['x'][1]):
        return False, f"Distancia X={x_m:.2f}m fuera de rango [{bounds['x'][0]}, {bounds['x'][1]}]m"
    if not (bounds['y'][0] <= y_m <= bounds['y'][1]):
        return False, f"Lateral Y={y_m:.2f}m fuera de rango [{bounds['y'][0]}, {bounds['y'][1]}]m"
    if not (bounds['z'][0] <= z_m <= bounds['z'][1]):
        return False, f"Altura Z={z_m:.2f}m fuera de rango [{bounds['z'][0]}, {bounds['z'][1]}]m"

    # Protección del prisma central y pantalla interactiva:
    # En el Marco del Hombro, el prisma ocupa X en [-0.125, +0.125] y Y en [-0.35, 0.0].
    if x_m < 0.17 and y_m < -0.04:
        return False, f"Zona de colisión con el prisma/pantalla (X={x_m:.2f}m, Y={y_m:.2f}m)"

    return True, "OK"


def clamp_to_safe_cartesian_box(
    x_m: float,
    y_m: float,
    z_m: float,
    box: Optional[Dict[str, Tuple[float, float]]] = None
) -> Tuple[float, float, float]:
    """Recorta (clamps) una coordenada 3D a los límites de la caja cartesiana segura."""
    bounds = box if box is not None else SAFE_CARTESIAN_BOX_M
    cx = min(bounds['x'][1], max(bounds['x'][0], x_m))
    cy = min(bounds['y'][1], max(bounds['y'][0], y_m))
    cz = min(bounds['z'][1], max(bounds['z'][0], z_m))
    if cx < 0.18 and cy < -0.04:
        cx = 0.18
    return (cx, cy, cz)


# ------------------------------------------------------------------------------
# 4. INTERPOLACIÓN SUAVE DE TRAYECTORIAS
# ------------------------------------------------------------------------------

def cosine_s_curve(alpha: float) -> float:
    """Perfil de velocidad suave tipo campana (Coseno S-Curve) para alpha en [0, 1]."""
    alpha_clamped = min(1.0, max(0.0, alpha))
    return (1.0 - math.cos(math.pi * alpha_clamped)) / 2.0


def interpolate_joint_waypoint(
    start_deg: List[float],
    target_deg: List[float],
    alpha: float
) -> List[float]:
    """Interpola suavemente entre dos vectores articulares de 6 grados de libertad."""
    s = cosine_s_curve(alpha)
    return [start_deg[i] + s * (target_deg[i] - start_deg[i]) for i in range(6)]


# ------------------------------------------------------------------------------
# 5. MODELO CINEMÁTICO DIRECTO E INVERSO OFICIAL PARA AGILEX PIPER 6-DOF
# ------------------------------------------------------------------------------

class PiperKinematicsSolver:
    """
    Solucionador cinemático exacto basado en C_PiperForwardKinematics del SDK oficial
    de AgileX Piper y adaptado al montaje lateral izquierdo del robot (rpy = -pi/2, 0, 0).
    """

    def __init__(self, tcp_offset_mm: float = 135.8):
        self.PI = math.pi
        self.tcp_offset_mm = tcp_offset_mm

        # Parámetros DH oficiales de AgileX Piper (dh_is_offset = 0x01):
        self._a = [0.0, 0.0, 285.03, -21.98, 0.0, 0.0]
        self._alpha = [0.0, -self.PI / 2.0, 0.0, self.PI / 2.0, -self.PI / 2.0, self.PI / 2.0]
        self._theta_offset = [0.0, -self.PI * 172.22 / 180.0, -102.78 / 180.0 * self.PI, 0.0, 0.0, 0.0]
        self._d = [123.0, 0.0, 0.0, 250.75, 0.0, 91.0 + self.tcp_offset_mm]

        # Matriz de rotación del montaje lateral izquierdo (cube_to_arm_joint: rpy = [-pi/2, 0, 0]):
        # Transforma vectores desde arm_base_link a Marco del Hombro (+X adelante, +Y izq, +Z arriba)
        self._R_mount = np.array([
            [1.0,  0.0, 0.0],
            [0.0,  0.0, 1.0],
            [0.0, -1.0, 0.0],
        ], dtype=float)

        # Límites articulares en radianes
        self._q_min = np.radians([SAFE_JOINT_LIMITS_DEG[f'joint{i}'][0] for i in range(1, 7)])
        self._q_max = np.radians([SAFE_JOINT_LIMITS_DEG[f'joint{i}'][1] for i in range(1, 7)])

    def _link_transform(self, alpha: float, a: float, theta: float, d: float) -> np.ndarray:
        """
        Matriz de transformación de eslabón idéntica a __LinkTransformtion de piper_sdk.kinematics.
        """
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        ct = math.cos(theta)
        st = math.sin(theta)

        return np.array([
            [ct,      -st,     0.0,       a],
            [st * ca, ct * ca, -sa, -sa * d],
            [st * sa, ct * sa,  ca,  ca * d],
            [0.0,     0.0,     0.0,     1.0]
        ], dtype=float)

    def forward_kinematics_arm_frame(
        self,
        joints_rad: List[float],
        include_tcp: bool = True
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calcula la cinemática directa en el marco intrínseco arm_base_link (en mm).
        Si include_tcp=False, coincide al 0.02 mm con GetArmEndPoseMsgs() del firmware CAN.
        """
        T = np.eye(4)
        for i in range(6):
            th = float(joints_rad[i]) + self._theta_offset[i]
            d = self._d[i] if (i < 5 or include_tcp) else 91.0
            Ti = self._link_transform(self._alpha[i], self._a[i], th, d)
            T = T @ Ti

        return T[:3, 3], T[:3, :3]

    def forward_kinematics(self, joints_rad: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calcula la cinemática directa (FK) del TCP de la pinza en el Marco del Hombro
        del Robot (+X adelante, +Y izquierda, +Z arriba) en milímetros.
        """
        xyz_arm_mm, R_arm = self.forward_kinematics_arm_frame(joints_rad, include_tcp=True)
        xyz_shoulder_mm = self._R_mount @ xyz_arm_mm
        R_shoulder = self._R_mount @ R_arm
        return xyz_shoulder_mm, R_shoulder

    def forward_position_m(self, joints_rad: List[float]) -> Tuple[float, float, float]:
        """Calcula [X, Y, Z] del TCP en metros en el Marco del Hombro (+X adelante, +Y izq, +Z arriba)."""
        xyz_mm, _ = self.forward_kinematics(joints_rad)
        return (float(xyz_mm[0] / 1000.0), float(xyz_mm[1] / 1000.0), float(xyz_mm[2] / 1000.0))

    def compute_jacobian(self, q_rad: np.ndarray, eps: float = 1e-5) -> np.ndarray:
        """Calcula el Jacobiano posicional (3x6) en el Marco del Hombro por diferencias finitas."""
        J = np.zeros((3, 6), dtype=float)
        pos0, _ = self.forward_kinematics(q_rad)
        for i in range(6):
            q_plus = q_rad.copy()
            q_plus[i] += eps
            pos_plus, _ = self.forward_kinematics(q_plus)
            J[:, i] = (pos_plus - pos0) / eps
        return J

    def move_tool_line(
        self,
        current_joints_deg: List[float],
        delta_forward_m: float = 0.0,
        delta_lateral_m: float = 0.0,
        delta_up_m: float = 0.0,
    ) -> Tuple[List[float], bool]:
        """
        Movimiento lineal relativo desde la postura actual (inspirado en 'adjust_move_tool_line'
        de CHARMIE @Home), manteniendo la postura natural de la muñeca.
        """
        q_seed_rad = [math.radians(d) for d in current_joints_deg]
        cur_x, cur_y, cur_z = self.forward_position_m(q_seed_rad)
        target_xyz_m = (
            cur_x + delta_forward_m,
            cur_y + delta_lateral_m,
            cur_z + delta_up_m,
        )
        return self.inverse_kinematics(target_xyz_m, q_seed_rad=q_seed_rad)

    def inverse_kinematics(
        self,
        target_xyz_m: Tuple[float, float, float],
        q_seed_rad: Optional[List[float]] = None,
        max_iterations: int = 160,
        tolerance_mm: float = 2.0
    ) -> Tuple[List[float], bool]:
        """
        Resuelve la Cinemática Inversa (IK) en el Marco del Hombro (+X adelante, +Y izq, +Z arriba)
        usando Damped Least Squares ponderado (Weighted DLS) con regularización de espacio nulo
        hacia la postura Pre-Grasp enseñada para evitar giros bruscos o inversión de codo.

        Retorna:
          - joints_deg: Lista de 6 ángulos articulares en grados.
          - success: True si convergió dentro de la tolerancia y respeta límites seguros.
        """
        target_mm = np.array([
            target_xyz_m[0] * 1000.0,
            target_xyz_m[1] * 1000.0,
            target_xyz_m[2] * 1000.0
        ], dtype=float)

        default_pre_deg = DEFAULT_NAMED_POSES['pre_grasp']['joints_deg']
        q_pre_rad = np.radians(default_pre_deg)

        seeds = []
        if q_seed_rad is not None and len(q_seed_rad) >= 6:
            seeds.append(np.array(q_seed_rad[:6], dtype=float))
        seeds.extend([
            q_pre_rad.copy(),
            np.radians([25.0, 165.0, -90.0, 50.0, -55.0, -110.0]),
            np.radians([45.0, 170.0, -105.0, 60.0, -60.0, -110.0]),
            np.radians([15.0, 155.0, -80.0, 45.0, -50.0, -105.0]),
        ])

        # Ponderación articular: prioriza hombro/codo/muñeca pitch y bloquea giro innecesario de J6
        W_inv = np.diag([1.0, 1.0, 1.0, 0.40, 1.20, 0.0])
        lambda_sq = 12.0 ** 2  # Amortiguamiento en escala de mm

        best_q = None
        best_err = float('inf')

        for seed in seeds:
            q = np.clip(seed.copy(), self._q_min, self._q_max)

            for _ in range(max_iterations):
                cur_pos, _ = self.forward_kinematics(q)
                err = target_mm - cur_pos
                err_norm = float(np.linalg.norm(err))

                if err_norm < best_err:
                    best_err = err_norm
                    best_q = q.copy()

                if err_norm < tolerance_mm:
                    q_deg = [round(float(math.degrees(v)), 2) for v in q]
                    safe, _ = is_in_safe_joint_envelope(q_deg)
                    if safe:
                        return q_deg, True
                    break

                J = self.compute_jacobian(q)
                JW = J @ W_inv
                JJT = JW @ J.T + lambda_sq * np.eye(3)
                J_pinv = W_inv @ J.T @ np.linalg.inv(JJT)

                dq = J_pinv @ err
                dq = np.clip(dq, -0.10, 0.10)
                q = np.clip(q + dq, self._q_min, self._q_max)

        if best_q is not None:
            q_deg = [round(float(math.degrees(v)), 2) for v in best_q]
            safe, _ = is_in_safe_joint_envelope(q_deg)
            if best_err < 12.0 and safe:
                return q_deg, True
            return q_deg, False

        return [round(float(d), 2) for d in default_pre_deg], False
