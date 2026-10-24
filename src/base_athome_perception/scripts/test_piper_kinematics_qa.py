#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# QA Test Suite - Validación de Cinemática, Transformaciones y Envolvente Segura
# ==============================================================================

import math
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from base_athome_perception.piper_kinematics import (
    transform_camera_to_arm,
    is_in_safe_joint_envelope,
    is_in_safe_cartesian_box,
    clamp_to_safe_cartesian_box,
    cosine_s_curve,
    interpolate_trajectory,
    PiperKinematicsSolver,
    SAFE_JOINT_LIMITS_DEG,
    SAFE_CARTESIAN_BOX_M,
    DEFAULT_NAMED_POSES,
)

def test_camera_to_arm_transform():
    print("[TEST 1] Transformación 3D RealSense -> Arm Base Link...")
    # Caso 1: Pringles al centro, 0.50m adelante, 0.10m abajo en frame óptico, Pan=0, Tilt=0
    x, y, z = transform_camera_to_arm(0.0, 0.10, 0.50, pan_rad=0.0, tilt_rad=0.0)
    assert abs(x - 0.415) < 1e-4, f"X esperado 0.415, obtenido {x}"
    assert abs(y - (-0.175)) < 1e-4, f"Y esperado -0.175, obtenido {y}"
    assert abs(z - 0.095) < 1e-4, f"Z esperado 0.095, obtenido {z}"

    # Caso 2: Con inclinación Tilt = -10° (-0.1745 rad)
    tilt_rad = math.radians(-10.0)
    x_t, y_t, z_t = transform_camera_to_arm(0.0, 0.05, 0.55, pan_rad=0.0, tilt_rad=tilt_rad)
    assert 0.12 <= x_t <= 0.55, f"X={x_t} fuera de rango"
    print(f"   -> OK: Objetivo frontal transformado: X={x:.3f}m, Y={y:.3f}m, Z={z:.3f}m")
    print(f"   -> OK: Objetivo con tilt -10° transformado: X={x_t:.3f}m, Y={y_t:.3f}m, Z={z_t:.3f}m")

def test_safe_envelope_interlock():
    print("[TEST 2] Verificación de Envolvente Segura e Interbloqueo Hard-Stop...")
    # Pose segura (pre_grasp_center)
    safe_pose = [0.0, 65.0, -75.0, 0.0, 10.0, 0.0]
    ok, reason = is_in_safe_joint_envelope(safe_pose)
    assert ok, f"Debería ser segura: {reason}"

    # Pose insegura 1: J1 fuera de rango (+120° > 95°)
    unsafe_j1 = [120.0, 65.0, -75.0, 0.0, 10.0, 0.0]
    ok1, reason1 = is_in_safe_joint_envelope(unsafe_j1)
    assert not ok1, "Debería fallar por J1 excesivo"
    assert "joint1" in reason1

    # Pose insegura 2: J2 fuera de rango (-5° < 10°)
    unsafe_j2 = [0.0, -5.0, -75.0, 0.0, 10.0, 0.0]
    ok2, reason2 = is_in_safe_joint_envelope(unsafe_j2)
    assert not ok2, "Debería fallar por J2 bajo"
    assert "joint2" in reason2

    # Pose insegura 3: J3 fuera de rango (+10° > -15°)
    unsafe_j3 = [0.0, 65.0, 10.0, 0.0, 10.0, 0.0]
    ok3, reason3 = is_in_safe_joint_envelope(unsafe_j3)
    assert not ok3, "Debería fallar por J3 colisión codo"
    assert "joint3" in reason3

    print("   -> OK: Interbloqueos de seguridad articular validan y rechazan correctamente.")

def test_inverse_kinematics():
    print("[TEST 3] Cinemática Inversa DLS para Agarre de Pringles...")
    solver = PiperKinematicsSolver()

    # Probar target en zona óptima de agarre: X=0.40m, Y=-0.15m, Z=0.05m
    target = (0.40, -0.15, 0.05)
    joints_deg, success = solver.inverse_kinematics(target)
    assert success, "La cinemática inversa debe converger para objetivo en área segura"

    # Verificar cinemática directa de la solución
    j_rad = [math.radians(v) for v in joints_deg]
    pos_res = solver.forward_position_m(j_rad)
    dist_err = math.dist(target, pos_res)
    assert dist_err < 0.005, f"Error de posición {dist_err*1000:.1f} mm supera 5 mm"

    # Verificar que la solución articular respeta la envolvente
    is_safe, reason = is_in_safe_joint_envelope(joints_deg)
    assert is_safe, f"La solución de IK violó la envolvente: {reason}"

    print(f"   -> OK: IK convergió a error={dist_err*1000:.2f}mm. Articulaciones: {[round(v, 2) for v in joints_deg]}°")

def test_s_curve_interpolation():
    print("[TEST 4] Interpolación Cosine S-Curve (Derivada Cero en Extremos)...")
    v0 = 0.0
    v1 = 100.0

    # Alpha 0 y 1
    assert cosine_s_curve(v0, v1, 0.0) == 0.0
    assert cosine_s_curve(v0, v1, 1.0) == 100.0

    # Punto medio debe ser exactamente 50.0
    assert abs(cosine_s_curve(v0, v1, 0.5) - 50.0) < 1e-4

    # Verificar aceleración suave: s(0.1) < 10.0 (arranque suave, menor que lineal)
    val_01 = cosine_s_curve(v0, v1, 0.1)
    assert val_01 < 10.0, "S-Curve debe arrancar con aceleración suave"

    # s(0.9) > 90.0 (frenado suave)
    val_09 = cosine_s_curve(v0, v1, 0.9)
    assert val_09 > 90.0, "S-Curve debe desacelerar suavemente al final"

    print("   -> OK: Perfil S-Curve garantiza aceleración y desaceleración suaves sin tirones.")

def main():
    print("==========================================================================")
    print("   SUITE DE PRUEBAS QA: CINEMÁTICA Y CONTROL BRAZO AGILEX PIPER 6-DOF    ")
    print("==========================================================================")
    test_camera_to_arm_transform()
    test_safe_envelope_interlock()
    test_inverse_kinematics()
    test_s_curve_interpolation()
    print("==========================================================================")
    print("   TODAS LAS PRUEBAS (4/4) PASARON CON ÉXITO!                            ")
    print("==========================================================================")

if __name__ == "__main__":
    main()
