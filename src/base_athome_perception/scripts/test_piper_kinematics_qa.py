#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# QA Test Suite - Validación de Cinemática Oficial AgileX Piper, Montaje Lateral
# Izquierdo, Transformación 3D Cabezal->Hombro y Avance CHARMIE Move Tool Line
# ==============================================================================

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from base_athome_perception.piper_kinematics import (
    transform_camera_to_arm,
    is_in_safe_joint_envelope,
    is_in_safe_cartesian_box,
    cosine_s_curve,
    interpolate_joint_waypoint,
    PiperKinematicsSolver,
    DEFAULT_NAMED_POSES,
)


def test_official_firmware_fk_match():
    print("[TEST 1] Verificación de Cinemática Directa contra Firmware CAN Piper (GetArmEndPoseMsgs)...")
    solver = PiperKinematicsSolver()
    # Pose real medida desde encoders del brazo Piper:
    # Encoders: [88.14°, 172.89°, -164.22°, 96.72°, -1.02°, -71.42°]
    # GetArmEndPoseMsgs reportó: X=21.995 mm, Y=627.056 mm, Z=119.507 mm
    q_rad = [math.radians(d) for d in [88.14, 172.89, -164.22, 96.72, -1.02, -71.42]]
    xyz_arm_mm, _ = solver.forward_kinematics_arm_frame(q_rad, include_tcp=False)

    err_x = abs(xyz_arm_mm[0] - 21.995)
    err_y = abs(xyz_arm_mm[1] - 627.056)
    err_z = abs(xyz_arm_mm[2] - 119.507)
    assert err_x < 0.05 and err_y < 0.05 and err_z < 0.05, (
        f"Desviación FK vs Firmware: {xyz_arm_mm} vs [21.995, 627.056, 119.507]"
    )
    print(f"   -> OK: FK coincide con el firmware Piper a [{err_x:.3f}, {err_y:.3f}, {err_z:.3f}] mm (< 0.05 mm).")


def test_left_shoulder_mount_and_taught_poses():
    print("[TEST 2] Verificación de Montaje Lateral Izquierdo (Rx(-90°)) en Poses Home y Pre-Grasp...")
    solver = PiperKinematicsSolver()

    home_deg = DEFAULT_NAMED_POSES["home"]["joints_deg"]
    pre_deg = DEFAULT_NAMED_POSES["pre_grasp"]["joints_deg"]

    ok_h, msg_h = is_in_safe_joint_envelope(home_deg)
    ok_p, msg_p = is_in_safe_joint_envelope(pre_deg)
    assert ok_h, f"Pose Home fuera de límites: {msg_h}"
    assert ok_p, f"Pose Pre-Grasp fuera de límites: {msg_p}"

    home_xyz = solver.forward_position_m([math.radians(d) for d in home_deg])
    pre_xyz = solver.forward_position_m([math.radians(d) for d in pre_deg])

    # En Home el brazo cuelga hacia abajo al costado izquierdo (Z_sh < -0.50 m)
    assert home_xyz[2] < -0.50, f"En Home el brazo debería colgar hacia abajo (-Z), obtenido Z={home_xyz[2]:.3f}m"
    # En Pre-Grasp el TCP se extiende hacia adelante (+X > 0.35 m) frente a la mesa
    assert pre_xyz[0] > 0.35, f"En Pre-Grasp el brazo debería extenderse hacia adelante (+X), obtenido X={pre_xyz[0]:.3f}m"

    print(f"   -> OK: Home TCP en Hombro = {[round(v, 3) for v in home_xyz]} m")
    print(f"   -> OK: Pre-Grasp TCP en Hombro = {[round(v, 3) for v in pre_xyz]} m")


def test_camera_to_arm_transform():
    print("[TEST 3] Transformación 3D RealSense D435 (Pan-Tilt) -> Hombro Izquierdo Piper...")
    # Botella en centro óptico a 0.50m de profundidad, con inclinación hacia la mesa (+18°)
    tilt_rad = math.radians(18.0)
    x_sh, y_sh, z_sh = transform_camera_to_arm(0.0, 0.05, 0.50, pan_rad=0.0, tilt_rad=tilt_rad)
    in_box, reason = is_in_safe_cartesian_box(x_sh, y_sh, z_sh)
    assert in_box, f"Objetivo de mesa debería estar en caja segura: {reason}"
    print(f"   -> OK: Objetivo con Tilt +18° transformado a Hombro: X={x_sh:.3f}m, Y={y_sh:.3f}m, Z={z_sh:.3f}m")


def test_charmie_pregrasp_seeded_ik_and_tool_line():
    print("[TEST 4] Cinemática Inversa Sembrada en Pre-Grasp y Avance Lineal (CHARMIE Move Tool Line)...")
    solver = PiperKinematicsSolver()
    pre_deg = DEFAULT_NAMED_POSES["pre_grasp"]["joints_deg"]
    pre_xyz = solver.forward_position_m([math.radians(d) for d in pre_deg])

    # 1. Avance de +6 cm hacia adelante hacia la botella
    grasp_deg, ok_g = solver.move_tool_line(pre_deg, delta_forward_m=0.06, delta_lateral_m=0.01, delta_up_m=0.0)
    assert ok_g, "move_tool_line de avance (+6 cm) debe converger"
    grasp_xyz = solver.forward_position_m([math.radians(d) for d in grasp_deg])
    err_g = math.dist((pre_xyz[0] + 0.06, pre_xyz[1] + 0.01, pre_xyz[2]), grasp_xyz)
    assert err_g < 0.002, f"Error de avance lineal {err_g*1000:.2f} mm supera 2 mm"

    # 2. Elevación vertical de +8 cm (+Z_sh) con la botella sujeta
    lift_deg, ok_l = solver.move_tool_line(grasp_deg, delta_forward_m=0.0, delta_lateral_m=0.0, delta_up_m=0.08)
    assert ok_l, "move_tool_line de elevación (+8 cm) debe converger"
    lift_xyz = solver.forward_position_m([math.radians(d) for d in lift_deg])
    err_l = math.dist((grasp_xyz[0], grasp_xyz[1], grasp_xyz[2] + 0.08), lift_xyz)
    assert err_l < 0.002, f"Error de elevación {err_l*1000:.2f} mm supera 2 mm"

    print(f"   -> OK: Avance +6cm convergió con error={err_g*1000:.2f}mm -> Joints={grasp_deg}")
    print(f"   -> OK: Elevación +8cm convergió con error={err_l*1000:.2f}mm -> Joints={lift_deg}")


def test_s_curve_interpolation():
    print("[TEST 5] Interpolación Cosine S-Curve...")
    assert abs(cosine_s_curve(0.0) - 0.0) < 1e-6
    assert abs(cosine_s_curve(0.5) - 0.5) < 1e-6
    assert abs(cosine_s_curve(1.0) - 1.0) < 1e-6
    wp = interpolate_joint_waypoint([0.0] * 6, [100.0] * 6, 0.5)
    assert abs(wp[0] - 50.0) < 1e-4
    print("   -> OK: Perfil Cosine S-Curve validado.")


def main():
    print("==========================================================================")
    print("   SUITE DE PRUEBAS QA: CINEMÁTICA OFICIAL PIPER 6-DOF & ARQUITECTURA     ")
    print("==========================================================================")
    test_official_firmware_fk_match()
    test_left_shoulder_mount_and_taught_poses()
    test_camera_to_arm_transform()
    test_charmie_pregrasp_seeded_ik_and_tool_line()
    test_s_curve_interpolation()
    print("==========================================================================")
    print("   TODAS LAS PRUEBAS (5/5) PASARON CON ÉXITO!                             ")
    print("==========================================================================")


if __name__ == "__main__":
    main()
