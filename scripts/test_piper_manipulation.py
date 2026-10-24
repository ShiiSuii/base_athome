#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Diagnostico, Cinematica y Control de Brazo AgileX Piper 6-DOF
# Robocup @Home (Open Platform League)
#
# Hardware:
#   - Brazo: AgileX Piper 6-DOF Manipulator con Pinza Paralela
#   - Bus: CAN bus SocketCAN @ 1,000,000 bps (1 Mbps) en can1 (Jetson Orin Nano)
#   - Camara en muneca: Orbbec DaBai DC1 (/dev/video6 en Jetson)
#   - Montaje: Sistema elevador NEMA 23 (subir y bajar)
# ==============================================================================

import argparse
import math
import os
import subprocess
import sys
import time


JETSON_HOST = "ubuntu@10.42.0.12"


def run_on_jetson(command: str) -> str:
    """Ejecuta un comando remoto en la Jetson Orin Nano mediante SSH."""
    cmd = ["ssh", "-o", "ConnectTimeout=4", JETSON_HOST, command]
    res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode != 0 and res.stderr:
        print(f"[Aviso SSH]: {res.stderr.strip()}", file=sys.stderr)
    return res.stdout


def detect_active_can() -> str:
    """Detecta si can0 o can1 está UP en la Jetson."""
    out0 = run_on_jetson("ip link show can0")
    if "state UP" in out0 or "<NOARP,UP" in out0:
        return "can0"
    out1 = run_on_jetson("ip link show can1")
    if "state UP" in out1 or "<NOARP,UP" in out1:
        return "can1"
    return "can0"


def check_can_interface():
    """Verifica y asegura que la interfaz CAN activa esté levantada a 1 Mbps."""
    port = detect_active_can()
    out = run_on_jetson(f"ip link show {port}")
    if "UP" not in out:
        print(f"[*] Levantando interfaz {port} a 1 Mbps...")
        run_on_jetson(f"sudo ip link set {port} down 2>/dev/null; sudo ip link set {port} type can bitrate 1000000 && sudo ip link set {port} up")
        time.sleep(0.3)
        out = run_on_jetson(f"ip link show {port}")

    is_up = "UP" in out
    print(f"[*] Interfaz {port} (Piper CAN bus @ 1Mbps): {'[ACTIVA / OK]' if is_up else '[ERROR]'}")
    return is_up


def get_telemetry():
    """Lee y muestra la telemetria completa, cinemática directa y gripper."""
    port = detect_active_can()
    script = """
import math, time
from piper_sdk import C_PiperInterface_V2, C_PiperForwardKinematics

piper = C_PiperInterface_V2('PORT_NAME')
piper.ConnectPort()
time.sleep(0.4)

st = piper.GetArmStatus().arm_status
joints = piper.GetArmJointMsgs().joint_state
raw_j = [joints.joint_1, joints.joint_2, joints.joint_3, joints.joint_4, joints.joint_5, joints.joint_6]
deg_j = [v / 1000.0 for v in raw_j]
rad_j = [math.radians(d) for d in deg_j]

fk = C_PiperForwardKinematics()
fk_end = fk.CalFK(rad_j)[-1]

hw_end = piper.GetArmEndPoseMsgs().end_pose
hw_xyz_rpy = [
    hw_end.X_axis / 1000.0, hw_end.Y_axis / 1000.0, hw_end.Z_axis / 1000.0,
    hw_end.RX_axis / 1000.0, hw_end.RY_axis / 1000.0, hw_end.RZ_axis / 1000.0
]

grip = piper.GetArmGripperMsgs().gripper_state
grip_mm = grip.grippers_angle / 1000.0
grip_n = grip.grippers_effort / 1000.0

print(f'STATUS|{st.ctrl_mode.name}|{st.arm_status.name}|{st.arm_status.value}')
print(f'JOINTS|' + ','.join(f'{d:.3f}' for d in deg_j))
print(f'RADS|' + ','.join(f'{r:.4f}' for r in rad_j))
print(f'FK|' + ','.join(f'{v:.2f}' for v in fk_end))
print(f'HW|' + ','.join(f'{v:.2f}' for v in hw_xyz_rpy))
print(f'GRIPPER|{grip_mm:.2f}|{grip_n:.3f}')
""".replace('PORT_NAME', port)
    raw = run_on_jetson(f"python3 -c \"{script}\"")
    lines = [line.strip() for line in raw.splitlines() if "|" in line]
    data = {}
    for l in lines:
        parts = l.split("|")
        data[parts[0]] = parts[1:]

    if not data:
        print("[!] No se recibio telemetria del brazo. Verifique conexion CAN.")
        return

    ctrl_mode = data.get("STATUS", ["DESCONOCIDO", "DESCONOCIDO"])[0]
    arm_status = data.get("STATUS", ["", "NORMAL"])[1]
    deg_j = [float(x) for x in data["JOINTS"][0].split(",")]
    rad_j = [float(x) for x in data["RADS"][0].split(",")]
    fk_vals = [float(x) for x in data["FK"][0].split(",")]
    hw_vals = [float(x) for x in data["HW"][0].split(",")]
    grip_mm = float(data["GRIPPER"][0])
    grip_n = float(data["GRIPPER"][1])

    print("==========================================================================")
    print("           TELEMETRIA EN VIVO: BRAZO AGILEX PIPER 6-DOF                   ")
    print("==========================================================================")
    print(f" Modo de Control : {ctrl_mode} | Estado General: {arm_status}")
    print("--------------------------------------------------------------------------")
    print(" ARTICULACIONES (Joint Angles):")
    for i, (d, r) in enumerate(zip(deg_j, rad_j), 1):
        bar_len = int(abs(d) / 10.0)
        bar = ("+" if d >= 0 else "-") * min(15, bar_len)
        print(f"   Joint {i}: {d:8.3f}° ({r:8.4f} rad)  [{bar:<15}]")
    print("--------------------------------------------------------------------------")
    print(" CINEMATICA DIRECTA (Forward Kinematics - Extremo Efector en mm / deg):")
    print(f"   Posicion X : {fk_vals[0]:8.2f} mm   (Reporte HW: {hw_vals[0]:8.2f} mm)")
    print(f"   Posicion Y : {fk_vals[1]:8.2f} mm   (Reporte HW: {hw_vals[1]:8.2f} mm)")
    print(f"   Posicion Z : {fk_vals[2]:8.2f} mm   (Reporte HW: {hw_vals[2]:8.2f} mm)")
    print(f"   Orient. Roll  (RX) : {fk_vals[3]:8.2f}°    (Reporte HW: {hw_vals[3]:8.2f}°)")
    print(f"   Orient. Pitch (RY) : {fk_vals[4]:8.2f}°    (Reporte HW: {hw_vals[4]:8.2f}°)")
    print(f"   Orient. Yaw   (RZ) : {fk_vals[5]:8.2f}°    (Reporte HW: {hw_vals[5]:8.2f}°)")
    print("--------------------------------------------------------------------------")
    print(" PINZA / GRIPPER:")
    print(f"   Apertura : {grip_mm:6.2f} mm  (Rango util: 0 a 75 mm)")
    print(f"   Fuerza   : {grip_n:6.3f} N/m")
    print("==========================================================================")


def test_gripper(target_mm: float):
    """Ejecuta una manipulacion segura de la pinza (abrir / cerrar a milimetros dados)."""
    target_mm = max(0.0, min(75.0, target_mm))
    print(f"\n[*] Comandando pinza del Piper a {target_mm:.1f} mm...")
    port = detect_active_can()
    script = f"""
import time
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2('{port}')
piper.ConnectPort()
time.sleep(0.3)
piper.EnablePiper()

target_range = round({target_mm} * 1000 * 1000)
for _ in range(70):
    piper.GripperCtrl(abs(target_range), 1200, 0x01, 0)
    time.sleep(0.01)

time.sleep(0.4)
final_mm = piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000.0
print(f'RESULT|{{final_mm:.2f}}')
"""
    raw = run_on_jetson(f"python3 -c \"{script}\"")
    for line in raw.splitlines():
        if "RESULT|" in line:
            final_mm = float(line.split("|")[1])
            print(f"[+] Pinza posicionada con exito en: {final_mm:.2f} mm")
            return
    print(raw)


def test_wrist_jog(delta_deg: float):
    """Mueve suavemente la articulacion 6 (rotacion de la muneca/camara) en delta grados."""
    delta_deg = max(-5.0, min(5.0, delta_deg))  # Limite estricto de seguridad de +/- 5 grados
    print(f"\n[!] Prueba de jog seguro en Joint 6 (Muneca/Camara): {delta_deg:+.1f}°")
    port = detect_active_can()
    script = f"""
import time, math
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2('{port}')
piper.ConnectPort()
time.sleep(0.3)

joints = piper.GetArmJointMsgs().joint_state
raw_j = [joints.joint_1, joints.joint_2, joints.joint_3, joints.joint_4, joints.joint_5, joints.joint_6]

target_j6 = raw_j[5] + round({delta_deg} * 1000)

piper.EnablePiper()
piper.MotionCtrl_2(0x01, 0x01, 20, 0x00) # 20% vel suave

# Enviar trayectoria progresiva
steps = 40
for i in range(steps):
    alpha = (i + 1) / steps
    cur_target = round(raw_j[5] + alpha * ({delta_deg} * 1000))
    piper.JointCtrl(raw_j[0], raw_j[1], raw_j[2], raw_j[3], raw_j[4], cur_target)
    time.sleep(0.02)

time.sleep(0.5)
final_j6 = piper.GetArmJointMsgs().joint_state.joint_6 / 1000.0
print(f'FINAL_J6|{{final_j6:.3f}}')
"""
    raw = run_on_jetson(f"python3 -c \"{script}\"")
    for line in raw.splitlines():
        if "FINAL_J6|" in line:
            final_j6 = float(line.split("|")[1])
            print(f"[+] Articulacion 6 ajustada con exito a: {final_j6:.3f}°")
            return
    print(raw)


def main():
    parser = argparse.ArgumentParser(description="Base@Home - Control y Cinematica de Brazo AgileX Piper")
    parser.add_argument("--gripper", type=float, default=None, help="Mover pinza a apertura en mm (0 = cerrado, 75 = abierto)")
    parser.add_argument("--wrist", type=float, default=None, help="Mover levemente Joint 6 (muneca/camara) en delta grados (+/- 5.0 deg)")
    parser.add_argument("--stream", action="store_true", help="Monitor continuo de cinematica a 5 Hz")
    args = parser.parse_args()

    if not check_can_interface():
        sys.exit(1)

    if args.gripper is not None:
        test_gripper(args.gripper)
    elif args.wrist is not None:
        test_wrist_jog(args.wrist)
    elif args.stream:
        print("[*] Iniciando monitor continuo (Presione Ctrl+C para salir)...")
        try:
            while True:
                print("\033[2J\033[H", end="")
                get_telemetry()
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n[*] Monitor detenido.")
    else:
        get_telemetry()


if __name__ == "__main__":
    main()
