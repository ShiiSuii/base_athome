#!/usr/bin/env python3
"""
Pan-Tilt Interactive Calibration Tool with Auto-Reconnect & Fault-Tolerance
RoboCup @Home Autonomous Robotics Architecture
"""

import os
import sys
import glob
import time
import math
import yaml
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
PAN_ID = 1
TILT_ID = 2

ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VEL = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE = 144
ADDR_HW_ERROR = 70

CONFIG_PATH = os.path.expanduser("~/athome_ws/src/pan_tilt_controller/config/pan_tilt_params.yaml")


class RobustDynamixelController:
    def __init__(self):
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.port = None
        self.device_name = None
        self.connect()

    def find_port(self):
        candidates = [
            "/dev/ttyU2D2",
            "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7W8ZD1-if00-port0",
            "/dev/ttyUSB0",
            "/dev/ttyUSB1",
        ]
        for c in candidates:
            if os.path.exists(c):
                return c
        ftdi = glob.glob("/dev/serial/by-id/*FTDI*")
        if ftdi:
            return ftdi[0]
        return "/dev/ttyU2D2"

    def connect(self):
        if self.port:
            try:
                self.port.closePort()
            except Exception:
                pass

        for attempt in range(5):
            self.device_name = self.find_port()
            try:
                self.port = PortHandler(self.device_name)
                if self.port.openPort() and self.port.setBaudRate(BAUDRATE):
                    return True
            except Exception:
                pass
            time.sleep(0.4)
        return False

    def ensure_connected(self):
        try:
            # Quick ping test
            _, res, _ = self.packet.ping(self.port, PAN_ID)
            if res == COMM_SUCCESS:
                return True
        except Exception:
            pass
        # Reconnect
        print("\n[Aviso: Reconectando con U2D2 por movimiento del cable USB...]")
        time.sleep(0.5)
        return self.connect()

    def read_pos(self, dxl_id):
        for _ in range(4):
            try:
                pos, res, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
                if res == COMM_SUCCESS:
                    return pos
            except Exception:
                pass
            self.ensure_connected()
            time.sleep(0.1)
        return None

    def read_telemetry(self, dxl_id):
        for _ in range(4):
            try:
                pos, res1, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
                vol, res2, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_VOLTAGE)
                err, res3, _ = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_HW_ERROR)
                tq, res4, _ = self.packet.read1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE)
                if res1 == COMM_SUCCESS and res2 == COMM_SUCCESS:
                    return {"pos": pos, "voltage": vol / 10.0, "error": err, "torque": tq}
            except Exception:
                pass
            self.ensure_connected()
            time.sleep(0.1)
        return None

    def set_torque(self, enable=True):
        val = 1 if enable else 0
        self.ensure_connected()
        try:
            self.packet.write1ByteTxRx(self.port, PAN_ID, ADDR_TORQUE_ENABLE, val)
            self.packet.write1ByteTxRx(self.port, TILT_ID, ADDR_TORQUE_ENABLE, val)
        except Exception:
            pass

    def move_to(self, pan_pos, tilt_pos, vel=80, accel=15):
        self.ensure_connected()
        try:
            self.packet.write4ByteTxRx(self.port, PAN_ID, ADDR_PROFILE_ACCEL, accel)
            self.packet.write4ByteTxRx(self.port, PAN_ID, ADDR_PROFILE_VEL, vel)
            self.packet.write4ByteTxRx(self.port, TILT_ID, ADDR_PROFILE_ACCEL, accel)
            self.packet.write4ByteTxRx(self.port, TILT_ID, ADDR_PROFILE_VEL, vel)
            self.packet.write1ByteTxRx(self.port, PAN_ID, ADDR_TORQUE_ENABLE, 1)
            self.packet.write1ByteTxRx(self.port, TILT_ID, ADDR_TORQUE_ENABLE, 1)
            self.packet.write4ByteTxRx(self.port, PAN_ID, ADDR_GOAL_POSITION, int(pan_pos))
            self.packet.write4ByteTxRx(self.port, TILT_ID, ADDR_GOAL_POSITION, int(tilt_pos))
        except Exception as e:
            print(f"Error en move_to: {e}")

    def close(self):
        if self.port:
            try:
                self.port.closePort()
            except Exception:
                pass


def capture_step(ctrl, step_name, instructions):
    print(f"\n{'-' * 60}")
    print(f"  {step_name}")
    print(f"{'-' * 60}")
    print(instructions)

    # Show live position until ENTER is pressed
    import select
    print("Posicion en vivo: ", end="", flush=True)

    while True:
        p_pos = ctrl.read_pos(PAN_ID)
        t_pos = ctrl.read_pos(TILT_ID)
        if p_pos is not None and t_pos is not None:
            print(f"\rPan (ID 1): {p_pos:4d} ticks | Tilt (ID 2): {t_pos:4d} ticks  [Presiona ENTER para fijar] ", end="", flush=True)

        # Check if user pressed ENTER
        if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
            sys.stdin.readline()
            break
        time.sleep(0.08)

    p_pos = ctrl.read_pos(PAN_ID)
    t_pos = ctrl.read_pos(TILT_ID)
    print(f"\n--> Capturado: Pan = {p_pos} ticks, Tilt = {t_pos} ticks")
    return p_pos, t_pos


def main():
    print("=" * 60)
    print("  ROBOTIS Dynamixel Pan-Tilt Calibration (Resilient)")
    print("=" * 60)

    ctrl = RobustDynamixelController()

    tel1 = ctrl.read_telemetry(PAN_ID)
    tel2 = ctrl.read_telemetry(TILT_ID)

    if not tel1 or not tel2:
        print("Error: No se pudo conectar con los servos. Verifica que el cable USB y los 12V esten firmes.")
        sys.exit(1)

    print(f"Dispositivo: {ctrl.device_name}")
    print(f"Motor 1 (Pan):  Voltaje = {tel1['voltage']}V, Pos = {tel1['pos']} ticks")
    print(f"Motor 2 (Tilt): Voltaje = {tel2['voltage']}V, Pos = {tel2['pos']} ticks")

    # Disable torque for teach mode
    ctrl.set_torque(False)

    print("\n[MODO CALIBRACION MANUAL (TEACH MODE)]")
    print("Torque desactivado. Mueve la cabeza con la mano.")
    print("NOTA: Asegurate de que el cable USB del U2D2 tenga suficiente holgura")
    print("para no tirantearse al girar hacia los lados.")

    # Step 1: Center
    pan_center, tilt_center = capture_step(
        ctrl,
        "PASO 1: Posicion de CENTRO (Mirando al frente, horizontal)",
        "Ubica la camara apuntando exactamente hacia el frente y horizontal."
    )

    # Step 2: Left
    pan_left, _ = capture_step(
        ctrl,
        "PASO 2: Limite Maximo a la IZQUIERDA (Pan Izquierda)",
        "Mueve suavemente la cabeza hacia la izquierda hasta el maximo deseado."
    )

    # Step 3: Right
    pan_right, _ = capture_step(
        ctrl,
        "PASO 3: Limite Maximo a la DERECHA (Pan Derecha)",
        "Mueve suavemente la cabeza hacia la derecha hasta el maximo deseado."
    )

    # Step 4: Up
    _, tilt_up = capture_step(
        ctrl,
        "PASO 4: Limite Maximo hacia ARRIBA (Tilt Up)",
        "Mueve suavemente la camara inclinada hacia arriba hasta el maximo deseado."
    )

    # Step 5: Down
    _, tilt_down = capture_step(
        ctrl,
        "PASO 5: Limite Maximo hacia ABAJO (Tilt Down)",
        "Mueve suavemente la camara inclinada hacia abajo hasta el maximo deseado."
    )

    # Angle conversions
    pan_left_deg = (pan_left - pan_center) * 360.0 / 4096.0
    pan_right_deg = (pan_right - pan_center) * 360.0 / 4096.0
    tilt_up_deg = (tilt_up - tilt_center) * 360.0 / 4096.0
    tilt_down_deg = (tilt_down - tilt_center) * 360.0 / 4096.0

    pan_min_rad = min(pan_left_deg, pan_right_deg) * math.pi / 180.0
    pan_max_rad = max(pan_left_deg, pan_right_deg) * math.pi / 180.0
    tilt_min_rad = min(tilt_up_deg, tilt_down_deg) * math.pi / 180.0
    tilt_max_rad = max(tilt_up_deg, tilt_down_deg) * math.pi / 180.0

    calib_data = {
        "pan_tilt_node": {
            "ros__parameters": {
                "device_name": "/dev/ttyU2D2",
                "baudrate": BAUDRATE,
                "protocol_version": PROTOCOL_VERSION,
                "pan_id": PAN_ID,
                "tilt_id": TILT_ID,
                "pan_center_tick": int(pan_center),
                "tilt_center_tick": int(tilt_center),
                "pan_min_rad": float(round(pan_min_rad, 4)),
                "pan_max_rad": float(round(pan_max_rad, 4)),
                "tilt_min_rad": float(round(tilt_min_rad, 4)),
                "tilt_max_rad": float(round(tilt_max_rad, 4)),
                "ticks_per_rad": 651.8986,
                "publish_rate_hz": 20.0,
            }
        }
    }

    print("\n" + "=" * 60)
    print("  RESUMEN DE CALIBRACION")
    print("=" * 60)
    print(f"  Pan Center:  {pan_center} ticks (0.0 rad)")
    print(f"  Pan Limits:  [{pan_min_rad:.2f} rad, {pan_max_rad:.2f} rad] ({min(pan_left_deg, pan_right_deg):.1f}° a {max(pan_left_deg, pan_right_deg):.1f}°)")
    print(f"  Tilt Center: {tilt_center} ticks (0.0 rad)")
    print(f"  Tilt Limits: [{tilt_min_rad:.2f} rad, {tilt_max_rad:.2f} rad] ({min(tilt_up_deg, tilt_down_deg):.1f}° a {max(tilt_up_deg, tilt_down_deg):.1f}°)")

    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    with open(CONFIG_PATH, "w") as f:
        yaml.dump(calib_data, f, default_flow_style=False)
    print(f"\nConfiguracion guardada exitosamente en: {CONFIG_PATH}")

    # Self test
    try:
        ans = input("\n¿Quieres realizar un auto-test de movimiento suave ahora? (s/n): ").strip().lower()
    except Exception:
        ans = 's'

    if ans in ['s', 'y', 'si']:
        print("\nActivando torque con aceleracion suave...")
        ctrl.move_to(pan_center, tilt_center, vel=70, accel=15)
        time.sleep(2.0)

        pan_delta = int(20.0 * 4096.0 / 360.0)
        tilt_delta = int(12.0 * 4096.0 / 360.0)

        print("Probando Pan Izquierda (+20°)...")
        ctrl.move_to(pan_center + pan_delta, tilt_center)
        time.sleep(1.5)

        print("Probando Pan Derecha (-20°)...")
        ctrl.move_to(pan_center - pan_delta, tilt_center)
        time.sleep(1.5)

        print("Probando Tilt Arriba (+12°)...")
        ctrl.move_to(pan_center, tilt_center + tilt_delta)
        time.sleep(1.5)

        print("Probando Tilt Abajo (-12°)...")
        ctrl.move_to(pan_center, tilt_center - tilt_delta)
        time.sleep(1.5)

        print("Regresando al CENTRO...")
        ctrl.move_to(pan_center, tilt_center)
        time.sleep(1.5)
        print("¡Auto-test completado con exito!")

    ctrl.close()
    print("\n¡Calibracion lista!")


if __name__ == "__main__":
    main()
