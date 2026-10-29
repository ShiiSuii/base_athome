#!/usr/bin/env python3
"""
Manual Motor Identification & Jogging Tool for Dynamixel XL430-W250
"""

import sys
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICENAME = "/dev/ttyU2D2"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VEL = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_VOLTAGE = 144


def connect():
    port = PortHandler(DEVICENAME)
    packet = PacketHandler(PROTOCOL_VERSION)
    if not port.openPort():
        print(f"Error abriendo {DEVICENAME}")
        sys.exit(1)
    if not port.setBaudRate(BAUDRATE):
        print(f"Error fijando baudrate {BAUDRATE}")
        sys.exit(1)
    return port, packet


def enable_smooth_torque(port, packet, dxl_id):
    # Safe slow velocity and acceleration
    packet.write4ByteTxRx(port, dxl_id, ADDR_PROFILE_ACCEL, 15)
    packet.write4ByteTxRx(port, dxl_id, ADDR_PROFILE_VEL, 60)
    packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)


def disable_torque(port, packet, dxl_id):
    packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)


def read_pos(port, packet, dxl_id):
    pos, res, _ = packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    if res != COMM_SUCCESS:
        return None
    return pos


def jog(port, packet, dxl_id, delta_deg):
    cur_pos = read_pos(port, packet, dxl_id)
    if cur_pos is None:
        print(f"Error leyendo motor ID {dxl_id}")
        return
    ticks_delta = int(delta_deg * 4096.0 / 360.0)
    goal_pos = cur_pos + ticks_delta

    # Clamp to valid range 0 - 4095
    goal_pos = max(100, min(3995, goal_pos))

    enable_smooth_torque(port, packet, dxl_id)
    print(f"Moviendo Motor ID {dxl_id}: actual={cur_pos} ticks -> objetivo={goal_pos} ticks ({delta_deg:+.1f}°)...")
    packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal_pos)
    time.sleep(1.0)
    new_pos = read_pos(port, packet, dxl_id)
    print(f"Motor ID {dxl_id} finalizo en: {new_pos} ticks")


def print_status(port, packet):
    print("\n" + "=" * 50)
    for i in [1, 2]:
        pos = read_pos(port, packet, i)
        vol, _, _ = packet.read2ByteTxRx(port, i, ADDR_PRESENT_VOLTAGE)
        tq, _, _ = packet.read1ByteTxRx(port, i, ADDR_TORQUE_ENABLE)
        tq_str = "ON (fijo)" if tq == 1 else "OFF (libre)"
        print(f"Motor ID {i}: Pos={pos} ticks ({(pos-2048)*360/4096:.1f}°), Voltaje={vol/10.0:.1f}V, Torque={tq_str}")
    print("=" * 50)


def main():
    if len(sys.argv) >= 3:
        # Modo comando directo por argumentos: python manual_motor_test.py <id> <delta_deg>
        target_id = int(sys.argv[1])
        delta = float(sys.argv[2])
        port, packet = connect()
        jog(port, packet, target_id, delta)
        port.closePort()
        return

    # Modo interactivo
    port, packet = connect()
    print("=" * 60)
    print("  HERRAMIENTA DE IDENTIFICACION Y PRUEBA DE MOTORES")
    print("=" * 60)
    print("Usa esta herramienta para saber cual motor hace Pan y cual Tilt:")
    print("  [1] Mover Motor ID 1 (+15 grados)")
    print("  [q] Mover Motor ID 1 (-15 grados)")
    print("  [2] Mover Motor ID 2 (+15 grados)")
    print("  [w] Mover Motor ID 2 (-15 grados)")
    print("  [f] APAGAR TORQUE (mover cabeza con la mano)")
    print("  [r] Leer estado / posicion actual")
    print("  [x] Salir")

    while True:
        print_status(port, packet)
        try:
            cmd = input("\nElige una opcion (1/q/2/w/f/r/x): ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            break

        if cmd == '1':
            jog(port, packet, 1, +15.0)
        elif cmd == 'q':
            jog(port, packet, 1, -15.0)
        elif cmd == '2':
            jog(port, packet, 2, +15.0)
        elif cmd == 'w':
            jog(port, packet, 2, -15.0)
        elif cmd == 'f':
            disable_torque(port, packet, 1)
            disable_torque(port, packet, 2)
            print("Torque apagado en ambos motores. Puedes moverlos con la mano.")
        elif cmd == 'r':
            pass
        elif cmd == 'x':
            break
        else:
            print("Opcion no valida.")

    port.closePort()
    print("Finalizado.")


if __name__ == "__main__":
    main()
