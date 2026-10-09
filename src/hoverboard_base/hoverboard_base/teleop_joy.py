#!/usr/bin/env python3
"""
Interactive Bluetooth PS4 / Gamepad Teleoperation Node for ROS 2.
Supports Sony DualShock 4 Wireless Controller natively:
- Dual-stick drive: Left Stick or Right Stick can be used to steer and drive!
- Deadman Safety: R1 or L1 shoulder button (must be held to drive)
- Turbo Boost: R2 trigger (instant maximum linear and angular speed!)
- Dynamic Speed Regulation (5 Gears each for Linear and Angular):
    * Linear (+ / -):   D-Pad UP / Triangle (△) [Subir]  ||  D-Pad DOWN / Cross (✕) [Bajar]
    * Angular (+ / -):  D-Pad RIGHT / Circle (◯) [Giro Rápido]  ||  D-Pad LEFT / Square (▢) [Giro Suave]
- Live interactive HUD in terminal with clear gear indicators
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import sys
import time
import math


class HoverboardTeleopJoy(Node):
    # 5 Niveles de velocidad lineal (m/s)
    SPEED_LEVELS = [0.15, 0.30, 0.50, 0.75, 1.00]
    
    # 10 Niveles de velocidad angular / giro (rad/s)
    # Rango ultra-amplio: desde micro-ajustes / docking milimétrico (0.05 rad/s ~ 2.9°/s)
    # hasta giros rápidos y ágiles (3.60 rad/s ~ 206°/s)
    ANGULAR_LEVELS = [0.05, 0.10, 0.18, 0.30, 0.50, 0.80, 1.20, 1.80, 2.60, 3.60]

    def __init__(self):
        super().__init__('hoverboard_teleop_joy')

        # --- Parámetros de Mapeo (DualShock 4 Linux) ---
        self.declare_parameter('axis_linear', 1)        # Stick izquierdo vertical (Axis 1)
        self.declare_parameter('axis_angular', 0)       # Stick izquierdo horizontal (Axis 0)
        self.declare_parameter('initial_linear_gear', 2)   # Nivel 3 por defecto (0.50 m/s)
        self.declare_parameter('initial_angular_gear', 5)  # Nivel 6 por defecto (0.80 rad/s)

        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value

        # Estado de marchas
        self.linear_gear = self.get_parameter('initial_linear_gear').value
        self.angular_gear = self.get_parameter('initial_angular_gear').value

        # Estado del mando
        self.linear_val = 0.0
        self.angular_val = 0.0
        self.enable_pressed = False
        self.turbo_pressed = False
        self.last_joy_time = 0.0
        self.stop_packets_sent = 3  # Silencioso al inicio para no interferir con Nav2

        # Flancos anteriores para detección de pulsación única
        self.prev_dpad_v = 0.0
        self.prev_dpad_h = 0.0
        self.prev_btn_up = 0
        self.prev_btn_down = 0
        self.prev_btn_turn_up = 0
        self.prev_btn_turn_down = 0

        # ROS 2 Interfaces
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 20)

        # Loop de publicación a 20 Hz
        self.timer = self.create_timer(0.05, self.publish_loop)

        print("\n" + "=" * 70)
        print("   🎮 TELEOPERACIÓN PS4 DUALSHOCK 4 (HOVERBOARD ROS 2)        ")
        print("=" * 70)
        print("  Controles de Conducción:                                     ")
        print("    [Stick L o R]          : Avanzar / Retroceder y Girar      ")
        print("    [R1] o [L1]            : HOMBRE MUERTO (Mantener presionado) ")
        print("    [R2]                   : MODO TURBO (Máxima potencia)      ")
        print("                                                               ")
        print("  Regulación de Velocidad de Giro:                             ")
        print("    👉 [Círculo (◯)] / [D-Pad ►] : AUMENTAR GIRO (+ Rápido)     ")
        print("    👈 [Cuadrado (▢)] / [D-Pad ◄] : DISMINUIR GIRO (- Suave)    ")
        print("                                                               ")
        print("  Regulación de Velocidad de Avance:                           ")
        print("    👆 [Triángulo (△)] / [D-Pad ▲]: SUBIR VELOCIDAD (+ Avance)  ")
        print("    👇 [Cruz (✕)] / [D-Pad ▼]    : BAJAR VELOCIDAD (- Avance)  ")
        print("=" * 70)
        self.print_gear_status()

    def print_gear_status(self):
        max_v = self.SPEED_LEVELS[self.linear_gear]
        max_w = self.ANGULAR_LEVELS[self.angular_gear]
        deg_s = math.degrees(max_w)
        print(f"\n⚡ CONFIGURACIÓN ACTUAL:")
        print(f"   🚗 Avance Lineal: [Nivel {self.linear_gear + 1}/{len(self.SPEED_LEVELS)}] -> {max_v:.2f} m/s")
        print(f"   🔄 Giro Angular : [Nivel {self.angular_gear + 1}/{len(self.ANGULAR_LEVELS)}] -> {max_w:.2f} rad/s (~{deg_s:.0f}°/s)\n")

    def joy_callback(self, msg: Joy):
        self.last_joy_time = time.time()
        axes = msg.axes
        buttons = msg.buttons

        # 1. Hombre muerto: R1 (botón 5) o L1 (botón 4)
        r1 = bool(buttons[5]) if len(buttons) > 5 else False
        l1 = bool(buttons[4]) if len(buttons) > 4 else False
        self.enable_pressed = r1 or l1

        # 2. Modo Turbo: R2 botón (botón 7) o gatillo analógico R2 presionado (Axis 5 < -0.2)
        r2_btn = bool(buttons[7]) if len(buttons) > 7 else False
        r2_axis = (axes[5] < -0.2) if len(axes) > 5 else False
        self.turbo_pressed = r2_btn or r2_axis

        # 3. REGULACIÓN DE VELOCIDAD LINEAL (Avance)
        # D-Pad vertical (Axis 7: Arriba=+1, Abajo=-1) | Triángulo (botón 2) | Cruz (botón 0)
        dpad_v = axes[7] if len(axes) > 7 else 0.0
        btn_v_up = buttons[2] if len(buttons) > 2 else 0
        btn_v_down = buttons[0] if len(buttons) > 0 else 0

        # Subir marcha lineal
        if (dpad_v > 0.5 and self.prev_dpad_v <= 0.5) or (btn_v_up and not self.prev_btn_up):
            if self.linear_gear < len(self.SPEED_LEVELS) - 1:
                self.linear_gear += 1
                self.print_gear_status()

        # Bajar marcha lineal
        elif (dpad_v < -0.5 and self.prev_dpad_v >= -0.5) or (btn_v_down and not self.prev_btn_down):
            if self.linear_gear > 0:
                self.linear_gear -= 1
                self.print_gear_status()

        self.prev_dpad_v = dpad_v
        self.prev_btn_up = btn_v_up
        self.prev_btn_down = btn_v_down

        # 4. REGULACIÓN DE VELOCIDAD ANGULAR (Giro)
        # D-Pad horizontal (Axis 6: Derecha=-1, Izquierda=+1) | Círculo (botón 1) | Cuadrado (botón 3)
        dpad_h = axes[6] if len(axes) > 6 else 0.0
        btn_turn_up = buttons[1] if len(buttons) > 1 else 0      # Círculo (◯) -> + Giro
        btn_turn_down = buttons[3] if len(buttons) > 3 else 0    # Cuadrado (▢) -> - Giro

        # Subir giro (D-Pad Derecha o Círculo)
        if (dpad_h < -0.5 and self.prev_dpad_h >= -0.5) or (btn_turn_up and not self.prev_btn_turn_up):
            if self.angular_gear < len(self.ANGULAR_LEVELS) - 1:
                self.angular_gear += 1
                self.print_gear_status()

        # Bajar giro (D-Pad Izquierda o Cuadrado)
        elif (dpad_h > 0.5 and self.prev_dpad_h <= 0.5) or (btn_turn_down and not self.prev_btn_turn_down):
            if self.angular_gear > 0:
                self.angular_gear -= 1
                self.print_gear_status()

        self.prev_dpad_h = dpad_h
        self.prev_btn_turn_up = btn_turn_up
        self.prev_btn_turn_down = btn_turn_down

        # 5. LECTURA DE STICKS ANALÓGICOS (Soporta Stick Izquierdo o Stick Derecho)
        # Stick izquierdo
        raw_lin_left = axes[1] if len(axes) > 1 else 0.0
        raw_ang_left = axes[0] if len(axes) > 0 else 0.0
        # Stick derecho
        raw_ang_right = axes[3] if len(axes) > 3 else 0.0

        # Si el stick derecho se usa para girar, tiene prioridad o se combina
        raw_lin = raw_lin_left
        raw_ang = raw_ang_right if abs(raw_ang_right) > abs(raw_ang_left) else raw_ang_left

        # Zona muerta para evitar derivas por desgaste del joystick
        if abs(raw_lin) < 0.05:
            raw_lin = 0.0
        if abs(raw_ang) < 0.05:
            raw_ang = 0.0

        # Escalamiento según marcha seleccionada o Turbo
        max_linear = 1.0 if self.turbo_pressed else self.SPEED_LEVELS[self.linear_gear]
        max_angular = self.ANGULAR_LEVELS[-1] if self.turbo_pressed else self.ANGULAR_LEVELS[self.angular_gear]

        self.linear_val = raw_lin * max_linear
        self.angular_val = raw_ang * max_angular

    def publish_loop(self):
        cmd = Twist()

        if self.enable_pressed and (time.time() - self.last_joy_time < 0.5):
            cmd.linear.x = float(self.linear_val)
            cmd.angular.z = float(self.angular_val)
            self.pub_cmd.publish(cmd)
            self.stop_packets_sent = 0
            status_text = "🟢 CONDUCIENDO [R1/L1]"
            if self.turbo_pressed:
                status_text = "🔥 TURBO ACTIVO [R2]"
        else:
            status_text = "🟡 PULSAR R1 O L1 PARA MOVER"
            if self.stop_packets_sent < 3:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.stop_packets_sent += 1
            # Si stop_packets_sent >= 3, no publicamos para dejar /cmd_vel libre para Nav2

        # HUD interactivo en tiempo real
        max_v = self.SPEED_LEVELS[self.linear_gear]
        max_w = self.ANGULAR_LEVELS[self.angular_gear]
        hud = (f"\r[{status_text}] "
               f"Avance: Nvl {self.linear_gear+1}/{len(self.SPEED_LEVELS)} ({max_v:.2f}m/s) | "
               f"Giro: Nvl {self.angular_gear+1}/{len(self.ANGULAR_LEVELS)} ({max_w:.2f}r/s) || "
               f"CMD: Vel={cmd.linear.x:+5.2f}m/s, Giro={cmd.angular.z:+5.2f}r/s   ")
        sys.stdout.write(hud)
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = HoverboardTeleopJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            stop_cmd = Twist()
            node.pub_cmd.publish(stop_cmd)
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        print("\n\n[+] Teleoperación finalizada con seguridad.")


if __name__ == '__main__':
    main()
