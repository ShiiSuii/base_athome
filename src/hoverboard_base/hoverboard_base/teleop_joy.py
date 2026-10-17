#!/usr/bin/env python3
"""
Interactive Bluetooth PS4 / Gamepad Teleoperation Node for ROS 2.
Supports Sony DualShock 4 Wireless Controller natively:
- Dual-stick drive: Left Stick or Right Stick can be used to steer and drive!
- Deadman Safety: R1 or L1 shoulder button (must be held to drive)
- Waypoint Recording: Press [R2] to mark current pose in map as a persistent waypoint!
- Delete Last Waypoint: Press [Share] button to remove the last marked point
- Turbo Boost: [L2] trigger (instant maximum linear and angular speed!)
- Dynamic Speed Regulation (5 Gears each for Linear and Angular):
    * Linear (+ / -):   D-Pad UP / Triangle (△) [Subir]  ||  D-Pad DOWN / Cross (✕) [Bajar]
    * Angular (+ / -):  D-Pad RIGHT / Circle (◯) [Giro Rápido]  ||  D-Pad LEFT / Square (▢) [Giro Suave]
- Real-time RViz2 3D visual markers for waypoints (/waypoint_markers)
- Live interactive HUD in terminal with clear gear and waypoint indicators
"""

import sys
import os
import time
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


WAYPOINTS_FILE_PRIMARY = '/home/athome/base_athome_ws/src/base_athome_navigation/config/waypoints.yaml'
WAYPOINTS_FILE_USER = os.path.expanduser('~/waypoints_base_athome.yaml')


class HoverboardTeleopJoy(Node):
    # 5 Niveles de velocidad lineal (m/s)
    SPEED_LEVELS = [0.15, 0.30, 0.50, 0.75, 1.00]
    
    # 10 Niveles de velocidad angular / giro (rad/s)
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
        self.prev_r2_pressed = False
        self.prev_btn_share = False

        # --- TF Listener para captura de waypoints en marco 'map' ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Cargar waypoints existentes
        self.waypoints = self.load_waypoints()

        # ROS 2 Interfaces
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 20)

        # Publicador de Marcadores RViz2 (Transient Local para persistencia)
        marker_qos = QoSProfile(depth=10)
        marker_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        marker_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub_markers = self.create_publisher(MarkerArray, '/waypoint_markers', marker_qos)

        # Loop de publicación de cmd_vel a 20 Hz
        self.timer = self.create_timer(0.05, self.publish_loop)

        # Timer para refrescar marcadores en RViz periódicamente (cada 2s)
        self.timer_markers = self.create_timer(2.0, self.publish_waypoint_markers)

        # Publicar marcadores iniciales
        self.publish_waypoint_markers()

        print("\n" + "=" * 74)
        print("   🎮 TELEOPERACIÓN PS4 DUALSHOCK 4 (HOVERBOARD ROS 2)        ")
        print("=" * 74)
        print("  Controles de Conducción:                                     ")
        print("    [Stick L o R]          : Avanzar / Retroceder y Girar      ")
        print("    [R1] o [L1]            : HOMBRE MUERTO (Mantener presionado) ")
        print("    [L2]                   : MODO TURBO (Máxima potencia)      ")
        print("                                                               ")
        print("  Gestión de Waypoints (Mapeo y Rutas):                        ")
        print("    📍 [R2]                : MARCAR Y GUARDAR PUNTO ACTUAL     ")
        print("    🗑️  [Share (Botón 8)]   : BORRAR ÚLTIMO PUNTO GUARDADO      ")
        print("                                                               ")
        print("  Regulación de Velocidad de Giro:                             ")
        print("    👉 [Círculo (◯)] / [D-Pad ►] : AUMENTAR GIRO (+ Rápido)     ")
        print("    👈 [Cuadrado (▢)] / [D-Pad ◄] : DISMINUIR GIRO (- Suave)    ")
        print("                                                               ")
        print("  Regulación de Velocidad de Avance:                           ")
        print("    👆 [Triángulo (△)] / [D-Pad ▲]: SUBIR VELOCIDAD (+ Avance)  ")
        print("    👇 [Cruz (✕)] / [D-Pad ▼]    : BAJAR VELOCIDAD (- Avance)  ")
        print("=" * 74)
        self.print_gear_status()

    def load_waypoints(self):
        """Carga waypoints previos desde archivo YAML."""
        path = WAYPOINTS_FILE_PRIMARY if os.path.exists(WAYPOINTS_FILE_PRIMARY) else WAYPOINTS_FILE_USER
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    data = yaml.safe_load(f)
                    if data and 'waypoints' in data and isinstance(data['waypoints'], dict):
                        return data['waypoints']
            except Exception as e:
                self.get_logger().warn(f"No se pudo cargar {path}: {e}")
        return {}

    def save_waypoints_to_file(self):
        """Guarda los waypoints en ambos destinos YAML."""
        data = {'waypoints': self.waypoints}
        for p in [WAYPOINTS_FILE_PRIMARY, WAYPOINTS_FILE_USER]:
            try:
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p, 'w') as f:
                    yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            except Exception as e:
                self.get_logger().error(f"Error guardando en {p}: {e}")

    def print_gear_status(self):
        max_v = self.SPEED_LEVELS[self.linear_gear]
        max_w = self.ANGULAR_LEVELS[self.angular_gear]
        deg_s = math.degrees(max_w)
        wp_list = list(self.waypoints.keys())
        print(f"\n⚡ CONFIGURACIÓN ACTUAL:")
        print(f"   🚗 Avance Lineal: [Nivel {self.linear_gear + 1}/{len(self.SPEED_LEVELS)}] -> {max_v:.2f} m/s")
        print(f"   🔄 Giro Angular : [Nivel {self.angular_gear + 1}/{len(self.ANGULAR_LEVELS)}] -> {max_w:.2f} rad/s (~{deg_s:.0f}°/s)")
        print(f"   📍 Puntos Guardados: {wp_list if wp_list else 'Ninguno aún (Presiona R2 para marcar)'}\n")

    def record_waypoint(self):
        """Captura la pose actual de base_footprint en map y la guarda."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.3)
            )
        except Exception as e:
            print("\n" + "!" * 74)
            print("⚠️  [ERROR AL MARCAR PUNTO CON R2]")
            print("   No se pudo obtener la pose del robot en el mapa ('map' -> 'base_footprint').")
            print("   👉 Asegúrate de que AMCL (navegacion) o SLAM esté activo y localizado.")
            print(f"   Detalle: {e}")
            print("!" * 74 + "\n")
            return

        x = round(t.transform.translation.x, 4)
        y = round(t.transform.translation.y, 4)
        z = round(t.transform.translation.z, 4)
        qx = round(t.transform.rotation.x, 4)
        qy = round(t.transform.rotation.y, 4)
        qz = round(t.transform.rotation.z, 4)
        qw = round(t.transform.rotation.w, 4)

        # Cálculo de ángulo yaw en grados
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)
        yaw_deg = round(math.degrees(yaw_rad), 1)

        # Asignar siguiente letra disponible: 'A', 'B', 'C'...
        existing = set(self.waypoints.keys())
        letter_idx = 0
        while True:
            candidate = chr(ord('A') + letter_idx) if letter_idx < 26 else f"P{letter_idx+1}"
            if candidate not in existing:
                wp_name = candidate
                break
            letter_idx += 1

        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self.waypoints[wp_name] = {
            'name': wp_name,
            'x': x,
            'y': y,
            'z': z,
            'qx': qx,
            'qy': qy,
            'qz': qz,
            'qw': qw,
            'yaw_deg': yaw_deg,
            'timestamp': timestamp
        }

        self.save_waypoints_to_file()
        self.publish_waypoint_markers()

        print("\n" + "=" * 74)
        print(f"📍 [NUEVO PUNTO GUARDADO CON R2] -> Punto '{wp_name}'")
        print(f"   Posición   : X = {x:+.3f} m  |  Y = {y:+.3f} m")
        print(f"   Orientación: Yaw = {yaw_deg:+6.1f}° (Qz={qz:+.3f}, Qw={qw:+.3f})")
        print(f"   Total puntos guardados: {list(self.waypoints.keys())}")
        print(f"   Archivo    : {WAYPOINTS_FILE_PRIMARY}")
        print("=" * 74 + "\n")

    def delete_last_waypoint(self):
        """Borra el último waypoint añadido si se presiona el botón Share."""
        if not self.waypoints:
            print("\n⚠️ No hay puntos para borrar.")
            return

        last_key = list(self.waypoints.keys())[-1]
        del self.waypoints[last_key]
        self.save_waypoints_to_file()
        self.publish_waypoint_markers()

        print("\n" + "-" * 74)
        print(f"🗑️ [PUNTO ELIMINADO CON SHARE] -> Se eliminó el Punto '{last_key}'")
        print(f"   Puntos restantes: {list(self.waypoints.keys()) if self.waypoints else 'Ninguno'}")
        print("-" * 74 + "\n")

    def publish_waypoint_markers(self):
        """Publica marcadores 3D en RViz2."""
        marker_array = MarkerArray()
        # Comando para borrar todo antes de repintar
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        marker_array.markers.append(del_marker)

        m_id = 0
        now = self.get_clock().now().to_msg()
        for name, data in self.waypoints.items():
            # Flecha que indica orientación y ubicación en el piso
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = now
            arrow.ns = 'waypoints_arrows'
            arrow.id = m_id
            m_id += 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = float(data['x'])
            arrow.pose.position.y = float(data['y'])
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.x = float(data.get('qx', 0.0))
            arrow.pose.orientation.y = float(data.get('qy', 0.0))
            arrow.pose.orientation.z = float(data.get('qz', 0.0))
            arrow.pose.orientation.w = float(data.get('qw', 1.0))
            arrow.scale.x = 0.45
            arrow.scale.y = 0.09
            arrow.scale.z = 0.09
            arrow.color.r = 0.0
            arrow.color.g = 0.85
            arrow.color.b = 1.0
            arrow.color.a = 0.95
            marker_array.markers.append(arrow)

            # Etiqueta de texto flotante encima de la flecha
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = now
            text.ns = 'waypoints_labels'
            text.id = m_id
            m_id += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(data['x'])
            text.pose.position.y = float(data['y'])
            text.pose.position.z = 0.38
            text.scale.z = 0.22
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 0.0
            text.color.a = 1.0
            text.text = f"📍 Punto {name}"
            marker_array.markers.append(text)

        self.pub_markers.publish(marker_array)

    def joy_callback(self, msg: Joy):
        self.last_joy_time = time.time()
        axes = msg.axes
        buttons = msg.buttons

        # 1. Hombre muerto: R1 (botón 5) o L1 (botón 4)
        r1 = bool(buttons[5]) if len(buttons) > 5 else False
        l1 = bool(buttons[4]) if len(buttons) > 4 else False
        self.enable_pressed = r1 or l1

        # 2. Modo Turbo: gatillo analógico L2 (Axis 2 < -0.2) o botón L2 (botón 6)
        l2_btn = bool(buttons[6]) if len(buttons) > 6 else False
        l2_axis = (axes[2] < -0.2) if len(axes) > 2 else False
        self.turbo_pressed = l2_btn or l2_axis

        # 3. MARCAR WAYPOINT CON R2 (Flanco ascendente)
        r2_btn = bool(buttons[7]) if len(buttons) > 7 else False
        r2_axis = (axes[5] < -0.5) if len(axes) > 5 else False
        r2_pressed = r2_btn or r2_axis

        if r2_pressed and not self.prev_r2_pressed:
            self.record_waypoint()
        self.prev_r2_pressed = r2_pressed

        # 4. BORRAR ÚLTIMO WAYPOINT CON BOTÓN SHARE (Botón 8)
        btn_share = bool(buttons[8]) if len(buttons) > 8 else False
        if btn_share and not self.prev_btn_share:
            self.delete_last_waypoint()
        self.prev_btn_share = btn_share

        # 5. REGULACIÓN DE VELOCIDAD LINEAL (Avance)
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

        # 6. REGULACIÓN DE VELOCIDAD ANGULAR (Giro)
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

        # 7. LECTURA DE STICKS ANALÓGICOS (Stick Izquierdo o Derecho)
        raw_lin_left = axes[1] if len(axes) > 1 else 0.0
        raw_ang_left = axes[0] if len(axes) > 0 else 0.0
        raw_ang_right = axes[3] if len(axes) > 3 else 0.0

        raw_lin = raw_lin_left
        raw_ang = raw_ang_right if abs(raw_ang_right) > abs(raw_ang_left) else raw_ang_left

        # Zona muerta contra drift analógico
        if abs(raw_lin) < 0.05:
            raw_lin = 0.0
        if abs(raw_ang) < 0.05:
            raw_ang = 0.0

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
                status_text = "🔥 TURBO ACTIVO [L2]"
        else:
            status_text = "🟡 PULSAR R1 O L1 PARA MOVER"
            if self.stop_packets_sent < 3:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.pub_cmd.publish(cmd)
                self.stop_packets_sent += 1

        # HUD interactivo en tiempo real con indicador de Waypoints
        max_v = self.SPEED_LEVELS[self.linear_gear]
        max_w = self.ANGULAR_LEVELS[self.angular_gear]
        wp_count = len(self.waypoints)
        hud = (f"\r[{status_text}] "
               f"Avance: Nvl {self.linear_gear+1} ({max_v:.2f}m/s) | "
               f"Giro: Nvl {self.angular_gear+1} ({max_w:.2f}r/s) | "
               f"📍 Ptos: {wp_count} [R2: Marcar] || "
               f"CMD: V={cmd.linear.x:+5.2f}, W={cmd.angular.z:+5.2f}  ")
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
