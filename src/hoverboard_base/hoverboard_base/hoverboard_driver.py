#!/usr/bin/env python3
"""
Hoverboard ROS 2 Hardware Driver Node
Supports Lenzod and Generic STM32 boards with 22-byte ENABLE_ODOMETRY feedback.
Features:
- Dedicated high-rate background RX thread (prevents dropped serial frames)
- 50 Hz command transmission watchdog
- Closed-loop wheel tick odometry with 9000-modulo unwrap
- TF broadcasting (odom -> base_footprint)
- Telemetry publishing (battery voltage, board temperature, RPM, lwheel/rwheel ticks)
"""

import rclpy
from rclpy.node import Node
import serial
import struct
import math
import time
import threading

import fcntl
import sys

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster


class HoverboardDriver(Node):
    START_FRAME = 0xABCD
    FEEDBACK_SIZE = 22  # 22 bytes when ENABLE_ODOMETRY is active

    def __init__(self):
        super().__init__('hoverboard_driver')

        # --- Parámetros Configurables ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius_m', 0.0825)    # Rueda 6.5 pulgadas (diametro 165mm)
        self.declare_parameter('track_width_m', 0.52)       # Separación entre ruedas
        self.declare_parameter('ticks_per_rev', 90.0)       # 15 pares de polos * 6 estados = 90
        self.declare_parameter('max_speed_raw', 450)        # Límite máximo de velocidad (unidades firmware)
        self.declare_parameter('max_steer_raw', 400)        # Límite máximo de giro
        self.declare_parameter('min_speed_raw', 75)         # Torque mínimo lineal para vencer fricción estática
        self.declare_parameter('min_steer_raw', 165)        # Torque mínimo angular para giro sobre eje (en firmware 165 * 0.5 = 82.5 efectivo)
        self.declare_parameter('linear_scale', 280.0)       # 1.0 m/s -> 280 unidades (más torque por velocidad comandada)
        self.declare_parameter('angular_scale', 260.0)      # 1.0 rad/s -> 260 unidades (torque rotacional firme)
        self.declare_parameter('publish_tf', True)          # Publicar TF odom -> base_link
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_vel_timeout', 0.5)      # Watchdog de seguridad (segundos)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baudrate').value
        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.track_width = float(self.get_parameter('track_width_m').value)
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.max_speed_raw = int(self.get_parameter('max_speed_raw').value)
        self.max_steer_raw = int(self.get_parameter('max_steer_raw').value)
        self.min_speed_raw = int(self.get_parameter('min_speed_raw').value)
        self.min_steer_raw = int(self.get_parameter('min_steer_raw').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.angular_scale = float(self.get_parameter('angular_scale').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)

        # Distancia métrica por tick de sensor Hall
        self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.get_logger().info(
            f"🚀 Iniciando HoverboardDriver en {self.port} @ {self.baud} | "
            f"Radio: {self.wheel_radius:.4f}m | Base: {self.track_width:.3f}m | "
            f"Resolución: {self.meters_per_tick*1000:.3f} mm/tick"
        )

        # --- Estado del Robot ---
        self.target_speed = 0
        self.target_steer = 0
        self.sent_speed = 0
        self.sent_steer = 0
        self.last_cmd_time = time.time()

        # Odometría Pose y Velocidad (protegidos por Lock)
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vx = 0.0
        self.vth = 0.0
        self.last_odom_time = time.time()

        # Encoders
        self.prev_wheelL = None
        self.prev_wheelR = None
        self.total_ticks_L = 0
        self.total_ticks_R = 0

        # Telemetría
        self.bat_voltage = 0.0
        self.board_temp = 0.0
        self.speed_L_rpm = 0
        self.speed_R_rpm = 0
        self.valid_packets = 0
        self.last_rx_time = 0.0
        self.loop_count = 0

        # --- Protección Singleton contra instancias duplicadas compitiendo por el puerto serial ---
        self.lock_file = None
        self._acquire_singleton_lock()

        # --- Conexión Serial ---
        self.ser = None
        self.running = True
        self.connect_serial()

        # Hilo de recepción serial en background
        self.rx_thread = threading.Thread(target=self._rx_worker, daemon=True)
        self.rx_thread.start()

        # --- Suscriptores y Publicadores ROS 2 ---
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 20)
        self.pub_odom_raw = self.create_publisher(Odometry, '/odom_raw', 20)
        self.pub_lwheel = self.create_publisher(Int64, '/lwheel', 10)
        self.pub_rwheel = self.create_publisher(Int64, '/rwheel', 10)
        self.pub_battery = self.create_publisher(BatteryState, '/hoverboard/battery', 5)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Servicio para reiniciar odometría a (0, 0, 0)
        self.srv_reset_odom = self.create_service(Trigger, '/hoverboard/reset_odom', self.reset_odom_cb)

        # Timer de control y transmisión a 50 Hz (20ms)
        self.timer = self.create_timer(0.02, self.control_loop)

    def _acquire_singleton_lock(self):
        try:
            self.lock_file = open('/tmp/hoverboard_driver.lock', 'w')
            fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (IOError, BlockingIOError):
            self.get_logger().warn(
                "⚠️ YA EXISTE una instancia activa de hoverboard_driver en ejecución. "
                "Cerrando este proceso duplicado para evitar colisión en puerto serial y saltos de TF."
            )
            sys.exit(0)

    def reset_odom_cb(self, request, response):
        with self.lock:
            self.x = 0.0
            self.y = 0.0
            self.th = 0.0
            self.vx = 0.0
            self.vth = 0.0
            self.total_ticks_L = 0
            self.total_ticks_R = 0
            self.prev_wheelL = None
            self.prev_wheelR = None
        self.get_logger().info("🔄 Odometría reiniciada exitosamente a (x=0.0m, y=0.0m, θ=0.0°)")
        response.success = True
        response.message = "Odometría reiniciada a (0, 0, 0)"
        return response

    def connect_serial(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"✅ Conectado exitosamente a Hoverboard en {self.port}")
        except Exception as e:
            self.get_logger().warn(f"⚠️ No se pudo abrir {self.port}: {e}. Reintentando en segundo plano...")
            self.ser = None

    def _rx_worker(self):
        """Hilo de alta frecuencia que drena el buffer serial y decodifica la telemetría."""
        rx_buf = bytearray()

        while self.running:
            if self.ser is None or not self.ser.is_open:
                time.sleep(0.5)
                continue

            try:
                if self.ser.in_waiting > 0:
                    data = self.ser.read(self.ser.in_waiting)
                    rx_buf += data

                    while len(rx_buf) >= self.FEEDBACK_SIZE:
                        # Cabecera 0xABCD (en little-endian: 0xCD, 0xAB)
                        if rx_buf[0] == 0xCD and rx_buf[1] == 0xAB:
                            pkt = rx_buf[:self.FEEDBACK_SIZE]
                            rx_buf = rx_buf[self.FEEDBACK_SIZE:]

                            try:
                                start, c1, c2, spR, spL, wR, wL, bat, temp, cmdLed, chk_rx = struct.unpack('<HhhhhhhhhHH', pkt)
                                calc_chk = (start ^ (c1 & 0xFFFF) ^ (c2 & 0xFFFF) ^ (spR & 0xFFFF) ^ (spL & 0xFFFF) ^
                                            (wR & 0xFFFF) ^ (wL & 0xFFFF) ^ (bat & 0xFFFF) ^ (temp & 0xFFFF) ^ (cmdLed & 0xFFFF)) & 0xFFFF

                                if calc_chk == chk_rx:
                                    now = time.time()
                                    with self.lock:
                                        self.valid_packets += 1
                                        self.last_rx_time = now
                                        self.bat_voltage = bat / 100.0
                                        self.board_temp = temp / 10.0
                                        self.speed_L_rpm = spL
                                        self.speed_R_rpm = spR
                                        self._update_odometry(wL, wR, now)
                            except Exception as e:
                                self.get_logger().debug(f"Error parseando feedback: {e}")
                        else:
                            rx_buf.pop(0)

                time.sleep(0.003)
            except Exception as e:
                time.sleep(0.1)

    def _update_odometry(self, wheelL_cnt, wheelR_cnt, now):
        """Calcula cinemática diferencial integrando ticks de encoders (con unwrap 9000)."""
        if self.prev_wheelL is None or self.prev_wheelR is None:
            self.prev_wheelL = wheelL_cnt
            self.prev_wheelR = wheelR_cnt
            self.last_odom_time = now
            return

        dt = now - self.last_odom_time
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02
        self.last_odom_time = now

        dL = self._delta_ticks(wheelL_cnt, self.prev_wheelL)
        dR = self._delta_ticks(wheelR_cnt, self.prev_wheelR)

        self.prev_wheelL = wheelL_cnt
        self.prev_wheelR = wheelR_cnt

        self.total_ticks_L += dL
        self.total_ticks_R += dR

        dist_L = dL * self.meters_per_tick
        dist_R = dR * self.meters_per_tick

        delta_dist = (dist_R + dist_L) / 2.0
        delta_th = (dist_R - dist_L) / self.track_width

        # Integración Runge-Kutta 2do orden
        self.x += delta_dist * math.cos(self.th + delta_th / 2.0)
        self.y += delta_dist * math.sin(self.th + delta_th / 2.0)
        self.th = math.atan2(math.sin(self.th + delta_th), math.cos(self.th + delta_th))

        self.vx = delta_dist / dt
        self.vth = delta_th / dt

    @staticmethod
    def _delta_ticks(curr, prev):
        d = curr - prev
        if d > 4500:
            d -= 9000
        elif d < -4500:
            d += 9000
        return d

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = time.time()
        linear = msg.linear.x
        angular = msg.angular.z

        # Zona muerta lineal para rotación pura en el lugar:
        # Si la velocidad lineal es ínfima (< 3.5 cm/s), forzar cero absoluto para permitir
        # que el robot dedique todo el torque disponible a alinear la orientación exacta del waypoint.
        if abs(linear) < 0.035:
            linear = 0.0

        raw_speed = int(linear * self.linear_scale)
        raw_steer = int(-angular * self.angular_scale)

        # Compensación de fricción estática (stiction) lineal para salida del reposo:
        # Si se solicita avance o retroceso, asegurar un mínimo de torque para que
        # las ruedas rompan la inercia del reposo por sí solas sin empujón manual.
        if raw_speed != 0:
            if 0 < raw_speed < self.min_speed_raw:
                raw_speed = self.min_speed_raw
            elif -self.min_speed_raw < raw_speed < 0:
                raw_speed = -self.min_speed_raw

        # Compensación de fricción estática (stiction) para giro sobre el eje:
        # Cuando no hay avance lineal y se pide girar, asegurar un torque mínimo
        # para que los motores venzan la fricción del piso y no se tranquen en el arranque.
        if raw_speed == 0 and raw_steer != 0:
            if 0 < raw_steer < self.min_steer_raw:
                raw_steer = self.min_steer_raw
            elif -self.min_steer_raw < raw_steer < 0:
                raw_steer = -self.min_steer_raw

        self.target_speed = max(min(raw_speed, self.max_speed_raw), -self.max_speed_raw)
        self.target_steer = max(min(raw_steer, self.max_steer_raw), -self.max_steer_raw)

    def control_loop(self):
        now_ros = self.get_clock().now()
        now_time = time.time()
        self.loop_count += 1

        # 1. Watchdog de seguridad
        if (now_time - self.last_cmd_time) > self.cmd_vel_timeout:
            self.target_speed = 0
            self.target_steer = 0

        # Rampa suave (anti-chatter / anti-vibración) con desaceleración rápida:
        # Aceleración suave (filtra oscilaciones de 20 Hz de Nav2 para evitar temblores)
        # y frenado rápido (evita que la inercia sobrepase el ángulo objetivo al detenerse).
        speed_diff = self.target_speed - self.sent_speed
        max_speed_step = 8 if abs(self.target_speed) >= abs(self.sent_speed) else 20
        if speed_diff > max_speed_step:
            self.sent_speed += max_speed_step
        elif speed_diff < -max_speed_step:
            self.sent_speed -= max_speed_step
        else:
            self.sent_speed = self.target_speed

        steer_diff = self.target_steer - self.sent_steer
        max_steer_step = 12 if abs(self.target_steer) >= abs(self.sent_steer) else 30
        if steer_diff > max_steer_step:
            self.sent_steer += max_steer_step
        elif steer_diff < -max_steer_step:
            self.sent_steer -= max_steer_step
        else:
            self.sent_steer = self.target_steer

        # 2. Reintento de reconexión si el puerto falló
        if self.ser is None or not self.ser.is_open:
            if self.loop_count % 100 == 0:
                self.connect_serial()
            return

        # 3. Transmisión del paquete de control a la placa (50 Hz)
        chk = (self.START_FRAME ^ (self.sent_steer & 0xFFFF) ^ (self.sent_speed & 0xFFFF)) & 0xFFFF
        cmd_pkt = struct.pack('<HhhH', self.START_FRAME, self.sent_steer, self.sent_speed, chk)
        try:
            self.ser.write(cmd_pkt)
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f"Error TX Serial: {e}")
            self.ser = None
            return

        # 4. Publicar Odometría y TF
        with self.lock:
            x, y, th = self.x, self.y, self.th
            vx, vth = self.vx, self.vth
            bat, temp = self.bat_voltage, self.board_temp
            ticks_L, ticks_R = self.total_ticks_L, self.total_ticks_R
            rx_age = now_time - self.last_rx_time if self.last_rx_time > 0 else 999.0

        stamp = now_ros.to_msg()
        qz = math.sin(th / 2.0)
        qw = math.cos(th / 2.0)

        # Odometry message
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = [
            0.001, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.001, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.03
        ]
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = [
            0.002, 0.0,   0.0, 0.0, 0.0, 0.0,
            0.0,   0.002, 0.0, 0.0, 0.0, 0.0,
            0.0,   0.0,   1e6, 0.0, 0.0, 0.0,
            0.0,   0.0,   0.0, 1e6, 0.0, 0.0,
            0.0,   0.0,   0.0, 0.0, 1e6, 0.0,
            0.0,   0.0,   0.0, 0.0, 0.0, 0.05
        ]
        self.pub_odom.publish(odom)
        self.pub_odom_raw.publish(odom)

        # Publicar TF si está habilitado
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        # Publicar ticks acumulados en /lwheel y /rwheel
        msg_l = Int64()
        msg_l.data = int(ticks_L)
        self.pub_lwheel.publish(msg_l)

        msg_r = Int64()
        msg_r.data = int(ticks_R)
        self.pub_rwheel.publish(msg_r)

        # 5. Log periódico (cada 1s / 50 ciclos)
        if self.loop_count % 50 == 0:
            deg = math.degrees(th)
            status_str = "🟢 CONECTADO" if rx_age < 0.5 else "🔴 SIN FEEDBACK"
            self.get_logger().info(
                f"[{status_str}] Bat: {bat:5.2f}V | Temp: {temp:4.1f}°C | "
                f"Odom: x={x:5.2f}m, y={y:5.2f}m, θ={deg:5.1f}° | "
                f"Vel: {vx:5.2f}m/s | Ticks: [L:{ticks_L}, R:{ticks_R}]"
            )
            # Publicar BatteryState
            bmsg = BatteryState()
            bmsg.header.stamp = stamp
            bmsg.header.frame_id = self.base_frame
            bmsg.voltage = float(bat)
            bmsg.present = True
            bmsg.percentage = float(max(min((bat - 30.0) / 12.0, 1.0), 0.0))
            self.pub_battery.publish(bmsg)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'wheel_radius_m':
                self.wheel_radius = float(param.value)
                self.meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
                self.get_logger().info(f"⚙️ Parámetro wheel_radius_m actualizado a {self.wheel_radius:.4f} m ({self.meters_per_tick*1000:.3f} mm/tick)")
            elif param.name == 'track_width_m':
                self.track_width = float(param.value)
                self.get_logger().info(f"⚙️ Parámetro track_width_m actualizado a {self.track_width:.4f} m")
            elif param.name == 'min_speed_raw':
                self.min_speed_raw = int(param.value)
                self.get_logger().info(f"⚙️ Parámetro min_speed_raw actualizado a {self.min_speed_raw}")
            elif param.name == 'min_steer_raw':
                self.min_steer_raw = int(param.value)
                self.get_logger().info(f"⚙️ Parámetro min_steer_raw actualizado a {self.min_steer_raw}")
            elif param.name == 'linear_scale':
                self.linear_scale = float(param.value)
                self.get_logger().info(f"⚙️ Parámetro linear_scale actualizado a {self.linear_scale}")
            elif param.name == 'angular_scale':
                self.angular_scale = float(param.value)
                self.get_logger().info(f"⚙️ Parámetro angular_scale actualizado a {self.angular_scale}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = HoverboardDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Deteniendo HoverboardDriver...")
    finally:
        node.running = False
        try:
            if node.ser and node.ser.is_open:
                stop_chk = (node.START_FRAME ^ 0 ^ 0) & 0xFFFF
                node.ser.write(struct.pack('<HhhH', node.START_FRAME, 0, 0, stop_chk))
                node.ser.flush()
                node.ser.close()
        except Exception:
            pass
        if hasattr(node, 'lock_file') and node.lock_file:
            try:
                fcntl.flock(node.lock_file, fcntl.LOCK_UN)
                node.lock_file.close()
            except Exception:
                pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
