#!/usr/bin/env python3
import os
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, DurabilityPolicy

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


# Dynamixel Protocol 2.0 Control Table (X-Series: XL430, XM430, XC430)
ADDR_OPERATING_MODE_P2 = 11
ADDR_HOMING_OFFSET_P2 = 20
ADDR_TORQUE_ENABLE_P2 = 64
ADDR_HW_ERROR_P2 = 70
ADDR_PROFILE_ACCEL_P2 = 108
ADDR_PWM_LIMIT_P2 = 36          # EEPROM (2 bytes), solo con torque apagado; 885 = 100 %
ADDR_POSITION_P_GAIN_P2 = 84    # RAM (2 bytes); fabrica 800
ADDR_PROFILE_VEL_P2 = 112
ADDR_GOAL_POSITION_P2 = 116
ADDR_PRESENT_POSITION_P2 = 132

# Dynamixel Protocol 1.0 Control Table (AX-12A / MX-28)
ADDR_TORQUE_ENABLE_P1 = 24
ADDR_GOAL_POSITION_P1 = 30
ADDR_MOVING_SPEED_P1 = 32
ADDR_PRESENT_POSITION_P1 = 36


class PanTiltNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_node')

        # Default Parameters
        self.declare_parameter('device_name', '/dev/ttyU2D2')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('protocol_version', 2.0)
        self.declare_parameter('pan_id', 1)
        self.declare_parameter('tilt_id', 2)
        self.declare_parameter('pan_min_rad', -1.57)
        self.declare_parameter('pan_max_rad', 1.57)
        self.declare_parameter('tilt_min_rad', -0.4878)
        self.declare_parameter('tilt_max_rad', 0.6150)
        self.declare_parameter('pan_center_tick', 2059)
        self.declare_parameter('tilt_center_tick', 2048)
        self.declare_parameter('ticks_per_rad', 651.8986)
        self.declare_parameter('publish_rate_hz', 20.0)
        # Seguro por defecto: no habilitar torque (ni escribir EEPROM) al detectar
        # los servos. Habilitar con /pan_tilt/set_torque o -p auto_torque:=true.
        self.declare_parameter('auto_torque', False)
        # Limitar la potencia de los servos: picos de corriente al sostener la cabeza hacian caer el U2D2 por USB
        # (2026-11-03: ~6 desconexiones/min con torque, ~0 sin torque). pwm_limit 0..885; position_p_gain 0..16383.
        self.declare_parameter('pwm_limit', 600)
        self.declare_parameter('position_p_gain', 450)
        # Sentido de cada eje: convencion de la interfaz = pan + izquierda, tilt + arriba.
        # Medido 2026-10-30: con el montaje actual ambos servos giraban al reves -> -1.
        self.declare_parameter('pan_direction', -1.0)
        self.declare_parameter('tilt_direction', -1.0)

        self.device_name = self.get_parameter('device_name').value
        self.baudrate = self.get_parameter('baudrate').value
        self.protocol_version = float(self.get_parameter('protocol_version').value)
        self.pan_id = self.get_parameter('pan_id').value
        self.tilt_id = self.get_parameter('tilt_id').value
        self.pan_min_rad = self.get_parameter('pan_min_rad').value
        self.pan_max_rad = self.get_parameter('pan_max_rad').value
        self.tilt_min_rad = self.get_parameter('tilt_min_rad').value
        self.tilt_max_rad = self.get_parameter('tilt_max_rad').value
        self.pan_center = self.get_parameter('pan_center_tick').value
        self.tilt_center = self.get_parameter('tilt_center_tick').value
        self.ticks_per_rad = self.get_parameter('ticks_per_rad').value
        self.torque_requested = bool(self.get_parameter('auto_torque').value)
        self.torque_enabled = False

        # Load Calibration File if present
        calib_file = os.path.expanduser('~/athome_ws/src/pan_tilt_controller/config/pan_tilt_params.yaml')
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    cfg = yaml.safe_load(f)
                    p = cfg.get('pan_tilt_node', {}).get('ros__parameters', {})
                    if 'pan_center_tick' in p:
                        self.pan_center = int(p['pan_center_tick'])
                    if 'tilt_center_tick' in p:
                        self.tilt_center = int(p['tilt_center_tick'])
                    if 'pan_min_rad' in p:
                        self.pan_min_rad = float(p['pan_min_rad'])
                    if 'pan_max_rad' in p:
                        self.pan_max_rad = float(p['pan_max_rad'])
                    if 'tilt_min_rad' in p:
                        self.tilt_min_rad = float(p['tilt_min_rad'])
                    if 'tilt_max_rad' in p:
                        self.tilt_max_rad = float(p['tilt_max_rad'])
                self.get_logger().info(f'Loaded calibration: pan_center={self.pan_center}, tilt_center={self.tilt_center}')
            except Exception as e:
                self.get_logger().warn(f'Could not load calibration file: {e}')

        # Los limites de los parametros son FISICOS (ticks del servo). Pasarlos a la convencion logica.
        self.pan_dir = 1.0 if float(self.get_parameter('pan_direction').value) >= 0 else -1.0
        self.tilt_dir = 1.0 if float(self.get_parameter('tilt_direction').value) >= 0 else -1.0
        self.pan_min_rad, self.pan_max_rad = sorted((self.pan_dir * self.pan_min_rad, self.pan_dir * self.pan_max_rad))
        self.tilt_min_rad, self.tilt_max_rad = sorted((self.tilt_dir * self.tilt_min_rad, self.tilt_dir * self.tilt_max_rad))

        self.port_handler = None
        self.packet_handler = PacketHandler(self.protocol_version)
        self.pan_turns = 0
        self.tilt_turns = 0
        self.is_connected = False
        self.last_retry_time = 0.0

        self.current_pan_rad = 0.0
        self.current_tilt_rad = 0.0
        self.target_pan_rad = 0.0
        self.target_tilt_rad = 0.0

        # ROS 2 Topics
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_legacy_joint_states = self.create_publisher(JointState, '/pan_tilt/joint_states', 10)
        self.sub_cmd = self.create_subscription(JointState, '/pan_tilt/command', self.cmd_callback, 10)
        self.sub_deg = self.create_subscription(Vector3, '/pan_tilt/target_degrees', self.deg_callback, 10)
        self.srv_torque = self.create_service(SetBool, '/pan_tilt/set_torque', self.srv_torque_cb)
        # Los joint_states se publican siempre (TF de la cabeza); con servos sin
        # respuesta son la ultima posicion conocida (0.0 al arrancar), no medidas.
        # Los consumidores deben chequear /pan_tilt/connected (latched).
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_connected = self.create_publisher(Bool, '/pan_tilt/connected', latched)
        self._published_connected = None

        self.init_hardware()

        # Feedback Timer
        timer_period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_hardware(self):
        try:
            port_to_open = self.device_name
            if not os.path.exists(port_to_open):
                for candidate in ['/dev/ttyU2D2', '/dev/ttyUSB0', '/dev/ttyUSB1']:
                    if os.path.exists(candidate):
                        port_to_open = candidate
                        break

            if not os.path.exists(port_to_open):
                self.get_logger().warn(f'Port {self.device_name} not found. Reintentando...', throttle_duration_sec=4.0)
                self.is_connected = False
                return False

            if self.port_handler is not None:
                try:
                    self.port_handler.closePort()
                except Exception:
                    pass

            self.port_handler = PortHandler(port_to_open)
            if not self.port_handler.openPort():
                self.get_logger().warn(f'Failed to open port {port_to_open}', throttle_duration_sec=4.0)
                self.is_connected = False
                return False

            if not self.port_handler.setBaudRate(self.baudrate):
                self.get_logger().warn(f'Failed to set baudrate {self.baudrate}', throttle_duration_sec=4.0)
                self.is_connected = False
                return False

            self.get_logger().info(f'Opened {port_to_open} @ {self.baudrate} baud (Proto {self.protocol_version})')
            self.check_servos()
            return self.is_connected
        except Exception as e:
            self.get_logger().warn(f'Serial exception in init_hardware: {e}', throttle_duration_sec=4.0)
            self.is_connected = False
            return False

    def check_servos(self):
        if self.port_handler is None:
            return
        try:
            # Check for HW Errors on Protocol 2.0 servos and auto-recover
            # (solo si se pidio torque: el reboot es una escritura al servo)
            if self.protocol_version >= 2.0 and self.torque_requested:
                for sid in [self.pan_id, self.tilt_id]:
                    hw, h_res, _ = self.packet_handler.read1ByteTxRx(self.port_handler, sid, ADDR_HW_ERROR_P2)
                    if h_res == COMM_SUCCESS and hw != 0:
                        self.get_logger().warn(f'Servo ID {sid} had HW error ({hw}). Rebooting servo...')
                        self.packet_handler.reboot(self.port_handler, sid)
                        time.sleep(0.6)

            p_model, p_res, _ = self.packet_handler.ping(self.port_handler, self.pan_id)
            t_model, t_res, _ = self.packet_handler.ping(self.port_handler, self.tilt_id)

            if p_res == COMM_SUCCESS and t_res == COMM_SUCCESS:
                self.get_logger().info(f'Servos detected: Pan (ID {self.pan_id}) Model: {p_model}, Tilt (ID {self.tilt_id}) Model: {t_model}')
                self.is_connected = True
                if self.torque_requested:
                    self.enable_torque()
                else:
                    self.get_logger().info('Torque deshabilitado (auto_torque=false). Usar /pan_tilt/set_torque.')
            else:
                self.get_logger().warn('Pan-Tilt servos not responding. Ensure 12V power is turned on.', throttle_duration_sec=5.0)
                self.is_connected = False
        except Exception as e:
            self.get_logger().warn(f'Error pinging servos: {e}', throttle_duration_sec=5.0)
            self.is_connected = False

    def enable_torque(self):
        try:
            if self.protocol_version >= 2.0:
                # Ensure Servo 2 (Tilt) homing offset is set to -2212 for center at 2048
                ho, ho_res, _ = self.packet_handler.read4ByteTxRx(self.port_handler, self.tilt_id, ADDR_HOMING_OFFSET_P2)
                if ho > 2147483647: ho -= 4294967296
                if ho_res == COMM_SUCCESS and ho != -2212:
                    self.packet_handler.write1ByteTxRx(self.port_handler, self.tilt_id, ADDR_TORQUE_ENABLE_P2, 0)
                    val = 4294967296 - 2212
                    self.packet_handler.write4ByteTxRx(self.port_handler, self.tilt_id, ADDR_HOMING_OFFSET_P2, val)
                    self.get_logger().info('Configured Tilt Homing Offset to -2212')

                # Ensure Operating Mode 4 (Extended Position Control)
                op, op_res, _ = self.packet_handler.read1ByteTxRx(self.port_handler, self.tilt_id, ADDR_OPERATING_MODE_P2)
                if op_res == COMM_SUCCESS and op != 4:
                    self.packet_handler.write1ByteTxRx(self.port_handler, self.tilt_id, ADDR_TORQUE_ENABLE_P2, 0)
                    self.packet_handler.write1ByteTxRx(self.port_handler, self.tilt_id, ADDR_OPERATING_MODE_P2, 4)
                    self.get_logger().info('Configured Tilt Operating Mode to 4 (Extended Position)')

                pwm = int(self.get_parameter('pwm_limit').value)
                kp = int(self.get_parameter('position_p_gain').value)
                for sid in (self.pan_id, self.tilt_id):
                    cur, res, _ = self.packet_handler.read2ByteTxRx(self.port_handler, sid, ADDR_PWM_LIMIT_P2)
                    if res == COMM_SUCCESS and cur != pwm and 0 < pwm <= 885:   # EEPROM: escribir solo si cambia
                        self.packet_handler.write1ByteTxRx(self.port_handler, sid, ADDR_TORQUE_ENABLE_P2, 0)
                        self.packet_handler.write2ByteTxRx(self.port_handler, sid, ADDR_PWM_LIMIT_P2, pwm)
                        self.get_logger().info(f'Servo {sid}: PWM limit {cur} -> {pwm}')
                    if 0 < kp <= 16383:
                        self.packet_handler.write2ByteTxRx(self.port_handler, sid, ADDR_POSITION_P_GAIN_P2, kp)
                self.packet_handler.write4ByteTxRx(self.port_handler, self.pan_id, ADDR_PROFILE_ACCEL_P2, 20)
                self.packet_handler.write4ByteTxRx(self.port_handler, self.pan_id, ADDR_PROFILE_VEL_P2, 80)
                self.packet_handler.write4ByteTxRx(self.port_handler, self.tilt_id, ADDR_PROFILE_ACCEL_P2, 20)
                self.packet_handler.write4ByteTxRx(self.port_handler, self.tilt_id, ADDR_PROFILE_VEL_P2, 80)

                self.packet_handler.write1ByteTxRx(self.port_handler, self.pan_id, ADDR_TORQUE_ENABLE_P2, 1)
                self.packet_handler.write1ByteTxRx(self.port_handler, self.tilt_id, ADDR_TORQUE_ENABLE_P2, 1)
            else:
                self.packet_handler.write2ByteTxRx(self.port_handler, self.pan_id, ADDR_MOVING_SPEED_P1, 150)
                self.packet_handler.write2ByteTxRx(self.port_handler, self.tilt_id, ADDR_MOVING_SPEED_P1, 150)
                self.packet_handler.write1ByteTxRx(self.port_handler, self.pan_id, ADDR_TORQUE_ENABLE_P1, 1)
                self.packet_handler.write1ByteTxRx(self.port_handler, self.tilt_id, ADDR_TORQUE_ENABLE_P1, 1)
            self.torque_enabled = True
        except Exception as e:
            self.get_logger().error(f'Error enabling torque: {e}')

    def disable_torque(self):
        addr = ADDR_TORQUE_ENABLE_P2 if self.protocol_version >= 2.0 else ADDR_TORQUE_ENABLE_P1
        for sid in [self.pan_id, self.tilt_id]:
            self.packet_handler.write1ByteTxRx(self.port_handler, sid, addr, 0)
        self.torque_enabled = False

    def srv_torque_cb(self, req, resp):
        self.torque_requested = bool(req.data)
        if not self.is_connected or self.port_handler is None:
            resp.success = not req.data
            resp.message = 'Servos no conectados; se aplicara al reconectar' if req.data else 'Torque deshabilitado'
            return resp
        try:
            if req.data:
                self.enable_torque()
            else:
                self.disable_torque()
            resp.success = self.torque_enabled == bool(req.data)
            resp.message = 'Torque habilitado' if self.torque_enabled else 'Torque deshabilitado'
        except Exception as e:
            resp.success = False
            resp.message = f'Error: {e}'
        return resp

    def _turns(self, rel_ticks: int, current: int, label: str) -> int:
        """Vueltas enteras (4096 ticks) en que quedo corrida la posicion multivuelta respecto del centro."""
        k = int(round(rel_ticks / 4096.0))
        if k != current:
            self.get_logger().warn(f'{label}: posicion corrida {k} vuelta(s) ({rel_ticks} ticks del centro); '
                                   'se corrige la referencia (objetivo y medicion en la misma vuelta)')
        return k

    def deg_callback(self, msg: Vector3):
        pan_rad = math.radians(msg.x)
        tilt_rad = math.radians(msg.y)
        self.set_target(pan_rad, tilt_rad)

    def cmd_callback(self, msg: JointState):
        pan_rad = self.target_pan_rad
        tilt_rad = self.target_tilt_rad
        for i, name in enumerate(msg.name):
            if 'pan' in name.lower() and i < len(msg.position):
                pan_rad = msg.position[i]
            elif 'tilt' in name.lower() and i < len(msg.position):
                tilt_rad = msg.position[i]
        self.set_target(pan_rad, tilt_rad)

    def set_target(self, pan_rad, tilt_rad):
        self.target_pan_rad = max(self.pan_min_rad, min(self.pan_max_rad, pan_rad))
        self.target_tilt_rad = max(self.tilt_min_rad, min(self.tilt_max_rad, tilt_rad))

        if not self.is_connected or self.port_handler is None or not self.torque_enabled:
            return

        # objetivo en la MISMA vuelta en la que esta el servo (modo extendido multivuelta): sin esto, con la
        # posicion corrida 4096 ticks el servo intentaba girar 360 grados contra el tope -> sobrecarga (HW error 32)
        pan_tick = int(self.pan_center + self.pan_turns * 4096 + self.pan_dir * self.target_pan_rad * self.ticks_per_rad)
        tilt_tick = int(self.tilt_center + self.tilt_turns * 4096 + self.tilt_dir * self.target_tilt_rad * self.ticks_per_rad)

        try:
            if self.protocol_version >= 2.0:
                p_goal = pan_tick if pan_tick >= 0 else (4294967296 + pan_tick)
                t_goal = tilt_tick if tilt_tick >= 0 else (4294967296 + tilt_tick)
                self.packet_handler.write4ByteTxRx(self.port_handler, self.pan_id, ADDR_GOAL_POSITION_P2, p_goal)
                self.packet_handler.write4ByteTxRx(self.port_handler, self.tilt_id, ADDR_GOAL_POSITION_P2, t_goal)
            else:
                self.packet_handler.write2ByteTxRx(self.port_handler, self.pan_id, ADDR_GOAL_POSITION_P1, pan_tick)
                self.packet_handler.write2ByteTxRx(self.port_handler, self.tilt_id, ADDR_GOAL_POSITION_P1, tilt_tick)
        except Exception as e:
            self.get_logger().warn(f'Error sending goal: {e}')

    def timer_callback(self):
        now = time.time()
        if not self.is_connected or self.port_handler is None:
            if (now - self.last_retry_time) >= 2.5:
                self.last_retry_time = now
                self.init_hardware()
        else:
            try:
                if self.protocol_version >= 2.0:
                    p_pos, p_res, _ = self.packet_handler.read4ByteTxRx(self.port_handler, self.pan_id, ADDR_PRESENT_POSITION_P2)
                    t_pos, t_res, _ = self.packet_handler.read4ByteTxRx(self.port_handler, self.tilt_id, ADDR_PRESENT_POSITION_P2)
                else:
                    p_pos, p_res, _ = self.packet_handler.read2ByteTxRx(self.port_handler, self.pan_id, ADDR_PRESENT_POSITION_P1)
                    t_pos, t_res, _ = self.packet_handler.read2ByteTxRx(self.port_handler, self.tilt_id, ADDR_PRESENT_POSITION_P1)

                if p_res == COMM_SUCCESS and t_res == COMM_SUCCESS:
                    if p_pos > 2147483647: p_pos -= 4294967296
                    if t_pos > 2147483647: t_pos -= 4294967296
                    self.pan_turns = self._turns(p_pos - self.pan_center, self.pan_turns, 'Pan')
                    self.tilt_turns = self._turns(t_pos - self.tilt_center, self.tilt_turns, 'Tilt')
                    self.current_pan_rad = self.pan_dir * (p_pos - self.pan_center - 4096 * self.pan_turns) / self.ticks_per_rad
                    self.current_tilt_rad = self.tilt_dir * (t_pos - self.tilt_center - 4096 * self.tilt_turns) / self.ticks_per_rad
                else:
                    self.is_connected = False
                    self.torque_enabled = False
                    self.get_logger().warn('Lost communication with servos. Retrying...', throttle_duration_sec=5.0)
            except Exception as e:
                self.is_connected = False
                self.torque_enabled = False
                self.get_logger().warn(f'Exception reading servos: {e}', throttle_duration_sec=5.0)

        if self.is_connected != self._published_connected:
            self._published_connected = self.is_connected
            self.pub_connected.publish(Bool(data=bool(self.is_connected)))

        # Publish joint state continuously
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['pan_joint', 'tilt_joint']
        msg.position = [self.current_pan_rad, self.current_tilt_rad]
        msg.velocity = [0.0, 0.0]
        msg.effort = [0.0, 0.0]
        self.pub_joint_states.publish(msg)
        self.pub_legacy_joint_states.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PanTiltNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
