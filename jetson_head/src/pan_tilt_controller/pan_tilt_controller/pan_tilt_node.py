#!/usr/bin/env python3
import os
import math
import time
import yaml
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


# Dynamixel Protocol 2.0 Control Table (X-Series: XL430, XM430, XC430)
ADDR_OPERATING_MODE_P2 = 11
ADDR_HOMING_OFFSET_P2 = 20
ADDR_TORQUE_ENABLE_P2 = 64
ADDR_HW_ERROR_P2 = 70
ADDR_PROFILE_ACCEL_P2 = 108
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

        self.port_handler = None
        self.packet_handler = PacketHandler(self.protocol_version)
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
            if self.protocol_version >= 2.0:
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
                self.enable_torque()
                self.is_connected = True
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
        except Exception as e:
            self.get_logger().error(f'Error enabling torque: {e}')

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

        if not self.is_connected or self.port_handler is None:
            return

        pan_tick = int(self.pan_center + self.target_pan_rad * self.ticks_per_rad)
        tilt_tick = int(self.tilt_center + self.target_tilt_rad * self.ticks_per_rad)

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
                    self.current_pan_rad = (p_pos - self.pan_center) / self.ticks_per_rad
                    self.current_tilt_rad = (t_pos - self.tilt_center) / self.ticks_per_rad
                else:
                    self.is_connected = False
                    self.get_logger().warn('Lost communication with servos. Retrying...', throttle_duration_sec=5.0)
            except Exception as e:
                self.is_connected = False
                self.get_logger().warn(f'Exception reading servos: {e}', throttle_duration_sec=5.0)

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
