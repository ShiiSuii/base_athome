#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Piper 6-DOF Robotic Arm - Persistent ROS 2 Controller Node
#
# Hardware: AgileX Piper 6-DOF Manipulator on can1 @ 1 Mbps (Jetson Orin Nano)
# Author: Antigravity AI & Team
# ==============================================================================

import json
import math
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import Float32, String, Bool
from std_srvs.srv import Trigger, SetBool

from piper_sdk import C_PiperInterface_V2

# Constantes de conversion
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
DEG0001_TO_RAD = (1.0 / 1000.0) * DEG_TO_RAD
RAD_TO_DEG0001 = RAD_TO_DEG * 1000.0

MM0001_TO_M = 1.0 / 1e6
M_TO_MM0001 = 1e6
MM_TO_MM0001 = 1000.0
MM0001_TO_MM = 1.0 / 1000.0

# Limites de Pinza Piper (0 a 105 mm - apertura fisica maxima real)
GRIPPER_MAX_MM = 105.0
GRIPPER_MIN_MM = 0.0

# Pose Home/Stow y Pre-Grasp calibradas (grados)
DEFAULT_HOME_DEG = [86.28, 173.91, -164.0, 66.98, 0.69, -146.41]
DEFAULT_READY_DEG = [30.96, 182.68, -98.68, 37.30, -75.12, -95.86]

class PiperArmController(Node):
    """Nodo ROS 2 para control continuo y cinematica del brazo Piper."""

    def __init__(self):
        super().__init__('piper_arm_controller')

        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('auto_enable', True)

        self.can_port = self.get_parameter('can_port').value
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.auto_enable = bool(self.get_parameter('auto_enable').value)

        self.get_logger().info(f"Iniciando PiperArmController en {self.can_port} a {self.rate_hz} Hz...")

        # Inicializar interfaz CAN Piper SDK V2
        self.piper = C_PiperInterface_V2(can_name=self.can_port)
        self.piper.ConnectPort()

        # Publicadores ROS 2
        # /joint_states para robot_state_publisher y RViz2
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_end_pose = self.create_publisher(PoseStamped, '/arm/current_pose', 10)
        self.pub_status = self.create_publisher(String, '/arm/status', 10)

        # Subscriptores de Control
        self.sub_joint_cmd = self.create_subscription(JointState, '/arm/joint_cmd', self.joint_cmd_cb, 5)
        self.sub_gripper_cmd = self.create_subscription(Float32, '/arm/gripper_cmd', self.gripper_cmd_cb, 5)
        self.sub_enable = self.create_subscription(Bool, '/arm/enable', self.enable_cb, 5)

        # Servicios ROS 2
        self.srv_home = self.create_service(Trigger, '/arm/home', self.srv_home_cb)
        self.srv_pre_grasp = self.create_service(Trigger, '/arm/pre_grasp', self.srv_pre_grasp_cb)
        self.srv_grasp = self.create_service(Trigger, '/arm/grasp_pringles', self.srv_grasp_cb)
        self.srv_release = self.create_service(Trigger, '/arm/release', self.srv_release_cb)
        self.srv_enable = self.create_service(SetBool, '/arm/set_enable', self.srv_enable_cb)

        # Estado interno y memoria de objetivos
        self.current_joints_deg = list(DEFAULT_HOME_DEG)
        self.current_gripper_mm = 69.4
        self.current_xyz_mm = [0.0, 0.0, 0.0]
        self.current_rpy_deg = [0.0, 0.0, 0.0]
        self.is_enabled = False

        # Generador de Trayectoria Activa
        self.lock = threading.Lock()
        self.active_trajectory = None
        self.autonomous_sequence = None
        self.tick_counter = 0

        # Inicializacion de Hardware
        self.setup_hardware()

        # Lanzar bucle de control a 50 Hz en hilo dedicado
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()

        self.get_logger().info("PiperArmController completamente activo y listo.")

    def setup_hardware(self):
        """Configura y habilita el brazo Piper en modo CAN."""
        try:
            self.get_logger().info("Configurando hardware Piper...")
            self.piper.MasterSlaveConfig(0xFC, 0, 0, 0)
            time.sleep(0.1)

            if self.auto_enable:
                self.piper.EnableArm(7)
                time.sleep(0.1)
                self.piper.GripperCtrl(0, 1000, 0x03, 0)
                time.sleep(0.1)

            for _ in range(15):
                self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
                time.sleep(0.02)

            self.is_enabled = True
            self.read_sensors()
            self.target_joints_deg = list(self.current_joints_deg)
            self.target_gripper_mm = float(self.current_gripper_mm)
            self.get_logger().info(f"Brazo habilitado. Posicion: {self.current_joints_deg}, Pinza: {self.current_gripper_mm:.1f} mm")
        except Exception as e:
            self.get_logger().error(f"Error inicializando hardware Piper: {e}")

    def read_sensors(self):
        """Lee encoders de articulaciones, pinza y pose cartesiana."""
        try:
            j_msg = self.piper.GetArmJointMsgs().joint_state
            self.current_joints_deg = [
                j_msg.joint_1 / 1000.0,
                j_msg.joint_2 / 1000.0,
                j_msg.joint_3 / 1000.0,
                j_msg.joint_4 / 1000.0,
                j_msg.joint_5 / 1000.0,
                j_msg.joint_6 / 1000.0
            ]

            g_msg = self.piper.GetArmGripperMsgs().gripper_state
            self.current_gripper_mm = g_msg.grippers_angle / 1000.0

            p_msg = self.piper.GetArmEndPoseMsgs().end_pose
            self.current_xyz_mm = [
                p_msg.X_axis / 1000.0,
                p_msg.Y_axis / 1000.0,
                p_msg.Z_axis / 1000.0
            ]
            self.current_rpy_deg = [
                p_msg.RX_axis / 1000.0,
                p_msg.RY_axis / 1000.0,
                p_msg.RZ_axis / 1000.0
            ]
        except Exception:
            pass

    def euler_to_quaternion(self, roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
        """Convierte angulos Euler en grados a cuaternion (x, y, z, w)."""
        r = math.radians(roll_deg) / 2.0
        p = math.radians(pitch_deg) / 2.0
        y = math.radians(yaw_deg) / 2.0
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

    def publish_telemetry(self):
        """Publica /joint_states y /arm/current_pose a ROS 2."""
        now = self.get_clock().now().to_msg()

        # 1. /joint_states para robot_state_publisher y RViz2
        js = JointState()
        js.header.stamp = now
        js.name = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
            'joint7', 'joint8'
        ]

        j_rad = [deg * DEG_TO_RAD for deg in self.current_joints_deg]
        half_open_m = min(0.035, max(0.0, (self.current_gripper_mm / 1000.0) / 2.0))
        js.position = j_rad + [half_open_m, -half_open_m]
        js.velocity = [0.0] * 8
        js.effort = [0.0] * 8
        self.pub_joint_states.publish(js)

        # 2. /arm/current_pose
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "base_link"
        ps.pose.position.x = self.current_xyz_mm[0] / 1000.0
        ps.pose.position.y = self.current_xyz_mm[1] / 1000.0
        ps.pose.position.z = self.current_xyz_mm[2] / 1000.0

        qx, qy, qz, qw = self.euler_to_quaternion(*self.current_rpy_deg)
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pub_end_pose.publish(ps)

        # 3. Telemetria JSON
        stat_msg = String()
        stat_msg.data = json.dumps({
            "is_moving": self.active_trajectory is not None,
            "gripper_mm": round(self.current_gripper_mm, 2),
            "xyz_mm": [round(v, 1) for v in self.current_xyz_mm],
            "joints_deg": [round(v, 2) for v in self.current_joints_deg]
        })
        self.pub_status.publish(stat_msg)

    def control_loop(self):
        """Bucle principal a 50 Hz. Satisface watchdog CAN y ejecuta trayectorias suaves."""
        period = 1.0 / self.rate_hz
        while self.running and rclpy.ok():
            t_start = time.time()
            self.tick_counter += 1

            self.read_sensors()
            self.publish_telemetry()

            with self.lock:
                if self.autonomous_sequence is not None:
                    self.update_autonomous_sequence(t_start)

                is_moving = self.active_trajectory is not None
                if is_moving:
                    traj = self.active_trajectory
                    elapsed = t_start - traj['start_time']
                    duration = max(0.1, traj['duration'])

                    if elapsed >= duration:
                        setpoint_deg = list(traj['target_deg'])
                        setpoint_g = traj['target_gripper']
                        self.active_trajectory = None
                    else:
                        alpha = (1.0 - math.cos(math.pi * elapsed / duration)) / 2.0
                        setpoint_deg = [
                            traj['start_deg'][k] + alpha * (traj['target_deg'][k] - traj['start_deg'][k])
                            for k in range(6)
                        ]
                        setpoint_g = traj['start_gripper'] + alpha * (traj['target_gripper'] - traj['start_gripper'])

                    self.target_joints_deg = setpoint_deg
                    self.target_gripper_mm = setpoint_g

                if self.is_enabled:
                    try:
                        # Si está en movimiento: transmitir a 50 Hz cada ciclo
                        # Si está en reposo (hold): transmitir MotionCtrl cada 2 ciclos (25Hz) y JointCtrl cada 5 ciclos (10Hz)
                        if is_moving or (self.tick_counter % 2 == 0):
                            spd = 35 if is_moving else 10
                            self.piper.MotionCtrl_2(0x01, 0x01, spd, 0x00)

                        if is_moving or (self.tick_counter % 5 == 0):
                            j_int = [int(deg * 1000.0) for deg in self.target_joints_deg]
                            self.piper.JointCtrl(*j_int)
                            g_val = int(min(GRIPPER_MAX_MM, max(GRIPPER_MIN_MM, self.target_gripper_mm)) * 1000.0)
                            self.piper.GripperCtrl(g_val, 1500, 0x01, 0)
                    except Exception:
                        pass

            t_spent = time.time() - t_start
            t_sleep = max(0.001, period - t_spent)
            time.sleep(t_sleep)

    def plan_trajectory(self, target_deg: List[float], duration: float = 2.0, target_gripper: Optional[float] = None):
        """Inicia una trayectoria suave interpolada hacia target_deg."""
        with self.lock:
            g_target = self.target_gripper_mm if target_gripper is None else target_gripper
            self.active_trajectory = {
                'start_deg': list(self.current_joints_deg),
                'target_deg': list(target_deg),
                'start_gripper': float(self.current_gripper_mm),
                'target_gripper': float(g_target),
                'start_time': time.time(),
                'duration': float(duration)
            }

    def plan_gripper(self, target_gripper_mm: float, duration: float = 1.2):
        """Mueve la pinza suavemente sin sobrescribir ni cancelar trayectorias articulares en curso."""
        with self.lock:
            if self.active_trajectory is not None:
                self.active_trajectory['target_gripper'] = float(target_gripper_mm)
                return
            self.active_trajectory = {
                'start_deg': list(self.current_joints_deg),
                'target_deg': list(self.target_joints_deg),
                'start_gripper': float(self.current_gripper_mm),
                'target_gripper': float(target_gripper_mm),
                'start_time': time.time(),
                'duration': float(duration)
            }

    # --- Callbacks de Comandos ROS 2 ---
    def joint_cmd_cb(self, msg: JointState):
        if len(msg.position) >= 6:
            is_rad = all(abs(v) <= 3.5 for v in msg.position[:6])
            targets_deg = [v * RAD_TO_DEG if is_rad else v for v in msg.position[:6]]

            g_target = None
            if len(msg.position) >= 7:
                g_val = msg.position[6]
                g_target = g_val * 1000.0 if g_val < 1.0 else g_val

            duration = 2.5
            if len(msg.velocity) > 0 and msg.velocity[0] > 0:
                duration = max(0.8, min(6.0, msg.velocity[0]))

            self.get_logger().info(f"Recibido joint_cmd: {[round(v, 2) for v in targets_deg]}, duracion: {duration}s")
            self.plan_trajectory(targets_deg, duration=duration, target_gripper=g_target)

    def gripper_cmd_cb(self, msg: Float32):
        val_mm = msg.data
        self.get_logger().info(f"Recibido gripper_cmd: {val_mm:.1f} mm")
        self.plan_gripper(val_mm)

    def enable_cb(self, msg: Bool):
        self.set_arm_enable(msg.data)

    def set_arm_enable(self, enable: bool):
        with self.lock:
            self.is_enabled = enable
            if enable:
                self.read_sensors()
                self.target_joints_deg = list(self.current_joints_deg)
                self.target_gripper_mm = float(self.current_gripper_mm)
                self.active_trajectory = None
                for _ in range(5):
                    self.piper.EnableArm(7)
                    time.sleep(0.02)
                self.piper.GripperCtrl(0, 1000, 0x03, 0)
                for _ in range(10):
                    self.piper.MotionCtrl_2(0x01, 0x01, 35, 0x00)
                    time.sleep(0.01)
                self.get_logger().info("Brazo HABILITADO en modo CAN y setpoint sincronizado en posicion actual")
            else:
                self.active_trajectory = None
                self.autonomous_sequence = None
                self.piper.DisableArm(7)
                self.get_logger().info("Brazo DESHABILITADO (modo manual libre)")

    # --- Secuencia Autonoma de Agarre de Pringles ---
    def start_grasp_sequence(self):
        """Inicia la maquina de estados de agarre de Pringles."""
        with self.lock:
            self.autonomous_sequence = {
                'step': 'OPEN_GRIPPER',
                'step_start_time': time.time(),
                'status': 'RUNNING'
            }
            self.plan_gripper(70.0, duration=1.2)
        self.get_logger().info("[Autonomo] Paso 1: Abriendo pinza a 70 mm...")

    def update_autonomous_sequence(self, now: float):
        seq = self.autonomous_sequence
        if seq is None or seq['status'] != 'RUNNING':
            return

        if self.active_trajectory is not None:
            return

        step = seq['step']

        if step == 'OPEN_GRIPPER':
            seq['step'] = 'APPROACH'
            seq['step_start_time'] = now
            approach_pose = [89.58, 172.74, -163.98, 84.59, -1.27, -153.97]
            self.plan_trajectory(approach_pose, duration=2.0, target_gripper=70.0)
            self.get_logger().info("[Autonomo] Paso 2: Moviendo a pose de aproximacion...")

        elif step == 'APPROACH':
            seq['step'] = 'INGRESS'
            seq['step_start_time'] = now
            # Adelantar codo y hombro para rodear el pote
            ingress_pose = [89.58, 169.5, -159.0, 82.0, -1.27, -153.97]
            self.plan_trajectory(ingress_pose, duration=1.8, target_gripper=70.0)
            self.get_logger().info("[Autonomo] Paso 3: Avanzando pinza sobre el pote de Pringles...")

        elif step == 'INGRESS':
            seq['step'] = 'CLAMP'
            seq['step_start_time'] = now
            self.plan_gripper(62.0, duration=1.5)
            self.get_logger().info("[Autonomo] Paso 4: Cerrando pinza (agarre seguro con torque)...")

        elif step == 'CLAMP':
            seq['step'] = 'LIFT'
            seq['step_start_time'] = now
            # Elevar brazo
            lift_pose = [89.58, 158.0, -148.0, 78.0, -1.27, -153.97]
            self.plan_trajectory(lift_pose, duration=2.0, target_gripper=62.0)
            self.get_logger().info("[Autonomo] Paso 5: Elevando el pote de Pringles...")

        elif step == 'LIFT':
            seq['status'] = 'COMPLETED'
            self.get_logger().info("[Autonomo] Agarre y elevacion completados con EXITO!")

    # --- Servicios ---
    def srv_grasp_cb(self, req, resp):
        self.start_grasp_sequence()
        resp.success = True
        resp.message = "Secuencia autonoma de agarre de Pringles iniciada."
        return resp

    def srv_release_cb(self, req, resp):
        self.plan_gripper(70.0, duration=1.2)
        resp.success = True
        resp.message = "Pinza abierta. Objeto liberado."
        return resp

    def srv_home_cb(self, req, resp):
        self.plan_trajectory(DEFAULT_HOME_DEG, duration=2.5, target_gripper=70.0)
        resp.success = True
        resp.message = "Brazo retornando a pose Home."
        return resp

    def srv_pre_grasp_cb(self, req, resp):
        self.plan_trajectory(DEFAULT_READY_DEG, duration=2.0, target_gripper=70.0)
        resp.success = True
        resp.message = "Brazo en pose de pre-agarre."
        return resp

    def srv_enable_cb(self, req, resp):
        self.set_arm_enable(req.data)
        resp.success = True
        resp.message = f"Brazo {'habilitado' if req.data else 'deshabilitado'}."
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = PiperArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.piper.DisconnectPort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
