#!/usr/bin/env python3
"""Cliente minimo de MoveIt 2 (move_group) para el Piper: IK, plan articular y camino cartesiano.

MoveIt solo PLANIFICA (base_athome_moveit_config/move_group.launch.py, sin ros2_control). La
trayectoria resultante se ejecuta enviando puntos a /arm/joint_cmd (piper_arm_controller
interpola cada tramo). Todas las peticiones llevan el estado inicial explicito (la posicion
MEDIDA del brazo), asi no dependen de /joint_states.
"""
import math
from typing import List, Optional, Sequence, Tuple

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest, PositionIKRequest,
                             RobotState, RobotTrajectory)
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
GROUP = "arm"
# Marco del modelo (raiz del URDF). base_footprint == base_link (base_joint identidad); usarlo
# evita depender de TF (move_group no puede transformar si no hay robot_state_publisher).
FRAME = "base_footprint"
TIP = "tcp_link"


def _robot_state(joints_rad: Sequence[float]) -> RobotState:
    rs = RobotState()
    rs.joint_state = JointState(name=list(ARM_JOINTS), position=[float(v) for v in joints_rad])
    rs.is_diff = True
    return rs


def trajectory_points(traj: RobotTrajectory) -> List[Tuple[List[float], float]]:
    """[(posiciones_rad de joint1..6, tiempo_desde_inicio_s)] en el orden ARM_JOINTS."""
    jt = traj.joint_trajectory
    idx = [jt.joint_names.index(j) for j in ARM_JOINTS]
    out = []
    for p in jt.points:
        t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
        out.append(([p.positions[i] for i in idx], t))
    return out


def decimate(points: List[Tuple[List[float], float]], min_dt: float = 0.25) -> List[Tuple[List[float], float]]:
    """Reduce la trayectoria a tramos de al menos min_dt s (el controlador interpola cada tramo)."""
    if not points:
        return []
    out = [points[0]]
    for q, t in points[1:-1]:
        if t - out[-1][1] >= min_dt:
            out.append((q, t))
    if len(points) > 1:
        out.append(points[-1])
    return out


class MoveItBridge:
    def __init__(self, node: Node, timeout_s: float = 2.0):
        self.node = node
        self.timeout_s = timeout_s
        self.cli_ik = node.create_client(GetPositionIK, "/compute_ik")
        self.cli_plan = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cli_cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")

    def available(self) -> bool:
        return self.cli_ik.service_is_ready() and self.cli_plan.service_is_ready()

    def _call(self, cli, req, timeout_s: Optional[float] = None):
        """Llamada bloqueante segura desde otro hilo (el nodo se spinea en su propio hilo)."""
        if not cli.wait_for_service(timeout_sec=self.timeout_s):
            raise RuntimeError(f"servicio {cli.srv_name} no disponible (¿move_group corriendo?)")
        fut = cli.call_async(req)
        import time
        t_end = time.time() + (timeout_s or 10.0)
        while not fut.done() and time.time() < t_end:
            time.sleep(0.01)
        if not fut.done():
            raise RuntimeError(f"timeout en {cli.srv_name}")
        return fut.result()

    def compute_ik(self, pose: Pose, seed_rad: Sequence[float], avoid_collisions: bool = True) -> Optional[List[float]]:
        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = GROUP
        ik.robot_state = _robot_state(seed_rad)
        ik.avoid_collisions = avoid_collisions
        ik.ik_link_name = TIP
        ik.pose_stamped = PoseStamped(pose=pose)
        ik.pose_stamped.header.frame_id = FRAME
        ik.timeout.nanosec = int(0.2e9)
        req.ik_request = ik
        res = self._call(self.cli_ik, req)
        if res.error_code.val != 1:
            return None
        js = res.solution.joint_state
        return [js.position[js.name.index(j)] for j in ARM_JOINTS]

    def plan_to_joints(self, start_rad: Sequence[float], goal_rad: Sequence[float],
                       vel_scale: float = 0.3) -> Optional[List[Tuple[List[float], float]]]:
        req = GetMotionPlan.Request()
        m = MotionPlanRequest()
        m.group_name = GROUP
        m.start_state = _robot_state(start_rad)
        m.num_planning_attempts = 3
        m.allowed_planning_time = 3.0
        m.max_velocity_scaling_factor = vel_scale
        m.max_acceleration_scaling_factor = vel_scale
        c = Constraints()
        for j, v in zip(ARM_JOINTS, goal_rad):
            c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v), tolerance_above=0.01,
                                                       tolerance_below=0.01, weight=1.0))
        m.goal_constraints = [c]
        req.motion_plan_request = m
        res = self._call(self.cli_plan, req, timeout_s=8.0)
        if res.motion_plan_response.error_code.val != 1:
            return None
        return trajectory_points(res.motion_plan_response.trajectory)

    def cartesian_path(self, start_rad: Sequence[float], waypoints: List[Pose], eef_step: float = 0.01,
                       min_fraction: float = 0.95) -> Optional[List[Tuple[List[float], float]]]:
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state = _robot_state(start_rad)
        req.group_name = GROUP
        req.link_name = TIP
        req.waypoints = waypoints
        req.max_step = eef_step
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = 0.15
        req.max_acceleration_scaling_factor = 0.15
        res = self._call(self.cli_cart, req, timeout_s=8.0)
        if res.error_code.val != 1 or res.fraction < min_fraction:
            return None
        return trajectory_points(res.solution)


def pose_from(position: Sequence[float], quat_xyzw: Sequence[float]) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in position)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = (float(v) for v in quat_xyzw)
    return p


def quat_from_matrix(R) -> List[float]:
    """Cuaternion (x, y, z, w) de una matriz de rotacion 3x3."""
    m = R
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return [(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s]
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return [0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s]
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return [(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s]
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return [(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s]
