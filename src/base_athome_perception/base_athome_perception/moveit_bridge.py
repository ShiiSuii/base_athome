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
from moveit_msgs.msg import AllowedCollisionEntry, PlanningScene
from moveit_msgs.srv import (ApplyPlanningScene, GetCartesianPath, GetMotionPlan, GetPlanningScene,
                             GetPositionIK, GetStateValidity)
from rclpy.node import Node
from sensor_msgs.msg import JointState

ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
GROUP = "arm"
# Marco del modelo (raiz del URDF). Usarlo evita depender de TF (move_group no transforma sin
# robot_state_publisher). Las poses de entrada se dan en base_link y se desplazan con base_joint.
FRAME = "base_footprint"
TIP = "tcp_link"


def _robot_state(joints_rad: Sequence[float], torso_m: Optional[float] = None) -> RobotState:
    """Estado inicial explicito: brazo MEDIDO y, si se conoce, la altura MEDIDA del elevador."""
    rs = RobotState()
    names, pos = list(ARM_JOINTS), [float(v) for v in joints_rad]
    if torso_m is not None:
        names.append("torso_lift_joint")
        pos.append(float(torso_m))
    rs.joint_state = JointState(name=names, position=pos)
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


def base_link_offset_in_model_frame() -> List[float]:
    """Traslacion base_footprint -> base_link leida del URDF (base_joint). Los objetivos se calculan
    en base_link y move_group trabaja en el marco del modelo (base_footprint) sin TF."""
    try:
        import xml.etree.ElementTree as ET
        from ament_index_python.packages import get_package_share_directory
        urdf = get_package_share_directory("hoverboard_base") + "/urdf/robot.urdf"
        for j in ET.parse(urdf).getroot().findall("joint"):
            if j.get("name") == "base_joint" and j.find("origin") is not None:
                return [float(v) for v in j.find("origin").get("xyz", "0 0 0").split()]
    except Exception:
        pass
    return [0.0, 0.0, 0.0]


class MoveItBridge:
    def __init__(self, node: Node, timeout_s: float = 2.0, torso_provider=None):
        self.node = node
        self.torso_provider = torso_provider or (lambda: None)  # -> torso_lift_joint medido o None
        self.timeout_s = timeout_s
        self.offset = base_link_offset_in_model_frame()
        self.cli_ik = node.create_client(GetPositionIK, "/compute_ik")
        self.cli_plan = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.cli_cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.cli_valid = node.create_client(GetStateValidity, "/check_state_validity")
        self.cli_get_scene = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.cli_apply_scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.allowed_pairs = set()  # pares liberados por solaparse ya en el estado medido

    def _to_model(self, pose: Pose) -> Pose:
        """Pose en base_link -> marco del modelo (base_footprint)."""
        p = Pose()
        p.orientation = pose.orientation
        p.position.x = pose.position.x + self.offset[0]
        p.position.y = pose.position.y + self.offset[1]
        p.position.z = pose.position.z + self.offset[2]
        return p

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

    def ensure_start_valid(self, start_rad: Sequence[float]) -> List[Tuple[str, str]]:
        """Si el estado MEDIDO del brazo ya "choca" en el modelo (geometria del URDF imperfecta,
        p.ej. link4/link5 contra torso_column mientras el brazo real no toca nada), MoveIt rechaza
        todo plan. Se permiten SOLO esos pares (el resto de las colisiones se sigue chequeando).
        Devuelve los pares liberados en esta llamada."""
        req = GetStateValidity.Request()
        req.robot_state = _robot_state(start_rad, self.torso_provider())
        req.group_name = GROUP
        res = self._call(self.cli_valid, req)
        if res.valid:
            return []
        pairs = {tuple(sorted((c.contact_body_1, c.contact_body_2))) for c in res.contacts}
        new = pairs - self.allowed_pairs
        if not new:
            return []
        scene_req = GetPlanningScene.Request()
        scene_req.components.components = scene_req.components.ALLOWED_COLLISION_MATRIX
        acm = self._call(self.cli_get_scene, scene_req).scene.allowed_collision_matrix
        for a, b in new:
            for name in (a, b):
                if name not in acm.entry_names:
                    acm.entry_names.append(name)
                    for e in acm.entry_values:
                        e.enabled.append(False)
                    acm.entry_values.append(AllowedCollisionEntry(enabled=[False] * len(acm.entry_names)))
            ia, ib = acm.entry_names.index(a), acm.entry_names.index(b)
            acm.entry_values[ia].enabled[ib] = True
            acm.entry_values[ib].enabled[ia] = True
        diff = PlanningScene()
        diff.is_diff = True
        diff.allowed_collision_matrix = acm
        self._call(self.cli_apply_scene, ApplyPlanningScene.Request(scene=diff))
        self.allowed_pairs |= new
        self.node.get_logger().warn(f"MoveIt: el estado medido solapa en el modelo {sorted(new)}; "
                                    "se ignoran SOLO esos pares (revisar geometria del URDF)")
        return sorted(new)

    def compute_ik(self, pose: Pose, seed_rad: Sequence[float], avoid_collisions: bool = True) -> Optional[List[float]]:
        try:
            self.ensure_start_valid(seed_rad)
        except Exception as e:
            self.node.get_logger().warn(f'MoveIt: no pude verificar el estado inicial: {e}')
        req = GetPositionIK.Request()
        ik = PositionIKRequest()
        ik.group_name = GROUP
        ik.robot_state = _robot_state(seed_rad, self.torso_provider())
        ik.avoid_collisions = avoid_collisions
        ik.ik_link_name = TIP
        ik.pose_stamped = PoseStamped(pose=self._to_model(pose))
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
        try:
            self.ensure_start_valid(start_rad)
        except Exception as e:
            self.node.get_logger().warn(f'MoveIt: no pude verificar el estado inicial: {e}')
        req = GetMotionPlan.Request()
        m = MotionPlanRequest()
        m.group_name = GROUP
        m.start_state = _robot_state(start_rad, self.torso_provider())
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
                       min_fraction: float = 0.95, vel_scale: float = 0.15) -> Optional[List[Tuple[List[float], float]]]:
        try:
            self.ensure_start_valid(start_rad)
        except Exception as e:
            self.node.get_logger().warn(f'MoveIt: no pude verificar el estado inicial: {e}')
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state = _robot_state(start_rad, self.torso_provider())
        req.group_name = GROUP
        req.link_name = TIP
        req.waypoints = [self._to_model(w) for w in waypoints]
        req.max_step = eef_step
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = vel_scale
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
