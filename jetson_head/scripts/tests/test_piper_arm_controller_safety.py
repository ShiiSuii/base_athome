"""Tests offline de arranque seguro de piper_arm_controller (sin CAN ni brazo).

Ejecutar en un dominio ROS aislado, nunca en el operativo (10):
    ROS_DOMAIN_ID=77 python3 -m pytest -q jetson_head/scripts/tests
"""
import importlib.util
import math
import os
import sys
import time
import types
from unittest import mock

import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

SCRIPT = os.path.join(os.path.dirname(__file__), '..', 'piper_arm_controller.py')
MOTION_CMDS = ('EnableArm', 'MotionCtrl_2', 'JointCtrl', 'GripperCtrl')
# Pose medida falsa DENTRO de los limites (joint3 <= 0): 10, 20, -30, 40, 50, 60 grados (en 0.001 grados)
VALID_FEEDBACK_MDEG = {'joint_1': 10000, 'joint_2': 20000, 'joint_3': -30000,
                       'joint_4': 40000, 'joint_5': 50000, 'joint_6': 60000}


@pytest.fixture(scope='module', autouse=True)
def isolated_ros():
    assert os.environ.get('ROS_DOMAIN_ID') != '10', 'no correr tests en el dominio operativo'
    rclpy.init()
    yield
    rclpy.shutdown()


def make_fake_piper():
    """SDK falso que reporta ceros, como un Piper desenergizado."""
    fake = mock.MagicMock()
    fake.GetArmJointMsgs.return_value.joint_state = types.SimpleNamespace(
        **{f'joint_{i}': 0 for i in range(1, 7)})
    fake.GetArmGripperMsgs.return_value.gripper_state = types.SimpleNamespace(grippers_angle=0)
    fake.GetArmEndPoseMsgs.return_value.end_pose = types.SimpleNamespace(
        X_axis=0, Y_axis=0, Z_axis=0, RX_axis=0, RY_axis=0, RZ_axis=0)
    return fake


def load_controller(fake_piper):
    sdk = types.ModuleType('piper_sdk')
    sdk.C_PiperInterface_V2 = mock.Mock(return_value=fake_piper)
    with mock.patch.dict(sys.modules, {'piper_sdk': sdk}):
        spec = importlib.util.spec_from_file_location('piper_arm_controller', SCRIPT)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    return module


def motion_calls(fake_piper):
    return [c for c in fake_piper.method_calls if c[0] in MOTION_CMDS]


def make_node(module, params=None):
    overrides = [Parameter(k, value=v) for k, v in (params or {}).items()]
    node_init = Node.__init__

    def init_with_overrides(self, name, **kwargs):
        node_init(self, name, parameter_overrides=overrides, **kwargs)

    with mock.patch.object(Node, '__init__', init_with_overrides):
        return module.PiperArmController()


def test_default_does_not_enable_or_command_arm():
    fake = make_fake_piper()
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.3)  # ~15 ciclos del bucle de control a 50 Hz
        assert node.is_enabled is False
        assert motion_calls(fake) == []
    finally:
        node.running = False
        node.destroy_node()


def test_explicit_auto_enable_keeps_previous_behaviour():
    fake = make_fake_piper()
    module = load_controller(fake)
    node = make_node(module, {'auto_enable': True})
    try:
        time.sleep(0.3)
        assert node.is_enabled is True
        names = {c[0] for c in motion_calls(fake)}
        assert {'EnableArm', 'MotionCtrl_2', 'JointCtrl'} <= names
    finally:
        node.running = False
        node.destroy_node()


def with_live_feedback(fake):
    """Simula tramas CAN nuevas: time_stamp creciente en cada lectura."""
    joints = types.SimpleNamespace(**VALID_FEEDBACK_MDEG)
    fake.GetArmJointMsgs.side_effect = lambda: types.SimpleNamespace(time_stamp=time.time(), joint_state=joints)
    return fake


def test_enable_rejected_without_can_feedback():
    fake = make_fake_piper()
    fake.GetArmJointMsgs.return_value.time_stamp = 0  # SDK sin tramas: brazo sin potencia
    module = load_controller(fake)
    node = make_node(module)
    try:
        assert node.set_arm_enable(True) is False
        time.sleep(0.2)
        assert node.is_enabled is False
        assert motion_calls(fake) == []
    finally:
        node.running = False
        node.destroy_node()


def test_enable_rejected_when_feedback_goes_stale():
    fake = make_fake_piper()
    fake.GetArmJointMsgs.return_value.time_stamp = 123.0  # una trama vieja y luego nada
    module = load_controller(fake)
    node = make_node(module, {'feedback_timeout_s': 0.1})
    try:
        time.sleep(0.25)
        assert node.feedback_ok() is False
        assert node.set_arm_enable(True) is False
        assert motion_calls(fake) == []
    finally:
        node.running = False
        node.destroy_node()


def test_enable_with_live_feedback_syncs_setpoint_to_measured_pose():
    fake = with_live_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.1)
        assert node.set_arm_enable(True) is True
        assert node.is_enabled is True
        assert node.target_joints_deg == [10.0, 20.0, -30.0, 40.0, 50.0, 60.0]
    finally:
        node.running = False
        node.destroy_node()


def test_timed_trajectory_is_followed_continuously():
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    fake = with_live_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.1)
        assert node.set_arm_enable(True)
        start = list(node.target_joints_deg)
        goal = [v + 5.0 for v in start]
        msg = JointTrajectory()
        msg.joint_names = [f'joint{i}' for i in range(1, 7)]
        for frac, t in ((0.5, 0.2), (1.0, 0.4)):
            p = JointTrajectoryPoint()
            p.positions = [math.radians(s + frac * (g - s)) for s, g in zip(start, goal)]
            p.time_from_start = Duration(sec=0, nanosec=int(t * 1e9))
            msg.points.append(p)
        fake.JointCtrl.reset_mock()
        node.trajectory_cb(msg)
        time.sleep(0.2)
        mid = node.target_joints_deg[0]
        assert start[0] < mid < goal[0]          # interpolando, sin saltar al final
        time.sleep(0.4)
        assert node.target_joints_deg == pytest.approx(goal, abs=1e-6)
        assert fake.JointCtrl.call_count >= 15   # transmision continua a 50 Hz mientras se mueve
    finally:
        node.running = False
        node.destroy_node()


# ---------------------------------------------------------------------------------------------
# HOME-41: comandos con el brazo deshabilitado, limites articulares y perdida de realimentacion
# ---------------------------------------------------------------------------------------------

def make_traj(start_deg, goal_deg, total_s=0.4):
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    msg = JointTrajectory()
    msg.joint_names = [f'joint{i}' for i in range(1, 7)]
    for frac in (0.5, 1.0):
        p = JointTrajectoryPoint()
        p.positions = [math.radians(s + frac * (g - s)) for s, g in zip(start_deg, goal_deg)]
        t = frac * total_s
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        msg.points.append(p)
    return msg


def make_joint_cmd(q_deg, gripper=None):
    from sensor_msgs.msg import JointState
    js = JointState()
    js.position = [float(v) for v in q_deg] + ([float(gripper)] if gripper is not None else [])
    return js


def with_switchable_feedback(fake):
    """Realimentacion que se puede cortar: alive['on']=False congela el time_stamp."""
    alive = {'on': True, 'stamp': 1.0}
    joints = types.SimpleNamespace(**VALID_FEEDBACK_MDEG)

    def read():
        if alive['on']:
            alive['stamp'] += 1.0
        return types.SimpleNamespace(time_stamp=alive['stamp'], joint_state=joints)
    fake.GetArmJointMsgs.side_effect = read
    return fake, alive


def test_trajectory_received_while_disabled_is_not_replayed_on_enable():
    """Antes: la trayectoria quedaba guardada y al habilitar el brazo saltaba a su punto final."""
    fake = with_live_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.1)
        measured = list(node.current_joints_deg)
        node.trajectory_cb(make_traj(measured, [v + 20.0 for v in measured], total_s=0.1))
        assert node.timed_traj is None
        time.sleep(0.2)
        assert node.set_arm_enable(True)
        time.sleep(0.15)
        assert node.target_joints_deg == pytest.approx(measured, abs=1e-6)
    finally:
        node.running = False
        node.destroy_node()


def test_joint_cmd_while_disabled_is_ignored():
    fake = with_live_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.1)
        node.joint_cmd_cb(make_joint_cmd([15.0, 25.0, -35.0, 45.0, 55.0, 65.0]))
        assert node.active_trajectory is None
        assert motion_calls(fake) == []
    finally:
        node.running = False
        node.destroy_node()


def test_disable_drops_pending_trajectory():
    fake = with_live_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.1)
        assert node.set_arm_enable(True)
        start = list(node.target_joints_deg)
        node.trajectory_cb(make_traj(start, [v + 5.0 for v in start], total_s=2.0))
        assert node.timed_traj is not None
        assert node.set_arm_enable(False)
        assert node.timed_traj is None and node.active_trajectory is None
    finally:
        node.running = False
        node.destroy_node()


def test_trajectory_outside_joint_limits_is_rejected():
    fake = with_live_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.1)
        assert node.set_arm_enable(True)
        start = list(node.target_joints_deg)
        goal = list(start)
        goal[1] = 200.0  # joint2 > 195 grados (limite del URDF)
        node.trajectory_cb(make_traj(start, goal))
        assert node.timed_traj is None
        time.sleep(0.1)
        assert node.target_joints_deg == pytest.approx(start, abs=1e-6)
    finally:
        node.running = False
        node.destroy_node()


def test_joint_cmd_outside_joint_limits_is_rejected():
    fake = with_live_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.1)
        assert node.set_arm_enable(True)
        node.joint_cmd_cb(make_joint_cmd([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]))  # joint3=+30: debe ser <= 0
        assert node.active_trajectory is None
    finally:
        node.running = False
        node.destroy_node()


def test_feedback_loss_while_moving_aborts_trajectory_and_holds_last_measured():
    fake, alive = with_switchable_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module, {'feedback_timeout_s': 0.1, 'reconnect_timeout_s': 5.0})
    try:
        time.sleep(0.1)
        assert node.set_arm_enable(True)
        start = list(node.target_joints_deg)
        node.trajectory_cb(make_traj(start, [v + 10.0 for v in start], total_s=3.0))
        time.sleep(0.1)
        alive['on'] = False
        time.sleep(0.35)
        assert node.timed_traj is None and node.active_trajectory is None
        assert node.target_joints_deg == pytest.approx(node.current_joints_deg, abs=1e-6)
        # mientras no vuelva la realimentacion, no se aceptan movimientos nuevos
        node.trajectory_cb(make_traj(start, [v + 10.0 for v in start], total_s=0.5))
        assert node.timed_traj is None
    finally:
        node.running = False
        node.destroy_node()


def test_current_pose_uses_arm_base_frame_and_only_with_feedback():
    fake = make_fake_piper()
    fake.GetArmJointMsgs.return_value.time_stamp = 0
    module = load_controller(fake)
    node = make_node(module)
    got = []
    node.pub_end_pose.publish = lambda m: got.append(m)
    try:
        time.sleep(0.15)
        assert got == []  # sin realimentacion no se publica una pose falsa (ceros)
        with_live_feedback(fake)
        time.sleep(0.15)
        assert got and got[-1].header.frame_id == 'arm_base_link'
    finally:
        node.running = False
        node.destroy_node()


def test_gripper_joint_state_not_clamped_to_70mm():
    fake = with_live_feedback(make_fake_piper())
    fake.GetArmGripperMsgs.return_value.gripper_state = types.SimpleNamespace(grippers_angle=90000)  # 90 mm
    module = load_controller(fake)
    node = make_node(module)
    got = []
    node.pub_arm_joint_states.publish = lambda m: got.append(m)
    try:
        time.sleep(0.15)
        assert got
        assert got[-1].position[6] == pytest.approx(0.045, abs=1e-6)  # mitad de 90 mm por dedo
    finally:
        node.running = False
        node.destroy_node()


def test_motion_services_refused_while_disabled():
    from std_srvs.srv import Trigger
    fake = with_live_feedback(make_fake_piper())
    module = load_controller(fake)
    node = make_node(module)
    try:
        time.sleep(0.1)
        for cb in (node.srv_home_cb, node.srv_pre_grasp_cb, node.srv_grasp_cb, node.srv_release_cb):
            resp = cb(Trigger.Request(), Trigger.Response())
            assert resp.success is False
        assert node.active_trajectory is None and node.autonomous_sequence is None
        assert motion_calls(fake) == []
    finally:
        node.running = False
        node.destroy_node()
