"""Tests offline de arranque seguro de piper_arm_controller (sin CAN ni brazo).

Ejecutar en un dominio ROS aislado, nunca en el operativo (10):
    ROS_DOMAIN_ID=77 python3 -m pytest -q jetson_head/scripts/tests
"""
import importlib.util
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
