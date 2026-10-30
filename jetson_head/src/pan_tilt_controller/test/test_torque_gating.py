"""Tests offline: el pan-tilt no escribe a los servos sin habilitacion explicita.

Ejecutar en un dominio ROS aislado (nunca 10):
    ROS_DOMAIN_ID=77 python3 -m pytest -q jetson_head/src/pan_tilt_controller/test/test_torque_gating.py
"""
import importlib
import os
import sys
import types
from unittest import mock

import pytest
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import SetBool

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


@pytest.fixture(scope='module', autouse=True)
def isolated_ros():
    assert os.environ.get('ROS_DOMAIN_ID') != '10', 'no correr tests en el dominio operativo'
    rclpy.init()
    yield
    rclpy.shutdown()


def make_fake_sdk():
    packet = mock.MagicMock()
    packet.ping.return_value = (1060, 0, 0)            # XL430 responde
    packet.read1ByteTxRx.return_value = (0, 0, 0)      # sin HW error
    packet.read4ByteTxRx.return_value = (2048, 0, 0)
    sdk = types.ModuleType('dynamixel_sdk')
    sdk.COMM_SUCCESS = 0
    sdk.PortHandler = mock.Mock(return_value=mock.MagicMock())
    sdk.PacketHandler = mock.Mock(return_value=packet)
    return sdk, packet


def writes(packet):
    return [c for c in packet.method_calls if c[0].startswith('write') or c[0] == 'reboot']


def make_node(params=None):
    sdk, packet = make_fake_sdk()
    overrides = [Parameter(k, value=v) for k, v in (params or {}).items()]
    node_init = Node.__init__

    def init_with_overrides(self, name, **kwargs):
        node_init(self, name, parameter_overrides=overrides, **kwargs)

    with mock.patch.dict(sys.modules, {'dynamixel_sdk': sdk}), \
            mock.patch('os.path.exists', return_value=True), \
            mock.patch('builtins.open', mock.mock_open(read_data='{}')), \
            mock.patch.object(Node, '__init__', init_with_overrides):
        module = importlib.reload(importlib.import_module('pan_tilt_controller.pan_tilt_node'))
        node = module.PanTiltNode()
    return node, packet


def test_default_connects_without_writing_to_servos():
    node, packet = make_node()
    try:
        assert node.is_connected is True
        assert node.torque_enabled is False
        node.set_target(0.5, 0.2)
        assert writes(packet) == []
    finally:
        node.destroy_node()


def test_service_enables_and_disables_torque():
    node, packet = make_node()
    try:
        resp = node.srv_torque_cb(SetBool.Request(data=True), SetBool.Response())
        assert resp.success and node.torque_enabled
        node.set_target(0.5, 0.2)
        assert any(c[0] == 'write4ByteTxRx' and c[1][2] == 116 for c in packet.method_calls)

        resp = node.srv_torque_cb(SetBool.Request(data=False), SetBool.Response())
        assert resp.success and not node.torque_enabled
        assert packet.method_calls[-1][1][2:] == (64, 0)
    finally:
        node.destroy_node()


def test_auto_torque_keeps_previous_behaviour():
    node, packet = make_node({'auto_torque': True})
    try:
        assert node.torque_enabled is True
        assert any(c[0] == 'write1ByteTxRx' and c[1][2:] == (64, 1) for c in packet.method_calls)
    finally:
        node.destroy_node()
