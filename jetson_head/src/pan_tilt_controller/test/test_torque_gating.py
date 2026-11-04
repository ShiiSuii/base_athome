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
    packet.read2ByteTxRx.return_value = (885, 0, 0)    # PWM limit de fabrica
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


def test_connected_flag_published_latched_on_change():
    node, packet = make_node()
    try:
        sent = []
        node.pub_connected.publish = lambda m: sent.append(m.data)
        node.timer_callback()
        node.timer_callback()
        assert sent == [True]
        packet.read4ByteTxRx.return_value = (0, 1, 0)  # deja de responder
        packet.ping.return_value = (0, 1, 0)
        node.timer_callback()
        node.timer_callback()
        assert sent == [True, False]
    finally:
        node.destroy_node()


def test_axis_directions_invert_commands_feedback_and_limits():
    node, packet = make_node({'auto_torque': True})
    try:
        tpr = node.ticks_per_rad
        node.set_target(0.2, 0.1)  # logico: pan izquierda, tilt arriba
        goals = {c[1][1]: c[1][3] for c in packet.method_calls if c[0] == 'write4ByteTxRx' and c[1][2] == 116}
        assert goals[node.pan_id] == int(node.pan_center + 0.2 * tpr)    # pan_direction = +1 (medido 2026-11-04)
        assert goals[node.tilt_id] == int(node.tilt_center - 0.1 * tpr)  # tilt_direction = -1
        # limites fisicos tilt [-0.4878, 0.6150] -> logicos [-0.6150, 0.4878]
        assert node.tilt_min_rad == pytest.approx(-0.6150) and node.tilt_max_rad == pytest.approx(0.4878)
        packet.read4ByteTxRx.return_value = (int(node.pan_center + 0.3 * tpr), 0, 0)
        node.timer_callback()
        assert node.current_pan_rad == pytest.approx(0.3, abs=2e-3)
    finally:
        node.destroy_node()



def test_torque_enable_applies_pwm_limit_and_p_gain():
    """2026-11-03: con torque, los picos de corriente tiraban el U2D2 por USB -> limitar PWM y ganancia P."""
    node, packet = make_node({'pwm_limit': 600, 'position_p_gain': 450})
    try:
        packet.method_calls.clear() if hasattr(packet.method_calls, 'clear') else None
        node.enable_torque()
        w2 = [c[1] for c in packet.method_calls if c[0] == 'write2ByteTxRx']
        assert (node.pan_id, 36, 600) in [a[1:] for a in w2] and (node.tilt_id, 36, 600) in [a[1:] for a in w2]
        assert (node.pan_id, 84, 450) in [a[1:] for a in w2]
        # la EEPROM (PWM limit) se escribe con el torque apagado: antes del torque ON
        idx_pwm = max(i for i, c in enumerate(packet.method_calls) if c[0] == 'write2ByteTxRx' and c[1][2] == 36)
        idx_on = max(i for i, c in enumerate(packet.method_calls) if c[0] == 'write1ByteTxRx' and c[1][2:] == (64, 1))
        assert idx_pwm < idx_on and node.torque_enabled
    finally:
        node.destroy_node()


def test_pwm_limit_not_rewritten_when_equal():
    node, packet = make_node({'pwm_limit': 885})
    try:
        node.enable_torque()
        assert not any(c[0] == 'write2ByteTxRx' and c[1][2] == 36 for c in packet.method_calls)  # sin desgaste de EEPROM
    finally:
        node.destroy_node()


def test_multiturn_offset_is_unwrapped_for_feedback_and_goal():
    """Tilt corrido una vuelta (-381 grados medidos): la medicion vuelve a ~-21 y el objetivo va a la misma vuelta."""
    node, packet = make_node({'auto_torque': True})
    try:
        center = node.tilt_center
        raw = center - 4096 - int(0.366 * node.ticks_per_rad * node.tilt_dir)   # una vuelta abajo + ~-21 grados
        packet.read4ByteTxRx.side_effect = lambda port, sid, addr: ((raw if sid == node.tilt_id else node.pan_center), 0, 0)
        node.timer_callback()
        assert node.tilt_turns == -1
        assert abs(node.current_tilt_rad - (-0.366)) < 0.01
        node.torque_enabled = True
        node.set_target(0.0, 0.0)
        goal = [c[1][3] for c in packet.method_calls if c[0] == 'write4ByteTxRx' and c[1][1] == node.tilt_id and c[1][2] == 116][-1]
        goal = goal - 4294967296 if goal > 2147483647 else goal   # entero con signo enviado como uint32
        assert abs(goal - (center - 4096)) <= 1          # la misma vuelta: ~21 grados de recorrido, no 381
    finally:
        node.destroy_node()
