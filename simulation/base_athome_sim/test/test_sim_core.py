"""Modelos de la simulacion (sin ROS)."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_sim.sim_core import SimArm, SimBase, SimMicrophone, SimPanTilt  # noqa: E402


def test_arm_follows_joint_cmd_only_when_enabled():
    a = SimArm([0] * 6)
    a.joint_cmd([10] * 6, 1.0, now=0.0)
    a.update(now=1.0)
    assert a.q == [0] * 6  # deshabilitado: no se mueve
    a.set_enable(True)
    a.joint_cmd([10] * 6, 1.0, now=0.0)
    a.update(now=0.5)
    assert 0 < a.q[0] < 10
    a.update(now=1.1)
    assert a.q == pytest.approx([10] * 6) and not a.moving


def test_arm_trajectory_linear():
    a = SimArm([0] * 6)
    a.set_enable(True)
    a.trajectory([1.0, 2.0], [[10] * 6, [20] * 6], now=0.0)
    a.update(now=1.5)
    assert a.q[0] == pytest.approx(15.0)


def test_pan_tilt_speed_limited_and_needs_torque():
    p = SimPanTilt(speed_dps=10)
    p.target = (30, -10)
    p.update(1.0)
    assert p.pan == 0
    p.torque = True
    p.update(1.0)
    assert p.pan == 10 and p.tilt == -10


def test_base_reaches_goal():
    b = SimBase(speed=1.0)
    b.set_goal(1.0, 0.0, 0.5)
    assert not b.update(0.5)
    assert b.update(0.6) and b.x == 1.0 and b.yaw == 0.5


def test_microphone_listens_by_request_with_result_always():
    m = SimMicrophone(start_timeout=8, end_silence=1.0, asr_s=1.0)
    assert not m.hear("lost before request", now=0.0)          # sin pedido: se pierde
    m.request("a", now=0.0)
    m.muted = True
    assert not m.hear("robot talking", now=0.5)               # con mute: se pierde
    m.muted = False
    assert m.hear("My name is Robin", now=2.0)
    assert m.due(now=3.0) == []
    rid, st, text = m.due(now=6.0)[0]
    assert (rid, st, text) == ("a", "ok", "My name is Robin")
    m.request("b", now=10.0)
    assert m.due(now=17.0) == []
    assert m.due(now=18.1) == [("b", "no_speech", "")]           # nadie hablo: resultado explicito
    m.request("c", now=20.0)
    assert m.request("d", now=21.0) == "c"                       # el nuevo reemplaza al anterior

def test_gz_urdf_has_plugins_and_bridges():
    from base_athome_sim import gz_model
    urdf = '<robot name="r"><link name="base_link"/></robot>'
    out = gz_model.gazebo_urdf(urdf)
    assert out.endswith("</robot>")
    assert "VelocityControl" in out and "gpu_lidar" in out and "rgbd_camera" in out
    assert out.count("JointPositionController") == len(gz_model.POSITION_JOINTS)
    assert any(a.startswith("/sim/gz/joint1/cmd_pos@") for a in gz_model.bridge_args())


def test_gripper_stops_at_object_width_when_closing():
    from base_athome_sim.sim_core import SimArm
    arm = SimArm()
    arm.set_enable(True)
    arm.gripper_mm = 100.0
    arm.grip_block_mm = 65.0
    arm.gripper(10.0, now=0.0)
    arm.update(now=5.0)
    assert arm.gripper_mm == 65.0          # el objeto impide cerrar mas
    arm.gripper(100.0, now=5.0)
    arm.update(now=10.0)
    assert arm.gripper_mm == 100.0         # abrir no esta bloqueado
