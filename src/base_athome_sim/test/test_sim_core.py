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


def test_microphone_only_hears_inside_window_and_not_muted():
    m = SimMicrophone(window_s=6, latency_s=1.0)
    assert not m.hear('hello', now=0.0)          # sin ventana: se pierde
    m.trigger(now=10.0)
    m.muted = True
    assert not m.hear('hello', now=11.0)         # robot hablando
    m.muted = False
    assert m.hear('my name is Robin', now=12.0)
    assert m.due(now=16.5) == []                 # se transcribe al cerrar la ventana + latencia
    assert m.due(now=17.1) == ['my name is Robin']


def test_gz_urdf_has_plugins_and_bridges():
    from base_athome_sim import gz_model
    urdf = '<robot name="r"><link name="base_link"/></robot>'
    out = gz_model.gazebo_urdf(urdf)
    assert out.endswith("</robot>")
    assert "VelocityControl" in out and "gpu_lidar" in out and "rgbd_camera" in out
    assert out.count("JointPositionController") == len(gz_model.POSITION_JOINTS)
    assert any(a.startswith("/sim/gz/joint1/cmd_pos@") for a in gz_model.bridge_args())
