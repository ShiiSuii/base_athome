import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from quest_teleop_core import (ControllerSample, GRIPPER_OPEN_MM, JOINT_LIMITS_DEG, TeleopState, step)  # noqa: E402

HOME = [85.0, 172.0, -160.0, -20.0, -10.0, 1.0]


def s(t, sx=0.0, sy=0.0, trigger=0.0, grip=1.0, a=0.0):
    return ControllerSample(t=t, axes=[0, 0, sx, sy], buttons=[trigger, grip, 0, 0, a, 0])


def test_no_motion_without_deadman():
    st = step(TeleopState(), s(0.0, sx=1.0, grip=0.0), HOME, now=0.01, dt=0.05)
    assert st.target_deg is None and not st.active and "hombre muerto" in st.reason


def test_timeout_freezes():
    st = step(TeleopState(), s(0.0, sx=1.0), HOME, now=1.0, dt=0.05)
    assert st.target_deg is None and "timeout" in st.reason


def test_no_feedback_no_command():
    st = step(TeleopState(), s(0.0, sx=1.0), None, now=0.0, dt=0.05)
    assert st.target_deg is None


def test_speed_limited_jog_from_measured():
    st = step(TeleopState(), s(0.0, sx=1.0), HOME, now=0.0, dt=0.05, max_speed_dps=20.0)
    assert st.active and abs(st.target_deg[0] - (HOME[0] + 1.0)) < 1e-6   # 20 grados/s * 0.05 s
    assert st.target_deg[1:] == HOME[1:]


def test_deadband_and_rest():
    st = step(TeleopState(), s(0.0, sx=0.1), HOME, now=0.0, dt=0.05)
    assert not st.active and st.target_deg == HOME


def test_mode_switch_on_rising_edge():
    st = TeleopState()
    st = step(st, s(0.0, a=1.0), HOME, now=0.0, dt=0.05)
    st = step(st, s(0.05, a=1.0), HOME, now=0.05, dt=0.05)   # mantener no vuelve a cambiar
    assert st.mode == 1
    st = step(st, s(0.10, a=0.0, sx=1.0), HOME, now=0.10, dt=0.05)
    assert st.target_deg[2] > HOME[2] and st.target_deg[0] == HOME[0]


def test_limits_and_lead_clamp():
    st = TeleopState()
    near = list(HOME)
    near[1] = JOINT_LIMITS_DEG[1][1] - 1.2
    for k in range(100):
        st = step(st, s(k * 0.05, sy=-1.0), near, now=k * 0.05, dt=0.05)   # adelante -> +j2
    assert st.target_deg[1] <= JOINT_LIMITS_DEG[1][1] - 1.0 + 1e-9
    st2 = TeleopState()
    for k in range(100):
        st2 = step(st2, s(k * 0.05, sx=1.0), HOME, now=k * 0.05, dt=0.05)  # el brazo "no se mueve"
    assert st2.target_deg[0] <= HOME[0] + 10.0 + 1e-9                     # objetivo no se escapa


def test_trigger_closes_gripper_proportionally():
    st = step(TeleopState(), s(0.0, trigger=0.0, grip=0.0), HOME, now=0.0, dt=0.05)
    assert st.gripper_mm == GRIPPER_OPEN_MM
    st = step(st, s(0.0, trigger=1.0, grip=0.0), HOME, now=0.0, dt=0.05)
    assert st.gripper_mm < 15.0
