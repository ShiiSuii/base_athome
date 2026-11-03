import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import quest_cartesian_core as qc  # noqa: E402

PRE = [72.0, 184.7, -78.3, 75.4, -74.8, -80.4]
Q_ID = (0.0, 0.0, 0.0, 1.0)


def test_fk_matches_repo_kinematics():
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "src", "base_athome_perception"))
    pk = pytest.importorskip("base_athome_perception.piper_kinematics")
    kin = pk.PiperKinematicsSolver()
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = np.radians([rng.uniform(lo, hi) for lo, hi in qc.JOINT_LIMITS_DEG])
        p1, R1 = qc.fk(q)
        p2, R2 = kin.forward_kinematics(q)
        assert np.allclose(p1, p2, atol=1e-6) and np.allclose(R1, R2, atol=1e-9)


def test_clutch_and_relative_translation_forward():
    st = qc.CartesianState()
    meas = list(PRE)
    st = qc.cart_step(st, ((0, 1.0, 0), Q_ID), [0, 1, 0, 0, 0, 0], meas, 0.0, 0.05)
    p0 = st.t0_p.copy()
    # el control se mueve 5 cm hacia ADELANTE (-z en WebXR): la pinza debe avanzar +x en el robot
    for k in range(40):
        st = qc.cart_step(st, ((0, 1.0, -0.05), Q_ID), [0, 1, 0, 0, 0, 0], st.q_cmd, 0.0, 0.05)
    p, _ = qc.fk(np.radians(st.q_cmd))
    d = p - p0
    assert d[0] > 45.0 and abs(d[1]) < 5.0 and abs(d[2]) < 5.0     # ~+50 mm en x
    assert st.err_mm < 5.0


def test_up_and_left_mapping():
    st = qc.CartesianState()
    st = qc.cart_step(st, ((0, 1.0, 0), Q_ID), [0, 1, 0, 0, 0, 0], PRE, 0.0, 0.05)
    p0 = st.t0_p.copy()
    for _ in range(40):
        st = qc.cart_step(st, ((-0.03, 1.03, 0), Q_ID), [0, 1, 0, 0, 0, 0], st.q_cmd, 0.0, 0.05)  # izq + arriba
    d = qc.fk(np.radians(st.q_cmd))[0] - p0
    assert d[1] > 25.0 and d[2] > 25.0


def test_released_grip_sends_nothing_and_speed_is_limited():
    st = qc.cart_step(qc.CartesianState(), ((0, 1, 0), Q_ID), [0, 0, 0, 0, 0, 0], PRE, 0.0, 0.05)
    assert st.q_cmd is None and not st.engaged
    st = qc.cart_step(qc.CartesianState(), ((0, 1, 0), Q_ID), [0, 1, 0, 0, 0, 0], PRE, 0.0, 0.05)
    st = qc.cart_step(st, ((0, 1.0, -0.5), Q_ID), [0, 1, 0, 0, 0, 0], PRE, 0.0, 0.05)   # salto de 50 cm
    assert max(abs(a - b) for a, b in zip(st.q_cmd, PRE)) <= 45.0 * 0.05 + 1e-6


def test_timeout_and_no_feedback():
    assert qc.cart_step(qc.CartesianState(), ((0, 1, 0), Q_ID), [0, 1], PRE, 1.0, 0.05).q_cmd is None
    assert qc.cart_step(qc.CartesianState(), ((0, 1, 0), Q_ID), [0, 1], None, 0.0, 0.05).q_cmd is None


CENTRAL = [80.0, 150.0, -90.0, 0.0, -30.0, 0.0]


def test_rotation_follows_and_x_toggles_position_only():
    st = qc.CartesianState()
    st = qc.cart_step(st, ((0, 1, 0), Q_ID), [0, 1, 0, 0, 0, 0], CENTRAL, 0.0, 0.05)
    R0 = st.t0_R.copy()
    yaw = (0.0, math.sin(math.radians(10) / 2), 0.0, math.cos(math.radians(10) / 2))   # giro 10 grados sobre y (arriba)
    for _ in range(30):
        st = qc.cart_step(st, ((0, 1, 0), yaw), [0, 1, 0, 0, 0, 0], st.q_cmd, 0.0, 0.05)
    R = qc.fk(np.radians(st.q_cmd))[1]
    ang = math.degrees(math.acos((np.trace(R0.T @ R) - 1) / 2))
    assert 7.0 < ang < 13.0
    st = qc.cart_step(st, ((0, 1, 0), yaw), [0, 1, 0, 0, 1, 0], st.q_cmd, 0.0, 0.05)   # X: solo posicion
    assert st.use_rot is False



def test_near_joint_limit_degrades_gracefully():
    """En pre_grasp j5 esta cerca de su limite: el giro horizontal queda parcial, sin salirse de limites."""
    st = qc.cart_step(qc.CartesianState(), ((0, 1, 0), Q_ID), [0, 1, 0, 0, 0, 0], PRE, 0.0, 0.05)
    yaw = (0.0, math.sin(math.radians(15) / 2), 0.0, math.cos(math.radians(15) / 2))
    for _ in range(60):
        st = qc.cart_step(st, ((0, 1, 0), yaw), [0, 1, 0, 0, 0, 0], st.q_cmd, 0.0, 0.05)
    for v, (lo, hi) in zip(st.q_cmd, qc.JOINT_LIMITS_DEG):
        assert lo <= v <= hi
    assert "siguiendo" in st.reason


def test_R_to_quat_roundtrip():
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        R = qc.quat_to_R(q)
        assert np.allclose(qc.quat_to_R(qc.R_to_quat(R)), R, atol=1e-9)


def test_rot_scale_doubles_gripper_rotation():
    q10 = (0.0, math.sin(math.radians(10) / 2), 0.0, math.cos(math.radians(10) / 2))
    res = {}
    for k in (1.0, 2.0):
        st = qc.cart_step(qc.CartesianState(), ((0, 1, 0), Q_ID), [0, 1, 0, 0, 0, 0], CENTRAL, 0.0, 0.05)
        R0 = st.t0_R.copy()
        for _ in range(60):
            st = qc.cart_step(st, ((0, 1, 0), q10), [0, 1, 0, 0, 0, 0], st.q_cmd, 0.0, 0.05, rot_scale=k, max_wrist_dps=90.0)
        R = qc.fk(np.radians(st.q_cmd))[1]
        res[k] = math.degrees(math.acos((np.trace(R0.T @ R) - 1) / 2))
    assert 8.0 < res[1.0] < 12.0 and 17.0 < res[2.0] < 23.0       # 10 grados de mano -> ~20 con k=2


def test_scale_rotation_identity_and_axis():
    R = qc.quat_to_R((0.0, math.sin(0.1), 0.0, math.cos(0.1)))
    R2 = qc.scale_rotation(R, 2.0)
    assert np.allclose(R2, R @ R, atol=1e-9)


def test_trigger_toggles_gripper_open_half():
    st = qc.CartesianState()
    kw = dict(grip_toggle=True, grip_open=90.0, grip_half=45.0)
    seq = [0, 1, 1, 0, 1, 0]          # dos pulsaciones
    out = []
    for t in seq:
        st = qc.cart_step(st, ((0, 1, 0), Q_ID), [t, 1, 0, 0, 0, 0], CENTRAL, 0.0, 0.05, **kw)
        out.append(st.gripper_mm)
    assert out == [90.0, 45.0, 45.0, 45.0, 90.0, 90.0]


def test_hand_tremor_is_filtered():
    rng = np.random.default_rng(5)
    st = qc.cart_step(qc.CartesianState(), ((0, 1, 0), Q_ID), [0, 1, 0, 0, 0, 0], CENTRAL, 0.0, 0.05)
    p0 = st.t0_p.copy()
    for _ in range(60):   # mano "quieta" con temblor de +-1 mm
        jit = rng.uniform(-0.001, 0.001, 3)
        st = qc.cart_step(st, ((jit[0], 1 + jit[1], jit[2]), Q_ID), [0, 1, 0, 0, 0, 0], st.q_cmd, 0.0, 0.05)
    assert np.linalg.norm(qc.fk(np.radians(st.q_cmd))[0] - p0) < 1.5   # la pinza no tiembla
