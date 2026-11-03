"""HOME-45: accion /arm/follow_joint_trajectory del controlador del Piper con un brazo FALSO
(sin CAN). El falso sigue los JointCtrl (o se queda quieto) para probar llegada, tolerancias y cancelacion.
    ROS_DOMAIN_ID=77 python3 -m pytest -q jetson_head/scripts/tests
"""
import math
import os
import threading
import time
import types

import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from test_piper_arm_controller_safety import load_controller, make_fake_piper, make_node  # noqa: F401

control_msgs = pytest.importorskip("control_msgs.action")
FJT = control_msgs.FollowJointTrajectory
START = [10.0, 20.0, -30.0, 40.0, 50.0, 60.0]


@pytest.fixture(scope='module', autouse=True)
def isolated_ros_fjt():
    assert os.environ.get('ROS_DOMAIN_ID') != '10', 'no correr tests en el dominio operativo'
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def tracking_fake(follow=True):
    """Brazo falso con feedback fresco; si follow, la posicion medida = ultimo JointCtrl."""
    fake = make_fake_piper()
    state = {'q': [int(v * 1000) for v in START], 'stamp': 1.0}

    def read():
        state['stamp'] += 1.0
        return types.SimpleNamespace(time_stamp=state['stamp'], joint_state=types.SimpleNamespace(
            **{f'joint_{i + 1}': state['q'][i] for i in range(6)}))
    fake.GetArmJointMsgs.side_effect = read
    if follow:
        fake.JointCtrl.side_effect = lambda *q: state.__setitem__('q', list(q))
    return fake


@pytest.fixture
def rig():
    nodes = []

    def build(follow=True, params=None):
        fake = tracking_fake(follow)
        module = load_controller(fake)
        node = make_node(module, params or {})
        client_node = rclpy.create_node('fjt_test_client')
        ex = MultiThreadedExecutor(num_threads=4)
        ex.add_node(node)
        ex.add_node(client_node)
        th = threading.Thread(target=ex.spin, daemon=True)
        th.start()
        cli = ActionClient(client_node, FJT, '/arm/follow_joint_trajectory')
        assert cli.wait_for_server(timeout_sec=5.0)
        nodes.append((node, client_node, ex))
        time.sleep(0.1)
        return node, cli, fake
    yield build
    for node, client_node, ex in nodes:
        node.running = False
        ex.shutdown()
        node.destroy_node()
        client_node.destroy_node()


def goal_to(q_deg, total_s=0.4):
    from trajectory_msgs.msg import JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    g = FJT.Goal()
    g.trajectory.joint_names = [f'joint{i}' for i in range(1, 7)]
    for frac in (0.5, 1.0):
        p = JointTrajectoryPoint()
        p.positions = [math.radians(s + frac * (t - s)) for s, t in zip(START, q_deg)]
        t = frac * total_s
        p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
        g.trajectory.points.append(p)
    return g


def send(cli, goal, timeout=8.0):
    fut = cli.send_goal_async(goal)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        time.sleep(0.02)
    gh = fut.result()
    if not gh.accepted:
        return gh, None
    rf = gh.get_result_async()
    while not rf.done() and time.time() - t0 < timeout:
        time.sleep(0.02)
    return gh, rf.result()


def test_goal_rejected_while_disabled(rig):
    node, cli, fake = rig()
    gh, res = send(cli, goal_to([v + 5 for v in START]))
    assert not gh.accepted and res is None
    assert fake.JointCtrl.call_count == 0


def test_success_verified_by_measured_feedback(rig):
    node, cli, _ = rig(follow=True)
    assert node.set_arm_enable(True)
    target = [v + 5 for v in START]
    gh, res = send(cli, goal_to(target))
    assert res.result.error_code == FJT.Result.SUCCESSFUL, res.result.error_string
    assert "verificada por feedback" in res.result.error_string
    assert node.current_joints_deg == pytest.approx(target, abs=2.0)  # fjt_goal_tolerance_deg


def test_arm_that_does_not_move_fails_goal_tolerance(rig):
    node, cli, _ = rig(follow=False, params={'fjt_goal_time_tolerance_s': 0.3})
    assert node.set_arm_enable(True)
    gh, res = send(cli, goal_to([v + 5 for v in START]))
    assert res.result.error_code == FJT.Result.GOAL_TOLERANCE_VIOLATED


def test_large_tracking_error_aborts_and_holds(rig):
    node, cli, _ = rig(follow=False)
    assert node.set_arm_enable(True)
    target = list(START)
    target[0] += 40.0
    gh, res = send(cli, goal_to(target, total_s=1.0))
    assert res.result.error_code == FJT.Result.PATH_TOLERANCE_VIOLATED
    assert node.timed_traj is None
    assert node.target_joints_deg == pytest.approx(node.current_joints_deg, abs=1e-6)


def test_out_of_limits_goal_is_invalid(rig):
    node, cli, fake = rig()
    assert node.set_arm_enable(True)
    target = list(START)
    target[1] = 200.0
    gh, res = send(cli, goal_to(target))
    assert res.result.error_code == FJT.Result.INVALID_GOAL
    assert node.timed_traj is None


def test_cancel_stops_and_holds_measured(rig):
    node, cli, _ = rig(follow=False, params={'fjt_path_tolerance_deg': 90.0})
    assert node.set_arm_enable(True)
    target = list(START)
    target[0] += 20.0
    fut = cli.send_goal_async(goal_to(target, total_s=5.0))
    while not fut.done():
        time.sleep(0.02)
    gh = fut.result()
    assert gh.accepted
    time.sleep(0.3)
    cf = gh.cancel_goal_async()
    while not cf.done():
        time.sleep(0.02)
    rf = gh.get_result_async()
    t0 = time.time()
    while not rf.done() and time.time() - t0 < 5:
        time.sleep(0.02)
    assert rf.result().status == 5  # CANCELED
    assert node.timed_traj is None
    assert node.target_joints_deg == pytest.approx(node.current_joints_deg, abs=1e-6)


def test_disabling_mid_trajectory_aborts_goal(rig):
    node, cli, _ = rig(follow=False, params={'fjt_path_tolerance_deg': 90.0})
    assert node.set_arm_enable(True)
    target = list(START)
    target[0] += 20.0
    fut = cli.send_goal_async(goal_to(target, total_s=5.0))
    while not fut.done():
        time.sleep(0.02)
    gh = fut.result()
    time.sleep(0.3)
    node.set_arm_enable(False)
    rf = gh.get_result_async()
    t0 = time.time()
    while not rf.done() and time.time() - t0 < 5:
        time.sleep(0.02)
    assert rf.result().result.error_code == FJT.Result.PATH_TOLERANCE_VIOLATED
    assert "deshabilitado" in rf.result().result.error_string
