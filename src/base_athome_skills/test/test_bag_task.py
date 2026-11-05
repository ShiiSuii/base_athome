"""Geometria de senalar y agarrar una bolsa."""
import math

import pytest

from base_athome_skills.mission import bag_task as bt


def test_pointing_needs_the_arm_away_from_the_body():
    shoulder, hip = (0, 0, 1.4), (0, 0, 0.9)
    assert not bt.is_pointing(shoulder, (0.02, 0, 0.9), hip)      # brazo colgando
    assert bt.is_pointing(shoulder, (0.45, 0, 1.1), hip)          # hacia adelante y abajo
    assert not bt.is_pointing(shoulder, (0.01, 0, 1.39), hip)     # sin brazo medible


def test_ray_hits_the_floor_where_it_points():
    x, y = bt.pointed_floor_point((0, 0, 1.4), (0.3, 0.0, 1.1))
    assert x == pytest.approx(1.4) and y == pytest.approx(0.0)


def test_ray_upward_horizontal_or_too_far_gives_nothing():
    assert bt.pointed_floor_point((0, 0, 1.4), (0.3, 0, 1.6)) is None
    assert bt.pointed_floor_point((0, 0, 1.4), (0.3, 0, 1.4)) is None
    assert bt.pointed_floor_point((0, 0, 1.4), (0.6, 0, 1.399)) is None   # casi horizontal: demasiado lejos


def test_choose_the_bag_nearest_to_the_ray_and_reject_far_ones():
    bags = [(2.0, 0.0, 0.3), (1.0, 0.1, 0.3)]
    assert bt.choose_bag((1.1, 0.0), bags) == bags[1]
    assert bt.choose_bag((5.0, 5.0), bags) is None
    assert bt.choose_bag((1.0, 0.0), []) is None


@pytest.mark.parametrize("start", [(0, 0), (-2, 1), (3, -3), (0.5, 4)])
def test_stand_pose_leaves_the_bag_at_the_reach_point(start):
    bag = (1.5, 1.2, 0.3)
    x, y, yaw = bt.stand_pose(bag, start)
    bx, by, _ = bt.object_in_base(bag, (x, y, yaw))
    assert bx == pytest.approx(bt.REACH_X, abs=1e-6) and by == pytest.approx(bt.REACH_Y, abs=1e-6)


def test_driving_straight_from_the_nav_goal_reaches_the_stand_pose():
    bag = (1.5, 1.2, 0.3)
    x0, y0, yaw0 = bt.stand_pose(bag, (0, 0), extra_back_m=bt.DOCK_DISTANCE_M)
    x1, y1, yaw1 = bt.stand_pose(bag, (0, 0))
    assert yaw0 == pytest.approx(yaw1)
    assert x0 + bt.DOCK_DISTANCE_M * math.cos(yaw0) == pytest.approx(x1)
    assert y0 + bt.DOCK_DISTANCE_M * math.sin(yaw0) == pytest.approx(y1)


def test_nav_goal_respects_the_robot_radius():
    bag = (1.5, 1.2, 0.3)
    x, y, _ = bt.stand_pose(bag, (0, 0), extra_back_m=bt.DOCK_DISTANCE_M)
    assert math.hypot(bag[0] - x, bag[1] - y) > 0.44 + 0.2       # robot_radius de nav2_params.yaml


def test_torso_goes_down_only_as_needed_inside_the_stroke():
    assert bt.torso_for_object(bt.REF_OBJECT_Z) == 0.0
    assert bt.torso_for_object(1.20) == 0.0                      # alto: se queda arriba
    assert bt.torso_for_object(bt.REF_OBJECT_Z - 0.30) == pytest.approx(-0.30)
    assert bt.torso_for_object(0.05) == pytest.approx(-0.58)     # piso: tope de carrera con margen


def test_dock_stops_on_distance_or_obstacle():
    assert bt.dock_verdict(0.1, 0.5) == "go"
    assert bt.dock_verdict(0.1, None) == "go"
    assert bt.dock_verdict(0.4, 0.05) == "done"
    assert bt.dock_verdict(0.1, 0.05) == "blocked"


def test_a_big_bag_stays_clear_of_the_chassis():
    assert bt.base_clearance(0.14) == pytest.approx(0.22)
    assert bt.base_clearance(0.30) >= bt.MIN_BASE_CLEARANCE_M


def test_lidar_stops_the_dock_before_the_chassis_touches():
    stop = bt.dock_stop_range()
    assert bt.LASER_X_M + stop == pytest.approx(bt.CHASSIS_FRONT_M + bt.MIN_BASE_CLEARANCE_M)


def test_floor_pose_matches_the_reach_point():
    """La pose y el punto de agarre tienen que decir lo mismo; y es la misma pose que usa la manipulacion."""
    import importlib.util
    from pathlib import Path
    import numpy as np
    root = Path(__file__).resolve().parents[2] / "base_athome_perception" / "base_athome_perception"
    spec = importlib.util.spec_from_file_location("pk", root / "piper_kinematics.py")
    pk = importlib.util.module_from_spec(spec)
    import sys
    sys.modules["pk"] = pk
    spec.loader.exec_module(pk)
    tcp, R = pk.PiperKinematicsSolver().forward_kinematics(np.radians(bt.FLOOR_PREGRASP_DEG))
    point = np.array([0.005, 0.175, 1.065]) + tcp / 1000.0 + 0.12 * R[:, 2]
    assert point == pytest.approx([bt.REACH_X, bt.REACH_Y, bt.REF_OBJECT_Z], abs=0.005)
    assert R[0, 2] > 0.2 and R[2, 2] < -0.7                  # apunta hacia adelante y abajo: lejos de la base
    source = (root / "arm_manipulation.py").read_text()
    assert str(bt.FLOOR_PREGRASP_DEG) in source
