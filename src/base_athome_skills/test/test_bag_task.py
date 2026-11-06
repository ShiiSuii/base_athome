"""Geometria de senalar y agarrar una bolsa."""
import math

import pytest

from base_athome_skills.mission import bag_task as bt


def test_pointing_needs_the_arm_away_from_the_body():
    shoulder, hip = (0, 0, 1.4), (0, 0, 0.9)
    assert not bt.is_pointing(shoulder, (0.02, 0, 0.9), hip)      # brazo colgando
    assert bt.is_pointing(shoulder, (0.45, 0, 1.1), hip)          # hacia adelante y abajo
    assert not bt.is_pointing(shoulder, (0.01, 0, 1.39), hip)     # sin brazo medible
    assert not bt.is_pointing(shoulder, (0.15, 0, 0.85), hip)     # colgando, algo separado del cuerpo
    assert bt.is_pointing(shoulder, (0.33, 0, 0.9), hip)          # bolsa a menos de 1 m de la persona


def test_ray_hits_the_floor_where_it_points():
    x, y = bt.pointed_floor_point((0, 0, 1.4), (0.3, 0.0, 1.1))
    assert x == pytest.approx(1.4) and y == pytest.approx(0.0)


def test_ray_upward_horizontal_or_too_far_gives_nothing():
    assert bt.pointed_floor_point((0, 0, 1.4), (0.3, 0, 1.6)) is None
    assert bt.pointed_floor_point((0, 0, 1.4), (0.3, 0, 1.4)) is None
    assert bt.pointed_floor_point((0, 0, 1.4), (0.6, 0, 1.399)) is None   # casi horizontal: demasiado lejos


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
    assert bt.base_clearance(0.14) == pytest.approx(0.20)
    assert bt.base_clearance(0.25) >= bt.MIN_BASE_CLEARANCE_M


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
    assert point[:2] == pytest.approx([bt.REACH_X, bt.REACH_Y], abs=0.005)
    assert point[2] + bt.GROUND_TO_BASE_LINK_M == pytest.approx(bt.REF_OBJECT_Z, abs=0.005)
    tcp_base = np.array([0.005, 0.175, 1.065]) + tcp / 1000.0
    grasp = bt.floor_grasp_from_tcp(tcp_base, R[:, 2])
    assert bt.grasp_advance(tcp_base, R[:, 2], 0.30, grasp) < 0.26          # tope real de avance
    assert tcp_base[2] + bt.torso_for_object(0.30, grasp=grasp) > 0.24 + 0.05  # la pinza baja hasta quedar encima
    assert R[0, 2] > 0.2 and R[2, 2] < -0.7                  # apunta hacia adelante y abajo: lejos de la base
    source = (root / "arm_manipulation.py").read_text()
    assert str(bt.FLOOR_PREGRASP_DEG) in source


def test_person_far_means_walk_to_a_distance_where_the_gesture_is_visible():
    x, y, yaw = bt.person_stand_pose((5.0, 0.0), (0.0, 0.0))
    assert x == pytest.approx(5.0 - bt.PERSON_STOP_M) and y == pytest.approx(0.0) and yaw == pytest.approx(0.0)
    assert math.hypot(5.0 - x, y) > 0.44 + 0.5                # no se le va encima


def test_person_close_means_stay():
    assert bt.person_stand_pose((1.5, 0.5), (0.0, 0.0)) is None


def test_grasp_height_takes_the_knot_and_stays_off_the_floor():
    assert bt.grasp_height(0.30) == pytest.approx(0.26)
    assert bt.grasp_height(0.05) == pytest.approx(bt.BAG_GRASP_MIN_Z_M)
