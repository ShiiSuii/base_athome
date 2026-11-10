"""Contrato Restaurant del Piper, solo fakes; no importa ni inicializa ROS."""

import ast
from copy import deepcopy
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1]))
from base_athome_perception import pose_store
from base_athome_perception.restaurant_arm import (
    RESTAURANT_POSES, RestaurantArm, RestaurantItemDetector, RestaurantMotionGuard,
    item_yolo_class, merge_restaurant_poses, poses_calibrated,
)
from base_athome_perception.restaurant_arm_teach import save_pose

CONFIG = Path(__file__).parents[1] / "config/restaurant_arm_poses.yaml"


def calibrated_poses():
    poses = merge_restaurant_poses({}, CONFIG)
    for pose in poses.values():
        pose["calibrated"] = True
    return poses


class FakeManip:
    def __init__(self, *, fail_at=None, fail_pick=False):
        self.poses = dict(calibrated_poses(), home={"joints_deg": [0]*6})
        self.events = []
        self.abort = False
        self.fail_at = fail_at
        self.fail_pick = fail_pick
        self.ready = True

    def motion_ready(self):
        return self.ready

    def select_item(self, label):
        self.events.append(("select", label))

    def go_to_blocking(self, name):
        self.events.append(("go", name))
        return name != self.fail_at, "pose"

    def go_to_holding(self, name):
        self.events.append(("hold", name))
        return name != self.fail_at, "pose"

    def pick_blocking(self, label, *, retreat):
        self.events.append(("pick", label, retreat))
        return not self.fail_pick, "pick", {}

    def give_blocking(self, *, wait_s):
        self.events.append(("give", wait_s))
        return True, "given", {}

    def detect_blocking(self, label, *, timeout_s):
        self.events.append(("detect", label, timeout_s))
        return True, "seen", {}


def harness(**kwargs):
    m = FakeManip(**kwargs)
    stopped = []
    arm = RestaurantArm(lambda: m, calibrated_poses(), lambda: True,
                        unsafe_fn=lambda: stopped.append(True))
    return arm, m, stopped


def test_load_additive_merge_preserves_main_poses_limits_and_input(tmp_path):
    main = {"home": {"joints_deg": [1]*6}, "restaurant_carry":
            dict(calibrated_poses()["restaurant_carry"], calibrated=False, source="current")}
    before = deepcopy(main)
    result = merge_restaurant_poses(main, CONFIG)
    assert main == before
    assert result["restaurant_carry"] == main["restaurant_carry"]
    assert set(result) == {"home", *RESTAURANT_POSES}
    assert not poses_calibrated(result)
    assert pose_store.load_pose_config(CONFIG).keys() == {"named_poses"}


@pytest.mark.parametrize("value", [False, None, "true", 1])
def test_each_uncalibrated_pose_blocks_every_callback_without_backend(value):
    for name in RESTAURANT_POSES:
        poses = calibrated_poses()
        poses[name]["calibrated"] = value
        arm = RestaurantArm(lambda: pytest.fail("No crear nodos/MoveIt sin calibracion"), poses,
                            lambda: True, unsafe_fn=lambda: pytest.fail("sin movimiento"))
        assert not arm.pick_item("apple")
        assert not arm.handover()
        assert not arm.release()
        assert not arm.stow()
        assert not arm.detect_item("apple", 2)


def test_disabled_arm_or_allow_motion_never_creates_backend():
    arm = RestaurantArm(lambda: pytest.fail("backend"), calibrated_poses(), lambda: False,
                        unsafe_fn=lambda: None)
    assert not arm.pick_item("water")


@pytest.mark.parametrize("joints", [[0]*5, [999]*6, [float("nan")]*6])
def test_extra_pose_rejects_invalid_joints(tmp_path, joints):
    path = tmp_path / "extra.yaml"
    path.write_text(yaml.safe_dump({"named_poses": {"restaurant_carry": {
        "joints_deg": joints, "gripper_mm": 50, "calibrated": True}}}))
    with pytest.raises(ValueError):
        merge_restaurant_poses({}, path)


@pytest.mark.parametrize("item,label", [("apple", "apple"), (" Orange ", "orange"),
    ("BANANA", "banana"), ("water", "bottle"), ("orange juice", "bottle"),
    ("coffee", "cup"), ("wine", "wine glass"), ("chips", None), ("not in menu", None)])
def test_item_classes(item, label):
    assert item_yolo_class(item) == label


def test_item_without_class_never_detects_or_grasps():
    arm, m, _ = harness()
    assert not arm.detect_item("chips", 1)
    assert not arm.pick_item("chips")
    assert not m.events


def test_detection_uses_class_and_timeout():
    arm, m, _ = harness()
    assert arm.detect_item("water", 2.5)
    assert m.events == [("select", "bottle"), ("detect", "bottle", 2.5)]
    assert not arm.detect_item("apple", float("nan"))


def test_pick_handover_release_and_stow_are_separate_and_ordered():
    arm, m, stopped = harness()
    assert arm.pick_item("apple")
    assert m.events == [("select", "apple"), ("go", "restaurant_pregrasp"),
                        ("pick", "apple", False), ("hold", "restaurant_postgrasp"),
                        ("hold", "restaurant_carry")]
    assert arm.handover()
    assert m.events[-1] == ("hold", "restaurant_handover")
    assert not any(e[0] == "give" for e in m.events)
    assert arm.release()
    assert m.events[-1] == ("give", 0.0)
    assert arm.stow()
    assert m.events[-1] == ("hold", "restaurant_carry")
    assert not stopped


def test_failed_pick_recovers_carry_without_delivering():
    arm, m, stopped = harness(fail_pick=True)
    assert not arm.pick_item("banana")
    assert m.events[-1] == ("hold", "restaurant_carry")
    assert not stopped


def test_failed_carry_recovers_home_and_if_both_fail_stops_session():
    arm, m, stopped = harness(fail_at="restaurant_carry")
    assert not arm.pick_item("banana")
    assert m.events[-1] == ("hold", "home")
    assert not stopped
    m.go_to_holding = lambda name: (False, "controller failure")
    assert not arm.stow()
    assert stopped == [True]


def test_controller_feedback_loss_cancels_operation_and_does_not_start_recovery():
    arm, m, stopped = harness()
    def fail_pick(*_args, **_kwargs):
        m.ready = False
        return False, "lost feedback", {}
    m.pick_blocking = fail_pick
    assert not arm.pick_item("apple")
    assert stopped
    assert not any(event[0] == "hold" for event in m.events)


def test_backend_failure_returns_false_for_tray_fallback():
    def unavailable():
        raise ImportError("missing dependency")
    arm = RestaurantArm(unavailable, calibrated_poses(), lambda: True, unsafe_fn=lambda: None)
    assert not arm.pick_item("apple")


class FakeMotionBase:
    def _status_cb(self, msg):
        self.feedback_ok = json.loads(msg.data)["feedback_ok"]

    def go_to_blocking(self, name):
        self.events.append(("pose", name))
        self.gripper(self.poses[name]["gripper_mm"])
        return True, "at pose"

    def gripper(self, mm):
        self.events.append(("gripper", mm))

    def _execute(self, points):
        self.events.append(("execute", points))
        return True


class FakeGuard(RestaurantMotionGuard, FakeMotionBase):
    def __init__(self):
        self.restaurant_feedback_at = time.monotonic()
        self.restaurant_permitted = lambda: True
        self.enabled = self.feedback_ok = True
        self.abort = False
        self.events = []
        self.poses = calibrated_poses()
        self.gripper_mm = 67.3
        self.fjt = NS(wait_for_server=lambda **_: True)


def test_motion_guard_never_enables_and_handover_preserves_measured_grip():
    m = FakeGuard()
    assert not m.set_torque(True)
    assert not m.set_torque(False)
    original = deepcopy(m.poses)
    assert m.go_to_holding("restaurant_handover")[0]
    assert m.events == [("pose", "restaurant_handover")]
    assert m.gripper_mm == 67.3
    assert m.poses == original
    m.gripper(105)
    assert m.events[-1] == ("gripper", 105)


@pytest.mark.parametrize("failure", ["stale", "disabled", "feedback", "permission", "abort", "action"])
def test_motion_guard_blocks_trajectory_and_gripper(failure):
    m = FakeGuard()
    if failure == "stale":
        m.restaurant_feedback_at -= 2
    elif failure == "disabled":
        m.enabled = False
    elif failure == "feedback":
        m.feedback_ok = False
    elif failure == "permission":
        m.restaurant_permitted = lambda: False
    elif failure == "abort":
        m.abort = True
    else:
        m.fjt = None
    assert not m._execute([1, 2])
    if failure != "action":
        with pytest.raises(RuntimeError):
            m.gripper(105)
    assert not m.events


def test_malformed_feedback_does_not_refresh_guard():
    m = FakeGuard()
    m._status_cb(NS(data="{}"))
    assert not m.motion_ready()
    m._status_cb(NS(data=json.dumps(dict(feedback_ok=True, joints_deg=[0]*6, gripper_mm=60))))
    assert m.motion_ready()


def test_fjt_disappearing_never_falls_back_to_timed_trajectory():
    path = Path(__file__).parents[1] / "base_athome_perception/arm_manipulation.py"
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "_execute")
    env = dict(decimate=lambda points, **_: points, JointTrajectory=lambda: NS(points=[]),
               JointTrajectoryPoint=NS, Duration=NS)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), "exec"), env)
    m = NS(require_feedback_action=True, fjt=None,
           pub_traj=NS(publish=lambda _: pytest.fail("No publicar sin FJT")))
    assert not env["_execute"](m, [([0]*6, 0.0), ([1]*6, 1.0)])


def test_detector_config_is_per_instance_and_filters_requested_item():
    fake = NS(coco_classes=[39], source="fake", detect=lambda _: [
        ((0, 0, 10, 10), .9, "bottle"), ((0, 0, 10, 10), .8, "apple")])
    detector = RestaurantItemDetector(fake)
    assert {46, 47, 49}.issubset(set(fake.coco_classes))
    detector.label = "apple"
    assert [d[2] for d in detector.detect(None)] == ["apple"]


def test_teaching_only_writes_extra_file_and_invalidates_calibration(tmp_path):
    limits = tmp_path / "current.yaml"
    limits.write_text("named_poses: {home: {joints_deg: [1, 2, -3, 4, 5, 6]}}\n")
    before = limits.read_bytes()
    extra = tmp_path / "restaurant.yaml"
    extra.write_text(yaml.safe_dump({"named_poses": calibrated_poses()}))
    q = calibrated_poses()["restaurant_carry"]["joints_deg"]
    assert save_pose("restaurant_carry", q, 64, extra_path=extra, limits_path=limits) is None
    assert limits.read_bytes() == before
    saved = pose_store.load_named_poses(extra)
    assert saved["restaurant_carry"]["calibrated"] is False
    assert saved["restaurant_carry_previous"]["calibrated"] is True
    assert save_pose("home", q, 64, extra_path=extra, limits_path=limits)
    assert save_pose("restaurant_carry", q, 64, extra_path=limits, limits_path=limits)
    assert limits.read_bytes() == before
