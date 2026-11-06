"""Ensenar la pose de piso y agarrar con la camara de muneca. Sin hardware: dominio aislado y nodo falso."""
import importlib.util
import math
import os
import threading
import time
from pathlib import Path

import pytest

from base_athome_perception import wrist_geometry as wg
from base_athome_perception.teach_wizard import FLOOR_POSE, TeachWizard
from base_athome_perception.wrist_detector import BAG_NAMES, BAG_WORLD_CLASSES
from base_athome_skills.mission import bag_task as bt

CAL = wg.WristCalib()
os.environ.setdefault("ROS_DOMAIN_ID", "77")


def test_bag_classes_keep_distractors_out_of_the_results():
    assert BAG_NAMES < set(BAG_WORLD_CLASSES) and "person" in BAG_WORLD_CLASSES and "person" not in BAG_NAMES
    assert all("bag" in n for n in BAG_NAMES)           # el pedido de la mision busca "bag"


def test_own_gripper_and_base_at_the_bottom_of_the_image_are_not_a_bag():
    from base_athome_perception.wrist_detector import is_own_robot_box
    assert is_own_robot_box((482, 351, 639, 479), 480)    # chasis, imagen real del 2026-11-05 (conf 0,15)
    assert is_own_robot_box((0, 360, 110, 480), 480)      # garra
    assert not is_own_robot_box((277, 5, 514, 238), 480)  # la bolsa real de esa imagen (conf 0,57)
    assert not is_own_robot_box((200, 150, 440, 400), 480)


def test_wizard_can_teach_only_the_floor_pose():
    saved = []
    wiz = TeachWizard(lambda on: True, lambda n, q, g: saved.append(n))
    wiz.start(only=FLOOR_POSE)
    assert wiz.current == FLOOR_POSE
    wiz.confirm([0.0] * 6, 70.0)
    assert saved == [FLOOR_POSE] and not wiz.active


rclpy = pytest.importorskip("rclpy")
if os.environ.get("ROS_DOMAIN_ID") != "77":
    pytest.skip("solo en el dominio 77", allow_module_level=True)
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "ensenar_pregrasp.py"
spec = importlib.util.spec_from_file_location("ensenar_pregrasp", SCRIPT)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)
GOOD = bt.FLOOR_PREGRASP_DEG
TOO_CLOSE = [48.06, 122.69, -51.16, 62.39, -61.1, -111.96]     # la pre_grasp de mesa: pinza horizontal, cerca


class FakeArm(Node):
    def __init__(self):
        super().__init__("fake_arm")
        self.q, self.enabled, self.calls = list(GOOD), True, []
        self.pub_js = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.pub_st = self.create_publisher(String, "/arm/status", 10)
        self.pub_torso = self.create_publisher(String, "/torso/status", 10)
        self.torso = 0.0
        self.create_service(SetBool, "/arm/set_enable", self._srv)
        self.create_timer(0.05, self._tick)

    def _srv(self, req, res):
        self.enabled = res.success = bool(req.data)
        self.calls.append(bool(req.data))
        return res

    def _tick(self):
        self.pub_js.publish(JointState(name=[f"joint{i}" for i in range(1, 7)], position=[math.radians(v) for v in self.q]))
        self.pub_st.publish(String(data='{"feedback_ok": true, "gripper_mm": 70.0}'))
        self.pub_torso.publish(String(data='{"position_m": %f}' % self.torso))


@pytest.fixture()
def rig(tmp_path):
    cfg = tmp_path / "poses.yaml"
    cfg.write_text("joint_limits_deg:\n  joint1: [-154, 154]\n  joint2: [0, 195]\n  joint3: [-175, 0]\n  joint4: [-110, 110]\n"
                   "  joint5: [-85, 85]\n  joint6: [-180, 180]\nnamed_poses: {}\n")
    rclpy.init()
    arm, said = FakeArm(), []
    node = tool.Teacher(out=said.append, config=str(cfg))
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(arm)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    end = time.time() + 5
    while not (node.fresh() and node.feedback and node.torso is not None) and time.time() < end:
        time.sleep(0.05)
    yield arm, node, said, cfg
    ex.shutdown()
    rclpy.shutdown()


def test_good_pose_is_saved_and_arm_ends_locked(rig):
    arm, node, _, cfg = rig
    assert node.start() and arm.calls == [False]         # suelto para moverlo a mano
    node.confirm()
    assert arm.calls == [False, True] and not node.wizard.active
    from base_athome_perception import pose_store
    assert pose_store.load_named_poses(str(cfg))[FLOOR_POSE]["joints_deg"] == pytest.approx(GOOD)


def test_pose_too_close_to_the_base_is_refused_and_stays_free(rig):
    arm, node, said, cfg = rig
    node.start()
    arm.q = list(TOO_CLOSE)
    time.sleep(0.2)
    node.confirm()
    assert node.wizard.active and "No guardado" in node.wizard.message
    assert FLOOR_POSE not in cfg.read_text()


def test_refuses_to_teach_with_the_lift_down(rig):
    arm, node, said, _ = rig
    arm.torso = -0.30
    time.sleep(0.2)
    assert node.start() is False and arm.calls == []     # ni suelta el brazo
    assert any("subilo" in s for s in said)


def test_teaching_changes_where_the_robot_stands():
    """La pose ensenada manda: una pose mas adelante deja la bolsa mas lejos de la base."""
    near = bt.floor_grasp_from_tcp((0.40, 0.10, 0.95), (0.4, 0.0, -0.9))
    far = bt.floor_grasp_from_tcp((0.60, 0.10, 0.95), (0.4, 0.0, -0.9))
    assert bt.base_clearance(grasp=far) > bt.base_clearance(grasp=near)
    bag = (2.0, 1.0, 0.3)
    x, y, yaw = bt.stand_pose(bag, (0, 0), grasp=far)
    bx, by, _ = bt.object_in_base(bag, (x, y, yaw))
    assert bx == pytest.approx(far.reach_x, abs=1e-6) and by == pytest.approx(far.reach_y, abs=1e-6)


def test_describe_rejects_a_pose_whose_bag_would_touch_the_base():
    _, _, problems = tool.describe([48.06, 122.69, -51.16, 62.39, -61.1, -111.96], tool.PiperKinematicsSolver())
    assert problems
    near = bt.floor_grasp_from_tcp((0.30, 0.10, 0.95), (0.3, 0.0, -0.9))
    assert bt.base_clearance(grasp=near) < bt.MIN_BASE_CLEARANCE_M


def test_chassis_comes_from_the_robot_model():
    assert bt.CHASSIS_FRONT_M == pytest.approx(0.24) and bt.LASER_X_M == pytest.approx(0.22)


def test_describe_accepts_the_default_pose_and_rejects_a_long_advance():
    kin = tool.PiperKinematicsSolver()
    assert tool.describe(GOOD, kin)[2] == []
    _, _, problems = tool.describe(GOOD, kin, max_advance_m=0.05)
    assert any("avanzar" in p for p in problems)


def test_full_calibration_refuses_unless_the_arm_is_in_the_floor_pose():
    spec2 = importlib.util.spec_from_file_location("calibrar_elevador", SCRIPT.parent / "calibrar_elevador.py")
    mod = importlib.util.module_from_spec(spec2)
    spec2.loader.exec_module(mod)
    assert mod.arm_near_pose(GOOD, GOOD)[0] is True
    hanging = [84.8, 172.2, -162.4, -102.3, -2.3, -38.0]   # colgando, medido en el robot
    assert mod.arm_near_pose(hanging, GOOD)[0] is False
