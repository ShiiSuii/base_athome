"""Calibrador del pan-tilt: calculos, y el flujo contra un nodo falso en un dominio aislado. No toca la cabeza real."""
import importlib.util
import math
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pytest

from base_athome_perception import pan_tilt_calibration as cal

PARAMS = dict(pan_center_tick=2059, tilt_center_tick=2048, pan_min_rad=-1.6567, pan_max_rad=1.5493,
              tilt_min_rad=-0.4878, tilt_max_rad=0.615, ticks_per_rad=651.8986, baudrate=1000000)


def test_new_center_makes_current_position_zero():
    """Con direccion -1: logico = -(tick - centro) / tpr. El centro nuevo anula la posicion actual."""
    tick = 2300
    logical = -1.0 * (tick - 2059) / 651.8986
    assert cal.new_center_tick(2059, logical, -1.0, 651.8986) == tick


def test_limits_are_kept_in_place_when_center_moves():
    """Mover el centro no mueve los topes fisicos: los limites relativos se corren lo mismo."""
    new = cal.updated_params(PARAMS, dict(pan_center=10.0, tilt_center=0.0), -1.0, -1.0)
    shift = (new["pan_center_tick"] - 2059) / 651.8986
    assert new["pan_center_tick"] < 2059                 # 10 grados a la izquierda con direccion -1
    assert new["pan_min_rad"] == pytest.approx(-1.6567 - shift, abs=1e-3)
    assert new["pan_max_rad"] == pytest.approx(1.5493 - shift, abs=1e-3)
    assert new["tilt_center_tick"] == 2048 and new["baudrate"] == 1000000


def test_marked_limits_translate_to_physical_range():
    new = cal.updated_params(PARAMS, dict(pan_left=80.0, pan_right=-70.0), -1.0, -1.0)
    assert new["pan_min_rad"] == pytest.approx(-math.radians(80.0), abs=1e-3)
    assert new["pan_max_rad"] == pytest.approx(math.radians(70.0), abs=1e-3)
    limits = cal.logical_limits(new, -1.0, -1.0)
    assert limits["pan"] == pytest.approx((-70.0, 80.0), abs=0.1)


def test_yaml_round_trip_is_what_the_node_reads():
    text = cal.params_to_yaml(PARAMS)
    assert cal.params_from_yaml(text) == PARAMS
    assert "pan_tilt_node" in text and "ros__parameters" in text


def test_ramp_respects_speed_and_ends_at_goal():
    steps = cal.ramp((0.0, 0.0), (30.0, -10.0), 30.0, 20.0)
    assert len(steps) == 20 and steps[-1] == (30.0, -10.0)


def test_scan_path_stays_inside_limits():
    limits = dict(pan=(-50.0, 50.0), tilt=(-20.0, 25.0))
    for p, t in cal.scan_path(limits):
        assert -50.0 <= p <= 50.0 and -20.0 <= t <= 25.0


def test_stats_find_the_lag():
    samples = [(i * 0.05, 20 * math.sin(i * 0.05), 0.0, 20 * math.sin((i - 6) * 0.05), 0.0) for i in range(200)]
    stats = cal.tracking_stats(samples)
    assert stats["lag_s"] == pytest.approx(0.3, abs=0.06) and stats["rms_sin_lag_deg"] < 0.5


def test_verdict_blames_electrics_first_and_says_ok_when_clean():
    assert cal.verdict([0.5], dict(lag_s=0.1), 0, 0)[0].startswith("OK")
    lines = cal.verdict([9.0], dict(lag_s=0.1), 2, 3, ["pan"])
    assert lines[0].startswith("FALLA ELECTRICA") and "cable" in lines[0]
    assert any("SENTIDO" in x for x in lines) and any("NO LLEGA" in x for x in lines)


def _scene(seed=3, h=480, w=640):
    rng = np.random.default_rng(seed)
    return cv2.GaussianBlur(rng.integers(0, 255, (h, w, 3), dtype=np.uint8), (15, 15), 0)


def test_image_shift_sign_right_and_down_are_positive():
    a = _scene()
    b = np.roll(a, 60, axis=1)                           # el contenido se corre 60 px a la derecha
    dx, dy, conf = cal.image_shift(a, b)
    assert dx == pytest.approx(60, abs=4) and abs(dy) < 4 and conf > 0.05
    dx, dy, _ = cal.image_shift(a, np.roll(a, 40, axis=0))
    assert dy == pytest.approx(40, abs=4)


def test_sense_is_about_the_image_not_the_viewer():
    assert cal.sense_from_shift(10.0, 90.0, 0.5, "pan") == "ok"           # izquierda: escena a la derecha
    assert cal.sense_from_shift(10.0, -90.0, 0.5, "pan") == "inverted"
    assert cal.sense_from_shift(-10.0, -90.0, 0.5, "pan") == "ok"
    assert cal.sense_from_shift(10.0, 90.0, 0.01, "pan") == "unknown"     # imagen sin informacion
    assert cal.sense_from_shift(10.0, 3.0, 0.5, "pan") == "unknown"       # casi no se movio: no concluir


# ---------------------------------------------------------------- flujo con ROS
os.environ.setdefault("ROS_DOMAIN_ID", "77")
if os.environ.get("ROS_DOMAIN_ID") != "77":
    pytest.skip("solo en el dominio 77", allow_module_level=True)
rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Vector3  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import Image, JointState  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "pan_tilt_calibrate.py"
spec = importlib.util.spec_from_file_location("pan_tilt_calibrate", SCRIPT)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


class FakeHead(Node):
    """pan_tilt_node falso: sigue el objetivo con un primer orden; `sense` -1 invierte el pan."""

    def __init__(self):
        super().__init__("pan_tilt_node")
        self.pan, self.tilt, self.tp, self.tt, self.torque, self.sense = 0.0, 0.0, 0.0, 0.0, False, 1.0
        self.commands = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_link = self.create_publisher(Bool, "/pan_tilt/connected", latched)
        self.pub_link.publish(Bool(data=True))
        self.pub_js = self.create_publisher(JointState, "/pan_tilt/joint_states", 10)
        self.create_subscription(Vector3, "/pan_tilt/target_degrees", self._cmd, 20)
        self.create_service(SetBool, "/pan_tilt/set_torque", self._srv)
        self.create_timer(0.05, self._tick)
        self.scene = _scene()
        self.image_sense = 1.0                           # -1: el motor gira al reves de lo que dice su posicion
        self.pub_img = self.create_publisher(Image, tool.CAMERA_TOPIC, qos_profile_sensor_data)
        self.create_timer(0.066, self._image)

    def _image(self):
        px = int(round(self.image_sense * self.pan * 10.0))    # ~10 px por grado
        py = int(round(self.image_sense * self.tilt * 10.0))
        img = np.roll(np.roll(self.scene, px, axis=1), py, axis=0)
        self.pub_img.publish(Image(height=img.shape[0], width=img.shape[1], encoding="bgr8", step=img.shape[1] * 3,
                                   data=img.tobytes()))

    def _cmd(self, m):
        self.commands += 1
        if self.torque:
            self.tp, self.tt = self.sense * m.x, m.y

    def _srv(self, req, res):
        self.torque, res.success = bool(req.data), True
        return res

    def _tick(self):
        self.pan += 0.4 * (self.tp - self.pan)
        self.tilt += 0.4 * (self.tt - self.tilt)
        self.pub_js.publish(JointState(name=["pan_joint", "tilt_joint"],
                                       position=[math.radians(self.pan), math.radians(self.tilt)]))


@pytest.fixture()
def rig():
    rclpy.init()
    head, said = FakeHead(), []
    node = tool.PanTiltCalibrator(use_ssh=False, out=said.append)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(head)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    end = time.time() + 5
    while (node.connected is None or not node.fresh()) and time.time() < end:
        time.sleep(0.05)
    yield head, node, said
    ex.shutdown()
    rclpy.shutdown()


def test_no_motion_without_torque(rig):
    head, node, said = rig
    node.key("a")
    assert head.commands == 0 and any("torque" in s for s in said)


def test_read_only_mode_never_commands(rig):
    head, node, said = rig
    node.read_only = True
    node.key("t")
    node.key("a")
    node.key("m")
    assert head.commands == 0 and not head.torque


def test_jog_moves_by_the_step_and_does_not_jump_on_torque(rig):
    head, node, _ = rig
    head.pan = head.tp = 20.0                            # la cabeza estaba corrida
    time.sleep(0.2)
    assert node.set_torque(True)
    time.sleep(0.2)
    assert head.tp == pytest.approx(20.0, abs=1.0)       # no salta a cero
    node.key("a")                                        # izquierda = pan +
    assert head.tp == pytest.approx(25.0, abs=1.0)


def test_full_test_passes_on_a_healthy_head(rig):
    _, node, _ = rig
    node.set_torque(True)
    report = node.full_test()
    assert report and not report["aborted"] and report["verdict"][0].startswith("OK"), report
    assert report["stats"]["n"] > 100


def test_full_test_detects_inverted_pan(rig):
    head, node, _ = rig
    node.set_torque(True)
    head.sense = -1.0
    report = node.full_test()
    assert "pan" in report["wrong"]


def test_full_test_stops_when_link_drops(rig):
    head, node, _ = rig
    node.set_torque(True)
    threading.Timer(1.5, lambda: head.pub_link.publish(Bool(data=False))).start()
    report = node.full_test()
    time.sleep(0.1)
    sent = head.commands
    time.sleep(0.5)
    assert report["aborted"] and report["link_drops"] == 1 and head.commands == sent
    assert report["verdict"][0].startswith("FALLA ELECTRICA")
    assert node.key("a") is None and head.commands == sent   # sin enlace no manda nada


def test_save_writes_only_with_marks(rig, tmp_path, monkeypatch):
    _, node, said = rig
    target = tmp_path / "params.yaml"
    monkeypatch.setattr(tool, "REPO_YAML", str(target))
    assert node.save() is False and not target.exists()
    node.marks = dict(pan_center=5.0, tilt_center=0.0)
    assert node.save() is True
    assert cal.params_from_yaml(target.read_text())["pan_center_tick"] != node.params["pan_center_tick"]


def test_verify_sense_says_ok_when_image_agrees(rig):
    _, node, said = rig
    node.set_torque(True)
    assert node.verify_sense() == dict(pan="ok", tilt="ok")
    assert any("no hay que cambiar nada" in x for x in said)


def test_verify_sense_detects_a_really_inverted_motor(rig):
    head, node, said = rig
    node.set_torque(True)
    head.image_sense = -1.0
    assert node.verify_sense() == dict(pan="inverted", tilt="inverted")
    assert any("pan_direction" in x for x in said)


def test_verify_sense_does_not_conclude_without_camera(rig, monkeypatch):
    _, node, _ = rig
    node.set_torque(True)
    monkeypatch.setattr(node, "_image", lambda timeout=3.0: None)
    assert node.verify_sense() == dict(pan="unknown", tilt="unknown")


def test_verify_sense_needs_torque_and_returns_to_start(rig):
    head, node, _ = rig
    assert node.verify_sense() is None and head.commands == 0
    node.set_torque(True)
    node.jog(5.0, 0.0)
    node.verify_sense()
    time.sleep(0.4)
    assert head.tp == pytest.approx(5.0, abs=0.5) and head.tt == pytest.approx(0.0, abs=0.5)
