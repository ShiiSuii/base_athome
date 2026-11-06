"""Dialogo por voz del commander contra un ASR y un ejecutor falsos, en un dominio ROS aislado. No mueve nada."""
import json
import os
import threading
import time
from pathlib import Path

import pytest

os.environ["ROS_DOMAIN_ID"] = "77"
rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

pytest.importorskip("base_athome_perception.voice_client")
from base_athome_skills.mission.commander_node import Commander  # noqa: E402

MAP = Path(__file__).resolve().parents[3] / "simulation" / "base_athome_sim" / "config" / "sim_semantic_map.yaml"


class FakeWorld(Node):
    """ASR que contesta un guion, y un ejecutor que solo anota lo que recibe."""

    def __init__(self):
        super().__init__("fake_world")
        self.script, self.plans, self.said, self.cancels = [], [], [], 0
        self.pub_result = self.create_publisher(String, "/voice/result", 20)
        self.pub_status = self.create_publisher(String, "/voice/status", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mission = self.create_publisher(String, "/mission/status", latched)
        self.create_timer(0.3, lambda: self.pub_status.publish(String(data="IDLE")))
        self.create_subscription(String, "/voice/listen_request", self._listen, 10)
        self.create_subscription(String, "/mission/run", lambda m: self.plans.append(json.loads(m.data)), 10)
        self.create_subscription(String, "/hri/say_something", lambda m: self.said.append(m.data), 20)
        self.create_service(Trigger, "/mission/cancel", self._cancel)

    def _cancel(self, _req, res):
        self.cancels += 1
        res.success = True
        return res

    def _listen(self, msg):
        req = json.loads(msg.data)
        threading.Timer(0.1, self._answer, args=(req["id"],)).start()

    def _answer(self, rid):
        if not self.script:              # sin guion: silencio, como si nadie hablara
            threading.Timer(0.3, lambda: self._send(rid, "no_speech", "")).start()
            return
        self._send(rid, "ok", self.script.pop(0))

    def _send(self, rid, status, text):
        self.pub_result.publish(String(data=json.dumps(dict(id=rid, event="result", status=status, text=text))))

    def mission(self, state):
        self.pub_mission.publish(String(data=json.dumps(dict(state=state, mode="sim", steps=[]))))


@pytest.fixture(scope="module")
def world():
    rclpy.init()
    fake = FakeWorld()
    node = _make_commander()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(fake)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    assert _until(lambda: any("ready" in s for s in fake.said), 15), fake.said
    yield fake
    ex.shutdown()
    node.httpd.shutdown()
    rclpy.shutdown()


def _make_commander():
    original = Node.__init__

    def with_params(self, name, **kw):
        if name == "commander":
            kw["parameter_overrides"] = [Parameter("voice", value=True), Parameter("tts", value=False),
                                         Parameter("port", value=0), Parameter("semantic_map", value=str(MAP)),
                                         Parameter("waypoints", value="/nonexistent")]
        original(self, name, **kw)
    Node.__init__ = with_params
    try:
        return Commander()
    finally:
        Node.__init__ = original


def _until(cond, timeout=8.0):
    end = time.time() + timeout
    while time.time() < end:
        if cond():
            return True
        time.sleep(0.05)
    return False


def _clear(w):
    w.plans.clear()
    w.said.clear()
    w.script.clear()
    w.mission("IDLE")
    time.sleep(0.2)


def test_yes_runs_the_plan(world):
    _clear(world)
    world.script[:] = ["Go to the kitchen.", "Yes."]
    assert _until(lambda: world.plans), world.said
    assert world.plans[0]["steps"][0]["args"] == {"place": "kitchen"}
    assert any("I will go to the kitchen" in s and "Should I do it" in s for s in world.said)


def test_no_does_not_run(world):
    _clear(world)
    world.script[:] = ["Go to the kitchen.", "No."]
    assert _until(lambda: any("will not do it" in s for s in world.said)), world.said
    assert world.plans == []


def test_silence_on_confirmation_does_not_run(world):
    _clear(world)
    world.script[:] = ["Go to the kitchen."]
    assert _until(lambda: any("will not do it" in s for s in world.said)), world.said
    assert world.plans == []


def test_misheard_confirmation_does_not_run(world):
    _clear(world)
    world.script[:] = ["Go to the kitchen.", "the weather is nice"]
    assert _until(lambda: any("will not do it" in s for s in world.said)), world.said
    assert world.plans == []


def test_unknown_place_is_spoken_and_not_run(world):
    _clear(world)
    world.script[:] = ["Go to the garage."]
    assert _until(lambda: any("do not know the place garage" in s for s in world.said)), world.said
    assert world.plans == []


def test_stop_cancels_a_running_mission_and_other_orders_wait(world):
    _clear(world)
    world.mission("RUNNING")
    time.sleep(0.3)
    before = world.cancels
    world.script[:] = ["Go to the kitchen.", "Stop."]
    assert _until(lambda: world.cancels > before), world.said
    assert _until(lambda: any("Stopping" in s for s in world.said)), world.said    # se dice justo despues de cancelar
    assert any("I am busy" in s for s in world.said)
    assert world.plans == []
    world.mission("IDLE")
