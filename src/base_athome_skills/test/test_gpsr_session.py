"""La sesion GPSR del commander y la gramatica oficial, con ASR y ejecutor falsos."""
import json
import threading
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from base_athome_skills.mission.command_parser import compile_gpsr_actions
from base_athome_skills.mission.commander_node import Commander
from base_athome_skills.mission.gpsr_plan import gpsr_actions
from base_athome_skills.mission.places import load_places

MAP = Path(__file__).resolve().parents[3] / "simulation" / "base_athome_sim" / "config" / "sim_semantic_map.yaml"


class World(Node):
    def __init__(self):
        super().__init__("gpsr_world")
        self.script, self.plans, self.said, self.n = [], [], [], 0
        self.pub_result = self.create_publisher(String, "/voice/result", 20)
        self.pub_status = self.create_publisher(String, "/voice/status", 10)
        self.pub_mission = self.create_publisher(String, "/mission/status", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.2, lambda: self.pub_status.publish(String(data="IDLE")))
        self.create_subscription(String, "/voice/listen_request", self._listen, 10)
        self.create_subscription(String, "/mission/run", self._run, 10)
        self.create_subscription(String, "/hri/say_something", lambda m: self.said.append(m.data), 20)

    def _listen(self, msg):
        rid = json.loads(msg.data)["id"]
        status, text = ("ok", self.script.pop(0)) if self.script else ("no_speech", "")
        threading.Timer(0.1, lambda: self.pub_result.publish(String(data=json.dumps(
            dict(id=rid, event="result", status=status, text=text))))).start()

    def _run(self, msg):
        self.plans.append(json.loads(msg.data))
        self.n += 1
        threading.Timer(0.2, lambda n=self.n: self.pub_mission.publish(String(data=json.dumps(
            dict(state="SUCCEEDED", mission_id=f"m{n}", mode="sim", steps=[]))))).start()


@pytest.fixture(scope="module")
def session():
    started_here = not rclpy.ok()
    if started_here:
        rclpy.init()
    w = World()
    w.script = ["go to the kitchen", "yes", "guide me to the table", "yes", "blue tomorrow sings", "blue tomorrow sings",
                "blue tomorrow sings"]
    original = Node.__init__

    def with_params(self, name, **kw):
        if name == "commander":
            kw["parameter_overrides"] = [Parameter("voice", value=True), Parameter("tts", value=False), Parameter("port", value=0),
                                         Parameter("gpsr", value=True), Parameter("gpsr_wait_door", value=False),
                                         Parameter("semantic_map", value=str(MAP)), Parameter("waypoints", value="/nonexistent")]
        original(self, name, **kw)
    Node.__init__ = with_params
    try:
        c = Commander()
    finally:
        Node.__init__ = original
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(w)
    ex.add_node(c)
    threading.Thread(target=ex.spin, daemon=True).start()
    end = time.time() + 40.0
    while time.time() < end and "I have finished all the commands." not in w.said:
        time.sleep(0.1)
    yield w
    ex.shutdown()
    c.destroy_node()
    w.destroy_node()
    if started_here:
        rclpy.shutdown()


def _places(plan):
    return [s["args"].get("place") for s in plan["steps"] if s["skill"] == "navigate_to"]


def test_session_goes_to_the_instruction_point_first_and_after_each_command(session):
    assert _places(session.plans[0]) == ["instruction_point"]
    assert _places(session.plans[1]) == ["kitchen"]
    assert _places(session.plans[2]) == ["instruction_point"]


def test_session_gives_up_on_a_command_after_three_tries(session):
    assert any("custom operator" in s for s in session.said)


def test_session_ends_after_three_commands(session):
    assert "I have finished all the commands." in session.said


def test_official_grammar_compiles_to_available_skills():
    places = load_places(str(MAP))
    c = compile_gpsr_actions(gpsr_actions("give me a bottle from the table"), places)
    assert c.ok, c.errors
    assert [s["skill"] for s in c.plan["steps"]][-1] == "give_object"


def test_official_command_it_cannot_do_is_refused_whole():
    places = load_places(str(MAP))
    c = compile_gpsr_actions(gpsr_actions("go to the kitchen then meet Angel and follow them"), places)
    assert not c.ok and c.errors


def test_it_does_not_pronounce_a_question_it_cannot_answer():
    c = compile_gpsr_actions([dict(action="say", params=dict(text="your teams country"))], load_places(str(MAP)))
    assert not c.ok
