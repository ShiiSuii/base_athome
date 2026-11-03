"""VoiceClient sin ROS real: un nodo falso captura publicadores y suscripciones."""
import json
import time

from std_msgs.msg import String

from base_athome_perception.voice_client import VoiceClient


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Cli:
    def __init__(self):
        self.calls = 0

    def service_is_ready(self):
        return True

    def wait_for_service(self, timeout_sec=0):
        return True

    def call_async(self, req):
        self.calls += 1


class _Log:
    def info(self, *a, **k):
        pass

    warn = error = info


class FakeNode:
    def __init__(self, new_protocol=True):
        self.pubs, self.subs = {}, {}
        self.cli = _Cli()
        self.new_protocol = new_protocol

    def get_logger(self):
        return _Log()

    def create_publisher(self, _t, topic, _q):
        self.pubs[topic] = _Pub()
        return self.pubs[topic]

    def create_subscription(self, _t, topic, cb, _q):
        self.subs[topic] = cb

    def create_client(self, _t, _name):
        return self.cli

    def count_subscribers(self, topic):
        return 1 if self.new_protocol else 0

    # helpers
    def status(self, s="IDLE"):
        self.subs["/voice/status"](String(data=s))

    def result(self, **d):
        self.subs["/voice/result"](String(data=json.dumps(d)))


def test_request_gets_its_own_result_and_ignores_stale_ones():
    n = FakeNode()
    v = VoiceClient(n)
    n.status()
    r1 = v.request(prompt="names")
    sent = json.loads(n.pubs["/voice/listen_request"].msgs[-1].data)
    assert sent["id"] == r1 and sent["prompt"] == "names" and sent["start_timeout"] == 8.0
    n.result(id="viejo", event="result", status="ok", text="texto de una pregunta anterior")
    assert v.poll(r1) is None
    n.result(id=r1, event="result", status="ok", text="My name is Robin", asr_sec=1.2)
    res = v.poll(r1)
    assert res.ok and res.text == "My name is Robin" and res.info["asr_sec"] == 1.2
    assert v.poll(r1) is None  # se consume una sola vez


def test_no_speech_is_explicit():
    n = FakeNode()
    v = VoiceClient(n)
    n.status()
    r = v.request()
    n.result(id=r, event="result", status="no_speech", text="", reason="start_timeout")
    res = v.poll(r)
    assert res.status == "no_speech" and not res.ok


def test_new_request_cancels_previous():
    n = FakeNode()
    v = VoiceClient(n)
    n.status()
    r1 = v.request()
    r2 = v.request()
    assert v.poll(r1).status == "cancelled"
    assert v.is_pending(r2)


def test_offline_asr_is_reported_fast():
    n = FakeNode()
    v = VoiceClient(n)
    r = v.request()              # nunca llego /voice/status: Jetson apagada
    v._pending[r]["t0"] -= 2.0   # pasaron 2 s
    assert v.poll(r).status == "offline"


def test_keyboard_inject_answers_open_request():
    n = FakeNode()
    v = VoiceClient(n)
    n.status()
    r = v.request()
    assert v.inject("Jules and milk") == r
    res = v.poll(r)
    assert res.ok and res.source == "keyboard"


def test_legacy_node_uses_trigger_and_transcription():
    n = FakeNode(new_protocol=False)
    v = VoiceClient(n)
    n.status()
    r = v.request()
    assert n.cli.calls == 1 and not n.pubs["/voice/listen_request"].msgs
    n.subs["/voice/transcription"](String(data="My name is Paris"))
    assert v.poll(r).text == "My name is Paris"


def test_speech_start_extends_deadline():
    n = FakeNode()
    v = VoiceClient(n, start_timeout=1.0, max_speech=2.0)
    n.status()
    r = v.request()
    v._pending[r]["deadline"] = time.time() + 0.1
    n.result(id=r, event="speech_start")
    assert v.speech_started(r) and v._pending[r]["deadline"] > time.time() + 5
