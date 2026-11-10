"""Contrato del nodo con fakes de rostros, cabeza y voz; sin ROS ni biometria real."""

import ast
import json
import math
from pathlib import Path
import sys
import threading
import os
import subprocess
from types import SimpleNamespace as NS
from typing import Optional

import pytest

from base_athome_skills.restaurant_interfaces import (
    RestaurantFaces, head_angles, person_map_point, reset_face_database,
    supported_kwargs, transform_point,
)


def test_kwargs_support_old_new_and_variadic_sessions():
    callback = lambda *args: True
    def old(*, speak_fn):
        return speak_fn
    def new(*, speak_fn, enroll_face_fn=None, pick_item_fn=None):
        return speak_fn, enroll_face_fn, pick_item_fn
    kwargs = dict(speak_fn=callback, enroll_face_fn=callback, pick_item_fn=callback, release_fn=callback)
    assert old(**supported_kwargs(old, kwargs)) is callback
    assert new(**supported_kwargs(new, kwargs)) == (callback, callback, callback)
    assert supported_kwargs(lambda **kw: kw, kwargs) == kwargs
    def positional(x, /, *, speak_fn):
        return speak_fn
    assert "x" not in supported_kwargs(positional, dict(x=1, speak_fn=callback))


def test_node_detection_uses_customer_pans_and_fixed_barman_head():
    methods = node_methods('_detect_customers', '_detect_barman')
    methods['_detect_customers'].__globals__.update(json=json, threading=threading,
                                                   time=NS(time=lambda: 100.))
    requests = []
    node = NS(_bag_lock=threading.Lock(), _bag_pending={},
              session=NS(remaining_s=lambda: 50.),
              get_logger=lambda: NS(info=lambda _: None, warn=lambda _: None))
    def publish(msg):
        request = json.loads(msg.data)
        requests.append(request)
        event, _ = node._bag_pending[request['id']]
        node._bag_pending[request['id']] = (event, {'ok': True, 'customers': [{'xy': [1., 2.]}]})
        event.set()
    node.pub_bag_request = NS(publish=publish)
    node._detect_customers = lambda *a, **kw: methods['_detect_customers'](node, *a, **kw)
    assert node._detect_customers(80., 3) == [{'xy': [1., 2.]}]
    assert requests[-1]['pans'] == [-90., 90., 0.]
    assert requests[-1]['stop_on_first'] is False
    assert methods['_detect_barman'](node, 14., 3) == [{'xy': [1., 2.]}]
    assert requests[-1]['pans'] == [0.]
    assert requests[-1]['stop_on_first'] is True
    assert not node._bag_pending


def test_node_barman_lease_survives_confirmation_and_enrollment_then_cancels():
    methods = node_methods('_track_person', '_stop_tracking', '_refresh_customer_tracking',
                           '_on_session_state_change')
    methods['_track_person'].__globals__.update(person_map_point=person_map_point, json=json,
                                               Dict=dict, Any=object)
    node = type('TrackingNode', (), {k: v for k, v in methods.items() if k.startswith('_')})()
    requests = []
    node.session = NS(customers=[], stopped=False, remaining_s=lambda: 50.)
    node._head_customer_xy = None
    node._person_tracking_xy = None
    node.pub_bag_request = NS(publish=lambda msg: requests.append(json.loads(msg.data)))
    node.pub_state = NS(publish=lambda msg: None)
    node._on_session_state_change({'state': 'CONFIRMING_BARMAN'})
    node._track_person({'x': 2., 'y': 3.})
    assert requests[-1] == {'mode': 'track_customer', 'xy': [2., 3.]}
    node._on_session_state_change({'state': 'ENROLLING_FACE', 'info': {'label': 'barman'}})
    node._refresh_customer_tracking()
    assert requests[-1]['xy'] == [2., 3.]
    node._stop_tracking()
    assert requests[-1]['xy'] is None
    node._track_person({'xy': [2., 3.]})
    node.session.stopped = True
    node._refresh_customer_tracking()
    assert requests[-1]['xy'] is None


def test_database_outside_repo_and_only_dedicated_database_is_cleared(tmp_path, monkeypatch):
    home = tmp_path / "home"
    home.mkdir()
    monkeypatch.setenv("HOME", str(home))
    repo = tmp_path / "public-repo"
    repo.mkdir()
    (repo / ".git").write_text("gitdir: elsewhere")
    monkeypatch.chdir(repo)
    db = reset_face_database()
    assert db == home / ".local/share/base_athome/restaurant_faces"
    assert not db.is_relative_to(repo)
    (db / "old-feature-fake.txt").write_text("fake")
    (db / "thumbnails").mkdir()
    (db / "thumbnails/fake.txt").write_text("fake")
    sibling = db.parent / "receptionist-untouched.txt"
    sibling.write_text("untouched")
    assert reset_face_database() == db
    assert not list(db.iterdir())
    assert sibling.read_text() == "untouched"
    assert db.stat().st_mode & 0o777 == 0o700


def test_database_rejects_repo_home_and_symlink_without_deleting(tmp_path):
    (tmp_path / ".git").mkdir()
    (tmp_path / ".git/HEAD").write_text("ref: refs/heads/main\n")
    with pytest.raises(ValueError):
        reset_face_database(home=tmp_path)
    (tmp_path / ".git/HEAD").unlink()
    (tmp_path / ".git").rmdir()
    dest = tmp_path / "private"
    dest.mkdir()
    (dest / "keep.txt").write_text("keep")
    (tmp_path / ".local").symlink_to(dest, target_is_directory=True)
    with pytest.raises(ValueError):
        reset_face_database(home=tmp_path)
    assert (dest / "keep.txt").read_text() == "keep"


class FakeRecognizer:
    cosine_thresh = .5

    def __init__(self):
        self.faces = [dict(bbox=[280, 150, 80, 80], feature=[1], quality_valid=True,
                           person_id="person_001", similarity=.8)]
        self.saved = []

    def detect_and_recognize(self, image):
        return self.faces

    def similarity(self, a, b):
        return 1.0 if a == b else 0.0

    def enroll_person(self, label, features, **kwargs):
        self.saved.append((label, features, kwargs))
        return "person_001", True


def face_harness(tmp_path):
    clock = NS(now=100.0)
    recognizer = FakeRecognizer()
    aimed = []
    def sleep(seconds):
        clock.now += seconds
    adapter = RestaurantFaces(
        lambda _: recognizer, lambda person: aimed.append(person) or True,
        lambda: (NS(shape=(480, 640, 3)), clock.now),
        clock_fn=lambda: clock.now, sleep_fn=sleep,
    )
    adapter.reset(home=tmp_path)
    return adapter, recognizer, clock, aimed


def test_enroll_points_head_uses_quality_samples_and_exact_contract_labels(tmp_path):
    faces, recognizer, _, aimed = face_harness(tmp_path)
    person = dict(id="track-9", x=1, y=2)
    assert faces.enroll("customer_1", person)
    assert aimed == [person]
    label, features, kwargs = recognizer.saved[0]
    assert label == "customer_1" and len(features) == 4
    assert kwargs["persistent"] is False
    assert "thumbnail_img" not in kwargs
    assert faces.identify(person) == "customer_1"
    assert len(aimed) == 2
    faces.reset(home=tmp_path)
    assert not faces.labels and faces.recognizer is None
    assert faces.identify(person) is None


def test_barman_without_position_can_enroll_single_centered_face(tmp_path):
    faces, _, _, aimed = face_harness(tmp_path)
    assert faces.enroll("barman", None)
    assert not aimed
    assert not faces.enroll("../bad-label", None)


@pytest.mark.parametrize("failure", ["unknown", "low_score", "bad_quality", "multiple", "off_center", "aim", "stale"])
def test_identification_rejects_unreliable_or_ambiguous_face(tmp_path, failure):
    faces, recognizer, clock, _ = face_harness(tmp_path)
    faces.labels["person_001"] = "barman"
    face = recognizer.faces[0]
    if failure == "unknown":
        face["person_id"] = None
    elif failure == "low_score":
        face["similarity"] = .3
    elif failure == "bad_quality":
        face["quality_valid"] = False
    elif failure == "multiple":
        recognizer.faces.append(dict(face))
    elif failure == "off_center":
        face["bbox"] = [0, 100, 80, 80]
    elif failure == "aim":
        faces.aim = lambda _: False
    elif failure == "stale":
        stamp = clock.now
        faces.frame = lambda: (NS(shape=(480, 640, 3)), stamp)
    assert faces.identify(dict(x=1, y=0)) is None


def test_enrollment_rejects_face_switch_and_low_quality(tmp_path):
    faces, recognizer, _, _ = face_harness(tmp_path)
    recognizer.faces[0]["quality_valid"] = False
    assert not faces.enroll("customer_1")
    assert not recognizer.saved


def test_model_failure_degrades_to_false_or_none(tmp_path):
    faces, _, _, _ = face_harness(tmp_path)
    def fail(_):
        raise FileNotFoundError("model")
    faces.recognizer_fn = fail
    assert not faces.enroll("barman")
    assert faces.identify(dict(x=1, y=0)) is None


def test_restaurant_recognizer_does_not_load_any_legacy_database(tmp_path, monkeypatch):
    from base_athome_skills.restaurant_interfaces import make_face_recognizer
    class FakeBase:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.load_database()
        def load_database(self):
            pytest.fail("No leer base del receptionist ni faces_db.pkl")
    monkeypatch.setitem(sys.modules, "base_athome_perception.face_recognizer", NS(FaceRecognizer=FakeBase))
    recognizer = make_face_recognizer(tmp_path / "models", tmp_path / "restaurant_faces")
    assert recognizer.profiles == {}
    assert recognizer.kwargs["db_path"] == str(tmp_path / "restaurant_faces")


def test_map_to_head_transform_and_pan_tilt():
    q = NS(x=0., y=0., z=math.sin(-math.pi/4), w=math.cos(-math.pi/4))
    tf = NS(translation=NS(x=0., y=0., z=-1.12), rotation=q)
    transformed = transform_point((0., 2., 1.2), tf)
    assert transformed == pytest.approx((2., 0., .08))
    assert head_angles(transformed) == pytest.approx((0., 0.))
    assert head_angles((-2., 0., .08)) is None
    assert head_angles((1., 0., 3.)) is None
    assert person_map_point(dict(x=1., y=2.)) == (1., 2., 1.2)
    assert person_map_point(dict(xy=[2., 3.], face_z=1.4)) == (2., 3., 1.4)
    assert person_map_point(dict(x=float("nan"), y=0.)) is None


def node_methods(*names):
    path = Path(__file__).parents[1] / "base_athome_skills/restaurant_node.py"
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    env = dict(Optional=Optional, String=NS, Dict=dict, Any=object)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), "exec"), env)
    return env


def test_speak_blocks_before_next_motion_and_audio_failure_stops():
    methods = node_methods("_speak")
    events = []
    playing, finish = threading.Event(), threading.Event()
    def speak(text, *, blocking):
        assert blocking is True
        events.append("audio-start")
        playing.set()
        assert finish.wait(2)
        events.append("audio-finished")
    node = NS(speaker=NS(speak=speak), _audio_ok=True,
              pub_say=NS(publish=lambda _: None),
              session=NS(stop=lambda: events.append("stop")),
              get_logger=lambda: NS(info=lambda _: None, warn=lambda _: None))
    def mission():
        methods["_speak"](node, "I will pick up the apple.")
        events.append("motion")
    worker = threading.Thread(target=mission)
    worker.start()
    assert playing.wait(1)
    assert events == ["audio-start"]
    finish.set()
    worker.join(2)
    assert events == ["audio-start", "audio-finished", "motion"]
    def fail(*_, **__):
        raise RuntimeError("speaker failure")
    node.speaker.speak = fail
    methods["_speak"](node, "I will move.")
    assert events[-1] == "stop" and not node._audio_ok


def test_face_callback_keeps_tracking_lease_cancelled_during_capture():
    methods = node_methods("_with_face_target", "_enroll_face", "_identify_face")
    node = NS(_face_aiming=False)
    node._with_face_target = lambda fn: methods["_with_face_target"](node, fn)
    node._refresh_customer_tracking = lambda: None
    def capture(*args):
        assert node._face_aiming
        return True
    node._faces = NS(enroll=capture, identify=capture)
    assert methods["_enroll_face"](node, "barman", None)
    assert not node._face_aiming
    assert methods["_identify_face"](node, {})
    assert not node._face_aiming


def test_node_points_head_using_tf_and_confirms_measured_angles():
    methods = node_methods("_aim_at_person")
    clock = NS(now=100.0)
    env = methods["_aim_at_person"].__globals__
    env.update(time=NS(monotonic=lambda: clock.now, sleep=lambda _: None),
               rclpy=NS(time=NS(Time=lambda: None)),
               head_angles=head_angles, transform_point=transform_point, person_map_point=person_map_point)
    lookups, commands = [], []
    tf = NS(translation=NS(x=0., y=0., z=-1.12), rotation=NS(x=0., y=0., z=0., w=1.))
    def lookup(target, source, stamp):
        lookups.append((target, source))
        return NS(transform=tf)
    node = NS(_audio_ok=True, _session_active=lambda: True, _face_aiming=False,
              get_parameter=lambda _: NS(value=1.2), tf_buffer=NS(lookup_transform=lookup),
              session=NS(remaining_s=lambda: 50), _head_feedback=None)
    def point(pan, tilt):
        assert node._face_aiming
        commands.append((pan, tilt))
        clock.now += .1
        node._head_feedback = (pan, tilt, clock.now)
    node._point_head = point
    node._refresh_customer_tracking = lambda: None
    assert methods["_aim_at_person"](node, dict(x=2., y=0.))
    assert lookups == [("pan_tilt_base_link", "map")]
    assert commands[0] == pytest.approx((0., 0.))
    assert not node._face_aiming
    assert not methods["_aim_at_person"](node, dict(x=-2., y=0.))
    assert len(commands) == 1


def test_script_arm_opt_in_defaults_off_in_both_modes():
    script = (Path(__file__).parents[1] / "scripts/restaurant.sh").read_text()
    assert "ARM=${ARM:-0}" in script
    prefix = script.split("HERE=", 1)[0]
    for arm, expected in (("0", "false"), ("1", "true")):
        result = subprocess.run(["bash", "-c", prefix + '\nprintf "%s\\n" "${ARM_ARG[@]}"'],
                                env=dict(os.environ, ARM=arm), capture_output=True, text=True, check=True)
        assert result.stdout.splitlines() == ["-p", f"arm:={expected}", "-p",
                                              f"restaurant_node:allow_motion:={expected}"]
    invocations = [s for s in script.splitlines() if "session_run_terminal ros2 run" in s]
    assert len(invocations) == 2
    assert all('"${ARM_ARG[@]}"' in s for s in invocations)
