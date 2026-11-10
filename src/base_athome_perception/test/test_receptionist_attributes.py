"""Synthetic images and mocked state transitions; no ROS, models or robot I/O."""

import json
from unittest import mock

import numpy as np
import pytest

from base_athome_perception.person_attributes import ALL_COLORS, AttributeResult, PersonAttributes
from base_athome_perception.receptionist_attributes import (
    confident_attributes, describe_guest_attributes, extract_guest_attributes, prefer_attributes,
)
from receptionist_offline import load_receptionist
from test_person_attributes import make_synthetic_person_image
from test_receptionist_start import make_mock_node


@pytest.fixture
def observation():
    frame, points = make_synthetic_person_image()
    pose = {"bbox": [250, 20, 390, 455], "keypoints": points[:, :2],
            "keypoints_conf": points[:, 2]}
    return frame, [pose], [{"bbox": [290, 30, 60, 60]}]


def test_extract_store_and_say_four_fields_in_one_short_sentence(observation):
    attributes = extract_guest_attributes(*observation)
    assert attributes["torso"]["color"] == "red"
    assert attributes["torso"]["item"] == "shirt"
    assert attributes["legs"]["color"] == "dark"
    assert attributes["legs"]["item"] == "trousers"
    phrase = describe_guest_attributes(json.loads(json.dumps(attributes)))
    assert phrase == "Our first guest is wearing a red shirt and dark trousers."
    assert len(phrase.split()) <= 20
    assert phrase.count(".") == 1
    assert "glasses" not in phrase


@pytest.mark.parametrize("confidence", [0.49, float("nan"), float("inf"), -1, 1.01])
def test_untrusted_garment_is_never_spoken(confidence):
    result = PersonAttributes(
        torso=AttributeResult("red", "shirt", "red shirt", confidence),
        legs=AttributeResult("blue", "trousers", "blue trousers", 0.9))
    attributes = confident_attributes(result)
    assert set(attributes) == {"legs"}
    assert describe_guest_attributes(attributes) == "Our first guest is wearing blue trousers."
    # Recheck confidence when reading persisted attributes, too.
    assert describe_guest_attributes({"torso": {
        "color": "red", "item": "shirt", "confidence": confidence}}) == ""


def test_missing_or_unsupported_attributes_are_not_filled_in():
    assert confident_attributes(None) == {}
    assert describe_guest_attributes({}) == ""
    assert describe_guest_attributes({"glasses": {"confidence": 1.0}}) == ""
    assert confident_attributes(PersonAttributes(torso=AttributeResult(
        "red", "dress", "red dress", 1.0))) == {}


@pytest.mark.parametrize("color", ALL_COLORS)
def test_full_description_stays_within_word_budget_for_every_color(color):
    phrase = describe_guest_attributes({
        "torso": {"color": color, "item": "shirt", "confidence": 0.9},
        "legs": {"color": color, "item": "trousers", "confidence": 0.9},
    })
    assert phrase.count(color) == 2
    assert "shirt" in phrase and "trousers" in phrase
    assert len(phrase.split()) <= 20
    assert phrase.count(".") == 1


@pytest.mark.parametrize("confidence", [float("nan"), float("inf"), -1, 1.01])
def test_invalid_keypoint_confidence_cannot_support_attributes(observation, confidence):
    frame, poses, faces = observation
    poses[0]["keypoints_conf"][5:15] = confidence
    assert extract_guest_attributes(frame, poses, faces) == {}


def test_occluded_legs_only_describe_observed_torso(observation):
    frame, poses, faces = observation
    poses[0]["keypoints_conf"][13:] = 0
    attributes = extract_guest_attributes(frame, poses, faces)
    assert set(attributes) == {"torso"}
    assert "trousers" not in describe_guest_attributes(attributes)


@pytest.mark.parametrize("missing", ["frame", "poses", "faces", "confidence"])
def test_missing_visual_evidence_abstains(observation, missing):
    frame, poses, faces = observation
    if missing == "frame":
        frame = None
    elif missing == "poses":
        poses = []
    elif missing == "faces":
        faces = []
    else:
        del poses[0]["keypoints_conf"]
    assert extract_guest_attributes(frame, poses, faces) == {}


def test_association_rejects_other_person_or_overlapping_bodies(observation):
    frame, poses, faces = observation
    assert extract_guest_attributes(frame, poses, [{"bbox": [0, 0, 30, 30]}]) == {}
    assert extract_guest_attributes(frame, poses * 2, faces) == {}
    # Another, larger body away from the interlocutor cannot replace their pose.
    distractor = dict(poses[0], bbox=[0, 0, 240, 480])
    assert extract_guest_attributes(frame, [distractor] + poses, faces) == extract_guest_attributes(*observation)


def test_best_snapshot_is_coherent_and_does_not_alias_worker_data(observation):
    complete = extract_guest_attributes(*observation)
    partial = {"torso": dict(complete["torso"], confidence=1.0)}
    saved = prefer_attributes(complete, partial)
    assert saved == complete
    saved["torso"]["color"] = "blue"
    assert complete["torso"]["color"] == "red"
    assert prefer_attributes({}, partial) == partial


@pytest.fixture
def module():
    return load_receptionist()


@pytest.mark.parametrize("timeout", [False, True])
def test_both_enrollment_paths_save_attributes(module, observation, monkeypatch, timeout):
    monkeypatch.setattr(module.time, "time", lambda: 100.0)
    n = make_mock_node(module.HriReceptionistChallengeNode)
    n.state = n.STATE_MEMORIZE_FACE
    n.state_start_time = 90.0 if timeout else 99.0
    n.visual_attributes_stamp = 99.5
    n.visual_attributes = extract_guest_attributes(*observation)
    n.current_attributes = {}
    n.registered_guests = []
    n.recording_samples = [] if timeout else [np.ones((1, 128))] * 4
    n.target_sample_count = 4
    n.asked_closer = True
    n.best_sharpness = 0.0
    n.best_thumbnail = None
    n.samples_synthetic = False
    n.seats = {}
    n.current_name, n.current_drink = "Alex", "Tea"
    n.recognizer = mock.Mock(profiles={})
    n.recognizer.enroll_person.return_value = ("guest1", True)
    n._update_session_storage = mock.Mock()
    n.session_file = "unused-offline"
    n.step()
    assert n.state == n.STATE_NAV_TO_LIVING_ROOM
    saved = n.registered_guests[0]["visual_attributes"]
    assert saved == n.visual_attributes
    assert n._update_session_storage.call_args.args[0]["visual_attributes"] == saved
    assert n.recognizer.enroll_person.call_args.kwargs["attributes"]["visual_attributes"] == saved


@pytest.mark.parametrize("stamp", [80.0, 98.0, 101.0])
def test_stale_pre_registration_or_future_observations_are_ignored(module, observation, monkeypatch, stamp):
    monkeypatch.setattr(module.time, "time", lambda: 100.0)
    n = make_mock_node(module.HriReceptionistChallengeNode)
    n.state = n.STATE_MEMORIZE_FACE
    n.state_start_time = 98.5
    n.visual_attributes_stamp = stamp
    n.visual_attributes = extract_guest_attributes(*observation)
    n.current_attributes = {}
    n.recording_samples = []
    n.target_sample_count = 10
    n.step()
    assert n.current_attributes == {}


def test_second_guest_hears_only_first_guests_stored_attributes(module, observation):
    n = make_mock_node(module.HriReceptionistChallengeNode)
    n.state = n.STATE_INTRODUCE_GUESTS
    n.guest_index = 2
    n.current_name, n.current_drink = "Taylor", "Coffee"
    n.registered_guests = [{"name": "Alex", "drink": "Tea",
                            "visual_attributes": extract_guest_attributes(*observation)}]
    n.current_attributes = {"torso": {"color": "blue", "item": "shirt", "confidence": 1.0}}
    n.step()
    phrases = [call.args[0] for call in n.say.call_args_list]
    assert len(phrases) == 2
    assert phrases[1] == describe_guest_attributes(n.registered_guests[0]["visual_attributes"])
    assert "blue" not in phrases[1]


def test_no_description_is_spoken_when_first_guest_has_no_attributes(module):
    n = make_mock_node(module.HriReceptionistChallengeNode)
    n.state = n.STATE_INTRODUCE_GUESTS
    n.guest_index = 2
    n.current_name, n.current_drink = "Taylor", "Coffee"
    n.registered_guests = [{"name": "Alex", "drink": "Tea"}]
    n.step()
    n.say.assert_called_once()


def test_worker_extracts_attributes_from_same_frame_as_face_and_pose(module, observation, monkeypatch):
    frame, poses, faces = observation
    n = make_mock_node(module.HriReceptionistChallengeNode)
    n.latest_frame = frame
    n.frame_count = 1
    n.last_pose_time = 0.0
    n.state = n.STATE_MEMORIZE_FACE
    n.face_worker_stop = mock.Mock()
    n.face_worker_stop.is_set.side_effect = [False, True]
    n.recognizer = mock.Mock()
    n.recognizer.detect_and_recognize.return_value = faces
    n.pose_detector = mock.Mock()
    n.pose_detector.detect_poses.return_value = poses
    monkeypatch.setattr(module.time, "sleep", lambda _: None)
    n._face_worker_loop()
    assert n.recognizer.detect_and_recognize.call_args.args[0] is frame
    assert n.pose_detector.detect_poses.call_args.args[0] is frame
    assert n.visual_attributes == extract_guest_attributes(frame, poses, faces)
    assert n.visual_attributes_stamp == n.face_dets_stamp


def test_memory_clear_removes_stored_attributes(module, observation):
    n = make_mock_node(module.HriReceptionistChallengeNode)
    n.current_attributes = extract_guest_attributes(*observation)
    n.registered_guests = [{"visual_attributes": n.current_attributes}]
    n.recognizer = mock.Mock()
    n._init_session_storage = mock.Mock()
    n.clear_temporary_memory()
    assert n.current_attributes == {}
    assert n.registered_guests == []
