"""Deadline policy and real DSP connected to mocked receptionist transitions."""

import threading
from types import SimpleNamespace

import numpy as np
import pytest

from base_athome_perception.doorbell_detector import DoorbellDetector
from base_athome_perception.receptionist_doorbell import doorbell_wait_status
from receptionist_offline import load_receptionist
from test_receptionist_start import make_mock_node


@pytest.mark.parametrize("enabled,start,now,event,expected", [
    (False, None, 0, None, "disabled"),
    (True, None, 0, None, "waiting"),
    (True, 10, 11, None, "waiting"),
    (True, 10, 12, None, "timeout"),
    (True, 10, 13, 11, "detected"),  # poll may happen after deadline
    (True, 10, 11, 9, "waiting"),  # previous guest
    (True, 10, 12, 12, "timeout"),  # exactly at deadline is too late
    (True, 10, 13, 12.1, "timeout"),
    (True, 10, 11, 11.5, "waiting"),  # future event cannot trigger
])
def test_wait_policy(enabled, start, now, event, expected):
    assert doorbell_wait_status(enabled, start, now, 2.0, event) == expected


@pytest.mark.parametrize("timeout", [0, -1, float("inf"), float("nan")])
def test_wait_requires_finite_positive_timeout(timeout):
    with pytest.raises(ValueError):
        doorbell_wait_status(True, 10, 11, timeout)


@pytest.fixture
def reception(monkeypatch):
    module = load_receptionist()
    clock = [10.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    n = make_mock_node(module.HriReceptionistChallengeNode)
    n.wait_doorbell = True
    n.doorbell_timeout_s = 2.0
    n.doorbell_lock = threading.Lock()
    n.doorbell_started_at = None
    n.doorbell_detected_at = None
    n.doorbell_detector = DoorbellDetector(armed=False)
    return n, clock


def send_tone(n, clock, duration=0.3):
    samples = (0.25 * 32767 * np.sin(2 * np.pi * 1000 *
                                   np.arange(int(duration * 16000)) / 16000)).astype(np.int16)
    for start in range(0, len(samples), 512):
        block = samples[start:start + 512]
        clock[0] += len(block) / 16000
        n._doorbell_audio_callback(SimpleNamespace(data=block.tolist()))


def test_disabled_mode_preserves_original_arrival(reception):
    n, _ = reception
    n.wait_doorbell = False
    n._prepare_door_arrival()
    assert n.state == n.STATE_NAV_TO_DOOR
    assert not n.doorbell_detector.is_armed


def test_start_transitions_to_wait_before_any_navigation(reception):
    n, _ = reception
    n.start_confirmed = True
    n.step()
    assert n.state == n.STATE_WAIT_DOORBELL
    n.navigate_to_waypoint.assert_not_called()


def test_wait_is_nonblocking_and_timeout_continues_without_false_detection(reception):
    n, clock = reception
    n._prepare_door_arrival()
    for _ in range(5):
        n.step()
        assert n.state == n.STATE_WAIT_DOORBELL
    n.say.assert_not_called()
    n.sleep_gui.assert_not_called()
    n.navigate_to_waypoint.assert_not_called()
    clock[0] = 12.0
    n.step()
    assert n.state == n.STATE_NAV_TO_DOOR
    assert n.doorbell_detected_at is None
    assert not n.doorbell_detector.is_armed
    n.get_logger().warn.assert_called_once()


def test_real_detector_triggers_arrival_from_synthetic_pcm(reception):
    n, clock = reception
    n._prepare_door_arrival()
    send_tone(n, clock)
    assert n.doorbell_detected_at is not None
    n.step()
    assert n.state == n.STATE_NAV_TO_DOOR
    assert not n.doorbell_detector.is_armed
    n.navigate_to_waypoint.assert_not_called()  # navigation belongs to the next cycle


def test_audio_outside_wait_silence_and_oversized_blocks_do_not_trigger(reception):
    n, clock = reception
    send_tone(n, clock)
    assert n.doorbell_detected_at is None
    n._prepare_door_arrival()
    for _ in range(10):
        n._doorbell_audio_callback(SimpleNamespace(data=[0] * 512))
    n._doorbell_audio_callback(SimpleNamespace(data=[32767] * 1601))
    n._doorbell_audio_callback(SimpleNamespace(data=[]))
    assert n.doorbell_detected_at is None
    n.step()
    assert n.state == n.STATE_WAIT_DOORBELL


def test_each_guest_rearms_and_discards_previous_detection(reception):
    n, clock = reception
    n._prepare_door_arrival()
    send_tone(n, clock)
    n.step()
    n.state = n.STATE_CHECK_NEXT_GUEST
    n.max_guests = 2
    n.current_attributes = {"old": {}}
    n.step()
    assert n.guest_index == 2
    assert n.current_attributes == {}
    assert n.state == n.STATE_WAIT_DOORBELL
    assert n.doorbell_detected_at is None
    n.step()
    assert n.state == n.STATE_WAIT_DOORBELL
    send_tone(n, clock)
    n.step()
    assert n.state == n.STATE_NAV_TO_DOOR


def test_timeout_ignores_late_audio_and_reset_discards_partial_tone(reception):
    n, clock = reception
    n._prepare_door_arrival()
    send_tone(n, clock, 0.08)
    assert n.doorbell_detected_at is None
    clock[0] = 12.0
    send_tone(n, clock)  # callback must respect deadline even before step polls
    assert n.doorbell_detected_at is None
    n.step()
    n._prepare_door_arrival()
    send_tone(n, clock, 0.08)
    assert n.doorbell_detected_at is None
