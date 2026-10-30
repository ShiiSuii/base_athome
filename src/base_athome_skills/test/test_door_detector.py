import math

from base_athome_skills.door_detector import DoorDetectorConfig, DoorOpenDetector, window_distance

# Geometria tipo Hokuyo UST-05LX: 270 grados, 0.25 grados por rayo
ANGLE_MIN = math.radians(-135)
INC = math.radians(0.25)
N = 1081


def scan(front_m, side_m=3.0, front_deg=10.0, noise=None):
    ranges = []
    for i in range(N):
        a = math.degrees(ANGLE_MIN + i * INC)
        r = front_m if abs(a) <= front_deg else side_m
        ranges.append(noise(i, r) if noise else r)
    return ranges


def dist(ranges, cfg=DoorDetectorConfig()):
    return window_distance(ranges, ANGLE_MIN, INC, 0.06, 5.0, cfg)


def run(detector, frames):
    return [detector.update(dist(f)) for f in frames]


def test_window_median_uses_only_front_rays():
    assert dist(scan(0.8)) == 0.8


def test_infinite_ranges_count_as_far():
    assert dist(scan(float('inf'))) == 5.0


def test_too_few_valid_rays_returns_none():
    assert dist(scan(float('nan'))) is None


def test_door_opening_triggers_open_after_consecutive_scans():
    det = DoorOpenDetector()
    states = run(det, [scan(0.8)] * 10)
    assert states[-1] == 'WAITING_CLOSED' and abs(det.reference_m - 0.8) < 1e-9
    states = run(det, [scan(3.5)] * 4)
    assert states[-1] == 'WAITING_CLOSED'
    assert run(det, [scan(3.5)])[-1] == 'OPEN'


def test_transient_glitch_does_not_trigger():
    det = DoorOpenDetector()
    run(det, [scan(0.8)] * 10)
    frames = [scan(3.5)] * 3 + [scan(0.8)] + [scan(3.5)] * 3
    assert run(det, frames)[-1] == 'WAITING_CLOSED'


def test_person_passing_close_does_not_trigger():
    det = DoorOpenDetector()
    run(det, [scan(0.8)] * 10)
    assert run(det, [scan(0.5)] * 20)[-1] == 'WAITING_CLOSED'


def test_no_door_in_front_is_reported():
    det = DoorOpenDetector()
    assert run(det, [scan(4.0)] * 10)[-1] == 'NO_DOOR'


def test_open_is_terminal():
    det = DoorOpenDetector(DoorDetectorConfig(open_scans_required=1))
    run(det, [scan(0.8)] * 10 + [scan(3.0)])
    assert run(det, [scan(0.8)])[-1] == 'OPEN'
