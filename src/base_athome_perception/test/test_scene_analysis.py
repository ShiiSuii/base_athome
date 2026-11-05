"""Analisis de escena por barrido del pan-tilt."""
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from base_athome_perception import scene_analysis as sa  # noqa: E402
from base_athome_perception.scene_analysis import SeenObject  # noqa: E402

FX, CX = 600.0, 320.0


def test_pixel_bearing_combines_head_and_pixel():
    assert sa.pixel_bearing(320, 30.0, FX, CX) == pytest.approx(30.0)
    assert sa.pixel_bearing(920, 0.0, FX, CX) == pytest.approx(-45.0)


def test_occupants_counts_lower_body_on_furniture():
    chair = (200, 250, 400, 470)
    sitting = (220, 100, 380, 460)
    standing_far = (450, 50, 560, 470)
    assert sa.occupants_in_image(chair, [sitting, standing_far]) == 1


def test_merge_same_couch_seen_from_two_angles():
    a = SeenObject('couch', 20.0, 30.0, occupants=1)
    b = SeenObject('couch', 26.0, 30.0, occupants=0)
    c = SeenObject('chair', -40.0, 10.0)
    merged = sa.merge_views([a, b, c])
    assert len(merged) == 2
    couch = next(o for o in merged if o.kind == 'couch')
    assert couch.occupants == 1 and couch.views == 2


def test_choose_seat_prefers_free_and_frontal():
    objs = [SeenObject('chair', 50.0, 10.0), SeenObject('couch', -10.0, 30.0, occupants=2),
            SeenObject('chair', -20.0, 10.0)]
    assert sa.choose_seat(objs).bearing_deg == -20.0
    assert sa.choose_seat([SeenObject('chair', 0.0, 10.0, occupants=1)]) is None


def test_describe_scene_mentions_host_and_free_seat_direction():
    objs = [SeenObject('person', 30.0, 10.0, seated=True, name='Charlie'), SeenObject('chair', -25.0, 10.0)]
    text = sa.describe_scene(objs, 'Robin', sa.choose_seat(objs))
    assert 'Charlie is sitting on your left' in text
    assert 'Robin, the chair on your right is free' in text


def test_person_seated_at_requires_sitting_posture_near_bearing():
    assert sa.person_seated_at(-20.0, [-18.0], ['SENTADO'])
    assert not sa.person_seated_at(-20.0, [-18.0], ['DE_PIE'])
    assert not sa.person_seated_at(-20.0, [30.0], ['SENTADO'])


def test_face_tracking_centers_face_with_deadband_and_limit():
    dpan, dtilt = sa.face_tracking_step(320, 240, 640, 480, FX, FX)
    assert (dpan, dtilt) == (0.0, 0.0)
    dpan, _ = sa.face_tracking_step(100, 240, 640, 480, FX, FX)  # cara a la izquierda -> pan +
    assert 0 < dpan <= 8.0
    dpan, dtilt = sa.face_tracking_step(640, 0, 640, 480, FX, FX)  # derecha y arriba
    assert dpan < 0 and dtilt > 0


def test_scan_plan_two_rows_zigzag_and_low_row():
    from base_athome_perception.scene_analysis import scan_plan
    plan = scan_plan((-65.0, 0.0, 65.0), (-12.0, -28.0))
    assert plan == [(-65.0, -12.0), (0.0, -12.0), (65.0, -12.0), (65.0, -28.0), (0.0, -28.0), (-65.0, -28.0)]
    assert min(t for _, t in scan_plan()) <= -25.0       # fila baja para personas sentadas


def test_head_steps_limit_speed():
    from base_athome_perception.scene_analysis import head_steps
    steps = head_steps((0.0, 0.0), (25.0, -10.0), speed_dps=25.0, rate_hz=20.0)
    assert len(steps) == 20                               # 25 grados a 25 grados/s = 1 s = 20 pasos
    assert steps[-1] == (25.0, -10.0)
    assert max(abs(b[0] - a[0]) for a, b in zip([(0.0, 0.0)] + steps, steps)) <= 25.0 / 20 + 1e-9


def test_seated_person_seen_in_low_row_is_merged_as_seated():
    from base_athome_perception.scene_analysis import SeenObject, merge_views
    standing_row = SeenObject("person", 20.0, 10.0, seated=False)
    low_row = SeenObject("person", 22.0, 12.0, seated=True)
    merged = merge_views([standing_row, low_row])
    assert len(merged) == 1 and merged[0].seated


def test_gentle_tracker_looks_up_at_tall_person_in_small_steps():
    from base_athome_perception.scene_analysis import GentleFaceTracker
    tr = GentleFaceTracker()
    pan, tilt, t = 0.0, 0.0, 0.0
    # cara de una persona alta: arriba en la imagen (y=60 de 480), centrada en x
    for _ in range(40):
        out = tr.update(320, 60, 640, 480, 600.0, 600.0, pan, tilt, t)
        if out:
            assert abs(out[1] - tilt) <= 6.0 + 1e-9      # pasos limitados
            pan, tilt = out
        t += 0.1
    assert tilt > 5.0 and tilt <= 25.0                    # subio, sin pasar el limite
    assert abs(pan) < 1e-6                                # cara centrada en x: no gira


def test_gentle_tracker_ignores_small_errors_and_limits_rate():
    from base_athome_perception.scene_analysis import GentleFaceTracker
    tr = GentleFaceTracker()
    assert tr.update(330, 250, 640, 480, 600.0, 600.0, 0.0, 0.0, 0.0) is None   # dentro de la zona muerta
    tr2 = GentleFaceTracker()
    first = tr2.update(320, 40, 640, 480, 600.0, 600.0, 0.0, 0.0, 0.0)
    assert first is not None
    assert tr2.update(320, 40, 640, 480, 600.0, 600.0, first[0], first[1], 0.1) is None  # < 0,12 s: no se mueve


def test_gentle_tracker_respects_limits():
    from base_athome_perception.scene_analysis import GentleFaceTracker
    tr = GentleFaceTracker(min_interval_s=0.0)
    pan, tilt = 0.0, 0.0
    for i in range(200):
        out = tr.update(0, 0, 640, 480, 600.0, 600.0, pan, tilt, float(i))
        if out:
            pan, tilt = out
    assert pan <= 40.0 + 1e-9 and tilt <= 25.0 + 1e-9


def _track(amp=14.0, period=4.0, offset=0.0, noise_px=0.0, tilt_face=0.0, seconds=40.0):
    """Lazo cerrado: la cara se mueve, la camara ve con retardo, la cabeza sigue con su propia dinamica."""
    import math
    import random
    rng = random.Random(1)
    w, h, fx, dt = 640, 480, 615.0, 0.01
    tracker = sa.GentleFaceTracker()
    pan = tilt = target_pan = target_tilt = 0.0
    queue, errs, moves, next_det, t = [], [], 0, 0.0, 0.0
    while t < seconds:
        face = offset + amp * math.sin(2 * math.pi * t / period)
        pan += (target_pan - pan) * min(1.0, dt / 0.08)
        tilt += (target_tilt - tilt) * min(1.0, dt / 0.08)
        if t >= next_det:
            next_det = t + 0.1
            u = w / 2 - fx * math.tan(math.radians(face - pan)) + rng.gauss(0, noise_px)
            v = h / 2 - fx * math.tan(math.radians(tilt_face - tilt)) + rng.gauss(0, noise_px)
            queue.append((t + 0.15, u, v))
        while queue and queue[0][0] <= t:
            _, u, v = queue.pop(0)
            out = tracker.update(u, v, w, h, fx, fx, target_pan, target_tilt, t)
            if out:
                target_pan, target_tilt = out
                moves += 1
        if t > 10:
            errs.append(abs(face - pan))
        t += dt
    return max(errs), moves, tilt


def test_tracker_goes_toward_a_still_face_and_stops():
    worst, moves, tilt = _track(amp=0.0, offset=20.0, tilt_face=15.0)
    assert worst <= 4.0 and abs(tilt - 15.0) <= 4.0 and moves < 30


def test_tracker_keeps_up_with_a_guest_pacing_side_to_side():
    worst, _, _ = _track(amp=14.0, period=4.0)
    assert worst <= 12.0                                 # antes: 16.6 con los valores del seguimiento lento


def test_tracker_does_not_chase_detection_noise():
    _, moves, _ = _track(amp=0.0, noise_px=6.0)
    assert moves < 10                                    # con ruido normal de la deteccion la cabeza queda quieta
