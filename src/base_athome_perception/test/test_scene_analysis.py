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
