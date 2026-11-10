"""PeopleTally: personas vistas desde un lugar, juntando cuadros por posicion."""

import math

from base_athome_perception.people_here import PeopleTally


def test_same_person_in_several_frames_is_one_and_sorted_by_distance():
    t = PeopleTally(match_dist_m=0.7)
    for xy in ([2.0, 0.0], [2.1, 0.05], [1.95, -0.02], [0.0, 1.0], [0.05, 1.02]):
        t.add(xy)
    out = t.result((0.0, 0.0), min_hits=2)
    assert [round(p["range_m"], 2) for p in out] == [1.01, 2.0]
    assert out[0]["frames"] == 2 and out[1]["frames"] == 3


def test_single_frame_false_positive_is_dropped_and_invalid_spots_ignored():
    t = PeopleTally()
    t.add([3.0, 3.0])
    for bad in (None, [math.nan, 1.0], [1.0], "xy"):
        t.add(bad)
    assert t.result((0.0, 0.0), min_hits=2) == []
    assert len(t.result((0.0, 0.0), min_hits=1)) == 1


def test_distance_is_from_the_robot_not_the_map_origin():
    t = PeopleTally()
    for _ in range(2):
        t.add([5.0, 0.0])
        t.add([1.0, 0.0])
    out = t.result((5.5, 0.0), min_hits=2)
    assert out[0]["xy"] == [5.0, 0.0] and math.isclose(out[0]["range_m"], 0.5)
