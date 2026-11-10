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


def test_raised_frames_and_bearing_median_per_person():
    t = PeopleTally()
    t.add([1.0, 0.0], raised=True, bearing_deg=10.0)
    t.add([1.05, 0.0], raised=True, bearing_deg=12.0)
    t.add([1.0, 0.02], raised=False, bearing_deg=40.0)
    t.add([0.0, 2.0], raised=False, bearing_deg=math.nan)
    t.add([0.0, 2.0])
    near, far = t.result((0.0, 0.0), min_hits=2)
    assert near["raised_frames"] == 2 and near["bearing_deg"] == 12.0
    assert far["raised_frames"] == 0 and "bearing_deg" not in far


def test_heading_is_a_circular_mean_across_plus_minus_180():
    t = PeopleTally()
    t.add([1.0, 0.0], heading_deg=179.0)
    t.add([1.0, 0.0], heading_deg=-179.0)
    (p,) = t.result((0.0, 0.0), min_hits=2)
    assert abs(abs(p["heading_deg"]) - 180.0) < 1e-6


def test_continuous_scan_can_cover_two_spins_and_still_has_a_finite_cap():
    """Exercise the adapter with a fake clock and no frames; no ROS or camera."""
    import ast
    from pathlib import Path
    from types import SimpleNamespace as NS
    script = Path(__file__).parents[1] / 'scripts/bag_finder.py'
    cls = next(n for n in ast.parse(script.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'BagFinder')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_people_here')
    for requested, timeout, expected, stop_after in [
            (100., 102., 100., None), (500., 500., 115., None),
            (2., 5., 2., None), (100., 102., 2., 2.)]:
        now = [100.]
        def sleep(seconds):
            now[0] += seconds
            if stop_after is not None and now[0] >= 100. + stop_after:
                node.scan_stop_id = 'scan'
        env = dict(time=NS(time=lambda: now[0], sleep=sleep), PeopleTally=PeopleTally,
                   RAISED_HANDS_SETTLE_S=1.5, SEARCH_TILT=0.)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(script), 'exec'), env)
        node = NS(head_stamp=100., rgb_stamp=None, _ensure_torque=lambda: None,
                  _move_head=lambda *_: None, _head_still=lambda: True,
                  _frames=lambda **_: (None, None, None), _robot_pos=lambda: (0., 0.),
                  get_logger=lambda: NS(info=lambda _: None))
        result = env['_people_here'](node, {'id': 'scan', 'view_s': requested, 'timeout_s': timeout})
        assert result['ok'] and result['customers'] == []
        assert expected <= now[0] - 100. < expected + .1
        assert result['diagnostics'].get('stopped', False) == (stop_after is not None)
