from base_athome_perception import door_open as d


def routine(fail=None):
    calls = []

    def go(name):
        calls.append(name)
        return (name != fail), ("ok" if name != fail else "sin plan")
    return d.DoorRoutine(go, lambda mm: calls.append(("grip", mm)), lambda s: None, d.clamp_params(None)), calls


def test_params_are_limited_and_keep_defaults():
    p = d.clamp_params(dict(speed_pct=100, grip_mm="x", pause_s=-3))
    assert p == dict(speed_pct=40.0, grip_mm=d.DEFAULTS["grip_mm"], pause_s=0.0)


def test_params_round_trip(tmp_path):
    path = tmp_path / "door.yaml"
    assert d.load_params(path) == d.DEFAULTS
    d.save_params(path, dict(speed_pct=20, grip_mm=30, pause_s=2))
    assert d.load_params(path) == dict(speed_pct=20.0, grip_mm=30.0, pause_s=2.0)


def test_routine_closes_on_the_handle_and_goes_in_order():
    r, calls = routine()
    assert r.run_all()[0] and r.finished
    assert calls == ["door_pre", "door_handle", ("grip", 25.0), "door_press", "door_swing", "door_release"]


def test_failure_stops_and_stays_on_that_step():
    r, calls = routine(fail="door_press")
    ok, message = r.run_all()
    assert not ok and "sin plan" in message
    assert d.STEPS[r.index].pose == "door_press" and "door_swing" not in calls


def test_abort_stops_before_the_next_move():
    r, calls = routine()
    r.run_step()
    r.abort = True
    assert not r.run_all()[0] and calls == ["door_pre"]


def test_missing_poses_are_reported():
    assert d.DoorRoutine.missing({"door_pre": {}}) == ["door_handle", "door_press", "door_swing", "door_release"]
