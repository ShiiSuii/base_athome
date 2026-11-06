"""Tests del nucleo de misiones (sin ROS): registro, validacion, ejecucion, reintentos, timeouts,
cancelacion y recuperacion."""
import threading
import time

import pytest

from base_athome_skills.mission import executor as ex
from base_athome_skills.mission.executor import MissionExecutor
from base_athome_skills.mission.places import load_places, resolve
from base_athome_skills.mission.plan import PlanError, from_gpsr, parse_plan, validate_plan
from base_athome_skills.mission.skills import SkillResult, default_registry

REG = default_registry()
PLACES = {"door": (1.0, 0.0, 0.0), "kitchen": (2.0, 1.0, 0.0), "living_room": (0.0, 2.0, 1.57),
          "home": (0.0, 0.0, 0.0)}


def ok_handler(log):
    def h(ctx):
        log.append(("run", ctx.args, ctx.attempt))
        return SkillResult.success("ok")
    return h


def plan(*steps, name="t"):
    return parse_plan({"name": name, "steps": list(steps)})


# ------------------------------------------------------------------ registro / validacion
def test_registry_marks_unimplemented_skills_unavailable():
    assert {"navigate_to", "navigate_sequence", "speak", "listen", "wait", "return_home"} <= set(REG.available())
    for name in ("find_person", "guide_person", "deliver_object", "answer_question"):
        assert REG.get(name).status == "unavailable"
    for name in ("detect_object", "pick_object", "place_object", "arm_pose"):   # SIM_PASS (HOME-47)
        assert REG.get(name).status == "available"


def test_validate_rejects_unavailable_unknown_and_bad_args():
    p = plan({"skill": "answer_question", "args": {}},
             {"skill": "fly", "args": {}},
             {"skill": "navigate_to", "args": {}},
             {"skill": "speak", "args": {"text": "hi", "volume": 3}})
    errs = validate_plan(p, REG, PLACES)
    assert any("answer_question" in e and "no esta implementada" in e for e in errs)
    assert any("desconocida 'fly'" in e for e in errs)
    assert any("faltan argumentos ['place']" in e for e in errs)
    assert any("desconocidos ['volume']" in e for e in errs)


def test_validate_checks_places_and_recovery_targets():
    p = plan({"id": "a", "skill": "navigate_to", "args": {"place": "garage"}, "on_failure": "nope"},
             {"id": "b", "skill": "navigate_sequence", "args": {"places": ["door", "attic"]}})
    errs = validate_plan(p, REG, PLACES)
    assert any("lugar desconocido 'garage'" in e for e in errs)
    assert any("lugar desconocido 'attic'" in e for e in errs)
    assert any("on_failure 'nope'" in e for e in errs)


def test_validate_accepts_demo_plan_and_place_normalization():
    p = plan({"skill": "navigate_to", "args": {"place": "Living Room"}},
             {"skill": "speak", "args": {"text": "hello"}},
             {"skill": "navigate_sequence", "args": {"places": ["door", "kitchen"]}},
             {"skill": "return_home"})
    assert validate_plan(p, REG, PLACES) == []
    assert resolve(PLACES, "Living Room") == PLACES["living_room"]


def test_parse_rejects_garbage():
    with pytest.raises(PlanError):
        parse_plan("[1, 2")
    with pytest.raises(PlanError):
        parse_plan({"steps": [{"args": {}}]})


def test_gpsr_conversion_keeps_unimplemented_steps_and_validation_rejects_them():
    actions = [{"action": "go_to", "params": {"location": "kitchen"}},
               {"action": "find_object", "params": {"object": "bottle"}},
               {"action": "pick", "params": {"object": "bottle"}},
               {"action": "say", "params": {"text": "done"}},
               {"action": "answer_question", "params": {}}]
    p = from_gpsr(actions)
    assert [s.skill for s in p.steps] == ["navigate_to", "detect_object", "pick_object", "speak", "answer_question"]
    errs = validate_plan(p, REG, PLACES)
    assert len([e for e in errs if "no esta implementada" in e]) == 1
    with pytest.raises(PlanError):
        from_gpsr([{"action": "teleport", "params": {}}])


def test_load_places_reads_waypoints_and_object_approach(tmp_path):
    sm = tmp_path / "semantic_map.yaml"
    sm.write_text("waypoints:\n  door: {x: 1.5, y: -0.5, yaw_deg: 90}\n"
                  "objects:\n  obj_1: {name: pringles, approach: {x: 2.0, y: 1.0, yaw_deg: 0}}\n")
    wp = tmp_path / "waypoints.yaml"
    wp.write_text("waypoints:\n  door: {x: 9, y: 9}\n  kitchen: {x: 3.0, y: 4.0, qz: 0.7071068, qw: 0.7071068}\n")
    places = load_places(str(sm), str(wp))
    assert places["door"] == pytest.approx((1.5, -0.5, 1.5707963))   # semantic_map manda
    assert places["kitchen"] == pytest.approx((3.0, 4.0, 1.5707963))
    assert places["object:obj_1"] == pytest.approx((2.0, 1.0, 0.0))


# ------------------------------------------------------------------ ejecucion
def test_runs_steps_in_order_and_succeeds():
    calls = []
    hs = {k: ok_handler(calls) for k in ("navigate_to", "speak")}
    updates = []
    e = MissionExecutor(REG, hs, on_update=updates.append)
    res = e.run(plan({"id": "a", "skill": "navigate_to", "args": {"place": "door"}},
                     {"id": "b", "skill": "speak", "args": {"text": "hi"}}))
    assert res["state"] == "SUCCEEDED"
    assert [c[1] for c in calls] == [{"place": "door"}, {"text": "hi"}]
    assert [s["state"] for s in res["steps"]] == ["SUCCEEDED", "SUCCEEDED"]
    assert all(s["duration_s"] is not None for s in res["steps"])
    assert updates and updates[-1]["state"] == "SUCCEEDED"


def test_failure_stops_mission_by_default():
    calls = []
    hs = {"navigate_to": lambda ctx: SkillResult.failure("nav_failed", "bloqueado"), "speak": ok_handler(calls)}
    res = MissionExecutor(REG, hs).run(plan({"id": "a", "skill": "navigate_to", "args": {"place": "door"}},
                                            {"id": "b", "skill": "speak", "args": {"text": "hi"}}))
    assert res["state"] == "FAILED"
    assert res["steps"][0]["error"] == "nav_failed"
    assert res["steps"][1]["state"] == "SKIPPED"
    assert calls == []


def test_retries_then_success():
    n = {"c": 0}

    def flaky(ctx):
        n["c"] += 1
        return SkillResult.success() if n["c"] == 3 else SkillResult.failure("nav_failed")
    res = MissionExecutor(REG, {"navigate_to": flaky}).run(
        plan({"skill": "navigate_to", "args": {"place": "door"}, "retries": 2}))
    assert res["state"] == "SUCCEEDED" and res["steps"][0]["attempts"] == 3


def test_continue_policy_tolerates_failure():
    calls = []
    hs = {"listen": lambda ctx: SkillResult.failure("no_speech"), "speak": ok_handler(calls)}
    res = MissionExecutor(REG, hs).run(plan({"id": "l", "skill": "listen", "on_failure": "continue"},
                                            {"id": "s", "skill": "speak", "args": {"text": "ok"}}))
    assert res["state"] == "SUCCEEDED" and "tolerados" in res["message"]
    assert len(calls) == 1


def test_forward_recovery_runs_recovery_but_reports_failed():
    calls = []
    hs = {"navigate_to": lambda ctx: SkillResult.failure("nav_failed"), "speak": ok_handler(calls),
          "return_home": ok_handler(calls)}
    res = MissionExecutor(REG, hs).run(plan(
        {"id": "go", "skill": "navigate_to", "args": {"place": "kitchen"}, "on_failure": "sorry"},
        {"id": "talk", "skill": "speak", "args": {"text": "arrived"}},
        {"id": "sorry", "skill": "speak", "args": {"text": "I could not get there"}},
        {"id": "home", "skill": "return_home"}))
    assert res["state"] == "FAILED" and "recuperacion" in res["message"]
    assert [s["state"] for s in res["steps"]] == ["FAILED", "SKIPPED", "SUCCEEDED", "SUCCEEDED"]
    assert [c[1].get("text") for c in calls] == ["I could not get there", None]


def test_backward_recovery_can_succeed():
    n = {"c": 0}

    def nav(ctx):
        n["c"] += 1
        return SkillResult.success() if ctx.args["place"] == "door" or n["c"] > 2 else SkillResult.failure("nav_failed")
    res = MissionExecutor(REG, {"navigate_to": nav}).run(plan(
        {"id": "back", "skill": "navigate_to", "args": {"place": "door"}},
        {"id": "go", "skill": "navigate_to", "args": {"place": "kitchen"}, "on_failure": "back"}))
    assert res["state"] == "SUCCEEDED"


def test_recovery_loop_is_bounded():
    hs = {"navigate_to": lambda ctx: SkillResult.failure("nav_failed"), "wait": lambda ctx: SkillResult.success()}
    res = MissionExecutor(REG, hs).run(plan(
        {"id": "w", "skill": "wait", "args": {"seconds": 0}},
        {"id": "go", "skill": "navigate_to", "args": {"place": "kitchen"}, "on_failure": "w"}))
    assert res["state"] == "FAILED" and "demasiadas recuperaciones" in res["message"]


def test_timeout_cancels_handler(monkeypatch):
    monkeypatch.setattr(ex, "CANCEL_GRACE_S", 1.0)
    seen = {}

    def slow(ctx):
        seen["canceled"] = ctx.cancel.wait(5.0)
        return SkillResult.failure("canceled")
    t0 = time.time()
    res = MissionExecutor(REG, {"wait": slow}).run(plan({"skill": "wait", "args": {"seconds": 5}, "timeout_s": 0.2}))
    assert res["state"] == "FAILED" and res["steps"][0]["error"] == "timeout"
    assert seen["canceled"] is True and time.time() - t0 < 2.0


def test_exception_in_handler_is_reported_not_swallowed():
    def boom(ctx):
        raise RuntimeError("se rompio")
    res = MissionExecutor(REG, {"speak": boom}).run(plan({"skill": "speak", "args": {"text": "x"}}))
    assert res["state"] == "FAILED" and res["steps"][0]["error"] == "exception"
    assert "se rompio" in res["steps"][0]["message"]


def test_cancel_stops_current_step_and_never_starts_next():
    started = threading.Event()
    calls = []

    def long_wait(ctx):
        started.set()
        return SkillResult.success() if ctx.sleep(10) else SkillResult.failure("canceled", "cancelado")
    e = MissionExecutor(REG, {"wait": long_wait, "speak": ok_handler(calls)})
    th = threading.Thread(target=lambda: e.run(plan({"id": "w", "skill": "wait", "args": {"seconds": 10}},
                                                    {"id": "s", "skill": "speak", "args": {"text": "x"}})))
    th.start()
    assert started.wait(2.0)
    assert e.cancel() is True
    th.join(3.0)
    res = e.snapshot()
    assert res["state"] == "CANCELED"
    assert [s["state"] for s in res["steps"]] == ["CANCELED", "SKIPPED"]
    assert calls == []
    assert e.cancel() is False   # nada que cancelar


def test_second_mission_rejected_while_running():
    started = threading.Event()

    def w(ctx):
        started.set()
        ctx.sleep(5)
        return SkillResult.failure("canceled")
    e = MissionExecutor(REG, {"wait": w})
    th = threading.Thread(target=lambda: e.run(plan({"skill": "wait", "args": {"seconds": 5}})))
    th.start()
    assert started.wait(2.0)
    with pytest.raises(RuntimeError):
        e.run(plan({"skill": "wait", "args": {"seconds": 1}}))
    e.cancel()
    th.join(3.0)


def test_recovery_section_runs_only_on_failure():
    calls = []
    hs = {"speak": ok_handler(calls), "navigate_to": lambda ctx: SkillResult.success()}
    data = {"name": "r", "steps": [{"id": "go", "skill": "navigate_to", "args": {"place": "door"}, "on_failure": "sorry"},
                                   {"id": "done", "skill": "speak", "args": {"text": "done"}}],
            "recovery": [{"id": "sorry", "skill": "speak", "args": {"text": "sorry"}}]}
    p = parse_plan(data)
    assert validate_plan(p, REG, PLACES) == []
    res = MissionExecutor(REG, hs).run(p)
    assert res["state"] == "SUCCEEDED"
    assert [c[1]["text"] for c in calls] == ["done"]
    assert [s["state"] for s in res["steps"]] == ["SUCCEEDED", "SUCCEEDED", "SKIPPED"]
    calls.clear()
    hs["navigate_to"] = lambda ctx: SkillResult.failure("nav_failed")
    res = MissionExecutor(REG, hs).run(parse_plan(data))
    assert res["state"] == "FAILED"
    assert [c[1]["text"] for c in calls] == ["sorry"]
    assert [s["state"] for s in res["steps"]] == ["FAILED", "SKIPPED", "SUCCEEDED"]


def test_demo_mission_file_is_valid_with_demo_places():
    import os
    path = os.path.join(os.path.dirname(__file__), "..", "missions", "demo_a_b_voice_c_home.yaml")
    p = parse_plan(open(path).read())
    assert validate_plan(p, REG, PLACES) == []
    assert [s.id for s in p.steps if s.recovery] == ["sorry"]


def test_node_preflight_refuses_domain_10_without_real_mode():
    node = pytest.importorskip("base_athome_skills.mission.node")
    assert node.preflight(["mission_executor"], {"ROS_DOMAIN_ID": "10"})
    assert node.preflight(["x", "--ros-args", "-p", "mode:=sim"], {"ROS_DOMAIN_ID": "10"})
    assert node.preflight(["x", "--ros-args", "-p", "mode:=real"], {"ROS_DOMAIN_ID": "10"}) is None
    assert node.preflight(["mission_executor"], {"ROS_DOMAIN_ID": "42"}) is None



def test_condition_branches_on_previous_step_state():
    calls = []
    hs = {"detect_object": lambda ctx: SkillResult.failure("not_found"), "speak": ok_handler(calls),
          "navigate_to": ok_handler(calls)}
    p = plan({"id": "look", "skill": "detect_object", "args": {"object": "bottle"}, "on_failure": "continue"},
             {"id": "found", "skill": "speak", "args": {"text": "found it"}, "if": {"step": "look", "state": "SUCCEEDED"}},
             {"id": "search", "skill": "navigate_to", "args": {"place": "kitchen"}, "if": {"step": "look", "state": "FAILED"}})
    assert validate_plan(p, REG, PLACES) == []
    res = MissionExecutor(REG, hs).run(p)
    assert [s["state"] for s in res["steps"]] == ["FAILED", "SKIPPED", "SUCCEEDED"]
    assert [c[1] for c in calls] == [{"place": "kitchen"}]
    assert "condicion no cumplida" in res["steps"][1]["message"]


def test_condition_must_reference_previous_step():
    p = plan({"id": "a", "skill": "speak", "args": {"text": "x"}, "if": {"step": "b"}},
             {"id": "b", "skill": "speak", "args": {"text": "y"}})
    assert any("paso ANTERIOR" in e for e in validate_plan(p, REG, PLACES))



def test_success_if_marks_mission_failed_when_goal_step_not_done():
    hs = {"detect_object": lambda ctx: SkillResult.failure("not_found"), "speak": lambda ctx: SkillResult.success()}
    p = parse_plan({"name": "g", "success_if": "say",
                    "steps": [{"id": "look", "skill": "detect_object", "args": {"object": "bottle"}, "on_failure": "continue"},
                              {"id": "say", "skill": "speak", "args": {"text": "got it"}, "if": {"step": "look"}}]})
    assert validate_plan(p, REG, PLACES) == []
    res = MissionExecutor(REG, hs).run(p)
    assert res["state"] == "FAILED" and "objetivo no cumplido" in res["message"]


def test_fetch_demo_valid_in_sim_map_and_rejected_in_empty_map():
    import os
    root = os.path.join(os.path.dirname(__file__), "..", "..")
    p = parse_plan(open(os.path.join(root, "base_athome_skills", "missions", "demo_fetch_bottle.yaml")).read())
    sim = load_places(os.path.join(root, "..", "simulation", "base_athome_sim", "config", "sim_semantic_map.yaml"))
    assert validate_plan(p, REG, sim) == []
    assert p.success_if == "put"
    assert any("reception" in e for e in validate_plan(p, REG, {"door": (0, 0, 0)}))
