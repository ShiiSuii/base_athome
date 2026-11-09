"""Mirar y decir lo que ve: frases, plan sin recuperacion y sin pedir "yes"."""
from base_athome_skills.mission.command_parser import compile_command, compile_gpsr_actions
from base_athome_skills.mission.skills import default_registry
from base_athome_skills.mission.voice_dialog import needs_confirmation, plan_to_speech

PLACES = {"kitchen": dict(name="kitchen", x=1.0, y=0.0, yaw_deg=0), "home": dict(name="home", x=0.0, y=0.0, yaw_deg=0)}


def skills(plan):
    return [s["skill"] for s in plan["steps"]]


def test_look_phrases_in_english_and_spanish():
    for text in ("what do you see?", "Can you describe the room", "look around", "¿Qué ves?", "describí lo que ves",
                 "robot, tell me what do you see"):
        c = compile_command(text, PLACES)
        assert c.ok, (text, c.errors)
        assert skills(c.plan) == ["describe_view"], text
        assert "recovery" not in c.plan, text


def test_how_many_asks_for_a_number():
    c = compile_command("how many cups do you see", PLACES)
    assert c.ok and c.plan["steps"][0]["args"]["question"].startswith("How many cups")


def test_looking_does_not_need_yes_but_moving_does():
    reg = default_registry()
    assert not needs_confirmation(compile_command("what do you see", PLACES).plan, reg)
    assert needs_confirmation(compile_command("go to the kitchen", PLACES).plan, reg)
    assert needs_confirmation({"steps": [{"skill": "no_existe"}]}, reg)
    assert "tell you what I see" in plan_to_speech(compile_command("what do you see", PLACES).plan)


def test_official_count_and_describe_go_first_and_never_name_people():
    c = compile_gpsr_actions([dict(action="go_to", params=dict(location="kitchen")),
                              dict(action="count", params=dict(target="snacks", location="kitchen"))], PLACES)
    assert c.ok and skills(c.plan) == ["navigate_to", "describe_view"]
    c = compile_gpsr_actions([dict(action="describe", params=dict(target="person", property="name"))], PLACES)
    assert not c.ok


def test_dock_phrases_move_and_ask_for_yes():
    reg = default_registry()
    for text, target in (("approach the wall", "wall"), ("acercate a la puerta", "door"), ("get close to the table", "table")):
        c = compile_command(text, PLACES)
        assert c.ok and c.plan["steps"][0]["skill"] == "dock_front", text
        assert c.plan["steps"][0]["args"]["target"] == target
        assert needs_confirmation(c.plan, reg)
