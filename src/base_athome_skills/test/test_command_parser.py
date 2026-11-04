"""Commander: pedidos en lenguaje natural -> plan (espanol e ingles)."""
import pytest

from base_athome_skills.mission.command_parser import compile_command
from base_athome_skills.mission.plan import parse_plan, validate_plan
from base_athome_skills.mission.skills import default_registry

PLACES = {k: (0.0, 0.0, 0.0) for k in ("reception", "table", "kitchen", "home", "door", "living_room", "A", "B")}
REG = default_registry()


def skills(res):
    return [(s["skill"], s["args"]) for s in res.plan["steps"]]


def names(res):
    return [s["skill"] for s in res.plan["steps"]]


def test_user_example_full_mission_spanish():
    r = compile_command("Andá a recepción, saludá a la persona, después dirigite a la mesa, buscá una botella y llevala a la cocina.", PLACES)
    assert r.ok, r.errors
    assert names(r) == ["navigate_to", "speak", "navigate_to", "arm_pose", "detect_object", "pick_object", "navigate_to", "place_object"]
    s = skills(r)
    assert s[0][1] == {"place": "reception"} and s[2][1] == {"place": "table"}
    assert s[4][1] == {"object": "bottle"} and s[5][1] == {"object": "bottle"}
    assert s[6][1] == {"place": "kitchen"} and s[7][1] == {"place": "kitchen"}
    assert validate_plan(parse_plan(r.plan), REG, PLACES) == []
    assert r.plan["success_if"] == r.plan["steps"][-1]["id"]


def test_user_example_points_a_b_with_ellipsis():
    r = compile_command("Primero andá al punto A, después al B y esperá 3 segundos", PLACES)
    assert r.ok, r.errors
    assert skills(r) == [("navigate_to", {"place": "A"}), ("navigate_to", {"place": "B"}), ("wait", {"seconds": 3.0})]


def test_english_command():
    r = compile_command("Go to the kitchen, find a bottle, pick it up, then go to the table and put it down. Return home.".replace(". ", ", "), PLACES)
    assert r.ok, r.errors
    assert names(r) == ["navigate_to", "arm_pose", "detect_object", "pick_object", "navigate_to", "place_object", "return_home"]


def test_gpsr_style_bring_it():
    r = compile_command("go to the kitchen, find a bottle and bring it to the table", PLACES)
    assert r.ok, r.errors
    assert names(r) == ["navigate_to", "arm_pose", "detect_object", "pick_object", "navigate_to", "place_object"]


def test_carry_named_object_adds_implicit_steps():
    r = compile_command("llevá la lata a la cocina", PLACES)
    assert r.ok, r.errors
    assert names(r) == ["arm_pose", "detect_object", "pick_object", "navigate_to", "place_object"]
    assert skills(r)[1][1] == {"object": "can"}


def test_say_with_and_inside_quotes():
    r = compile_command('decí "hola y chau" y volvé a casa', PLACES)
    assert r.ok, r.errors
    assert skills(r) == [("speak", {"text": "hola y chau"}), ("return_home", {})]


def test_unknown_place_is_an_error_and_lists_known():
    r = compile_command("andá al garaje", PLACES)
    assert not r.ok and "garaje" in r.errors[0] and "kitchen" in r.errors[0]
    assert r.plan is None


def test_gibberish_is_rejected_never_guessed():
    r = compile_command("andá a la cocina y hacé una voltereta", PLACES)
    assert not r.ok and any("voltereta" in e for e in r.errors)


def test_place_without_object_rejected():
    assert not compile_command("dejá el objeto", PLACES).ok
    assert not compile_command("agarrala", PLACES).ok


def test_cannot_pick_twice():
    r = compile_command("agarrá la botella y agarrá la lata", PLACES)
    assert not r.ok and any("ya tengo" in e for e in r.errors)


@pytest.mark.parametrize("cmd,sec", [("esperá 10 segundos", 10.0), ("wait 2 minutes", 120.0), ("espera dos segundos", 2.0)])
def test_wait(cmd, sec):
    r = compile_command(cmd, PLACES)
    assert r.ok and r.plan["steps"][0]["args"]["seconds"] == sec


def test_no_home_means_recovery_only_speaks():
    places = {"kitchen": (0, 0, 0)}
    r = compile_command("andá a la cocina", places)
    assert r.ok and [s["skill"] for s in r.plan["recovery"]] == ["speak"]


def test_empty():
    assert not compile_command("  ", PLACES).ok


def test_take_x_to_place_is_carry_not_object_kitchen():
    r = compile_command("take the bottle to the kitchen", PLACES)
    assert r.ok, r.errors
    assert skills(r)[1][1] == {"object": "bottle"} and skills(r)[-1] == ("place_object", {"place": "kitchen"})


def test_person_requests_are_rejected_honestly():
    for t in ("buscá a la persona", "find the guest", "llevá al invitado a la cocina"):
        r = compile_command(t, PLACES)
        assert not r.ok and "no esta implementado" in r.errors[0], t


def test_pronouns_use_context_or_fail():
    assert not compile_command("agarrá eso", PLACES).ok
    r = compile_command("buscá una botella y agarrá eso", PLACES)
    assert r.ok and names(r) == ["arm_pose", "detect_object", "pick_object"] and skills(r)[2][1] == {"object": "bottle"}


def test_conditional_if_not_found():
    r = compile_command("andá a la cocina, buscá una taza. Si no la ves volvé a casa", PLACES)
    assert r.ok, r.errors
    st = r.plan["steps"]
    det = next(s for s in st if s["skill"] == "detect_object")
    assert det["on_failure"] == "continue"
    assert st[-1]["skill"] == "return_home" and st[-1]["if"] == {"step": det["id"], "state": "FAILED"}
    assert r.plan["success_if"] == det["id"]         # no verla = mision NO cumplida, aunque vuelva a casa
    assert validate_plan(parse_plan(r.plan), REG, PLACES) == []


def test_steps_after_conditional_require_the_object():
    r = compile_command("buscá una botella, si no la ves decí \"no hay\", agarrala y llevala a la mesa", PLACES)
    assert r.ok, r.errors
    st = r.plan["steps"]
    det = next(s for s in st if s["skill"] == "detect_object")
    pick = next(s for s in st if s["skill"] == "pick_object")
    assert pick["if"] == {"step": det["id"], "state": "SUCCEEDED"}
    assert next(s for s in st if s["skill"] == "speak")["if"]["state"] == "FAILED"
    assert validate_plan(parse_plan(r.plan), REG, PLACES) == []


def test_named_place_navigates_first():
    r = compile_command("buscá una botella en la cocina", PLACES)
    assert names(r) == ["navigate_to", "arm_pose", "detect_object"] and skills(r)[0][1] == {"place": "kitchen"}
    r = compile_command("agarrá la botella de la mesa", PLACES)
    assert names(r)[0] == "navigate_to" and skills(r)[0][1] == {"place": "table"}
    assert not compile_command("buscá una botella en el garaje", PLACES).ok
    r = compile_command("buscá la bolsa de papas", PLACES)     # "de" que no es lugar
    assert r.ok and names(r)[0] == "arm_pose"


def test_put_x_on_y_is_carry():
    r = compile_command("poné la botella en la mesa", PLACES)
    assert r.ok, r.errors
    assert names(r) == ["arm_pose", "detect_object", "pick_object", "navigate_to", "place_object"]


def test_filler_and_tell_person():
    assert compile_command("Robot, por favor andá a la cocina", PLACES).ok
    r = compile_command("dile a la persona que venga", PLACES)
    assert skills(r) == [("speak", {"text": "venga"})]


@pytest.mark.parametrize("t", ["stop", "cancelá", "pará", "Robot, cancelá la misión", "alto"])
def test_cancel(t):
    r = compile_command(t, PLACES)
    assert r.cancel and not r.ok


def test_redetect_after_moving():
    r = compile_command("buscá una botella, andá a la mesa y agarrala", PLACES)
    assert r.ok, r.errors
    assert names(r) == ["arm_pose", "detect_object", "navigate_to", "arm_pose", "detect_object", "pick_object"]


def test_follow_person_is_explicitly_unimplemented():
    c = compile_command("segui a la persona", PLACES)
    assert not c.ok and "no esta implementado" in c.errors[0]


def test_plan_text_shows_the_condition():
    c = compile_command("anda a la mesa, busca una botella, si no la ves deci no esta", PLACES)
    assert c.ok and c.steps_text[-1].startswith("si no lo encuentra:")
