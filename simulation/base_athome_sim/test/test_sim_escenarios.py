"""Tests puros de la logica de evaluacion de escenarios de simulacion (HOME-75)."""
from pathlib import Path
import importlib.util
import sys

import pytest

SCRIPT_PATH = Path(__file__).resolve().parents[2] / "scripts" / "sim_escenarios.py"
spec = importlib.util.spec_from_file_location("sim_escenarios", SCRIPT_PATH)
se = importlib.util.module_from_spec(spec)
sys.modules["sim_escenarios"] = se
spec.loader.exec_module(se)


def test_all_scenarios_defined_with_expected_parameters():
    names = [s.name for s in se.SCENARIOS]
    assert names == [
        "normal", "lejos", "mueve", "distractor", "alta",
        "tilt3", "sin_gesto", "sin_persona", "apunta_mal", "plan_b", "plan_b_sin_manos",
    ]
    m = se.SCENARIO_MAP
    assert m["normal"].env["ATHOME_SIM_BAG"] == "1.5,1.2,0.30"
    assert m["normal"].env["ATHOME_SIM_PERSON"] == "2.6,0.3"
    assert m["normal"].max_cm == 4.0

    assert m["lejos"].env["ATHOME_SIM_PERSON"] == "5.0,0.6"
    assert m["lejos"].env["ATHOME_SIM_BAG"] == "4.0,1.3,0.30"
    assert m["lejos"].max_cm == 6.0

    assert m["mueve"].env["ATHOME_SIM_PERSON_SWAY_M"] == "0.25"
    assert m["distractor"].env["ATHOME_SIM_BAG2"] == "1.0,0.45,0.25"
    assert m["alta"].env["ATHOME_SIM_BAG"] == "1.5,1.2,0.45"
    assert m["tilt3"].env["ATHOME_SIM_WRIST_TILT_ERR_DEG"] == "3"

    assert m["sin_gesto"].env["ATHOME_SIM_PERSON_POINTS"] == "0"
    assert m["sin_gesto"].expected_state == "FAILED"
    assert m["sin_gesto"].expected_failed_step == "s3_find_pointed_bag"

    assert m["sin_persona"].env["ATHOME_SIM_PERSON"] == "-6,-6"
    assert m["sin_persona"].expected_state == "FAILED"
    assert m["sin_persona"].expected_failed_step == "s1_approach_person"

    assert m["apunta_mal"].env["ATHOME_SIM_BAG"] == "0.5,2.5,0.30"
    assert m["apunta_mal"].env["ATHOME_SIM_POINT_ERR_M"] == "1.0"
    assert m["apunta_mal"].expected_state == "FAILED"
    assert m["apunta_mal"].expected_failed_step == "s3_find_pointed_bag"


def test_parse_sim_log_extracts_first_grip_distance_cm():
    log = (
        "[INFO] [sim_robot]: Robot simulado listo\n"
        "[INFO] [1793920000.123] [sim_robot]: Pinza cerrada: TCP [0.51, 0.17, 0.29], objeto [0.52, 0.18, 0.3], a 1.6 cm\n"
        "[INFO] [1793920050.456] [sim_robot]: Pinza cerrada: TCP [0.50, 0.17, 0.29], objeto [0.80, 0.18, 0.3], a 28.4 cm\n"
    )
    assert se.parse_sim_log(log) == pytest.approx(1.6)
    assert se.parse_sim_log("log sin cierre de pinza") is None


def test_parse_client_output_extracts_steps_and_final_state():
    out = (
        "\n=== take out the trash\n"
        "  1. buscar a la persona\n"
        "  [  3.2s] s1_approach_person: SUCCEEDED la persona ya esta cerca\n"
        "  [  5.0s] s2_speak: SUCCEEDED dicho (simulado)\n"
        "  [ 21.4s] s3_find_pointed_bag: FAILED no vi a nadie senalando\n"
        "  [ 23.1s] avisar: SUCCEEDED dicho (simulado)\n"
        "  FIN: FAILED fallo s3_find_pointed_bag (ejecutada recuperacion 'avisar')\n"
    )
    state, msg, steps = se.parse_client_output(out)
    assert state == "FAILED"
    assert "s3_find_pointed_bag" in msg
    assert steps == [
        ("s1_approach_person", "SUCCEEDED", "la persona ya esta cerca"),
        ("s2_speak", "SUCCEEDED", "dicho (simulado)"),
        ("s3_find_pointed_bag", "FAILED", "no vi a nadie senalando"),
        ("avisar", "SUCCEEDED", "dicho (simulado)"),
    ]
    assert se.first_failed_step(steps) == "s3_find_pointed_bag"


def test_evaluate_succeeded_with_and_without_max_cm():
    sc_normal = se.SCENARIO_MAP["normal"]
    ok_data = se.RunData(mission_state="SUCCEEDED", steps=[("s1_approach_person", "SUCCEEDED", "")], grip_cm=1.6)
    res = se.evaluate(sc_normal, ok_data)
    assert res.ok is True
    assert "1.6 cm" in res.reason

    too_far = se.RunData(mission_state="SUCCEEDED", steps=[], grip_cm=4.0)
    res_far = se.evaluate(sc_normal, too_far)
    assert res_far.ok is False
    assert "4.0 cm >= 4 cm" in res_far.reason

    no_grip = se.RunData(mission_state="SUCCEEDED", steps=[], grip_cm=None)
    res_none = se.evaluate(sc_normal, no_grip)
    assert res_none.ok is False
    assert "sin medicion de pinza" in res_none.reason

    sc_mueve = se.SCENARIO_MAP["mueve"]
    assert se.evaluate(sc_mueve, se.RunData(mission_state="SUCCEEDED", grip_cm=2.0)).ok is True
    assert se.evaluate(sc_mueve, se.RunData(mission_state="FAILED", grip_cm=2.0)).ok is False


def test_evaluate_expected_failure_checks_exact_failed_step():
    sc = se.SCENARIO_MAP["sin_gesto"]
    good = se.RunData(
        mission_state="FAILED",
        steps=[
            ("s1_approach_person", "SUCCEEDED", ""),
            ("s2_speak", "SUCCEEDED", ""),
            ("s3_find_pointed_bag", "FAILED", "not_pointing"),
            ("avisar", "SUCCEEDED", ""),
        ],
    )
    res = se.evaluate(sc, good)
    assert res.ok is True
    assert res.failed_step == "s3_find_pointed_bag"

    wrong_step = se.RunData(
        mission_state="FAILED",
        steps=[("s1_approach_person", "FAILED", "no_person")],
    )
    res_wrong = se.evaluate(sc, wrong_step)
    assert res_wrong.ok is False
    assert "s1_approach_person != esperado s3_find_pointed_bag" in res_wrong.reason

    unexpected_ok = se.RunData(
        mission_state="SUCCEEDED",
        steps=[("s1_approach_person", "SUCCEEDED", "")],
    )
    res_unexp = se.evaluate(sc, unexpected_ok)
    assert res_unexp.ok is False
    assert "estado SUCCEEDED != esperado FAILED" in res_unexp.reason


def test_select_scenarios_and_format_table():
    assert len(se.select_scenarios([])) == 11
    sub = se.select_scenarios(["normal", "sin_persona"])
    assert [s.name for s in sub] == ["normal", "sin_persona"]
    with pytest.raises(ValueError):
        se.select_scenarios(["inexistente"])

    r1 = se.evaluate(se.SCENARIO_MAP["normal"], se.RunData(mission_state="SUCCEEDED", grip_cm=1.6, duration_s=52.3))
    r2 = se.evaluate(
        se.SCENARIO_MAP["sin_persona"],
        se.RunData(mission_state="FAILED", steps=[("s1_approach_person", "FAILED", "")], duration_s=18.0),
    )
    table = se.format_table([r1, r2])
    assert "normal" in table and "PASS" in table
    assert "sin_persona" in table and "s1_approach_person" in table
    assert "Resumen: 2/2 escenarios PASS" in table


def test_clean_sim_env_strips_previous_athome_sim_vars():
    dirty = {"PATH": "/usr/bin", "ATHOME_SIM_BAG2": "9,9,9", "ATHOME_SIM_POINT_ERR_M": "5.0"}
    cleaned = se._clean_sim_env(dirty, {"ATHOME_SIM_BAG": "1.5,1.2,0.30"})
    assert "ATHOME_SIM_BAG2" not in cleaned
    assert "ATHOME_SIM_POINT_ERR_M" not in cleaned
    assert cleaned["ATHOME_SIM_BAG"] == "1.5,1.2,0.30"
    assert cleaned["ROS_DOMAIN_ID"] == "42"
    assert cleaned["ATHOME_SIM"] == "1"
