"""Tests para la conexion entre tools/gpsr_parser y el plan de ejecucion (HOME-89).

Valida:
  1. Compilacion determinista de comandos GPSR a objetos Plan de Base@Home.
  2. Rechazo explicito con error claro ante acciones sin skill en Base@Home (count, describe).
  3. Rechazo en modo estricto de habilidades no implementadas en el hardware (find_person,
     guide_person, deliver_object, answer_question).
  4. Aceptacion en modo lenient (allow_unavailable=True) para evaluar sintaxis y argumentos.
  5. Validacion contra mapa semantico de lugares.
  6. Medicion de cobertura real sobre al menos 30 comandos oficiales de commands.jsonl.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List

import pytest

from base_athome_skills.mission.gpsr_plan import (
    GPSR_SKILL_MAP,
    NO_SKILL_ACTIONS,
    compile_gpsr,
    evaluate_batch,
    gpsr_actions_to_plan,
    gpsr_to_plan,
)
from base_athome_skills.mission.plan import Plan, PlanError, validate_plan
from base_athome_skills.mission.skills import default_registry

REG = default_registry()


def _get_commands_path() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        cand = parent / "tools" / "gpsr_parser" / "data" / "commands.jsonl"
        if cand.is_file():
            return cand
    raise FileNotFoundError("No se encontro tools/gpsr_parser/data/commands.jsonl")


def _load_commands(n: int = 30) -> List[Dict[str, Any]]:
    path = _get_commands_path()
    cmds: List[Dict[str, Any]] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                cmds.append(json.loads(line))
            if len(cmds) >= n:
                break
    return cmds


# ---------------------------------------------------------------------------
# 1. Tests unitarios basicos de compilacion
# ---------------------------------------------------------------------------

def test_gpsr_compiles_navigation_and_speech():
    """Comandos que solo usan navigate_to y speak deben compilar y pasar validate_plan en estricto."""
    cmd = "go to the kitchen"
    res = compile_gpsr(cmd, registry=REG)
    assert res.ok is True
    assert res.errors == []
    assert res.plan is not None
    assert len(res.plan.steps) == 1
    assert res.plan.steps[0].skill == "navigate_to"
    assert res.plan.steps[0].args == {"place": "kitchen"}


def test_gpsr_compiles_manipulation_only_available_skills():
    """El comando 19 de commands.jsonl solo usa navigate_to, detect_object, pick_object y place_object.
    Todas estan AVAILABLE en el registro actual, por lo que compila al 100% en estricto."""
    cmd = "navigate to the side tables then find a lemon and grasp it and put it on the storage rack"
    res = compile_gpsr(cmd, registry=REG)
    assert res.ok is True
    assert res.errors == []
    assert res.plan is not None
    skills = [s.skill for s in res.plan.steps]
    assert skills == ["navigate_to", "detect_object", "pick_object", "navigate_to", "place_object"]
    assert res.plan.steps[1].args == {"object": "lemon"}
    assert res.plan.steps[2].args == {"object": "lemon"}
    assert res.plan.steps[4].args == {"place": "storage rack"}


def test_gpsr_rejects_missing_skills_count_and_describe():
    """Las acciones 'count' y 'describe' carecen de skill en Base@Home y deben rechazarse con error claro."""
    # Count: tell me how many people...
    cmd_count = "tell me how many people in the living room are wearing gray jackets"
    res_count = compile_gpsr(cmd_count, registry=REG)
    assert res_count.ok is False
    assert "count" in res_count.missing_skills
    assert any("la accion GPSR 'count' no tiene skill disponible en Base@Home" in e for e in res_count.errors)

    with pytest.raises(PlanError) as exc_count:
        gpsr_to_plan(cmd_count)
    assert "la accion GPSR 'count' no tiene skill disponible" in str(exc_count.value)

    # Describe: tell me what is the largest drink...
    cmd_desc = "tell me what is the largest drink on the storage rack"
    res_desc = compile_gpsr(cmd_desc, registry=REG)
    assert res_desc.ok is False
    assert "describe" in res_desc.missing_skills
    assert any("la accion GPSR 'describe' no tiene skill disponible en Base@Home" in e for e in res_desc.errors)

    with pytest.raises(PlanError) as exc_desc:
        gpsr_to_plan(cmd_desc)
    assert "la accion GPSR 'describe' no tiene skill disponible" in str(exc_desc.value)


def test_gpsr_rejects_unavailable_skills_in_strict_mode():
    """Habilidades no implementadas en el robot actual (status=UNAVAILABLE) son rechazadas por defecto."""
    # find_person visual disponible; nombre sigue fuera del contrato
    cmd_follow = "go to the bed then meet Angel and follow them"
    res_follow = compile_gpsr(cmd_follow, registry=REG, allow_unavailable=False)
    assert res_follow.ok is False
    assert not res_follow.unavailable_skills
    assert any("name" in e for e in res_follow.errors)

    # deliver_object
    cmd_deliver = "bring me a bowl from the cabinet"
    res_deliver = compile_gpsr(cmd_deliver, registry=REG, allow_unavailable=False)
    assert res_deliver.ok is False
    assert "deliver_object" in res_deliver.unavailable_skills
    assert any("la skill 'deliver_object' no esta implementada" in e for e in res_deliver.errors)

    # guide_person
    cmd_guide = "meet Simone in the kitchen and guide them to the bedroom"
    res_guide = compile_gpsr(cmd_guide, registry=REG, allow_unavailable=False)
    assert res_guide.ok is False
    assert "guide_person" in res_guide.unavailable_skills
    assert any("la skill 'guide_person' no esta implementada" in e for e in res_guide.errors)


def test_gpsr_accepts_unavailable_skills_in_lenient_mode():
    """En modo lenient (allow_unavailable=True), comandos cuyas acciones si corresponden a skills registradas
    generan un Plan valido sin errores bloqueantes."""
    cmd_follow = "bring me a bowl from the cabinet"
    res = compile_gpsr(cmd_follow, registry=REG, allow_unavailable=True)
    assert res.ok is True
    assert res.errors == []
    assert res.plan is not None
    # Las skills unavailable se registran informativamente
    assert set(res.unavailable_skills) == {"deliver_object"}

    # Pero acciones sin skill (count/describe) siguen fallando aun en lenient
    cmd_count = "tell me how many people in the living room are wearing gray jackets"
    res_count = compile_gpsr(cmd_count, registry=REG, allow_unavailable=True)
    assert res_count.ok is False
    assert "count" in res_count.missing_skills


def test_gpsr_places_validation():
    """validate_plan verifica que los lugares existan en el diccionario places provisto."""
    places = {
        "side tables": (1.0, 1.0, 0.0),
        "storage rack": (2.0, 2.0, 0.0),
    }
    cmd = "navigate to the side tables then find a lemon and grasp it and put it on the storage rack"
    res_ok = compile_gpsr(cmd, registry=REG, places=places)
    assert res_ok.ok is True

    bad_places = {"kitchen": (1.0, 1.0, 0.0)}
    res_bad = compile_gpsr(cmd, registry=REG, places=bad_places)
    assert res_bad.ok is False
    assert any("lugar desconocido 'side tables'" in e for e in res_bad.errors)
    assert any("lugar desconocido 'storage rack'" in e for e in res_bad.errors)


def test_gpsr_invalid_and_empty_inputs():
    """Comandos vacios o no interpretables deben ser rechazados de forma determinista."""
    res_empty = compile_gpsr("", registry=REG)
    assert res_empty.ok is False
    assert any("vacio" in e for e in res_empty.errors)

    with pytest.raises(PlanError) as exc_empty:
        gpsr_to_plan("")
    assert "vacio" in str(exc_empty.value)

    res_garbage = compile_gpsr("qwerty asdfgh zxcvbn completely invalid non command", registry=REG)
    assert res_garbage.ok is False
    assert any("error del parser GPSR" in e for e in res_garbage.errors)


def test_gpsr_plan_result_input_compatibility():
    """compile_gpsr acepta tanto string crudo como PlanResult proveniente de GPSRParser."""
    from gpsr_parser.parser import GPSRParser

    parser = GPSRParser()
    p_res = parser.parse("go to the kitchen")
    compiled = compile_gpsr(p_res, registry=REG)
    assert compiled.ok is True
    assert compiled.raw_command == "go to the kitchen"
    assert len(compiled.plan.steps) == 1


# ---------------------------------------------------------------------------
# 2. Benchmark obligatorio: al menos 30 comandos de commands.jsonl
# ---------------------------------------------------------------------------

def test_gpsr_commands_jsonl_benchmark_30():
    """Prueba de compilacion sobre 30 comandos oficiales de tools/gpsr_parser/data/commands.jsonl.

    Mide de forma exacta:
      - Cuantos compilan en estricto (ejecutables con hardware actual)
      - Cuantos compilan en lenient (sintaxis y argumentos conformes al registro)
      - Cuantos se rechazan y el motivo exacto de rechazo.
    """
    cmds = _load_commands(n=30)
    assert len(cmds) == 30, f"Se esperaban 30 comandos, se cargaron {len(cmds)}"

    report = evaluate_batch(cmds, registry=REG)

    total = report["total"]
    strict_ok = report["strict_ok"]
    lenient_ok = report["lenient_ok"]
    reasons = report["rejection_reasons"]

    # Imprimir tabla de cobertura clara para registro de evidencia
    print("\n" + "=" * 80)
    print("TABLA DE COBERTURA REAL GPSR -> PLAN (30 COMANDOS OFICIALES)")
    print("=" * 80)
    print(f"Total comandos evaluados:          {total:2d}")
    print(f"Compilan en estricto (100% lista): {strict_ok:2d} ({report['strict_rate'] * 100:.1f}%)")
    print(f"Compilan en lenient (sintacticos): {lenient_ok:2d} ({report['lenient_rate'] * 100:.1f}%)")
    print("-" * 80)
    print("Desglose de motivos de rechazo en modo estricto:")
    for reason, count in sorted(reasons.items()):
        pct = (count / total) * 100
        print(f"  * {reason:<45}: {count:2d} ({pct:4.1f}%)")
    print("=" * 80)

    # Registro actualizado: find_person visual disponible, name rechazado como argumento.
    assert strict_ok == 7
    assert lenient_ok == 16
    assert reasons == {"no_skill:count": 1, "no_skill:describe": 3,
                       "unavailable_skill:deliver_object": 12, "validation_error": 7}
    assert sum(reasons.values()) == total - strict_ok

    # 6. Cada detalle contiene un motivo no vacio y errores claros
    for item in report["details"]:
        if not item["strict_ok"]:
            assert len(item["errors"]) > 0, f"Comando {item['id']} rechazado sin mensaje de error"


def test_gpsr_commands_jsonl_benchmark_50():
    """Extension del benchmark a 50 comandos para confirmar estabilidad de la tasa."""
    cmds = _load_commands(n=50)
    assert len(cmds) == 50

    report = evaluate_batch(cmds, registry=REG)
    assert report["total"] == 50
    assert report["strict_ok"] == 13
    assert report["lenient_ok"] == 27
    reasons = report["rejection_reasons"]
    assert reasons.get("no_skill:count", 0) == 3
    assert reasons.get("no_skill:describe", 0) == 5
