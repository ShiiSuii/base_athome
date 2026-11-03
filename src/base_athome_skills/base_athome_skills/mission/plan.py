"""Plan de mision estructurado y determinista (no ordenes libres de un LLM a los motores).

Formato (JSON o YAML):
  {"name": "demo", "steps": [
     {"id": "a", "skill": "navigate_to", "args": {"place": "door"}, "retries": 1, "timeout_s": 120,
      "on_failure": "abort"},            # abort (por defecto) | continue | <id de un paso de recuperacion>
     {"id": "hi", "skill": "speak", "args": {"text": "Hello"}}]}
`on_failure: <id>` salta a ese paso (recuperacion explicita) y la mision sigue desde alli.
Lista opcional `recovery: [...]` (mismo formato): pasos que SOLO se ejecutan si un on_failure salta a
ellos; el flujo normal termina antes de la seccion de recuperacion.
"""
import json
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml

from .skills import AVAILABLE, SkillRegistry

ABORT, CONTINUE = "abort", "continue"
MAX_RETRIES = 5
MAX_STEPS = 50


class PlanError(ValueError):
    """Plan invalido: se rechaza ANTES de mover nada."""


@dataclass
class Step:
    id: str
    skill: str
    args: Dict[str, Any] = field(default_factory=dict)
    retries: int = 0
    timeout_s: Optional[float] = None
    on_failure: str = ABORT
    recovery: bool = False
    condition: Optional[Dict[str, str]] = None   # {"step": <id>, "state": SUCCEEDED|FAILED|...}: si no, SKIPPED

    def to_dict(self) -> Dict[str, Any]:
        return dict(id=self.id, skill=self.skill, args=dict(self.args), retries=self.retries,
                    timeout_s=self.timeout_s, on_failure=self.on_failure, recovery=self.recovery,
                    **({"if": dict(self.condition)} if self.condition else {}))


@dataclass
class Plan:
    name: str
    steps: List[Step]
    success_if: Optional[str] = None   # id del paso OBJETIVO: si no termina SUCCEEDED, la mision es FAILED

    def index_of(self, step_id: str) -> int:
        for i, s in enumerate(self.steps):
            if s.id == step_id:
                return i
        raise KeyError(step_id)

    def to_dict(self) -> Dict[str, Any]:
        return dict(name=self.name, steps=[s.to_dict() for s in self.steps if not s.recovery],
                    recovery=[s.to_dict() for s in self.steps if s.recovery])


def parse_plan(data: Any) -> Plan:
    """dict / str JSON / str YAML -> Plan (solo forma; validar luego con validate_plan)."""
    if isinstance(data, (str, bytes)):
        text = data.decode() if isinstance(data, bytes) else data
        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            try:
                data = yaml.safe_load(text)
            except yaml.YAMLError as e:
                raise PlanError(f"plan ilegible: {e}") from e
    if not isinstance(data, dict) or not isinstance(data.get("steps"), list):
        raise PlanError("el plan debe ser un objeto con una lista 'steps'")
    if not isinstance(data.get("recovery", []) or [], list):
        raise PlanError("'recovery' debe ser una lista")
    steps = []
    n_main = len(data["steps"])
    for i, raw in enumerate(list(data["steps"]) + list(data.get("recovery") or [])):
        if not isinstance(raw, dict) or not raw.get("skill"):
            raise PlanError(f"paso {i}: falta 'skill'")
        args = raw.get("args") or {}
        if not isinstance(args, dict):
            raise PlanError(f"paso {i}: 'args' debe ser un objeto")
        try:
            retries = int(raw.get("retries", 0))
            timeout = None if raw.get("timeout_s") is None else float(raw["timeout_s"])
        except (TypeError, ValueError) as e:
            raise PlanError(f"paso {i}: retries/timeout_s invalidos") from e
        steps.append(Step(id=str(raw.get("id") or f"s{i + 1}"), skill=str(raw["skill"]), args=dict(args),
                          retries=retries, timeout_s=timeout, on_failure=str(raw.get("on_failure", ABORT)),
                          recovery=i >= n_main, condition=_condition(raw.get("if"), i)))
    succ = data.get("success_if")
    if isinstance(succ, dict):
        succ = succ.get("step")
    return Plan(name=str(data.get("name") or "mission"), steps=steps, success_if=str(succ) if succ else None)


def _condition(raw, i) -> Optional[Dict[str, str]]:
    if raw is None:
        return None
    if not isinstance(raw, dict) or "step" not in raw:
        raise PlanError(f"paso {i}: 'if' debe ser {{step: <id>, state: <estado>}}")
    return {"step": str(raw["step"]), "state": str(raw.get("state", "SUCCEEDED")).upper()}


def validate_plan(plan: Plan, registry: SkillRegistry, places: Optional[Dict[str, Any]] = None,
                  allow_unavailable: bool = False) -> List[str]:
    """Lista de errores (vacia = plan ejecutable). `places`: si se da, verifica que existan los lugares."""
    from .places import resolve
    errors: List[str] = []
    if not [s for s in plan.steps if not s.recovery]:
        errors.append("el plan no tiene pasos")
    if len(plan.steps) > MAX_STEPS:
        errors.append(f"demasiados pasos ({len(plan.steps)} > {MAX_STEPS})")
    ids = [s.id for s in plan.steps]
    if plan.success_if and plan.success_if not in ids:
        errors.append(f"success_if: '{plan.success_if}' no es un id de paso")
    dup = sorted({i for i in ids if ids.count(i) > 1})
    if dup:
        errors.append(f"ids duplicados: {dup}")
    for s in plan.steps:
        spec = registry.get(s.skill)
        if spec is None:
            errors.append(f"{s.id}: skill desconocida '{s.skill}'")
            continue
        if spec.status != AVAILABLE and not allow_unavailable:
            errors.append(f"{s.id}: la skill '{s.skill}' no esta implementada/validada ({spec.note or spec.status})")
        bad = spec.check_args(s.args)
        if bad:
            errors.append(f"{s.id}: {bad}")
        if not 0 <= s.retries <= MAX_RETRIES:
            errors.append(f"{s.id}: retries debe estar entre 0 y {MAX_RETRIES}")
        if s.timeout_s is not None and not 0 < s.timeout_s <= spec.max_timeout_s:
            errors.append(f"{s.id}: timeout_s debe estar en (0, {spec.max_timeout_s}]")
        if s.on_failure not in (ABORT, CONTINUE) and s.on_failure not in ids:
            errors.append(f"{s.id}: on_failure '{s.on_failure}' no es abort/continue ni un id de paso")
        if s.condition:
            if s.condition["step"] not in ids or ids.index(s.condition["step"]) >= ids.index(s.id):
                errors.append(f"{s.id}: 'if' debe referirse a un paso ANTERIOR (recibido '{s.condition['step']}')")
            if s.condition["state"] not in ("SUCCEEDED", "FAILED", "SKIPPED", "CANCELED"):
                errors.append(f"{s.id}: estado de 'if' invalido '{s.condition['state']}'")
        if s.on_failure == s.id:
            errors.append(f"{s.id}: on_failure no puede apuntar al mismo paso (usar retries)")
        if places is not None:
            wanted = []
            if s.skill == "navigate_to":
                wanted = [s.args.get("place")]
            elif s.skill == "navigate_sequence":
                wanted = list(s.args.get("places") or [])
                if not isinstance(s.args.get("places"), list):
                    errors.append(f"{s.id}: 'places' debe ser una lista")
                    wanted = []
            for p in wanted:
                if resolve(places, str(p)) is None:
                    errors.append(f"{s.id}: lugar desconocido '{p}' (no esta en el mapa semantico)")
            if s.skill == "return_home" and resolve(places, "home") is None and resolve(places, "default") is None:
                errors.append(f"{s.id}: no hay waypoint 'home' ni 'default' en el mapa semantico")
        if s.skill == "wait":
            try:
                if not 0 <= float(s.args.get("seconds", -1)) <= 600:
                    errors.append(f"{s.id}: wait.seconds debe estar entre 0 y 600")
            except (TypeError, ValueError):
                errors.append(f"{s.id}: wait.seconds no es un numero")
    return errors


# GPSR (tools/gpsr_parser) -> skills de este registro. Lo que no tiene skill validada queda como
# skill 'unavailable' y el plan se rechaza al validar: nunca se ejecuta un paso a medias en silencio.
_GPSR_MAP = {
    "go_to": ("navigate_to", lambda p: {"place": p.get("location")}),
    "find_object": ("detect_object", lambda p: {"object": p.get("object") or p.get("category")}),
    "pick": ("pick_object", lambda p: {"object": p.get("object")}),
    "place": ("place_object", lambda p: {"place": p.get("location")} if p.get("location") else {}),
    "deliver_to": ("deliver_object", lambda p: {k: p[k] for k in ("person",) if p.get(k)} |
                   ({"place": p["location"]} if p.get("location") else {})),
    "find_person": ("find_person", lambda p: {k: p[k] for k in ("name", "gesture", "posture", "clothing") if p.get(k)}),
    "follow": ("follow_person", lambda p: {k: p[k] for k in ("person", "destination") if p.get(k)}),
    "guide": ("guide_person", lambda p: {"place": p.get("location"), **({"person": p["person"]} if p.get("person") else {})}),
    "say": ("speak", lambda p: {"text": p.get("text")}),
    "answer_question": ("answer_question", lambda p: {}),
}


def from_gpsr(actions: List[Dict[str, Any]], name: str = "gpsr") -> Plan:
    """Plan JSON del parser GPSR ([{action, params}]) -> Plan. Acciones sin equivalente -> PlanError."""
    steps = []
    for i, a in enumerate(actions):
        act = a.get("action")
        if act not in _GPSR_MAP:
            raise PlanError(f"accion GPSR sin equivalente: '{act}'")
        skill, conv = _GPSR_MAP[act]
        steps.append(Step(id=f"g{i + 1}_{act}", skill=skill, args=conv(a.get("params") or {})))
    return Plan(name=name, steps=steps)
