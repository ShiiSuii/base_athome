"""Conector y compilador entre tools/gpsr_parser y el plan del ejecutor de misiones.

Convierte la salida de GPSRParser (o texto de ordenes) a un Plan determinista
de Base@Home apto para `validate_plan`, identificando de forma explicita:
  1. Habilidades no disponibles en el hardware actual (status=UNAVAILABLE en SkillRegistry:
     find_person, follow_person, guide_person, deliver_object, answer_question).
  2. Acciones del vocabulario GPSR sin ninguna habilidad definida en Base@Home (count, describe).
  3. Errores de sintaxis de argumentos o lugares desconocidos.

Solo compilacion y analisis de cobertura: NUNCA ejecuta motores ni interactua con hardware.
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

# Asegurar acceso a tools/gpsr_parser independientemente del directorio de ejecucion
def _resolve_gpsr_parser_path() -> Optional[Path]:
    current = Path(__file__).resolve()
    for parent in current.parents:
        cand = parent / "tools" / "gpsr_parser"
        if cand.is_dir():
            return cand
    ws_env = os.environ.get("BASE_ATHOME_WS")
    if ws_env:
        cand = Path(ws_env) / "tools" / "gpsr_parser"
        if cand.is_dir():
            return cand
    return None


_parser_dir = _resolve_gpsr_parser_path()
if _parser_dir and str(_parser_dir) not in sys.path:
    sys.path.insert(0, str(_parser_dir))

try:
    from gpsr_parser.actions import Action, PlanResult
    from gpsr_parser.parser import GPSRParser
except ImportError:
    GPSRParser = None  # type: ignore
    PlanResult = None  # type: ignore
    Action = None  # type: ignore

from .plan import Plan, PlanError, Step, validate_plan
from .skills import AVAILABLE, UNAVAILABLE, SkillRegistry, default_registry

# Acciones del closed action vocabulary de tools/gpsr_parser que NO tienen skill
# implementada ni registrada en Base@Home (RuleBook §5.3 / docs/INVESTIGACION_GPSR.txt).
NO_SKILL_ACTIONS: Dict[str, str] = {
    "count": "conteo de entidades/personas/objetos no soportado por ninguna skill de Base@Home",
    "describe": "inspeccion o descripcion de propiedades no soportada por ninguna skill de Base@Home",
}


def _conv_go_to(p: Dict[str, Any]) -> Dict[str, Any]:
    loc = p.get("location")
    return {"place": str(loc)} if loc is not None else {}


def _conv_find_object(p: Dict[str, Any]) -> Dict[str, Any]:
    obj = p.get("object") or p.get("category") or "object"
    return {"object": str(obj)}


def _conv_pick(p: Dict[str, Any]) -> Dict[str, Any]:
    obj = p.get("object") or "object"
    return {"object": str(obj)}


def _conv_place(p: Dict[str, Any]) -> Dict[str, Any]:
    loc = p.get("location")
    return {"place": str(loc)} if loc is not None else {}


def _conv_deliver_to(p: Dict[str, Any]) -> Dict[str, Any]:
    args: Dict[str, Any] = {}
    if p.get("person"):
        args["person"] = str(p["person"])
    if p.get("location"):
        args["place"] = str(p["location"])
    return args


def _conv_find_person(p: Dict[str, Any]) -> Dict[str, Any]:
    # Skill find_person tiene optional=("name", "gesture", "posture", "clothing").
    # Se omiten parametros no reconocidos como 'location' para evitar error de check_args.
    return {k: str(p[k]) for k in ("name", "gesture", "posture", "clothing") if p.get(k) is not None}


def _conv_follow(p: Dict[str, Any]) -> Dict[str, Any]:
    # Skill follow_person tiene optional=("person", "destination")
    return {k: str(p[k]) for k in ("person", "destination") if p.get(k) is not None}


def _conv_guide(p: Dict[str, Any]) -> Dict[str, Any]:
    # Skill guide_person tiene required=("place",), optional=("person",)
    dest = p.get("location") or p.get("destination") or "destination"
    args: Dict[str, Any] = {"place": str(dest)}
    if p.get("person"):
        args["person"] = str(p["person"])
    return args


def _conv_say(p: Dict[str, Any]) -> Dict[str, Any]:
    # Skill speak tiene required=("text",)
    return {"text": str(p.get("text") or "")}


def _conv_answer_question(p: Dict[str, Any]) -> Dict[str, Any]:
    # Skill answer_question no requiere argumentos
    return {}


# Mapeo determinista de acciones GPSR hacia habilidades de Base@Home
GPSR_SKILL_MAP: Dict[str, Tuple[str, Callable[[Dict[str, Any]], Dict[str, Any]]]] = {
    "go_to": ("navigate_to", _conv_go_to),
    "find_object": ("detect_object", _conv_find_object),
    "pick": ("pick_object", _conv_pick),
    "place": ("place_object", _conv_place),
    "deliver_to": ("deliver_object", _conv_deliver_to),
    "find_person": ("find_person", _conv_find_person),
    "follow": ("follow_person", _conv_follow),
    "guide": ("guide_person", _conv_guide),
    "say": ("speak", _conv_say),
    "answer_question": ("answer_question", _conv_answer_question),
}


@dataclass
class GPSRCompiledPlan:
    """Resultado estructurado de la compilacion de un comando GPSR hacia un Plan."""
    ok: bool
    plan: Optional[Plan] = None
    errors: List[str] = field(default_factory=list)
    unavailable_skills: List[str] = field(default_factory=list)
    missing_skills: List[str] = field(default_factory=list)
    actions: List[Dict[str, Any]] = field(default_factory=list)
    explanation: str = ""
    raw_command: str = ""

    @property
    def has_missing_skills(self) -> bool:
        """True si hay acciones del parser que carecen de skill definida en Base@Home."""
        return len(self.missing_skills) > 0

    @property
    def has_unavailable_skills(self) -> bool:
        """True si hay skills requeridas que existen pero estan marcadas como UNAVAILABLE."""
        return len(self.unavailable_skills) > 0

    def summary(self) -> str:
        """Resumen en una linea del estado de compilacion."""
        if self.ok:
            n_steps = len(self.plan.steps) if self.plan else 0
            return f"OK: {n_steps} pasos compilados y verificados"
        parts = []
        if self.missing_skills:
            parts.append(f"acciones sin skill: {self.missing_skills}")
        if self.unavailable_skills:
            parts.append(f"skills unavailable: {self.unavailable_skills}")
        if not parts and self.errors:
            parts.append(f"errores: {len(self.errors)}")
        return f"RECHAZADO ({'; '.join(parts)})"


def gpsr_actions_to_plan(
    actions: Sequence[Dict[str, Any]],
    name: str = "gpsr",
    allow_unmapped: bool = False,
) -> Plan:
    """Convierte una lista de acciones de GPSRParser ([{'action': ..., 'params': ...}]) a un Plan.

    Si allow_unmapped=False (defecto), lanza PlanError si encuentra acciones sin skill en Base@Home
    (como 'count' o 'describe').
    Si allow_unmapped=True, mapea la accion a un Step con skill=act para que validate_plan lo rechace
    explicitamente en su fase de validacion.
    """
    if not actions:
        raise PlanError("el plan GPSR no contiene acciones")

    steps: List[Step] = []
    for i, a in enumerate(actions):
        act = a.get("action", "")
        params = a.get("params") or {}
        if act in GPSR_SKILL_MAP:
            skill_name, conv = GPSR_SKILL_MAP[act]
            steps.append(Step(id=f"g{i + 1}_{act}", skill=skill_name, args=conv(params)))
        elif allow_unmapped:
            steps.append(Step(id=f"g{i + 1}_{act}", skill=act, args=dict(params)))
        else:
            reason = NO_SKILL_ACTIONS.get(act, "sin skill disponible en Base@Home")
            raise PlanError(f"la accion GPSR '{act}' no tiene skill disponible en Base@Home ({reason})")

    return Plan(name=name, steps=steps)


def gpsr_to_plan(
    command_or_result: Union[str, Any, Sequence[Dict[str, Any]]],
    name: str = "gpsr",
    parser: Optional[Any] = None,
) -> Plan:
    """Convierte salida de GPSRParser (o texto de comando) directamente a un Plan de Base@Home.

    Lanza PlanError si el comando no puede parsearse o contiene acciones sin skill en Base@Home.
    """
    if isinstance(command_or_result, str):
        if not command_or_result.strip():
            raise PlanError("comando GPSR vacio")
        if parser is None:
            if GPSRParser is None:
                raise PlanError("modulo gpsr_parser no disponible")
            parser = GPSRParser()
        res = parser.parse(command_or_result)
        if not res.success:
            raise PlanError(f"error del parser GPSR: {res.error or 'no se pudo parsear el comando'}")
        actions = res.actions
    elif hasattr(command_or_result, "actions"):  # PlanResult
        if not getattr(command_or_result, "success", True):
            err = getattr(command_or_result, "error", None) or "comando fallido en parser"
            raise PlanError(f"error del parser GPSR: {err}")
        actions = command_or_result.actions
    elif isinstance(command_or_result, (list, tuple)):
        actions = list(command_or_result)
    else:
        raise PlanError(f"tipo de entrada GPSR no soportado: {type(command_or_result)}")

    return gpsr_actions_to_plan(actions, name=name)


def compile_gpsr(
    command_or_result: Union[str, Any, Sequence[Dict[str, Any]]],
    registry: Optional[SkillRegistry] = None,
    places: Optional[Dict[str, Any]] = None,
    allow_unavailable: bool = False,
    parser: Optional[Any] = None,
    name: str = "gpsr",
) -> GPSRCompiledPlan:
    """Compila un comando GPSR (texto, PlanResult o lista de acciones) a un Plan evaluado por validate_plan.

    Parametros:
      command_or_result: texto del comando, PlanResult de GPSRParser, o lista de dicts de acciones.
      registry: registro de habilidades a consultar (por defecto: default_registry()).
      places: mapa opcional de lugares para verificar existencia de waypoints con resolve().
      allow_unavailable: si es False (por defecto), skills no disponibles (status=UNAVAILABLE)
        hacen que ok=False y se reportan en errors. Si es True, no se consideran errores bloqueantes.
      parser: instancia reutilizable de GPSRParser.
      name: nombre del plan resultante.

    Retorna:
      GPSRCompiledPlan con el estado de compilacion, plan, errores claros y desglose de skills.
    """
    reg = registry or default_registry()
    raw_cmd = ""
    explanation = ""
    actions: List[Dict[str, Any]] = []

    # 1. Obtencion y parseo de entrada
    if isinstance(command_or_result, str):
        raw_cmd = command_or_result
        if not raw_cmd.strip():
            return GPSRCompiledPlan(ok=False, raw_command=raw_cmd, errors=["comando GPSR vacio"])
        if parser is None:
            if GPSRParser is None:
                return GPSRCompiledPlan(
                    ok=False,
                    raw_command=raw_cmd,
                    errors=["modulo gpsr_parser no disponible en el sistema"],
                )
            parser = GPSRParser()
        res = parser.parse(raw_cmd)
        if not res.success:
            return GPSRCompiledPlan(
                ok=False,
                raw_command=raw_cmd,
                errors=[f"error del parser GPSR: {res.error or 'no se pudo parsear el comando'}"],
                explanation=res.explanation,
            )
        actions = res.actions
        explanation = res.explanation
    elif hasattr(command_or_result, "actions"):
        raw_cmd = getattr(command_or_result, "command", "")
        explanation = getattr(command_or_result, "explanation", "")
        if not getattr(command_or_result, "success", True):
            err = getattr(command_or_result, "error", None) or "comando fallido en parser"
            return GPSRCompiledPlan(
                ok=False,
                raw_command=raw_cmd,
                errors=[f"error del parser GPSR: {err}"],
                explanation=explanation,
            )
        actions = command_or_result.actions
    elif isinstance(command_or_result, (list, tuple)):
        actions = list(command_or_result)
        raw_cmd = ""
        explanation = ""
    else:
        return GPSRCompiledPlan(
            ok=False,
            errors=[f"tipo de entrada no soportado: {type(command_or_result)}"],
        )

    # 2. Deteccion de acciones sin ninguna skill en Base@Home (count, describe, etc.)
    missing_skills: List[str] = []
    for a in actions:
        act = a.get("action", "")
        if act not in GPSR_SKILL_MAP:
            if act not in missing_skills:
                missing_skills.append(act)

    errors: List[str] = []
    if missing_skills:
        for act in missing_skills:
            reason = NO_SKILL_ACTIONS.get(act, "sin skill disponible en Base@Home")
            errors.append(f"la accion GPSR '{act}' no tiene skill disponible en Base@Home ({reason})")

    # 3. Construccion del plan de ejecucion
    plan: Optional[Plan] = None
    try:
        plan = gpsr_actions_to_plan(actions, name=name, allow_unmapped=True)
    except PlanError as pe:
        errors.append(str(pe))
        return GPSRCompiledPlan(
            ok=False,
            raw_command=raw_cmd,
            actions=actions,
            explanation=explanation,
            errors=errors,
            missing_skills=missing_skills,
        )

    # 4. Validacion formal con validate_plan
    val_errs = validate_plan(plan, registry=reg, places=places, allow_unavailable=allow_unavailable)

    # 5. Deteccion de skills UNAVAILABLE
    unavailable_skills: List[str] = []
    for s in plan.steps:
        spec = reg.get(s.skill)
        if spec is not None and spec.status != AVAILABLE:
            if s.skill not in unavailable_skills:
                unavailable_skills.append(s.skill)

    for ve in val_errs:
        if ve not in errors:
            errors.append(ve)

    ok = (len(errors) == 0)
    return GPSRCompiledPlan(
        ok=ok,
        plan=plan,
        errors=errors,
        unavailable_skills=unavailable_skills,
        missing_skills=missing_skills,
        actions=actions,
        explanation=explanation,
        raw_command=raw_cmd,
    )


def evaluate_batch(
    commands: Sequence[Union[str, Dict[str, Any]]],
    registry: Optional[SkillRegistry] = None,
    places: Optional[Dict[str, Any]] = None,
    parser: Optional[Any] = None,
) -> Dict[str, Any]:
    """Evalua un lote de comandos GPSR midiendo cuantos compilan, se rechazan y los motivos."""
    reg = registry or default_registry()
    if parser is None and GPSRParser is not None:
        parser = GPSRParser()

    total = len(commands)
    strict_ok = 0
    lenient_ok = 0
    rejection_reasons: Dict[str, int] = {}
    details: List[Dict[str, Any]] = []

    for item in commands:
        if isinstance(item, dict):
            cmd_id = item.get("id")
            cmd_text = item.get("command", "")
            category = item.get("category", "")
        else:
            cmd_id = None
            cmd_text = str(item)
            category = ""

        strict_res = compile_gpsr(cmd_text, registry=reg, places=places, allow_unavailable=False, parser=parser)
        lenient_res = compile_gpsr(cmd_text, registry=reg, places=places, allow_unavailable=True, parser=parser)

        if strict_res.ok:
            strict_ok += 1
            reason = "OK_STRICT"
        else:
            if strict_res.has_missing_skills:
                reason = f"no_skill:{','.join(sorted(strict_res.missing_skills))}"
            elif strict_res.has_unavailable_skills:
                reason = f"unavailable_skill:{','.join(sorted(strict_res.unavailable_skills))}"
            else:
                reason = "validation_error"
            rejection_reasons[reason] = rejection_reasons.get(reason, 0) + 1

        if lenient_res.ok:
            lenient_ok += 1

        details.append({
            "id": cmd_id,
            "category": category,
            "command": cmd_text,
            "strict_ok": strict_res.ok,
            "lenient_ok": lenient_res.ok,
            "reason": reason,
            "errors": strict_res.errors,
            "unavailable_skills": strict_res.unavailable_skills,
            "missing_skills": strict_res.missing_skills,
        })

    return {
        "total": total,
        "strict_ok": strict_ok,
        "strict_rate": (strict_ok / total) if total > 0 else 0.0,
        "lenient_ok": lenient_ok,
        "lenient_rate": (lenient_ok / total) if total > 0 else 0.0,
        "rejection_reasons": rejection_reasons,
        "details": details,
    }
