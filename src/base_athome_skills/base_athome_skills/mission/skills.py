"""Registro de habilidades: contrato de cada skill (entradas, errores, timeouts, cancelacion, estado real).

`status` es honesto: solo `available` se puede ejecutar. `unavailable` = existe en el vocabulario
(GPSR, pedidos del usuario) pero NO hay implementacion validada; un plan que la use se rechaza.
"""
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

AVAILABLE = "available"
UNAVAILABLE = "unavailable"

# Codigos de error comunes a todas las habilidades.
ERR_TIMEOUT = "timeout"
ERR_CANCELED = "canceled"
ERR_PRECONDITION = "precondition_failed"
ERR_BAD_ARGS = "bad_args"
ERR_EXCEPTION = "exception"


@dataclass
class SkillResult:
    ok: bool
    message: str = ""
    error: Optional[str] = None          # codigo (ERR_* o propio de la skill) si ok=False
    data: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def success(cls, message: str = "", **data) -> "SkillResult":
        return cls(True, message, None, dict(data))

    @classmethod
    def failure(cls, error: str, message: str = "", **data) -> "SkillResult":
        return cls(False, message, error, dict(data))


@dataclass(frozen=True)
class SkillSpec:
    name: str
    description: str
    required: Tuple[str, ...] = ()               # argumentos obligatorios
    optional: Tuple[str, ...] = ()               # argumentos opcionales
    default_timeout_s: float = 30.0
    max_timeout_s: float = 600.0
    cancellable: bool = True
    preconditions: Tuple[str, ...] = ()          # descripcion legible (las verifica el handler)
    errors: Tuple[str, ...] = ()                 # codigos de error propios ademas de los comunes
    status: str = AVAILABLE
    moves_hardware: bool = False                 # base/brazo/cabeza: en modo real exige supervision
    note: str = ""

    def check_args(self, args: Dict[str, Any]) -> Optional[str]:
        missing = [a for a in self.required if a not in args or args[a] in (None, "", [])]
        if missing:
            return f"{self.name}: faltan argumentos {missing}"
        unknown = sorted(set(args) - set(self.required) - set(self.optional))
        if unknown:
            return f"{self.name}: argumentos desconocidos {unknown}"
        return None


class SkillRegistry:
    def __init__(self, specs: List[SkillSpec] = ()):
        self._specs: Dict[str, SkillSpec] = {}
        for s in specs:
            self.add(s)

    def add(self, spec: SkillSpec):
        if spec.name in self._specs:
            raise ValueError(f"skill duplicada: {spec.name}")
        self._specs[spec.name] = spec

    def get(self, name: str) -> Optional[SkillSpec]:
        return self._specs.get(name)

    def names(self) -> List[str]:
        return sorted(self._specs)

    def available(self) -> List[str]:
        return sorted(n for n, s in self._specs.items() if s.status == AVAILABLE)

    def describe(self) -> List[Dict[str, Any]]:
        return [dict(name=s.name, status=s.status, required=list(s.required), optional=list(s.optional),
                     timeout_s=s.default_timeout_s, cancellable=s.cancellable, moves_hardware=s.moves_hardware,
                     preconditions=list(s.preconditions), errors=list(s.errors), description=s.description,
                     note=s.note)
                for s in (self._specs[n] for n in self.names())]


def default_registry() -> SkillRegistry:
    """Vocabulario de Base@Home con el estado REAL de cada habilidad (2026-11-03)."""
    nav_pre = ("accion navigate_to_pose disponible", "lugar definido en semantic_map.yaml/waypoints.yaml")
    nav_err = ("unknown_place", "nav_unavailable", "goal_rejected", "nav_failed", "nav_canceled")
    return SkillRegistry([
        SkillSpec("navigate_to", "Navegar a un lugar semantico (cliente de navigate_to_pose de Nav2)",
                  required=("place",), default_timeout_s=180.0, preconditions=nav_pre, errors=nav_err,
                  moves_hardware=True),
        SkillSpec("navigate_sequence", "Navegar a varios lugares en orden; falla en el primero que falle",
                  required=("places",), default_timeout_s=600.0, max_timeout_s=1800.0, preconditions=nav_pre,
                  errors=nav_err, moves_hardware=True),
        SkillSpec("return_home", "Volver al waypoint 'home' (o 'default') del mapa semantico",
                  default_timeout_s=180.0, preconditions=nav_pre, errors=nav_err, moves_hardware=True),
        SkillSpec("speak", "Decir un texto (TTS de la NUC; en simulacion solo se registra)",
                  required=("text",), default_timeout_s=30.0, errors=("tts_failed",)),
        SkillSpec("listen", "Escuchar una respuesta con el ASR de la Jetson (VoiceClient, /voice/*)",
                  optional=("prompt", "expect", "listen_timeout_s"), default_timeout_s=20.0,
                  preconditions=("whisper_node activo (/voice/*)",), errors=("no_speech", "asr_unavailable",
                                                                            "unexpected_answer")),
        SkillSpec("wait", "Esperar un tiempo (cancelable)", required=("seconds",), default_timeout_s=60.0),
        # Vocabulario sin implementacion validada: se rechaza en la validacion del plan.
        SkillSpec("detect_object", "Detectar un objeto con la camara", required=("object",),
                  status=UNAVAILABLE, note="YOLO TRT publica /arm_camera/detections; falta skill con resultado 3D"),
        SkillSpec("find_person", "Buscar una persona", optional=("name", "gesture", "posture", "clothing"),
                  status=UNAVAILABLE, note="logica en el receptionist; falta extraerla como skill"),
        SkillSpec("pick_object", "Tomar un objeto con el Piper", required=("object",), status=UNAVAILABLE,
                  moves_hardware=True, note="arm_manipulation: sin verificacion de agarre ni hand-eye (ROBOT.md §5)"),
        SkillSpec("place_object", "Colocar el objeto que se lleva", optional=("place",), status=UNAVAILABLE,
                  moves_hardware=True),
        SkillSpec("follow_person", "Seguir a una persona", optional=("person", "destination"), status=UNAVAILABLE,
                  moves_hardware=True),
        SkillSpec("guide_person", "Guiar a una persona a un lugar", required=("place",), optional=("person",),
                  status=UNAVAILABLE, moves_hardware=True),
        SkillSpec("deliver_object", "Entregar el objeto a una persona o lugar", optional=("person", "place"),
                  status=UNAVAILABLE, moves_hardware=True),
        SkillSpec("answer_question", "Escuchar y responder una pregunta", status=UNAVAILABLE),
    ])
