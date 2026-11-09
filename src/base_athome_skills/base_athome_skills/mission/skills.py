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
    dock_sensor_pre = ("lidar y odometria recibidos hace no mas de 0.3 s",)
    dock_sensor_err = ("no_scan", "no_odom", "stale_scan", "stale_odom")
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
        SkillSpec("find_pointed_bag", "Ver a quien senala y ubicar la bolsa senalada con la RealSense",
                  optional=("find_timeout_s",), default_timeout_s=40.0, moves_hardware=True,
                  preconditions=("bag_finder corriendo (/bag_finder/*)",), errors=("not_pointing", "bag_not_found",
                                                                                   "finder_unavailable"),
                  note="SIM_PASS; detector de bolsas probado con fotos, no con la camara real"),
        SkillSpec("approach_person", "Buscar a la persona con la cabeza, centrarla y acercarse si esta lejos",
                  optional=("find_timeout_s",), default_timeout_s=120.0, moves_hardware=True,
                  errors=("no_person", "finder_unavailable", "torso_unavailable", "torso_not_calibrated", *nav_err),
                  note="SIM_PASS; seguimiento real sin probar"),
        SkillSpec("approach_bag", "Ir con Nav2 hasta quedar lejos de la bolsa, ya orientado",
                  default_timeout_s=240.0, moves_hardware=True, errors=("no_bag_target", "pose_too_close", *nav_err), note="SIM_PASS"),
        SkillSpec("dock_bag", "Tramo final en linea recta, lento, con parada por lidar para no tocar la bolsa",
                  default_timeout_s=30.0, moves_hardware=True, errors=("blocked",),
                  preconditions=("brazo en pre_grasp_floor", "elevador arriba"),
                  note="SIM_PASS; /cmd_vel real sin probar"),
        SkillSpec("remember_bag", "Junto a la bolsa: medirla con la camara de muneca y guardar la posicion",
                  default_timeout_s=30.0, errors=("no_bag_target", "bag_not_found"),
                  preconditions=("brazo en pre_grasp_floor", "base quieta"), note="SIM_PASS; camara real sin calibrar"),
        SkillSpec("set_torso", "Mover el elevador: target_m, o for_bag para quedar a la altura de la bolsa",
                  optional=("target_m", "for_bag"), default_timeout_s=120.0, moves_hardware=True,
                  errors=("torso_unavailable", "torso_not_calibrated", "torso_timeout", "no_bag_target", "pose_too_close"),
                  note="SIM_PASS; elevador real sin prueba fisica registrada"),
        # Vocabulario sin implementacion validada: se rechaza en la validacion del plan.
        SkillSpec("detect_object", "Detectar un objeto con la camara de muneca (deteccion fresca + centro en base_link)",
                  required=("object",), optional=("detect_timeout_s",), default_timeout_s=20.0,
                  preconditions=("camara de muneca publicando", "brazo en una pose de observacion"),
                  errors=("not_found", "manipulation_unavailable"),
                  note="SIM_PASS; real: distancia por tamano de caja, sin hand-eye (ROBOT.md §5.1)"),
        SkillSpec("find_person", "Buscar una persona", optional=("name", "gesture", "posture", "clothing"),
                  status=UNAVAILABLE, note="logica en el receptionist; falta extraerla como skill"),
        SkillSpec("arm_pose", "Llevar el brazo a una pose ensenada (home, pre_grasp, post_grasp) con MoveIt",
                  required=("pose",), default_timeout_s=40.0, moves_hardware=True,
                  preconditions=("feedback del brazo", "move_group"), errors=("arm_failed", "manipulation_unavailable")),
        SkillSpec("pick_object", "Tomar un objeto: deteccion, IK, aproximacion, cierre y CONFIRMACION por ancho medido",
                  required=("object",), optional=("use_bag_target",), default_timeout_s=90.0, moves_hardware=True,
                  preconditions=("brazo en pre_grasp mirando el objeto", "feedback del brazo", "move_group"),
                  errors=("not_found", "grasp_failed", "grasp_empty", "manipulation_unavailable"),
                  note="SIM_PASS; real requiere allow_manipulation_real:=true y LAB_SUPERVISADO"),
        SkillSpec("dock_table", "Acercarse al borde de la mesa con el lidar: Nav2 frena lejos para el brazo",
                  default_timeout_s=40.0, moves_hardware=True, preconditions=dock_sensor_pre,
                  errors=("blocked", *dock_sensor_err),
                  note="SIM_PASS; sin probar en mesas reales con patas"),
        SkillSpec("undock_table", "Retroceder lo avanzado hacia la mesa, sin mirar atras: despacio y sin nadie detras",
                  default_timeout_s=30.0, moves_hardware=True, preconditions=dock_sensor_pre,
                  errors=("blocked", *dock_sensor_err),
                  note="SIM_PASS; el lidar no cubre atras: supervisar en el robot real"),
        SkillSpec("receive_bag", "Plan B: abrir la pinza, esperar a que la persona ponga la bolsa, cerrar y confirmar por ancho",
                  default_timeout_s=30.0, moves_hardware=True, errors=("grasp_empty", "manipulation_unavailable"),
                  note="SIM_PASS; real requiere allow_manipulation_real:=true y LAB_SUPERVISADO"),
        SkillSpec("place_object", "Colocar el objeto adelante (pre_grasp), abrir y volver a home", optional=("place",),
                  default_timeout_s=60.0, moves_hardware=True, errors=("place_failed", "manipulation_unavailable"),
                  note="SIM_PASS; 'place' es informativo (se coloca frente al robot)"),
        SkillSpec("follow_person", "Seguir a la persona mas cercana hasta que se quede quieta o digan stop",
                  optional=("person", "destination"), default_timeout_s=300.0, moves_hardware=True,
                  errors=("person_lost", "finder_unavailable", "nav_unavailable"),
                  note="SIM_PASS con una persona; sin re-identificacion: puede pasarse a otra persona que se cruce"),
        SkillSpec("guide_person", "Guiar a una persona a un lugar (el commander lo arma con speak y navigate_to)",
                  required=("place",), optional=("person",), status=UNAVAILABLE, moves_hardware=True),
        SkillSpec("give_object", "Avisar, esperar y abrir la pinza para que la persona tome el objeto. No mueve el brazo",
                  default_timeout_s=30.0, moves_hardware=True, errors=("place_failed", "manipulation_unavailable"),
                  note="SIM_PASS; real requiere allow_manipulation_real:=true y LAB_SUPERVISADO"),
        SkillSpec("deliver_object", "Entregar el objeto a una persona o lugar", optional=("person", "place"),
                  status=UNAVAILABLE, moves_hardware=True),
        SkillSpec("answer_question", "Escuchar y responder una pregunta", status=UNAVAILABLE),
        SkillSpec("dock_front", "Acercarse con el lidar a lo que hay adelante (pared, puerta o mesa) y frenar a distancia segura",
                  optional=("target", "standoff_m"), default_timeout_s=40.0, moves_hardware=True,
                  preconditions=dock_sensor_pre,
                  errors=("door_closed", "blocked", "nothing_ahead", "too_far", *dock_sensor_err)),
        SkillSpec("describe_view", "Mirar con la camara de la cabeza y decir lo que ve (VLM local)",
                  optional=("question",), default_timeout_s=45.0, preconditions=("vlm_server activo (127.0.0.1:8772)",),
                  errors=("no_image", "vlm_unavailable", "tts_failed")),
    ])
