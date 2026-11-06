"""Módulo de lógica pura para seguimiento de personas en Base@Home (RoboCup@Home OPL).

Diseño desacoplado sin dependencias de ROS ni Nav2:
- Entra: lista de personas (posición en el mapa y color de ropa) y pose del robot.
- Sale: objetivo de navegación (x, y, yaw), pan de la cabeza (grados) y estado
  ('siguiendo', 'buscando', 'perdida', 'llegó').

Referencia de diseño: docs/INVESTIGACION_SEGUIR_PERSONA.txt (HOME-90 / HOME-92).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Optional, Sequence, Tuple, Union

# ==============================================================================
# CONSTANTES DE CALIBRACIÓN - LÍMITES DE VELOCIDAD, DISTANCIA Y TIEMPOS
# Modificar aquí para calibrar con hardware real (Nav2, Dynamixel XL430, Hokuyo)
# ==============================================================================

# Distancia nominal de seguimiento respecto a la persona (metros)
DEFAULT_FOLLOW_DISTANCE_M: float = 1.10  # [CALIBRAR: 0.80 a 1.50 m]

# Tolerancia de distancia de seguimiento (metros)
FOLLOW_DISTANCE_TOLERANCE_M: float = 0.15  # [CALIBRAR]

# Distancia mínima de seguridad absoluta (frenado / margen de talones)
MIN_SAFETY_DISTANCE_M: float = 0.50  # [CALIBRAR]

# Límites de velocidad del robot en seguimiento
MAX_LINEAR_VEL_MPS: float = 0.38  # [CALIBRAR: límite Nav2 desired_linear_vel]
MIN_LINEAR_VEL_MPS: float = 0.08  # [CALIBRAR: piso anti-stiction base hoverboard]
MAX_ANGULAR_VEL_RADPS: float = 0.80  # [CALIBRAR: ~45.8 deg/s en Nav2]

# Límites y parámetros de giro del cabezal Pan-Tilt (Dynamixel XL430-W250)
MAX_HEAD_PAN_DEG: float = 65.0  # [CALIBRAR: rango seguro sin tensar cables]
MIN_HEAD_PAN_DEG: float = -65.0  # [CALIBRAR]
HEAD_PAN_DEADBAND_DEG: float = 2.0  # [CALIBRAR: zona muerta para evitar oscilación]
HEAD_PAN_SWEEP_AMP_DEG: float = 45.0  # [CALIBRAR: amplitud de barrido en búsqueda]
HEAD_PAN_SWEEP_FREQ_HZ: float = 0.50  # [CALIBRAR: frecuencia de oscilación del barrido]

# Anticipación por velocidad de la persona (segundos de lead time)
LEAD_TIME_S: float = 0.40  # [CALIBRAR: 0.20 a 0.50 s]

# Suavizado exponencial (EMA) para posición de la persona (alpha)
ALPHA_SMOOTH: float = 0.35  # [CALIBRAR: 0.1 suave a 0.8 responsivo]

# Filtro de velocidad máxima atribuible a una persona caminando (m/s)
MAX_PERSON_SPEED_MPS: float = 1.60  # [CALIBRAR: límite superior de caminata rápida]

# Umbral de velocidad para considerar a la persona detenida/quieta (m/s)
PERSON_STATIONARY_SPEED_MPS: float = 0.12  # [CALIBRAR]

# Tiempos de la máquina de estados de pérdida y recuperación (segundos)
LOST_GRACE_TIME_S: float = 0.60  # [CALIBRAR: tiempo sin detección antes de BUSCANDO]
SEARCH_TIMEOUT_S: float = 5.00  # [CALIBRAR: tiempo buscando antes de PERDIDA]

# Parámetros para considerar que se llegó a destino ("llegó")
ARRIVAL_DISTANCE_TOLERANCE_M: float = 1.20  # [CALIBRAR: radio de proximidad a destino]
ARRIVAL_STOP_TIME_S: float = 2.00  # [CALIBRAR: segundos quieta para declarar 'llegó']

# Radio de compuerta espacial para asociar detecciones con el objetivo (metros)
GATE_RADIUS_M: float = 1.50  # [CALIBRAR: radio máximo de asociación]


# ==============================================================================
# TIPOS Y ESTRUCTURAS DE DATOS
# ==============================================================================

class FollowState(str, Enum):
    """Estados del seguidor de personas."""
    SIGUIENDO = "siguiendo"
    BUSCANDO = "buscando"
    PERDIDA = "perdida"
    LLEGO = "llegó"

    def __eq__(self, other: Any) -> bool:
        if isinstance(other, str):
            s1 = self.value.replace("ó", "o")
            s2 = other.replace("ó", "o")
            return self.value == other or s1 == s2
        return super().__eq__(other)


@dataclass
class DetectedPerson:
    """Representación de una persona detectada en el mapa."""
    x: float
    y: float
    clothing_color: str = ""
    track_id: Optional[int] = None
    confidence: float = 1.0
    z: float = 0.0


@dataclass
class RobotPose:
    """Pose del robot en el frame del mapa."""
    x: float
    y: float
    yaw: float = 0.0  # Orientación en radianes (-pi a pi)


@dataclass
class FollowOutput:
    """Salida del cálculo de seguimiento en cada ciclo."""
    nav_goal: Optional[Tuple[float, float, float]]  # (x, y, yaw) en mapa o None
    head_pan_deg: float  # Grados de pan del cabezal (-65 a +65)
    state: FollowState  # siguiendo, buscando, perdida, llegó
    target_pos: Optional[Tuple[float, float]] = None  # (x, y) persona rastreada
    target_vel: Tuple[float, float] = (0.0, 0.0)  # (vx, vy) velocidad estimada
    message: str = ""  # Diagnóstico / log legible

    def __iter__(self):
        """Permite desempaquetar directamente: goal, pan, state = output."""
        yield self.nav_goal
        yield self.head_pan_deg
        yield self.state.value if isinstance(self.state, Enum) else self.state


# ==============================================================================
# FUNCIONES AUXILIARES DE PARSEO Y GEOMETRÍA
# ==============================================================================

def normalize_angle(angle: float) -> float:
    """Normaliza un ángulo al rango [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(val: float, min_val: float, max_val: float) -> float:
    """Acota un valor numérico entre min_val y max_val."""
    return max(min_val, min(max_val, val))


def parse_person(p: Any) -> DetectedPerson:
    """Parsea una entrada heterogénea a DetectedPerson."""
    if isinstance(p, DetectedPerson):
        return p
    if isinstance(p, dict):
        return DetectedPerson(
            x=float(p.get("x", 0.0)),
            y=float(p.get("y", 0.0)),
            clothing_color=str(p.get("clothing_color", p.get("color", ""))),
            track_id=p.get("track_id", p.get("id")),
            confidence=float(p.get("confidence", 1.0)),
            z=float(p.get("z", 0.0)),
        )
    if isinstance(p, (tuple, list)):
        x = float(p[0])
        y = float(p[1])
        color = str(p[2]) if len(p) > 2 else ""
        return DetectedPerson(x=x, y=y, clothing_color=color)
    if hasattr(p, "x") and hasattr(p, "y"):
        color = getattr(p, "clothing_color", getattr(p, "color", ""))
        tid = getattr(p, "track_id", getattr(p, "id", None))
        return DetectedPerson(x=float(p.x), y=float(p.y), clothing_color=str(color), track_id=tid)
    raise ValueError(f"Formato de persona no soportado: {p!r}")


def parse_robot_pose(r: Any) -> RobotPose:
    """Parsea una entrada heterogénea a RobotPose."""
    if isinstance(r, RobotPose):
        return r
    if isinstance(r, dict):
        return RobotPose(
            x=float(r.get("x", 0.0)),
            y=float(r.get("y", 0.0)),
            yaw=float(r.get("yaw", r.get("theta", 0.0))),
        )
    if isinstance(r, (tuple, list)):
        x = float(r[0])
        y = float(r[1])
        yaw = float(r[2]) if len(r) > 2 else 0.0
        return RobotPose(x=x, y=y, yaw=yaw)
    if hasattr(r, "x") and hasattr(r, "y"):
        yaw = getattr(r, "yaw", getattr(r, "theta", 0.0))
        return RobotPose(x=float(r.x), y=float(r.y), yaw=float(yaw))
    raise ValueError(f"Formato de pose de robot no soportado: {r!r}")


def match_clothing_color(detected: str, target: str) -> bool:
    """Compara si el color de ropa coincide de forma robusta e insensible a mayúsculas."""
    if not detected or not target:
        return False
    d = detected.strip().lower()
    t = target.strip().lower()
    return t in d or d in t


# ==============================================================================
# CLASE PRINCIPAL: PersonFollowerLogic
# ==============================================================================

class PersonFollowerLogic:
    """Lógica pura de seguimiento de personas.

    Sin dependencias de ROS ni Nav2. Mantiene internamente el filtrado temporal,
    estimación de velocidad, selección por color/distancia, máquina de estados
    (siguiendo, buscando, perdida, llegó) y cálculo del objetivo de navegación y
    pan de cabeza.
    """

    def __init__(
        self,
        target_color: Optional[str] = None,
        target_track_id: Optional[int] = None,
        destination: Optional[Tuple[float, float]] = None,
        follow_distance_m: float = DEFAULT_FOLLOW_DISTANCE_M,
        lead_time_s: float = LEAD_TIME_S,
        alpha_smooth: float = ALPHA_SMOOTH,
        lost_grace_time_s: float = LOST_GRACE_TIME_S,
        search_timeout_s: float = SEARCH_TIMEOUT_S,
        arrival_stop_time_s: float = ARRIVAL_STOP_TIME_S,
        arrival_distance_tolerance_m: float = ARRIVAL_DISTANCE_TOLERANCE_M,
        auto_lock_first: bool = True,
        strict_color: bool = True,
    ) -> None:
        self.target_color = target_color
        self.target_track_id = target_track_id
        self.destination = destination
        self.follow_distance_m = follow_distance_m
        self.lead_time_s = lead_time_s
        self.alpha_smooth = alpha_smooth
        self.lost_grace_time_s = lost_grace_time_s
        self.search_timeout_s = search_timeout_s
        self.arrival_stop_time_s = arrival_stop_time_s
        self.arrival_distance_tolerance_m = arrival_distance_tolerance_m
        self.auto_lock_first = auto_lock_first
        self.strict_color = strict_color

        # Estado interno de la persona rastreada
        self.target_pos: Optional[Tuple[float, float]] = None
        self.target_vel: Tuple[float, float] = (0.0, 0.0)
        self.last_seen_time: Optional[float] = None
        self.last_known_pos: Optional[Tuple[float, float]] = None
        self.last_known_vel: Tuple[float, float] = (0.0, 0.0)

        # Timers y suavizado
        self._last_step_time: Optional[float] = None
        self._prev_raw_pos: Optional[Tuple[float, float]] = None
        self.stationary_duration: float = 0.0
        self.state: FollowState = FollowState.BUSCANDO
        self.last_pan_deg: float = 0.0

    def reset(self) -> None:
        """Reinicia el estado del seguidor."""
        self.target_pos = None
        self.target_vel = (0.0, 0.0)
        self.last_seen_time = None
        self.last_known_pos = None
        self.last_known_vel = (0.0, 0.0)
        self._last_step_time = None
        self._prev_raw_pos = None
        self.stationary_duration = 0.0
        self.state = FollowState.BUSCANDO
        self.last_pan_deg = 0.0

    def set_destination(self, x: float, y: float) -> None:
        """Configura el punto de destino para evaluar si llegó."""
        self.destination = (float(x), float(y))

    def set_target_color(self, color: str) -> None:
        """Configura el color de ropa objetivo."""
        self.target_color = str(color)

    def lock_target(self, person: Union[DetectedPerson, dict, tuple]) -> None:
        """Fija explícitamente a una persona como objetivo a seguir."""
        p = parse_person(person)
        self.target_pos = (p.x, p.y)
        self.last_known_pos = (p.x, p.y)
        self.target_vel = (0.0, 0.0)
        self.last_known_vel = (0.0, 0.0)
        if p.clothing_color:
            self.target_color = p.clothing_color
        if p.track_id is not None:
            self.target_track_id = p.track_id
        self.state = FollowState.SIGUIENDO
        self.stationary_duration = 0.0

    def _select_target(
        self,
        candidates: Sequence[DetectedPerson],
        robot: RobotPose,
        dt: float,
    ) -> Optional[DetectedPerson]:
        """Selecciona al mejor candidato de la lista de personas detectadas."""
        if not candidates:
            return None

        # Si aún no tenemos objetivo fijado:
        if self.target_pos is None:
            # 1. Si se especificó target_color, buscar coincidencias de color
            if self.target_color:
                matching = [c for c in candidates if match_clothing_color(c.clothing_color, self.target_color)]
                if matching:
                    # Seleccionar la más cercana al robot
                    return min(matching, key=lambda c: math.hypot(c.x - robot.x, c.y - robot.y))
                if self.strict_color:
                    return None

            # 2. Si no hay color o auto_lock_first está activo, seleccionar la más cercana al robot
            if self.auto_lock_first:
                return min(candidates, key=lambda c: math.hypot(c.x - robot.x, c.y - robot.y))
            return None

        # Ya tenemos un objetivo rastreado: predecir posición esperada
        pred_x = self.target_pos[0] + self.target_vel[0] * dt
        pred_y = self.target_pos[1] + self.target_vel[1] * dt

        best_candidate: Optional[DetectedPerson] = None
        best_score = float("inf")

        for c in candidates:
            dist = math.hypot(c.x - pred_x, c.y - pred_y)

            # Compuerta espacial de radio
            gate = GATE_RADIUS_M
            if dist > gate:
                continue

            # Evaluación de color
            color_penalty = 0.0
            if self.target_color:
                is_match = match_clothing_color(c.clothing_color, self.target_color)
                if not is_match:
                    if self.strict_color:
                        # Descartar si el color no coincide con el objetivo fijado
                        continue
                    color_penalty = 10.0
                else:
                    color_penalty = -2.0  # Bonificación por coincidencia de color

            # Evaluación de track_id
            id_bonus = 0.0
            if self.target_track_id is not None and c.track_id is not None:
                if c.track_id == self.target_track_id:
                    id_bonus = -5.0
                else:
                    id_bonus = 2.0

            score = dist + color_penalty + id_bonus
            if score < best_score:
                best_score = score
                best_candidate = c

        return best_candidate

    def step(
        self,
        people: Sequence[Union[DetectedPerson, dict, tuple]],
        robot_pose: Union[RobotPose, dict, tuple],
        timestamp: Optional[float] = None,
        dt: Optional[float] = None,
    ) -> FollowOutput:
        """Ejecuta un ciclo de decisión de seguimiento.

        Args:
            people: Lista de personas detectadas (DetectedPerson, dict o tuple).
            robot_pose: Pose del robot en frame map (RobotPose, dict o tuple).
            timestamp: Marca temporal en segundos (opcional).
            dt: Delta de tiempo en segundos desde el último ciclo (opcional).

        Returns:
            FollowOutput con nav_goal, head_pan_deg y state.
        """
        # 1. Normalización de entradas
        robot = parse_robot_pose(robot_pose)
        parsed_people = [parse_person(p) for p in people]

        # 2. Manejo de tiempo
        if timestamp is not None:
            now = float(timestamp)
        elif self._last_step_time is not None and dt is not None:
            now = self._last_step_time + float(dt)
        else:
            now = 0.0 if self._last_step_time is None else self._last_step_time + (dt if dt is not None else 0.10)

        step_dt = dt if dt is not None else (0.10 if self._last_step_time is None else max(1e-4, now - self._last_step_time))
        self._last_step_time = now

        # 3. Asociación de objetivo
        matched = self._select_target(parsed_people, robot, step_dt)

        if matched is not None:
            # ==============================================================
            # CASO A: OBJETIVO DETECTADO EN ESTE CICLO
            # ==============================================================
            self.last_seen_time = now
            raw_x, raw_y = matched.x, matched.y

            if matched.clothing_color and not self.target_color:
                self.target_color = matched.clothing_color
            if matched.track_id is not None:
                self.target_track_id = matched.track_id

            # Filtrado EMA y estimación de velocidad
            if self._prev_raw_pos is not None and step_dt > 0.005:
                raw_disp = math.hypot(raw_x - self._prev_raw_pos[0], raw_y - self._prev_raw_pos[1])
                raw_speed = raw_disp / step_dt
            else:
                raw_speed = 0.0
            self._prev_raw_pos = (raw_x, raw_y)

            if self.target_pos is None:
                smooth_x, smooth_y = raw_x, raw_y
                vel_x, vel_y = (0.0, 0.0)
            elif raw_speed < PERSON_STATIONARY_SPEED_MPS:
                # Persona físicamente detenida: cancelar velocidad residual de inmediato
                smooth_x, smooth_y = raw_x, raw_y
                vel_x, vel_y = (0.0, 0.0)
            else:
                prev_x, prev_y = self.target_pos
                smooth_x = self.alpha_smooth * raw_x + (1.0 - self.alpha_smooth) * prev_x
                smooth_y = self.alpha_smooth * raw_y + (1.0 - self.alpha_smooth) * prev_y

                # Diferencia finita para velocidad estimada
                if step_dt > 0.005:
                    vx_inst = (smooth_x - prev_x) / step_dt
                    vy_inst = (smooth_y - prev_y) / step_dt

                    # Acotar velocidad a límites físicos humanos
                    speed = math.hypot(vx_inst, vy_inst)
                    if speed > MAX_PERSON_SPEED_MPS:
                        scale = MAX_PERSON_SPEED_MPS / speed
                        vx_inst *= scale
                        vy_inst *= scale

                    # Suavizado de velocidad
                    vel_x = 0.50 * vx_inst + 0.50 * self.target_vel[0]
                    vel_y = 0.50 * vy_inst + 0.50 * self.target_vel[1]
                else:
                    vel_x, vel_y = self.target_vel

            self.target_pos = (smooth_x, smooth_y)
            self.target_vel = (vel_x, vel_y)
            self.last_known_pos = (smooth_x, smooth_y)
            self.last_known_vel = (vel_x, vel_y)

            # Monitoreo de quietud (persona estacionaria)
            current_speed = math.hypot(vel_x, vel_y)
            if current_speed < PERSON_STATIONARY_SPEED_MPS:
                self.stationary_duration += step_dt
            else:
                self.stationary_duration = 0.0

            # 4. Evaluación de Estado: ¿Llegó?
            arrived = False
            dist_to_dest = float("inf")
            if self.destination is not None:
                dist_to_dest = math.hypot(smooth_x - self.destination[0], smooth_y - self.destination[1])
                if dist_to_dest <= self.arrival_distance_tolerance_m:
                    if self.stationary_duration >= self.arrival_stop_time_s:
                        arrived = True

            if arrived or (
                self.state == FollowState.LLEGO
                and self.destination is not None
                and dist_to_dest <= self.arrival_distance_tolerance_m
                and current_speed < PERSON_STATIONARY_SPEED_MPS
            ):
                self.state = FollowState.LLEGO
            else:
                self.state = FollowState.SIGUIENDO

            # 5. Cálculo de Pan del Cabezal (Gaze Tracking)
            # Ángulo absoluto hacia la persona en mapa
            target_bearing = math.atan2(smooth_y - robot.y, smooth_x - robot.x)
            rel_pan_rad = normalize_angle(target_bearing - robot.yaw)
            rel_pan_deg = math.degrees(rel_pan_rad)

            if abs(rel_pan_deg) < HEAD_PAN_DEADBAND_DEG:
                pan_deg = 0.0
            else:
                pan_deg = clamp(rel_pan_deg, MIN_HEAD_PAN_DEG, MAX_HEAD_PAN_DEG)
            self.last_pan_deg = pan_deg

            # 6. Cálculo de Objetivo de Navegación
            if self.state == FollowState.LLEGO:
                # Al llegar, el robot mantiene su posición orientándose a la persona
                nav_goal = (robot.x, robot.y, target_bearing)
                msg = f"Llegó a destino con la persona (dist_dest={dist_to_dest:.2f}m)"
            else:
                # Proyección predictiva con lead_time
                pred_x = smooth_x + vel_x * self.lead_time_s
                pred_y = smooth_y + vel_y * self.lead_time_s

                dx = pred_x - robot.x
                dy = pred_y - robot.y
                dist_to_pred = math.hypot(dx, dy)

                goal_yaw = math.atan2(dy, dx)

                if dist_to_pred <= self.follow_distance_m:
                    # El robot ya está dentro de la distancia de seguimiento: no empuja hacia adelante
                    goal_x = robot.x
                    goal_y = robot.y
                else:
                    # Meta ubicada a follow_distance_m de la persona sobre la línea robot->target
                    ux = dx / dist_to_pred
                    uy = dy / dist_to_pred
                    goal_x = pred_x - self.follow_distance_m * ux
                    goal_y = pred_y - self.follow_distance_m * uy

                nav_goal = (goal_x, goal_y, goal_yaw)
                msg = f"Siguiendo persona en ({smooth_x:.2f}, {smooth_y:.2f}) vel={current_speed:.2f}m/s"

            return FollowOutput(
                nav_goal=nav_goal,
                head_pan_deg=pan_deg,
                state=self.state,
                target_pos=self.target_pos,
                target_vel=self.target_vel,
                message=msg,
            )

        # ==============================================================
        # CASO B: OBJETIVO NO DETECTADO EN ESTE CICLO
        # ==============================================================
        if self.last_seen_time is None:
            # Nunca se ha visto a la persona desde el inicio
            t_lost = (now - (self._last_step_time or now))
        else:
            t_lost = now - self.last_seen_time

        # Transiciones de estado ante pérdida
        if self.last_seen_time is None:
            # Si nunca se detectó nadie desde que arrancó
            if now >= self.search_timeout_s:
                self.state = FollowState.PERDIDA
            else:
                self.state = FollowState.BUSCANDO
        elif t_lost >= self.search_timeout_s:
            # Tiempo máximo de búsqueda excedido: perdida definitiva
            self.state = FollowState.PERDIDA
        elif t_lost >= self.lost_grace_time_s:
            # Superó el tiempo de gracia: buscando activamente
            self.state = FollowState.BUSCANDO
        else:
            # Dentro del tiempo de gracia: se mantiene en estado anterior
            pass

        # Generación de salidas según el estado
        if self.state == FollowState.PERDIDA:
            # Detener navegación y centrar cabezal
            nav_goal = None
            pan_deg = 0.0
            msg = f"Persona perdida definitivamente tras {t_lost:.1f}s sin contacto visual"

        elif self.state == FollowState.BUSCANDO:
            # Protocolo de recuperación (Fases 1 y 2 de INVESTIGACION_SEGUIR_PERSONA.txt):
            # Fase 1: Si hay última posición conocida, avanzar hacia ella ("asomarse a la esquina")
            if self.last_known_pos is not None:
                lx, ly = self.last_known_pos
                dx = lx - robot.x
                dy = ly - robot.y
                dist_to_last = math.hypot(dx, dy)

                if dist_to_last > 0.40:
                    goal_yaw = math.atan2(dy, dx)
                    nav_goal = (lx, ly, goal_yaw)
                else:
                    # Ya llegó a la última posición conocida: detener base
                    nav_goal = None

                # Fase 2: Barrido sinusoidal del cabezal Pan-Tilt
                sweep_t = t_lost - self.lost_grace_time_s
                phase = sweep_t * 2.0 * math.pi * HEAD_PAN_SWEEP_FREQ_HZ
                pan_deg = HEAD_PAN_SWEEP_AMP_DEG * math.sin(phase)
                pan_deg = clamp(pan_deg, MIN_HEAD_PAN_DEG, MAX_HEAD_PAN_DEG)
                msg = f"Buscando persona en última posición ({lx:.2f}, {ly:.2f}), barrido={pan_deg:.1f}°"
            else:
                nav_goal = None
                sweep_t = now
                phase = sweep_t * 2.0 * math.pi * HEAD_PAN_SWEEP_FREQ_HZ
                pan_deg = clamp(HEAD_PAN_SWEEP_AMP_DEG * math.sin(phase), MIN_HEAD_PAN_DEG, MAX_HEAD_PAN_DEG)
                msg = f"Buscando persona (sin posición previa), barrido={pan_deg:.1f}°"

        else:
            # Estado SIGUIENDO durante el breve tiempo de gracia (< LOST_GRACE_TIME_S)
            # Proyectar con la última velocidad conocida (coasting inercial)
            if self.last_known_pos is not None:
                coast_x = self.last_known_pos[0] + self.last_known_vel[0] * t_lost
                coast_y = self.last_known_pos[1] + self.last_known_vel[1] * t_lost
                dx = coast_x - robot.x
                dy = coast_y - robot.y
                dist = math.hypot(dx, dy)
                goal_yaw = math.atan2(dy, dx)

                if dist <= self.follow_distance_m:
                    goal_x, goal_y = robot.x, robot.y
                else:
                    ux, uy = dx / dist, dy / dist
                    goal_x = coast_x - self.follow_distance_m * ux
                    goal_y = coast_y - self.follow_distance_m * uy
                nav_goal = (goal_x, goal_y, goal_yaw)

                # Mantener mirada hacia la posición estimada
                target_bearing = math.atan2(coast_y - robot.y, coast_x - robot.x)
                rel_pan_deg = math.degrees(normalize_angle(target_bearing - robot.yaw))
                pan_deg = clamp(rel_pan_deg, MIN_HEAD_PAN_DEG, MAX_HEAD_PAN_DEG)
                msg = f"Coasting inercial breve ({t_lost:.2f}s < gracia)"
            else:
                nav_goal = None
                pan_deg = 0.0
                msg = "Esperando detección de persona..."

        self.last_pan_deg = pan_deg
        return FollowOutput(
            nav_goal=nav_goal,
            head_pan_deg=pan_deg,
            state=self.state,
            target_pos=self.last_known_pos,
            target_vel=self.last_known_vel,
            message=msg,
        )


# ==============================================================================
# FUNCIÓN DE CONVENIENCIA DE UN SOLO PASO
# ==============================================================================

def follow_person_step(
    people: Sequence[Any],
    robot_pose: Any,
    follower_state: Optional[PersonFollowerLogic] = None,
    **kwargs: Any,
) -> FollowOutput:
    """Función de conveniencia para ejecutar un ciclo de seguimiento.

    Si no se provee un follower_state, crea una instancia temporal.
    Devuelve un FollowOutput que puede desempaquetarse como:
        nav_goal, head_pan, state = follow_person_step(people, robot_pose)
    """
    follower = follower_state or PersonFollowerLogic(**kwargs)
    return follower.step(people, robot_pose)
