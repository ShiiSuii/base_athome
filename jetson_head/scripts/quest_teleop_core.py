"""Logica PURA de la teleoperacion del Piper con un control del Meta Quest 2 (sin ROS, testeable).

Mapeo (un solo control, el derecho por defecto):
  grip (squeeze) MANTENIDO = hombre muerto: sin grip no hay movimiento.
  joystick X/Y             = mueve el par de articulaciones del modo actual.
  boton A (o X)            = cambia de modo: j1/j2 -> j3/j4 -> j5/j6 (flanco de subida).
  gatillo                  = pinza: soltado = abierta, apretado = cerrada (proporcional).
Seguridad: velocidad maxima por articulacion, zona muerta del joystick, recorte a los limites del
controlador, y si los datos del visor tienen mas de `timeout_s` el objetivo se congela (sin movimiento).
"""
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

JOINT_LIMITS_DEG = [(-154.0, 154.0), (0.0, 195.0), (-175.0, 0.0), (-110.0, 110.0), (-80.0, 80.0), (-180.0, 180.0)]
MODES: List[Tuple[int, int]] = [(0, 1), (2, 3), (4, 5)]          # pares (eje X, eje Y) de joint1..6
MODE_NAMES = ["base/hombro (j1/j2)", "codo/muneca (j3/j4)", "muneca (j5/j6)"]
GRIPPER_OPEN_MM, GRIPPER_CLOSED_MM = 90.0, 10.0
MARGIN_DEG = 1.0     # no apoyarse exactamente sobre el limite (el controlador rechaza fuera de rango)


@dataclass
class ControllerSample:
    """Lo que manda la pagina WebXR para un control."""
    t: float                      # tiempo de llegada (monotonic del nodo)
    axes: Sequence[float]         # [.., .., stick_x, stick_y] en Quest (indices 2 y 3)
    buttons: Sequence[float]      # 0 trigger, 1 squeeze, 4 A/X, 5 B/Y (valor 0..1)


@dataclass
class TeleopState:
    mode: int = 0
    prev_a: bool = False
    target_deg: Optional[List[float]] = None
    gripper_mm: float = GRIPPER_OPEN_MM
    active: bool = False
    reason: str = "sin datos"
    log: List[str] = field(default_factory=list)


def _deadband(v: float, db: float) -> float:
    if abs(v) < db:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - db) / (1.0 - db)


def step(state: TeleopState, sample: Optional[ControllerSample], measured_deg: Optional[Sequence[float]],
         now: float, dt: float, max_speed_dps: float = 20.0, deadband: float = 0.15,
         timeout_s: float = 0.3, invert_y: bool = True) -> TeleopState:
    """Avanza un ciclo. Devuelve el estado con target_deg (None = no mandar nada) y active (moviendose)."""
    state.active = False
    if measured_deg is None:
        state.target_deg, state.reason = None, "sin feedback del brazo"
        return state
    if sample is None or now - sample.t > timeout_s:
        state.target_deg, state.reason = None, "sin datos del visor (timeout)"
        return state
    b = list(sample.buttons) + [0.0] * 6
    a_pressed = b[4] > 0.5
    if a_pressed and not state.prev_a:
        state.mode = (state.mode + 1) % len(MODES)
        state.log.append(f"modo: {MODE_NAMES[state.mode]}")
    state.prev_a = a_pressed
    state.gripper_mm = GRIPPER_OPEN_MM + (GRIPPER_CLOSED_MM - GRIPPER_OPEN_MM) * max(0.0, min(1.0, b[0]))
    if b[1] < 0.5:                                   # hombre muerto suelto
        state.target_deg, state.reason = None, "grip suelto (hombre muerto)"
        return state
    ax = list(sample.axes) + [0.0] * 4
    sx, sy = _deadband(ax[2], deadband), _deadband(ax[3], deadband)
    if invert_y:
        sy = -sy                                     # en el Quest, empujar adelante da Y negativo
    if state.target_deg is None:                     # al apretar el grip se arranca desde lo MEDIDO
        state.target_deg = list(measured_deg)
    if not sx and not sy:
        state.reason = "grip apretado, joystick en reposo"
        return state
    jx, jy = MODES[state.mode]
    step_deg = max_speed_dps * dt
    tgt = list(state.target_deg)
    for j, v in ((jx, sx), (jy, sy)):
        lo, hi = JOINT_LIMITS_DEG[j]
        tgt[j] = min(hi - MARGIN_DEG, max(lo + MARGIN_DEG, tgt[j] + v * step_deg))
    # no adelantarse demasiado al brazo real: el objetivo nunca se aleja mas de 10 grados de lo medido
    for j in range(6):
        tgt[j] = min(measured_deg[j] + 10.0, max(measured_deg[j] - 10.0, tgt[j]))
    state.target_deg = tgt
    state.active = True
    state.reason = f"moviendo {MODE_NAMES[state.mode]}"
    return state
