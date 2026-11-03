"""Ejecutor de misiones: secuencia supervisada con reintentos, timeouts, cancelacion y recuperacion.

Reglas:
  - Un paso fallido (tras sus reintentos) DETIENE la mision, salvo `on_failure: continue` o un id de
    recuperacion definido en el plan.
  - Timeout por paso: se pide cancelar al handler (ctx.cancel) y el paso falla con 'timeout'.
  - Cancelacion de la mision: ctx.cancel en el paso en curso; nunca se arranca un paso nuevo.
  - Estado publicado con el contrato de /mission/status (ver README del paquete).
"""
import itertools
import threading
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from .plan import ABORT, CONTINUE, Plan
from .skills import (ERR_BAD_ARGS, ERR_CANCELED, ERR_EXCEPTION, ERR_TIMEOUT, SkillRegistry, SkillResult)

IDLE, RUNNING, SUCCEEDED, FAILED, CANCELED = "IDLE", "RUNNING", "SUCCEEDED", "FAILED", "CANCELED"
PENDING, SKIPPED = "PENDING", "SKIPPED"
LOG_LIMIT = 100
CANCEL_GRACE_S = 6.0       # cuanto se espera a que un handler cancelado devuelva el control
                           # (> 5 s que espera navigate_to a que Nav2 cierre el goal cancelado)
MAX_JUMPS = 20             # tope de saltos de recuperacion (evita bucles)


@dataclass
class SkillContext:
    """Lo que recibe cada handler: argumentos, evento de cancelacion, deadline y log."""
    args: Dict[str, Any]
    cancel: threading.Event
    deadline: float
    log: Callable[[str, str], None]
    attempt: int = 1

    def remaining(self) -> float:
        return max(0.0, self.deadline - time.monotonic())

    def sleep(self, seconds: float) -> bool:
        """Duerme cancelable. False si se cancelo antes de terminar."""
        return not self.cancel.wait(max(0.0, seconds))


Handler = Callable[[SkillContext], SkillResult]


@dataclass
class StepStatus:
    index: int
    id: str
    skill: str
    args: Dict[str, Any]
    max_attempts: int
    state: str = PENDING
    attempts: int = 0
    started_at: Optional[float] = None
    finished_at: Optional[float] = None
    duration_s: Optional[float] = None
    message: str = ""
    error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return dict(index=self.index, id=self.id, skill=self.skill, args=dict(self.args), state=self.state,
                    attempts=self.attempts, max_attempts=self.max_attempts, started_at=self.started_at,
                    finished_at=self.finished_at, duration_s=self.duration_s, message=self.message,
                    error=self.error)


@dataclass
class MissionStatus:
    mission_id: str = ""
    name: str = ""
    mode: str = "sim"
    state: str = IDLE
    current_step: Optional[int] = None
    started_at: Optional[float] = None
    finished_at: Optional[float] = None
    message: str = ""
    steps: List[StepStatus] = field(default_factory=list)
    log: List[Dict[str, Any]] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return dict(mission_id=self.mission_id, name=self.name, mode=self.mode, state=self.state,
                    current_step=self.current_step, started_at=self.started_at, finished_at=self.finished_at,
                    message=self.message, steps=[s.to_dict() for s in self.steps], log=list(self.log))


class MissionExecutor:
    def __init__(self, registry: SkillRegistry, handlers: Dict[str, Handler], mode: str = "sim",
                 on_update: Optional[Callable[[Dict[str, Any]], None]] = None,
                 clock: Callable[[], float] = time.time):
        self.registry = registry
        self.handlers = dict(handlers)
        self.mode = mode
        self.on_update = on_update
        self.clock = clock
        self.status = MissionStatus(mode=mode)
        self._lock = threading.RLock()
        self._cancel_mission = threading.Event()
        self._step_cancel: Optional[threading.Event] = None
        self._in_recovery = False

    # ------------------------------------------------------------------ estado
    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return self.status.to_dict()

    def _emit(self):
        if self.on_update:
            try:
                self.on_update(self.snapshot())
            except Exception:  # un consumidor roto no debe romper la ejecucion
                pass

    def log(self, level: str, msg: str):
        with self._lock:
            self.status.log.append(dict(t=self.clock(), level=level, msg=msg))
            del self.status.log[:-LOG_LIMIT]
        self._emit()

    @property
    def running(self) -> bool:
        with self._lock:
            return self.status.state == RUNNING

    # ------------------------------------------------------------------ control
    def cancel(self, reason: str = "cancelada por el operador") -> bool:
        with self._lock:
            if self.status.state != RUNNING:
                return False
            self._cancel_mission.set()
            if self._step_cancel is not None:
                self._step_cancel.set()
        self.log("warn", f"Cancelacion pedida: {reason}")
        return True

    def run(self, plan: Plan) -> Dict[str, Any]:
        """Ejecuta el plan (bloqueante). El plan ya debe estar validado. Devuelve el estado final."""
        with self._lock:
            if self.status.state == RUNNING:
                raise RuntimeError("ya hay una mision en curso")
            self._cancel_mission.clear()
            self._in_recovery = False
            self.status = MissionStatus(
                mission_id=uuid.uuid4().hex[:8], name=plan.name, mode=self.mode, state=RUNNING,
                started_at=self.clock(),
                steps=[StepStatus(i, s.id, s.skill, dict(s.args), s.retries + 1) for i, s in enumerate(plan.steps)])
        self.log("info", f"Mision '{plan.name}' iniciada ({len(plan.steps)} pasos, modo {self.mode})")
        final, message = SUCCEEDED, "mision completada"
        i, jumps = 0, 0
        while i < len(plan.steps):
            if plan.steps[i].recovery and not self._in_recovery:
                break  # fin del flujo normal: la seccion de recuperacion solo se alcanza por on_failure
            if self._cancel_mission.is_set():
                final, message = CANCELED, "mision cancelada"
                break
            step = plan.steps[i]
            if step.condition:
                ref = self.status.steps[plan.index_of(step.condition["step"])]
                if ref.state != step.condition["state"]:
                    with self._lock:
                        st = self.status.steps[i]
                        st.state, st.message = SKIPPED, (f"condicion no cumplida: '{ref.id}' es {ref.state}, "
                                                         f"se esperaba {step.condition['state']}")
                    self.log("info", f"'{step.id}' omitido: {st.message}")
                    i += 1
                    continue
            ok = self._run_step(i, step)
            if ok:
                i += 1
                continue
            if self._cancel_mission.is_set():
                final, message = CANCELED, f"mision cancelada durante '{step.id}'"
                break
            if step.on_failure == CONTINUE:
                self.log("warn", f"'{step.id}' fallo; el plan indica continuar")
                i += 1
            elif step.on_failure == ABORT:
                final, message = FAILED, f"fallo '{step.id}': {self.status.steps[i].message}"
                break
            else:
                jumps += 1
                if jumps > MAX_JUMPS:
                    final, message = FAILED, f"demasiadas recuperaciones (>{MAX_JUMPS}); ultimo fallo '{step.id}'"
                    break
                target = plan.index_of(step.on_failure)
                self._in_recovery = plan.steps[target].recovery
                self.log("warn", f"'{step.id}' fallo; recuperacion -> '{step.on_failure}'")
                with self._lock:
                    if target > i:   # hacia adelante: lo salteado queda SKIPPED, el fallo queda registrado
                        for st in self.status.steps[i + 1:target]:
                            st.state = SKIPPED
                    else:            # hacia atras: se re-ejecuta desde target (el paso fallido incluido)
                        for st in self.status.steps[target:]:
                            if st.index != i and st.state in (SUCCEEDED, FAILED, SKIPPED, CANCELED):
                                st.state, st.attempts, st.message, st.error = PENDING, 0, "", None
                i = target
        if final == SUCCEEDED:
            tolerated = {s.id for s in plan.steps if s.on_failure == CONTINUE}
            failed = [st.id for st in self.status.steps if st.state == FAILED]
            not_tolerated = [f for f in failed if f not in tolerated]
            if not_tolerated:   # se llego al final por una recuperacion: la mision NO se cumplio
                final, message = FAILED, f"fallo {not_tolerated}; se ejecuto la recuperacion del plan"
            elif failed:
                message = f"mision completada con pasos fallidos tolerados por el plan: {failed}"
        if final == SUCCEEDED and plan.success_if:
            goal = self.status.steps[plan.index_of(plan.success_if)]
            if goal.state != SUCCEEDED:
                final = FAILED
                message = f"objetivo no cumplido: '{goal.id}' termino {goal.state} ({message})"
        with self._lock:
            for st, sp in zip(self.status.steps, plan.steps):
                if st.state == PENDING and (final != SUCCEEDED or sp.recovery):
                    st.state = SKIPPED
            self.status.state, self.status.message = final, message
            self.status.finished_at = self.clock()
            self.status.current_step = None
            self._step_cancel = None
        self.log("info" if final == SUCCEEDED else "error",
                 f"Mision '{plan.name}' terminada: {final} ({message}), "
                 f"{self.status.finished_at - self.status.started_at:.1f} s")
        return self.snapshot()

    # ------------------------------------------------------------------ pasos
    def _run_step(self, i: int, step) -> bool:
        spec = self.registry.get(step.skill)
        handler = self.handlers.get(step.skill)
        st = self.status.steps[i]
        timeout = step.timeout_s or (spec.default_timeout_s if spec else 30.0)
        with self._lock:
            self.status.current_step = i
            st.state, st.started_at = RUNNING, self.clock()
        self._emit()
        result = SkillResult.failure(ERR_BAD_ARGS, "sin handler para esta skill")
        for attempt in itertools.count(1):
            if attempt > step.retries + 1 or self._cancel_mission.is_set():
                break
            with self._lock:
                st.attempts = attempt
            if attempt > 1:
                self.log("warn", f"'{step.id}': reintento {attempt - 1}/{step.retries}")
            else:
                self.log("info", f"Paso {i + 1} '{step.id}' ({step.skill}) {step.args}")
            if handler is None:
                break
            result = self._call(handler, step, attempt, timeout)
            if result.ok or result.error == ERR_CANCELED:
                break
        with self._lock:
            st.finished_at = self.clock()
            st.duration_s = round(st.finished_at - st.started_at, 3)
            st.message, st.error = result.message, (None if result.ok else result.error)
            if result.ok:
                st.state = SUCCEEDED
            elif result.error == ERR_CANCELED or self._cancel_mission.is_set():
                st.state, st.error = CANCELED, ERR_CANCELED
            else:
                st.state = FAILED
        self.log("info" if result.ok else "error",
                 f"'{step.id}' -> {st.state} en {st.duration_s:.1f} s ({st.attempts} intento/s)"
                 + (f": {result.error} {result.message}" if not result.ok else ""))
        return result.ok

    def _call(self, handler: Handler, step, attempt: int, timeout: float) -> SkillResult:
        cancel = threading.Event()
        with self._lock:
            self._step_cancel = cancel
            if self._cancel_mission.is_set():
                cancel.set()
        ctx = SkillContext(args=dict(step.args), cancel=cancel, deadline=time.monotonic() + timeout,
                           log=self.log, attempt=attempt)
        box: Dict[str, SkillResult] = {}

        def target():
            try:
                r = handler(ctx)
                box["r"] = r if isinstance(r, SkillResult) else SkillResult.failure(
                    ERR_EXCEPTION, f"el handler devolvio {type(r).__name__}")
            except Exception as e:  # noqa: BLE001 - se reporta como fallo del paso, no se traga
                box["r"] = SkillResult.failure(ERR_EXCEPTION, f"{type(e).__name__}: {e}")

        th = threading.Thread(target=target, daemon=True, name=f"skill-{step.id}")
        th.start()
        th.join(timeout)
        if th.is_alive():
            cancel.set()
            th.join(CANCEL_GRACE_S)
            if self._cancel_mission.is_set():
                return SkillResult.failure(ERR_CANCELED, "cancelado")
            msg = f"sin resultado en {timeout:.0f} s"
            if th.is_alive():
                msg += f"; el handler no respondio a la cancelacion en {CANCEL_GRACE_S:.0f} s"
            return SkillResult.failure(ERR_TIMEOUT, msg)
        r = box.get("r") or SkillResult.failure(ERR_EXCEPTION, "sin resultado")
        if not r.ok and cancel.is_set() and self._cancel_mission.is_set():
            return SkillResult.failure(ERR_CANCELED, r.message or "cancelado")
        return r
