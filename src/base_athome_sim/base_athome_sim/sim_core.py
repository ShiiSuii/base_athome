#!/usr/bin/env python3
"""Modelos puros (sin ROS) de la simulacion: brazo, pan-tilt, base movil y microfono.

Se testean sin ROS; sim_robot.py los conecta a los mismos topicos/servicios que el robot real.
"""
import math
import time
from typing import List, Optional, Sequence, Tuple

REST_DEG = [86.97, 172.87, -162.63, -99.89, -0.68, 44.98]


class SimArm:
    """Piper simulado: sigue joint_cmd (coseno) y trayectorias temporizadas como el controlador real."""

    def __init__(self, joints_deg: Optional[Sequence[float]] = None):
        self.q = list(joints_deg or REST_DEG)
        self.gripper_mm = 60.0
        self.enabled = False
        self._seg = None   # (t0, dur, q0, q1)
        self._traj = None  # (t0, times, points)
        self._grip = None  # (t0, dur, g0, g1)

    def set_enable(self, on: bool) -> bool:
        self.enabled = bool(on)
        if not on:
            self._seg = self._traj = None
        return True

    def joint_cmd(self, target_deg: Sequence[float], duration: float, now: Optional[float] = None):
        now = time.time() if now is None else now
        self._traj = None
        self._seg = (now, max(0.6, min(10.0, duration)), list(self.q), list(target_deg))

    def trajectory(self, times: Sequence[float], points: Sequence[Sequence[float]], now: Optional[float] = None):
        now = time.time() if now is None else now
        self._seg = None
        self._traj = (now, [0.0] + list(times), [list(self.q)] + [list(p) for p in points])

    def gripper(self, mm: float, now: Optional[float] = None):
        now = time.time() if now is None else now
        self._grip = (now, 1.2, self.gripper_mm, float(max(0.0, min(105.0, mm))))

    @property
    def moving(self) -> bool:
        return self._seg is not None or self._traj is not None

    def update(self, now: Optional[float] = None):
        now = time.time() if now is None else now
        if self.enabled and self._seg:
            t0, dur, q0, q1 = self._seg
            a = min(1.0, (now - t0) / dur)
            s = (1 - math.cos(math.pi * a)) / 2
            self.q = [x + s * (y - x) for x, y in zip(q0, q1)]
            if a >= 1.0:
                self._seg = None
        elif self.enabled and self._traj:
            t0, times, pts = self._traj
            el = now - t0
            if el >= times[-1]:
                self.q, self._traj = list(pts[-1]), None
            else:
                k = max(0, next(i for i, t in enumerate(times) if t > el) - 1)
                a = (el - times[k]) / max(1e-3, times[k + 1] - times[k])
                self.q = [x + a * (y - x) for x, y in zip(pts[k], pts[k + 1])]
        if self.enabled and self._grip:
            t0, dur, g0, g1 = self._grip
            a = min(1.0, (now - t0) / dur)
            self.gripper_mm = g0 + a * (g1 - g0)
            if a >= 1.0:
                self._grip = None


class SimPanTilt:
    """Pan-tilt simulado: va al objetivo a velocidad limitada (grados/s) si tiene torque."""

    def __init__(self, speed_dps: float = 90.0):
        self.pan = self.tilt = 0.0
        self.target = (0.0, 0.0)
        self.torque = False
        self.speed = speed_dps

    def update(self, dt: float):
        if not self.torque:
            return
        step = self.speed * dt
        tp, tt = self.target
        self.pan += max(-step, min(step, tp - self.pan))
        self.tilt += max(-step, min(step, tt - self.tilt))


class SimBase:
    """Base movil simulada: Nav2 'teletransporta' suavemente hacia el objetivo (0.5 m/s)."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0, speed=0.5):
        self.x, self.y, self.yaw = x, y, yaw
        self.speed = speed
        self.goal: Optional[Tuple[float, float, float]] = None

    def set_goal(self, x, y, yaw):
        self.goal = (x, y, yaw)

    def update(self, dt: float) -> bool:
        """Avanza; devuelve True cuando llego."""
        if self.goal is None:
            return True
        gx, gy, gyaw = self.goal
        dx, dy = gx - self.x, gy - self.y
        d = math.hypot(dx, dy)
        step = self.speed * dt
        if d < 0.02 or step >= d:
            self.x, self.y, self.yaw, self.goal = gx, gy, gyaw, None
            return True
        self.x += step * dx / d
        self.y += step * dy / d
        self.yaw = math.atan2(dy, dx)
        return False


class SimMicrophone:
    """Microfono simulado con la MISMA semantica que whisper_node (escucha por pedido con id):

    - request(id) abre una escucha: espera a que la persona EMPIECE a hablar hasta start_timeout
      (el reloj se detiene mientras el robot habla: mute) y corta end_silence despues de que termina.
    - Lo que se dice sin escucha abierta o con el robot hablando se pierde (como en el real).
    - due() devuelve los resultados (id, status, texto): 'ok' o 'no_speech'. Siempre hay resultado.
    """

    WORD_SEC = 0.35  # duracion aproximada de cada palabra hablada

    def __init__(self, start_timeout: float = 8.0, end_silence: float = 1.1, asr_s: float = 1.3):
        self.start_timeout = start_timeout
        self.end_silence = end_silence
        self.asr_s = asr_s
        self.muted = False
        self.active: Optional[dict] = None          # {'id', 'deadline'}
        self.pending: List[Tuple[float, str, str, str]] = []  # (t_publicar, id, status, texto)
        self._last = None

    def request(self, rid: str, start_timeout: Optional[float] = None, end_silence: Optional[float] = None,
                now: Optional[float] = None) -> Optional[str]:
        """Abre una escucha. Devuelve el id reemplazado (que debe terminar como 'cancelled'), si habia."""
        now = time.time() if now is None else now
        old = self.active["id"] if self.active else None
        self.active = {"id": rid, "deadline": now + (start_timeout or self.start_timeout),
                       "end_silence": end_silence or self.end_silence}
        self._last = now
        return old

    def cancel(self, rid: str = "") -> Optional[str]:
        if self.active and rid in ("", "*", self.active["id"]):
            old, self.active = self.active["id"], None
            return old
        return None

    @property
    def listening(self) -> bool:
        return self.active is not None and not self.muted

    def hear(self, text: str, now: Optional[float] = None) -> bool:
        """El 'invitado' habla. Devuelve False si el robot no estaba escuchando (se pierde, como en el real)."""
        now = time.time() if now is None else now
        if self.active is None or self.muted:
            return False
        dur = max(0.5, len(text.split()) * self.WORD_SEC)
        self.pending.append((now + dur + self.active["end_silence"] + self.asr_s, self.active["id"], "ok", text))
        self.active = None
        return True

    def due(self, now: Optional[float] = None) -> List[Tuple[str, str, str]]:
        now = time.time() if now is None else now
        if self.active is not None:
            if self.muted and self._last is not None:
                self.active["deadline"] += now - self._last   # mientras el robot habla no corre la espera
            if now >= self.active["deadline"]:
                self.pending.append((now, self.active["id"], "no_speech", ""))
                self.active = None
        self._last = now
        out = [(rid, st, t) for tp, rid, st, t in self.pending if tp <= now]
        self.pending = [x for x in self.pending if x[0] > now]
        return out
