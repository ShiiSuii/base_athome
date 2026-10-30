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
    """Microfono simulado con la MISMA semantica que whisper_node (modo bajo demanda):
    solo entrega lo que se 'dice' durante una ventana abierta por listen_trigger y sin mute."""

    def __init__(self, window_s: float = 6.0, latency_s: float = 1.5, require_window: bool = True):
        self.window_s = window_s
        self.latency_s = latency_s
        self.require_window = require_window
        self.muted = False
        self.window_until = 0.0
        self.pending: List[Tuple[float, str]] = []  # (t_publicar, texto)

    def trigger(self, now: Optional[float] = None):
        now = time.time() if now is None else now
        self.window_until = now + self.window_s

    @property
    def listening(self) -> bool:
        return time.time() < self.window_until and not self.muted

    def hear(self, text: str, now: Optional[float] = None) -> bool:
        """El 'invitado' habla. Devuelve False si el robot no estaba escuchando (se pierde, como en el real)."""
        now = time.time() if now is None else now
        if self.require_window and (now >= self.window_until or self.muted):
            return False
        t_pub = max(now, self.window_until if self.require_window else now) + self.latency_s
        self.pending.append((t_pub, text))
        return True

    def due(self, now: Optional[float] = None) -> List[str]:
        now = time.time() if now is None else now
        out = [t for tp, t in self.pending if tp <= now]
        self.pending = [(tp, t) for tp, t in self.pending if tp > now]
        return out
