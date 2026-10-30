#!/usr/bin/env python3
"""Asistente de ensenanza de poses: HOME -> PRE-GRASP -> POST-GRASP, una tecla ENTER por pose.

Logica pura (sin ROS ni OpenCV) para poder testearla:
  start()        -> el brazo queda libre (torque off) y se pide la primera pose
  confirm(q, g)  -> guarda la pose actual y pasa a la siguiente; tras la ultima, termina
  cancel()       -> termina sin guardar lo pendiente
Las acciones sobre el hardware se piden con callbacks (set_torque, save).
"""
from typing import Callable, List, Optional, Sequence

POSES = ("home", "pre_grasp", "post_grasp")
LABELS = {"home": "HOME", "pre_grasp": "PRE-GRASP", "post_grasp": "POST-GRASP"}
HINTS = {
    "home": "Pose de reposo / segura del brazo.",
    "pre_grasp": "Pinza abierta, mirando el objeto desde ~25-30 cm.",
    "post_grasp": "Donde llevar el objeto despues de agarrarlo.",
}


class TeachWizard:
    def __init__(self, set_torque: Callable[[bool], bool], save: Callable[[str, Sequence[float], float], Optional[str]]):
        """set_torque(on) -> exito; save(nombre, joints_deg, gripper_mm) -> None si ok, texto de error si no."""
        self._set_torque = set_torque
        self._save = save
        self.index: Optional[int] = None
        self.saved: List[str] = []
        self.message = "Presiona [T] para ensenar HOME, PRE-GRASP y POST-GRASP."

    @property
    def active(self) -> bool:
        return self.index is not None

    @property
    def current(self) -> Optional[str]:
        return POSES[self.index] if self.active else None

    def start(self) -> None:
        if self.active:
            return
        self.saved = []
        self.index = 0
        self._set_torque(False)
        self._prompt()

    def _prompt(self) -> None:
        name = self.current
        self.message = (f"Paso {self.index + 1}/3: mueve el brazo a {LABELS[name]} con la mano y presiona [ENTER]. "
                        f"{HINTS[name]}")

    def confirm(self, joints_deg: Sequence[float], gripper_mm: float, feedback_ok: bool = True) -> None:
        if not self.active:
            return
        name = self.current
        if not feedback_ok or len(joints_deg) < 6:
            self.message = f"No guardado: no hay datos reales del brazo. {LABELS[name]} sigue pendiente."
            return
        err = self._save(name, list(joints_deg[:6]), float(gripper_mm))
        if err:
            self.message = f"No guardado ({err}). Corrige la pose y presiona [ENTER] de nuevo."
            return
        self.saved.append(name)
        self.index += 1
        if self.index >= len(POSES):
            self.index = None
            self._set_torque(True)
            self.message = "Listo: HOME, PRE-GRASP y POST-GRASP guardadas. Brazo bloqueado donde quedo."
        else:
            self._prompt()

    def cancel(self) -> None:
        if not self.active:
            return
        self.index = None
        self._set_torque(True)
        done = ", ".join(LABELS[s] for s in self.saved) or "ninguna"
        self.message = f"Ensenanza cancelada. Guardadas: {done}. Brazo bloqueado."
