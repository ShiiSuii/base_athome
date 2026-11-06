"""Rutina para abrir una puerta de palanca con el brazo. Logica pura: sin ROS ni OpenCV, para poder testearla.

La rutina es una lista de poses ensenadas a mano; los parametros se ajustan en la ventana (alias `abrir_puerta`) y se
guardan en door_open.yaml. Todavia no mueve la base: el robot se deja a mano frente a la puerta.
"""
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Optional, Tuple

import yaml


@dataclass(frozen=True)
class Step:
    pose: str
    label: str
    hint: str
    grip: bool = False       # al terminar el paso, cerrar la pinza sobre el picaporte


STEPS: Tuple[Step, ...] = (
    Step("door_pre", "Frente al picaporte", "Pinza abierta, a 10-15 cm del picaporte, sin tocarlo."),
    Step("door_handle", "En el picaporte", "Pinza abierta rodeando la palanca, cerca del extremo.", grip=True),
    Step("door_press", "Palanca abajo", "Palanca bajada hasta que destrabe. Sin forzar el tope."),
    Step("door_swing", "Puerta entreabierta", "Tirar o empujar siguiendo el arco de la puerta, sin soltar."),
    Step("door_release", "Soltar", "Palanca arriba otra vez; el programa abre la pinza al llegar."),
)
DEFAULTS: Dict[str, float] = dict(speed_pct=15.0, grip_mm=25.0, pause_s=1.0)
LIMITS: Dict[str, Tuple[float, float]] = dict(speed_pct=(5.0, 40.0), grip_mm=(0.0, 70.0), pause_s=(0.0, 5.0))


def clamp_params(raw: Optional[Mapping[str, Any]]) -> Dict[str, float]:
    """Parametros validos: lo que falte toma el valor por defecto y lo que se pase, el limite."""
    out = dict(DEFAULTS)
    for key, (low, high) in LIMITS.items():
        try:
            out[key] = min(high, max(low, float((raw or {}).get(key, out[key]))))
        except (TypeError, ValueError):
            pass
    return out


def load_params(path: Path) -> Dict[str, float]:
    path = Path(path)
    return clamp_params(yaml.safe_load(path.read_text(encoding="utf-8")) if path.exists() else None)


def save_params(path: Path, params: Mapping[str, Any]) -> None:
    Path(path).write_text(yaml.safe_dump(clamp_params(params), sort_keys=True), encoding="utf-8")


class DoorRoutine:
    """Avanza de a un paso. Los movimientos se piden por callbacks y cualquier falla detiene la rutina."""

    def __init__(self, go_to: Callable[[str], Tuple[bool, str]], gripper: Callable[[float], None],
                 sleep: Callable[[float], None], params: Dict[str, float]):
        self.go_to, self.gripper, self.sleep, self.params = go_to, gripper, sleep, params
        self.index = 0
        self.abort = False

    @staticmethod
    def missing(poses: Mapping[str, Any]) -> List[str]:
        return [s.pose for s in STEPS if s.pose not in poses]

    @property
    def finished(self) -> bool:
        return self.index >= len(STEPS)

    def restart(self) -> None:
        self.index = 0
        self.abort = False

    def run_step(self) -> Tuple[bool, str]:
        """Ejecuta el paso actual y solo si salio bien pasa al siguiente."""
        if self.finished:
            return False, "La rutina termino: B para volver al principio."
        step = STEPS[self.index]
        ok, message = self.go_to(step.pose)
        if not ok:
            return False, f"{step.label}: {message}"
        if step.grip:
            self.gripper(self.params["grip_mm"])
            self.sleep(self.params["pause_s"])
        self.index += 1
        return True, f"{step.label}: listo."

    def run_all(self) -> Tuple[bool, str]:
        """Lo que queda de la rutina. Se detiene en la primera falla o si se pide abortar."""
        message = "Nada para hacer."
        while not self.finished:
            if self.abort:
                return False, "Detenida a pedido."
            ok, message = self.run_step()
            if not ok:
                return False, message
            self.sleep(self.params["pause_s"])
        return True, "Rutina completa."
