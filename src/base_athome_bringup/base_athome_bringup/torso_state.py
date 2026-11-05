"""Estado MEDIDO del elevador (HOME-43): parseo de la linea STATUS del firmware nema23_esp32.

Firmware (firmware/nema23_esp32/nema23_esp32.ino): `?` imprime
  "[STATUS] Modo: IDLE | Accion: ... | Limites: [TOP(Pin7)=LIBRE, BOT(Pin8)=PULSADO] | Restante: 0 | Pos: 1234 / 150000 [CALIBRADO]"
Pos = pasos desde el sensor INFERIOR (0) ; carrera = pasos hasta el SUPERIOR (tras HOME/CALIBRATE).
URDF: torso_lift_joint = 0 en el tope superior y -stroke_m en el inferior (stroke 0,60 m entre sensores,
xacro/properties.xacro `torso_switch_stroke`; medida del modelo, PENDIENTE de verificar en el robot).
Sin calibracion la posicion NO se conoce: no se inventa.
"""
import re
from dataclasses import dataclass
from typing import Optional

_RE = re.compile(r"\[STATUS\].*?Modo:\s*(?P<mode>\w+).*?TOP\(Pin7\)=(?P<top>\w+),\s*BOT\(Pin8\)=(?P<bot>\w+).*?"
                 r"Pos:\s*(?P<pos>-?\d+)(?:\s*/\s*(?P<max>\d+)\s*\[CALIBRADO\])?")


@dataclass
class TorsoStatus:
    mode: str
    top_hit: bool
    bottom_hit: bool
    steps: int
    max_steps: Optional[int]

    @property
    def calibrated(self) -> bool:
        return bool(self.max_steps) and self.max_steps > 0

    def joint_position(self, stroke_m: float = 0.60) -> Optional[float]:
        """torso_lift_joint (m) o None si no esta calibrado. Recorta a [-stroke, 0]."""
        if not self.calibrated:
            return None
        frac = min(1.0, max(0.0, self.steps / float(self.max_steps)))
        return -stroke_m * (1.0 - frac)

    def steps_to(self, target_m: float, stroke_m: float = 0.60, margin_m: float = 0.01) -> Optional[int]:
        """Pasos relativos (+ sube) para llegar a torso_lift_joint = target_m. None si no esta calibrado."""
        if not self.calibrated:
            return None
        target = max(-(stroke_m - margin_m), min(-margin_m, float(target_m)))
        goal = round((1.0 + target / stroke_m) * self.max_steps)
        return goal - self.steps

    @property
    def moving(self) -> bool:
        return self.mode.upper() != "IDLE"


def parse_status(line: str) -> Optional[TorsoStatus]:
    m = _RE.search(line or "")
    if not m:
        return None
    return TorsoStatus(mode=m["mode"], top_hit=m["top"].upper() == "PULSADO", bottom_hit=m["bot"].upper() == "PULSADO",
                       steps=int(m["pos"]), max_steps=int(m["max"]) if m["max"] else None)
