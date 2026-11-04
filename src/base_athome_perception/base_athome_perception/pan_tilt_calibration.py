"""Calculos del calibrador del pan-tilt (sin ROS). Convencion logica de la interfaz: pan + izquierda, tilt + arriba.
Los parametros del nodo son FISICOS (ticks del servo): fisico = direccion * logico."""
import math
from typing import Dict, List, Sequence, Tuple

import yaml

SCAN_PANS = (-60.0, -30.0, 0.0, 30.0, 60.0)    # el barrido del receptionist
SCAN_TILTS = (-12.0, -28.0)
USB_HELP = ("El U2D2 se esta desconectando: es electrico y ninguna calibracion lo arregla. Revisar en este orden: "
            "cable USB y su conector, fijacion del cable al moverse la cabeza, 12 V bajo carga, power hub y masa comun.")


def new_center_tick(center: int, logical_rad: float, direction: float, ticks_per_rad: float) -> int:
    """Centro nuevo para que la posicion actual pase a ser (0, 0)."""
    return int(round(center + direction * logical_rad * ticks_per_rad))


def physical_limit(logical_rad: float, direction: float, center_old: int, center_new: int,
                   ticks_per_rad: float) -> float:
    """Limite medido (logico, con el centro viejo) en radianes fisicos respecto del centro nuevo."""
    return direction * logical_rad - (center_new - center_old) / ticks_per_rad


def updated_params(params: Dict, marks: Dict[str, float], pan_dir: float, tilt_dir: float) -> Dict:
    """Parametros nuevos a partir de lo marcado (grados logicos): pan_center, tilt_center, pan_left, pan_right,
    tilt_up, tilt_down. Lo que no se marco queda igual."""
    out = dict(params)
    tpr = float(params["ticks_per_rad"])
    centers = {}
    for axis, d in (("pan", pan_dir), ("tilt", tilt_dir)):
        old = int(params[f"{axis}_center_tick"])
        new = old
        if f"{axis}_center" in marks:
            new = new_center_tick(old, math.radians(marks[f"{axis}_center"]), d, tpr)
        centers[axis] = (old, new)
        out[f"{axis}_center_tick"] = new
        moved = (new - old) / tpr
        lo, hi = float(params[f"{axis}_min_rad"]) - moved, float(params[f"{axis}_max_rad"]) - moved
        ends = ("pan_left", "pan_right") if axis == "pan" else ("tilt_up", "tilt_down")
        vals = {}
        for name in ends:
            if name in marks:
                vals[name] = physical_limit(math.radians(marks[name]), d, old, new, tpr)
        if len(vals) == 1:                      # un solo extremo: reemplaza el del mismo lado
            v = next(iter(vals.values()))
            if v >= 0:
                hi = v
            else:
                lo = v
        elif len(vals) == 2:
            lo, hi = sorted(vals.values())
        if lo >= hi:
            raise ValueError(f"limites invalidos de {axis}: {lo:.3f} >= {hi:.3f}")
        out[f"{axis}_min_rad"], out[f"{axis}_max_rad"] = round(lo, 4), round(hi, 4)
    return out


def params_to_yaml(params: Dict) -> str:
    return yaml.safe_dump({"pan_tilt_node": {"ros__parameters": params}}, sort_keys=True)


def params_from_yaml(text: str) -> Dict:
    return dict((yaml.safe_load(text) or {})["pan_tilt_node"]["ros__parameters"])


def logical_limits(params: Dict, pan_dir: float, tilt_dir: float) -> Dict[str, Tuple[float, float]]:
    """Rango logico en grados que acepta el nodo."""
    out = {}
    for axis, d in (("pan", pan_dir), ("tilt", tilt_dir)):
        a, b = sorted(math.degrees(d * float(params[f"{axis}_{k}_rad"])) for k in ("min", "max"))
        out[axis] = (a, b)
    return out


def ramp(start: Sequence[float], goal: Sequence[float], speed_dps: float, rate_hz: float) -> List[Tuple[float, float]]:
    """Objetivos intermedios a velocidad limitada (siempre termina en goal)."""
    dist = max(abs(goal[0] - start[0]), abs(goal[1] - start[1]))
    n = max(1, int(math.ceil(dist / max(speed_dps, 1e-3) * rate_hz)))
    return [(start[0] + (goal[0] - start[0]) * i / n, start[1] + (goal[1] - start[1]) * i / n) for i in range(1, n + 1)]


def scan_path(limits: Dict[str, Tuple[float, float]]) -> List[Tuple[float, float]]:
    """Las vistas del barrido del receptionist, dentro de los limites."""
    def clip(v, lim):
        return max(lim[0], min(lim[1], v))
    views = [(clip(p, limits["pan"]), clip(t, limits["tilt"])) for t in SCAN_TILTS for p in SCAN_PANS]
    return views + [(0.0, 0.0)]


def tracking_stats(samples: Sequence[Tuple[float, float, float, float, float]]) -> Dict[str, float]:
    """samples: (t, cmd_pan, cmd_tilt, med_pan, med_tilt) en grados. Error tal cual y retardo que mejor explica el error."""
    if len(samples) < 5:
        return dict(n=len(samples), rms_deg=float("nan"), max_deg=float("nan"), lag_s=float("nan"), rms_sin_lag_deg=float("nan"))
    t = [s[0] for s in samples]
    dt = (t[-1] - t[0]) / (len(t) - 1)
    errs = [max(abs(s[1] - s[3]), abs(s[2] - s[4])) for s in samples]
    best = (float("inf"), 0)
    for k in range(0, min(int(1.5 / max(dt, 1e-3)), len(samples) - 2) + 1):
        e = [(samples[i][1] - samples[i + k][3]) ** 2 + (samples[i][2] - samples[i + k][4]) ** 2
             for i in range(len(samples) - k)]
        rms = math.sqrt(sum(e) / len(e))
        if rms < best[0]:
            best = (rms, k)
    return dict(n=len(samples), rms_deg=math.sqrt(sum(x * x for x in errs) / len(errs)), max_deg=max(errs),
                lag_s=best[1] * dt, rms_sin_lag_deg=best[0])


def verdict(step_errors: Sequence[float], stats: Dict[str, float], link_drops: int, usb_drops: int,
            wrong_sense: Sequence[str] = ()) -> List[str]:
    """Conclusiones en orden de importancia: primero lo que invalida a lo demas."""
    out = []
    if link_drops or usb_drops:
        out.append(f"FALLA ELECTRICA: {link_drops} perdidas de enlace, {usb_drops} desconexiones USB. {USB_HELP}")
    for axis in wrong_sense:
        out.append(f"SENTIDO INVERTIDO en {axis}: cambiar {axis}_direction en el lanzador de la Jetson.")
    if step_errors and max(step_errors) > 3.0:
        out.append(f"NO LLEGA AL OBJETIVO: error final {max(step_errors):.1f} grados (roce, tope o falta de fuerza).")
    lag = stats.get("lag_s", float("nan"))
    if lag == lag and lag > 0.6:
        out.append(f"RESPUESTA LENTA: {lag:.2f} s de retardo; el seguimiento de rostro va a ir atrasado.")
    if not out:
        out.append("OK: enlace estable, sentido correcto y llega al objetivo.")
    return out
