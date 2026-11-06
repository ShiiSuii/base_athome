#!/usr/bin/env python3
"""Escenarios de regresion en simulacion para 'take out the trash' (HOME-75).

Corre un escenario por vez con `sim_commander.sh`, lee del cliente el estado de cada paso y del log de
`sim_robot` la linea 'Pinza cerrada ... a X cm', imprime una tabla y sale con rc != 0 si algun escenario
no cumple su expectativa.

Uso:
  GAZEBO=1 python3 simulation/scripts/sim_escenarios.py    (sin GAZEBO=1: simulador liviano)
  python3 simulation/scripts/sim_escenarios.py [nombre ...]
"""
from dataclasses import dataclass, field
from pathlib import Path
import json
import os
import re
import signal
import subprocess
import sys
import tempfile
import time
from typing import Dict, List, Optional, Sequence, Tuple

COMMAND = "take out the trash"
RESULTS_FILE = Path.home() / ".local" / "share" / "base_athome" / "sim_escenarios.json"
DEFAULT_BAG = "1.5,1.2,0.30"
DEFAULT_PERSON = "2.6,0.3"

GRIP_RE = re.compile(r"Pinza cerrada:.*?\ba\s+([0-9]+(?:\.[0-9]+)?)\s*cm\b")
STEP_RE = re.compile(r"^\s*\[\s*[0-9]+(?:\.[0-9]+)?s\]\s+(\S+):\s+(SUCCEEDED|FAILED|CANCELED|SKIPPED|RUNNING)\s*(.*)$")
FIN_RE = re.compile(r"^\s*FIN:\s+(SUCCEEDED|FAILED|CANCELED|ABORTED)\s*(.*)$")


@dataclass(frozen=True)
class Scenario:
    name: str
    env: Dict[str, str]
    expected_state: str = "SUCCEEDED"
    max_cm: Optional[float] = None
    expected_failed_step: Optional[str] = None
    command: str = COMMAND


@dataclass
class RunData:
    mission_state: str = "UNKNOWN"
    mission_message: str = ""
    steps: List[Tuple[str, str, str]] = field(default_factory=list)
    grip_cm: Optional[float] = None
    duration_s: float = 0.0
    log_dir: str = ""


@dataclass
class EvalResult:
    scenario: Scenario
    data: RunData
    ok: bool
    failed_step: Optional[str]
    reason: str


SCENARIOS: List[Scenario] = [
    Scenario(
        name="normal",
        env={"ATHOME_SIM_BAG": "1.5,1.2,0.30", "ATHOME_SIM_PERSON": "2.6,0.3"},
        expected_state="SUCCEEDED",
        max_cm=4.0,
    ),
    Scenario(
        name="lejos",
        env={"ATHOME_SIM_PERSON": "5.0,0.6", "ATHOME_SIM_BAG": "4.0,1.3,0.30"},
        expected_state="SUCCEEDED",
        max_cm=6.0,
    ),
    Scenario(
        name="mueve",
        env={"ATHOME_SIM_BAG": DEFAULT_BAG, "ATHOME_SIM_PERSON": DEFAULT_PERSON, "ATHOME_SIM_PERSON_SWAY_M": "0.25"},
        expected_state="SUCCEEDED",
    ),
    Scenario(
        name="distractor",
        env={"ATHOME_SIM_BAG": DEFAULT_BAG, "ATHOME_SIM_PERSON": DEFAULT_PERSON, "ATHOME_SIM_BAG2": "1.0,0.45,0.25"},
        expected_state="SUCCEEDED",
    ),
    Scenario(
        name="alta",
        env={"ATHOME_SIM_BAG": "1.5,1.2,0.45", "ATHOME_SIM_PERSON": DEFAULT_PERSON},
        expected_state="SUCCEEDED",
    ),
    Scenario(
        name="tilt3",
        env={"ATHOME_SIM_BAG": DEFAULT_BAG, "ATHOME_SIM_PERSON": DEFAULT_PERSON, "ATHOME_SIM_WRIST_TILT_ERR_DEG": "3"},
        expected_state="SUCCEEDED",
    ),
    Scenario(
        name="sin_gesto",
        env={"ATHOME_SIM_BAG": DEFAULT_BAG, "ATHOME_SIM_PERSON": DEFAULT_PERSON, "ATHOME_SIM_PERSON_POINTS": "0"},
        expected_state="FAILED",
        expected_failed_step="s3_find_pointed_bag",
    ),
    Scenario(
        name="sin_persona",
        env={"ATHOME_SIM_BAG": DEFAULT_BAG, "ATHOME_SIM_PERSON": "-6,-6"},
        expected_state="FAILED",
        expected_failed_step="s1_approach_person",
    ),
    Scenario(
        name="apunta_mal",
        env={"ATHOME_SIM_BAG": "0.5,2.5,0.30", "ATHOME_SIM_PERSON": DEFAULT_PERSON, "ATHOME_SIM_POINT_ERR_M": "1.0"},
        expected_state="FAILED",
        expected_failed_step="s3_find_pointed_bag",
    ),
    Scenario(
        name="seguir",
        env={"ATHOME_SIM_BAG": "-2.5,-2.5,0.30", "ATHOME_SIM_PERSON": "2.0,0.0", "ATHOME_SIM_PERSON_POINTS": "0",
             "ATHOME_SIM_PERSON_WALK": "3.5,0.0;3.5,-1.5;2.5,-1.5"},
        expected_state="SUCCEEDED",
        command="follow me",
    ),
    Scenario(
        name="plan_b",
        env={"ATHOME_SIM_BAG": DEFAULT_BAG, "ATHOME_SIM_PERSON": DEFAULT_PERSON, "ATHOME_SIM_GRASP_SLIPS": "1"},
        expected_state="SUCCEEDED",
    ),
    Scenario(
        name="plan_b_sin_manos",
        env={"ATHOME_SIM_BAG": DEFAULT_BAG, "ATHOME_SIM_PERSON": DEFAULT_PERSON, "ATHOME_SIM_GRASP_SLIPS": "1",
             "ATHOME_SIM_PERSON_HANDS": "0"},
        expected_state="FAILED",
        expected_failed_step="s12_receive_bag",
    ),
]

SCENARIO_MAP: Dict[str, Scenario] = {s.name: s for s in SCENARIOS}


def parse_sim_log(text: str) -> Optional[float]:
    """Extrae la distancia en cm de 'Pinza cerrada ... a X cm' del log de sim_robot (primera ocurrencia)."""
    m = GRIP_RE.search(text)
    return float(m.group(1)) if m else None


def parse_client_output(text: str) -> Tuple[str, str, List[Tuple[str, str, str]]]:
    """Lee la salida de commander_client.py: (estado_mision, mensaje_fin, [(step_id, state, msg), ...])."""
    steps: List[Tuple[str, str, str]] = []
    mission_state = "UNKNOWN"
    mission_msg = ""
    for line in text.splitlines():
        sm = STEP_RE.match(line)
        if sm:
            steps.append((sm.group(1), sm.group(2), sm.group(3).strip()))
            continue
        fm = FIN_RE.match(line)
        if fm:
            mission_state = fm.group(1)
            mission_msg = fm.group(2).strip()
        elif line.strip() == "TIMEOUT":
            mission_state = "TIMEOUT"
    return mission_state, mission_msg, steps


def first_failed_step(steps: Sequence[Tuple[str, str, str]]) -> Optional[str]:
    """Devuelve el id del primer paso que termino en FAILED."""
    for step_id, state, _ in steps:
        if state == "FAILED":
            return step_id
    return None


def evaluate(scenario: Scenario, data: RunData) -> EvalResult:
    """Evalua si los datos de ejecucion cumplen la expectativa del escenario."""
    failed_step = first_failed_step(data.steps)
    reasons: List[str] = []

    if data.mission_state != scenario.expected_state:
        reasons.append(f"estado {data.mission_state} != esperado {scenario.expected_state}")

    if scenario.expected_failed_step is not None:
        # Un paso puede fallar y dar lugar a un plan B: vale cualquiera de los fallidos
        if scenario.expected_failed_step not in [st for st, state, _ in data.steps if state == "FAILED"]:
            reasons.append(f"fallo en {failed_step or 'ninguno'} != esperado {scenario.expected_failed_step}")

    if scenario.max_cm is not None:
        if data.grip_cm is None:
            reasons.append(f"sin medicion de pinza (esperado < {scenario.max_cm:g} cm)")
        elif not (data.grip_cm < scenario.max_cm):
            reasons.append(f"pinza a {data.grip_cm:.1f} cm >= {scenario.max_cm:g} cm")

    ok = len(reasons) == 0
    if ok:
        if scenario.expected_state == "SUCCEEDED":
            reason = f"OK ({data.grip_cm:.1f} cm)" if data.grip_cm is not None else "OK"
        else:
            reason = f"OK (fallo esperado en {failed_step})"
    else:
        reason = "; ".join(reasons)
    return EvalResult(scenario=scenario, data=data, ok=ok, failed_step=failed_step, reason=reason)


def select_scenarios(names: Sequence[str]) -> List[Scenario]:
    if not names:
        return list(SCENARIOS)
    unknown = [n for n in names if n not in SCENARIO_MAP]
    if unknown:
        raise ValueError(f"Escenario(s) desconocido(s): {', '.join(unknown)}. Disponibles: {', '.join(SCENARIO_MAP)}")
    return [SCENARIO_MAP[n] for n in names]


def format_table(results: Sequence[EvalResult]) -> str:
    headers = ("Escenario", "Esperado", "Estado", "Paso fallido", "Pinza (cm)", "Tiempo (s)", "Veredicto")
    rows: List[Tuple[str, ...]] = []
    for r in results:
        s = r.scenario
        exp = s.expected_state
        if s.max_cm is not None:
            exp += f" (<{s.max_cm:g}cm)"
        elif s.expected_failed_step:
            exp += f" ({s.expected_failed_step})"
        cm_str = f"{r.data.grip_cm:.1f}" if r.data.grip_cm is not None else "-"
        step_str = r.failed_step or "-"
        verdict = "PASS" if r.ok else f"FAIL: {r.reason}"
        rows.append((s.name, exp, r.data.mission_state, step_str, cm_str, f"{r.data.duration_s:.1f}", verdict))

    widths = [max(len(headers[i]), *(len(row[i]) for row in rows)) if rows else len(headers[i]) for i in range(len(headers))]
    fmt = " | ".join(f"{{:<{w}}}" for w in widths)
    sep = "-+-".join("-" * w for w in widths)
    lines = [fmt.format(*headers), sep]
    for row in rows:
        lines.append(fmt.format(*row))
    passed = sum(1 for r in results if r.ok)
    lines.append(sep)
    lines.append(f"Resumen: {passed}/{len(results)} escenarios PASS")
    return "\n".join(lines)


def _clean_sim_env(base_env: Dict[str, str], scenario_env: Dict[str, str]) -> Dict[str, str]:
    env = {k: v for k, v in base_env.items() if not k.startswith("ATHOME_SIM_")}
    env["ROS_DOMAIN_ID"] = "42"
    env["ATHOME_SIM"] = "1"
    env["ATHOME_SIM_OBJECT_LABEL"] = "handbag"
    env["RVIZ"] = "0"
    env.update(scenario_env)
    return env


def run_scenario(scenario: Scenario, script_path: Path, timeout_s: float = 360.0) -> EvalResult:
    log_root = Path(tempfile.mkdtemp(prefix=f"sim_escenario_{scenario.name}_"))
    env = _clean_sim_env(dict(os.environ), scenario.env)
    env["SIM_LOG_DIR"] = str(log_root)

    t0 = time.time()
    proc = subprocess.Popen(
        [str(script_path), scenario.command],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    try:
        out, _ = proc.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        out, _ = proc.communicate(timeout=30.0)
        out = (out or "") + "\n  TIMEOUT\n"
    finally:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=15.0)
            except Exception:
                pass
    duration = time.time() - t0

    mission_state, mission_msg, steps = parse_client_output(out or "")
    sim_logs = sorted(log_root.glob("sim_commander_*/sim.log"))
    grip_cm: Optional[float] = None
    log_dir = str(sim_logs[-1].parent) if sim_logs else str(log_root)
    if sim_logs:
        grip_cm = parse_sim_log(sim_logs[-1].read_text(encoding="utf-8", errors="replace"))

    data = RunData(
        mission_state=mission_state,
        mission_message=mission_msg,
        steps=steps,
        grip_cm=grip_cm,
        duration_s=duration,
        log_dir=log_dir,
    )
    return evaluate(scenario, data)


def save_results(results: Sequence[EvalResult], path: Path = RESULTS_FILE) -> None:
    """Guarda la ultima corrida de cada escenario para el tablero `debug`."""
    rows = {}
    if path.exists():
        try:
            rows = {r["nombre"]: r for r in json.loads(path.read_text(encoding="utf-8"))["filas"]}
        except (ValueError, KeyError, TypeError):
            rows = {}
    head = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=Path(__file__).resolve().parent,
                          capture_output=True, text=True).stdout.strip()
    when = time.strftime("%Y-%m-%d %H:%M")
    mode = "Gazebo + Nav2" if os.environ.get("NAV2") == "1" else "Gazebo"
    for r in results:
        rows[r.scenario.name] = dict(nombre=r.scenario.name, ok=r.ok, cm=r.data.grip_cm, detalle=r.reason,
                                     cuando=when, modo=mode, commit=head)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(dict(filas=sorted(rows.values(), key=lambda r: r["nombre"]))), encoding="utf-8")


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    try:
        selected = select_scenarios(args)
    except ValueError as e:
        print(str(e), file=sys.stderr)
        return 2

    script_path = Path(__file__).resolve().parent / "sim_commander.sh"
    results: List[EvalResult] = []
    for sc in selected:
        print(f">>> Corriendo escenario '{sc.name}' ({sc.env})...", flush=True)
        res = run_scenario(sc, script_path)
        results.append(res)
        status_tag = "PASS" if res.ok else "FAIL"
        cm_info = f", pinza={res.data.grip_cm:.1f} cm" if res.data.grip_cm is not None else ""
        step_info = f", paso_fallido={res.failed_step}" if res.failed_step else ""
        print(f"    [{status_tag}] {sc.name}: estado={res.data.mission_state}{step_info}{cm_info} ({res.data.duration_s:.1f}s) -> {res.reason}", flush=True)

    print("\n" + format_table(results))
    save_results(results)
    return 0 if all(r.ok for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
