"""Calcula el estado del robot para el tablero. Solo lee: no publica en ROS ni escribe en el repo."""
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import yaml

WS = Path(__file__).resolve().parents[2]
MANIFEST = WS / "tools" / "debug_dashboard" / "capacidades.yaml"
MATRIX = WS / "docs" / "MATRIZ_REGLAS_HABILIDADES.txt"
COMMANDS = WS / "tools" / "gpsr_parser" / "data" / "commands.jsonl"
SCENARIOS = Path.home() / ".local" / "share" / "base_athome" / "sim_escenarios.json"
STAGES = ("robot", "gazebo", "simulador", "tests", "parcial", "falta")
TESTS = {"1": "HRI", "2": "PP", "3": "GPSR", "4": "LAUNDRY"}
READY = ("robot",)
WORKING = ("gazebo", "simulador", "tests", "parcial")


def _git(*args):
    try:
        return subprocess.run(["git", *args], cwd=WS, capture_output=True, text=True, timeout=10).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def parse_matrix(text):
    """Items de la matriz: id, titulo y puntos."""
    items = []
    for m in re.finditer(r"^ITEM (\d)\.(\d+): (.+?)\n(.*?)(?=^ITEM |^-{20,}|^={20,}|\Z)", text, re.M | re.S):
        pts = re.search(r"^- Puntos:\s*(\d+)", m.group(4), re.M)
        # Sin "- Puntos:" (extras), los puntos son la suma de sus lineas "- nombre: N pts".
        total = int(pts.group(1)) if pts else sum(int(x) for x in re.findall(r"^- [^\n]*?: (\d+) pts", m.group(4), re.M))
        items.append(dict(id=f"{m.group(1)}.{m.group(2)}", prueba=TESTS.get(m.group(1), "?"), titulo=m.group(3).strip(), puntos=total))
    return items


def parse_maximums(text):
    """Maximo oficial de cada prueba, segun el titulo de su seccion."""
    return {TESTS[m.group(1)]: int(m.group(2)) for m in re.finditer(r"^2\.(\d)\. .*\(max\. (\d+) pts\)", text, re.M) if m.group(1) in TESTS}


def changed_since(sha, files, git=_git):
    """Archivos de la lista que cambiaron despues de la ultima prueba, incluidos los cambios sin commitear."""
    if not sha or not files:
        return []
    out = git("diff", "--name-only", sha, "--", *files)
    return [f for f in out.splitlines() if f]


def summarize(caps, items, maximums=None):
    """Por prueba: puntos en cada grupo. Un item sin capacidad cuenta como sin clasificar, no como hecho."""
    owner = {i: c for c in caps for i in c.get("items", [])}
    out, unmapped = {}, []
    for it in items:
        cap = owner.get(it["id"])
        it = dict(it, capacidad=cap["nombre"] if cap else None, etapa=cap["etapa"] if cap else "sin_clasificar")
        if not cap:
            unmapped.append(it["id"])
        t = out.setdefault(it["prueba"], dict(items=[], total=0, robot=0, avance=0, falta=0))
        t["items"].append(it)
        t["total"] += it["puntos"]
        key = "robot" if it["etapa"] in READY else "avance" if it["etapa"] in WORKING else "falta"
        t[key] += it["puntos"]
    for name, t in out.items():
        # Lo que el reglamento da y la matriz no desglosa no se cuenta como hecho.
        t["sin_desglosar"] = max(0, (maximums or {}).get(name, t["total"]) - t["total"])
        t["total"] += t["sin_desglosar"]
    return out, unmapped


def registry_skills():
    """Skills del registro real, para avisar si aparece una sin clasificar."""
    code = ("import sys,json;sys.path.insert(0,sys.argv[1]);"
            "from base_athome_skills.mission.skills import default_registry as d;r=d().describe();"
            "print(json.dumps(r if isinstance(r,list) else r.get('skills',r)))")
    try:
        res = subprocess.run([sys.executable, "-c", code, str(WS / "src" / "base_athome_skills")], capture_output=True, text=True, timeout=20)
        return json.loads(res.stdout)
    except (OSError, ValueError, subprocess.SubprocessError):
        return []


COVERAGE_CODE = r"""
import collections,json,os,sys,tempfile,yaml
ws=sys.argv[1];sys.path[:0]=[ws+'/src/base_athome_skills',ws+'/tools/gpsr_parser']
from base_athome_skills.mission.gpsr_plan import gpsr_actions
from base_athome_skills.mission.command_parser import compile_gpsr_actions
from base_athome_skills.mission.places import load_places
L=[json.loads(l) for l in open(ws+'/tools/gpsr_parser/data/commands.jsonl')]
A=[gpsr_actions(d['command']) for d in L]
locs={x['params'].get('location') for a in A for x in a if x['params'].get('location')}
f=tempfile.NamedTemporaryFile('w',suffix='.yaml',delete=False);yaml.safe_dump({'waypoints':{l.replace(' ','_'):dict(name=l,x=0.0,y=0.0,yaw_deg=0) for l in locs|{'home'}}},f);f.close();P=load_places(f.name);os.unlink(f.name)
ok=collections.Counter();tot=collections.Counter();why=collections.Counter()
for d,a in zip(L,A):
    c=compile_gpsr_actions(a,P);tot[d['category']]+=1
    if c.ok: ok[d['category']]+=1
    else:
        e=(c.errors or ['?'])[0]
        why['reconocer personas' if 'reconocer a una persona' in e else 'responder o decir algo' if 'responder' in e else 'contar o describir' if 'implementado' in e else 'otras']+=1
print(json.dumps(dict(ok=ok,total=tot,motivos=why.most_common())))
"""


def gpsr_coverage():
    """Comandos oficiales que compilan hoy, sin LLM. Compilar no significa que se ejecuten bien."""
    try:
        res = subprocess.run(["nice", "-n", "15", sys.executable, "-c", COVERAGE_CODE, str(WS)], capture_output=True, text=True, timeout=120)
        return json.loads(res.stdout.strip().splitlines()[-1])
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return None


def robot_mode():
    """Que esta corriendo ahora, leyendo /proc."""
    seen = {}
    for p in Path("/proc").iterdir():
        if not p.name.isdigit():
            continue
        try:
            cmd = (p / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
            if "bash -c" in cmd or "debug_dashboard" in cmd:
                continue
            env = (p / "environ").read_bytes().replace(b"\0", b"\n").decode(errors="ignore")
        except OSError:
            continue
        name = next((n for n in ("mission_executor", "commander", "bag_finder", "hoverboard_driver", "torso_state", "sim_robot",
                                 "llm_server", "nav2_bringup", "realsense2_camera_node", "stepper_controller") if n in cmd), None)
        if name:
            dom = re.search(r"^ROS_DOMAIN_ID=(\d+)", env, re.M)
            seen[name] = dom.group(1) if dom else "?"
    sim = "sim_robot" in seen or seen.get("mission_executor") not in (None, "10")
    real = any(seen.get(n) == "10" for n in ("mission_executor", "hoverboard_driver", "torso_state"))
    return dict(modo="robot real" if real else "simulación" if sim else "apagado", procesos=seen)


def fingerprint():
    """Cambia si hay algo nuevo que recalcular."""
    status = _git("status", "--porcelain")
    # Las fechas de lo sucio detectan una segunda edicion de un archivo que ya figuraba modificado.
    files = [MANIFEST, MATRIX, COMMANDS, SCENARIOS] + [WS / line[3:].strip('"') for line in status.splitlines()]
    stamps = [f.stat().st_mtime if f.is_file() else 0 for f in files]
    return (tuple(stamps), _git("rev-parse", "HEAD"), status)


def compute():
    data = yaml.safe_load(MANIFEST.read_text(encoding="utf-8"))
    caps = data["capacidades"]
    for c in caps:
        c["cambio"] = changed_since(c.get("probado_en"), c.get("archivos", []))
    text = MATRIX.read_text(encoding="utf-8")
    items = parse_matrix(text)
    tests, unmapped = summarize(caps, items, parse_maximums(text))
    skills = registry_skills()
    counts = {s: sum(1 for c in caps if c["etapa"] == s) for s in STAGES}
    scenarios = json.loads(SCENARIOS.read_text(encoding="utf-8")) if SCENARIOS.exists() else None
    return dict(calculado=time.strftime("%H:%M:%S"), rama=_git("rev-parse", "--abbrev-ref", "HEAD"), commit=_git("log", "-1", "--format=%h %s"),
                sucio=[l[3:] for l in _git("status", "--porcelain").splitlines()], capacidades=caps, conteo=counts, pruebas=tests,
                nombres=data["pruebas"], sin_desglose=data.get("sin_desglose", {}), sin_clasificar=unmapped, skills=skills, cobertura=gpsr_coverage(), escenarios=scenarios)
