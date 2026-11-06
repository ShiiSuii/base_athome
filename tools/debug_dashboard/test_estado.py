"""Tests del tablero. Tambien obligan a mantener el manifiesto al dia con la matriz y el repo."""
import subprocess
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).parent))
import estado  # noqa: E402

MATRIX = """2.2. PICK AND PLACE CHALLENGE (max. 500 pts)
---------------
ITEM 2.1: Navigate
- Puntos: 15 pts

ITEM 2.2: Pick
- Puntos: 100 pts max (2 x 50)

ITEM 2.6: Extras
- Open: 200 pts max (2 x 100 pts por abrir).
  * Cubierto por: 999 pts
- Pour: 100 pts (1x).

PENALIDADES:
- Dropped: 12 x -40 pts.
"""


def test_matrix_items_and_extras_points():
    items = {i["id"]: i for i in estado.parse_matrix(MATRIX)}
    assert items["2.1"]["puntos"] == 15 and items["2.2"]["puntos"] == 100
    assert items["2.6"]["puntos"] == 300 and items["2.1"]["prueba"] == "PP"


def test_points_are_grouped_by_real_progress_and_not_invented():
    caps = [dict(nombre="a", etapa="robot", items=["2.1"]), dict(nombre="b", etapa="parcial", items=["2.2"])]
    tests, unmapped = estado.summarize(caps, estado.parse_matrix(MATRIX), estado.parse_maximums(MATRIX))
    pp = tests["PP"]
    assert (pp["robot"], pp["avance"], pp["falta"], pp["sin_desglosar"], pp["total"]) == (15, 100, 300, 85, 500)
    assert unmapped == ["2.6"]


def test_change_after_last_test_is_flagged():
    calls = []

    def git(*args):
        calls.append(args)
        return "a.py\n"
    assert estado.changed_since("abc", ["a.py", "b.py"], git) == ["a.py"]
    assert calls == [("diff", "--name-only", "abc", "--", "a.py", "b.py")]
    assert estado.changed_since(None, ["a.py"], git) == [] and estado.changed_since("abc", [], git) == []


def test_manifest_matches_the_repo():
    data = yaml.safe_load(estado.MANIFEST.read_text(encoding="utf-8"))
    caps = data["capacidades"]
    assert len({c["id"] for c in caps}) == len(caps)
    owners = [i for c in caps for i in c.get("items", [])]
    assert len(set(owners)) == len(owners), "cada item va en una sola capacidad"
    assert set(owners) == {i["id"] for i in estado.parse_matrix(estado.MATRIX.read_text(encoding="utf-8"))}
    for c in caps:
        assert c["etapa"] in estado.STAGES, c["id"]
        assert c["etapa"] == "falta" or c.get("falta") is not None or c["etapa"] == "robot"
        for f in c.get("archivos", []):
            assert subprocess.run(["git", "ls-files", f], cwd=estado.WS, capture_output=True, text=True).stdout, f
        sha = c.get("probado_en")
        if sha:
            assert subprocess.run(["git", "cat-file", "-e", sha + "^{commit}"], cwd=estado.WS).returncode == 0, c["id"]


def test_scenario_results_keep_the_last_run_of_each(tmp_path):
    from types import SimpleNamespace as N
    sys.path.insert(0, str(estado.WS / "simulation" / "scripts"))
    import sim_escenarios as se
    import json
    path = tmp_path / "r.json"
    row = lambda name, ok: N(scenario=N(name=name), ok=ok, reason="", data=N(grip_cm=None))  # noqa: E731
    se.save_results([row("a", False), row("b", True)], path)
    se.save_results([row("a", True)], path)
    rows = {r["nombre"]: r["ok"] for r in json.loads(path.read_text())["filas"]}
    assert rows == {"a": True, "b": True}
