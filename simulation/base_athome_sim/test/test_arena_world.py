"""Tests de la ARENA de simulacion: el mundo arena.sdf, su mapa (arena.pgm/.yaml) y su mapa semantico.

Comprueban lo que hace falta para que una misma casa sirva para todas las pruebas: el SDF es valido, las puertas
tienen el ancho declarado, cada waypoint cae en celda libre con el radio del robot (0.44 m) despejado, los puntos de
aproximacion no caen dentro de un mueble y las posiciones de persona y bolsa de los escenarios estan en el living.
"""
import importlib.util
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

from base_athome_skills.mission.places import load_objects, load_places, load_tables, point_in_polygon

SIM_DIR = Path(__file__).resolve().parents[2]
WORLD_SDF = SIM_DIR / "base_athome_sim" / "worlds" / "arena.sdf"
MAP_YAML = SIM_DIR / "config" / "arena.yaml"
MAP_PGM = SIM_DIR / "config" / "arena.pgm"
SEMANTIC_MAP_YAML = SIM_DIR / "base_athome_sim" / "config" / "arena_semantic_map.yaml"
ROBOT_RADIUS_M = 0.44
LASER_Z_M = 0.38


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


hacer_mapa = _load(SIM_DIR / "scripts" / "hacer_mapa.py", "hacer_mapa_arena")


@pytest.fixture(scope="module")
def semantic():
    with open(SEMANTIC_MAP_YAML, encoding="utf-8") as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def map_info():
    with open(MAP_YAML, encoding="utf-8") as f:
        info = yaml.safe_load(f)
    with open(MAP_PGM, "rb") as f:
        assert f.readline().strip() == b"P5"
        w, h = [int(v) for v in f.readline().split()]
        assert int(f.readline()) == 255
        img = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    occupied = img[::-1] < 128       # fila 0 = y minimo, como la grilla de hacer_mapa
    return dict(res=float(info["resolution"]), origin=info["origin"], grid=occupied)


def _cell(map_info, x, y):
    return int((x - map_info["origin"][0]) / map_info["res"]), int((y - map_info["origin"][1]) / map_info["res"])


def _clearance_ok(map_info, x, y, radius):
    """Ninguna celda ocupada a menos de `radius` del punto."""
    grid, res = map_info["grid"], map_info["res"]
    h, w = grid.shape
    c, r = _cell(map_info, x, y)
    n = int(math.ceil(radius / res)) + 1
    for dr in range(-n, n + 1):
        for dc in range(-n, n + 1):
            rr, cc = r + dr, c + dc
            if not (0 <= rr < h and 0 <= cc < w):
                return False
            if grid[rr, cc] and math.hypot(dc * res, dr * res) <= radius:
                return False
    return True


def _gap_m(map_info, x, y, axis):
    """Ancho libre alrededor de (x, y) a lo largo del eje de la puerta hasta tocar celdas ocupadas."""
    grid, res = map_info["grid"], map_info["res"]
    c, r = _cell(map_info, x, y)
    assert not grid[r, c], f"el centro de la puerta ({x}, {y}) esta ocupado"
    lo = hi = 0
    while True:
        rr, cc = (r, c - lo - 1) if axis == "x" else (r - lo - 1, c)
        if grid[rr, cc]:
            break
        lo += 1
    while True:
        rr, cc = (r, c + hi + 1) if axis == "x" else (r + hi + 1, c)
        if grid[rr, cc]:
            break
        hi += 1
    return (lo + hi + 1) * res


# ----------------------------------------------------------------------------- mundo
@pytest.mark.skipif(shutil.which("gz") is None, reason="gz no instalado")
def test_arena_sdf_valid_with_gz():
    res = subprocess.run(["gz", "sdf", "-k", str(WORLD_SDF)], capture_output=True, text=True)
    assert res.returncode == 0, f"gz sdf -k fallo: {res.stderr}\n{res.stdout}"


def test_arena_world_name_matches_file():
    """sim_robot crea bolsa y persona en el mundo que se llama como el archivo (ATHOME_SIM_WORLD)."""
    world = ET.parse(WORLD_SDF).getroot().find("world")
    assert world is not None and world.attrib["name"] == WORLD_SDF.stem


def test_arena_models_are_static_boxes_without_meshes():
    root = ET.parse(WORLD_SDF).getroot()
    names = set()
    for m in root.iter("model"):
        names.add(m.attrib["name"])
        assert m.findtext("static") == "true", f"{m.attrib['name']} no es estatico"
        assert m.find(".//mesh") is None, f"{m.attrib['name']} usa una malla"
    for required in ("paredes", "puerta_calle", "sofa", "mesa_ratona", "cama", "mesada", "estante", "mesa", "barra",
                     "mesa_1", "mesa_2", "mesa_3", "lavarropas", "tacho"):
        assert required in names, f"falta el modelo {required}"


def test_arena_low_coffee_table_is_invisible_to_lidar_and_map(map_info):
    root = ET.parse(WORLD_SDF).getroot()
    mesa = next(m for m in root.iter("model") if m.attrib["name"] == "mesa_ratona")
    size = [float(v) for v in mesa.find(".//collision/geometry/box/size").text.split()]
    assert size[2] < LASER_Z_M, "la mesa ratona tiene que quedar por debajo del lidar: es el obstaculo bajo de la casa"
    cx, cy = [float(v) for v in mesa.findtext("pose").split()[:2]]
    c, r = _cell(map_info, cx, cy)
    assert not map_info["grid"][r, c], "la mesa ratona no debe figurar en el mapa (el lidar real no la ve)"


# ----------------------------------------------------------------------------- mapa
def test_arena_map_matches_world():
    """El .pgm versionado es el que sale de hacer_mapa.py con el origen y tamano documentados."""
    with open(MAP_YAML, encoding="utf-8") as f:
        info = yaml.safe_load(f)
    assert info["image"] == "arena.pgm"
    origin = tuple(float(v) for v in info["origin"][:2])
    with open(MAP_PGM, "rb") as f:
        f.readline()
        w, h = [int(v) for v in f.readline().split()]
        f.readline()
        img = np.frombuffer(f.read(), np.uint8).reshape(h, w)
    grid = hacer_mapa.occupancy(str(WORLD_SDF), origin, (w * info["resolution"], h * info["resolution"]))
    assert grid.shape == (h, w)
    assert np.array_equal(grid, img[::-1] < 128), "regenerar: python3 simulation/scripts/hacer_mapa.py ... arena -4.5 -5.0 16 12"
    assert 8000 < int(grid.sum()) < 20000


def test_arena_doors_have_declared_width(semantic, map_info):
    doors = semantic["doors"]
    assert len(doors) >= 5
    for name, d in doors.items():
        if d.get("closed"):
            c, r = _cell(map_info, d["x"], d["y"])
            assert map_info["grid"][r, c], f"la puerta cerrada {name} tiene que figurar en el mapa"
            continue
        assert 0.8 <= d["width_m"] <= 0.95, f"{name}: ancho fuera de 0.80-0.95 m"
        gap = _gap_m(map_info, d["x"], d["y"], d["axis"])
        assert abs(gap - d["width_m"]) <= 2 * map_info["res"], f"{name}: hueco de {gap:.2f} m, declarado {d['width_m']} m"
        assert gap >= 0.85 - 1e-6, f"{name}: en el mapa quedan {gap:.2f} m; con menos de 0.85 m el robot (0.88 m) no pasa"


# ----------------------------------------------------------------------------- mapa semantico
def test_arena_semantic_waypoints_free_with_robot_radius(semantic, map_info):
    for name, wp in semantic["waypoints"].items():
        assert _clearance_ok(map_info, wp["x"], wp["y"], ROBOT_RADIUS_M), f"waypoint {name} ({wp['x']}, {wp['y']}) pisa un obstaculo"


def test_arena_semantic_has_required_places_and_furniture(semantic):
    places = load_places(str(SEMANTIC_MAP_YAML))
    for wp in ("home", "instruction_point", "door", "hall", "living_room", "kitchen", "bedroom", "dining_room", "laundry",
               "table", "shelf", "kitchen_bar", "trash_drop"):
        assert wp in places, f"falta el waypoint {wp}"
    assert places["home"][:2] == (0.0, 0.0), "home es el origen del mundo: ahi aparece el robot y arranca AMCL"
    tables = load_tables(str(SEMANTIC_MAP_YAML))
    for t in ("mesa", "estante", "barra", "mesa_1", "mesa_2", "mesa_3", "tacho"):
        assert t in tables, f"falta el mueble {t}"
    objects = load_objects(str(SEMANTIC_MAP_YAML))
    assert len(objects) >= 5
    for obj in objects:
        assert not obj["problem"], f"{obj['name']}: {obj['problem']}"


def test_arena_waypoints_inside_their_zone(semantic):
    zones, wps = semantic["zones"], semantic["waypoints"]
    for zone in ("hall", "living_room", "bedroom", "kitchen", "dining_room", "laundry"):
        wp = wps[zone]
        assert point_in_polygon(wp["x"], wp["y"], zones[zone]["polygon"]), f"{zone} no cae en su zona"
    for extra, zone in (("home", "living_room"), ("door", "hall"), ("table", "kitchen"), ("kitchen_bar", "dining_room"),
                        ("trash_drop", "laundry"), ("bed", "bedroom")):
        wp = wps[extra]
        assert point_in_polygon(wp["x"], wp["y"], zones[zone]["polygon"]), f"{extra} deberia estar en {zone}"


def test_arena_furniture_matches_world(semantic):
    """Cada mueble del mapa semantico esta en el mundo con el mismo centro (x, y)."""
    root = ET.parse(WORLD_SDF).getroot()
    poses = {m.attrib["name"]: [float(v) for v in m.findtext("pose").split()[:2]] for m in root.iter("model") if m.findtext("pose")}
    for fid, f in semantic["furniture"].items():
        assert fid in poses, f"{fid} no existe en arena.sdf"
        assert poses[fid] == pytest.approx([f["center"]["x"], f["center"]["y"]], abs=1e-6), f"{fid}: centro distinto"


# ----------------------------------------------------------------------------- escenarios en la arena
def test_arena_trash_scenarios_in_living_room(semantic, map_info):
    se = _load(SIM_DIR / "scripts" / "sim_escenarios.py", "sim_escenarios_arena")
    living = semantic["zones"]["living_room"]["polygon"]
    arena = se.arena_scenarios()
    assert [s.name for s in arena] == [s.name for s in se.SCENARIOS]
    for sc in arena:
        base = se.SCENARIO_MAP[sc.name]
        assert (sc.expected_state, sc.expected_failed_step, sc.command) == (base.expected_state, base.expected_failed_step, base.command)
        px, py = [float(v) for v in sc.env["ATHOME_SIM_PERSON"].split(",")]
        if sc.name != "sin_persona":
            assert point_in_polygon(px, py, living), f"{sc.name}: la persona ({px}, {py}) no esta en el living"
            assert _clearance_ok(map_info, px, py, 0.3), f"{sc.name}: la persona pisa un mueble"
        else:
            assert not point_in_polygon(px, py, living)
        for key in ("ATHOME_SIM_BAG", "ATHOME_SIM_BAG2"):
            if key in sc.env:
                bx, by = [float(v) for v in sc.env[key].split(",")[:2]]
                assert point_in_polygon(bx, by, living), f"{sc.name}: {key} ({bx}, {by}) no esta en el living"
                assert _clearance_ok(map_info, bx, by, 0.2), f"{sc.name}: {key} pisa un mueble"
        for p in sc.env.get("ATHOME_SIM_PERSON_WALK", "").split(";"):
            if p:
                wx, wy = [float(v) for v in p.split(",")]
                assert point_in_polygon(wx, wy, living) and _clearance_ok(map_info, wx, wy, 0.3), f"{sc.name}: camino {p}"


def test_arena_nav_eval_scenarios(semantic, map_info, monkeypatch):
    monkeypatch.setenv("ATHOME_SIM_ARENA", "1")
    ne = _load(SIM_DIR / "scripts" / "nav_eval.py", "nav_eval_arena")
    assert ne.ARENA and set(ne.SCENARIOS) == {"puerta", "persona", "se_corre"}
    places = load_places(str(SEMANTIC_MAP_YAML))
    for name, sc in ne.SCENARIOS.items():
        target = sc["command"].replace("go to the ", "").replace(" ", "_")
        assert target in places, f"{name}: '{sc['command']}' no apunta a un waypoint de la arena"
        px, py = [float(v) for v in sc["person"].split(",")]
        c, r = _cell(map_info, px, py)
        assert not map_info["grid"][r, c], f"{name}: la persona ({px}, {py}) esta dentro de una pared"
    door = semantic["doors"]["living_dining"]
    px, py = [float(v) for v in ne.SCENARIOS["se_corre"]["person"].split(",")]
    assert (px, py) == (door["x"], door["y"]), "se_corre: la persona tiene que estar parada en la puerta living-comedor"


def test_arena_all_waypoints_connected_for_robot_radius(semantic, map_info):
    """Con el mapa engordado por el radio del robot, desde 'home' se llega a todos los waypoints (pasando las puertas)."""
    grid, res = map_info["grid"], map_info["res"]
    h, w = grid.shape
    n = int(math.ceil(ROBOT_RADIUS_M / res))
    blocked = np.zeros_like(grid)
    rows, cols = np.nonzero(grid)
    for dr in range(-n, n + 1):
        for dc in range(-n, n + 1):
            if math.hypot(dr * res, dc * res) < ROBOT_RADIUS_M:        # menor estricto: como el radio inscripto del costmap
                blocked[np.clip(rows + dr, 0, h - 1), np.clip(cols + dc, 0, w - 1)] = True
    c0, r0 = _cell(map_info, 0.0, 0.0)
    reach = np.zeros_like(grid)
    reach[r0, c0] = True
    stack = [(r0, c0)]
    while stack:
        r, c = stack.pop()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < h and 0 <= cc < w and not blocked[rr, cc] and not reach[rr, cc]:
                reach[rr, cc] = True
                stack.append((rr, cc))
    for name, wp in semantic["waypoints"].items():
        c, r = _cell(map_info, wp["x"], wp["y"])
        assert reach[r, c], f"desde home no se llega a {name} con un robot de radio {ROBOT_RADIUS_M} m"
    c, r = _cell(map_info, -2.25, -4.6)
    assert not reach[r, c], "la casa tiene que estar cerrada: no se sale por la puerta de calle"
