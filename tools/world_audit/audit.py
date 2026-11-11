#!/usr/bin/env python3
"""Auditor automatico de los mundos de Gazebo de Base@Home / Robbie@Home.

Objetivo: que lo que funciona en simulacion se parezca a la arena de RoboCup@Home
y a una casa real, y que los defectos geometricos se detecten antes de una corrida.

El auditor es Python puro (mas numpy; scipy es opcional) y NO arranca ROS ni Gazebo.
Parsea los SDF, los mapas semanticos y los escenarios, y reporta:

  * objetos que se intersectan o atraviesan paredes;
  * objetos flotando o hundidos respecto de la superficie que los soporta;
  * muebles con medidas fuera del rango de referencia (mesa, barra, silla, sofa, cama);
  * puertas y pasillos con margen insuficiente para el robot (radio 0.44 m);
  * waypoints / approach points dentro de muebles, fuera del mundo o inalcanzables;
  * incoherencias entre el mapa semantico y los modelos visibles;
  * clientes/invitados sentados sin silla o dentro de un mueble/pared.

Uso:
  python3 tools/world_audit/audit.py                     # todos los mundos con mapa asociado
  python3 tools/world_audit/audit.py arena.sdf nav_eval.sdf
  python3 tools/world_audit/audit.py --json /tmp/w.json --tsv /tmp/w.tsv --strict

Criterio de salida: rc=0 si no hay ERROR (los WARN se informan siempre);
con --strict tambien falla ante cualquier WARN. Un INFO nunca falla.

Referencias de medida (fuente explicita; NO VERIFICADO cuando no hay norma):
  - Mesa de comedor 0.72-0.76 m: docs/INVESTIGACION_PICK_PLACE.txt:185
    ("La tabla de una mesa de comedor estandar esta entre 72 y 76 cm").
  - Mesa 0.72-0.78 m: rango conservador del pedido de la tarea (incluye 0.78).
  - Barra / mostrador 0.90-1.10 m: pedido de la tarea. NO VERIFICADO en RuleBook.
  - Silla (asiento) 0.40-0.48 m, sofa 0.40-0.50 m, cama 0.45-0.60 m: pedido de la tarea.
    NO VERIFICADO en RuleBook; son alturas tipicas de mobiliario.
  - Puerta de arena: ancho minimo 900 mm, altura minima 2000 mm:
    docs/INVESTIGACION_REGLAS_GENERALES.txt:194-197 (RuleBook 2026.2,
    pages/appendices/LeagueRequirements.tex). Una puerta de 0.90 m deja
    (0.90 - 2*0.44)/2 = 0.01 m de margen por lado.
  - Radio de navegacion 0.44 m y lidar a 0.38 m: docs/ROBOT.md, y
    /home/athome/sim_lab/specs/conocimiento_robot.txt:11,35.

Todo dato que no provenga de esas fuentes se marca NO VERIFICADO en el hallazgo.
"""
from __future__ import annotations

import argparse
import dataclasses
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np
import yaml

WS = Path(__file__).resolve().parents[2]
SIM = WS / "simulation"
WORLDS = SIM / "base_athome_sim" / "worlds"
SEM_DIR = SIM / "base_athome_sim" / "config"
MAP_DIR = SIM / "config"

ROBOT_RADIUS_M = 0.44
LASER_Z_M = 0.38
MAP_RES_M = 0.05

# (regex del nombre, tipo, (min, max) en metros, fuente)
MEASURE_RANGES = (
    (r"^mesa_ratona$", "mesa_ratona", (0.30, 0.45), "mueble bajo tipico; NO VERIFICADO en RuleBook"),
    (r"^mesada$|^barra$|^mostrador$|^counter$|^kitchen_bar$", "barra", (0.90, 1.10),
     "pedido de la tarea; NO VERIFICADO en RuleBook"),
    (r"^mesa(_\d+.*)?$", "mesa", (0.72, 0.78),
     "docs/INVESTIGACION_PICK_PLACE.txt:185 (72-76 cm) + margen 0.78 del pedido"),
    (r"^silla", "silla", (0.40, 0.48), "altura de asiento tipica; NO VERIFICADO en RuleBook"),
    (r"^sofa$|^sillon$", "sofa", (0.40, 0.50), "pedido de la tarea; NO VERIFICADO en RuleBook"),
    (r"^cama$", "cama", (0.45, 0.60), "pedido de la tarea; NO VERIFICADO en RuleBook"),
)

# Mundos con mapa semantico asociado. El mundo restaurant.sdf pertenece a Codex
# Restaurant y su mapa se arma con SLAM; igual se audita, pero no se edita.
SEMANTIC_PAIRS = {
    "arena.sdf": "arena_semantic_map.yaml",
    "nav_eval.sdf": "nav_eval_semantic_map.yaml",
    "casa_pick_place.sdf": "sim_pick_place_semantic_map.yaml",
    "casa_simple.sdf": "receptionist_semantic_map.yaml",
    "casa_basura.sdf": None,
    "restaurant.sdf": None,
}

WALL_NAMES = {"paredes", "walls", "pared"}
FLOOR_NAMES = {"piso", "suelo", "floor"}


# --------------------------------------------------------------------------- geometria
def parse_pose(text: str | None) -> list[float]:
    vals = [float(v) for v in (text or "0 0 0 0 0 0").split()]
    return vals + [0.0] * (6 - len(vals))


def rot_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], float)
    return rz @ ry @ rx


def compose_pose(parent: tuple[np.ndarray, np.ndarray], pose: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    """Compone (R,t) padre con una pose local x y z roll pitch yaw."""
    rp, tp = parent
    rl = rot_matrix(*pose[3:6])
    return rp @ rl, rp @ np.array(pose[:3], float) + tp


@dataclasses.dataclass(frozen=True)
class AABB:
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]

    @property
    def center(self) -> tuple[float, float, float]:
        return tuple((a + b) / 2.0 for a, b in zip(self.lo, self.hi))  # type: ignore[return-value]

    @property
    def size(self) -> tuple[float, float, float]:
        return tuple(b - a for a, b in zip(self.lo, self.hi))  # type: ignore[return-value]

    def overlap(self, other: "AABB") -> tuple[float, float, float]:
        return tuple(min(a1, b1) - max(a0, b0) for a0, a1, b0, b1 in
                     zip(self.lo, self.hi, other.lo, other.hi))  # type: ignore[return-value]

    def footprint_area(self) -> float:
        return max(0.0, self.size[0]) * max(0.0, self.size[1])

    def footprint_overlap_area(self, other: "AABB") -> float:
        ox = min(self.hi[0], other.hi[0]) - max(self.lo[0], other.lo[0])
        oy = min(self.hi[1], other.hi[1]) - max(self.lo[1], other.lo[1])
        return max(0.0, ox) * max(0.0, oy)


@dataclasses.dataclass
class Solid:
    model: str
    part: str
    kind: str
    aabb: AABB
    static: bool


@dataclasses.dataclass
class Model:
    name: str
    static: bool
    solids: list[Solid] = dataclasses.field(default_factory=list)
    has_plane: bool = False
    has_mesh: bool = False
    line: int = 0

    @property
    def aabb(self) -> AABB | None:
        if not self.solids:
            return None
        return AABB(tuple(min(s.aabb.lo[i] for s in self.solids) for i in range(3)),
                    tuple(max(s.aabb.hi[i] for s in self.solids) for i in range(3)))

    @property
    def is_wall(self) -> bool:
        return (self.name in WALL_NAMES or self.name.startswith("pared")
                or self.name.startswith("puerta"))


@dataclasses.dataclass
class World:
    path: Path
    name: str
    models: dict[str, Model]
    per_line: dict[str, int] = dataclasses.field(default_factory=dict)

    def wall_models(self) -> list[Model]:
        return [m for m in self.models.values() if m.is_wall]

    def world_bounds(self) -> tuple[float, float, float, float]:
        walls = [m.aabb for m in self.wall_models() if m.aabb]
        if not walls:
            return (-5.0, -5.0, 5.0, 5.0)
        return (min(a.lo[0] for a in walls), min(a.lo[1] for a in walls),
                max(a.hi[0] for a in walls), max(a.hi[1] for a in walls))


def _model_lines(path: Path) -> dict[str, int]:
    lines: dict[str, int] = {}
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return lines
    for i, line in enumerate(text.splitlines(), 1):
        m = re.search(r'<model\b[^>]*\bname="([^"]+)"', line)
        if m and m.group(1) not in lines:
            lines[m.group(1)] = i
    return lines


def _geometry_aabb(geom: ET.Element) -> tuple[str, AABB | None]:
    box = geom.find("box/size")
    if box is not None and box.text:
        sx, sy, sz = (float(v) for v in box.text.split()[:3])
        return "box", AABB((-sx / 2, -sy / 2, -sz / 2), (sx / 2, sy / 2, sz / 2))
    cyl = geom.find("cylinder")
    if cyl is not None:
        r = float(cyl.findtext("radius") or 0.0)
        length = float(cyl.findtext("length") or 0.0)
        return "cylinder", AABB((-r, -r, -length / 2), (r, r, length / 2))
    sph = geom.find("sphere/radius")
    if sph is not None:
        r = float(sph.text or 0.0)
        return "sphere", AABB((-r, -r, -r), (r, r, r))
    plane = geom.find("plane")
    if plane is not None:
        return "plane", None
    if geom.find("mesh") is not None:
        return "mesh", None
    return "unknown", None


def _transform_aabb(R: np.ndarray, t: np.ndarray, box: AABB) -> AABB | None:
    if box is None:
        return None
    corners = np.array([[x, y, z] for x in (box.lo[0], box.hi[0])
                        for y in (box.lo[1], box.hi[1]) for z in (box.lo[2], box.hi[2])], float)
    pts = corners @ R.T + t
    return AABB(tuple(pts.min(axis=0)), tuple(pts.max(axis=0)))


def load_world(path: Path, include_dirs: Sequence[Path] = ()) -> World:
    root = ET.parse(path).getroot()
    world_el = root.find("world")
    if world_el is None:
        raise ValueError(f"{path}: no tiene <world>")
    name = world_el.attrib.get("name", path.stem)
    models: dict[str, Model] = {}
    lines = _model_lines(path)

    def walk(parent_el: ET.Element, parent_T: tuple[np.ndarray, np.ndarray],
             inherited_static: bool, prefix: str) -> None:
        for mel in parent_el.findall("model"):
            mname = mel.attrib.get("name", "model")
            full = f"{prefix}{mname}"
            static = (mel.findtext("static") or "").lower() == "true" or inherited_static
            pose = parent_T
            if mel.find("pose") is not None:
                pose = compose_pose(parent_T, parse_pose(mel.findtext("pose")))
            model = Model(name=full, static=static, line=lines.get(mname, 0))
            for link in mel.findall("link"):
                link_T = pose
                if link.find("pose") is not None:
                    link_T = compose_pose(pose, parse_pose(link.findtext("pose")))
                for col in link.findall("collision"):
                    col_T = link_T
                    if col.find("pose") is not None:
                        col_T = compose_pose(link_T, parse_pose(col.findtext("pose")))
                    geom = col.find("geometry")
                    if geom is None:
                        continue
                    kind, local = _geometry_aabb(geom)
                    if kind == "plane":
                        model.has_plane = True
                        continue
                    if kind == "mesh":
                        model.has_mesh = True
                        continue
                    if local is None:
                        continue
                    world_box = _transform_aabb(col_T[0], col_T[1], local)
                    if world_box is not None:
                        model.solids.append(Solid(full, col.attrib.get("name", "c"), kind, world_box, static))
            if model.solids or model.has_plane or model.has_mesh:
                models[full] = model
            walk(mel, pose, static, full + "::")

    walk(world_el, (np.eye(3), np.zeros(3)), False, "")

    # <include> resolubles (model:// o file://); hoy los mundos no usan includes,
    # pero el auditor no debe ignorarlos en silencio.
    for inc in world_el.findall("include"):
        uri = (inc.findtext("uri") or "").strip()
        found = None
        if uri.startswith("model://"):
            rel = uri[len("model://"):]
            for base in (path.parent / "models", *include_dirs):
                cand = base / rel / "model.sdf"
                if cand.exists():
                    found = cand
                    break
        elif uri.startswith("file://"):
            cand = Path(uri[len("file://"):])
            found = cand if cand.exists() else None
        if found is None:
            continue
        sub = load_world(_wrap_model_as_world(found), include_dirs)
        inc_pose = parse_pose(inc.findtext("pose"))
        R, t = rot_matrix(*inc_pose[3:6]), np.array(inc_pose[:3], float)
        for sm in sub.models.values():
            solids = []
            for s in sm.solids:
                aabb = _transform_aabb(R, t, s.aabb)
                if aabb is not None:
                    solids.append(Solid(s.model, s.part, s.kind, aabb, s.static))
            if solids:
                models[sm.name] = Model(sm.name, sm.static, solids, sm.has_plane, sm.has_mesh, sm.line)
    return World(path=path, name=name, models=models, per_line=lines)


def _wrap_model_as_world(model_sdf: Path) -> Path:
    """Devuelve un SDF temporal con <world><model>...</model></world> a partir de un model.sdf."""
    text = model_sdf.read_text(encoding="utf-8")
    if "<world" in text:
        return model_sdf
    tmp = model_sdf.with_suffix(".world.tmp.sdf")
    body = re.sub(r"^.*?<model\b", "<model", text, flags=re.S)
    tmp.write_text('<?xml version="1.0"?><sdf version="1.9"><world name="inc">' + body + "</world></sdf>",
                   encoding="utf-8")
    return tmp


def rasterize(world: World, origin: tuple[float, float], size: tuple[float, float],
              res: float = MAP_RES_M, z: float = LASER_Z_M) -> np.ndarray:
    """Ocupacion a la altura del lidar (igual criterio que simulation/scripts/hacer_mapa.py)."""
    w, h = int(round(size[0] / res)), int(round(size[1] / res))
    grid = np.zeros((h, w), bool)
    for model in world.models.values():
        if not model.static:
            continue
        for s in model.solids:
            if not (s.aabb.lo[2] <= z <= s.aabb.hi[2]):
                continue
            c0 = int(math.floor((s.aabb.lo[0] - origin[0]) / res))
            c1 = int(math.ceil((s.aabb.hi[0] - origin[0]) / res))
            r0 = int(math.floor((s.aabb.lo[1] - origin[1]) / res))
            r1 = int(math.ceil((s.aabb.hi[1] - origin[1]) / res))
            if c1 <= 0 or r1 <= 0 or c0 >= w or r0 >= h:
                continue
            grid[max(0, r0):min(h, r1), max(0, c0):min(w, c1)] = True
    return grid


def clearance_grid(grid: np.ndarray, res: float = MAP_RES_M) -> np.ndarray:
    """Distancia (m) de cada celda libre al obstaculo mas cercano. scipy si esta; si no, BFS."""
    try:
        from scipy.ndimage import distance_transform_edt  # type: ignore
        padded = np.pad(grid, 1, constant_values=True)
        dist = distance_transform_edt(~padded)[1:-1, 1:-1] * res
        return dist
    except Exception:  # pragma: no cover - scipy suele estar instalado
        h, w = grid.shape
        INF = 1e9
        dist = np.full((h, w), INF)
        from collections import deque
        q: deque[tuple[int, int]] = deque()
        for r in range(h):
            for c in range(w):
                if grid[r, c]:
                    dist[r, c] = 0.0
                    q.append((r, c))
        while q:
            r, c = q.popleft()
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w:
                    step = res * math.hypot(dr, dc)
                    if dist[r, c] + step < dist[rr, cc]:
                        dist[rr, cc] = dist[r, c] + step
                        q.append((rr, cc))
        return dist


def reachable(dist: np.ndarray, start: tuple[int, int], radius: float) -> np.ndarray:
    h, w = dist.shape
    free = dist >= radius
    out = np.zeros_like(free)
    r0, c0 = start
    if not (0 <= r0 < h and 0 <= c0 < w) or not free[r0, c0]:
        return out
    stack = [(r0, c0)]
    out[r0, c0] = True
    while stack:
        r, c = stack.pop()
        for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if 0 <= rr < h and 0 <= cc < w and free[rr, cc] and not out[rr, cc]:
                out[rr, cc] = True
                stack.append((rr, cc))
    return out


def point_in_polygon(x: float, y: float, polygon: Sequence[Sequence[float]]) -> bool:
    inside = False
    pts = list(polygon)
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = float(pts[i][0]), float(pts[i][1])
        xj, yj = float(pts[j][0]), float(pts[j][1])
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
        j = i
    return inside


# --------------------------------------------------------------------------- hallazgos
@dataclasses.dataclass
class Finding:
    level: str
    code: str
    world: str
    subject: str
    where: str
    message: str

    def as_dict(self) -> dict:
        return dataclasses.asdict(self)


def _add(out: list[Finding], level: str, code: str, world: World, subject: str, message: str,
         line: int | None = None) -> None:
    ln = line if line is not None else world.models.get(subject, Model(subject, True)).line
    where = f"{world.path}:{ln}" if ln else str(world.path)
    out.append(Finding(level, code, world.name, subject, where, message))


def _kind_and_range(name: str):
    for pattern, kind, rng, source in MEASURE_RANGES:
        if re.search(pattern, name):
            return kind, rng, source
    return None, None, None


def _solid_extent(world: World) -> float | None:
    return None


def infer_doors(world: World) -> list[dict]:
    """Detecta huecos entre tramos de pared sobre una misma linea (puertas)."""
    groups: dict[tuple[str, float], list[tuple[float, float]]] = {}
    for model in world.wall_models():
        for s in model.solids:
            sx, sy = s.aabb.size[0], s.aabb.size[1]
            cx, cy = s.aabb.center[0], s.aabb.center[1]
            if sx <= 0.16 and sy >= 0.30:
                # pared delgada en x: el hueco corre a lo largo de y
                groups.setdefault(("y", round(cx, 2)), []).append((s.aabb.lo[1], s.aabb.hi[1]))
            elif sy <= 0.16 and sx >= 0.30:
                # pared delgada en y: el hueco corre a lo largo de x
                groups.setdefault(("x", round(cy, 2)), []).append((s.aabb.lo[0], s.aabb.hi[0]))
    doors: list[dict] = []
    for (axis, coord), intervals in groups.items():
        intervals.sort()
        if len(intervals) < 2:
            continue
        for (lo1, hi1), (lo2, hi2) in zip(intervals, intervals[1:]):
            gap = lo2 - hi1
            if 0.60 <= gap <= 1.60:
                center = (hi1 + lo2) / 2.0
                # axis es la direccion del hueco (la que recorre la pared)
                doors.append(dict(width_m=gap, axis=axis,
                                  x=center if axis == "x" else coord,
                                  y=center if axis == "y" else coord,
                                  source="inferida del SDF"))
    return doors


def audit_world(world: World, semantic_path: Path | None = None,
                map_yaml: Path | None = None) -> list[Finding]:
    out: list[Finding] = []
    bounds = world.world_bounds()

    # --- estructura basica
    if not any(m.has_plane for m in world.models.values()):
        _add(out, "ERROR", "NO_FLOOR", world, world.name, "no hay un piso (plane) en el mundo")
    for m in world.models.values():
        if m.has_mesh:
            _add(out, "WARN", "MESH", world, m.name,
                 "usa una malla: el mapa offline (hacer_mapa.py) no la rasteriza y puede no coincidir")
        if not m.static and m.name not in FLOOR_NAMES:
            _add(out, "INFO", "DYNAMIC_MODEL", world, m.name,
                 "modelo dinamico: no entra en el mapa de ocupacion; verificar que sea intencional")

    # --- intersecciones
    furn = [(n, m) for n, m in world.models.items() if m.aabb and not m.is_wall and not m.has_plane]
    walls = [(n, m) for n, m in world.models.items() if m.is_wall and m.aabb]
    for i, (n1, m1) in enumerate(furn):
        for n2, m2 in furn[i + 1:]:
            ov = m1.aabb.overlap(m2.aabb)  # type: ignore[union-attr]
            if all(v > 0.005 for v in ov):
                chair_table = (_kind_and_range(n1)[0] == "silla" and _kind_and_range(n2)[0] == "mesa") or \
                              (_kind_and_range(n2)[0] == "silla" and _kind_and_range(n1)[0] == "mesa")
                if chair_table and min(ov) <= 0.06:
                    _add(out, "WARN", "INTERSECT", world, f"{n1} / {n2}",
                         f"silla y mesa comparten {min(ov)*100:.1f} cm de planta: "
                         f"la mesa del SDF es una caja solida sin hueco bajo el tablero")
                else:
                    _add(out, "ERROR", "INTERSECT", world, f"{n1} / {n2}",
                         f"cajas envolventes 3D se intersectan {ov[0]*100:.1f} x {ov[1]*100:.1f} x {ov[2]*100:.1f} cm")
    # Contra las cajas individuales de pared, no contra el AABB del modelo completo
    # (paredes es un solo modelo con todos los tramos de la casa).
    wall_solids = [(m.name, s) for _, m in walls for s in m.solids]
    for n1, m1 in furn:
        for owner, s in wall_solids:
            ov = m1.aabb.overlap(s.aabb)  # type: ignore[union-attr]
            if all(v > 0.005 for v in ov):
                pen = max(ov)
                level = "ERROR" if pen > 0.02 else "INFO"
                _add(out, level, "WALL_PENETRATION", world, f"{n1} / {owner}",
                     f"{n1} atraviesa la pared {owner}/{s.part} {pen*100:.1f} cm" if pen > 0.02
                     else f"{n1} toca la pared {owner}/{s.part} ({pen*100:.1f} cm, contacto)")

    # --- flotando / hundido
    for name, m in furn:
        a = m.aabb
        assert a is not None
        bottom = a.lo[2]
        if bottom < -0.02:
            _add(out, "ERROR", "SUNK", world, name, f"base a {bottom:.3f} m: hundido en el piso")
            continue
        if bottom <= 0.02:
            continue
        support_top = None
        support_name = "piso"
        sunk_reported = False
        for other_name, other in world.models.items():
            if other_name == name or other.is_wall or other.has_plane:
                continue
            for s in other.solids:
                oa = s.aabb
                area = a.footprint_overlap_area(oa)
                if a.footprint_area() <= 0 or area / a.footprint_area() < 0.30:
                    continue
                if oa.hi[2] > bottom + 0.02 and oa.lo[2] < bottom:
                    _add(out, "ERROR", "SUNK", world, name,
                         f"base a {bottom:.3f} m, por debajo de la superficie de {other_name} "
                         f"({oa.hi[2]:.3f} m)")
                    sunk_reported = True
                    break
                if oa.hi[2] <= bottom + 0.02 and (support_top is None or oa.hi[2] > support_top):
                    support_top, support_name = oa.hi[2], other_name
            if sunk_reported:
                break
        if sunk_reported:
            continue
        if support_top is None:
            _add(out, "ERROR", "FLOATING", world, name,
                 f"base a {bottom:.3f} m y sin superficie que lo sostenga")
        elif bottom - support_top > 0.03:
            _add(out, "ERROR", "FLOATING", world, name,
                 f"flota {100*(bottom-support_top):.1f} cm sobre {support_name} ({support_top:.3f} m)")

    # --- medidas
    for name, m in world.models.items():
        if m.is_wall or m.has_plane or m.aabb is None:
            continue
        kind, rng, source = _kind_and_range(name)
        if kind is None:
            continue
        height = m.aabb.size[2]
        lo, hi = rng  # type: ignore[misc]
        if height < lo - 0.02 or height > hi + 0.02:
            _add(out, "ERROR", "HEIGHT_RANGE", world, name,
                 f"{kind} de {height:.3f} m fuera de {lo:.2f}-{hi:.2f} m ({source})")
        elif height < lo or height > hi:
            _add(out, "WARN", "HEIGHT_RANGE", world, name,
                 f"{kind} de {height:.3f} m apenas fuera de {lo:.2f}-{hi:.2f} m ({source})")

    # --- puertas
    inferred = infer_doors(world)
    semantic: dict = {}
    if semantic_path and semantic_path.exists():
        with open(semantic_path, encoding="utf-8") as f:
            semantic = yaml.safe_load(f) or {}
    declared = semantic.get("doors") or {}
    if declared:
        # Preferir las puertas declaradas y completar con las inferidas que no coincidan.
        doors = []
        unused = list(inferred)
        for name, d in declared.items():
            match = None
            for cand in unused:
                if cand["axis"] == d.get("axis") and math.hypot(cand["x"] - float(d["x"]), cand["y"] - float(d["y"])) < 0.20:
                    match = cand
                    break
            if match:
                unused.remove(match)
            doors.append(dict(width_m=d.get("width_m"), axis=d.get("axis"), x=d.get("x"), y=d.get("y"),
                              name=name, closed=bool(d.get("closed")), source=str(semantic_path)))
        doors.extend(unused)
    else:
        doors = inferred
    for d in doors:
        width = d.get("width_m")
        if width is None or d.get("closed"):
            continue
        margin_side = (width - 2 * ROBOT_RADIUS_M) / 2
        label = d.get("name") or f"puerta inferida en ({d.get('x'):.2f}, {d.get('y'):.2f})"
        if width < 2 * ROBOT_RADIUS_M:
            _add(out, "ERROR", "DOOR_NARROW", world, label,
                 f"ancho {width:.2f} m < diametro del robot {2*ROBOT_RADIUS_M:.2f} m: no pasa "
                 f"(RuleBook 2026.2 exige >= 0.90 m para la puerta de arena, "
                 f"docs/INVESTIGACION_REGLAS_GENERALES.txt:194-197)")
        elif margin_side < 0.05:
            _add(out, "WARN", "DOOR_RISK", world, label,
                 f"ancho {width:.2f} m: margen de {margin_side*100:.1f} cm por lado con radio "
                 f"{ROBOT_RADIUS_M:.2f} m; Nav2 puede marcar la puerta como intransitable (radio/ inflado)")

    # --- mapa / waypoints / coherencia
    if semantic:
        out.extend(_audit_semantic(world, semantic, semantic_path, map_yaml, bounds))

    return out


def _map_extent(world: World, semantic: dict, map_yaml: Path | None) -> tuple[tuple[float, float], tuple[float, float], Path | None]:
    info = semantic.get("map_info") or {}
    yaml_file = info.get("yaml_file")
    if map_yaml is None and yaml_file:
        cand = WS / yaml_file
        if cand.exists():
            map_yaml = cand
    if map_yaml and map_yaml.exists():
        with open(map_yaml, encoding="utf-8") as f:
            meta = yaml.safe_load(f) or {}
        origin = tuple(float(v) for v in meta.get("origin", [-5.0, -5.0, 0.0])[:2])
        img = MAP_DIR / str(meta.get("image", ""))
        res = float(meta.get("resolution", MAP_RES_M))
        if img.exists():
            with open(img, "rb") as f:
                assert f.readline().strip() == b"P5"
                w, h = (int(v) for v in f.readline().split())
            return (origin[0], origin[1]), (w * res, h * res), map_yaml
    lo_x, lo_y, hi_x, hi_y = world.world_bounds()
    return (lo_x - 1.0, lo_y - 1.0), (hi_x - lo_x + 2.0, hi_y - lo_y + 2.0), None


def _audit_semantic(world: World, semantic: dict, semantic_path: Path,
                    map_yaml: Path | None, bounds) -> list[Finding]:
    out: list[Finding] = []
    origin, size, map_yaml = _map_extent(world, semantic, map_yaml)
    grid = rasterize(world, origin, size)
    dist = clearance_grid(grid)
    res = MAP_RES_M
    h, w = grid.shape

    def cell(x: float, y: float) -> tuple[int, int] | None:
        c = int((x - origin[0]) / res)
        r = int((y - origin[1]) / res)
        if 0 <= r < h and 0 <= c < w:
            return r, c
        return None

    # coherencia mueble <-> modelo
    furniture = semantic.get("furniture") or {}
    for fid, f in furniture.items():
        model = world.models.get(fid)
        if model is None or model.aabb is None:
            _add(out, "ERROR", "FURNITURE_MISMATCH", world, fid,
                 f"el mapa semantico define '{fid}' pero no existe como modelo en el SDF")
            continue
        cx, cy = model.aabb.center[0], model.aabb.center[1]
        fc = f.get("center") or {}
        if abs(cx - float(fc.get("x", cx))) > 0.02 or abs(cy - float(fc.get("y", cy))) > 0.02:
            _add(out, "ERROR", "FURNITURE_MISMATCH", world, fid,
                 f"centro semantico ({fc.get('x')}, {fc.get('y')}) != centro del modelo ({cx:.2f}, {cy:.2f})")
        poly = f.get("polygon")
        if poly:
            xs = [float(p[0]) for p in poly]
            ys = [float(p[1]) for p in poly]
            if (abs(min(xs) - model.aabb.lo[0]) > 0.05 or abs(max(xs) - model.aabb.hi[0]) > 0.05 or
                    abs(min(ys) - model.aabb.lo[1]) > 0.05 or abs(max(ys) - model.aabb.hi[1]) > 0.05):
                _add(out, "ERROR", "FURNITURE_MISMATCH", world, fid,
                     f"poligono semantico [{min(xs):.2f},{min(ys):.2f}]-[{max(xs):.2f},{max(ys):.2f}] "
                     f"!= huella del modelo [{model.aabb.lo[0]:.2f},{model.aabb.lo[1]:.2f}]-"
                     f"[{model.aabb.hi[0]:.2f},{model.aabb.hi[1]:.2f}]")
        h_sem = f.get("height_m")
        if h_sem is not None and abs(float(h_sem) - model.aabb.size[2]) > 0.03:
            _add(out, "ERROR", "FURNITURE_MISMATCH", world, fid,
                 f"altura semantica {float(h_sem):.2f} m != altura del modelo {model.aabb.size[2]:.2f} m")

    # waypoints y puntos de aproximacion
    home = (semantic.get("waypoints") or {}).get("home") or {"x": 0.0, "y": 0.0}
    start = cell(float(home.get("x", 0.0)), float(home.get("y", 0.0)))
    reach = reachable(dist, start, ROBOT_RADIUS_M) if start else np.zeros_like(grid)
    points: list[tuple[str, float, float]] = []
    for name, wp in (semantic.get("waypoints") or {}).items():
        points.append((f"waypoint {name}", float(wp["x"]), float(wp["y"])))
    for oid, obj in (semantic.get("objects") or {}).items():
        ap = obj.get("approach")
        if ap:
            points.append((f"approach {oid}", float(ap["x"]), float(ap["y"])))
    for label, x, y in points:
        cr = cell(x, y)
        if cr is None:
            _add(out, "ERROR", "WAYPOINT_OUTSIDE", world, label,
                 f"({x:.2f}, {y:.2f}) cae fuera del mapa {map_yaml or origin}")
            continue
        r, c = cr
        if dist[r, c] < ROBOT_RADIUS_M:
            _add(out, "ERROR", "WAYPOINT_BLOCKED", world, label,
                 f"({x:.2f}, {y:.2f}) deja {dist[r, c]:.2f} m libres < radio {ROBOT_RADIUS_M:.2f} m")
        elif dist[r, c] < ROBOT_RADIUS_M + 0.05:
            _add(out, "WARN", "WAYPOINT_BLOCKED", world, label,
                 f"({x:.2f}, {y:.2f}) deja {dist[r, c]:.2f} m libres: al limite del radio "
                 f"{ROBOT_RADIUS_M:.2f} m")
        if not reach[r, c]:
            _add(out, "ERROR", "WAYPOINT_UNREACHABLE", world, label,
                 f"({x:.2f}, {y:.2f}) no es alcanzable desde home con radio {ROBOT_RADIUS_M:.2f} m")
        for fid, f in furniture.items():
            poly = f.get("polygon")
            if poly and point_in_polygon(x, y, poly):
                inside_model = world.models.get(fid)
                if inside_model and inside_model.aabb and inside_model.aabb.lo[2] < 0.05:
                    _add(out, "ERROR", "WAYPOINT_BLOCKED", world, label,
                         f"({x:.2f}, {y:.2f}) cae dentro del mueble '{fid}'")

    # objetos sobre muebles
    for oid, obj in (semantic.get("objects") or {}).items():
        on = obj.get("on")
        if not on:
            continue
        f = furniture.get(on)
        if f is None:
            _add(out, "ERROR", "OBJECT_NOT_ON", world, oid, f"esta sobre '{on}', que no existe en el mapa")
            continue
        poly = f.get("polygon")
        ox, oy, oz = float(obj.get("x", 0)), float(obj.get("y", 0)), float(obj.get("z", 0))
        if poly:
            if not point_in_polygon(ox, oy, poly):
                _add(out, "ERROR", "OBJECT_NOT_ON", world, oid,
                     f"({ox:.2f}, {oy:.2f}) no cae dentro de '{on}'")
            top = float(f.get("height_m", 0))
            interior = str(f.get("type", "")).lower() in ("shelf", "cabinet", "fridge", "estante")
            if interior:
                if oz < 0.05 or oz > top + 0.05:
                    _add(out, "WARN", "OBJECT_FLOATING", world, oid,
                         f"centro z={oz:.2f} fuera del cuerpo de '{on}' (0-{top:.2f} m)")
            elif oz < top - 0.02:
                _add(out, "ERROR", "OBJECT_SUNK", world, oid,
                     f"centro z={oz:.2f} por debajo de la superficie de '{on}' ({top:.2f} m)")
            elif oz - top > 0.50:
                _add(out, "WARN", "OBJECT_FLOATING", world, oid,
                     f"centro z={oz:.2f} a mas de 0.50 m sobre '{on}' ({top:.2f} m): revisar")

    # clientes / invitados / barman
    out.extend(_audit_actors(world, semantic, map_yaml))
    return out


def _customers_defaults() -> tuple[str, str]:
    """(arena, restaurant) con los ATHOME_SIM_CUSTOMERS por defecto de restaurant.sh."""
    sh = WS / "src/base_athome_skills/scripts/restaurant.sh"
    text = sh.read_text(encoding="utf-8")
    arena = re.search(r"ATHOME_SIM_CUSTOMERS=\"\$\{ATHOME_SIM_CUSTOMERS:-([^}]+)\}\"", text)
    matches = re.findall(r"ATHOME_SIM_CUSTOMERS=\"\$\{ATHOME_SIM_CUSTOMERS:-([^}]+)\}\"", text)
    a = matches[0] if matches else ""
    r = matches[1] if len(matches) > 1 else ""
    return a, r


def _audit_actors(world: World, semantic: dict, map_yaml: Path | None) -> list[Finding]:
    out: list[Finding] = []
    arena, restaurant = _customers_defaults()
    raw = ""
    if world.path.name == "arena.sdf":
        raw = arena
    elif world.path.name == "restaurant.sdf":
        raw = restaurant
    if raw:
        chairs = [m for n, m in world.models.items() if n.startswith("silla") and m.aabb]
        for entry in raw.split(";"):
            parts = [p for p in entry.split(",") if p != ""]
            if len(parts) < 2:
                continue
            x, y = float(parts[0]), float(parts[1])
            seated = len(parts) >= 4 and parts[3].lower() == "seated"
            label = f"cliente ({x:.2f}, {y:.2f}) {'sentado' if seated else 'de pie'}"
            nearest = min(chairs, key=lambda m: math.hypot(m.aabb.center[0] - x, m.aabb.center[1] - y),
                          default=None)
            if seated:
                if nearest is None:
                    _add(out, "ERROR", "SEATED_NO_CHAIR", world, label, "no hay ninguna silla en el mundo")
                else:
                    d = math.hypot(nearest.aabb.center[0] - x, nearest.aabb.center[1] - y)
                    if d > 0.30:
                        _add(out, "ERROR", "SEATED_NO_CHAIR", world, label,
                             f"esta sentado a {d:.2f} m de la silla mas cercana ({nearest.name})")
            for name, m in world.models.items():
                if not m.is_wall and m.aabb and _kind_and_range(name)[0] not in ("silla",) \
                        and point_in_polygon(x, y, [(m.aabb.lo[0], m.aabb.lo[1]), (m.aabb.hi[0], m.aabb.lo[1]),
                                                     (m.aabb.hi[0], m.aabb.hi[1]), (m.aabb.lo[0], m.aabb.hi[1])]):
                    _add(out, "ERROR", "CUSTOMER_IN_FURNITURE", world, label,
                         f"cae dentro de '{name}'")
                    break
            if world.path.name == "restaurant.sdf" and "barman" in raw:
                pass
    return out


def audit_scenarios() -> list[Finding]:
    """Personas/bolsas de los escenarios de simulacion contra el mundo arena."""
    out: list[Finding] = []
    script = SIM / "scripts" / "sim_escenarios.py"
    if not script.exists():
        return out
    spec = importlib.util.spec_from_file_location("sim_escenarios_audit", script)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["sim_escenarios_audit"] = mod
    spec.loader.exec_module(mod)
    world = load_world(WORLDS / "arena.sdf")
    origin, size, _ = _map_extent(world, yaml.safe_load((SEM_DIR / "arena_semantic_map.yaml").read_text()),
                                  MAP_DIR / "arena.yaml")
    grid = rasterize(world, origin, size)
    dist = clearance_grid(grid)
    h, w = grid.shape

    def cell(x, y):
        c, r = int((x - origin[0]) / MAP_RES_M), int((y - origin[1]) / MAP_RES_M)
        return (r, c) if 0 <= r < h and 0 <= c < w else None

    walkable = world.world_bounds()
    for sc in mod.arena_scenarios():
        for key, radius in (("ATHOME_SIM_PERSON", 0.30), ("ATHOME_SIM_BAG", 0.20),
                            ("ATHOME_SIM_BAG2", 0.20)):
            val = sc.env.get(key)
            if not val:
                continue
            x, y = (float(v) for v in val.split(",")[:2])
            if sc.name == "sin_persona" and key == "ATHOME_SIM_PERSON":
                continue  # centinela explicito fuera de la casa
            cr = cell(x, y)
            if cr is None or dist[cr] < radius:
                _add(out, "ERROR", "SCENARIO_BLOCKED", world, f"{sc.name} {key}",
                     f"({x:.2f}, {y:.2f}) sin {radius:.2f} m libres para persona/bolsa")
            if not (walkable[0] <= x <= walkable[2] and walkable[1] <= y <= walkable[3]):
                _add(out, "ERROR", "SCENARIO_OUTSIDE", world, f"{sc.name} {key}",
                     f"({x:.2f}, {y:.2f}) fuera de los limites de la casa")
        for p in (sc.env.get("ATHOME_SIM_PERSON_WALK") or "").split(";"):
            if not p:
                continue
            x, y = (float(v) for v in p.split(",")[:2])
            cr = cell(x, y)
            if cr is None or dist[cr] < 0.30:
                _add(out, "ERROR", "SCENARIO_BLOCKED", world, f"{sc.name} camino",
                     f"({x:.2f}, {y:.2f}) sin 0.30 m libres en el camino")
    return out


# --------------------------------------------------------------------------- CLI
def paired_semantic(world_path: Path) -> Path | None:
    rel = SEMANTIC_PAIRS.get(world_path.name)
    return SEM_DIR / rel if rel else None


def paired_map(world_path: Path) -> Path | None:
    cand = MAP_DIR / f"{world_path.stem}.yaml"
    return cand if cand.exists() else None


def run(paths: Sequence[Path], strict: bool = False) -> list[Finding]:
    findings: list[Finding] = []
    for p in paths:
        if not p.exists():
            findings.append(Finding("ERROR", "MISSING", p.stem, "", str(p), "no existe"))
            continue
        try:
            world = load_world(p)
        except Exception as exc:  # SDF roto
            findings.append(Finding("ERROR", "SDF_INVALID", p.stem, "", str(p), f"no se pudo parsear: {exc}"))
            continue
        findings.extend(audit_world(world, paired_semantic(p), paired_map(p)))
        if p.name == "arena.sdf":
            findings.extend(audit_scenarios())
    return findings


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("worlds", nargs="*", help="nombres o rutas de .sdf (por defecto todos)")
    ap.add_argument("--json", type=Path, help="escribir hallazgos en JSON")
    ap.add_argument("--tsv", type=Path, help="escribir hallazgos en TSV")
    ap.add_argument("--strict", action="store_true", help="fallar tambien ante WARN")
    ap.add_argument("--quiet", action="store_true", help="no imprimir la tabla")
    args = ap.parse_args(argv)

    if args.worlds:
        paths = [Path(w) if str(w).endswith(".sdf") else WORLDS / f"{w}.sdf" for w in args.worlds]
    else:
        paths = sorted(WORLDS.glob("*.sdf"))

    findings = run(paths, args.strict)
    levels = {"ERROR": 0, "WARN": 0, "INFO": 0}
    for f in findings:
        levels[f.level] = levels.get(f.level, 0) + 1

    if not args.quiet:
        for f in findings:
            print(f"[{f.level:5s}] {f.world:16s} {f.code:20s} {f.subject:34s} {f.message}")
        print(f"\n{len(findings)} hallazgos: {levels.get('ERROR',0)} ERROR, "
              f"{levels.get('WARN',0)} WARN, {levels.get('INFO',0)} INFO")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps([f.as_dict() for f in findings], ensure_ascii=False, indent=1),
                             encoding="utf-8")
    if args.tsv:
        args.tsv.parent.mkdir(parents=True, exist_ok=True)
        with open(args.tsv, "w", encoding="utf-8") as fh:
            fh.write("level\tcode\tworld\tsubject\twhere\tmessage\n")
            for f in findings:
                fh.write("\t".join([f.level, f.code, f.world, f.subject, f.where,
                                    f.message.replace("\t", " ")]) + "\n")

    errors = levels.get("ERROR", 0)
    warns = levels.get("WARN", 0)
    return 1 if errors or (args.strict and warns) else 0


if __name__ == "__main__":
    sys.exit(main())
