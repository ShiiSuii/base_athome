"""Pruebas del auditor de mundos (sin ROS ni Gazebo, solo SDF y numpy).

Ejecutar:
    python3 -m pytest tools/world_audit/test_audit.py -q
"""
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))

import audit  # noqa: E402

TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <model name="suelo"><static>true</static><pose>0 0 0 0 0 0</pose>
      <link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision></link>
    </model>
    <model name="paredes"><static>true</static>{walls}</model>
    {body}
  </world>
</sdf>
"""


def _wall(x, y, sx, sy, h=1.2, z=0.6):
    return (f'<link name="l"><collision name="c"><pose>{x} {y} {z} 0 0 0</pose>'
            f'<geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision></link>')


def _box(name, x, y, z, sx, sy, sz):
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>'
            f'<link name="l"><collision name="c"><geometry>'
            f'<box><size>{sx} {sy} {sz}</size></box></geometry></collision></link></model>')


def _world(tmp_path, body="", walls="", name="w"):
    p = tmp_path / f"{name}.sdf"
    p.write_text(TEMPLATE.format(name=name, body=body, walls=walls), encoding="utf-8")
    return p


def _codes(path, level=None):
    return [(f.code, f.level) for f in audit.run([path])
            if level is None or f.level == level]


def test_overlapping_furniture_is_error(tmp_path):
    body = _box("caja_a", 0, 0, 0.25, 0.5, 0.5, 0.5) + _box("caja_b", 0.3, 0, 0.25, 0.5, 0.5, 0.5)
    findings = _codes(_world(tmp_path, body))
    assert ("INTERSECT", "ERROR") in findings


def test_chair_tucked_under_table_is_warn(tmp_path):
    body = _box("mesa_1", 0, 0, 0.38, 0.8, 1.2, 0.76) + _box("silla_1_a", 0.0, 0.8, 0.225, 0.45, 0.45, 0.45)
    findings = _codes(_world(tmp_path, body))
    assert ("INTERSECT", "WARN") in findings
    assert ("INTERSECT", "ERROR") not in findings


def test_floating_box_is_error(tmp_path):
    findings = _codes(_world(tmp_path, _box("caja", 0, 0, 0.5, 0.4, 0.4, 0.2)))
    assert ("FLOATING", "ERROR") in findings


def test_box_sunk_into_table_is_error(tmp_path):
    body = _box("mesa_1", 0, 0, 0.38, 0.8, 1.2, 0.76) + _box("caja", 0, 0, 0.5, 0.2, 0.2, 0.2)
    findings = _codes(_world(tmp_path, body))
    assert ("SUNK", "ERROR") in findings


def test_table_height_out_of_range_is_error(tmp_path):
    findings = _codes(_world(tmp_path, _box("mesa", 0, 0, 0.4075, 0.6, 0.9, 0.815)))
    assert ("HEIGHT_RANGE", "ERROR") in findings


def test_narrow_door_is_error(tmp_path):
    walls = _wall(-3, -1.7125, 0.1, 2.575) + _wall(-3, 1.7125, 0.1, 2.575)
    findings = _codes(_world(tmp_path, walls=walls))
    assert ("DOOR_NARROW", "ERROR") in findings


def test_furniture_beside_wall_is_not_penetration(tmp_path):
    walls = _wall(-3, 0, 0.1, 6) + _wall(3, 0, 0.1, 6)
    body = _box("mesa", 0, 0, 0.38, 0.6, 0.9, 0.76)
    findings = _codes(_world(tmp_path, body, walls))
    assert not [c for c in findings if c[0] == "WALL_PENETRATION"]


def test_furniture_crossing_wall_is_penetration(tmp_path):
    walls = _wall(-3, 0, 0.1, 6)
    body = _box("mesa", -2.95, 0, 0.38, 0.6, 0.9, 0.76)
    findings = _codes(_world(tmp_path, body, walls))
    assert ("WALL_PENETRATION", "ERROR") in findings


def test_json_output_and_exit_code(tmp_path, capsys):
    bad = _world(tmp_path, _box("caja", 0, 0, 0.5, 0.4, 0.4, 0.2), name="malo")
    out = tmp_path / "out.json"
    rc = audit.main([str(bad), "--json", str(out), "--quiet"])
    assert rc == 1
    data = json.loads(out.read_text(encoding="utf-8"))
    assert any(d["code"] == "FLOATING" for d in data)


@pytest.mark.parametrize("world", ["arena", "casa_basura", "casa_simple", "casa_pick_place", "nav_eval"])
def test_owned_worlds_have_no_errors(world):
    findings = audit.run([audit.WORLDS / f"{world}.sdf"])
    errors = [f"{f.code} {f.subject}: {f.message}" for f in findings if f.level == "ERROR"]
    assert not errors, "\n".join(errors)


def test_restaurant_has_no_errors_only_warns():
    findings = audit.run([audit.WORLDS / "restaurant.sdf"])
    assert not [f for f in findings if f.level == "ERROR"]


def test_nav_eval_semantic_map_matches_world():
    findings = audit.run([audit.WORLDS / "nav_eval.sdf"])
    assert not [f for f in findings if f.code in ("FURNITURE_MISMATCH", "WAYPOINT_OUTSIDE")]


def test_pick_place_approach_points_are_reachable():
    findings = audit.run([audit.WORLDS / "casa_pick_place.sdf"])
    blocked = [f.subject for f in findings
               if f.code in ("WAYPOINT_BLOCKED", "WAYPOINT_UNREACHABLE")
               and f.subject.startswith("approach")]
    assert not blocked
