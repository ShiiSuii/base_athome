"""Offline checks for Receptionist's simulation-only semantic map override."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[3]


def load_method():
    script = ROOT / 'src/base_athome_perception/scripts/hri_receptionist_challenge.py'
    cls = next(n for n in ast.parse(script.read_text()).body if isinstance(n, ast.ClassDef)
               and n.name == 'HriReceptionistChallengeNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_load_waypoints')
    ns = {'os': os, 'yaml': yaml, 'Dict': dict}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(script), 'exec'), ns)
    return ns['_load_waypoints']


def test_override_is_only_used_in_simulation(tmp_path, monkeypatch):
    package = tmp_path / 'perception'
    package.mkdir()
    config = tmp_path / 'base_athome_navigation/config'
    config.mkdir(parents=True)
    (config / 'semantic_map.yaml').write_text('waypoints: {door: {x: 99}}')
    overlay = ROOT / 'simulation/base_athome_sim/config/receptionist_semantic_map.yaml'
    monkeypatch.setenv('ATHOME_SIM_SEMANTIC_MAP', str(overlay))
    node = SimpleNamespace(pkg_dir=str(package), get_logger=lambda: SimpleNamespace(info=lambda *a: None))
    method = load_method()
    monkeypatch.delenv('ATHOME_SIM', raising=False)
    assert method(node)['door']['x'] == 99
    monkeypatch.setenv('ATHOME_SIM', '1')
    assert method(node)['door']['x'] == -2.0
    assert set(node.seats) == {'chair_left', 'chair_right'}
    assert (config / 'semantic_map.yaml').read_text() == 'waypoints: {door: {x: 99}}'


def test_chairs_match_gazebo_world():
    sim = ROOT / 'simulation/base_athome_sim'
    data = yaml.safe_load((sim / 'config/receptionist_semantic_map.yaml').read_text())
    world = ET.parse(sim / 'worlds/casa_simple.sdf')
    for key, model in [('chair_left', 'silla_1'), ('chair_right', 'silla_2')]:
        pose = world.find(f".//model[@name='{model}']/pose").text.split()
        assert [data['seats'][key][axis] for axis in ('x', 'y')] == list(map(float, pose[:2]))
        assert data['seats'][key]['capacity'] == 1
