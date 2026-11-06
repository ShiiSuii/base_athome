"""Tests para el mundo de Pick and Place en simulacion, mapa y mapa semantico (HOME-94)."""
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from base_athome_skills.mission.places import load_places, load_tables, load_objects

SIM_DIR = Path(__file__).resolve().parents[2]
WORLD_SDF = SIM_DIR / "base_athome_sim" / "worlds" / "casa_pick_place.sdf"
MAP_YAML = SIM_DIR / "config" / "casa_pick_place.yaml"
MAP_PGM = SIM_DIR / "config" / "casa_pick_place.pgm"
SEMANTIC_MAP_YAML = SIM_DIR / "base_athome_sim" / "config" / "sim_pick_place_semantic_map.yaml"


def test_pick_place_sdf_valid_with_gz():
    assert WORLD_SDF.exists(), f"No existe {WORLD_SDF}"
    res = subprocess.run(["gz", "sdf", "-k", str(WORLD_SDF)], capture_output=True, text=True)
    assert res.returncode == 0, f"gz sdf -k fallo: {res.stderr}\n{res.stdout}"
    assert "Valid" in res.stdout or res.returncode == 0


def test_pick_place_sdf_contains_required_static_boxes():
    root = ET.parse(WORLD_SDF).getroot()
    models = {m.attrib.get("name"): m for m in root.iter("model")}

    required = ["mesa", "estante", "lavavajillas", "tacho"]
    for name in required:
        assert name in models, f"Falta el modelo '{name}' en casa_pick_place.sdf"
        m = models[name]
        assert m.findtext("static") == "true", f"El modelo '{name}' debe ser static=true"

        # Debe contener cajas como geometria de colision
        box = m.find(".//collision/geometry/box/size")
        assert box is not None and box.text, f"El modelo '{name}' debe tener geometria de caja"

        # Verificar que no contenga mallas (.dae o .obj o .stl con licencia dudosa)
        assert m.find(".//mesh") is None, f"El modelo '{name}' no debe usar mallas externas"


def test_pick_place_map_files_exist_and_cover_furniture():
    assert MAP_YAML.exists(), f"No existe {MAP_YAML}"
    assert MAP_PGM.exists(), f"No existe {MAP_PGM}"

    # Correr occupancy de hacer_mapa para verificar que genera celdas ocupadas
    import importlib.util
    hacer_mapa_path = SIM_DIR / "scripts" / "hacer_mapa.py"
    spec = importlib.util.spec_from_file_location("hacer_mapa", hacer_mapa_path)
    hm = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(hm)
    grid = hm.occupancy(str(WORLD_SDF))
    occupied_count = int(grid.sum())
    assert occupied_count > 1500, f"Muy pocas celdas ocupadas ({occupied_count})"


def test_pick_place_semantic_map_loads_properly():
    assert SEMANTIC_MAP_YAML.exists(), f"No existe {SEMANTIC_MAP_YAML}"

    places = load_places(str(SEMANTIC_MAP_YAML))
    tables = load_tables(str(SEMANTIC_MAP_YAML))
    objects = load_objects(str(SEMANTIC_MAP_YAML))

    # Muebles
    for expected_table in ["mesa", "estante", "lavavajillas", "tacho"]:
        assert expected_table in tables, f"Falta mueble '{expected_table}' en semantic map"

    # Waypoints esenciales
    for wp in ["home", "door", "table", "shelf", "dishwasher", "trash_bin"]:
        assert wp in places, f"Falta waypoint '{wp}' en semantic map"

    # Objetos deben tener punto de aproximacion valido (sin caer dentro de muebles)
    assert len(objects) >= 4, f"Se esperaban al menos 4 objetos en el mapa, hay {len(objects)}"
    for obj in objects:
        assert not obj["problem"], f"Objeto {obj['name']} tiene problema: {obj['problem']}"
