#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Tests unitarios offline para la gestión de objetos semánticos (HOME-22).
"""

import math
import os
import sys
import tempfile
from pathlib import Path
import yaml
import pytest

# Asegurar que el paquete sea importable sin requerir PYTHONPATH previo
_pkg_root = Path(__file__).resolve().parent.parent
if str(_pkg_root) not in sys.path:
    sys.path.insert(0, str(_pkg_root))

from base_athome_navigation.semantic_objects import (
    point_in_polygon,
    find_containing_furniture,
    get_next_free_number,
    is_number_unique,
    calculate_approach_pose,
    create_object_entry,
    validate_object_entry,
)


# Fixture con el polígono real de mesa_de_centro definido en semantic_map.yaml
@pytest.fixture
def sample_furniture():
    return {
        "mesa_de_centro": {
            "name": "Mesa de Centro",
            "type": "table",
            "height_m": 0.76,
            "polygon": [
                [-3.9306, -3.1254],
                [-1.3343, -3.1254],
                [-1.3343, -0.4484],
                [-3.9306, -0.4484],
            ],
            "center": {
                "x": -2.6324,
                "y": -1.7869,
                "z": 0.38,
            },
        }
    }


def test_get_next_free_number():
    # Caso 1: Colección vacía
    assert get_next_free_number({}) == 1

    # Caso 2: Números consecutivos
    objects = {
        "obj_1": {"number": 1, "name": "pringles"},
        "obj_2": {"number": 2, "name": "coke"},
    }
    assert get_next_free_number(objects) == 3

    # Caso 3: Hueco en la secuencia (se liberó el 2)
    objects_with_gap = {
        "obj_1": {"number": 1, "name": "pringles"},
        "obj_3": {"number": 3, "name": "coke"},
    }
    assert get_next_free_number(objects_with_gap) == 2

    # Caso 4: Números almacenados como strings numéricos
    objects_str = {
        "obj_1": {"number": "1", "name": "pringles"},
    }
    assert get_next_free_number(objects_str) == 2


def test_is_number_unique():
    objects = {
        "obj_1": {"number": 1, "name": "pringles"},
        "obj_2": {"number": 2, "name": "coke"},
    }
    # 3 está libre
    assert is_number_unique(objects, 3) is True
    # 1 ya está ocupado
    assert is_number_unique(objects, 1) is False
    # Al editar obj_1, mantener el número 1 es válido para sí mismo
    assert is_number_unique(objects, 1, current_id="obj_1") is True
    # Pero no puede tomar el 2 que pertenece a obj_2
    assert is_number_unique(objects, 2, current_id="obj_1") is False


def test_point_in_polygon():
    square = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0]]

    # Punto interior
    assert point_in_polygon(1.0, 1.0, square) is True
    # Puntos exteriores
    assert point_in_polygon(3.0, 1.0, square) is False
    assert point_in_polygon(-1.0, 1.0, square) is False
    assert point_in_polygon(1.0, 3.0, square) is False
    # Polígonos inválidos o vacíos
    assert point_in_polygon(1.0, 1.0, []) is False
    assert point_in_polygon(1.0, 1.0, [[0.0, 0.0], [1.0, 1.0]]) is False


def test_find_containing_furniture(sample_furniture):
    # Punto en el centro de mesa_de_centro (-2.6324, -1.7869)
    f_id, f_data = find_containing_furniture(-2.6, -1.8, sample_furniture)
    assert f_id == "mesa_de_centro"
    assert f_data is not None
    assert f_data["height_m"] == 0.76

    # Punto fuera de la mesa (por ejemplo, en el origen 0, 0)
    f_id_none, f_data_none = find_containing_furniture(0.0, 0.0, sample_furniture)
    assert f_id_none is None
    assert f_data_none is None


def test_calculate_approach_pose():
    # Objeto a la derecha del centro de referencia (cx=0, cy=0, obj=(1, 0))
    pose = calculate_approach_pose(obj_x=1.0, obj_y=0.0, ref_center=(0.0, 0.0), distance=0.60)
    # Approach debe estar a 0.60 m alejándose del centro: (1.60, 0.0)
    assert math.isclose(pose["x"], 1.60, abs_tol=1e-3)
    assert math.isclose(pose["y"], 0.0, abs_tol=1e-3)

    # Distancia euclídea exacta de 0.60 m
    dist = math.hypot(pose["x"] - 1.0, pose["y"] - 0.0)
    assert math.isclose(dist, 0.60, abs_tol=1e-3)

    # Mirando desde (1.60, 0.0) hacia (1.0, 0.0): ángulo de 180° o -180°
    assert abs(abs(pose["yaw_deg"]) - 180.0) < 0.5

    # Objeto arriba del centro (cx=0, cy=0, obj=(0, 1))
    pose_top = calculate_approach_pose(obj_x=0.0, obj_y=1.0, ref_center=(0.0, 0.0), distance=0.60)
    assert math.isclose(pose_top["x"], 0.0, abs_tol=1e-3)
    assert math.isclose(pose_top["y"], 1.60, abs_tol=1e-3)
    # Mirando desde (0, 1.60) hacia (0, 1.0) hacia abajo: ángulo de -90°
    assert math.isclose(pose_top["yaw_deg"], -90.0, abs_tol=0.5)


def test_create_object_entry_with_table(sample_furniture):
    # Objeto sobre mesa_de_centro
    entry = create_object_entry(
        number=1,
        name="pringles",
        obj_class="can",
        x=-2.6,
        y=-1.8,
        furniture=sample_furniture,
    )

    assert entry["number"] == 1
    assert entry["name"] == "pringles"
    assert entry["class"] == "can"
    assert entry["on"] == "mesa_de_centro"
    # Z debe ser altura de mesa (0.76) + 0.10 = 0.86
    assert math.isclose(entry["z"], 0.86, abs_tol=1e-3)

    # Approach debe existir a 0.60m del objeto
    app = entry["approach"]
    dist = math.hypot(app["x"] - (-2.6), app["y"] - (-1.8))
    assert math.isclose(dist, 0.60, abs_tol=1e-3)

    # Validación
    valid, msg = validate_object_entry(entry)
    assert valid is True, msg


def test_create_object_entry_outside_table():
    # Objeto en suelo o zona libre
    entry = create_object_entry(
        number=2,
        name="coke",
        obj_class="can",
        x=1.5,
        y=2.0,
        furniture={},
    )

    assert entry["number"] == 2
    assert entry["on"] == ""
    # Z fallback
    assert math.isclose(entry["z"], 0.865, abs_tol=1e-3)

    valid, msg = validate_object_entry(entry)
    assert valid is True, msg


def test_roundtrip_yaml_preserving_keys():
    """
    Verifica que cargar un semantic_map.yaml completo, agregar objetos
    y volver a guardarlo preserve intactas las claves existentes.
    """
    initial_yaml_content = """\
map_info:
  yaml_file: /home/athome/mapa_base_athome.yaml
  image_file: /home/athome/mapa_base_athome.pgm
  resolution: 0.05
  origin: [-5.12, -8.732, 0.0]
  width: 204
  height: 264
zones:
  entrance:
    name: Entrada
    type: entrance
    color: '#00d2ff'
    polygon:
    - [2.8, -0.8]
    - [2.8, 0.18]
furniture:
  mesa_de_centro:
    name: Mesa de Centro
    type: table
    height_m: 0.76
keepout_zones: {}
waypoints:
  door:
    name: door
    zone: entrance
    x: 2.45
    y: -0.23
    yaw_deg: 0
seats:
  sofa:
    name: Sofá
    type: couch
    height_m: 0.85
"""

    with tempfile.NamedTemporaryFile("w+", suffix=".yaml", delete=False) as tf:
        tf.write(initial_yaml_content)
        temp_path = tf.name

    try:
        # Cargar datos iniciales
        with open(temp_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        assert "objects" not in data or data.get("objects") is None

        # Agregar objeto
        new_obj = create_object_entry(
            number=1,
            name="pringles",
            obj_class="can",
            x=-2.6,
            y=-1.8,
            z=0.86,
            on="mesa_de_centro",
            approach={"x": -2.6, "y": -1.2, "yaw_deg": -90.0},
        )
        data["objects"] = {"obj_1": new_obj}

        # Guardar nuevamente
        with open(temp_path, "w", encoding="utf-8") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

        # Recargar y verificar
        with open(temp_path, "r", encoding="utf-8") as f:
            reloaded = yaml.safe_load(f)

        # Claves originales preservadas
        assert "map_info" in reloaded
        assert reloaded["map_info"]["resolution"] == 0.05
        assert "zones" in reloaded
        assert "entrance" in reloaded["zones"]
        assert "furniture" in reloaded
        assert "mesa_de_centro" in reloaded["furniture"]
        assert "keepout_zones" in reloaded
        assert "waypoints" in reloaded
        assert "seats" in reloaded
        assert "sofa" in reloaded["seats"]

        # Nueva clave objects presente y correcta
        assert "objects" in reloaded
        assert "obj_1" in reloaded["objects"]
        loaded_obj = reloaded["objects"]["obj_1"]
        assert loaded_obj["number"] == 1
        assert loaded_obj["name"] == "pringles"
        assert loaded_obj["class"] == "can"
        assert loaded_obj["on"] == "mesa_de_centro"
        assert loaded_obj["approach"]["yaw_deg"] == -90.0
    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)


def test_semantic_map_manager_objects_integration():
    """
    Verifica que la clase SemanticMapManager de map_zone_editor
    cargue y guarde objetos adecuadamente preservando la estructura.
    """
    from base_athome_navigation.map_zone_editor import SemanticMapManager

    initial_content = {
        "map_info": {
            "resolution": 0.05,
            "origin": [-5.0, -5.0, 0.0],
            "width": 100,
            "height": 100,
            "yaml_file": "/tmp/dummy.yaml",
            "image_file": "/tmp/dummy.pgm",
        },
        "zones": {},
        "furniture": {
            "mesa_test": {
                "name": "Mesa Test",
                "type": "table",
                "height_m": 0.75,
                "polygon": [[-1.0, -1.0], [1.0, -1.0], [1.0, 1.0], [-1.0, 1.0]],
            }
        },
        "keepout_zones": {},
        "waypoints": {},
        "seats": {},
        "objects": {
            "obj_1": {
                "number": 1,
                "name": "pringles",
                "class": "can",
                "x": 0.5,
                "y": 0.5,
                "z": 0.85,
                "on": "mesa_test",
                "approach": {"x": 0.9243, "y": 0.9243, "yaw_deg": -135.0},
            }
        },
    }

    with tempfile.NamedTemporaryFile("w+", suffix=".yaml", delete=False) as tf:
        yaml.dump(initial_content, tf, default_flow_style=False, sort_keys=False)
        sem_yaml_path = tf.name

    dummy_map_yaml = sem_yaml_path + ".map.yaml"
    with open(dummy_map_yaml, "w") as f:
        yaml.dump({"resolution": 0.05, "origin": [-5.0, -5.0, 0.0], "image": "dummy.pgm"}, f)

    try:
        manager = SemanticMapManager(map_yaml_path=dummy_map_yaml, semantic_yaml_path=sem_yaml_path)

        # Verificar carga de objects
        assert len(manager.objects) == 1
        assert "obj_1" in manager.objects
        assert manager.objects["obj_1"]["number"] == 1
        assert manager.objects["obj_1"]["name"] == "pringles"

        # Guardar nuevo objeto mediante save_data
        new_objects = dict(manager.objects)
        new_objects["obj_2"] = {
            "number": 2,
            "name": "coke",
            "class": "can",
            "x": 0.2,
            "y": 0.2,
            "z": 0.85,
            "on": "mesa_test",
            "approach": {"x": 0.6243, "y": 0.6243, "yaw_deg": -135.0},
        }

        save_success = manager.save_data({"objects": new_objects})
        assert save_success is True

        # Recargar desde archivo
        manager2 = SemanticMapManager(map_yaml_path=dummy_map_yaml, semantic_yaml_path=sem_yaml_path)
        assert len(manager2.objects) == 2
        assert "obj_1" in manager2.objects
        assert "obj_2" in manager2.objects
        assert manager2.objects["obj_2"]["number"] == 2
        assert manager2.objects["obj_2"]["name"] == "coke"
        assert "mesa_test" in manager2.furniture
    finally:
        if os.path.exists(sem_yaml_path):
            os.remove(sem_yaml_path)
        if os.path.exists(dummy_map_yaml):
            os.remove(dummy_map_yaml)

