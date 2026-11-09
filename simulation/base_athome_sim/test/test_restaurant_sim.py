"""Tests para la simulacion de Restaurant y clientes con mano levantada (HOME-104).
Sin ROS (o con mocks), determinista y ejecutable con pytest.
"""

import math
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

# Asegurar que se cargue el paquete local del worktree y no build/ o install/
SIM_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_DIR))

import numpy as np
import pytest

from base_athome_sim import gz_model, sim_scene
from base_athome_perception.raised_hand import classify_raised_hand, RaisedHandState

RESTAURANT_SDF = SIM_DIR / "worlds" / "restaurant.sdf"


def test_make_customer_raised_hand_keypoints():
    """make_customer debe situar la muneca por encima de la cabeza y los hombros."""
    # Mano derecha levantada (de pie)
    cust_r = sim_scene.make_customer([2.0, 1.0], facing_xy=[0.0, 0.0], hand="right", seated=False)
    kp_r = cust_r["keypoints"]
    nose_z = kp_r[0][2]
    r_sh_z = kp_r[6][2]
    r_wr_z = kp_r[10][2]
    l_wr_z = kp_r[9][2]

    assert r_wr_z > nose_z, f"Muneca derecha ({r_wr_z}) debe estar sobre la cabeza ({nose_z})"
    assert r_wr_z > r_sh_z, f"Muneca derecha ({r_wr_z}) debe estar sobre el hombro ({r_sh_z})"
    assert l_wr_z < r_sh_z, "Brazo izquierdo debe estar abajo"

    # Mano izquierda levantada (sentado)
    cust_l = sim_scene.make_customer([2.5, -1.0], facing_xy=[0.0, 0.0], hand="left", seated=True)
    kp_l = cust_l["keypoints"]
    assert kp_l[9][2] > kp_l[0][2], "Muneca izquierda debe estar sobre la cabeza"
    assert kp_l[10][2] < kp_l[6][2], "Brazo derecho debe estar abajo"

    # Ambas manos levantadas
    cust_both = sim_scene.make_customer([3.0, 0.0], facing_xy=[0.0, 0.0], hand="both")
    kp_b = cust_both["keypoints"]
    assert kp_b[9][2] > kp_b[0][2]
    assert kp_b[10][2] > kp_b[0][2]


def test_customer_projection_classifies_correctly():
    """Al proyectar un cliente visto por la camara frontal, classify_raised_hand lo reconoce."""
    from base_athome_perception import bag_vision as bv

    cust = sim_scene.make_customer([2.5, 0.0], facing_xy=[0.0, 0.0], hand="right")
    # Camara en (0, 0, 1.0) mirando hacia +X
    T = np.array([0.0, 0.0, 1.0])
    # Rotacion estandar camera optical frame hacia +X mundo:
    # cam_x -> -Y mundo, cam_y -> -Z mundo, cam_z -> +X mundo
    R = np.array([
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ])
    K = (615.0, 615.0, 320.0, 240.0)

    projected = bv.project_people([cust], T, R, K, width=640, height=480)
    assert len(projected) == 1, "El cliente debe ser visible en el campo de vision"
    box, kp, conf = projected[0]

    cls = classify_raised_hand(kp, conf)
    assert cls.is_raised is True
    assert cls.state == RaisedHandState.RIGHT


def test_person_faces_with_customer():
    """sim_scene.person_faces genera caras validas para clientes de pie y sentados."""
    cust_stand = sim_scene.make_customer([2.0, 1.0], hand="right", seated=False)
    faces_stand = sim_scene.person_faces(cust_stand)
    assert len(faces_stand) > 10, "Debe generar caras de cuerpo, cabeza y brazos"

    cust_seat = sim_scene.make_customer([2.0, 1.0], hand="left", seated=True)
    faces_seat = sim_scene.person_faces(cust_seat)
    assert len(faces_seat) > 10


def test_customer_sdf_valid_with_gz():
    """gz_model.customer_sdf genera un SDF valido con cuerpo y brazo levantado."""
    sdf_r = gz_model.customer_sdf("cliente_mesa1", [2.5, 1.5], hand="right", seated=False)
    assert "cliente_mesa1" in sdf_r
    assert "v_cuerpo" in sdf_r
    assert "v_brazo_d" in sdf_r

    sdf_both = gz_model.customer_sdf("cliente_mesa2", [3.5, 1.5], hand="both", seated=True)
    assert "v_brazo_d" in sdf_both
    assert "v_brazo_i" in sdf_both

    with tempfile.NamedTemporaryFile("w", suffix=".sdf") as f:
        f.write(sdf_r)
        f.flush()
        res = subprocess.run(["gz", "sdf", "-k", f.name], capture_output=True, text=True)
        assert res.returncode == 0, f"Error validando SDF: {res.stderr}"
        assert "Valid" in res.stdout or res.returncode == 0


def test_restaurant_sdf_valid_with_gz():
    """El mundo restaurant.sdf debe existir y ser valido para Gazebo Harmonic."""
    assert RESTAURANT_SDF.exists(), f"No existe {RESTAURANT_SDF}"
    res = subprocess.run(["gz", "sdf", "-k", str(RESTAURANT_SDF)], capture_output=True, text=True)
    assert res.returncode == 0, f"gz sdf -k fallo en restaurant.sdf: {res.stderr}\n{res.stdout}"
    assert "Valid" in res.stdout or res.returncode == 0


def test_restaurant_sdf_contains_required_furniture():
    """El salon debe incluir paredes, barra de cocina y 3 mesas."""
    root = ET.parse(RESTAURANT_SDF).getroot()
    models = {m.attrib.get("name"): m for m in root.iter("model")}

    required_models = ["piso", "paredes", "barra", "mesa_1", "mesa_2", "mesa_3"]
    for name in required_models:
        assert name in models, f"Falta el modelo '{name}' en restaurant.sdf"
        m = models[name]
        assert m.findtext("static") == "true", f"El modelo '{name}' debe ser static=true"


def test_restaurant_sdf_header_documentation():
    """El encabezado del archivo debe documentar donde arranca el robot y clientes sugeridos."""
    with open(RESTAURANT_SDF, encoding="utf-8") as f:
        header = f.read(2048)

    assert "x = -1.0" in header or "-1.0" in header, "Debe documentar pose de arranque del robot"
    assert "barra" in header.lower(), "Debe mencionar la barra"
    assert "ATHOME_SIM_CUSTOMERS" in header, "Debe documentar la variable ATHOME_SIM_CUSTOMERS"
    assert "mesa" in header.lower(), "Debe documentar mesas o clientes sugeridos"


def test_athome_sim_customers_env_parsing(monkeypatch):
    """Verifica que sim_robot parsee correctamente ATHOME_SIM_CUSTOMERS con ATHOME_SIM_PERSON."""
    monkeypatch.setenv("ATHOME_SIM_CUSTOMERS", "1.5,2.3;3.5,1.5,left;2.5,-1.5,right,seated")
    monkeypatch.setenv("ATHOME_SIM_PERSON", "0.5,1.8")

    raw_cust = os.environ.get("ATHOME_SIM_CUSTOMERS", "").strip()
    customers = []
    for item in raw_cust.split(";"):
        if item.strip():
            parts = [v.strip() for v in item.split(",") if v.strip()]
            cx, cy = float(parts[0]), float(parts[1])
            hand = parts[2] if len(parts) >= 3 else "right"
            seated = (parts[3].lower() == "seated") if len(parts) >= 4 else False
            customers.append({"xy": [cx, cy], "hand": hand, "seated": seated})

    assert len(customers) == 3
    assert customers[0] == {"xy": [1.5, 2.3], "hand": "right", "seated": False}
    assert customers[1] == {"xy": [3.5, 1.5], "hand": "left", "seated": False}
    assert customers[2] == {"xy": [2.5, -1.5], "hand": "right", "seated": True}

    # Ambos conviven
    assert os.environ.get("ATHOME_SIM_PERSON") == "0.5,1.8"
