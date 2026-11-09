"""Tests para el modo raised_hands de Restaurant (HOME-104).
Sin ROS, puramente determinista y conforme al contrato:
- Pedido:   /bag_finder/request  JSON {"id": str, "mode": "raised_hands", "timeout_s": float, "min_frames": int}
- Respuesta:/bag_finder/result   JSON {"id": str, "ok": bool, "customers": [{"xy": [x, y], "hand": "left|right|both", "frames": int, "range_m": float}], "error": str}
"""

import json
import math
import numpy as np
import pytest

from base_athome_perception.raised_hand import (
    COCO_L_SHOULDER,
    COCO_L_WRIST,
    COCO_NOSE,
    COCO_R_SHOULDER,
    COCO_R_WRIST,
    RaisedHandState,
    RaisedHandsAggregator,
    TrackedCustomer,
    classify_raised_hand,
    process_raised_hands_candidates,
)


def _make_keypoints(hand="none", y_nose=100.0, y_sh=200.0, conf_val=0.9):
    """Genera 17 keypoints COCO con la posicion de manos especificada ('none', 'left', 'right', 'both').
    Coordenadas de imagen: Y crece hacia abajo. y_wrist < y_sh => muneca arriba del hombro.
    """
    kpts = np.full((17, 2), [150.0, 250.0], dtype=float)
    conf = np.full(17, conf_val, dtype=float)

    kpts[COCO_NOSE] = [150.0, y_nose]
    kpts[COCO_L_SHOULDER] = [100.0, y_sh]
    kpts[COCO_R_SHOULDER] = [200.0, y_sh]

    # Por defecto abajo (y = 300)
    kpts[COCO_L_WRIST] = [80.0, 300.0]
    kpts[COCO_R_WRIST] = [220.0, 300.0]

    # Arriba del hombro y de la nariz si estan levantadas (y = 80)
    if hand in ("left", "both"):
        kpts[COCO_L_WRIST] = [80.0, 80.0]
    if hand in ("right", "both"):
        kpts[COCO_R_WRIST] = [220.0, 80.0]

    return kpts, conf


def test_two_raised_one_down_ordered_by_distance():
    """Dos clientes con mano levantada, uno sin mano levantada:
    - Se descarta al que no levanta la mano.
    - Se ordenan por cercania al robot (menor range_m primero).
    - ok=True y lista de clientes conforme al contrato.
    """
    kpts_r, conf_r = _make_keypoints(hand="right")
    kpts_l, conf_l = _make_keypoints(hand="left")
    kpts_down, conf_down = _make_keypoints(hand="none")

    # Robot en el origen (0, 0)
    # Cliente A: en (3.0, 4.0), distancia = 5.0 m, mano derecha
    # Cliente B: en (1.0, 1.0), distancia = sqrt(2) ~ 1.41 m, mano izquierda
    # Cliente C: en (0.5, 0.5), distancia = sqrt(0.5) ~ 0.71 m, SIN mano levantada
    candidates = [
        {"xy": [3.0, 4.0], "keypoints": kpts_r, "conf": conf_r, "frames": 3},
        {"xy": [1.0, 1.0], "keypoints": kpts_l, "conf": conf_l, "frames": 3},
        {"xy": [0.5, 0.5], "keypoints": kpts_down, "conf": conf_down, "frames": 3},
    ]

    res = process_raised_hands_candidates(
        candidates,
        robot_pos=(0.0, 0.0),
        min_frames=3,
        request_id="req_test_01"
    )

    assert res["id"] == "req_test_01"
    assert res["ok"] is True
    assert res["error"] == ""
    assert len(res["customers"]) == 2

    # Cliente B es el mas cercano (1.41 m vs 5.0 m)
    c0 = res["customers"][0]
    assert c0["xy"] == [1.0, 1.0]
    assert c0["hand"] == "left"
    assert c0["frames"] == 3
    assert c0["range_m"] == pytest.approx(1.41, abs=0.02)

    # Cliente A es el segundo
    c1 = res["customers"][1]
    assert c1["xy"] == [3.0, 4.0]
    assert c1["hand"] == "right"
    assert c1["frames"] == 3
    assert c1["range_m"] == pytest.approx(5.00, abs=0.02)


def test_no_customers_with_raised_hand_returns_ok_false():
    """Sin clientes con la mano levantada -> ok=False, customers=[], error no vacio."""
    kpts_down, conf_down = _make_keypoints(hand="none")
    candidates = [
        {"xy": [1.0, 2.0], "keypoints": kpts_down, "conf": conf_down, "frames": 5},
        {"xy": [3.0, 1.0], "keypoints": kpts_down, "conf": conf_down, "frames": 5},
    ]

    res = process_raised_hands_candidates(candidates, robot_pos=(0.0, 0.0), min_frames=1, request_id="req_empty")
    assert res["id"] == "req_empty"
    assert res["ok"] is False
    assert res["customers"] == []
    assert res["error"] == "no_raised_hand"

    # Lista completamente vacia
    res_empty = process_raised_hands_candidates([], robot_pos=(0.0, 0.0), request_id="req_none")
    assert res_empty["ok"] is False
    assert res_empty["customers"] == []
    assert res_empty["error"] != ""


def test_customer_with_both_hands_raised():
    """Cliente con ambas manos levantadas -> hand='both'."""
    kpts_both, conf_both = _make_keypoints(hand="both")
    candidates = [
        {"xy": [2.5, 1.5], "keypoints": kpts_both, "conf": conf_both, "frames": 4}
    ]

    res = process_raised_hands_candidates(candidates, robot_pos=(0.0, 0.0), min_frames=3, request_id="req_both")
    assert res["ok"] is True
    assert len(res["customers"]) == 1
    assert res["customers"][0]["hand"] == "both"
    assert res["customers"][0]["frames"] == 4


def test_temporal_filter_requires_min_frames():
    """Si no se alcanza min_frames, no se confirma el cliente."""
    kpts_up, conf_up = _make_keypoints(hand="right")
    kpts_down, conf_down = _make_keypoints(hand="none")

    # Secuencia con solo 2 cuadros de mano levantada cuando min_frames=3
    frames_seq_unconfirmed = [kpts_up, kpts_up, kpts_down, kpts_down]
    res_unconf = process_raised_hands_candidates(
        [{"xy": [2.0, 2.0], "frames_keypoints": frames_seq_unconfirmed, "conf": conf_up}],
        min_frames=3,
        request_id="req_unconf"
    )
    assert res_unconf["ok"] is False
    assert res_unconf["customers"] == []

    # Secuencia con 3 cuadros de mano levantada
    frames_seq_confirmed = [kpts_up, kpts_up, kpts_up, kpts_down]
    res_conf = process_raised_hands_candidates(
        [{"xy": [2.0, 2.0], "frames_keypoints": frames_seq_confirmed, "conf": conf_up}],
        min_frames=3,
        request_id="req_conf"
    )
    assert res_conf["ok"] is True
    assert len(res_conf["customers"]) == 1
    assert res_conf["customers"][0]["hand"] == "right"
    assert res_conf["customers"][0]["frames"] >= 3


def test_contract_json_serializability():
    """El diccionario devuelto debe ser estrictamente serializable a JSON."""
    kpts_r, conf_r = _make_keypoints(hand="right")
    res = process_raised_hands_candidates(
        [{"xy": [np.float64(2.1234), np.float64(-1.5678)], "keypoints": kpts_r, "conf": conf_r, "frames": 3}],
        robot_pos=(0.5, -0.5),
        request_id="req_json"
    )
    s = json.dumps(res)
    loaded = json.loads(s)
    assert loaded["ok"] is True
    assert isinstance(loaded["customers"][0]["range_m"], float)
    assert isinstance(loaded["customers"][0]["frames"], int)
    assert isinstance(loaded["customers"][0]["xy"], list)
    assert len(loaded["customers"][0]["xy"]) == 2


def test_distance_relative_to_robot_pose():
    """La distancia (range_m) y el orden dependen de la posicion actual del robot."""
    kpts_r, conf_r = _make_keypoints(hand="right")
    kpts_l, conf_l = _make_keypoints(hand="left")

    candidates = [
        {"xy": [1.0, 0.0], "keypoints": kpts_r, "conf": conf_r, "frames": 3},
        {"xy": [5.0, 0.0], "keypoints": kpts_l, "conf": conf_l, "frames": 3},
    ]

    # Con robot en (4.0, 0.0), el cliente en (5.0, 0.0) esta a 1.0 m, mas cerca que el de (1.0, 0.0) a 3.0 m
    res = process_raised_hands_candidates(candidates, robot_pos=(4.0, 0.0), min_frames=1)
    assert res["ok"] is True
    assert res["customers"][0]["xy"] == [5.0, 0.0]
    assert res["customers"][0]["range_m"] == pytest.approx(1.0, abs=0.01)
    assert res["customers"][1]["xy"] == [1.0, 0.0]
    assert res["customers"][1]["range_m"] == pytest.approx(3.0, abs=0.01)


def test_aggregator_clusters_nearby_observations():
    """Observaciones sucesivas del mismo cliente dentro de match_dist_m se agrupan."""
    kpts_r, conf_r = _make_keypoints(hand="right")
    agg = RaisedHandsAggregator(min_frames=3, match_dist_m=0.8)

    # Cuadro 1 en (2.00, 1.00)
    agg.update_observation((2.00, 1.00), kpts_r, conf_r)
    # Cuadro 2 en (2.05, 1.02) - mismo cliente con ruido
    agg.update_observation((2.05, 1.02), kpts_r, conf_r)
    # Cuadro 3 en (1.98, 0.99) - mismo cliente
    agg.update_observation((1.98, 0.99), kpts_r, conf_r)

    # Debe haber solo 1 cliente rastreado
    assert len(agg.customers) == 1
    assert agg.customers[0].is_confirmed is True
    assert agg.customers[0].raised_frames == 3

    resp = agg.build_response("req_agg", robot_pos=(0.0, 0.0))
    assert resp["ok"] is True
    assert len(resp["customers"]) == 1
    # Coordenadas medianas centradas alrededor de (2.0, 1.0)
    assert resp["customers"][0]["xy"] == [2.0, 1.0]
