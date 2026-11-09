"""Tests para raised_hand: clasificacion geometrica, mapeo BODY_25, filtro temporal y ordenamiento.
Sin ROS, puramente determinista.
"""

import numpy as np
import pytest

from base_athome_perception.raised_hand import (
    COCO_L_SHOULDER,
    COCO_L_WRIST,
    COCO_NOSE,
    COCO_R_SHOULDER,
    COCO_R_WRIST,
    RaisedHandState,
    body25_to_coco,
    classify_raised_hand,
    normalize_keypoints,
    order_clients,
    TemporalRaisedHandFilter,
    MultiPersonRaisedHandTracker,
)


def _make_dummy_coco(y_nose=100.0, y_l_shoulder=200.0, y_r_shoulder=200.0,
                      y_l_wrist=300.0, y_r_wrist=300.0, default_conf=0.9):
    """Genera 17 keypoints COCO donde por defecto los brazos estan abajo (y_wrist > y_shoulder)."""
    kpts = [[150.0, 250.0] for _ in range(17)]
    conf = [default_conf] * 17

    kpts[COCO_NOSE] = [150.0, y_nose]
    kpts[COCO_L_SHOULDER] = [100.0, y_l_shoulder]
    kpts[COCO_R_SHOULDER] = [200.0, y_r_shoulder]
    kpts[COCO_L_WRIST] = [80.0, y_l_wrist]
    kpts[COCO_R_WRIST] = [220.0, y_r_wrist]

    return np.array(kpts), np.array(conf)


def test_classify_no_hand_raised():
    kpts, conf = _make_dummy_coco(y_l_wrist=300.0, y_r_wrist=300.0)
    res = classify_raised_hand(kpts, conf)
    assert not res.is_raised
    assert res.state == RaisedHandState.NONE


def test_classify_left_hand_raised_above_shoulder():
    # Coordenadas de imagen: Y crece hacia abajo. y_wrist < y_shoulder => arriba
    kpts, conf = _make_dummy_coco(y_l_shoulder=200.0, y_l_wrist=150.0, y_r_wrist=300.0)
    res = classify_raised_hand(kpts, conf)
    assert res.is_raised
    assert res.state == RaisedHandState.LEFT
    assert res.left_raised and not res.right_raised


def test_classify_right_hand_raised_above_nose():
    # Muneca derecha arriba de la nariz
    kpts, conf = _make_dummy_coco(y_nose=100.0, y_r_wrist=80.0, y_l_wrist=300.0)
    res = classify_raised_hand(kpts, conf)
    assert res.is_raised
    assert res.state == RaisedHandState.RIGHT
    assert res.right_raised and not res.left_raised


def test_classify_both_hands_raised():
    kpts, conf = _make_dummy_coco(y_l_wrist=120.0, y_r_wrist=110.0)
    res = classify_raised_hand(kpts, conf)
    assert res.is_raised
    assert res.state == RaisedHandState.BOTH
    assert res.left_raised and res.right_raised


def test_confidence_threshold_ignores_low_confidence():
    kpts, conf = _make_dummy_coco(y_l_wrist=120.0, y_r_wrist=120.0)
    conf[COCO_L_WRIST] = 0.20  # debajo del umbral por defecto (0.40)
    conf[COCO_R_WRIST] = 0.20
    res = classify_raised_hand(kpts, conf, conf_thresh=0.40)
    assert not res.is_raised
    assert res.state == RaisedHandState.NONE


def test_body25_to_coco_mapping():
    # BODY_25 tiene 25 keypoints: Nose=0, RShoulder=2, RWrist=4, LShoulder=5, LWrist=7
    b25_kpts = [[0.0, 0.0] for _ in range(25)]
    b25_conf = [0.9] * 25

    b25_kpts[0] = [150.0, 100.0]   # Nose
    b25_kpts[5] = [100.0, 200.0]   # LShoulder
    b25_kpts[7] = [80.0, 120.0]    # LWrist (arriba del hombro)
    b25_kpts[2] = [200.0, 200.0]   # RShoulder
    b25_kpts[4] = [220.0, 300.0]   # RWrist (abajo)

    coco_pts, coco_conf = body25_to_coco(b25_kpts, b25_conf)
    assert len(coco_pts) == 17
    assert len(coco_conf) == 17
    assert coco_pts[COCO_L_WRIST] == (80.0, 120.0)
    assert coco_pts[COCO_L_SHOULDER] == (100.0, 200.0)

    # Debe clasificar directo como mano izquierda levantada
    res = classify_raised_hand(b25_kpts, b25_conf)
    assert res.is_raised
    assert res.state == RaisedHandState.LEFT


def test_format_compatible_with_person_pose_detector_dict():
    kpts, conf = _make_dummy_coco(y_r_wrist=90.0)
    det_dict = {
        "bbox": [50, 50, 250, 400],
        "score": 0.95,
        "keypoints": kpts,
        "keypoints_conf": conf,
    }
    res = classify_raised_hand(det_dict)
    assert res.is_raised
    assert res.state == RaisedHandState.RIGHT


def test_temporal_filter_single_frame_does_not_trigger():
    # N=3 de M=5: un solo cuadro con mano levantada NO debe disparar la confirmacion
    tf = TemporalRaisedHandFilter(window_size=5, min_raised_frames=3)

    kpts_up, conf_up = _make_dummy_coco(y_l_wrist=120.0)
    kpts_down, conf_down = _make_dummy_coco(y_l_wrist=300.0)

    # Cuadro 1: mano levantada transitoria (ej. ruido o rascarse)
    confirmed, state = tf.update(kpts_up, conf_up)
    assert not confirmed, "Un solo cuadro no debe disparar la deteccion"
    assert state == RaisedHandState.NONE

    # Cuadros 2, 3, 4: mano abajo
    for _ in range(3):
        confirmed, state = tf.update(kpts_down, conf_down)
        assert not confirmed
        assert state == RaisedHandState.NONE


def test_temporal_filter_triggers_after_n_frames():
    # N=3 de M=5: al cumplirse 3 cuadros de los ultimos 5, se confirma
    tf = TemporalRaisedHandFilter(window_size=5, min_raised_frames=3)
    kpts_up, conf_up = _make_dummy_coco(y_l_wrist=120.0)

    # Cuadro 1: 1 de 1 -> False
    c1, _ = tf.update(kpts_up, conf_up)
    assert not c1

    # Cuadro 2: 2 de 2 -> False
    c2, _ = tf.update(kpts_up, conf_up)
    assert not c2

    # Cuadro 3: 3 de 3 -> True!
    c3, s3 = tf.update(kpts_up, conf_up)
    assert c3, "Debe disparar al alcanzar min_raised_frames"
    assert s3 == RaisedHandState.LEFT


def test_temporal_filter_handles_noisy_keypoint_stream():
    # Secuencia ruidosa: [Up, Down, Up, Down, Up] -> en el 5to cuadro suma 3 de 5
    tf = TemporalRaisedHandFilter(window_size=5, min_raised_frames=3)
    kpts_up, conf_up = _make_dummy_coco(y_r_wrist=90.0)
    kpts_down, conf_down = _make_dummy_coco(y_r_wrist=300.0)

    stream = [True, False, True, False, True]
    results = []
    for is_up in stream:
        k, c = (kpts_up, conf_up) if is_up else (kpts_down, conf_down)
        results.append(tf.update(k, c)[0])

    assert results == [False, False, False, False, True]


def test_multi_person_tracker():
    tracker = MultiPersonRaisedHandTracker(window_size=5, min_raised_frames=2)
    kpts_up, conf_up = _make_dummy_coco(y_l_wrist=100.0)
    kpts_down, conf_down = _make_dummy_coco(y_l_wrist=300.0)

    # Persona A levanta la mano dos cuadros seguidos -> confirmada
    tracker.update_person("client_a", kpts_up, conf_up)
    conf_a, state_a = tracker.update_person("client_a", kpts_up, conf_up)
    assert conf_a
    assert state_a == RaisedHandState.LEFT

    # Persona B con mano abajo -> no confirmada
    tracker.update_person("client_b", kpts_down, conf_down)
    conf_b, state_b = tracker.update_person("client_b", kpts_down, conf_down)
    assert not conf_b
    assert state_b == RaisedHandState.NONE


def test_order_clients_by_distance():
    clients = [
        {"id": "c1", "pos": (5.0, 0.0), "is_calling": True},
        {"id": "c2", "pos": (2.0, 0.0), "is_calling": True},
        {"id": "c3", "pos": (8.0, 0.0), "is_calling": True},
    ]
    # Desde el robot en (0, 0): orden debe ser c2 (2m), c1 (5m), c3 (8m)
    ordered = order_clients(clients, by="distance", robot_pos=(0.0, 0.0))
    assert [c["id"] for c in ordered] == ["c2", "c1", "c3"]


def test_order_clients_by_detection():
    clients = [
        {"id": "c1", "detection_index": 2, "is_calling": True},
        {"id": "c2", "detection_index": 0, "is_calling": True},
        {"id": "c3", "detection_index": 1, "is_calling": True},
    ]
    ordered = order_clients(clients, by="detection")
    assert [c["id"] for c in ordered] == ["c2", "c3", "c1"]


def test_order_clients_filter_only_calling():
    clients = [
        {"id": "c1", "distance": 2.0, "is_calling": False},
        {"id": "c2", "distance": 4.0, "is_calling": True},
        {"id": "c3", "distance": 1.0, "is_calling": False},
        {"id": "c4", "distance": 3.0, "is_calling": True},
    ]
    ordered = order_clients(clients, by="distance", only_calling=True)
    assert [c["id"] for c in ordered] == ["c4", "c2"]
