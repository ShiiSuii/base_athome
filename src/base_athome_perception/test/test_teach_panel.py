"""Tests unitarios offline para TeachPanel."""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from base_athome_perception.teach_panel import TeachPanel  # noqa: E402


def make_sample_state(
    feedback_ok=True, torque_on=True, teaching=None, close_to_home=True
):
    """Genera un diccionario de estado de prueba para el panel."""
    home_joints = [86.97, 172.87, -162.63, -99.89, -0.68, 44.98]
    pre_joints = [58.1, 169.44, -68.31, 68.15, -74.94, -79.41]
    post_joints = [42.9, 180.83, -89.01, 60.57, -74.02, -96.89]

    if close_to_home:
        # A 0.5 grados de home
        curr_joints = [87.2, 173.0, -162.5, -99.7, -0.5, 45.1]
    else:
        curr_joints = [0.0, 50.0, -50.0, 0.0, 0.0, 0.0]

    return {
        "feedback_ok": feedback_ok,
        "torque_on": torque_on,
        "teaching": teaching,
        "current_joints_deg": curr_joints,
        "current_gripper_mm": 25.0,
        "saved_poses": {
            "home": {
                "joints_deg": home_joints,
                "gripper_mm": 24.6,
                "timestamp": "2026-10-30 14:07:00",
            },
            "pre_grasp": {
                "joints_deg": pre_joints,
                "gripper_mm": 105.0,
                "timestamp": "2026-10-27 15:27:12",
            },
            "post_grasp": {
                "joints_deg": post_joints,
                "gripper_mm": 50.0,
                "timestamp": "2026-10-27 14:00:00",
            },
        },
    }


def test_init():
    """Verifica inicializacion con valores default y personalizados."""
    panel_default = TeachPanel()
    assert panel_default.width_px == 340
    assert panel_default.poses == ("home", "pre_grasp", "post_grasp")

    panel_custom = TeachPanel(width_px=400, poses=("pose_a", "pose_b"))
    assert panel_custom.width_px == 400
    assert panel_custom.poses == ("pose_a", "pose_b")


def test_render_no_exceptions():
    """Renderiza en un canvas negro bajo diversos estados sin lanzar excepciones."""
    canvas = np.zeros((720, 1280, 3), dtype=np.uint8)
    panel = TeachPanel()

    # 1. Con estado None
    panel.render(canvas, 10, 10, None)
    assert len(panel.buttons) > 0

    # 2. Con estado normal completo
    state = make_sample_state(feedback_ok=True)
    panel.render(canvas, 10, 10, state)
    assert len(panel.buttons) > 0

    # 3. Con estado feedback_ok=False ("SIN DATOS DEL BRAZO")
    state_no_fb = make_sample_state(feedback_ok=False)
    panel.render(canvas, 10, 10, state_no_fb)

    # 4. Con ensenanza activa
    state_teach = make_sample_state(feedback_ok=True, teaching="home")
    panel.render(canvas, 10, 10, state_teach)


def test_button_clicks_actions():
    """Verifica que el clic en el centro de cada boton devuelva la accion esperada."""
    canvas = np.zeros((720, 1280, 3), dtype=np.uint8)
    panel = TeachPanel()
    state = make_sample_state(feedback_ok=True, teaching="home")
    panel.render(canvas, 20, 20, state)

    # Mapa de acciones esperadas por nombre de boton
    expected_actions = {
        "go_home": ("go", "home"),
        "teach_home": ("teach", "home"),
        "go_pre_grasp": ("go", "pre_grasp"),
        "teach_pre_grasp": ("teach", "pre_grasp"),
        "go_post_grasp": ("go", "post_grasp"),
        "teach_post_grasp": ("teach", "post_grasp"),
        "torque_off": ("torque_off", None),
        "torque_on": ("torque_on", None),
        "save": ("save", None),
        "cancel": ("cancel", None),
    }

    found_names = set()
    for btn in panel.buttons:
        if btn.name in expected_actions:
            found_names.add(btn.name)
            bx, by, bw, bh = btn.rect
            cx = bx + bw // 2
            cy = by + bh // 2
            action = panel.handle_click(cx, cy)
            assert action == expected_actions[btn.name], (
                f"Boton {btn.name}: esperado {expected_actions[btn.name]}, "
                f"obtenido {action}"
            )

    # Asegurar que se encontraron todos los botones esperados
    assert found_names == set(expected_actions.keys())

    # Clic fuera de cualquier boton devuelve None
    assert panel.handle_click(0, 0) is None
    assert panel.handle_click(1000, 1000) is None


def test_disabled_buttons_when_no_feedback():
    """Con feedback_ok=False, IR, ENSEÑAR y GUARDAR quedan deshabilitados y devuelven None al clickear."""
    canvas = np.zeros((720, 1280, 3), dtype=np.uint8)
    panel = TeachPanel()
    state_no_fb = make_sample_state(feedback_ok=False, teaching="home")
    panel.render(canvas, 20, 20, state_no_fb)

    for btn in panel.buttons:
        bx, by, bw, bh = btn.rect
        cx = bx + bw // 2
        cy = by + bh // 2
        action = panel.handle_click(cx, cy)

        if btn.name in ("save", "go_home", "teach_home", "go_pre_grasp", "teach_pre_grasp"):
            assert btn.enabled is False
            assert action is None, f"Boton deshabilitado {btn.name} no debe ejecutar accion"
        elif btn.name in ("cancel", "torque_off", "torque_on"):
            assert btn.enabled is True
            assert action is not None


def test_keyboard_shortcuts():
    """Verifica todos los atajos de teclado especificados."""
    panel = TeachPanel()

    # 1. Atajos numericos (IR a pose)
    assert panel.handle_key(ord("1")) == ("go", "home")
    assert panel.handle_key("1") == ("go", "home")
    assert panel.handle_key(ord("2")) == ("go", "pre_grasp")
    assert panel.handle_key("2") == ("go", "pre_grasp")
    assert panel.handle_key(ord("3")) == ("go", "post_grasp")
    assert panel.handle_key("3") == ("go", "post_grasp")

    # 2. Atajos Shift + numero (ENSEÑAR pose)
    assert panel.handle_key(ord("!")) == ("teach", "home")
    assert panel.handle_key("!") == ("teach", "home")
    assert panel.handle_key(ord("@")) == ("teach", "pre_grasp")
    assert panel.handle_key("@") == ("teach", "pre_grasp")
    assert panel.handle_key(ord("#")) == ("teach", "post_grasp")
    assert panel.handle_key("#") == ("teach", "post_grasp")

    # 3. Enter (GUARDAR)
    assert panel.handle_key(13) == ("save", None)
    assert panel.handle_key(10) == ("save", None)
    assert panel.handle_key("\r") == ("save", None)
    assert panel.handle_key("\n") == ("save", None)

    # 4. Escape (CANCELAR)
    assert panel.handle_key(27) == ("cancel", None)
    assert panel.handle_key("\x1b") == ("cancel", None)

    # 5. Liberar brazo ('l'/'L')
    assert panel.handle_key(ord("l")) == ("torque_off", None)
    assert panel.handle_key("l") == ("torque_off", None)
    assert panel.handle_key("L") == ("torque_off", None)

    # 6. Bloquear brazo ('k'/'K')
    assert panel.handle_key(ord("k")) == ("torque_on", None)
    assert panel.handle_key("k") == ("torque_on", None)
    assert panel.handle_key("K") == ("torque_on", None)

    # 7. Teclas invalidas devuelven None
    assert panel.handle_key(ord("z")) is None
    assert panel.handle_key("9") is None
    assert panel.handle_key(None) is None
