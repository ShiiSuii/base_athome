"""Tests offline de la presentacion mutua del HRI Challenge 2026.2 (§5.1) y la mirada.

    python3 -m pytest -q src/base_athome_perception/test/test_hri_introductions.py
"""
import importlib.util
import math
import os
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, '..'))
SCRIPT = os.path.join(HERE, '..', 'scripts', 'hri_receptionist_challenge.py')


@pytest.fixture(scope='module')
def node_cls():
    spec = importlib.util.spec_from_file_location('hri_receptionist_challenge', SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as exc:  # dependencias de vision ausentes en este entorno
        pytest.skip(f'no se pudo importar el nodo: {exc}')
    cls = next(v for v in vars(mod).values() if isinstance(v, type) and hasattr(v, '_introduce_guests_to_each_other'))
    return cls


def make(node_cls, robot_pose=None):
    n = node_cls.__new__(node_cls)
    n.seats = {'sofa': {'x': 1.0, 'y': 1.0}, 'chair': {'x': 1.0, 'y': -1.0}}
    n.robot_pose = robot_pose
    n.head_pan_sign = 1.0
    n.fallback_guest_pan_deg = [25.0, -25.0]
    n.registered_guests = [{'name': 'Alex', 'drink': 'Lemonade', 'seat': 'sofa'},
                           {'name': 'Taylor', 'drink': 'Coffee', 'seat': 'chair'}]
    n.log = []
    n.move_head_to = lambda pan, tilt: n.log.append(('head', round(pan, 1)))
    n.say = lambda text, **kw: n.log.append(('say', text))
    n.sleep_gui = lambda s: None
    return n


def test_pan_to_seat_uses_robot_pose(node_cls):
    n = make(node_cls, robot_pose=(0.0, 0.0, 0.0))
    assert math.isclose(n._head_pan_to_seat('sofa'), 45.0)
    assert math.isclose(n._head_pan_to_seat('chair'), -45.0)
    n.robot_pose = (0.0, 0.0, math.pi / 2)  # mirando hacia +y
    assert math.isclose(n._head_pan_to_seat('sofa'), -45.0)


def test_pan_to_seat_without_pose_is_none(node_cls):
    assert make(node_cls)._head_pan_to_seat('sofa') is None


def test_mutual_introduction_looks_at_listener_and_states_other_guest(node_cls):
    n = make(node_cls, robot_pose=(0.0, 0.0, 0.0))
    n._introduce_guests_to_each_other()
    assert n.log == [
        ('head', -45.0), ('say', "Taylor, this is Alex. Alex's favorite drink is Lemonade."),
        ('head', 45.0), ('say', "Alex, this is Taylor. Taylor's favorite drink is Coffee."),
    ]


def test_mutual_introduction_falls_back_to_fixed_angles(node_cls):
    n = make(node_cls)
    n._introduce_guests_to_each_other()
    assert [e for e in n.log if e[0] == 'head'] == [('head', -25.0), ('head', 25.0)]


def test_single_guest_does_nothing(node_cls):
    n = make(node_cls)
    n.registered_guests = n.registered_guests[:1]
    n._introduce_guests_to_each_other()
    assert n.log == []
