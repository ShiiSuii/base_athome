"""Imagenes sinteticas y metodos reales extraidos por AST; sin nodos ROS ni modelos."""
import ast
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import cv2
import numpy as np
import pytest
import yaml

from base_athome_perception.pose_debug import (
    draw_pose_debug, JOINT_COLORS, RAISED_COLOR, LOWERED_COLOR,
)


def person(hand='none'):
    points = np.array([(180, 100), (170, 90), (190, 90), (160, 100), (200, 100),
                       (140, 160), (220, 160), (110, 200), (250, 200), (90, 250),
                       (270, 250), (150, 260), (210, 260), (140, 320), (220, 320),
                       (130, 380), (230, 380)], dtype=float)
    if hand in ('left', 'both'):
        points[9] = (90, 120)
    if hand in ('right', 'both'):
        points[10] = (270, 120)
    return ([50, 60, 310, 410], points, np.ones(17))


@pytest.mark.parametrize('hand', ['left', 'right', 'both', 'none'])
def test_skeleton_joint_colors_box_and_hand_label(hand, monkeypatch):
    labels = []
    real_text = cv2.putText
    def text(img, label, origin, font, scale, color, thickness):
        labels.append((label, color))
        return real_text(img, label, origin, font, scale, color, thickness)
    monkeypatch.setattr(cv2, 'putText', text)
    rgb = np.zeros((480, 640, 3), np.uint8)
    candidate = person(hand)
    image = draw_pose_debug(rgb, [candidate])
    expected = LOWERED_COLOR if hand == 'none' else RAISED_COLOR
    assert (f'MANO {hand}', expected) in labels
    np.testing.assert_array_equal(image[60, 50], expected)
    # Interior del antebrazo, lejos de ambas articulaciones: prueba que hay una linea.
    a, b = candidate[1][7], candidate[1][9]
    x, y = ((a + b) / 2).astype(int)
    np.testing.assert_array_equal(image[y, x], JOINT_COLORS[7])
    np.testing.assert_array_equal(image[160, 140], JOINT_COLORS[5])
    np.testing.assert_array_equal(image[160, 220], JOINT_COLORS[6])
    assert not rgb.any(), 'el dibujo no modifica el cuadro de la camara'


@pytest.mark.parametrize('bad_point', [(90, 250), (float('nan'), 120), (-10, 120), (90, float('inf'))])
def test_invalid_wrist_suppresses_point_limb_and_raised_hand(bad_point):
    box, points, conf = person('left')
    points[9] = bad_point
    if np.isfinite(bad_point).all() and bad_point[0] >= 0:
        conf[9] = 0.1
    image = draw_pose_debug(np.zeros((480, 640, 3), np.uint8), [(box, points, conf)])
    np.testing.assert_array_equal(image[60, 50], LOWERED_COLOR)
    assert not image[225, 100].any()  # antebrazo de la pose baja: no unir un extremo invalido
    assert not image[250, 90].any()


def test_empty_scene_returns_copy():
    image = np.zeros((30, 40, 3), np.uint8)
    out = draw_pose_debug(image, [])
    np.testing.assert_array_equal(out, image)
    assert out is not image


def debug_node():
    path = Path(__file__).parents[1] / 'scripts/bag_finder.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BagFinder')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in ('_debug_tick', '_publish_debug', '_serve')]
    scope = dict(draw_pose_debug=draw_pose_debug,
                 Image=lambda: NS(header=NS(frame_id='', stamp=None)))
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), scope)
    node = type('DebugUnderTest', (), {n.name: scope[n.name] for n in methods})()
    node.lock, node.serve_lock = threading.Lock(), threading.Lock()
    node.rgb, node.rgb_stamp = np.zeros((480, 640, 3), np.uint8), object()
    node.head = (0., 0.)
    node.published, node.calls, node.warnings = [], [], []
    node.pub_debug = NS(get_subscription_count=lambda: 1, publish=node.published.append)
    node.get_logger = lambda: NS(warn=node.warnings.append)
    def poses(rgb):
        node.calls.append(rgb)
        return [person('left')]
    node._poses = poses
    return node


def test_idle_subscriber_gets_pose_without_depth_or_map_tf():
    node = debug_node()
    node._debug_tick()
    assert len(node.calls) == len(node.published) == 1
    msg, = node.published
    assert msg.header.stamp is node.rgb_stamp
    assert msg.header.frame_id == 'camera_color_optical_frame'
    assert (msg.encoding, msg.width, msg.height, msg.step) == ('bgr8', 640, 480, 1920)
    assert len(msg.data) == 480 * 640 * 3
    assert node.calls[0] is not node.rgb
    assert not node.serve_lock.locked()


def test_no_subscriber_means_no_inference():
    node = debug_node()
    node.pub_debug.get_subscription_count = lambda: 0
    node._debug_tick()
    assert not node.calls and not node.published


def test_active_search_does_not_duplicate_inference():
    node = debug_node()
    # Ejecuta _serve real: el timer no debe inferir mientras _serve_one tiene el lock.
    node._serve_one = lambda _: node._debug_tick()
    node._serve({})
    assert not node.calls and not node.published
    node._debug_tick()
    assert len(node.published) == 1


def test_missing_rgb_and_inference_failure_release_lock():
    node = debug_node()
    rgb, node.rgb = node.rgb, None
    node._debug_tick()
    assert not node.calls and not node.serve_lock.locked()
    node.rgb = rgb
    def broken(_):
        raise RuntimeError('fake inference failed')
    node._poses = broken
    node._debug_tick()
    assert node.warnings == ['No se pudo actualizar debug de pose: fake inference failed']
    assert not node.serve_lock.locked()


def test_rviz_openpose_display_uses_debug_topic():
    path = Path(__file__).parents[2] / 'base_athome_skills/config/restaurant.rviz'
    config = yaml.safe_load(path.read_text())
    displays = config['Visualization Manager']['Displays']
    panel, = [d for d in displays if d['Name'] == 'Detecciones OpenPose (pose y manos)']
    assert panel['Topic']['Value'] == '/bag_finder/debug_image'
    assert panel['Enabled'] and panel['Class'] == 'rviz_default_plugins/Image'
