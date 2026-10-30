"""Deteccion del /dev/videoX de la DaBai sin v4l2-ctl (Jetson 2026-10-30: caia a video6 inexistente)."""
import importlib.util
import os
import sys
import types
from unittest import mock

HERE = os.path.dirname(__file__)


def load():
    for name in ('cv2', 'rclpy', 'rclpy.node', 'sensor_msgs', 'sensor_msgs.msg'):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules['rclpy.node'].Node = object
    sys.modules['sensor_msgs.msg'].Image = object
    spec = importlib.util.spec_from_file_location('stream_orbbec', os.path.join(HERE, '..', 'stream_orbbec.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_uses_by_id_symlink_without_v4l2ctl():
    mod = load()
    link = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._Dabai_DC1_X-video-index0'
    with mock.patch.object(mod.glob, 'glob', return_value=[link]), \
         mock.patch.object(mod.os.path, 'realpath', return_value='/dev/video0'), \
         mock.patch.object(mod.subprocess, 'check_output', side_effect=AssertionError('no deberia llamarse')):
        assert mod.find_dabai_video_device(6) == 0


def test_falls_back_to_default_when_nothing_found():
    mod = load()
    with mock.patch.object(mod.glob, 'glob', return_value=[]), \
         mock.patch.object(mod.subprocess, 'check_output', side_effect=FileNotFoundError):
        assert mod.find_dabai_video_device(6) == 6
