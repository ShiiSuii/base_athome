#!/usr/bin/env python3
"""Compara donde ve la bolsa cada camara, para calibrar (alias `calibrar_bolsa`). SOLO LECTURA: no mueve nada.

Coordenadas en metros desde el piso, bajo el centro del robot: x adelante, y a su izquierda, z arriba.
Medi la bolsa con una cinta y comparala con lo que dice cada camara. Imagenes con las cajas: /tmp/calibrar_bolsa_*.jpg
"""
import importlib.util
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener

from base_athome_perception import pose_store
from base_athome_perception import wrist_geometry as wg
from base_athome_perception.wrist_detector import BagDetector

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config", "arm_poses_and_limits.yaml")
BASE = "base_footprint"


def _load_finder_helpers():
    spec = importlib.util.spec_from_file_location("bag_finder", os.path.join(HERE, "bag_finder.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.deproject, mod.depth_at


deproject, depth_at = _load_finder_helpers()


def quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _bgr(msg):
    img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
    return img[:, :, ::-1].copy() if msg.encoding == "rgb8" else img.copy()


class Checker(Node):
    def __init__(self):
        super().__init__("calibrar_bolsa")
        self.head_img, self.depth, self.wrist_img, self.K, self.torso, self.head_angles = None, None, None, None, None, None
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        qos = qos_profile_sensor_data
        self.create_subscription(Image, "/camera/camera/color/image_raw", lambda m: setattr(self, "head_img", _bgr(m)), qos)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self._depth, qos)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self._info, qos)
        self.create_subscription(Image, "/arm_camera/image_raw", lambda m: setattr(self, "wrist_img", _bgr(m)), qos)
        self.create_subscription(JointState, "/torso/joint_states", self._torso, 10)
        self.calib = wg.WristCalib.from_dict(pose_store.load_pose_config(CONFIG).get("wrist_camera_calibration"))
        self.bags = BagDetector(os.path.join(HERE, "..", "models"))
        if not self.bags.available():
            raise SystemExit(f"Faltan los pesos del detector: {self.bags.weights}")
        self.bags.load()

    def _depth(self, m):
        self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()

    def _info(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _torso(self, m):
        if "torso_lift_joint" in m.name:
            self.torso = m.position[list(m.name).index("torso_lift_joint")]

    def _tf(self, frame):
        try:
            t = self.tf_buf.lookup_transform(BASE, frame, rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001
            return None
        return np.array([t.translation.x, t.translation.y, t.translation.z]), quat_to_R(t.rotation)

    def _best(self, img, name):
        dets = self.bags.detect(img)
        view = img.copy()
        for b, c, label in dets:
            cv2.rectangle(view, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), (0, 255, 0), 2)
            cv2.putText(view, f"{label} {c:.2f}", (int(b[0]), max(15, int(b[1]) - 5)), 0, 0.5, (0, 255, 0), 1)
        cv2.imwrite(f"/tmp/calibrar_bolsa_{name}.jpg", view)
        return max(dets, key=lambda d: d[1]) if dets else None

    def head(self):
        """(x, y, z superior) desde la cabeza, o un texto si no se puede."""
        if self.head_img is None or self.depth is None or self.K is None:
            return None, "sin imagen o profundidad de la RealSense"
        cam = self._tf("camera_color_optical_frame")
        if cam is None:
            return None, "sin TF de la camara de la cabeza"
        best = self._best(self.head_img, "cabeza")
        if best is None:
            return None, "no ve una bolsa"
        b = best[0]
        cu, cv_ = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
        z = depth_at(self.depth, cu, cv_)
        if z is None:
            return None, "ve una bolsa, sin profundidad valida"
        center = cam[0] + cam[1] @ deproject(cu, cv_, z, *self.K)
        top = cam[0] + cam[1] @ deproject(cu, b[1], z, *self.K)
        return (center[0], center[1], top[2], best[1]), ""

    def wrist(self, plane_z):
        """(x, y) con la camara de muneca cortando el rayo en plane_z, o un texto."""
        if self.wrist_img is None:
            return None, "sin imagen de la camara de muneca"
        cam = self._tf("arm_camera_optical_frame")
        if cam is None:
            return None, "sin TF de la camara de muneca (falta joint_state_publisher?)"
        best = self._best(self.wrist_img, "muneca")
        if best is None:
            return None, "no ve una bolsa"
        b = best[0]
        p = wg.ray_plane_point(cam[0], cam[1], (b[0] + b[2]) / 2, b[1] + 0.15 * (b[3] - b[1]), self.calib, plane_z)
        return (None, "el rayo no corta esa altura") if p is None else ((p[0], p[1], best[1]), "")


def main():
    rclpy.init()
    node = Checker()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(__doc__.split("\n\n", 1)[1])
    default_z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
    try:
        while True:
            time.sleep(1.0)
            h, why_h = node.head()
            z = h[2] if h else default_z
            w, why_w = node.wrist(z)
            torso = "sin medir" if node.torso is None else f"{node.torso:+.2f} m"
            print(f"elevador {torso}")
            print("  cabeza: " + (f"x {h[0]:.2f}  y {h[1]:+.2f}  alto {h[2]:.2f}  (conf {h[3]:.2f})" if h else why_h))
            print("  muneca: " + (f"x {w[0]:.2f}  y {w[1]:+.2f}  a altura {z:.2f}  (conf {w[2]:.2f})" if w else why_w))
            if h and w:
                print(f"  diferencia entre camaras: {math.hypot(h[0] - w[0], h[1] - w[1]) * 100:.0f} cm")
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
