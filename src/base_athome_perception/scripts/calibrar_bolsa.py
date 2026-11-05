#!/usr/bin/env python3
"""Debug visual de la bolsa (alias `calibrar_bolsa`). SOLO LECTURA: no mueve nada.

Ventana con las dos camaras. Verde: bolsa. Rojo: descartada (propio robot o poca confianza). Gris: otras clases.
Amarillo: brazo que senala.
Coordenadas en metros desde el piso, bajo el centro del robot: x adelante, y a su izquierda. Comparalas con una cinta.
Teclas sobre la ventana: Q salir, S guardar imagen en /tmp, + y - cambiar la confianza minima.
"""
import importlib.util
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
import torch
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener

from base_athome_perception import pose_store
from base_athome_perception import wrist_geometry as wg
from base_athome_perception.wrist_detector import BAG_NAMES, WRIST_BAG_MIN_CONF, BagDetector, is_own_robot_box
from base_athome_skills.mission import bag_task

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
        self.views, self.min_conf = {}, 0.15
        from ultralytics import YOLO
        self.pose_model = YOLO(os.path.join(HERE, "..", "models", "yolov8n-pose.pt"))

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
        """Dibuja todo lo que ve el detector y devuelve la bolsa de mayor confianza."""
        r = self.bags.model.predict(img, imgsz=640, conf=self.min_conf, verbose=False, device="cpu")[0]
        view, best = img.copy(), None
        for box in r.boxes:
            b, c, label = box.xyxy[0].tolist(), float(box.conf), r.names[int(box.cls)]
            bag = label in BAG_NAMES
            ignored = bag and name == "muneca" and (is_own_robot_box(b, img.shape[0]) or c < WRIST_BAG_MIN_CONF)
            if ignored:                # lo mismo que descarta la mision: propio robot o confianza baja
                bag, label = False, f"IGNORADA {label}"
            color = (0, 0, 255) if ignored else (0, 255, 0) if bag else (150, 150, 150)
            cv2.rectangle(view, (int(b[0]), int(b[1])), (int(b[2]), int(b[3])), color, 3 if bag else 1)
            cv2.putText(view, f"{label} {c:.2f}", (int(b[0]) + 2, max(16, int(b[1]) - 5)), 0, 0.55, color, 2 if bag else 1)
            if bag and (best is None or c > best[1]):
                best = (b, c, label)
        self.views[name] = view
        return best

    def pointing(self):
        """Dibuja el gesto sobre la vista de la cabeza. Devuelve el punto del piso senalado o un texto."""
        if self.head_img is None or self.depth is None or self.K is None or self.pose_model is None:
            return None, "sin imagen"
        cam = self._tf("camera_color_optical_frame")
        if cam is None:
            return None, "sin TF"
        r = self.pose_model.predict(self.head_img, conf=0.45, verbose=False, device="cpu")[0]
        if r.keypoints is None or r.keypoints.conf is None or len(r.keypoints.xy) == 0:
            return None, "no ve personas"
        view = self.views.get("cabeza")
        for kp, conf in zip(r.keypoints.xy.cpu().numpy(), r.keypoints.conf.cpu().numpy()):
            for sh, wr, hip in ((5, 9, 11), (6, 10, 12)):
                if min(conf[sh], conf[wr], conf[hip]) < 0.4:
                    continue
                pts = []
                for i in (sh, wr, hip):
                    z = depth_at(self.depth, kp[i][0], kp[i][1])
                    pts.append(None if z is None else cam[0] + cam[1] @ deproject(kp[i][0], kp[i][1], z, *self.K))
                if view is not None:
                    cv2.line(view, tuple(int(v) for v in kp[sh]), tuple(int(v) for v in kp[wr]), (0, 220, 255), 3)
                if any(p is None for p in pts):
                    continue
                if bag_task.is_pointing(pts[0], pts[1], pts[2]):
                    target = bag_task.pointed_floor_point(pts[0], pts[1])
                    if target:
                        return target, ""
                    return None, "brazo separado, pero no apunta al piso"
        return None, "ve persona, sin gesto de senalar"

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


def _screen_is_portrait():
    try:
        out = subprocess.run(["xdpyinfo"], capture_output=True, text=True, timeout=3).stdout
        w, h = re.search(r"dimensions:\s+(\d+)x(\d+)", out).groups()
        return int(h) > int(w)
    except Exception:  # noqa: BLE001
        return False


def _panel(view, lines, size=(640, 480)):
    img = cv2.resize(view, size) if view is not None else np.zeros((size[1], size[0], 3), np.uint8)
    cv2.rectangle(img, (0, 0), (size[0], 26 * len(lines) + 8), (30, 30, 30), -1)
    for i, (text, color) in enumerate(lines):
        cv2.putText(img, text, (8, 24 + 26 * i), 0, 0.6, color, 2)
    return img


PORTRAIT = _screen_is_portrait()
PERIOD_S = 0.5                # 2 veces por segundo alcanzan para mirar; es solo un debug
GREEN, RED, WHITE, YELLOW = (80, 255, 80), (80, 80, 255), (240, 240, 240), (0, 220, 255)


def main():
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))     # cerrar de verdad con TERM
    torch.set_num_threads(2)      # no quitarle CPU a Nav2 ni a los controladores
    rclpy.init()
    node = Checker()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(__doc__.split("\n\n", 1)[1])
    default_z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
    name = "Debug bolsa (solo lectura) - Q sale"
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, 640, 960) if PORTRAIT else cv2.resizeWindow(name, 1280, 480)
    try:
        while True:
            started = time.time()
            h, why_h = node.head()
            target, why_p = node.pointing()
            z = h[2] if h else default_z
            w, why_w = node.wrist(z)
            torso = "sin medir" if node.torso is None else f"{node.torso:+.2f} m"
            l_head = [(f"CABEZA  conf min {node.min_conf:.2f}  elevador {torso}", WHITE)]
            l_head.append((f"BOLSA x {h[0]:.2f} y {h[1]:+.2f} alto {h[2]:.2f} ({h[3]:.2f})", GREEN) if h else (f"bolsa: {why_h}", RED))
            l_head.append((f"SENALA al piso en x {target[0]:.2f} y {target[1]:+.2f}", YELLOW) if target else (f"gesto: {why_p}", WHITE))
            if target and h:
                d = math.hypot(target[0] - h[0], target[1] - h[1])
                l_head.append((f"del gesto a la bolsa: {d * 100:.0f} cm ({'la acepta' if d <= bag_task.MAX_BAG_FROM_RAY_M else 'muy lejos'})", GREEN if d <= bag_task.MAX_BAG_FROM_RAY_M else RED))
            l_wrist = [(f"MUNECA  usando alto {z:.2f}" + ("" if h else " (por defecto)"), WHITE)]
            l_wrist.append((f"BOLSA x {w[0]:.2f} y {w[1]:+.2f} ({w[2]:.2f})", GREEN) if w else (f"bolsa: {why_w}", RED))
            if h and w:
                d = math.hypot(h[0] - w[0], h[1] - w[1]) * 100
                l_wrist.append((f"diferencia entre camaras: {d:.0f} cm", GREEN if d <= 5 else RED))
            panels = [_panel(node.views.get("cabeza", node.head_img), l_head), _panel(node.views.get("muneca", node.wrist_img), l_wrist)]
            canvas = np.vstack(panels) if PORTRAIT else np.hstack(panels)
            cv2.imshow(name, canvas)
            key = cv2.waitKey(max(1, int((PERIOD_S - (time.time() - started)) * 1000))) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key in (ord("s"), ord("S")):
                path = time.strftime("/tmp/debug_bolsa_%H%M%S.jpg")
                cv2.imwrite(path, canvas)
                print("Guardada", path)
            if key in (ord("+"), ord("=")):
                node.min_conf = min(0.9, node.min_conf + 0.05)
            if key == ord("-"):
                node.min_conf = max(0.05, node.min_conf - 0.05)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
