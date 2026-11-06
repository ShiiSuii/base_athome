#!/usr/bin/env python3
"""Debug visual de la bolsa (alias `calibrar_bolsa`). SOLO LECTURA: no mueve nada.

Ventana con las dos camaras. Verde: bolsa. Rojo: descartada, con su motivo. Gris: otras clases. Amarillo: brazo que senala.
Mide con base_athome_perception.bag_vision, lo mismo que la mision: lo que muestra es lo que la mision creeria.
Coordenadas en metros desde el piso, bajo el centro del robot: x adelante, y a su izquierda. Comparalas con una cinta.
Teclas sobre la ventana: Q salir, S guardar imagen en /tmp, + y - cambiar la confianza minima.
"""
import json
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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from base_athome_perception import bag_vision as bv
from base_athome_perception import pose_store
from base_athome_perception import wrist_geometry as wg
from base_athome_perception.wrist_detector import BAG_NAMES, WRIST_BAG_MIN_CONF, is_own_robot_box, make_bag_detector
from base_athome_skills.mission import bag_task

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "..", "config", "arm_poses_and_limits.yaml")
BASE = "base_footprint"
SIM = os.environ.get("ATHOME_SIM") == "1"


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
        self.bags = make_bag_detector(os.path.join(HERE, "..", "models"), wrist=False)
        self.views, self.min_conf = {}, 0.15
        self.pose_model, self.sim_people = None, []
        if SIM:
            self.create_subscription(String, "/sim/people", lambda m: setattr(self, "sim_people", json.loads(m.data)["people"]), 5)
        else:
            if not self.bags.available():
                raise SystemExit(f"Faltan los pesos del detector: {self.bags.weights}")
            self.bags.load()
            from ultralytics import YOLO
            self.pose_model = YOLO(os.path.join(HERE, "..", "models", "yolov8n-pose.pt"))

    def _depth(self, m):
        self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()

    def _info(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _torso(self, m):
        if "torso_lift_joint" in m.name:
            self.torso = m.position[list(m.name).index("torso_lift_joint")]

    def _tf(self, frame, base=BASE):
        try:
            t = self.tf_buf.lookup_transform(base, frame, rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001
            return None
        return np.array([t.translation.x, t.translation.y, t.translation.z]), bv.quat_to_R(t.rotation)

    def _draw(self, view, box, color, text, thick=1):
        cv2.rectangle(view, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), color, thick)
        cv2.putText(view, text, (int(box[0]) + 2, max(16, int(box[1]) - 5)), 0, 0.55, color, thick)

    def _bags(self, img, name, measure):
        """Mide cada bolsa detectada y dibuja todo. Devuelve [(medicion, confianza)] de las que no se descartan."""
        view, out = img.copy(), []
        for b, c, label in self.bags.detect_all(img):
            if c < self.min_conf:
                continue
            if label not in BAG_NAMES:
                self._draw(view, b, (150, 150, 150), f"{label} {c:.2f}")
                continue
            if name == "muneca" and is_own_robot_box(b, img.shape[0]):
                self._draw(view, b, (0, 0, 255), f"descartada: propio robot {c:.2f}", 2)
            elif name == "muneca" and c < WRIST_BAG_MIN_CONF:
                self._draw(view, b, (0, 0, 255), f"descartada: confianza {c:.2f}", 2)
            else:
                m = measure(b)
                if m.ok:
                    self._draw(view, b, (0, 255, 0), f"{label} {c:.2f}", 3)
                    out.append((m, c))
                else:
                    self._draw(view, b, (0, 0, 255), f"descartada: {m.reason}", 2)
        self.views[name] = view
        return out

    def pointing(self):
        """Dibuja el gesto sobre la vista de la cabeza. Devuelve el punto del piso senalado o un texto."""
        if self.head_img is None or self.depth is None or self.K is None:
            return None, "sin imagen"
        cam = self._tf("camera_color_optical_frame")
        if cam is None:
            return None, "sin TF"
        if SIM:
            world = self._tf("camera_color_optical_frame", "map")
            people = bv.project_people(self.sim_people, world[0], world[1], self.K, 640, 480) if world else []
        else:
            people = bv.yolo_people(self.pose_model.predict(self.head_img, conf=0.45, verbose=False, device="cpu")[0])
        view = self.views.get("cabeza")
        for _, kp, conf in people:
            for sh, wr in ((5, 9), (6, 10)):
                if view is not None and min(conf[sh], conf[wr]) >= bv.MIN_KP_CONF:
                    cv2.line(view, tuple(int(v) for v in kp[sh]), tuple(int(v) for v in kp[wr]), (0, 220, 255), 3)
        return bv.pointed_target(people, self.depth, cam[0], cam[1], self.K)

    def head(self):
        """Medicion de la bolsa desde la cabeza con su confianza, o un texto si no se puede."""
        if self.head_img is None or self.depth is None or self.K is None:
            return None, "sin imagen o profundidad de la RealSense"
        cam = self._tf("camera_color_optical_frame")
        if cam is None:
            return None, "sin TF de la camara de la cabeza"
        found = self._bags(self.head_img, "cabeza", lambda b: bv.head_measure(b, self.depth, cam[0], cam[1], self.K))
        return max(found, key=lambda f: f[1]) if found else None, "no ve una bolsa valida"

    def wrist(self, top_z, expected=None):
        """Medicion de la bolsa desde la muneca, la mas cercana a la de la cabeza si la hay; o un texto."""
        if self.wrist_img is None:
            return None, "sin imagen de la camara de muneca"
        cam = self._tf("arm_camera_optical_frame")
        if cam is None:
            return None, "sin TF de la camara de muneca (falta joint_state_publisher?)"
        K = (self.calib.focal_px, self.calib.focal_px, self.calib.cx_px, self.calib.cy_px)
        found = self._bags(self.wrist_img, "muneca", lambda b: bv.wrist_measure(b, cam[0], cam[1], K, top_z, (self.wrist_img.shape[1], self.wrist_img.shape[0])))
        best, why = bv.choose_nearest([f[0] for f in found], expected)
        return (found[best], "") if best is not None else (None, why)


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
    if not SIM:
        import torch
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
            z = h[0].top_z if h else default_z
            w, why_w = node.wrist(z, h[0].xy if h else None)
            torso = "sin medir" if node.torso is None else f"{node.torso:+.2f} m"
            l_head = [(f"CABEZA  conf min {node.min_conf:.2f}  elevador {torso}", WHITE)]
            l_head.append((f"BOLSA x {h[0].xy[0]:.2f} y {h[0].xy[1]:+.2f} alto {h[0].top_z:.2f} ({h[1]:.2f})", GREEN) if h else (f"bolsa: {why_h}", RED))
            l_head.append((f"SENALA al piso en x {target[0]:.2f} y {target[1]:+.2f}", YELLOW) if target else (f"gesto: {why_p}", WHITE))
            if target and h:
                d = math.hypot(target[0] - h[0].xy[0], target[1] - h[0].xy[1])
                l_head.append((f"del gesto a la bolsa: {d * 100:.0f} cm ({'la acepta' if d <= bag_task.MAX_BAG_FROM_RAY_M else 'muy lejos'})", GREEN if d <= bag_task.MAX_BAG_FROM_RAY_M else RED))
            l_wrist = [(f"MUNECA  usando alto {z:.2f}" + ("" if h else " (por defecto)"), WHITE)]
            l_wrist.append((f"BOLSA x {w[0].xy[0]:.2f} y {w[0].xy[1]:+.2f} ({w[1]:.2f})", GREEN) if w else (f"bolsa: {why_w}", RED))
            if h and w:     # solo con las dos mediciones validas: comparar basura con basura confunde
                d = math.hypot(h[0].xy[0] - w[0].xy[0], h[0].xy[1] - w[0].xy[1]) * 100
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
                node.min_conf = max(0.15, node.min_conf - 0.05)
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
