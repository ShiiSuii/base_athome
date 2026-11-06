#!/usr/bin/env python3
"""Antes de probar "take out the trash" (alias `revisar_basura`). SOLO LECTURA: no mueve nada ni publica comandos.

Dice que falta, con el arreglo. Sale con rc 0 solo si todo esta bien. Correr con el robot energizado y `commander` abierto.
"""
import importlib.util
import json
import os
import sys
import time

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from base_athome_perception import pose_store
from base_athome_perception.arm_manipulation import CONFIG
from base_athome_perception.piper_kinematics import PiperKinematicsSolver

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, "..", "models")
SEMANTIC_MAP = os.path.join(HERE, "..", "..", "base_athome_navigation", "config", "semantic_map.yaml")
WAIT_S = 6.0
TOPICS = (
    ("/scan", LaserScan, "lidar", "revisar el Hokuyo y su cable de red"),
    ("/pan_tilt/joint_states", JointState, "cabeza", "calibrar_pantilt: revisar alimentacion del pan-tilt"),
    ("/camera/camera/color/image_raw", Image, "RealSense color", "reconectar la RealSense al USB 3"),
    ("/camera/camera/aligned_depth_to_color/image_raw", Image, "RealSense profundidad", "reconectar la RealSense al USB 3"),
    ("/camera/camera/color/camera_info", CameraInfo, "RealSense calibracion", "reconectar la RealSense al USB 3"),
    ("/arm_camera/image_raw", Image, "camara de muneca", "revisar la DaBai en la Jetson"),
    ("/torso/joint_states", JointState, "altura del elevador", "falta torso_state: abrir commander"),
)
FRAMES = (
    ("map", "base_footprint", "ubicacion en el mapa", "dar la pose inicial en RViz; revisar Nav2"),
    ("base_footprint", "camera_color_optical_frame", "TF de la cabeza", "falta joint_state_publisher o datos de la cabeza"),
    ("base_footprint", "arm_camera_optical_frame", "TF de la muneca", "falta joint_state_publisher o /arm/joint_states"),
)


def _load_describe():
    spec = importlib.util.spec_from_file_location("ensenar_pregrasp", os.path.join(HERE, "ensenar_pregrasp.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.describe


def check_files(out):
    for name in ("yolov8s-worldv2.pt", "yolov8n-pose.pt"):
        out(os.path.exists(os.path.join(MODELS, name)), f"pesos {name}", "correr scripts/download_models.sh")
    cfg = pose_store.load_pose_config(CONFIG)
    pose = (cfg.get("named_poses") or cfg.get("poses") or {}).get("pre_grasp_floor")
    if not pose:
        out(False, "pose de piso", "ensenarla con ensenar_pregrasp")
    else:
        _, text, problems = _load_describe()(pose["joints_deg"], PiperKinematicsSolver())
        out(not problems, f"pose de piso ({text})", "; ".join(problems) + ": volver a ensenarla con ensenar_pregrasp")
    try:
        with open(SEMANTIC_MAP) as f:
            places = yaml.safe_load(f).get("waypoints") or {}
    except (OSError, AttributeError):
        places = {}
    out("trash_drop" in places, "punto trash_drop en el mapa", "editor_mapa: marcar Punto de la Basura y guardar")


class Probe(Node):
    def __init__(self):
        super().__init__("revisar_basura")
        self.seen, self.arm, self.torso, self.amcl = {}, None, None, None
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        for topic, kind, _, _ in TOPICS:
            self.create_subscription(kind, topic, lambda _m, t=topic: self.seen.__setitem__(t, time.time()), qos_profile_sensor_data)
        self.create_subscription(String, "/arm/status", lambda m: setattr(self, "arm", json.loads(m.data)), 10)
        self.create_subscription(String, "/torso/status", lambda m: setattr(self, "torso", json.loads(m.data)), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: setattr(self, "amcl", m), latched)


def check_live(out):
    rclpy.init()
    node = Probe()
    end = time.time() + WAIT_S
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for topic, _, label, fix in TOPICS:
        out(topic in node.seen, label, fix)
    arm = node.arm or {}
    out(bool(arm.get("feedback_ok")), "brazo con feedback", "energizar el Piper; revisar CAN en la Jetson")
    torso = node.torso or {}
    out(bool(torso.get("connected")), "elevador conectado", "revisar el USB del ESP32; cerrar el CLI stepper")
    out(bool(torso.get("calibrated")), "elevador ubicado", "calibrar_elevador")
    out(node.amcl is not None, "AMCL publica la pose", "revisar Nav2")
    for parent, child, label, fix in FRAMES:
        out(node.tf_buf.can_transform(parent, child, rclpy.time.Time()), label, fix)
    finder = node.count_subscribers("/bag_finder/request")
    out(finder > 0, "bag_finder corriendo", "abrir commander; ver su log si se cerro")
    node.destroy_node()
    rclpy.shutdown()


def main():
    bad = []

    def out(ok, label, fix):
        print(f"[{'OK' if ok else 'FALTA'}] {label}" + ("" if ok else f"\n        -> {fix}"))
        if not ok:
            bad.append(label)

    check_files(out)
    check_live(out)
    print(f"\n{'Todo listo para probar.' if not bad else f'Faltan {len(bad)} cosas: no empezar la prueba todavia.'}")
    return 1 if bad else 0


if __name__ == "__main__":
    np.set_printoptions(precision=3)
    sys.exit(main())
