#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Robot @Home - Manipulación por Profundidad 3D en RViz + Pre-Grasp + Cinemática Inversa
#
# Flujo de Operación:
#   1. Cámara Principal + Profundidad (RealSense D435):
#      - Detecta la botella o tubo Pringles, mide su profundidad real 3D (X, Y, Z)
#        y la transforma al marco del hombro del brazo y a base_link.
#   2. Visualización 3D en Tiempo Real en RViz (/arm_camera/object_markers):
#      - Muestra en RViz el cilindro 3D del objeto en su posición espacial real,
#        la flecha desde la pinza (tcp_link) hasta el objeto y la distancia en cm.
#   3. Secuencia de Agarre (Pre-Grasp -> Cinemática Inversa -> Agarre -> Home):
#      - Paso A: Fija el objetivo 3D en RViz y resuelve la Cinemática Inversa (IK)
#                sembrada desde la pose Pre-Grasp enseñada.
#      - Paso B: El brazo va primero a la posición Pre-Grasp enseñada ([3]).
#      - Paso C: Desde Pre-Grasp, el brazo se desplaza con Cinemática Inversa
#                hacia el objeto 3D para rodearlo con la pinza.
#      - Paso D: Cierra la pinza (55 mm) y eleva +8 cm con Cinemática Inversa.
#      - Paso E: Regresa con el objeto a la posición Home ([2]).
#
# Robot: "Robot"
# ==============================================================================

import argparse
import json
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, ColorRGBA, Float32, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

# Asegurar acceso a ultralytics del venv de visión
VENV_SITE_PACKAGES = "/home/athome/ros2_vision_venv/lib/python3.12/site-packages"
if os.path.exists(VENV_SITE_PACKAGES) and VENV_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, VENV_SITE_PACKAGES)

from ultralytics import YOLO

# Módulo cinemático oficial del Brazo Piper
from base_athome_perception.piper_kinematics import (
    transform_camera_to_arm,
    is_in_safe_joint_envelope,
    is_in_safe_cartesian_box,
    clamp_to_safe_cartesian_box,
    PiperKinematicsSolver,
    SAFE_CARTESIAN_BOX_M,
    SAFE_JOINT_LIMITS_DEG,
)

CONFIG_FILE_PATH = "/home/athome/base_athome_ws/src/base_athome_perception/config/arm_poses_and_limits.yaml"

# Posición del hombro izquierdo (arm_base_link) respecto a base_link en el URDF:
SHOULDER_IN_BASE_LINK_M = np.array([0.155, 0.175, 1.015], dtype=float)


class ManipulacionAutonomaNode(Node):
    """Nodo integrado de detección 3D en RViz, calibración de poses y agarre por Cinemática Inversa."""

    # Fases de Configuración
    FASE_1_RANGO = 1
    FASE_2_HOME = 2
    FASE_3_PREGRASP = 3

    # Estados del Ciclo Autónomo (RViz 3D Depth -> Pre-Grasp -> IK Grasp -> Lift -> Home)
    AUTO_IDLE = 10
    AUTO_PASO_A_DETECCION_RVIZ = 11     # Detectar 3D con cámara principal y mostrar en RViz
    AUTO_PASO_B_MOVER_PREGRASP = 12     # Ir primero a la pose Pre-Grasp enseñada
    AUTO_PASO_C_IR_CON_CINEMATICA = 13  # Ir con Cinemática Inversa desde Pre-Grasp al objeto
    AUTO_PASO_D_AGARRE_ELEVACION = 14   # Cerrar pinza (55 mm) y elevar +8 cm con IK
    AUTO_PASO_E_RETORNO_HOME = 15       # Volver a Home con el objeto
    AUTO_SUCCESS = 16

    FASE_NAMES = {
        FASE_1_RANGO: "FASE 1: RANGO DE MOVIMIENTOS SEGURO",
        FASE_2_HOME: "FASE 2: POSICION HOME DEL BRAZO",
        FASE_3_PREGRASP: "FASE 3: POSICION PRE-GRASP DEL BRAZO",
        AUTO_IDLE: "LISTO: OBJETO 3D EN RVIZ (PRESIONE [ESPACIO] O [G] PARA AGARRAR)",
        AUTO_PASO_A_DETECCION_RVIZ: "PASO A: FIJANDO POSICION 3D EN RVIZ Y CALCULANDO IK",
        AUTO_PASO_B_MOVER_PREGRASP: "PASO B: YENDO A POSICION PRE-GRASP",
        AUTO_PASO_C_IR_CON_CINEMATICA: "PASO C: MOVIENDO CON CINEMATICA INVERSA HACIA EL OBJETO",
        AUTO_PASO_D_AGARRE_ELEVACION: "PASO D: CERRANDO PINZA Y ELEVANDO CON CINEMATICA",
        AUTO_PASO_E_RETORNO_HOME: "PASO E: LLEVANDO OBJETO A POSICION HOME",
        AUTO_SUCCESS: "CICLO COMPLETADO: OBJETO EN POSICION HOME",
    }

    def __init__(self, dry_run: bool = False):
        super().__init__("manipulacion_autonoma_node")
        self.dry_run = dry_run
        self.get_logger().info("Iniciando Sistema de Manipulacion 3D en RViz + Cinematica Inversa...")

        # 1. Cargar Configuración de Poses desde YAML
        self.load_configuration()

        # 2. Subscripciones a Cámara Principal + Profundidad (RealSense D435)
        qos_cam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_head_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.head_color_callback, qos_cam
        )
        self.sub_head_depth = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.head_depth_callback, qos_cam
        )
        self.sub_head_info = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.head_info_callback, 10
        )

        # 3. Subscripción a Cámara del Brazo (Muñeca Orbbec DaBai)
        self.sub_wrist_cam = self.create_subscription(
            Image, "/arm_camera/image_raw", self.wrist_cam_callback, qos_cam
        )

        # 4. Telemetría del Brazo Piper y Cabezal Pan-Tilt
        self.sub_arm_status = self.create_subscription(
            String, "/arm/status", self.arm_status_callback, 10
        )
        self.sub_pan_tilt = self.create_subscription(
            JointState, "/pan_tilt/joint_states", self.pan_tilt_callback, 10
        )

        # 5. Publicadores ROS 2 (incluyendo MarkerArray en /arm_camera/object_markers para RViz)
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_joint_cmd = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_gripper_cmd = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_pan_tilt_cmd = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)
        self.pub_enable = self.create_publisher(Bool, "/arm/enable", 5)
        self.pub_rviz_markers = self.create_publisher(MarkerArray, "/arm_camera/object_markers", 10)

        # 6. Servicios de Manipulación
        self.cli_set_enable = self.create_client(SetBool, "/arm/set_enable")

        # Buffers de imagen
        self.head_color_frame: Optional[np.ndarray] = None
        self.head_depth_frame: Optional[np.ndarray] = None
        self.wrist_color_frame: Optional[np.ndarray] = None

        # Intrínsecos RealSense D435
        self.fx = 604.2
        self.fy = 603.7
        self.cx = 316.6
        self.cy = 253.7
        self.has_intrinsics = False

        # Ángulos del cabezal Pan-Tilt
        self.head_pan_rad = 0.0
        self.head_tilt_rad = 0.0
        self.pan_tilt_inverted = True

        # Modo de Enseñanza con la Mano
        self.is_teaching_mode = False
        self.teaching_target: Optional[str] = None  # 'home' o 'pre_grasp'

        # Telemetría del brazo Piper
        self.arm_is_moving = False
        self.gripper_opening_mm = 69.5
        self.arm_xyz_mm = [0.0, 0.0, 0.0]
        self.arm_joints_deg = list(self.pose_home)

        # Solucionador cinemático oficial del brazo Piper
        self.kinematics_solver = PiperKinematicsSolver()

        # Modelo YOLO
        self.get_logger().info("Cargando modelo YOLO para deteccion 3D...")
        try:
            self.yolo_model = YOLO("yolov8s-worldv2.pt")
            self.yolo_model.set_classes(["bottle", "pringles can", "can", "cup", "cylinder"])
        except Exception:
            self.yolo_model = YOLO("yolov8n.pt")

        # Estado y detecciones 3D
        self.fase_actual = self.FASE_2_HOME
        self.auto_state = self.AUTO_IDLE
        self.step_start_time = time.time()

        self.target_2d_head: Optional[Tuple[int, int, int, int]] = None
        self.target_3d_head: Optional[Tuple[float, float, float]] = None   # En marco óptico (X, Y, Z_prof)
        self.target_3d_arm: Optional[Tuple[float, float, float]] = None    # En marco del hombro (X_sh, Y_sh, Z_sh)
        self.target_2d_wrist: Optional[Tuple[int, int, int, int]] = None
        self.wrist_error_u = 0.0
        self.wrist_error_v = 0.0
        self.dist_tcp_to_obj_cm = 0.0
        self.is_in_reach = False
        self.last_head_det_time = 0.0
        self.last_marker_pub_time = 0.0
        self.last_out_of_reach_say_time = 0.0

        # Trayectorias articulares calculadas por IK
        self.joints_grasp: Optional[List[float]] = None
        self.joints_lift: Optional[List[float]] = None

        self.status_message = "Listo. Objeto 3D publicandose en RViz (/arm_camera/object_markers)."

    # --------------------------------------------------------------------------
    # CONFIGURACIÓN Y PERSISTENCIA (YAML)
    # --------------------------------------------------------------------------
    def load_configuration(self):
        """Carga o inicializa poses nombradas desde YAML."""
        self.pose_home = [86.28, 173.91, -164.0, 66.98, 0.69, -146.41]
        self.pose_pre_grasp = [30.96, 182.68, -98.68, 37.3, -75.12, -95.86]
        self.gripper_home_mm = 70.0
        self.gripper_pre_grasp_mm = 70.0

        if os.path.exists(CONFIG_FILE_PATH):
            try:
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                    poses = data.get("named_poses", {})
                    if "home" in poses:
                        self.pose_home = list(poses["home"].get("joints_deg", self.pose_home))
                        self.gripper_home_mm = float(poses["home"].get("gripper_mm", 70.0))
                    if "pre_grasp" in poses:
                        self.pose_pre_grasp = list(poses["pre_grasp"].get("joints_deg", self.pose_pre_grasp))
                        self.gripper_pre_grasp_mm = float(poses["pre_grasp"].get("gripper_mm", 70.0))
                self.get_logger().info("Configuracion de poses cargada desde YAML.")
            except Exception as e:
                self.get_logger().warn(f"Error leyendo YAML ({e}), usando valores por defecto.")

    def save_named_pose(self, pose_name: str, joints_deg: List[float], gripper_mm: float):
        """Guarda la pose enseñada en el archivo YAML."""
        try:
            data = {}
            if os.path.exists(CONFIG_FILE_PATH):
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}

            if "named_poses" not in data:
                data["named_poses"] = {}

            data["named_poses"][pose_name] = {
                "joints_deg": [round(float(v), 2) for v in joints_deg],
                "gripper_mm": round(float(gripper_mm), 1),
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }

            with open(CONFIG_FILE_PATH, "w", encoding="utf-8") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)

            self.get_logger().info(f"Pose '{pose_name}' guardada en {CONFIG_FILE_PATH}")
            self.say(f"Pose {pose_name} updated successfully.")
            self.load_configuration()
        except Exception as e:
            self.get_logger().error(f"Error guardando pose en YAML: {e}")

    # --------------------------------------------------------------------------
    # CALLBACKS DE SENSORES Y TELEMETRÍA
    # --------------------------------------------------------------------------
    def head_color_callback(self, msg: Image):
        try:
            self.head_color_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        except Exception:
            pass

    def head_depth_callback(self, msg: Image):
        try:
            self.head_depth_frame = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
        except Exception:
            pass

    def head_info_callback(self, msg: CameraInfo):
        if not self.has_intrinsics:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.has_intrinsics = True

    def wrist_cam_callback(self, msg: Image):
        try:
            self.wrist_color_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        except Exception:
            pass

    def arm_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.arm_is_moving = bool(data.get("is_moving", False))
            self.gripper_opening_mm = float(data.get("gripper_mm", self.gripper_opening_mm))
            self.arm_joints_deg = list(data.get("joints_deg", self.arm_joints_deg))
            if hasattr(self, "kinematics_solver") and len(self.arm_joints_deg) >= 6:
                xyz_m = self.kinematics_solver.forward_position_m([math.radians(d) for d in self.arm_joints_deg[:6]])
                self.arm_xyz_mm = [xyz_m[0] * 1000.0, xyz_m[1] * 1000.0, xyz_m[2] * 1000.0]
        except Exception:
            pass

    def pan_tilt_callback(self, msg: JointState):
        try:
            for name, pos in zip(msg.name, msg.position):
                if "pan" in name:
                    self.head_pan_rad = float(pos)
                elif "tilt" in name:
                    self.head_tilt_rad = float(pos)
        except Exception:
            pass

    # --------------------------------------------------------------------------
    # DETECCIÓN 3D POR PROFUNDIDAD DE CÁMARA PRINCIPAL Y VISUALIZACIÓN EN RVIZ
    # --------------------------------------------------------------------------
    def detect_object_2d(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """Detecta la botella o tubo Pringles en el frame RGB."""
        results = self.yolo_model(frame, verbose=False, conf=0.20)
        candidates = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.yolo_model.names.get(cls_id, "").lower()
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                bw, bh = max(1, x2 - x1), max(1, y2 - y1)
                aspect = bh / bw

                if any(k in cls_name for k in ["bottle", "pringles", "can", "cup", "cylinder"]):
                    score = conf * (1.25 if aspect > 1.05 else 1.0)
                    candidates.append((x1, y1, x2, y2, score))

        if candidates:
            return max(candidates, key=lambda c: c[4])

        # Respaldo por segmentación de color
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 110, 70]), np.array([12, 255, 255])) | \
               cv2.inRange(hsv, np.array([168, 110, 70]), np.array([180, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 1000:
                x, y, w, h = cv2.boundingRect(cnt)
                if 1.1 <= (h / max(1, w)) <= 4.5:
                    return (x, y, x + w, y + h, 0.60)

        return None

    def calculate_3d_from_head_depth(self, bbox: Tuple[int, int, int, int]) -> Optional[Tuple[float, float, float]]:
        """Desproyecta el bounding box 2D usando el mapa de profundidad de la RealSense D435."""
        if self.head_depth_frame is None:
            return None

        x1, y1, x2, y2 = bbox
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        d_h, d_w = self.head_depth_frame.shape
        x_min, x_max = max(0, cx - 6), min(d_w, cx + 7)
        y_min, y_max = max(0, cy - 6), min(d_h, cy + 7)

        depth_patch = self.head_depth_frame[y_min:y_max, x_min:x_max]
        valid_depths = depth_patch[(depth_patch > 180) & (depth_patch < 1500)]
        if len(valid_depths) == 0:
            return None

        z_m = float(np.median(valid_depths)) / 1000.0
        x_m = (cx - self.cx) * z_m / self.fx
        y_m = (cy - self.cy) * z_m / self.fy
        return (x_m, y_m, z_m)

    def update_perception_and_rviz(self, now: float):
        """
        Ejecuta continuamente la detección 3D con la profundidad de la cámara principal
        y publica el cilindro 3D + vector + distancia en RViz (/arm_camera/object_markers).
        """
        # 1. Cámara Principal + Profundidad -> Posición 3D en Hombro y Base_Link
        if self.head_color_frame is not None:
            det_h = self.detect_object_2d(self.head_color_frame)
            if det_h is not None:
                x1, y1, x2, y2, _ = det_h
                self.target_2d_head = (x1, y1, x2, y2)
                self.last_head_det_time = now
                coords_3d = self.calculate_3d_from_head_depth((x1, y1, x2, y2))
                if coords_3d is not None:
                    self.target_3d_head = coords_3d
                    raw_x_sh, raw_y_sh, raw_z_sh = transform_camera_to_arm(
                        coords_3d[0], coords_3d[1], coords_3d[2],
                        pan_rad=self.head_pan_rad,
                        tilt_rad=self.head_tilt_rad,
                    )
                    # Suavizado exponencial (EMA) para estabilidad en RViz y en IK
                    if self.target_3d_arm is None:
                        self.target_3d_arm = (raw_x_sh, raw_y_sh, raw_z_sh)
                    else:
                        ox, oy, oz = self.target_3d_arm
                        self.target_3d_arm = (
                            0.65 * ox + 0.35 * raw_x_sh,
                            0.65 * oy + 0.35 * raw_y_sh,
                            0.65 * oz + 0.35 * raw_z_sh,
                        )
                    in_box, _ = is_in_safe_cartesian_box(*self.target_3d_arm)
                    self.is_in_reach = in_box
            else:
                if now - self.last_head_det_time > 1.0:
                    self.target_2d_head = None

        # 2. Cámara de Muñeca (Secundaria para centrado fino en Pre-Grasp)
        if self.wrist_color_frame is not None:
            det_w = self.detect_object_2d(self.wrist_color_frame)
            if det_w is not None:
                wx1, wy1, wx2, wy2, _ = det_w
                self.target_2d_wrist = (wx1, wy1, wx2, wy2)
                wcx = (wx1 + wx2) / 2.0
                wcy = (wy1 + wy2) / 2.0
                self.wrist_error_u = (wcx - 320.0) / 320.0
                self.wrist_error_v = (wcy - 240.0) / 240.0
            else:
                self.target_2d_wrist = None

        # 3. Publicar Marcadores 3D a RViz a 15 Hz
        if now - self.last_marker_pub_time >= 0.065:
            self.last_marker_pub_time = now
            self.publish_rviz_3d_markers()

    def publish_rviz_3d_markers(self):
        """
        Publica en /arm_camera/object_markers (MarkerArray en frame base_link):
          - Marker 0 (CYLINDER): Objeto 3D en las coordenadas medidas por la profundidad de la cámara principal.
          - Marker 1 (ARROW): Vector desde la pinza actual (tcp_link) hasta el objeto 3D.
          - Marker 2 (TEXT_VIEW_FACING): Distancia Pinza->Objeto y coordenadas 3D.
        """
        ma = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        if self.target_3d_arm is None:
            for mid in [0, 1, 2]:
                m = Marker()
                m.header.frame_id = "base_link"
                m.header.stamp = now_msg
                m.ns = "arm_target"
                m.id = mid
                m.action = Marker.DELETE
                ma.markers.append(m)
            self.pub_rviz_markers.publish(ma)
            return

        # Posición 3D del objeto en base_link (a partir de la profundidad de la cámara principal)
        obj_sh_m = np.array(self.target_3d_arm, dtype=float)
        obj_base_m = SHOULDER_IN_BASE_LINK_M + obj_sh_m

        # Posición 3D actual del TCP de la pinza en base_link
        q_rad = [math.radians(d) for d in self.arm_joints_deg[:6]]
        tcp_sh_mm, _ = self.kinematics_solver.forward_kinematics(q_rad)
        tcp_base_m = SHOULDER_IN_BASE_LINK_M + (tcp_sh_mm / 1000.0)

        # Distancia 3D real entre los dedos de la pinza y el objeto (en cm)
        self.dist_tcp_to_obj_cm = float(np.linalg.norm(obj_base_m - tcp_base_m) * 100.0)

        if self.is_in_reach:
            col = ColorRGBA(r=0.10, g=0.95, b=0.30, a=0.90)
            status_tag = "ALCANCE IK OK"
        else:
            col = ColorRGBA(r=1.00, g=0.35, b=0.10, a=0.90)
            status_tag = "FUERA DE RANGO"

        # 1. Cilindro 3D del Objeto en RViz
        cyl = Marker()
        cyl.header.frame_id = "base_link"
        cyl.header.stamp = now_msg
        cyl.ns = "arm_target"
        cyl.id = 0
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x = float(obj_base_m[0])
        cyl.pose.position.y = float(obj_base_m[1])
        cyl.pose.position.z = float(obj_base_m[2])
        cyl.pose.orientation.w = 1.0
        cyl.scale.x = 0.075
        cyl.scale.y = 0.075
        cyl.scale.z = 0.21
        cyl.color = col
        cyl.lifetime = Duration(sec=0, nanosec=500000000)
        ma.markers.append(cyl)

        # 2. Flecha Guía en RViz desde la Pinza (tcp_link) hasta el Objeto 3D
        arr = Marker()
        arr.header.frame_id = "base_link"
        arr.header.stamp = now_msg
        arr.ns = "arm_target"
        arr.id = 1
        arr.type = Marker.ARROW
        arr.action = Marker.ADD
        arr.points = [
            Point(x=float(tcp_base_m[0]), y=float(tcp_base_m[1]), z=float(tcp_base_m[2])),
            Point(x=float(obj_base_m[0]), y=float(obj_base_m[1]), z=float(obj_base_m[2])),
        ]
        arr.scale.x = 0.012
        arr.scale.y = 0.024
        arr.scale.z = 0.030
        arr.color = ColorRGBA(r=0.0, g=0.88, b=1.0, a=0.95)
        arr.lifetime = Duration(sec=0, nanosec=500000000)
        ma.markers.append(arr)

        # 3. Texto 3D Flotante en RViz con Distancia Pinza->Objeto y Coordenadas
        txt = Marker()
        txt.header.frame_id = "base_link"
        txt.header.stamp = now_msg
        txt.ns = "arm_target"
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = float(obj_base_m[0])
        txt.pose.position.y = float(obj_base_m[1])
        txt.pose.position.z = float(obj_base_m[2] + 0.16)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.045
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        txt.text = (
            f"OBJETO 3D | Dist Pinza: {self.dist_tcp_to_obj_cm:.1f} cm | "
            f"Hombro:[{obj_sh_m[0]:.2f}, {obj_sh_m[1]:.2f}, {obj_sh_m[2]:.2f}]m [{status_tag}]"
        )
        txt.lifetime = Duration(sec=0, nanosec=500000000)
        ma.markers.append(txt)

        self.pub_rviz_markers.publish(ma)

    # --------------------------------------------------------------------------
    # COMANDOS DE ACCIONAMIENTO (CON FIX DE TRAYECTORIA JOINT_CMD)
    # --------------------------------------------------------------------------
    def say(self, text: str):
        clean_ascii = text.encode("ascii", "ignore").decode("ascii")
        msg = String()
        msg.data = clean_ascii
        self.pub_say.publish(msg)
        self.status_message = clean_ascii
        print(f"[Robot]: \"{clean_ascii}\"")

    def send_arm_joint_goal(self, joints_deg: List[float], duration_s: float = 2.2, gripper_mm: Optional[float] = None) -> bool:
        """
        Publica comando articular e incluye la apertura de pinza dentro del mismo JointState
        (sin publicar un /arm/gripper_cmd separado que pueda interrumpir la trayectoria).
        """
        clamped_deg = list(joints_deg[:6])
        clamped_deg[1] = min(clamped_deg[1], 182.7)  # Respetar tope mecánico físico de Joint 2

        safe, reason = is_in_safe_joint_envelope(clamped_deg)
        if not safe:
            self.get_logger().error(f"[SEGURIDAD] Comando articular bloqueado: {reason}")
            self.status_message = f"BLOQUEO SEGURIDAD: {reason}"
            return False

        if self.dry_run:
            self.get_logger().info(f"[DRY-RUN] joint_cmd: {[round(v, 1) for v in clamped_deg]}, dur={duration_s}s, pinza={gripper_mm}mm")
            return True

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [math.radians(d) for d in clamped_deg]
        if gripper_mm is not None:
            js.name.append("gripper")
            js.position.append(float(gripper_mm))
        js.velocity = [float(duration_s)]
        self.pub_joint_cmd.publish(js)
        return True

    def mover_a_home(self, duration_s: float = 2.5, keep_gripper: bool = False):
        if self.is_teaching_mode:
            self.cancel_teaching_mode()
        self.fase_actual = self.FASE_2_HOME
        g_mm = self.gripper_opening_mm if keep_gripper else 70.0
        self.send_arm_joint_goal(self.pose_home, duration_s=duration_s, gripper_mm=g_mm)
        self.status_message = "Moviendo a Posicion HOME."

    def mover_a_pre_grasp(self, duration_s: float = 2.2):
        if self.is_teaching_mode:
            self.cancel_teaching_mode()
        self.fase_actual = self.FASE_3_PREGRASP
        self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=duration_s, gripper_mm=70.0)
        self.status_message = "Moviendo a Posicion PRE-GRASP (70mm)."

    def set_arm_torque(self, enable: bool):
        msg = Bool()
        msg.data = bool(enable)
        self.pub_enable.publish(msg)

    def toggle_teaching_pose(self, target_pose: str):
        if not self.is_teaching_mode:
            self.is_teaching_mode = True
            self.teaching_target = target_pose
            self.set_arm_torque(False)
            self.status_message = f"MODO ENSENANZA [{target_pose.upper()}]: Mueva el brazo con la mano y presione [ENTER] o [E] para guardar."
            self.say(f"Teach mode for {target_pose} activated. Move the arm with your hand and press Enter to save.")
        else:
            save_target = self.teaching_target or target_pose
            self.set_arm_torque(True)
            self.is_teaching_mode = False
            self.save_named_pose(save_target, self.arm_joints_deg, self.gripper_opening_mm)
            self.teaching_target = None
            self.status_message = f"Pose {save_target.upper()} guardada: {[round(v, 1) for v in self.arm_joints_deg]}."

    def cancel_teaching_mode(self):
        if self.is_teaching_mode:
            self.set_arm_torque(True)
            self.is_teaching_mode = False
            self.teaching_target = None
            self.status_message = "Modo ensenanza cancelado. Motores reactivados."

    def send_pan_tilt_goal(self, pan_deg: float, tilt_deg: float):
        v = Vector3()
        v.x = float(pan_deg)
        v.y = float(tilt_deg)
        v.z = 0.0
        self.pub_pan_tilt_cmd.publish(v)

    # --------------------------------------------------------------------------
    # MÁQUINA DE ESTADOS: OBJETO 3D EN RVIZ -> PRE-GRASP -> CINEMÁTICA INVERSA
    # --------------------------------------------------------------------------
    def start_autonomous_grasp(self):
        """Inicia la secuencia: Detectar 3D en RViz -> Pre-Grasp -> IK Grasp -> Lift -> Home."""
        self.auto_state = self.AUTO_PASO_A_DETECCION_RVIZ
        self.step_start_time = time.time()
        self._ik_sent = False
        self._lift_sent = False
        self.say("Starting 3D depth detection and inverse kinematics grasping sequence.")

    def update_autonomous_state_machine(self, now: float):
        if self.auto_state == self.AUTO_IDLE:
            return

        # PASO A: DETECCIÓN 3D POR PROFUNDIDAD DE CÁMARA PRINCIPAL Y CÁLCULO DE IK
        if self.auto_state == self.AUTO_PASO_A_DETECCION_RVIZ:
            if self.target_3d_arm is None:
                self.status_message = "Esperando deteccion 3D de la botella en la camara principal..."
                if now - self.step_start_time > 8.0:
                    self.say("Could not detect bottle 3D depth. Returning to idle.")
                    self.auto_state = self.AUTO_IDLE
                return

            x_sh, y_sh, z_sh = self.target_3d_arm
            in_box, reason = is_in_safe_cartesian_box(x_sh, y_sh, z_sh)
            if not in_box:
                self.status_message = f"Objeto en RViz fuera de alcance: {reason}"
                if now - self.last_out_of_reach_say_time > 4.5:
                    self.say("Bottle detected in RViz, but it is outside my safe reach.")
                    self.last_out_of_reach_say_time = now
                return

            # Sembrar la Cinemática Inversa en la pose Pre-Grasp enseñada
            q_pre_rad = [math.radians(d) for d in self.pose_pre_grasp]
            target_clamped = clamp_to_safe_cartesian_box(x_sh, y_sh, z_sh)
            j_grasp, ok_grasp = self.kinematics_solver.inverse_kinematics(target_clamped, q_seed_rad=q_pre_rad)

            if not ok_grasp:
                # Si el objetivo está ligeramente al borde, avanzar desde Pre-Grasp hacia la dirección del objeto
                pre_x, pre_y, pre_z = self.kinematics_solver.forward_position_m(q_pre_rad)
                dx = float(np.clip(x_sh - pre_x, 0.03, 0.09))
                dy = float(np.clip(y_sh - pre_y, -0.08, 0.08))
                dz = float(np.clip(z_sh - pre_z, -0.06, 0.06))
                j_grasp, ok_grasp = self.kinematics_solver.move_tool_line(
                    self.pose_pre_grasp, delta_forward_m=dx, delta_lateral_m=dy, delta_up_m=dz
                )

            if ok_grasp:
                j_lift, ok_lift = self.kinematics_solver.move_tool_line(
                    j_grasp, delta_forward_m=0.0, delta_lateral_m=0.0, delta_up_m=0.08
                )
                self.joints_grasp = j_grasp
                self.joints_lift = j_lift if ok_lift else j_grasp

                self.get_logger().info(
                    f"[Paso A -> B] Objeto 3D en RViz: Hombro=({x_sh:.2f}, {y_sh:.2f}, {z_sh:.2f})m | "
                    f"IK Grasp={self.joints_grasp} | Moviendo primero a Pre-Grasp..."
                )
                self.say(f"Object locked in RViz at {x_sh:.2f} meters forward. Moving to pre-grasp.")
                self.auto_state = self.AUTO_PASO_B_MOVER_PREGRASP
                self.step_start_time = now
                self.fase_actual = self.FASE_3_PREGRASP
                self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=2.5, gripper_mm=70.0)
            else:
                self.get_logger().warn("Cinematica Inversa no convergio para el punto actual.")

        # PASO B: LLEGAR A LA POSICIÓN PRE-GRASP
        elif self.auto_state == self.AUTO_PASO_B_MOVER_PREGRASP:
            if now - self.step_start_time > 2.7:
                self.auto_state = self.AUTO_PASO_C_IR_CON_CINEMATICA
                self.step_start_time = now
                self._ik_sent = False
                self.say("Pre-grasp reached. Moving with inverse kinematics to grasp the object.")

        # PASO C: IR HACIA EL OBJETO CON CINEMÁTICA INVERSA
        elif self.auto_state == self.AUTO_PASO_C_IR_CON_CINEMATICA:
            step_elapsed = now - self.step_start_time
            if step_elapsed > 0.6 and not getattr(self, "_ik_sent", False):
                self._ik_sent = True
                # Si la cámara de muñeca ve el objeto en Pre-Grasp, refinar suavemente el centrado lateral
                if self.target_2d_wrist is not None and self.joints_grasp is not None:
                    y_corr = float(np.clip(-self.wrist_error_u * 0.035, -0.035, 0.035))
                    z_corr = float(np.clip(-self.wrist_error_v * 0.025, -0.025, 0.025))
                    if abs(y_corr) > 0.004 or abs(z_corr) > 0.004:
                        j_ref, ok_ref = self.kinematics_solver.move_tool_line(
                            self.joints_grasp, delta_forward_m=0.0, delta_lateral_m=y_corr, delta_up_m=z_corr
                        )
                        if ok_ref:
                            self.joints_grasp = j_ref
                            j_lift_ref, ok_l = self.kinematics_solver.move_tool_line(
                                self.joints_grasp, delta_forward_m=0.0, delta_lateral_m=0.0, delta_up_m=0.08
                            )
                            if ok_l:
                                self.joints_lift = j_lift_ref

                if self.joints_grasp is not None:
                    self.get_logger().info(f"[Paso C] Ejecutando trayectoria IK hacia el objeto: {self.joints_grasp}")
                    self.send_arm_joint_goal(self.joints_grasp, duration_s=2.0, gripper_mm=70.0)

            if step_elapsed > 2.9:
                self.auto_state = self.AUTO_PASO_D_AGARRE_ELEVACION
                self.step_start_time = now
                self._lift_sent = False
                self.get_logger().info("[Paso D] Cerrando pinza a 55 mm...")
                if not self.dry_run:
                    g_msg = Float32()
                    g_msg.data = 55.0
                    self.pub_gripper_cmd.publish(g_msg)

        # PASO D: CERRAR PINZA Y ELEVAR +8 CM CON CINEMÁTICA INVERSA
        elif self.auto_state == self.AUTO_PASO_D_AGARRE_ELEVACION:
            step_elapsed = now - self.step_start_time
            if step_elapsed > 1.5 and not getattr(self, "_lift_sent", False):
                self._lift_sent = True
                if self.joints_lift is not None:
                    self.get_logger().info(f"[Paso D] Elevando objeto +8 cm con IK: {self.joints_lift}")
                    self.send_arm_joint_goal(self.joints_lift, duration_s=1.8, gripper_mm=55.0)

            if step_elapsed > 3.5:
                self.auto_state = self.AUTO_PASO_E_RETORNO_HOME
                self.step_start_time = now
                self.say("Object grasped and lifted. Returning to home position.")
                self.mover_a_home(duration_s=2.8, keep_gripper=True)

        # PASO E: RETORNAR A POSICIÓN HOME
        elif self.auto_state == self.AUTO_PASO_E_RETORNO_HOME:
            if now - self.step_start_time > 3.0:
                self.auto_state = self.AUTO_SUCCESS
                self.step_start_time = now
                self.say("Object successfully grasped and brought to home position! I am Robot.")

        elif self.auto_state == self.AUTO_SUCCESS:
            if now - self.step_start_time > 4.0:
                self.auto_state = self.AUTO_IDLE

    # --------------------------------------------------------------------------
    # RENDERIZADO VISUAL DEL HUD (1280 x 640)
    # --------------------------------------------------------------------------
    def draw_hud(self) -> np.ndarray:
        canvas_w, canvas_h = 1280, 640
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        feed_w, feed_h = 620, 430

        # --- PANEL IZQUIERDO: CÁMARA PRINCIPAL + PROFUNDIDAD 3D (PUBLICANDO A RVIZ) ---
        if self.head_color_frame is not None:
            h_img = cv2.resize(self.head_color_frame, (feed_w, feed_h))
            orig_h, orig_w = self.head_color_frame.shape[:2]
            sx, sy = feed_w / orig_w, feed_h / orig_h

            if self.target_2d_head is not None:
                x1, y1, x2, y2 = self.target_2d_head
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                box_col = (0, 255, 0) if self.is_in_reach else (0, 140, 255)
                cv2.rectangle(h_img, (bx1, by1), (bx2, by2), box_col, 2)

                if self.target_3d_arm is not None:
                    x_s, y_s, z_s = self.target_3d_arm
                    tag1 = f"RVIZ 3D: X={x_s:.2f}m, Y={y_s:.2f}m, Z={z_s:.2f}m"
                    tag2 = f"Dist Pinza->Obj: {self.dist_tcp_to_obj_cm:.1f} cm"
                    cv2.putText(h_img, tag1, (bx1, max(22, by1 - 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.50, box_col, 2)
                    cv2.putText(h_img, tag2, (bx1, max(40, by1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

            canvas[45:45 + feed_h, 15:15 + feed_w] = h_img
        else:
            cv2.rectangle(canvas, (15, 45), (15 + feed_w, 45 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM CAMARA PRINCIPAL (REALSENSE D435)...", (55, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(canvas, "1. CAMARA PRINCIPAL + PROFUNDIDAD 3D -> PUBLICANDO OBJETO A RVIZ (/arm_camera/object_markers)", (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 255, 200), 1)

        # --- PANEL DERECHO: CÁMARA DEL BRAZO (MUÑECA EYE-IN-HAND) ---
        if self.wrist_color_frame is not None:
            w_img = cv2.resize(self.wrist_color_frame, (feed_w, feed_h))
            orig_h, orig_w = self.wrist_color_frame.shape[:2]
            sx, sy = feed_w / orig_w, feed_h / orig_h

            wcx, wcy = int(feed_w / 2), int(feed_h / 2)
            cv2.drawMarker(w_img, (wcx, wcy), (0, 255, 255), cv2.MARKER_CROSS, 28, 1)

            if self.target_2d_wrist is not None:
                x1, y1, x2, y2 = self.target_2d_wrist
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                cv2.rectangle(w_img, (bx1, by1), (bx2, by2), (0, 200, 255), 2)
                cv2.putText(w_img, f"MUNECA err_u={self.wrist_error_u:+.2f}", (bx1, max(20, by1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 200, 255), 1)

            canvas[45:45 + feed_h, 645:645 + feed_w] = w_img
        else:
            cv2.rectangle(canvas, (645, 45), (645 + feed_w, 45 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM CAMARA DEL BRAZO...", (740, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(canvas, "2. CAMARA DEL BRAZO: APOYO DE CENTRADO FINO EN PRE-GRASP", (645, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 240, 255), 1)

        # --- PANEL INFERIOR: TELEMETRÍA Y ESTADO ---
        bar_y = 485
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (12, 16, 24), -1)
        cv2.line(canvas, (0, bar_y), (canvas_w, bar_y), (60, 80, 110), 1)

        state_str = self.FASE_NAMES.get(self.auto_state, "LISTO")
        cv2.putText(canvas, f"ESTADO: {state_str}", (20, bar_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2)

        obj_str = (
            f"[{self.target_3d_arm[0]:.2f}, {self.target_3d_arm[1]:.2f}, {self.target_3d_arm[2]:.2f}]m (Dist Pinza: {self.dist_tcp_to_obj_cm:.1f}cm)"
            if self.target_3d_arm is not None else "BUSCANDO..."
        )
        reach_badge = "ALCANCE IK: OK" if self.is_in_reach else "FUERA DE RANGO"
        joints_str = ", ".join([f"{v:+.1f}" for v in self.arm_joints_deg[:6]])

        telemetry_line1 = f"Objeto 3D en RViz: {obj_str} | {reach_badge} | Pinza: {self.gripper_opening_mm:.1f} mm"
        telemetry_line2 = f"Joints [J1..J6]: [{joints_str}] deg | {self.status_message}"

        cv2.putText(canvas, telemetry_line1, (20, bar_y + 56), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 255, 180), 1)
        cv2.putText(canvas, telemetry_line2, (20, bar_y + 82), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (200, 220, 240), 1)

        if self.is_teaching_mode:
            cv2.rectangle(canvas, (15, 370), (canvas_w - 15, 470), (0, 180, 255), -1)
            cv2.rectangle(canvas, (15, 370), (canvas_w - 15, 470), (255, 255, 255), 2)
            t_name = (self.teaching_target or "pose").upper()
            cv2.putText(canvas, f">>> MODO ENSENANZA [{t_name}] ACTIVO (BRAZO LIBERADO) <<<", (180, 412), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 0, 0), 2)
            cv2.putText(canvas, "Mueva el brazo con la mano y presione [ENTER] o [E] para GUARDAR | [C] Cancelar", (140, 448), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (15, 15, 15), 2)

        shortcuts = "[2]/[h] Home (Shift+H Ensenar) | [3]/[p] Pre-Grasp ([E] Ensenar) | [ESPACIO]/[G] Ir a Pre-Grasp y Agarrar con IK | [W/S/A/D] Cabezal | [R] Abrir Pinza"
        cv2.putText(canvas, shortcuts, (20, bar_y + 115), cv2.FONT_HERSHEY_SIMPLEX, 0.39, (255, 205, 50), 1)

        return canvas

    # --------------------------------------------------------------------------
    # BUCLE PRINCIPAL INTERACTIVO
    # --------------------------------------------------------------------------
    def run(self):
        window_name = "Robot @Home - Manipulacion 3D en RViz + Cinematica Inversa"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 640)

        print("==========================================================================")
        print("   ROBOT @HOME - OBJETO 3D EN RVIZ (PROFUNDIDAD) + PRE-GRASP + IK         ")
        print("--------------------------------------------------------------------------")
        print("   * En RViz (/arm_camera/object_markers) se muestra el cilindro 3D del   ")
        print("     objeto medido por la profundidad de la camara principal y la flecha  ")
        print("     desde la pinza hasta el objeto.                                      ")
        print("--------------------------------------------------------------------------")
        print("   [2] o [h]       : Mover a Posicion HOME      | [Shift+H] : Ensenar HOME")
        print("   [3] o [p]       : Mover a Posicion PRE-GRASP | [E]       : Ensenar PRE-GRASP")
        print("   [ESPACIO] o [G] : EJECUTAR SECUENCIA COMPLETA:                         ")
        print("                     1) Fija posicion 3D del objeto en RViz               ")
        print("                     2) El brazo va a la posicion PRE-GRASP               ")
        print("                     3) Con Cinematica Inversa va hacia el objeto y agarra")
        print("                     4) Eleva +8 cm y regresa a HOME                      ")
        print("   [W]/[S]/[A]/[D] : Mover cabezal Pan-Tilt     | [T] : Mirar a la mesa   ")
        print("   [R] / [O]       : Abrir pinza (70 mm)        | [C] : Cerrar (55 mm)    ")
        print("==========================================================================")

        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:
                break

            now = time.time()
            self.update_perception_and_rviz(now)
            self.update_autonomous_state_machine(now)

            canvas = self.draw_hud()
            cv2.imshow(window_name, canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                if self.is_teaching_mode:
                    self.set_arm_torque(True)
                break

            elif key == ord("1"):
                self.fase_actual = self.FASE_1_RANGO
                self.status_message = f"FASE 1: Caja segura (m): {SAFE_CARTESIAN_BOX_M}"

            elif key in [ord("2"), ord("h")]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "home")
                else:
                    self.auto_state = self.AUTO_IDLE
                    self.mover_a_home()

            elif key == ord("H"):
                self.toggle_teaching_pose("home")

            elif key in [ord("3"), ord("p")]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "pre_grasp")
                else:
                    self.auto_state = self.AUTO_IDLE
                    self.mover_a_pre_grasp()

            elif key in [ord("e"), ord("E"), ord("P"), 13]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "pre_grasp")
                else:
                    self.toggle_teaching_pose("pre_grasp")

            elif key in [ord("g"), ord("G"), 32]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "pre_grasp")
                else:
                    self.start_autonomous_grasp()

            elif key in [ord("r"), ord("R"), ord("o"), ord("O")]:
                msg = Float32()
                msg.data = 70.0
                self.pub_gripper_cmd.publish(msg)
                self.status_message = "Pinza abierta a 70 mm."
            elif key in [ord("c"), ord("C")]:
                if self.is_teaching_mode:
                    self.cancel_teaching_mode()
                else:
                    msg = Float32()
                    msg.data = 55.0
                    self.pub_gripper_cmd.publish(msg)
                    self.status_message = "Pinza cerrada a 55 mm."

            elif key in [ord("w"), ord("W")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                delta = -4.0 if self.pan_tilt_inverted else 4.0
                self.send_pan_tilt_goal(cur_pan, min(35.0, max(-35.0, cur_tilt + delta)))
            elif key in [ord("s"), ord("S")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                delta = 4.0 if self.pan_tilt_inverted else -4.0
                self.send_pan_tilt_goal(cur_pan, min(35.0, max(-35.0, cur_tilt + delta)))
            elif key in [ord("a"), ord("A")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                delta = -5.0 if self.pan_tilt_inverted else 5.0
                self.send_pan_tilt_goal(min(90.0, max(-90.0, cur_pan + delta)), cur_tilt)
            elif key in [ord("d"), ord("D")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                delta = 5.0 if self.pan_tilt_inverted else -5.0
                self.send_pan_tilt_goal(min(90.0, max(-90.0, cur_pan + delta)), cur_tilt)
            elif key in [ord("t"), ord("T")]:
                tilt_target = 18.0 if self.pan_tilt_inverted else -18.0
                self.send_pan_tilt_goal(0.0, tilt_target)

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Manipulacion 3D en RViz + Pre-Grasp + Cinematica Inversa")
    parser.add_argument("--dry-run", action="store_true", help="Modo simulacion sin mover motores")
    args = parser.parse_args()

    rclpy.init()
    node = ManipulacionAutonomaNode(dry_run=args.dry_run)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
