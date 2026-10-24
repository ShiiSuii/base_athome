#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Robot @Home - Sistema Simplificado de Manipulación Eye-in-Hand + RViz 3D
#
# Flujo Simplificado Basado en la Cámara del Brazo (Orbbec DaBai en la Muñeca):
#   1. Detección 100% con la cámara del brazo (/arm_camera/image_raw):
#      - Calcula desviación lateral (cm), desviación vertical (cm) y distancia
#        directa de la pinza al objeto (cm).
#   2. Visualización 3D en tiempo real en RViz (/arm_camera/object_markers):
#      - Dibuja el cilindro 3D de la botella frente a la mano del robot, la
#        flecha de alineación desde la pinza y el texto flotante con la
#        distancia en cm y el estado de alineación.
#   3. Alineación y Agarre Directo en Espacio Articular (Sin IK global frágil):
#      - Paso 1: Ir a Posición Pre-Grasp enseñada ([3]).
#      - Paso 2: Alinear la pinza centrando el objeto en la cámara del brazo ([L]).
#      - Paso 3: Avanzar suavemente hacia el objeto según la distancia medida ([F]).
#      - Paso 4: Cerrar pinza (55 mm) y elevar verticalmente (joint1 - 8°).
#      - Paso 5: Retornar con el objeto a la Posición Home ([2]).
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
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray

# Asegurar acceso a ultralytics del venv de visión
VENV_SITE_PACKAGES = "/home/athome/ros2_vision_venv/lib/python3.12/site-packages"
if os.path.exists(VENV_SITE_PACKAGES) and VENV_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, VENV_SITE_PACKAGES)

from ultralytics import YOLO

# Módulo cinemático del Brazo Piper (para FK exacta en RViz y límites seguros)
from base_athome_perception.piper_kinematics import (
    is_in_safe_joint_envelope,
    PiperKinematicsSolver,
    SAFE_CARTESIAN_BOX_M,
    SAFE_JOINT_LIMITS_DEG,
)

CONFIG_FILE_PATH = "/home/athome/base_athome_ws/src/base_athome_perception/config/arm_poses_and_limits.yaml"

# Posición del hombro izquierdo (arm_base_link) respecto a base_link en el URDF:
SHOULDER_IN_BASE_LINK_M = np.array([0.155, 0.175, 1.015], dtype=float)

# Vector de avance lineal suave por cada +1 cm hacia adelante desde Pre-Grasp (en grados/cm):
# Mueve la pinza hacia adelante en +X mientras Joint2 disminuye suavemente (alejándose del tope de 182.7°)
ADVANCE_STEP_DEG_PER_CM = np.array([-0.55, -0.40, -1.90, 0.0, +2.80, 0.0], dtype=float)


class ManipulacionAutonomaNode(Node):
    """Nodo de manipulación guiado por la cámara del brazo con visualización 3D en RViz."""

    # Fases de Configuración
    FASE_1_RANGO = 1
    FASE_2_HOME = 2
    FASE_3_PREGRASP = 3
    FASE_4_GRASP = 4

    # Estados del Ciclo Autónomo Simplificado (Eye-in-Hand)
    AUTO_IDLE = 10
    AUTO_PASO_1_IR_PREGRASP = 11       # Ir a pose Pre-Grasp enseñada
    AUTO_PASO_2_ALINEAR_BRAZO = 12     # Centrar botella con cámara del brazo
    AUTO_PASO_3_AVANZAR_OBJETO = 13    # Avanzar según distancia medida
    AUTO_PASO_4_AGARRE_ELEVACION = 14  # Cerrar pinza y elevar
    AUTO_PASO_5_RETORNO_HOME = 15      # Volver a Home con botella
    AUTO_SUCCESS = 16

    FASE_NAMES = {
        FASE_1_RANGO: "FASE 1: RANGO DE MOVIMIENTOS SEGURO",
        FASE_2_HOME: "FASE 2: POSICION HOME DEL BRAZO",
        FASE_3_PREGRASP: "FASE 3: POSICION PRE-GRASP DEL BRAZO",
        FASE_4_GRASP: "FASE 4: POSICION FINAL DE AGARRE",
        AUTO_IDLE: "LISTO: [ESPACIO]/[G] AGARRE AUTO | [L] ALINEAR | [F] AVANZAR +2CM",
        AUTO_PASO_1_IR_PREGRASP: "PASO 1: MOVIENDO A POSICION PRE-GRASP",
        AUTO_PASO_2_ALINEAR_BRAZO: "PASO 2: ALINEANDO PINZA CON CAMARA DEL BRAZO",
        AUTO_PASO_3_AVANZAR_OBJETO: "PASO 3: AVANZANDO HACIA EL OBJETO DETECTADO",
        AUTO_PASO_4_AGARRE_ELEVACION: "PASO 4: CERRANDO PINZA Y ELEVANDO OBJETO",
        AUTO_PASO_5_RETORNO_HOME: "PASO 5: LLEVANDO OBJETO A POSICION HOME",
        AUTO_SUCCESS: "CICLO COMPLETADO: OBJETO EN POSICION HOME",
    }

    def __init__(self, dry_run: bool = False):
        super().__init__("manipulacion_autonoma_node")
        self.dry_run = dry_run
        self.get_logger().info("Iniciando Sistema Eye-in-Hand + RViz 3D de Manipulacion...")

        # 1. Cargar Poses desde YAML
        self.load_configuration()

        # 2. Subscripción a Cámara del Brazo (Orbbec DaBai Eye-in-Hand - Principal)
        qos_cam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_wrist_cam = self.create_subscription(
            Image, "/arm_camera/image_raw", self.wrist_cam_callback, qos_cam
        )

        # 3. Subscripción opcional a Cámara del Cabezal (para vista secundaria en HUD)
        self.sub_head_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.head_color_callback, qos_cam
        )
        self.sub_head_depth = self.create_subscription(
            Image, "/camera/camera/aligned_depth_to_color/image_raw", self.head_depth_callback, qos_cam
        )

        # 4. Telemetría del Brazo Piper y Pan-Tilt
        self.sub_arm_status = self.create_subscription(
            String, "/arm/status", self.arm_status_callback, 10
        )
        self.sub_pan_tilt = self.create_subscription(
            JointState, "/pan_tilt/joint_states", self.pan_tilt_callback, 10
        )

        # 5. Publicadores ROS 2 (incluyendo MarkerArray para RViz)
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_joint_cmd = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_gripper_cmd = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_pan_tilt_cmd = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)
        self.pub_enable = self.create_publisher(Bool, "/arm/enable", 5)
        self.pub_rviz_markers = self.create_publisher(MarkerArray, "/arm_camera/object_markers", 10)

        # 6. Servicios de Manipulación
        self.cli_set_enable = self.create_client(SetBool, "/arm/set_enable")

        # Buffers de imagen
        self.wrist_color_frame: Optional[np.ndarray] = None
        self.head_color_frame: Optional[np.ndarray] = None
        self.head_depth_frame: Optional[np.ndarray] = None

        # Ángulos del cabezal Pan-Tilt
        self.head_pan_rad = 0.0
        self.head_tilt_rad = 0.0
        self.pan_tilt_inverted = True

        # Modo de Enseñanza con la Mano (Teach Mode)
        self.is_teaching_mode = False
        self.teaching_target: Optional[str] = None  # 'home', 'pre_grasp', o 'grasp'

        # Telemetría del brazo Piper
        self.arm_is_moving = False
        self.gripper_opening_mm = 69.5
        self.arm_xyz_mm = [0.0, 0.0, 0.0]
        self.arm_joints_deg = list(self.pose_home)
        self.current_working_joints_deg = list(self.pose_pre_grasp)

        # Solucionador cinemático (para FK exacta de los marcadores 3D en RViz)
        self.kinematics_solver = PiperKinematicsSolver()

        # Modelo YOLO para detección en la cámara del brazo
        self.get_logger().info("Cargando modelo YOLO para deteccion en camara del brazo...")
        try:
            self.yolo_model = YOLO("yolov8s-worldv2.pt")
            self.yolo_model.set_classes(["bottle", "pringles can", "can", "cup", "cylinder"])
        except Exception:
            self.yolo_model = YOLO("yolov8n.pt")

        # Estado y mediciones en vivo de la Cámara del Brazo
        self.fase_actual = self.FASE_3_PREGRASP
        self.auto_state = self.AUTO_IDLE
        self.step_start_time = time.time()
        self.align_attempts = 0
        self.last_align_cmd_time = 0.0

        self.target_2d_wrist: Optional[Tuple[int, int, int, int]] = None
        self.wrist_conf = 0.0
        self.wrist_error_u = 0.0         # [-1.0 izq .. +1.0 der]
        self.wrist_error_v = 0.0         # [-1.0 arriba .. +1.0 abajo]
        self.wrist_distance_cm = 0.0     # Distancia estimada Pinza -> Botella en cm
        self.wrist_offset_x_cm = 0.0     # Desviación lateral en cm
        self.wrist_offset_y_cm = 0.0     # Desviación vertical en cm
        self.is_wrist_aligned = False
        self.last_wrist_det_time = 0.0
        self.last_marker_pub_time = 0.0

        # Parámetros ópticos calibrados de la cámara de muñeca Orbbec DaBai (640x480)
        self.wrist_fx = 525.0
        self.bottle_real_diameter_cm = 7.4  # Diámetro típico botella / tubo Pringles (7.4 cm)

        self.status_message = "Listo. Coloque el objeto frente a la camara del brazo y presione [ESPACIO] o [L]."

    # --------------------------------------------------------------------------
    # CONFIGURACIÓN Y PERSISTENCIA (YAML)
    # --------------------------------------------------------------------------
    def load_configuration(self):
        """Carga poses nombradas desde YAML."""
        self.pose_home = [86.28, 173.91, -164.0, 66.98, 0.69, -146.41]
        self.pose_pre_grasp = [30.96, 182.68, -98.68, 37.3, -75.12, -95.86]
        self.pose_grasp: Optional[List[float]] = None
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
                    if "grasp" in poses and "joints_deg" in poses["grasp"]:
                        self.pose_grasp = list(poses["grasp"]["joints_deg"])
                self.get_logger().info("Configuracion de poses cargada desde YAML.")
            except Exception as e:
                self.get_logger().warn(f"Error leyendo YAML ({e}), usando poses por defecto.")

    def save_named_pose(self, pose_name: str, joints_deg: List[float], gripper_mm: float):
        """Guarda una pose enseñada con la mano en el archivo YAML."""
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
            self.say(f"Pose {pose_name} saved successfully.")
            self.load_configuration()
        except Exception as e:
            self.get_logger().error(f"Error guardando pose en YAML: {e}")

    # --------------------------------------------------------------------------
    # CALLBACKS DE SENSORES Y TELEMETRÍA
    # --------------------------------------------------------------------------
    def wrist_cam_callback(self, msg: Image):
        try:
            self.wrist_color_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
        except Exception:
            pass

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
    # DETECCIÓN EN CÁMARA DEL BRAZO Y MARCADORES 3D EN RVIZ
    # --------------------------------------------------------------------------
    def detect_object_in_frame(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """Detecta la botella o tubo Pringles en el frame de la cámara del brazo."""
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

        # Respaldo por segmentación de color (objetos cilíndricos rojos / brillantes)
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

    def update_wrist_perception(self, now: float):
        """
        Procesa en todo momento la imagen de la cámara del brazo (/arm_camera/image_raw),
        calcula la distancia en cm y la desviación lateral/vertical, y actualiza RViz.
        """
        if self.wrist_color_frame is None:
            return

        det = self.detect_object_in_frame(self.wrist_color_frame)
        if det is not None:
            x1, y1, x2, y2, conf = det
            self.target_2d_wrist = (x1, y1, x2, y2)
            self.wrist_conf = conf
            self.last_wrist_det_time = now

            h, w = self.wrist_color_frame.shape[:2]
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            bw = max(12.0, float(x2 - x1))

            # Errores normalizados [-1, +1] respecto al centro de la pinza
            err_u = (cx - (w / 2.0)) / (w / 2.0)
            err_v = (cy - (h / 2.0)) / (h / 2.0)
            self.wrist_error_u = 0.6 * self.wrist_error_u + 0.4 * err_u
            self.wrist_error_v = 0.6 * self.wrist_error_v + 0.4 * err_v

            # Distancia estimada desde la cámara del brazo hasta la botella (en cm)
            raw_dist_cm = (self.wrist_fx * self.bottle_real_diameter_cm) / bw
            raw_dist_cm = float(np.clip(raw_dist_cm, 6.0, 65.0))
            if self.wrist_distance_cm <= 0.1:
                self.wrist_distance_cm = raw_dist_cm
            else:
                self.wrist_distance_cm = 0.65 * self.wrist_distance_cm + 0.35 * raw_dist_cm

            # Desviación métrica en cm en el plano del objeto
            self.wrist_offset_x_cm = self.wrist_error_u * self.wrist_distance_cm * 0.52
            self.wrist_offset_y_cm = self.wrist_error_v * self.wrist_distance_cm * 0.39

            # Consideramos alineado cuando la desviación lateral es menor a 2.2 cm
            self.is_wrist_aligned = (abs(self.wrist_offset_x_cm) <= 2.2) and (abs(self.wrist_offset_y_cm) <= 3.5)
        else:
            if now - self.last_wrist_det_time > 0.8:
                self.target_2d_wrist = None
                self.is_wrist_aligned = False

        # Publicar marcadores 3D a RViz a ~15 Hz
        if now - self.last_marker_pub_time >= 0.065:
            self.last_marker_pub_time = now
            self.publish_rviz_object_markers()

    def publish_rviz_object_markers(self):
        """
        Publica en /arm_camera/object_markers (MarkerArray) para visualizar en RViz:
          1. Cilindro 3D representando el objeto detectado frente al brazo.
          2. Flecha guía desde la pinza (TCP) hacia el objeto.
          3. Texto 3D flotante con la distancia en cm y estado de alineación.
        """
        ma = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        if self.target_2d_wrist is None or self.wrist_distance_cm <= 0.1:
            # Borrar marcadores si no hay objeto detectado
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

        # Calcular posición 3D actual del TCP de la pinza en base_link
        q_rad = [math.radians(d) for d in self.arm_joints_deg[:6]]
        xyz_sh_mm, R_sh = self.kinematics_solver.forward_kinematics(q_rad)
        tcp_base_m = SHOULDER_IN_BASE_LINK_M + (xyz_sh_mm / 1000.0)

        # Dirección frontal de la pinza (+Z del TCP) proyectada horizontalmente hacia adelante
        fwd_vec = R_sh[:, 2].copy()
        norm_fwd = float(np.linalg.norm(fwd_vec))
        if norm_fwd > 1e-4:
            fwd_vec /= norm_fwd
        else:
            fwd_vec = np.array([1.0, 0.0, 0.0])

        dist_m = self.wrist_distance_cm / 100.0
        lat_m = -self.wrist_offset_x_cm / 100.0   # +Y en base_link es izquierda
        vert_m = -self.wrist_offset_y_cm / 100.0  # +Z en base_link es arriba

        obj_pos_m = np.array([
            tcp_base_m[0] + dist_m * max(0.5, fwd_vec[0]),
            tcp_base_m[1] + dist_m * fwd_vec[1] + lat_m,
            tcp_base_m[2] + vert_m,
        ], dtype=float)

        # Color: Verde brillante si está alineado, Naranja si está desalineado
        if self.is_wrist_aligned:
            col = ColorRGBA(r=0.10, g=0.95, b=0.30, a=0.90)
            align_tag = "ALINEADO"
        else:
            col = ColorRGBA(r=1.00, g=0.60, b=0.05, a=0.90)
            align_tag = "ALINEANDO"

        # 1. Cilindro 3D del Objeto (Botella / Pringles)
        cyl = Marker()
        cyl.header.frame_id = "base_link"
        cyl.header.stamp = now_msg
        cyl.ns = "arm_target"
        cyl.id = 0
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x = float(obj_pos_m[0])
        cyl.pose.position.y = float(obj_pos_m[1])
        cyl.pose.position.z = float(obj_pos_m[2])
        cyl.pose.orientation.w = 1.0
        cyl.scale.x = 0.075
        cyl.scale.y = 0.075
        cyl.scale.z = 0.21
        cyl.color = col
        cyl.lifetime = Duration(sec=0, nanosec=400000000)
        ma.markers.append(cyl)

        # 2. Flecha Guía desde el TCP de la Pinza hasta el Objeto
        arr = Marker()
        arr.header.frame_id = "base_link"
        arr.header.stamp = now_msg
        arr.ns = "arm_target"
        arr.id = 1
        arr.type = Marker.ARROW
        arr.action = Marker.ADD
        p_start = Point(x=float(tcp_base_m[0]), y=float(tcp_base_m[1]), z=float(tcp_base_m[2]))
        p_end = Point(x=float(obj_pos_m[0]), y=float(obj_pos_m[1]), z=float(obj_pos_m[2]))
        arr.points = [p_start, p_end]
        arr.scale.x = 0.012  # Diámetro del eje de la flecha
        arr.scale.y = 0.024  # Diámetro de la punta
        arr.scale.z = 0.030
        arr.color = ColorRGBA(r=0.0, g=0.88, b=1.0, a=0.95)
        arr.lifetime = Duration(sec=0, nanosec=400000000)
        ma.markers.append(arr)

        # 3. Texto 3D Flotante con Distancia y Desviación (100% ASCII para compatibilidad DDS)
        txt = Marker()
        txt.header.frame_id = "base_link"
        txt.header.stamp = now_msg
        txt.ns = "arm_target"
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = float(obj_pos_m[0])
        txt.pose.position.y = float(obj_pos_m[1])
        txt.pose.position.z = float(obj_pos_m[2] + 0.16)
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.045
        txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        txt.text = f"OBJETO: {self.wrist_distance_cm:.1f} cm | DesvX: {self.wrist_offset_x_cm:+.1f} cm [{align_tag}]"
        txt.lifetime = Duration(sec=0, nanosec=400000000)
        ma.markers.append(txt)

        self.pub_rviz_markers.publish(ma)

    # --------------------------------------------------------------------------
    # COMANDOS DE ACCIONAMIENTO Y SERVO VISUAL DIRECTO (SIN IK COMPLEJA)
    # --------------------------------------------------------------------------
    def say(self, text: str):
        """Síntesis de voz (ASCII puro para evitar errores de serialización DDS)."""
        clean_ascii = text.encode("ascii", "ignore").decode("ascii")
        msg = String()
        msg.data = clean_ascii
        self.pub_say.publish(msg)
        self.status_message = clean_ascii
        print(f"[Robot]: \"{clean_ascii}\"")

    def clamp_joints_to_safe_limits(self, joints_deg: List[float]) -> List[float]:
        """Recorta suavemente cada articulación a sus límites físicos reales."""
        clamped = list(joints_deg[:6])
        for i in range(6):
            low, high = SAFE_JOINT_LIMITS_DEG[f"joint{i+1}"]
            # Asegurar que Joint 2 nunca supere el tope físico de 182.7°
            if i == 1:
                high = min(high, 182.7)
            clamped[i] = float(np.clip(clamped[i], low, high))
        return [round(v, 2) for v in clamped]

    def send_arm_joint_goal(self, joints_deg: List[float], duration_s: float = 2.0, gripper_mm: Optional[float] = None) -> bool:
        """Envía comando articular seguro a /arm/joint_cmd."""
        safe_joints = self.clamp_joints_to_safe_limits(joints_deg)
        safe, reason = is_in_safe_joint_envelope(safe_joints)
        if not safe:
            self.get_logger().error(f"[SEGURIDAD] Comando bloqueado: {reason}")
            self.status_message = f"BLOQUEO SEGURIDAD: {reason}"
            return False

        self.current_working_joints_deg = list(safe_joints)

        if self.dry_run:
            self.get_logger().info(f"[DRY-RUN] joint_cmd: {safe_joints}, dur={duration_s}s, pinza={gripper_mm}mm")
            return True

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [math.radians(d) for d in safe_joints]
        if gripper_mm is not None:
            js.name.append("gripper")
            js.position.append(float(gripper_mm))
        js.velocity = [float(duration_s)]
        self.pub_joint_cmd.publish(js)

        if gripper_mm is not None:
            g_msg = Float32()
            g_msg.data = float(gripper_mm)
            self.pub_gripper_cmd.publish(g_msg)

        return True

    def execute_visual_alignment_step(self) -> bool:
        """
        Realiza 1 paso simple y directo de alineación articular usando la cámara del brazo:
          - Corrige desviación lateral (err_u) girando suavemente la muñeca/codo (Joint5 y Joint3).
          - Corrige desviación vertical (err_v) ajustando suavemente el hombro (Joint1).
        Retorna True si ya está alineado, False si envió un ajuste para seguir alineando.
        """
        if self.target_2d_wrist is None:
            self.status_message = "No se detecta objeto en la camara del brazo para alinear."
            return False

        if self.is_wrist_aligned:
            self.status_message = f"ALINEADO con el objeto! Distancia: {self.wrist_distance_cm:.1f} cm."
            return True

        base_q = list(self.arm_joints_deg[:6])
        # Ajuste proporcional directo suave (sin IK):
        # Si err_u > 0 (objeto a la derecha en la imagen): incrementar J5 y J3 apunta hacia la derecha (-Y)
        d_j5 = float(np.clip(self.wrist_error_u * 6.0, -5.0, 5.0))
        d_j3 = float(np.clip(self.wrist_error_u * 2.0, -2.5, 2.5))
        # Si err_v > 0 (objeto abajo en la imagen): incrementar J1 baja la pinza (-Z)
        d_j1 = float(np.clip(self.wrist_error_v * 3.0, -3.0, 3.0))

        new_q = [
            base_q[0] + d_j1,
            base_q[1],
            base_q[2] + d_j3,
            base_q[3],
            base_q[4] + d_j5,
            base_q[5],
        ]
        self.get_logger().info(
            f"[Alineacion Visual] DesvX={self.wrist_offset_x_cm:+.1f}cm, Dist={self.wrist_distance_cm:.1f}cm -> "
            f"Ajustando dJ1={d_j1:+.1f} deg, dJ3={d_j3:+.1f} deg, dJ5={d_j5:+.1f} deg"
        )
        self.send_arm_joint_goal(new_q, duration_s=1.1, gripper_mm=70.0)
        self.status_message = f"Alineando con camara del brazo (DesvX={self.wrist_offset_x_cm:+.1f}cm, Dist={self.wrist_distance_cm:.1f}cm)..."
        return False

    def execute_forward_advance_step(self, step_cm: float = 2.5):
        """
        Avanza la pinza hacia adelante `step_cm` centímetros en línea recta usando
        el vector articular seguro calibrado (donde Joint2 disminuye suavemente).
        """
        base_q = np.array(self.arm_joints_deg[:6], dtype=float)
        # Incluir corrección fina lateral si el objeto es visible
        lat_corr_j5 = float(np.clip(self.wrist_error_u * 3.5, -3.5, 3.5)) if self.target_2d_wrist else 0.0
        target_q = base_q + step_cm * ADVANCE_STEP_DEG_PER_CM
        target_q[4] += lat_corr_j5

        self.get_logger().info(f"[Avance Directo] Avanzando +{step_cm:.1f} cm hacia el objeto -> Joints={np.round(target_q, 1).tolist()}")
        self.send_arm_joint_goal(target_q.tolist(), duration_s=1.4, gripper_mm=70.0)
        self.status_message = f"Avanzando +{step_cm:.1f} cm hacia el objeto (Dist actual: {self.wrist_distance_cm:.1f} cm)."

    # --------------------------------------------------------------------------
    # MOVIMIENTOS A POSES Y ENSEÑANZA MANUAL (TEACH MODE)
    # --------------------------------------------------------------------------
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

    def mover_a_grasp_ensenado(self, duration_s: float = 2.0):
        if self.is_teaching_mode:
            self.cancel_teaching_mode()
        self.fase_actual = self.FASE_4_GRASP
        if self.pose_grasp is not None:
            self.send_arm_joint_goal(self.pose_grasp, duration_s=duration_s, gripper_mm=70.0)
            self.status_message = "Moviendo a Posicion GRASP ensenada."
        else:
            self.execute_forward_advance_step(step_cm=5.5)

    def set_arm_torque(self, enable: bool):
        msg = Bool()
        msg.data = bool(enable)
        self.pub_enable.publish(msg)
        if self.cli_set_enable.service_is_ready():
            req = SetBool.Request()
            req.data = bool(enable)
            self.cli_set_enable.call_async(req)

    def toggle_teaching_pose(self, target_pose: str):
        """Activa o guarda el modo enseñanza con la mano para 'home', 'pre_grasp', o 'grasp'."""
        if not self.is_teaching_mode:
            self.is_teaching_mode = True
            self.teaching_target = target_pose
            self.set_arm_torque(False)
            if target_pose in ["pre_grasp", "grasp"]:
                msg_g = Float32()
                msg_g.data = 70.0
                self.pub_gripper_cmd.publish(msg_g)
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
    # MÁQUINA DE ESTADOS DEL CICLO AUTÓNOMO SIMPLIFICADO CON CÁMARA DEL BRAZO
    # --------------------------------------------------------------------------
    def start_autonomous_grasp(self):
        """Inicia la secuencia simplificada guiada por la cámara del brazo."""
        self.auto_state = self.AUTO_PASO_1_IR_PREGRASP
        self.step_start_time = time.time()
        self.align_attempts = 0
        self._advance_sent = False
        self._lift_sent = False
        self.say("Starting wrist camera visual alignment and grasping sequence.")
        self.mover_a_pre_grasp(duration_s=2.3)

    def update_autonomous_state_machine(self, now: float):
        if self.auto_state == self.AUTO_IDLE:
            return

        # PASO 1: LLEGAR A PRE-GRASP
        if self.auto_state == self.AUTO_PASO_1_IR_PREGRASP:
            if now - self.step_start_time > 2.5:
                self.auto_state = self.AUTO_PASO_2_ALINEAR_BRAZO
                self.step_start_time = now
                self.last_align_cmd_time = now - 1.0
                self.align_attempts = 0
                self.say("Pre-grasp reached. Aligning gripper with wrist camera.")

        # PASO 2: ALINEAR LA PINZA CON EL OBJETO USANDO LA CÁMARA DEL BRAZO
        elif self.auto_state == self.AUTO_PASO_2_ALINEAR_BRAZO:
            if self.target_2d_wrist is None:
                self.status_message = "Buscando botella en la camara del brazo (/arm_camera/image_raw)..."
                if now - self.step_start_time > 8.0:
                    self.say("No object detected in wrist camera. Returning to idle.")
                    self.auto_state = self.AUTO_IDLE
                return

            # Si ya está alineado o completamos hasta 3 micro-ajustes suaves, pasar a Avanzar!
            if self.is_wrist_aligned or self.align_attempts >= 3:
                self.auto_state = self.AUTO_PASO_3_AVANZAR_OBJETO
                self.step_start_time = now
                self._advance_sent = False
                self.say(f"Aligned with bottle at {self.wrist_distance_cm:.0f} centimeters. Advancing to grasp.")
                return

            if now - self.last_align_cmd_time > 1.3:
                self.last_align_cmd_time = now
                self.align_attempts += 1
                self.execute_visual_alignment_step()

        # PASO 3: AVANZAR HACIA LA BOTELLA
        elif self.auto_state == self.AUTO_PASO_3_AVANZAR_OBJETO:
            step_elapsed = now - self.step_start_time
            if not getattr(self, "_advance_sent", False) and step_elapsed > 0.3:
                self._advance_sent = True
                if self.pose_grasp is not None:
                    # Si el usuario enseñó la pose exacta de agarre [4], ir a ella aplicando el centrado visual
                    target_q = list(self.pose_grasp)
                    if self.target_2d_wrist is not None:
                        target_q[4] += float(np.clip(self.wrist_error_u * 4.5, -5.0, 5.0))
                    self.send_arm_joint_goal(target_q, duration_s=1.8, gripper_mm=70.0)
                else:
                    # Calcular cuántos cm avanzar según la distancia medida por la cámara del brazo
                    dist_to_advance_cm = float(np.clip(self.wrist_distance_cm - 8.5, 4.0, 8.5))
                    self.execute_forward_advance_step(step_cm=dist_to_advance_cm)

            if step_elapsed > 2.2:
                self.auto_state = self.AUTO_PASO_4_AGARRE_ELEVACION
                self.step_start_time = now
                self._lift_sent = False
                self.get_logger().info("[Autonomo] Paso 4: Cerrando pinza a 55 mm...")
                if not self.dry_run:
                    g_msg = Float32()
                    g_msg.data = 55.0
                    self.pub_gripper_cmd.publish(g_msg)

        # PASO 4: CERRAR PINZA Y ELEVAR VERTICALMENTE (SIN IK)
        elif self.auto_state == self.AUTO_PASO_4_AGARRE_ELEVACION:
            step_elapsed = now - self.step_start_time
            if step_elapsed > 1.5 and not getattr(self, "_lift_sent", False):
                self._lift_sent = True
                # Elevar rotando suavemente Joint1 -8 grados (sube la pinza +7.3 cm en Z_base!)
                lift_q = list(self.arm_joints_deg[:6])
                lift_q[0] -= 8.0
                self.get_logger().info(f"[Autonomo] Elevando botella (Joint1 -8 deg) -> {lift_q}")
                self.send_arm_joint_goal(lift_q, duration_s=1.6, gripper_mm=55.0)

            if step_elapsed > 3.4:
                self.auto_state = self.AUTO_PASO_5_RETORNO_HOME
                self.step_start_time = now
                self.say("Bottle grasped and lifted. Returning to home position.")
                self.mover_a_home(duration_s=2.8, keep_gripper=True)

        # PASO 5: RETORNAR A HOME CON LA BOTELLA
        elif self.auto_state == self.AUTO_PASO_5_RETORNO_HOME:
            if now - self.step_start_time > 3.0:
                self.auto_state = self.AUTO_SUCCESS
                self.step_start_time = now
                self.say("Bottle successfully grasped and brought to home position! I am Robot.")

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

        # --- PANEL IZQUIERDO: CÁMARA DEL BRAZO (PRINCIPAL - EYE-IN-HAND) ---
        if self.wrist_color_frame is not None:
            w_img = cv2.resize(self.wrist_color_frame, (feed_w, feed_h))
            orig_h, orig_w = self.wrist_color_frame.shape[:2]
            sx, sy = feed_w / orig_w, feed_h / orig_h

            # Retículo central de alineación de la pinza
            wcx, wcy = int(feed_w / 2), int(feed_h / 2)
            ret_col = (0, 255, 0) if self.is_wrist_aligned else (0, 220, 255)
            cv2.drawMarker(w_img, (wcx, wcy), ret_col, cv2.MARKER_CROSS, 34, 2)
            cv2.circle(w_img, (wcx, wcy), 24, ret_col, 1)

            if self.target_2d_wrist is not None:
                x1, y1, x2, y2 = self.target_2d_wrist
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                obj_cx, obj_cy = int((bx1 + bx2) / 2), int((by1 + by2) / 2)

                box_col = (0, 255, 80) if self.is_wrist_aligned else (0, 165, 255)
                cv2.rectangle(w_img, (bx1, by1), (bx2, by2), box_col, 2)
                # Vector visual desde el centro de la pinza hasta el centro del objeto
                cv2.arrowedLine(w_img, (wcx, wcy), (obj_cx, obj_cy), (255, 255, 0), 2, tipLength=0.2)

                align_txt = "ALINEADO" if self.is_wrist_aligned else "ALINEAR"
                label1 = f"DISTANCIA: {self.wrist_distance_cm:.1f} cm [{align_txt}]"
                label2 = f"DesvX: {self.wrist_offset_x_cm:+.1f} cm | DesvY: {self.wrist_offset_y_cm:+.1f} cm"
                cv2.putText(w_img, label1, (bx1, max(22, by1 - 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_col, 2)
                cv2.putText(w_img, label2, (bx1, max(40, by1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

            canvas[45:45 + feed_h, 15:15 + feed_w] = w_img
        else:
            cv2.rectangle(canvas, (15, 45), (15 + feed_w, 45 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM CAMARA DEL BRAZO (/arm_camera/image_raw)...", (45, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(canvas, "1. CAMARA DEL BRAZO (PRINCIPAL): DETECCION, DISTANCIA Y ALINEACION (PUBLICANDO A RVIZ)", (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 200), 1)

        # --- PANEL DERECHO: VISTA GENERAL CABEZAL + ESTADO RVIZ 3D ---
        if self.head_color_frame is not None:
            h_img = cv2.resize(self.head_color_frame, (feed_w, feed_h))
            canvas[45:45 + feed_h, 645:645 + feed_w] = h_img
        else:
            cv2.rectangle(canvas, (645, 45), (645 + feed_w, 45 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "VISTA SECUNDARIA CABEZAL REALSENSE...", (760, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(canvas, "2. CAMARA SECUNDARIA (CABEZAL) | MARCADOR 3D ACTIVO EN RVIZ: /arm_camera/object_markers", (645, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (220, 230, 255), 1)

        # --- PANEL INFERIOR: TELEMETRÍA Y ATAJOS ---
        bar_y = 485
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (12, 16, 24), -1)
        cv2.line(canvas, (0, bar_y), (canvas_w, bar_y), (60, 80, 110), 1)

        state_str = self.FASE_NAMES.get(self.auto_state, "LISTO")
        cv2.putText(canvas, f"ESTADO: {state_str}", (20, bar_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)

        dist_str = f"{self.wrist_distance_cm:.1f} cm" if self.target_2d_wrist else "SIN OBJETO"
        align_badge = "ALINEADO OK" if self.is_wrist_aligned else f"DESV: {self.wrist_offset_x_cm:+.1f} cm"
        joints_str = ", ".join([f"{v:+.1f}" for v in self.arm_joints_deg[:6]])

        telemetry_line1 = f"Camara Brazo -> Distancia: {dist_str} | {align_badge} | Pinza: {self.gripper_opening_mm:.1f} mm | RViz Topic: /arm_camera/object_markers"
        telemetry_line2 = f"Joints [J1..J6]: [{joints_str}] deg | {self.status_message}"

        cv2.putText(canvas, telemetry_line1, (20, bar_y + 56), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 180), 1)
        cv2.putText(canvas, telemetry_line2, (20, bar_y + 82), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (200, 220, 240), 1)

        # Banner de Modo Enseñanza
        if self.is_teaching_mode:
            cv2.rectangle(canvas, (15, 370), (canvas_w - 15, 470), (0, 180, 255), -1)
            cv2.rectangle(canvas, (15, 370), (canvas_w - 15, 470), (255, 255, 255), 2)
            t_name = (self.teaching_target or "pose").upper()
            cv2.putText(canvas, f">>> MODO ENSENANZA [{t_name}] ACTIVO (BRAZO LIBERADO) <<<", (180, 412), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 0, 0), 2)
            cv2.putText(canvas, "Mueva el brazo con la mano y presione [ENTER] o [E] para GUARDAR | [C] Cancelar", (140, 448), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (15, 15, 15), 2)

        shortcuts = "[2] Home (Shift+H Ensenar) | [3] Pre-Grasp ([E] Ensenar) | [4] Grasp (Shift+G Ensenar) | [L] Alinear 1 Paso | [F] Avanzar +2cm | [ESPACIO] Agarre Auto | [R] Abrir Pinza"
        cv2.putText(canvas, shortcuts, (20, bar_y + 115), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (255, 205, 50), 1)

        return canvas

    # --------------------------------------------------------------------------
    # BUCLE PRINCIPAL INTERACTIVO
    # --------------------------------------------------------------------------
    def run(self):
        window_name = "Robot @Home - Manipulacion con Camara del Brazo + RViz 3D"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 640)

        print("==========================================================================")
        print("   ROBOT @HOME - MANIPULACION SIMPLIFICADA CON CAMARA DEL BRAZO + RVIZ    ")
        print("--------------------------------------------------------------------------")
        print("   * En RViz veras el objeto 3D, la flecha desde la pinza y la distancia  ")
        print("     en el topic: /arm_camera/object_markers (MarkerArray)                ")
        print("--------------------------------------------------------------------------")
        print("   [2] o [h]       : Ir a Posicion HOME      | [Shift+H] : Ensenar HOME   ")
        print("   [3] o [p]       : Ir a Posicion PRE-GRASP | [E]       : Ensenar PRE-GRASP")
        print("   [4]             : Ir a Posicion GRASP     | [Shift+G] : Ensenar GRASP  ")
        print("--------------------------------------------------------------------------")
        print("   PASO A PASO SIMPLE CON LA CAMARA DEL BRAZO:                            ")
        print("   [L]             : ALINEAR (centra la pinza con el objeto detectado)    ")
        print("   [F]             : AVANZAR +2.5 cm hacia adelante hacia el objeto       ")
        print("   [ESPACIO] o [g] : CICLO AUTOMATICO COMPLETO (Pre-Grasp -> Alinear ->   ")
        print("                     Avanzar -> Cerrar Pinza -> Elevar -> Volver a Home)  ")
        print("   [R] / [O]       : Abrir pinza (70 mm)     | [C]       : Cerrar (55 mm) ")
        print("   [Q] o [ESC]     : Salir                                                ")
        print("==========================================================================")

        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:
                break

            now = time.time()
            self.update_wrist_perception(now)
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
                self.status_message = f"FASE 1: Limites seguros activos: {SAFE_JOINT_LIMITS_DEG}"

            elif key in [ord("2"), ord("h")]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "home")
                else:
                    self.auto_state = self.AUTO_IDLE
                    self.mover_a_home()

            elif key == ord("H"):  # Shift + H
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

            elif key == ord("4"):
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "grasp")
                else:
                    self.auto_state = self.AUTO_IDLE
                    self.mover_a_grasp_ensenado()

            elif key == ord("G"):  # Shift + G: Enseñar posición exacta de agarre (Grasp)
                self.toggle_teaching_pose("grasp")

            # PASO MANUAL 1: ALINEAR CON CÁMARA DEL BRAZO [L]
            elif key in [ord("l"), ord("L")]:
                self.auto_state = self.AUTO_IDLE
                self.execute_visual_alignment_step()

            # PASO MANUAL 2: AVANZAR +2.5 CM HACIA EL OBJETO [F]
            elif key in [ord("f"), ord("F")]:
                self.auto_state = self.AUTO_IDLE
                self.execute_forward_advance_step(step_cm=2.5)

            # CICLO AUTOMÁTICO COMPLETO [ESPACIO] o [g]
            elif key in [ord("g"), 32]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "pre_grasp")
                else:
                    self.start_autonomous_grasp()

            # PINZA
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

            # CONTROLES PAN-TILT SECUNDARIOS
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

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Manipulacion Guiada por Camara del Brazo + RViz 3D")
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
