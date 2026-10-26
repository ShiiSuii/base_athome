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
#   3. Secuencia de Agarre y Rutina Post-Grasping:
#      - Paso A: Fija el objetivo 3D en RViz y resuelve la Cinemática Inversa (IK).
#      - Paso B: El brazo va a la posición Pre-Grasp ([3]) con el GRIPPER ABIERTO AL MÁXIMO (105 mm).
#      - Paso C1: En Pre-Grasp (gripper al máximo 105 mm), se ALINEA primero con el objeto.
#      - Paso C2: Desde la postura alineada, AVANZA recto hacia el objeto (+grasp_forward_cm).
#      - Paso D: Cierra el gripper (50 mm) para agarrar el objeto y despega +2 cm de la mesa.
#      - Paso E (Rutina Post-Grasping): VUELVE RECTO HACIA ATRÁS hasta la posición Pre-Grasp
#                manteniendo el objeto agarrado.
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
    WRIST_CAMERA_RIGHT_OFFSET_M,
)

CONFIG_FILE_PATH = "/home/athome/base_athome_ws/src/base_athome_perception/config/arm_poses_and_limits.yaml"

# Posición del hombro izquierdo (arm_base_link) respecto a base_link en el URDF:
SHOULDER_IN_BASE_LINK_M = np.array([0.155, 0.175, 1.015], dtype=float)

# Apertura física máxima del gripper Piper (~104 mm reales) y cierre de agarre firme
GRIPPER_MAX_OPEN_MM = 105.0
GRIPPER_CLOSE_GRASP_MM = 50.0


class ManipulacionAutonomaNode(Node):
    """Nodo integrado de detección 3D en RViz, calibración en vivo, Pre-Grasp con apertura máxima y Rutina Post-Grasping."""

    # Fases de Configuración y Calibración
    FASE_1_RANGO = 1
    FASE_2_HOME = 2
    FASE_3_PREGRASP = 3
    FASE_4_GRASP = 4

    # Estados del Ciclo Autónomo y Rutina Post-Grasping
    AUTO_IDLE = 10
    AUTO_PASO_A_DETECCION_RVIZ = 11          # Detectar 3D con cámara principal y mostrar en RViz
    AUTO_PASO_B_MOVER_PREGRASP = 12          # Ir a Pre-Grasp con Gripper ABIERTO AL MÁXIMO (105 mm)
    AUTO_PASO_C1_ALINEAR_CON_OBJETO = 13     # PRIMERO: Alinearse lateralmente con el objeto en Pre-Grasp (105 mm)
    AUTO_PASO_C2_AVANZAR_AL_OBJETO = 14      # SEGUNDO: Avanzar recto hacia el objeto (grasp_forward_cm, 105 mm)
    AUTO_PASO_D_AGARRE = 15                  # Cerrar pinza (50 mm) y despegar +2 cm de la mesa
    AUTO_PASO_E_POSTGRASP_PREGRASP = 16      # RUTINA POST-GRASPING: Volver RECTO HACIA ATRÁS a Pre-Grasp con el objeto
    AUTO_SUCCESS = 17

    FASE_NAMES = {
        FASE_1_RANGO: "FASE 1: RANGO DE MOVIMIENTOS SEGURO",
        FASE_2_HOME: "FASE 2: POSICION HOME DEL BRAZO",
        FASE_3_PREGRASP: "FASE 3: MODO PRE-GRASP (GRIPPER ABIERTO AL MAXIMO 105 MM)",
        FASE_4_GRASP: "FASE 4: CALIBRANDO LLEGADA ([+]/[-] Avance, [9]/[0] Der/Izq, [5] Post-Grasp)",
        AUTO_IDLE: "LISTO: OBJETO 3D EN RVIZ ([ESPACIO]/[G] Agarrar | [5]/[B] Rutina Post-Grasping)",
        AUTO_PASO_A_DETECCION_RVIZ: "PASO A: FIJANDO POSICION 3D EN RVIZ",
        AUTO_PASO_B_MOVER_PREGRASP: "PASO B: YENDO A PRE-GRASP CON GRIPPER AL MAXIMO (105 MM)",
        AUTO_PASO_C1_ALINEAR_CON_OBJETO: "PASO C1: ALINEANDOSE CON EL OBJETO EN PRE-GRASP (105 MM)",
        AUTO_PASO_C2_AVANZAR_AL_OBJETO: "PASO C2: AVANZANDO RECTO HACIA EL OBJETO HASTA LLEGAR",
        AUTO_PASO_D_AGARRE: "PASO D: CERRANDO GRIPPER Y AGARRANDO OBJETO",
        AUTO_PASO_E_POSTGRASP_PREGRASP: "PASO E (RUTINA POST-GRASPING): VOLVIENDO RECTO ATRAS A PRE-GRASP",
        AUTO_SUCCESS: "POST-GRASPING COMPLETADO: OBJETO EN POSICION PRE-GRASP ([2] Ir a Home)",
    }

    def __init__(self, dry_run: bool = False):
        super().__init__("manipulacion_autonoma_node")
        self.dry_run = dry_run
        self.get_logger().info("Iniciando Sistema de Manipulacion 3D en RViz + Cinematica Inversa...")

        # 1. Parámetros Calibrables de Llegada al Objeto (cargados y guardados en YAML)
        self.wrist_camera_right_offset_m: float = float(WRIST_CAMERA_RIGHT_OFFSET_M)  # 0.096 m (9.6 cm derecha)
        self.grasp_forward_cm: float = 25.0        # Cuántos cm avanza desde Pre-Grasp para llegar al objeto
        self.grasp_height_offset_cm: float = 4.2   # Ajuste fino de altura (cm) al llegar al objeto
        self.pose_grasp: Optional[List[float]] = None
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
        self.teaching_target: Optional[str] = None  # 'home', 'pre_grasp' o 'grasp'

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
        self.last_wrist_det_time = 0.0
        self.last_marker_pub_time = 0.0
        self.last_out_of_reach_say_time = 0.0

        # Trayectorias articulares y avance calculados por IK
        self.planned_dx_m: float = self.grasp_forward_cm / 100.0
        self.planned_dy_m: float = -self.wrist_camera_right_offset_m
        self.joints_aligned: Optional[List[float]] = None
        self.joints_grasp: Optional[List[float]] = None
        self.joints_lift: Optional[List[float]] = None
        self.joints_retract_straight: Optional[List[float]] = None
        self._align_sent = False
        self._ik_sent = False
        self._lift_sent = False
        self._retract_line_sent = False
        self._retract_pregrasp_sent = False

        self.status_message = (
            f"Listo. Pre-Grasp Pinza Max={GRIPPER_MAX_OPEN_MM:.0f}mm | "
            f"Avance={self.grasp_forward_cm:.1f}cm ([+]/[-]) | OffsetDer={self.wrist_camera_right_offset_m*100:.1f}cm"
        )

    # --------------------------------------------------------------------------
    # CONFIGURACIÓN Y PERSISTENCIA (YAML)
    # --------------------------------------------------------------------------
    def load_configuration(self):
        """Carga o inicializa poses nombradas y parámetros de llegada desde YAML."""
        self.pose_home = [72.09, 176.5, -155.15, 102.62, -31.51, -99.91]
        self.pose_pre_grasp = [53.33, 176.38, -79.88, 69.54, -75.36, -89.05]
        self.pose_grasp = None
        self.gripper_home_mm = GRIPPER_MAX_OPEN_MM
        self.gripper_pre_grasp_mm = GRIPPER_MAX_OPEN_MM

        if os.path.exists(CONFIG_FILE_PATH):
            try:
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                    env = data.get("safety_envelope", {})
                    if "wrist_camera_right_offset_m" in env:
                        self.wrist_camera_right_offset_m = float(env["wrist_camera_right_offset_m"])
                    if "grasp_forward_cm" in env:
                        self.grasp_forward_cm = float(env["grasp_forward_cm"])
                    if "grasp_height_offset_cm" in env:
                        self.grasp_height_offset_cm = float(env["grasp_height_offset_cm"])
                    poses = data.get("named_poses", {})
                    if "home" in poses:
                        self.pose_home = list(poses["home"].get("joints_deg", self.pose_home))
                        self.gripper_home_mm = float(poses["home"].get("gripper_mm", GRIPPER_MAX_OPEN_MM))
                    if "pre_grasp" in poses:
                        self.pose_pre_grasp = list(poses["pre_grasp"].get("joints_deg", self.pose_pre_grasp))
                        # En Pre-Grasp siempre mantener la apertura máxima física (105 mm)
                        self.gripper_pre_grasp_mm = GRIPPER_MAX_OPEN_MM
                    if "grasp" in poses and "joints_deg" in poses["grasp"]:
                        self.pose_grasp = list(poses["grasp"]["joints_deg"])
                self.get_logger().info("Configuracion de poses y parametros cargada desde YAML.")
            except Exception as e:
                self.get_logger().warn(f"Error leyendo YAML ({e}), usando valores por defecto.")

    def save_calibration_params(self):
        """Guarda los parámetros calibrables de llegada al objeto en el archivo YAML."""
        try:
            data = {}
            if os.path.exists(CONFIG_FILE_PATH):
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}

            if "safety_envelope" not in data:
                data["safety_envelope"] = {}

            data["safety_envelope"]["wrist_camera_right_offset_m"] = round(float(self.wrist_camera_right_offset_m), 4)
            data["safety_envelope"]["grasp_forward_cm"] = round(float(self.grasp_forward_cm), 1)
            data["safety_envelope"]["grasp_height_offset_cm"] = round(float(self.grasp_height_offset_cm), 1)

            with open(CONFIG_FILE_PATH, "w", encoding="utf-8") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        except Exception as e:
            self.get_logger().error(f"Error guardando parametros en YAML: {e}")

    def save_named_pose(self, pose_name: str, joints_deg: List[float], gripper_mm: float):
        """Guarda la pose enseñada en el archivo YAML y si es 'grasp', calibra automáticamente el avance."""
        try:
            data = {}
            if os.path.exists(CONFIG_FILE_PATH):
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}

            if "named_poses" not in data:
                data["named_poses"] = {}

            # Si se guarda pre_grasp, forzar que el gripper en Pre-Grasp quede al máximo abierto (105 mm)
            saved_gripper_mm = GRIPPER_MAX_OPEN_MM if pose_name == "pre_grasp" else float(gripper_mm)

            data["named_poses"][pose_name] = {
                "joints_deg": [round(float(v), 2) for v in joints_deg],
                "gripper_mm": round(saved_gripper_mm, 1),
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            }

            # Si el usuario enseñó 'grasp' (cuando de verdad llega al objeto), calcular automáticamente
            # cuántos cm avanzó respecto a Pre-Grasp y actualizar grasp_forward_cm y grasp_height_offset_cm!
            if pose_name == "grasp" and hasattr(self, "kinematics_solver"):
                p_pre = self.kinematics_solver.forward_position_m([math.radians(d) for d in self.pose_pre_grasp])
                p_g = self.kinematics_solver.forward_position_m([math.radians(d) for d in joints_deg[:6]])
                dx_cm = round(float((p_g[0] - p_pre[0]) * 100.0), 1)
                dz_cm = round(float((p_g[2] - p_pre[2]) * 100.0), 1)
                if dx_cm >= 4.0:
                    self.grasp_forward_cm = float(np.clip(dx_cm, 5.0, 30.0))
                self.grasp_height_offset_cm = float(np.clip(dz_cm, -10.0, 10.0))
                if "safety_envelope" not in data:
                    data["safety_envelope"] = {}
                data["safety_envelope"]["grasp_forward_cm"] = self.grasp_forward_cm
                data["safety_envelope"]["grasp_height_offset_cm"] = self.grasp_height_offset_cm

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

        # 2. Cámara de Muñeca (Con persistencia de 2.0s y suavizado EMA para no perder alineación)
        if self.wrist_color_frame is not None:
            det_w = self.detect_object_2d(self.wrist_color_frame)
            if det_w is not None:
                wx1, wy1, wx2, wy2, _ = det_w
                self.target_2d_wrist = (wx1, wy1, wx2, wy2)
                self.last_wrist_det_time = now
                wcx = (wx1 + wx2) / 2.0
                wcy = (wy1 + wy2) / 2.0
                raw_u = (wcx - 320.0) / 320.0
                raw_v = (wcy - 240.0) / 240.0
                self.wrist_error_u = 0.55 * self.wrist_error_u + 0.45 * raw_u
                self.wrist_error_v = 0.55 * self.wrist_error_v + 0.45 * raw_v
            else:
                if now - self.last_wrist_det_time > 2.0:
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
            f"Avance Calibrado: +{self.grasp_forward_cm:.1f} cm [{status_tag}]"
        )
        txt.lifetime = Duration(sec=0, nanosec=500000000)
        ma.markers.append(txt)

        self.pub_rviz_markers.publish(ma)

    # --------------------------------------------------------------------------
    # COMANDOS DE ACCIONAMIENTO Y CALIBRACIÓN EN VIVO DE LLEGADA AL OBJETO
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
        clamped_deg[1] = min(clamped_deg[1], 194.5)  # Permitir extensión completa de Joint 2 según URDF (195°)

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
        # Si ya tiene un objeto agarrado (pinza < 65 mm) o se pide keep_gripper, mantener el cierre para no soltarlo
        holding_object = self.gripper_opening_mm < 65.0
        g_mm = self.gripper_opening_mm if (keep_gripper or holding_object) else GRIPPER_MAX_OPEN_MM
        self.send_arm_joint_goal(self.pose_home, duration_s=duration_s, gripper_mm=g_mm)
        self.status_message = f"Moviendo a Posicion HOME (Gripper={g_mm:.0f}mm)."

    def mover_a_pre_grasp(self, duration_s: float = 2.2, keep_gripper: bool = False):
        """
        Mueve el brazo a la posición Pre-Grasp.
        Por defecto (keep_gripper=False), deja el GRIPPER ABIERTO AL MÁXIMO (105 mm).
        Cuando vuelve desde Post-Grasping con el objeto agarrado (keep_gripper=True), mantiene la pinza cerrada.
        """
        if self.is_teaching_mode:
            self.cancel_teaching_mode()
        self.fase_actual = self.FASE_3_PREGRASP
        g_mm = GRIPPER_CLOSE_GRASP_MM if keep_gripper else GRIPPER_MAX_OPEN_MM
        self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=duration_s, gripper_mm=g_mm)
        if not keep_gripper:
            self.status_message = f"Modo PRE-GRASP activo: Gripper ABIERTO AL MAXIMO ({GRIPPER_MAX_OPEN_MM:.0f} mm)."
        else:
            self.status_message = f"Retorno Post-Grasping a PRE-GRASP manteniendo objeto ({g_mm:.0f} mm)."

    def compute_aligned_and_grasp_joints(self) -> Tuple[List[float], List[float], float]:
        """
        Calcula en 2 etapas claras desde Pre-Grasp:
          1) joints_aligned: Postura en Pre-Grasp ALINEADA lateralmente y en altura con el objeto
             (compensando el desfase a la derecha de la cámara del brazo) ANTES de avanzar.
          2) joints_grasp: Postura de LLEGADA AL OBJETO avanzando recto hacia adelante
             exactamente el parámetro calibrado `grasp_forward_cm`.
        """
        dz_m = float(self.grasp_height_offset_cm / 100.0)
        dx_m = float(self.grasp_forward_cm / 100.0)

        # Determinar alineación lateral (dy_align) respecto al Eje de la Pinza (compensando offset cámara)
        if self.target_2d_wrist is not None:
            dy_align = float(np.clip(-self.wrist_camera_right_offset_m - (self.wrist_error_u * 0.08), -0.095, 0.095))
        elif self.target_3d_arm is not None:
            q_pre_rad = [math.radians(d) for d in self.pose_pre_grasp]
            _, pre_y, _ = self.kinematics_solver.forward_position_m(q_pre_rad)
            dy_align = float(np.clip((self.target_3d_arm[1] - pre_y) - (0.5 * self.wrist_camera_right_offset_m), -0.095, 0.095))
        else:
            dy_align = float(-self.wrist_camera_right_offset_m)

        # Etapa 1: Alinear en Pre-Grasp (sin avanzar todavía: delta_forward_m = 0.0)
        j_aligned, ok_align = self.kinematics_solver.move_tool_line(
            self.pose_pre_grasp,
            delta_forward_m=0.0,
            delta_lateral_m=dy_align,
            delta_up_m=dz_m,
        )
        if not ok_align:
            j_aligned = list(self.pose_pre_grasp)

        # Etapa 2: Desde la postura YA ALINEADA, avanzar recto hacia el objeto (delta_forward_m = dx_m)
        j_grasp, ok_grasp = self.kinematics_solver.move_tool_line(
            j_aligned,
            delta_forward_m=dx_m,
            delta_lateral_m=0.0,
            delta_up_m=0.0,
        )
        if not ok_grasp and self.pose_grasp is not None:
            # Si el usuario enseñó una pose_grasp manual con [K], aplicarle el delta de alineación lateral
            delta_align = [j_aligned[i] - self.pose_pre_grasp[i] for i in range(6)]
            j_grasp = [round(self.pose_grasp[i] + delta_align[i], 2) for i in range(6)]

        return j_aligned, j_grasp, dy_align

    def mover_a_calibracion_grasp(self, duration_s: float = 1.2):
        """
        Mueve el brazo en vivo al punto de llegada al objeto (FASE 4) con el gripper al máximo (105 mm)
        usando los parámetros actuales (`grasp_forward_cm`, `wrist_camera_right_offset_m`, `grasp_height_offset_cm`).
        """
        if self.is_teaching_mode:
            self.cancel_teaching_mode()
        self.fase_actual = self.FASE_4_GRASP
        j_aligned, j_grasp, dy_align = self.compute_aligned_and_grasp_joints()
        self.joints_aligned = j_aligned
        self.joints_grasp = j_grasp
        self.send_arm_joint_goal(j_grasp, duration_s=duration_s, gripper_mm=GRIPPER_MAX_OPEN_MM)
        self.status_message = (
            f"CALIBRANDO LLEGADA (Gripper {GRIPPER_MAX_OPEN_MM:.0f}mm): Avance=+{self.grasp_forward_cm:.1f}cm ([+]/[-]) | "
            f"OffsetDer={self.wrist_camera_right_offset_m*100:.1f}cm ([9]/[0]) | Presione [5]/[B] para Agarrar y Volver Recto Atras"
        )

    def adjust_calibration_param(self, d_forward_cm: float = 0.0, d_right_cm: float = 0.0, d_height_cm: float = 0.0, window_name: Optional[str] = None):
        """Ajusta en vivo los parámetros de llegada al objeto, actualiza barras y guarda en YAML."""
        self.grasp_forward_cm = float(np.clip(self.grasp_forward_cm + d_forward_cm, 5.0, 30.0))
        self.wrist_camera_right_offset_m = float(np.clip(self.wrist_camera_right_offset_m + (d_right_cm / 100.0), -0.15, 0.15))
        self.grasp_height_offset_cm = float(np.clip(self.grasp_height_offset_cm + d_height_cm, -10.0, 10.0))
        self.save_calibration_params()

        if window_name is not None:
            try:
                cv2.setTrackbarPos("Avance_cm", window_name, int(round(self.grasp_forward_cm)))
                cv2.setTrackbarPos("Offset_Der_mm", window_name, int(round(150 + self.wrist_camera_right_offset_m * 1000.0)))
                cv2.setTrackbarPos("Altura_mm", window_name, int(round(100 + self.grasp_height_offset_cm * 10.0)))
            except Exception:
                pass

        # Si estamos en FASE 4 (Calibrando llegada al objeto), mover el brazo en tiempo real!
        if self.fase_actual == self.FASE_4_GRASP and self.auto_state == self.AUTO_IDLE:
            self.mover_a_calibracion_grasp(duration_s=0.65)
        else:
            self.status_message = (
                f"GUARDADO EN YAML -> Avance: +{self.grasp_forward_cm:.1f}cm | "
                f"Offset Cam Der: {self.wrist_camera_right_offset_m*100:.1f}cm | "
                f"Altura: {self.grasp_height_offset_cm:+.1f}cm (Presione [4] para probar en vivo)"
            )

    def set_arm_torque(self, enable: bool):
        msg = Bool()
        msg.data = bool(enable)
        self.pub_enable.publish(msg)

    def toggle_teaching_pose(self, target_pose: str):
        if not self.is_teaching_mode:
            self.is_teaching_mode = True
            self.teaching_target = target_pose
            self.set_arm_torque(False)
            self.status_message = f"MODO ENSENANZA [{target_pose.upper()}]: Mueva el brazo con la mano hasta el punto y presione [ENTER] para guardar."
            self.say(f"Teach mode for {target_pose} activated. Move the arm with your hand and press Enter to save.")
        else:
            save_target = self.teaching_target or target_pose
            self.set_arm_torque(True)
            self.is_teaching_mode = False
            self.save_named_pose(save_target, self.arm_joints_deg, self.gripper_opening_mm)
            self.teaching_target = None
            self.status_message = (
                f"Pose {save_target.upper()} guardada! Avance={self.grasp_forward_cm:.1f}cm, "
                f"Altura={self.grasp_height_offset_cm:+.1f}cm."
            )

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
    # MÁQUINA DE ESTADOS Y RUTINA POST-GRASPING (VOLVER RECTO ATRÁS A PRE-GRASP)
    # --------------------------------------------------------------------------
    def start_autonomous_grasp(self):
        """
        Inicia la secuencia completa:
          Detectar 3D en RViz -> Pre-Grasp (Gripper 105mm) -> Alinear -> Avanzar ->
          Cerrar Gripper (50mm) -> Rutina Post-Grasping (Volver recto atrás a Pre-Grasp).
        """
        self.auto_state = self.AUTO_PASO_A_DETECCION_RVIZ
        self.step_start_time = time.time()
        self._align_sent = False
        self._ik_sent = False
        self._lift_sent = False
        self._retract_line_sent = False
        self._retract_pregrasp_sent = False
        self.say("Starting 3D detection, maximum gripper opening in pre-grasp, alignment and post-grasping routine.")

    def ejecutar_rutina_post_grasping(self):
        """
        Rutina Post-Grasping ejecutable también manualmente con [5] o [B]:
          1. Cierra el gripper firmemente sobre el objeto (50 mm).
          2. Despega suavemente +2.5 cm en vertical para no arrastrar el objeto en la mesa.
          3. Retrocede RECTO HACIA ATRÁS (sobre la misma línea de avance) hasta la postura
             alineada de Pre-Grasp y luego a la posición Pre-Grasp manteniendo el objeto agarrado.
        """
        if self.is_teaching_mode:
            self.cancel_teaching_mode()

        current_grasp_joints = list(self.joints_grasp) if self.joints_grasp is not None else list(self.arm_joints_deg[:6])
        self.joints_grasp = current_grasp_joints

        # Postura con +2.5 cm de despeje vertical sobre la mesa antes de retroceder recto
        j_lift, ok_lift = self.kinematics_solver.move_tool_line(
            current_grasp_joints, delta_forward_m=0.0, delta_lateral_m=0.0, delta_up_m=0.025
        )
        self.joints_lift = j_lift if ok_lift else current_grasp_joints

        # Postura de retroceso RECTO HACIA ATRÁS (-grasp_forward_cm) conservando la alineación lateral
        if self.joints_aligned is not None:
            j_retract, ok_ret = self.kinematics_solver.move_tool_line(
                self.joints_aligned, delta_forward_m=0.0, delta_lateral_m=0.0, delta_up_m=0.020
            )
            self.joints_retract_straight = j_retract if ok_ret else list(self.joints_aligned)
        else:
            back_m = -float(self.grasp_forward_cm / 100.0)
            j_retract, ok_ret = self.kinematics_solver.move_tool_line(
                current_grasp_joints, delta_forward_m=back_m, delta_lateral_m=0.0, delta_up_m=0.020
            )
            self.joints_retract_straight = j_retract if ok_ret else list(self.pose_pre_grasp)

        self.auto_state = self.AUTO_PASO_D_AGARRE
        self.step_start_time = time.time()
        self._lift_sent = False
        self._retract_line_sent = False
        self._retract_pregrasp_sent = False

        self.get_logger().info(f"[Rutina Post-Grasping] Cerrando gripper a {GRIPPER_CLOSE_GRASP_MM} mm para agarrar y volver recto atras a Pre-Grasp...")
        if not self.dry_run:
            g_msg = Float32()
            g_msg.data = float(GRIPPER_CLOSE_GRASP_MM)
            self.pub_gripper_cmd.publish(g_msg)
        self.say("Grasping object and executing post-grasping straight retraction to pre-grasp.")

    def update_autonomous_state_machine(self, now: float):
        if self.auto_state == self.AUTO_IDLE:
            return

        # PASO A: DETECCIÓN 3D POR PROFUNDIDAD DE CÁMARA PRINCIPAL Y ENVÍO A PRE-GRASP (CON GRIPPER AL MÁXIMO 105 MM)
        if self.auto_state == self.AUTO_PASO_A_DETECCION_RVIZ:
            if self.target_3d_arm is None and self.target_2d_wrist is None:
                self.status_message = "Esperando deteccion de la botella en camara principal o camara del brazo..."
                if now - self.step_start_time > 8.0:
                    self.say("Could not detect bottle. Returning to idle.")
                    self.auto_state = self.AUTO_IDLE
                return

            if self.target_3d_arm is not None:
                x_sh, y_sh, z_sh = self.target_3d_arm
                self.get_logger().info(
                    f"[Paso A -> B] Objeto 3D en RViz: Hombro=({x_sh:.2f}, {y_sh:.2f}, {z_sh:.2f})m | "
                    f"Moviendo a Pre-Grasp con Gripper al maximo ({GRIPPER_MAX_OPEN_MM} mm)..."
                )
            self.say("Object detected. Moving to pre-grasp with maximum gripper opening.")
            self.auto_state = self.AUTO_PASO_B_MOVER_PREGRASP
            self.step_start_time = now
            self.fase_actual = self.FASE_3_PREGRASP
            self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=2.4, gripper_mm=GRIPPER_MAX_OPEN_MM)

        # PASO B: LLEGAR A LA POSICIÓN PRE-GRASP (GRIPPER ABIERTO AL MÁXIMO 105 MM)
        elif self.auto_state == self.AUTO_PASO_B_MOVER_PREGRASP:
            if now - self.step_start_time > 2.6:
                self.auto_state = self.AUTO_PASO_C1_ALINEAR_CON_OBJETO
                self.step_start_time = now
                self._align_sent = False
                self.say("Pre-grasp reached with gripper fully open. Aligning with the bottle first.")

        # PASO C1: PRIMERO ALINEARSE CON EL OBJETO EN PRE-GRASP (MANTENIENDO GRIPPER AL MÁXIMO 105 MM)
        elif self.auto_state == self.AUTO_PASO_C1_ALINEAR_CON_OBJETO:
            step_elapsed = now - self.step_start_time
            if step_elapsed > 0.45 and not self._align_sent:
                self._align_sent = True
                j_aligned, j_grasp, dy_align = self.compute_aligned_and_grasp_joints()
                self.joints_aligned = j_aligned
                self.joints_grasp = j_grasp

                # Despeje vertical de +2.5 cm en el punto de agarre antes de retroceder recto
                j_lift, ok_lift = self.kinematics_solver.move_tool_line(
                    self.joints_grasp, delta_forward_m=0.0, delta_lateral_m=0.0, delta_up_m=0.025
                )
                self.joints_lift = j_lift if ok_lift else self.joints_grasp

                # Punto de retroceso recto hacia atrás (misma alineación lateral que el agarre, +2 cm Z)
                j_retract, ok_ret = self.kinematics_solver.move_tool_line(
                    self.joints_aligned, delta_forward_m=0.0, delta_lateral_m=0.0, delta_up_m=0.020
                )
                self.joints_retract_straight = j_retract if ok_ret else list(self.joints_aligned)

                self.get_logger().info(
                    f"[Paso C1 - ALINEAR PRIMERO] dY_align={dy_align*100:+.1f}cm | Gripper={GRIPPER_MAX_OPEN_MM}mm | "
                    f"Joints Aligned={self.joints_aligned}"
                )
                self.status_message = (
                    f"PASO C1: Alineando con la botella (dY={dy_align*100:+.1f}cm, Gripper Max {GRIPPER_MAX_OPEN_MM:.0f}mm)..."
                )
                self.send_arm_joint_goal(self.joints_aligned, duration_s=1.4, gripper_mm=GRIPPER_MAX_OPEN_MM)

            # Una vez terminada la alineación lateral en Pre-Grasp, pasar a avanzar recto hacia el objeto
            if step_elapsed > 2.1:
                self.auto_state = self.AUTO_PASO_C2_AVANZAR_AL_OBJETO
                self.step_start_time = now
                self._ik_sent = False
                self.say(f"Aligned. Advancing {int(round(self.grasp_forward_cm))} centimeters to grasp the bottle.")

        # PASO C2: AVANZAR RECTO HACIA EL OBJETO HASTA LLEGAR POR COMPLETO (GRIPPER AL MÁXIMO 105 MM)
        elif self.auto_state == self.AUTO_PASO_C2_AVANZAR_AL_OBJETO:
            step_elapsed = now - self.step_start_time
            if not self._ik_sent:
                self._ik_sent = True
                if self.joints_grasp is not None:
                    self.get_logger().info(
                        f"[Paso C2 - AVANZAR AL OBJETO] Avance=+{self.grasp_forward_cm:.1f}cm | "
                        f"Joints Grasp={self.joints_grasp}"
                    )
                    self.status_message = f"PASO C2: Avanzando recto +{self.grasp_forward_cm:.1f} cm con Gripper al maximo ({GRIPPER_MAX_OPEN_MM:.0f}mm)..."
                    self.send_arm_joint_goal(self.joints_grasp, duration_s=2.1, gripper_mm=GRIPPER_MAX_OPEN_MM)

            if step_elapsed > 2.4:
                self.auto_state = self.AUTO_PASO_D_AGARRE
                self.step_start_time = now
                self._lift_sent = False
                self._retract_line_sent = False
                self._retract_pregrasp_sent = False
                self.get_logger().info(f"[Paso D] Cerrando pinza a {GRIPPER_CLOSE_GRASP_MM} mm...")
                if not self.dry_run:
                    g_msg = Float32()
                    g_msg.data = float(GRIPPER_CLOSE_GRASP_MM)
                    self.pub_gripper_cmd.publish(g_msg)

        # PASO D: CERRAR PINZA SOBRE EL OBJETO Y DESPEGAR +2.5 CM DE LA MESA
        elif self.auto_state == self.AUTO_PASO_D_AGARRE:
            step_elapsed = now - self.step_start_time
            if step_elapsed > 1.4 and not self._lift_sent:
                self._lift_sent = True
                if self.joints_lift is not None:
                    self.get_logger().info(f"[Paso D] Despegando objeto +2.5 cm de la mesa: {self.joints_lift}")
                    self.status_message = "PASO D: Objeto agarrado, despegando +2.5 cm antes de volver recto atras..."
                    self.send_arm_joint_goal(self.joints_lift, duration_s=0.9, gripper_mm=GRIPPER_CLOSE_GRASP_MM)

            if step_elapsed > 2.4:
                self.auto_state = self.AUTO_PASO_E_POSTGRASP_PREGRASP
                self.step_start_time = now
                self._retract_line_sent = False
                self._retract_pregrasp_sent = False
                self.say("Object grasped. Retracting straight back to pre-grasp position.")

        # PASO E (RUTINA POST-GRASPING): VOLVER RECTO HACIA ATRÁS HASTA LA POSICIÓN PRE-GRASP
        elif self.auto_state == self.AUTO_PASO_E_POSTGRASP_PREGRASP:
            step_elapsed = now - self.step_start_time

            # Etapa E1 (t=0.0s): Retroceso lineal RECTO HACIA ATRÁS por el mismo eje de avance hasta joints_retract_straight
            if not self._retract_line_sent:
                self._retract_line_sent = True
                target_retract = self.joints_retract_straight if self.joints_retract_straight is not None else self.pose_pre_grasp
                self.get_logger().info(
                    f"[Paso E1 - POST-GRASPING] Retrocediendo RECTO HACIA ATRAS (-{self.grasp_forward_cm:.1f}cm): {target_retract}"
                )
                self.status_message = (
                    f"PASO E (POST-GRASPING): Volviendo RECTO HACIA ATRAS (-{self.grasp_forward_cm:.1f} cm) hacia Pre-Grasp..."
                )
                self.send_arm_joint_goal(target_retract, duration_s=1.8, gripper_mm=GRIPPER_CLOSE_GRASP_MM)

            # Etapa E2 (t=1.9s): Asentar en la posición Pre-Grasp enseñada manteniendo la pinza cerrada con el objeto
            if step_elapsed > 1.9 and not self._retract_pregrasp_sent:
                self._retract_pregrasp_sent = True
                self.get_logger().info(
                    f"[Paso E2 - POST-GRASPING] Posicionando en Pre-Grasp con el objeto agarrado: {self.pose_pre_grasp}"
                )
                self.mover_a_pre_grasp(duration_s=1.3, keep_gripper=True)

            if step_elapsed > 3.3:
                self.auto_state = self.AUTO_SUCCESS
                self.step_start_time = now
                self.say("Post-grasping complete! Object is held in pre-grasp position.")

        elif self.auto_state == self.AUTO_SUCCESS:
            if now - self.step_start_time > 4.0:
                self.auto_state = self.AUTO_IDLE

    # --------------------------------------------------------------------------
    # RENDERIZADO VISUAL DEL HUD (1280 x 640)
    # --------------------------------------------------------------------------
    def draw_hud(self) -> np.ndarray:
        canvas_w, canvas_h = 1280, 640
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        feed_w, feed_h = 620, 420

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

            canvas[40:40 + feed_h, 15:15 + feed_w] = h_img
        else:
            cv2.rectangle(canvas, (15, 40), (15 + feed_w, 40 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM CAMARA PRINCIPAL (REALSENSE D435)...", (55, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(canvas, "1. CAMARA PRINCIPAL + PROFUNDIDAD 3D -> PUBLICANDO OBJETO A RVIZ (/arm_camera/object_markers)", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 255, 200), 1)

        # --- PANEL DERECHO: CÁMARA DEL BRAZO (CON COMPENSACIÓN CALIBRABLE DE +6 CM A LA DERECHA) ---
        if self.wrist_color_frame is not None:
            w_img = cv2.resize(self.wrist_color_frame, (feed_w, feed_h))
            orig_h, orig_w = self.wrist_color_frame.shape[:2]
            sx, sy = feed_w / orig_w, feed_h / orig_h

            # Centro óptico de la cámara (amarillo) y Eje Real de la Pinza (verde, desplazado según offset_der)
            wcx, wcy = int(feed_w / 2), int(feed_h / 2)
            gripper_cx = int(np.clip(wcx - (self.wrist_camera_right_offset_m / 0.08) * (feed_w / 2), 20, feed_w - 20))
            cv2.drawMarker(w_img, (wcx, wcy), (0, 220, 255), cv2.MARKER_CROSS, 22, 1)
            cv2.putText(w_img, "Cam", (wcx + 6, wcy - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1)
            cv2.line(w_img, (gripper_cx, 25), (gripper_cx, feed_h - 25), (0, 255, 100), 2)
            cv2.drawMarker(w_img, (gripper_cx, wcy), (0, 255, 100), cv2.MARKER_TILTED_CROSS, 22, 2)
            cv2.putText(
                w_img,
                f"EJE PINZA ({-self.wrist_camera_right_offset_m*100:+.1f}cm)",
                (max(10, gripper_cx - 60), 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                (0, 255, 100),
                1,
            )

            if self.target_2d_wrist is not None:
                x1, y1, x2, y2 = self.target_2d_wrist
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                cv2.rectangle(w_img, (bx1, by1), (bx2, by2), (0, 200, 255), 2)
                dy_cm = (-self.wrist_camera_right_offset_m - (self.wrist_error_u * 0.08)) * 100.0
                cv2.putText(w_img, f"Alinear Pinza dY={dy_cm:+.1f}cm", (bx1, max(20, by1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 150), 2)

            canvas[40:40 + feed_h, 645:645 + feed_w] = w_img
        else:
            cv2.rectangle(canvas, (645, 40), (645 + feed_w, 40 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM CAMARA DEL BRAZO...", (740, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(
            canvas,
            f"2. CAMARA DEL BRAZO (Offset Der={self.wrist_camera_right_offset_m*100:.1f}cm): ALINEACION PREVIA AL AVANCE",
            (645, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (230, 240, 255),
            1,
        )

        # --- PANEL INFERIOR: PARÁMETROS CALIBRABLES Y TELEMETRÍA ---
        bar_y = 468
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (12, 16, 24), -1)
        cv2.line(canvas, (0, bar_y), (canvas_w, bar_y), (60, 80, 110), 1)

        state_Key = self.auto_state if self.auto_state != self.AUTO_IDLE else self.fase_actual
        state_str = self.FASE_NAMES.get(state_Key, "LISTO")
        cv2.putText(canvas, f"ESTADO: {state_str}", (20, bar_y + 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)

        # Línea destacada de Parámetros Calibrables de Llegada al Objeto
        calib_line = (
            f"PARAMETROS DE LLEGADA AL OBJETO (Guardado auto en YAML):  "
            f"1) Avance Frontal = +{self.grasp_forward_cm:.1f} cm (Teclas [+]/[-])  |  "
            f"2) Offset Cam Der = {self.wrist_camera_right_offset_m*100:.1f} cm (Teclas [9]/[0])  |  "
            f"3) Altura = {self.grasp_height_offset_cm:+.1f} cm (Teclas [7]/[8])"
        )
        cv2.putText(canvas, calib_line, (20, bar_y + 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 255, 120), 1)

        obj_str = (
            f"[{self.target_3d_arm[0]:.2f}, {self.target_3d_arm[1]:.2f}, {self.target_3d_arm[2]:.2f}]m (Dist: {self.dist_tcp_to_obj_cm:.1f}cm)"
            if self.target_3d_arm is not None else "BUSCANDO..."
        )
        joints_str = ", ".join([f"{v:+.1f}" for v in self.arm_joints_deg[:6]])
        telemetry_line = f"RViz 3D: {obj_str} | Pinza: {self.gripper_opening_mm:.1f}mm | Joints: [{joints_str}] | {self.status_message}"
        cv2.putText(canvas, telemetry_line, (20, bar_y + 80), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 220, 240), 1)

        if self.is_teaching_mode:
            cv2.rectangle(canvas, (15, 355), (canvas_w - 15, 455), (0, 180, 255), -1)
            cv2.rectangle(canvas, (15, 355), (canvas_w - 15, 455), (255, 255, 255), 2)
            t_name = (self.teaching_target or "pose").upper()
            cv2.putText(canvas, f">>> MODO ENSENANZA [{t_name}] ACTIVO (BRAZO LIBERADO) <<<", (180, 397), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 0, 0), 2)
            cv2.putText(canvas, "Lleve el brazo con la mano al punto y presione [ENTER] para GUARDAR | [C] Cancelar", (140, 433), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (15, 15, 15), 2)

        shortcuts1 = "[2]/[h] Home | [3]/[p] Pre-Grasp (Gripper Max 105mm) | [4] Probar Llegada | [5]/[B] Rutina Post-Grasping (Cerrar y Volver Recto Atras)"
        shortcuts2 = "[ESPACIO]/[G] Secuencia Completa (Pre-Grasp 105mm -> Alinear -> Avanzar -> Agarrar -> Volver Recto Atras a Pre-Grasp) | [R] Abrir Max | [C] Cerrar"
        cv2.putText(canvas, shortcuts1, (20, bar_y + 112), cv2.FONT_HERSHEY_SIMPLEX, 0.39, (255, 215, 60), 1)
        cv2.putText(canvas, shortcuts2, (20, bar_y + 138), cv2.FONT_HERSHEY_SIMPLEX, 0.39, (0, 230, 255), 1)

        return canvas

    # --------------------------------------------------------------------------
    # BUCLE PRINCIPAL INTERACTIVO (CON SLIDERS Y TECLAS DE CALIBRACIÓN EN VIVO)
    # --------------------------------------------------------------------------
    def run(self):
        window_name = "Robot @Home - Manipulacion 3D en RViz + Cinematica Inversa"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)

        # Crear barras deslizantes (Sliders) en la ventana para calibrar la llegada al objeto
        def on_avance_slider(val: int):
            new_cm = float(max(5, val))
            if abs(new_cm - self.grasp_forward_cm) >= 0.5:
                self.grasp_forward_cm = new_cm
                self.save_calibration_params()
                if self.fase_actual == self.FASE_4_GRASP and self.auto_state == self.AUTO_IDLE:
                    self.mover_a_calibracion_grasp(duration_s=0.6)

        def on_offset_der_slider(val: int):
            new_m = float(val - 150) / 1000.0
            if abs(new_m - self.wrist_camera_right_offset_m) >= 0.004:
                self.wrist_camera_right_offset_m = new_m
                self.save_calibration_params()
                if self.fase_actual == self.FASE_4_GRASP and self.auto_state == self.AUTO_IDLE:
                    self.mover_a_calibracion_grasp(duration_s=0.6)

        def on_altura_slider(val: int):
            new_h_cm = float(val - 100) / 10.0
            if abs(new_h_cm - self.grasp_height_offset_cm) >= 0.4:
                self.grasp_height_offset_cm = new_h_cm
                self.save_calibration_params()
                if self.fase_actual == self.FASE_4_GRASP and self.auto_state == self.AUTO_IDLE:
                    self.mover_a_calibracion_grasp(duration_s=0.6)

        cv2.createTrackbar("Avance_cm", window_name, int(round(self.grasp_forward_cm)), 30, on_avance_slider)
        cv2.createTrackbar("Offset_Der_mm", window_name, int(round(150 + self.wrist_camera_right_offset_m * 1000.0)), 300, on_offset_der_slider)
        cv2.createTrackbar("Altura_mm", window_name, int(round(100 + self.grasp_height_offset_cm * 10.0)), 200, on_altura_slider)

        print("==========================================================================")
        print("   ROBOT @HOME - PRE-GRASP (105 MM MAX) + RUTINA POST-GRASPING            ")
        print("--------------------------------------------------------------------------")
        print("   1. MODO PRE-GRASP ([3] / [p]):                                         ")
        print("      - Deja el brazo en Pre-Grasp con el GRIPPER ABIERTO AL MAXIMO (105mm)")
        print("   2. CALIBRAR LLEGADA AL OBJETO ([4]):                                   ")
        print("      - [+] / [-] : Ajustar AVANCE FRONTAL (cm)                           ")
        print("      - [9] / [0] : Ajustar OFFSET DERECHA camara (cm)                    ")
        print("      - [7] / [8] : Ajustar ALTURA (cm)                                   ")
        print("   3. RUTINA POST-GRASPING ([5] o [B]):                                   ")
        print("      - Cierra el gripper sobre el objeto (50 mm), despega +2.5 cm y      ")
        print("        VUELVE RECTO HACIA ATRAS hasta la posicion de PRE-GRASP.          ")
        print("--------------------------------------------------------------------------")
        print("   [ESPACIO] o [G] : EJECUTAR SECUENCIA COMPLETA:                         ")
        print("                     1) Fija posicion 3D del objeto en RViz               ")
        print("                     2) Va a PRE-GRASP con GRIPPER AL MAXIMO (105 mm)     ")
        print("                     3) PRIMERO se ALINEA con la botella en Pre-Grasp     ")
        print("                     4) LUEGO AVANZA recto (+Avance_cm) hasta el objeto   ")
        print("                     5) Cierra gripper y VUELVE RECTO ATRAS a PRE-GRASP   ")
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
                    self.mover_a_pre_grasp(keep_gripper=False)

            elif key in [ord("e"), ord("E"), ord("P")]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "pre_grasp")
                else:
                    self.toggle_teaching_pose("pre_grasp")

            elif key == 13:  # ENTER guarda cualquier pose que se esté enseñando (home, pre_grasp o grasp)
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "pre_grasp")

            elif key in [ord("4"), ord("v"), ord("V")]:
                self.auto_state = self.AUTO_IDLE
                self.mover_a_calibracion_grasp(duration_s=1.8)

            elif key in [ord("5"), ord("b"), ord("B")]:
                self.ejecutar_rutina_post_grasping()

            elif key in [ord("k"), ord("K")]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "grasp")
                else:
                    self.toggle_teaching_pose("grasp")

            # Teclas de calibración en tiempo real del punto de llegada al objeto
            elif key in [ord("+"), ord("=")]:
                self.adjust_calibration_param(d_forward_cm=+1.0, window_name=window_name)
            elif key in [ord("-"), ord("_")]:
                self.adjust_calibration_param(d_forward_cm=-1.0, window_name=window_name)
            elif key == ord("0"):
                self.adjust_calibration_param(d_right_cm=+0.5, window_name=window_name)
            elif key == ord("9"):
                self.adjust_calibration_param(d_right_cm=-0.5, window_name=window_name)
            elif key == ord("8"):
                self.adjust_calibration_param(d_height_cm=+0.5, window_name=window_name)
            elif key == ord("7"):
                self.adjust_calibration_param(d_height_cm=-0.5, window_name=window_name)

            elif key in [ord("g"), ord("G"), 32]:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "pre_grasp")
                else:
                    self.start_autonomous_grasp()

            elif key in [ord("r"), ord("R"), ord("o"), ord("O")]:
                msg = Float32()
                msg.data = float(GRIPPER_MAX_OPEN_MM)
                self.pub_gripper_cmd.publish(msg)
                self.status_message = f"Pinza abierta al MAXIMO ({GRIPPER_MAX_OPEN_MM:.0f} mm)."
            elif key in [ord("c"), ord("C")]:
                if self.is_teaching_mode:
                    self.cancel_teaching_mode()
                else:
                    msg = Float32()
                    msg.data = float(GRIPPER_CLOSE_GRASP_MM)
                    self.pub_gripper_cmd.publish(msg)
                    self.status_message = f"Pinza cerrada a {GRIPPER_CLOSE_GRASP_MM:.0f} mm."

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
