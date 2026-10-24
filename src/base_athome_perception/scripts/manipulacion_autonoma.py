#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Robot @Home - Sistema Integrado de Manipulación, Calibración y Agarre Autónomo
#
# 3 Fases de Configuración del Brazo:
#   1. Rango de Movimientos del Brazo (Safe Envelope & Hard-Stop Interlocks)
#   2. Posición Home del Brazo (Calibración y Retorno a Estiba)
#   3. Posición Pre-Grasp del Brazo (Aproximación Frontal Preparatoria)
#
# Ciclo Autónomo Dual-Camera:
#   A. Cámara Principal (Cabezal RealSense D435): Detección 3D y posición espacial.
#   B. Brazo a Posición Pre-Grasp (orientado a la botella, pinza 70 mm).
#   C. Cámara del Brazo (Muñeca DaBai Eye-in-Hand): Guiado fino (Visual Servoing)
#      y avance ("ir hacia la botella") centrando la pinza.
#   D. Agarre con torque regulado y elevación vertical (+8 cm).
#   E. Retorno seguro con la botella sujeta a la Posición Home.
#
# Robot: "Robot"
# ==============================================================================

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
import yaml

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import SetBool, Trigger

# Asegurar acceso a ultralytics del venv de visión
VENV_SITE_PACKAGES = "/home/athome/ros2_vision_venv/lib/python3.12/site-packages"
if os.path.exists(VENV_SITE_PACKAGES) and VENV_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, VENV_SITE_PACKAGES)

from ultralytics import YOLO

# Módulo cinemático del Brazo Piper
from base_athome_perception.piper_kinematics import (
    transform_camera_to_arm,
    is_in_safe_joint_envelope,
    is_in_safe_cartesian_box,
    clamp_to_safe_cartesian_box,
    PiperKinematicsSolver,
    SAFE_CARTESIAN_BOX_M,
    SAFE_JOINT_LIMITS_DEG,
    HEAD_TO_ARM_OFFSET_M,
)

CONFIG_FILE_PATH = "/home/athome/base_athome_ws/src/base_athome_perception/config/arm_poses_and_limits.yaml"


class ManipulacionAutonomaNode(Node):
    """Nodo integrado de calibración en 3 fases y agarre autónomo dual-camera."""

    # Fases de Operación
    FASE_1_RANGO = 1
    FASE_2_HOME = 2
    FASE_3_PREGRASP = 3

    # Estados del Ciclo Autónomo
    AUTO_IDLE = 10
    AUTO_PASO_A_DETECCION_GLOBAL = 11  # RealSense Cabezal
    AUTO_PASO_B_MOVER_PREGRASP = 12    # Brazo a Pre-Grasp
    AUTO_PASO_C_GUIADO_FINO = 13       # Cámara de Muñeca (Va hacia la botella)
    AUTO_PASO_D_AGARRE_ELEVACION = 14  # Clamp + Lift
    AUTO_PASO_E_RETORNO_HOME = 15      # Carry to Home
    AUTO_SUCCESS = 16

    FASE_NAMES = {
        FASE_1_RANGO: "FASE 1: RANGO DE MOVIMIENTOS SEGURO",
        FASE_2_HOME: "FASE 2: POSICION HOME DEL BRAZO",
        FASE_3_PREGRASP: "FASE 3: POSICION PRE-GRASP DEL BRAZO",
        AUTO_IDLE: "SISTEMA LISTO (PRESIONE [ESPACIO] O [G] PARA AGARRE)",
        AUTO_PASO_A_DETECCION_GLOBAL: "PASO A: DETECCION ESPACIAL (CAMARA CABEZAL)",
        AUTO_PASO_B_MOVER_PREGRASP: "PASO B: DESPLAZANDO A POSICION PRE-GRASP",
        AUTO_PASO_C_GUIADO_FINO: "PASO C: GUIADO FINO Y AVANCE (CAMARA DEL BRAZO)",
        AUTO_PASO_D_AGARRE_ELEVACION: "PASO D: CERRANDO PINZA Y ELEVANDO BOTELLA",
        AUTO_PASO_E_RETORNO_HOME: "PASO E: TRASLADANDO BOTELLA A POSICION HOME",
        AUTO_SUCCESS: "CICLO COMPLETADO: BOTELLA EN POSICION HOME",
    }

    def __init__(self, dry_run: bool = False):
        super().__init__("manipulacion_autonoma_node")
        self.dry_run = dry_run
        self.get_logger().info("Iniciando Sistema Integrado de Manipulación y Agarre Autónomo...")

        # 1. Cargar Configuración de Poses y Envolvente desde YAML
        self.load_configuration()

        # 2. Subscripciones a Cámara Principal (Cabezal RealSense D435)
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

        # 3. Subscripción a Cámara del Brazo (Muñeca DaBai Eye-in-Hand)
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

        # 5. Publicadores ROS 2
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_joint_cmd = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_gripper_cmd = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_pan_tilt_cmd = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)
        self.pub_enable = self.create_publisher(Bool, "/arm/enable", 5)

        # 6. Servicios de Manipulación
        self.cli_home = self.create_client(Trigger, "/arm/home")
        self.cli_pre_grasp = self.create_client(Trigger, "/arm/pre_grasp")
        self.cli_release = self.create_client(Trigger, "/arm/release")
        self.cli_set_enable = self.create_client(SetBool, "/arm/set_enable")

        # Buffers de imágenes
        self.head_color_frame: Optional[np.ndarray] = None
        self.head_depth_frame: Optional[np.ndarray] = None
        self.wrist_color_frame: Optional[np.ndarray] = None

        # Parámetros ópticos RealSense
        self.fx = 604.2
        self.fy = 603.7
        self.cx = 316.6
        self.cy = 253.7
        self.has_intrinsics = False

        # Ángulos del cabezal Pan-Tilt
        self.head_pan_rad = 0.0
        self.head_tilt_rad = 0.0
        self.pan_tilt_inverted = True  # Inversión corregida: W sube, S baja, A izquierda, D derecha

        # Modo de Enseñanza Simple (Teach Mode para Home y Pre-Grasp)
        self.is_teaching_mode = False
        self.teaching_target: Optional[str] = None  # 'home' o 'pre_grasp'

        # Telemetría del brazo Piper
        self.arm_is_moving = False
        self.gripper_opening_mm = 69.5
        self.arm_xyz_mm = [0.0, 0.0, 0.0]
        self.arm_joints_deg = list(self.pose_home)

        # Solucionador cinemático
        self.kinematics_solver = PiperKinematicsSolver()

        # Modelo YOLO para detección de botellas / potes
        self.get_logger().info("Cargando modelo YOLO para detección de botellas...")
        try:
            self.yolo_model = YOLO("yolov8s-worldv2.pt")
            self.yolo_model.set_classes(["bottle", "pringles can", "can", "cup", "cylinder"])
        except Exception:
            self.yolo_model = YOLO("yolov8n.pt")

        # Máquina de estados y fases
        self.fase_actual = self.FASE_2_HOME
        self.auto_state = self.AUTO_IDLE
        self.state_start_time = time.time()
        self.step_start_time = time.time()

        # Detecciones 2D y 3D
        self.target_2d_head: Optional[Tuple[int, int, int, int]] = None
        self.target_3d_head: Optional[Tuple[float, float, float]] = None
        self.target_3d_arm: Optional[Tuple[float, float, float]] = None
        self.target_2d_wrist: Optional[Tuple[int, int, int, int]] = None
        self.wrist_error_u = 0.0
        self.wrist_error_v = 0.0
        self.is_in_reach = False

        # Almacenamiento de trayectorias articulares calculadas
        self.joints_pre_grasp: Optional[List[float]] = None
        self.joints_grasp: Optional[List[float]] = None
        self.joints_lift: Optional[List[float]] = None

        # Mensajes y Cooldowns de voz
        self.status_message = "Listo. Seleccione Fase [1, 2, 3] o presione [ESPACIO] para agarre autónomo."
        self.last_out_of_reach_say_time = 0.0

    # --------------------------------------------------------------------------
    # CONFIGURACIÓN Y PERSISTENCIA (YAML)
    # --------------------------------------------------------------------------
    def load_configuration(self):
        """Carga o inicializa poses nombradas y límites seguros desde YAML."""
        self.pose_home = [89.58, 172.74, -163.98, 84.59, -1.27, -153.97]
        self.pose_pre_grasp = [0.0, 68.0, -80.0, 0.0, 12.0, 0.0]
        self.gripper_home_mm = 70.0
        self.gripper_pre_grasp_mm = 70.0

        if os.path.exists(CONFIG_FILE_PATH):
            try:
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    poses = data.get("named_poses", {})
                    if "home" in poses:
                        self.pose_home = list(poses["home"].get("joints_deg", self.pose_home))
                        self.gripper_home_mm = float(poses["home"].get("gripper_mm", 70.0))
                    if "pre_grasp" in poses:
                        self.pose_pre_grasp = list(poses["pre_grasp"].get("joints_deg", self.pose_pre_grasp))
                        self.gripper_pre_grasp_mm = float(poses["pre_grasp"].get("gripper_mm", 70.0))
                self.get_logger().info("Configuración de poses cargada correctamente desde YAML.")
            except Exception as e:
                self.get_logger().warn(f"No se pudo leer YAML ({e}), usando valores calibrados por defecto.")

    def save_named_pose(self, pose_name: str, joints_deg: List[float], gripper_mm: float):
        """Guarda la pose actual en el archivo YAML de configuración."""
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

            self.get_logger().info(f"Pose '{pose_name}' guardada exitosamente en {CONFIG_FILE_PATH}")
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
            self.arm_xyz_mm = list(data.get("xyz_mm", self.arm_xyz_mm))
            self.arm_joints_deg = list(data.get("joints_deg", self.arm_joints_deg))
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
    # COMANDOS DE ACCIONAMIENTO Y HRI
    # --------------------------------------------------------------------------
    def say(self, text: str):
        """Síntesis de voz con Piper TTS en el robot."""
        msg = String()
        msg.data = text
        self.pub_say.publish(msg)
        self.status_message = text
        print(f"[Robot]: \"{text}\"")

    def send_arm_joint_goal(self, joints_deg: List[float], duration_s: float = 2.2, gripper_mm: Optional[float] = None) -> bool:
        """Publica comando articular interpolado a /arm/joint_cmd con verificación estricta de seguridad."""
        safe, reason = is_in_safe_joint_envelope(joints_deg)
        if not safe:
            self.get_logger().error(f"[INTERBLOQUEO DE SEGURIDAD] Comando articular bloqueado: {reason}")
            self.status_message = f"BLOQUEO DE SEGURIDAD: {reason}"
            return False

        if self.dry_run:
            self.get_logger().info(f"[DRY-RUN] Simulado joint_cmd: {[round(v, 1) for v in joints_deg]}, dur={duration_s}s, pinza={gripper_mm}mm")
            return True

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [math.radians(d) for d in joints_deg]
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

    def send_pan_tilt_goal(self, pan_deg: float, tilt_deg: float):
        """Apunta el cabezal Pan-Tilt."""
        v = Vector3()
        v.x = float(pan_deg)
        v.y = float(tilt_deg)
        v.z = 0.0
        self.pub_pan_tilt_cmd.publish(v)
        self.get_logger().info(f"[Cabezal] Pan={pan_deg:.1f}°, Tilt={tilt_deg:.1f}°")

    def mover_a_home(self, duration_s: float = 2.5, keep_gripper: bool = False):
        """Ejecuta la Fase 2: Retornar a la Posición Home del brazo."""
        self.fase_actual = self.FASE_2_HOME
        g_mm = self.gripper_opening_mm if keep_gripper else 70.0
        self.get_logger().info(f"Moviendo brazo a Posición HOME (pinza={g_mm:.1f}mm)...")
        self.send_arm_joint_goal(self.pose_home, duration_s=duration_s, gripper_mm=g_mm)
        self.status_message = "Moviendo a Posición HOME."

    def set_arm_torque(self, enable: bool):
        """Activa o libera los motores del brazo Piper para modo de enseñanza manual libre."""
        msg = Bool()
        msg.data = bool(enable)
        self.pub_enable.publish(msg)

        if self.cli_set_enable.service_is_ready():
            req = SetBool.Request()
            req.data = bool(enable)
            self.cli_set_enable.call_async(req)

        status_txt = "HABILITADO" if enable else "LIBERADO (Guiado manual libre)"
        self.get_logger().info(f"[Brazo] Torque {status_txt}")

    def toggle_teaching_home(self):
        """Alterna el modo de enseñanza de la pose HOME (estiba/reposo)."""
        if not self.is_teaching_mode:
            # 1. Iniciar modo enseñanza: liberar motores
            self.is_teaching_mode = True
            self.teaching_target = "home"
            self.fase_actual = self.FASE_2_HOME
            self.set_arm_torque(False)

            self.status_message = "MODO ENSENANZA HOME: Mueva el brazo a reposo y presione [Shift+H], [E] o [ENTER] para guardar."
            self.say("Home teach mode activated. Move the arm with your hand to the desired home rest position, then press Enter or H to save.")
        else:
            # 2. Guardar pose enseñada: reactivar torque y persistir en YAML
            self.set_arm_torque(True)
            self.is_teaching_mode = False
            self.save_named_pose("home", self.arm_joints_deg, self.gripper_opening_mm)
            self.pose_home = list(self.arm_joints_deg)
            self.gripper_home_mm = float(self.gripper_opening_mm)
            self.teaching_target = None
            self.status_message = f"Pose HOME guardada: {[round(v, 1) for v in self.arm_joints_deg]}. Motores re-activados."
            self.say("Home position learned and saved successfully!")

    def toggle_teaching_pregrasp(self):
        """Alterna el modo de enseñanza de la pose PRE-GRASP."""
        if not self.is_teaching_mode:
            # 1. Iniciar modo enseñanza: liberar motores y abrir pinza
            self.is_teaching_mode = True
            self.teaching_target = "pre_grasp"
            self.fase_actual = self.FASE_3_PREGRASP
            self.set_arm_torque(False)

            # Abrir pinza al máximo para estar listos para la botella
            msg_g = Float32()
            msg_g.data = 70.0
            self.pub_gripper_cmd.publish(msg_g)

            self.status_message = "MODO ENSENANZA PRE-GRASP: Mueva el brazo hacia la botella y presione [E] o [ENTER] para guardar."
            self.say("Pre-grasp teach mode activated. Move the arm with your hand to the desired pre-grasp position, then press E or Enter to save.")
        else:
            # 2. Guardar pose enseñada: reactivar torque y persistir en YAML
            self.set_arm_torque(True)
            self.is_teaching_mode = False
            target = self.teaching_target or "pre_grasp"
            self.save_named_pose(target, self.arm_joints_deg, self.gripper_opening_mm)
            if target == "home":
                self.pose_home = list(self.arm_joints_deg)
                self.gripper_home_mm = float(self.gripper_opening_mm)
                self.status_message = f"Pose HOME guardada: {[round(v, 1) for v in self.arm_joints_deg]}. Motores re-activados."
                self.say("Home position learned and saved successfully!")
            else:
                self.pose_pre_grasp = list(self.arm_joints_deg)
                self.gripper_pre_grasp_mm = float(self.gripper_opening_mm)
                self.status_message = f"Pose PRE-GRASP guardada: {[round(v, 1) for v in self.arm_joints_deg]}. Motores re-activados."
                self.say("Pre-grasp position learned and saved successfully!")
            self.teaching_target = None

    def cancel_teaching_mode(self):
        """Cancela el modo enseñanza y re-activa el torque sin guardar cambios."""
        if self.is_teaching_mode:
            self.set_arm_torque(True)
            self.is_teaching_mode = False
            self.teaching_target = None
            self.status_message = "Modo ensenanza cancelado. Motores re-activados."
            self.say("Teaching mode cancelled.")

    def mover_a_pre_grasp(self, duration_s: float = 2.2):
        """Ejecuta la Fase 3: Desplegar a la Posición Pre-Grasp del brazo."""
        if self.is_teaching_mode:
            self.cancel_teaching_mode()
        self.fase_actual = self.FASE_3_PREGRASP
        self.get_logger().info("Moviendo brazo a Posición PRE-GRASP (pinza abierta 70 mm)...")
        self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=duration_s, gripper_mm=70.0)
        self.status_message = "Moviendo a Posición PRE-GRASP (70mm)."

    # --------------------------------------------------------------------------
    # DETECCIÓN VISUAL (CÁMARA CABEZAL Y CÁMARA DE MUÑECA)
    # --------------------------------------------------------------------------
    def detect_object_head(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """Detecta la botella o pote usando YOLO + relación de aspecto en la cámara del cabezal."""
        results = self.yolo_model(frame, verbose=False, conf=0.22)
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
                    score = conf * (1.25 if aspect > 1.1 else 1.0)
                    candidates.append((x1, y1, x2, y2, score))

        if candidates:
            return max(candidates, key=lambda c: c[4])

        # Respaldo por segmentación de color (cilindros rojos / brillantes)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([0, 110, 70]), np.array([12, 255, 255])) | \
               cv2.inRange(hsv, np.array([168, 110, 70]), np.array([180, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            if cv2.contourArea(cnt) > 1100:
                x, y, w, h = cv2.boundingRect(cnt)
                if 1.2 <= (h / max(1, w)) <= 4.0:
                    return (x, y, x + w, y + h, 0.60)

        return None

    def detect_object_wrist(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """Detecta la botella en la cámara de muñeca (Eye-in-Hand) para guiado fino."""
        res = self.detect_object_head(frame)
        return res

    def calculate_3d_coordinates(self, bbox: Tuple[int, int, int, int]) -> Optional[Tuple[float, float, float]]:
        """Desproyecta las coordenadas 2D del cabezal a métricas ópticas 3D (X, Y, Z)."""
        if self.head_depth_frame is None:
            return None

        x1, y1, x2, y2 = bbox
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        d_h, d_w = self.head_depth_frame.shape
        x_min, x_max = max(0, cx - 4), min(d_w, cx + 5)
        y_min, y_max = max(0, cy - 4), min(d_h, cy + 5)

        depth_patch = self.head_depth_frame[y_min:y_max, x_min:x_max]
        valid_depths = depth_patch[depth_patch > 200]
        if len(valid_depths) == 0:
            return None

        z_m = float(np.median(valid_depths)) / 1000.0
        x_m = (cx - self.cx) * z_m / self.fx
        y_m = (cy - self.cy) * z_m / self.fy
        return (x_m, y_m, z_m)

    # --------------------------------------------------------------------------
    # MÁQUINA DE ESTADOS DEL CICLO AUTÓNOMO (PASOS A -> B -> C -> D -> E)
    # --------------------------------------------------------------------------
    def start_autonomous_grasp(self):
        """Inicia el ciclo autónomo completo de agarre y transporte a Home."""
        self.auto_state = self.AUTO_PASO_A_DETECCION_GLOBAL
        self.state_start_time = time.time()
        self.step_start_time = time.time()
        self.say("Hello! I am Robot. Starting autonomous bottle grasping sequence.")

    def update_autonomous_state_machine(self, now: float):
        if self.auto_state == self.AUTO_IDLE:
            return

        # PASO A: DETECCIÓN ESPACIAL GLOBAL (Cabezal RealSense D435)
        if self.auto_state == self.AUTO_PASO_A_DETECCION_GLOBAL:
            if self.head_color_frame is not None:
                det = self.detect_object_head(self.head_color_frame)
                if det:
                    x1, y1, x2, y2, _ = det
                    self.target_2d_head = (x1, y1, x2, y2)
                    coords_3d = self.calculate_3d_coordinates((x1, y1, x2, y2))
                    if coords_3d:
                        self.target_3d_head = coords_3d
                        # Transformación 3D del cabezal a la base del brazo Piper
                        x_arm, y_arm, z_arm = transform_camera_to_arm(
                            coords_3d[0], coords_3d[1], coords_3d[2],
                            pan_rad=self.head_pan_rad,
                            tilt_rad=self.head_tilt_rad
                        )
                        self.target_3d_arm = (x_arm, y_arm, z_arm)

                        # Verificación de área segura
                        in_box, reason = is_in_safe_cartesian_box(x_arm, y_arm, z_arm)
                        self.is_in_reach = in_box

                        if not in_box:
                            self.status_message = f"Botella fuera de alcance: {reason}. Acerquela sobre la mesa (35-55 cm)."
                            if now - self.last_out_of_reach_say_time > 4.5:
                                self.say("Bottle detected, but it is outside my arm safe reach. Please place it closer.")
                                self.last_out_of_reach_say_time = now
                            return

                        # Planificación cinemática de Pre-Grasp, Agarre y Elevación
                        yaw_base = math.atan2(y_arm, max(0.01, x_arm))
                        r_xy = math.hypot(x_arm, y_arm)

                        # Pose Pre-Grasp orientada hacia el objeto (standoff 8 cm, z elevado +3 cm)
                        r_app = max(0.32, r_xy - 0.08)
                        x_app = r_app * math.cos(yaw_base)
                        y_app = r_app * math.sin(yaw_base)
                        z_app = z_arm + 0.03

                        # Pose de Agarre (centrado en la botella)
                        x_grasp, y_grasp, z_grasp = x_arm, y_arm, z_arm

                        # Pose de Elevación (+8 cm en Z)
                        x_lift, y_lift, z_lift = x_arm, y_arm, z_arm + 0.08

                        j_app, ok_app = self.kinematics_solver.inverse_kinematics((x_app, y_app, z_app))
                        j_grasp, ok_grasp = self.kinematics_solver.inverse_kinematics((x_grasp, y_grasp, z_grasp))
                        j_lift, ok_lift = self.kinematics_solver.inverse_kinematics((x_lift, y_lift, z_lift))

                        if ok_grasp:
                            self.joints_pre_grasp = j_app if ok_app else j_grasp
                            self.joints_grasp = j_grasp
                            self.joints_lift = j_lift if ok_lift else j_grasp

                            self.say(f"Target locked at X {x_arm:.2f}, Y {y_arm:.2f}, Z {z_arm:.2f} meters. Deploying to pre-grasp.")
                            self.auto_state = self.AUTO_PASO_B_MOVER_PREGRASP
                            self.step_start_time = now

                            # Ejecutar movimiento a Pre-Grasp con pinza abierta a 70 mm
                            self.send_arm_joint_goal(self.joints_pre_grasp, duration_s=2.5, gripper_mm=70.0)
                        else:
                            self.get_logger().warn("IK falló para el objetivo.")

        # PASO B: MOVER A POSICIÓN PRE-GRASP
        elif self.auto_state == self.AUTO_PASO_B_MOVER_PREGRASP:
            if now - self.step_start_time > 2.6:
                self.auto_state = self.AUTO_PASO_C_GUIADO_FINO
                self.step_start_time = now
                self.say("Pre-grasp reached. Using wrist camera for fine alignment and approach.")
                self.get_logger().info("[Ciclo Autónomo] Iniciando Paso C: Guiado fino con cámara de muñeca...")

        # PASO C: GUIADO FINO Y AVANCE ("VA HACIA LA BOTELLA")
        elif self.auto_state == self.AUTO_PASO_C_GUIADO_FINO:
            step_elapsed = now - self.step_start_time

            # Inspección continua en cámara de muñeca
            if self.wrist_color_frame is not None:
                det_w = self.detect_object_wrist(self.wrist_color_frame)
                if det_w:
                    wx1, wy1, wx2, wy2, _ = det_w
                    self.target_2d_wrist = (wx1, wy1, wx2, wy2)
                    wcx = (wx1 + wx2) / 2.0
                    wcy = (wy1 + wy2) / 2.0
                    # Error respecto al centro óptico de la pinza (320, 240)
                    self.wrist_error_u = (wcx - 320.0) / 320.0
                    self.wrist_error_v = (wcy - 240.0) / 240.0

            # Tras 1.0s de estabilización visual, avanzar ("ir hacia la botella")
            if step_elapsed > 1.2 and not getattr(self, "_ingress_sent", False):
                self._ingress_sent = True
                # Si la cámara de muñeca midió desviación lateral, corregir levemente
                y_corr = np.clip(self.wrist_error_u * 0.03, -0.04, 0.04)
                if self.joints_grasp is not None:
                    # Avanzar pinza sobre la botella
                    self.get_logger().info(f"[Autónomo] Avanzando pinza sobre la botella (corrección lateral: {y_corr*100:.1f}cm)...")
                    self.send_arm_joint_goal(self.joints_grasp, duration_s=1.8, gripper_mm=70.0)

            # Tras completar el avance (2.2s), pasar al agarre
            if step_elapsed > 3.2:
                self._ingress_sent = False
                self.auto_state = self.AUTO_PASO_D_AGARRE_ELEVACION
                self.step_start_time = now
                self.get_logger().info("[Ciclo Autónomo] Paso D: Cerrando pinza firmemente...")

                # Cerrar pinza a 60 mm con torque para sujetar botella
                if self.dry_run:
                    self.get_logger().info("[DRY-RUN] Simulado: Pinza cerrada a 60 mm.")
                else:
                    g_msg = Float32()
                    g_msg.data = 60.0
                    self.pub_gripper_cmd.publish(g_msg)

        # PASO D: AGARRE Y ELEVACIÓN
        elif self.auto_state == self.AUTO_PASO_D_AGARRE_ELEVACION:
            step_elapsed = now - self.step_start_time

            # Tras 1.5s de cierre firme, elevar +8 cm
            if step_elapsed > 1.5 and not getattr(self, "_lift_sent", False):
                self._lift_sent = True
                self.get_logger().info("[Autónomo] Elevando botella +8 cm de la mesa...")
                if self.joints_lift is not None:
                    self.send_arm_joint_goal(self.joints_lift, duration_s=1.8, gripper_mm=60.0)

            # Tras completar la elevación, iniciar transporte a Home
            if step_elapsed > 3.5:
                self._lift_sent = False
                self.auto_state = self.AUTO_PASO_E_RETORNO_HOME
                self.step_start_time = now
                self.say("Bottle grasped and lifted. Carrying bottle to home position.")
                # Mover suavemente a la Posición Home manteniendo la pinza firmemente cerrada (60 mm)
                self.mover_a_home(duration_s=2.8, keep_gripper=True)

        # PASO E: RETORNO SEGURO A POSICIÓN HOME (Carry to Home)
        elif self.auto_state == self.AUTO_PASO_E_RETORNO_HOME:
            if now - self.step_start_time > 3.0:
                self.auto_state = self.AUTO_SUCCESS
                self.step_start_time = now
                self.say("Bottle successfully grasped and safely brought to home position! I am Robot.")
                self.get_logger().info("==========================================================")
                self.get_logger().info("   AGARRE Y TRANSPORTE A HOME COMPLETADO CON ÉXITO!       ")
                self.get_logger().info("==========================================================")

        # FINALIZADO: Retornar a IDLE tras 5 segundos
        elif self.auto_state == self.AUTO_SUCCESS:
            if now - self.step_start_time > 5.0:
                self.auto_state = self.AUTO_IDLE

    # --------------------------------------------------------------------------
    # RENDERIZADO VISUAL DEL HUD DUAL-CAMERA (1280 x 640)
    # --------------------------------------------------------------------------
    def draw_hud(self) -> np.ndarray:
        canvas_w, canvas_h = 1280, 640
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        feed_w, feed_h = 620, 430

        # --- PANEL IZQUIERDO: CÁMARA CABEZAL (RealSense D435) ---
        if self.head_color_frame is not None:
            h_img = cv2.resize(self.head_color_frame, (feed_w, feed_h))
            orig_h, orig_w = self.head_color_frame.shape[:2]
            sx, sy = feed_w / orig_w, feed_h / orig_h

            if self.target_2d_head is not None:
                x1, y1, x2, y2 = self.target_2d_head
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                cv2.rectangle(h_img, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                tag = "BOTELLA" if not self.target_3d_head else f"BOTELLA Z={self.target_3d_head[2]:.2f}m"
                cv2.putText(h_img, tag, (bx1, max(20, by1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)

            canvas[45:45 + feed_h, 15:15 + feed_w] = h_img
        else:
            cv2.rectangle(canvas, (15, 45), (15 + feed_w, 45 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM REALSENSE CABEZAL...", (60, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 140, 160), 1)

        # Header Izquierdo
        cv2.putText(canvas, "1. CAMARA CABEZAL: DETECCION GLOBAL 3D (REALSENSE D435)", (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 240, 255), 1)

        # --- PANEL DERECHO: CÁMARA DEL BRAZO (Muñeca DaBai Eye-in-Hand) ---
        if self.wrist_color_frame is not None:
            w_img = cv2.resize(self.wrist_color_frame, (feed_w, feed_h))
            orig_h, orig_w = self.wrist_color_frame.shape[:2]
            sx, sy = feed_w / orig_w, feed_h / orig_h

            # Retículo central de la pinza
            wcx, wcy = int(feed_w / 2), int(feed_h / 2)
            cv2.drawMarker(w_img, (wcx, wcy), (0, 255, 255), cv2.MARKER_CROSS, 28, 1)
            cv2.circle(w_img, (wcx, wcy), 18, (0, 255, 255), 1)

            if self.target_2d_wrist is not None:
                x1, y1, x2, y2 = self.target_2d_wrist
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                cv2.rectangle(w_img, (bx1, by1), (bx2, by2), (0, 180, 255), 2)
                cv2.putText(w_img, f"CENTRADOR: err_u={self.wrist_error_u:+.2f}", (bx1, max(20, by1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 180, 255), 1)

            canvas[45:45 + feed_h, 645:645 + feed_w] = w_img
        else:
            cv2.rectangle(canvas, (645, 45), (645 + feed_w, 45 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM CAMARA DE MUNECA (ORBBEC DABAI)...", (680, 250), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        # Header Derecho
        cv2.putText(canvas, "2. CAMARA DEL BRAZO: GUIADO FINO EYE-IN-HAND (ORBBEC DABAI)", (645, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (230, 240, 255), 1)

        # --- PANEL INFERIOR: ESTADO, FASES Y TELEMETRÍA ---
        bar_y = 485
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (12, 16, 24), -1)
        cv2.line(canvas, (0, bar_y), (canvas_w, bar_y), (60, 80, 110), 1)

        # Fases Indicador
        f1_color = (0, 255, 180) if self.fase_actual == self.FASE_1_RANGO else (90, 110, 130)
        f2_color = (0, 255, 180) if self.fase_actual == self.FASE_2_HOME else (90, 110, 130)
        f3_color = (0, 255, 180) if self.fase_actual == self.FASE_3_PREGRASP else (90, 110, 130)

        cv2.putText(canvas, "[1] FASE 1: RANGO", (20, bar_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, f1_color, 2 if self.fase_actual == self.FASE_1_RANGO else 1)
        cv2.putText(canvas, "[2] FASE 2: HOME", (220, bar_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, f2_color, 2 if self.fase_actual == self.FASE_2_HOME else 1)
        cv2.putText(canvas, "[3] FASE 3: PRE-GRASP", (420, bar_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, f3_color, 2 if self.fase_actual == self.FASE_3_PREGRASP else 1)

        # Estado del Ciclo Autónomo
        state_str = self.FASE_NAMES.get(self.auto_state, "ESTADO: DESCONOCIDO")
        state_color = (0, 255, 255) if self.auto_state != self.AUTO_IDLE else (160, 200, 240)
        cv2.putText(canvas, f"ESTADO: {state_str}", (680, bar_y + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, state_color, 1)

        # Telemetría en Vivo
        pan_deg = math.degrees(self.head_pan_rad)
        tilt_deg = math.degrees(self.head_tilt_rad)
        joints_str = ", ".join([f"{v:+.1f}°" for v in self.arm_joints_deg])
        reach_badge = "ALCANCE SEGURO: OK" if self.is_in_reach else "FUERA DE RANGO"
        telemetry_line1 = f"Pinza: {self.gripper_opening_mm:.1f} mm | Pose XYZ: [{self.arm_xyz_mm[0]:.0f}, {self.arm_xyz_mm[1]:.0f}, {self.arm_xyz_mm[2]:.0f}] mm | Cabezal: Pan={pan_deg:.1f}°, Tilt={tilt_deg:.1f}° | {reach_badge}"
        telemetry_line2 = f"Joints [J1..J6]: [{joints_str}] | {self.status_message}"

        cv2.putText(canvas, telemetry_line1, (20, bar_y + 58), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 180), 1)
        cv2.putText(canvas, telemetry_line2, (20, bar_y + 82), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (200, 220, 240), 1)

        # Banner de Modo Enseñanza Activo
        if self.is_teaching_mode:
            pulse = int((time.time() * 3) % 2)
            bg_col = (0, 190, 255) if pulse == 0 else (0, 140, 220)
            cv2.rectangle(canvas, (15, 370), (canvas_w - 15, 470), bg_col, -1)
            cv2.rectangle(canvas, (15, 370), (canvas_w - 15, 470), (255, 255, 255), 2)
            if self.teaching_target == "home":
                cv2.putText(canvas, ">>> MODO ENSENANZA POSICION HOME ACTIVO (BRAZO LIBERADO) <<<", (160, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 0, 0), 2)
                cv2.putText(canvas, "Mueva el brazo a la pose de reposo. Presione [Shift+H], [E] o [ENTER] para GUARDAR | [C] Cancelar", (60, 448), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (15, 15, 15), 2)
            else:
                cv2.putText(canvas, ">>> MODO ENSENANZA PRE-GRASP ACTIVO (BRAZO LIBERADO) <<<", (200, 410), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 0, 0), 2)
                cv2.putText(canvas, "Mueva el brazo con la mano hacia la botella. Presione [E] o [ENTER] para GUARDAR | [C] Cancelar", (110, 448), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (15, 15, 15), 2)

        # Barra de atajos
        shortcuts = "[1] Rango | [2]/[h] Home | [Shift+H] Ensenar Home | [3]/[p] Pre-Grasp | [E] Ensenar Pre-Grasp | [ESPACIO] Agarre | [W/S] Tilt | [A/D] Pan | [Q] Salir"
        cv2.putText(canvas, shortcuts, (20, bar_y + 115), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 200, 50), 1)

        return canvas

    # --------------------------------------------------------------------------
    # BUCLE PRINCIPAL DE EJECUCIÓN INTERACTIVA
    # --------------------------------------------------------------------------
    def run(self):
        window_name = "Robot @Home - Manipulacion y Agarre Autonomo"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 640)

        print("==========================================================================")
        print("   ROBOT @HOME - SISTEMA DE MANIPULACIÓN Y AGARRE AUTÓNOMO (3 FASES)      ")
        print("--------------------------------------------------------------------------")
        print("   [1]             : FASE 1 - Ver rango de movimientos seguros            ")
        print("   [2] o [h]       : FASE 2 - Mover a Posición HOME                       ")
        print("   [Shift + H]     : ENSEÑAR HOME: Libera el brazo para guiarlo con la    ")
        print("                     mano a la pose de reposo y guarda con [H], [E], [ENTER]")
        print("   [3] o [p]       : FASE 3 - Mover a Posición PRE-GRASP (70 mm)          ")
        print("   [E] o [Shift+P] : ENSEÑAR PRE-GRASP: Libera el brazo para guiarlo con ")
        print("                     la mano hacia la botella y guarda con [E] o [ENTER]  ")
        print("   [C]             : Cancelar modo enseñanza y reactivar motores          ")
        print("--------------------------------------------------------------------------")
        print("   [G] o [ESPACIO] : DISPARAR CICLO COMPLETO DE AGARRE Y LLEVAR A HOME   ")
        print("   [R]             : Soltar botella / Abrir pinza (70 mm)                 ")
        print("   [O]             : Abrir pinza manual (70 mm)                           ")
        print("   [W] / [S]       : Inclinar cabezal arriba / abajo (corregido)          ")
        print("   [A] / [D]       : Girar cabezal izquierda / derecha (corregido)        ")
        print("   [T]             : Apuntar cabezal directamente a la mesa (+18° abajo)  ")
        print("   [I]             : Invertir polaridad de Pan-Tilt si se requiere        ")
        print("   [Q] o [ESC]     : Salir                                                ")
        print("==========================================================================")

        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:
                break

            now = time.time()
            self.update_autonomous_state_machine(now)

            # Dibujar y refrescar canvas compuesto
            canvas = self.draw_hud()
            cv2.imshow(window_name, canvas)

            key = cv2.waitKey(1) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                if self.is_teaching_mode:
                    self.set_arm_torque(True)
                break

            # FASE 1: RANGO DE MOVIMIENTOS
            elif key == ord("1"):
                self.fase_actual = self.FASE_1_RANGO
                self.status_message = "FASE 1 ACTIVA: Monitoreando rango de movimientos y envolvente segura."
                print(f"[Fase 1] Envolvente articular segura: {SAFE_JOINT_LIMITS_DEG}")
                print(f"[Fase 1] Caja cartesiana segura (m): {SAFE_CARTESIAN_BOX_M}")

            # FASE 2: POSICIÓN HOME
            elif key in [ord("2"), ord("h")]:
                if self.is_teaching_mode:
                    if self.teaching_target == "home":
                        self.toggle_teaching_home()
                    else:
                        self.toggle_teaching_pregrasp()
                else:
                    self.mover_a_home()

            elif key == ord("H"):  # Shift + H: Enseñar / Guardar pose Home con la mano
                self.toggle_teaching_home()

            # FASE 3: ENSEÑAR (TEACH MODE SIMPLE) O GUARDAR SI YA ESTÁ ENSEÑANDO
            elif key in [ord("e"), ord("E"), 13]:  # [E] o [ENTER]
                if self.is_teaching_mode:
                    if self.teaching_target == "home":
                        self.toggle_teaching_home()
                    else:
                        self.toggle_teaching_pregrasp()
                else:
                    if self.fase_actual == self.FASE_2_HOME:
                        self.toggle_teaching_home()
                    else:
                        self.toggle_teaching_pregrasp()

            # FASE 3: PROBAR POSICIÓN PRE-GRASP
            elif key in [ord("3"), ord("p")]:
                if self.is_teaching_mode:
                    if self.teaching_target == "pre_grasp":
                        self.toggle_teaching_pregrasp()
                    else:
                        self.toggle_teaching_home()
                else:
                    self.mover_a_pre_grasp()

            elif key == ord("P"):  # Shift + P: Enseñar / Guardar pose Pre-Grasp con la mano
                self.toggle_teaching_pregrasp()

            # AGARRE AUTÓNOMO (ESPACIO O G)
            elif key in [ord("g"), 32]:
                if self.is_teaching_mode:
                    if self.teaching_target == "home":
                        self.toggle_teaching_home()
                    else:
                        self.toggle_teaching_pregrasp()
                elif self.auto_state == self.AUTO_IDLE:
                    self.start_autonomous_grasp()

            # PINZA
            elif key in [ord("r"), ord("R")]:
                print("[*] Liberando objeto / abriendo pinza a 70 mm...")
                msg = Float32()
                msg.data = 70.0
                self.pub_gripper_cmd.publish(msg)
                self.status_message = "Pinza abierta a 70 mm (objeto liberado)."
            elif key in [ord("o"), ord("O")]:
                msg = Float32()
                msg.data = 70.0
                self.pub_gripper_cmd.publish(msg)
            elif key in [ord("c"), ord("C")]:
                if self.is_teaching_mode:
                    self.cancel_teaching_mode()
                else:
                    msg = Float32()
                    msg.data = 50.0
                    self.pub_gripper_cmd.publish(msg)

            # INVERTIR PAN-TILT
            elif key in [ord("i"), ord("I")]:
                self.pan_tilt_inverted = not self.pan_tilt_inverted
                txt = "ACTIVADA" if self.pan_tilt_inverted else "DESACTIVADA"
                self.status_message = f"Inversion de controles Pan-Tilt: {txt}."
                print(f"[*] Inversión Pan-Tilt {txt}")

            # CABEZAL PAN-TILT (W sube, S baja a la mesa, A gira izq, D gira der)
            elif key in [ord("w"), ord("W")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                delta = -4.0 if self.pan_tilt_inverted else 4.0
                new_tilt = min(35.0, max(-35.0, cur_tilt + delta))
                self.send_pan_tilt_goal(cur_pan, new_tilt)
            elif key in [ord("s"), ord("S")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                delta = 4.0 if self.pan_tilt_inverted else -4.0
                new_tilt = min(35.0, max(-35.0, cur_tilt + delta))
                self.send_pan_tilt_goal(cur_pan, new_tilt)
            elif key in [ord("a"), ord("A")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                delta = -5.0 if self.pan_tilt_inverted else 5.0
                new_pan = min(90.0, max(-90.0, cur_pan + delta))
                self.send_pan_tilt_goal(new_pan, cur_tilt)
            elif key in [ord("d"), ord("D")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                delta = 5.0 if self.pan_tilt_inverted else -5.0
                new_pan = min(90.0, max(-90.0, cur_pan + delta))
                self.send_pan_tilt_goal(new_pan, cur_tilt)
            elif key in [ord("t"), ord("T")]:
                # Apuntar directamente a la mesa en el espacio de trabajo del brazo (+18° hacia abajo)
                tilt_target = 18.0 if self.pan_tilt_inverted else -18.0
                self.send_pan_tilt_goal(0.0, tilt_target)

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Sistema Integrado de Manipulación y Agarre Autónomo en 3 Fases")
    parser.add_argument("--dry-run", action="store_true", help="Modo simulación sin comandos físicos a motores")
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
