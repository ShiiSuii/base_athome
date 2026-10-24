#!/usr/bin/env python3
# ==============================================================================
# Robot @Home - Sistema Autónomo de Detección Dual-Camera y Agarre de Pringles
#
# Arquitectura Dual-Camera:
#   1. Cámara Global (Cabezal): Intel RealSense D435 (RGB + Profundidad Alineada)
#      - Detección de objeto en 3D (X, Y, Z) en el espacio de trabajo.
#   2. Cámara Local (Muñeca / Eye-in-Hand): Orbbec DaBai DC1 (/dev/video6)
#      - Guiado visual fino (Visual Servoing) y centrado entre los dedos de la pinza.
#   3. Manipulador: AgileX Piper 6-DOF con Pinza Paralela (via ROS 2 @ 50 Hz)
#
# Interacción HRI:
#   - Diálogos y narración oral mediante Piper TTS ("Robot")
#   - Canvas visual HUD en tiempo real mostrando ambas cámaras y vectores de agarre
#   - Integración nativa con RViz2 (/joint_states del URDF completo)
# ==============================================================================

import argparse
import json
import math
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
# Asegurar acceso a ultralytics del entorno virtual de visión
VENV_SITE_PACKAGES = "/home/athome/ros2_vision_venv/lib/python3.12/site-packages"
if os.path.exists(VENV_SITE_PACKAGES) and VENV_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, VENV_SITE_PACKAGES)

from ultralytics import YOLO

# Importaciones cinemáticas del Brazo Piper y Transformación Cabezal-a-Brazo
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

PRINGLES_DIAMETER_MM = 75.0  # Diámetro típico de un pote de Pringles
GRIPPER_MAX_MM = 70.0        # Apertura máxima de la pinza Piper


class PringlesGraspNode(Node):
    """Nodo autónomo de detección dual-camera y agarre con brazo robótico Piper."""

    STATE_IDLE = 0
    STATE_SEARCHING = 1
    STATE_TARGET_LOCKED = 2
    STATE_GRASPING = 3
    STATE_SUCCESS = 4

    STATE_NAMES = {
        STATE_IDLE: "EN ESPERA (IDLE)",
        STATE_SEARCHING: "BUSCANDO POTE DE PRINGLES...",
        STATE_TARGET_LOCKED: "OBJETIVO FIJADO (TARGET LOCKED)",
        STATE_GRASPING: "EJECUTANDO AGARRE Y ELEVACION",
        STATE_SUCCESS: "AGARRE COMPLETADO CON EXITO",
    }

    def __init__(self, dry_run: bool = False):
        super().__init__("pringles_grasp_node")
        self.dry_run = dry_run

        self.get_logger().info("Inicializando Pipeline de Detección y Agarre de Pringles...")

        # Subscripciones a Cámara RealSense (Cabezal)
        qos_cam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_head_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.head_color_callback, qos_cam
        )
        self.sub_head_depth = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.head_depth_callback,
            qos_cam,
        )
        self.sub_head_info = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.head_info_callback, 10
        )

        # Subscripción a Cámara de Muñeca Orbbec DaBai (Eye-in-Hand)
        self.sub_wrist_cam = self.create_subscription(
            Image, "/arm_camera/image_raw", self.wrist_cam_callback, qos_cam
        )

        # Subscripción a Telemetría del Brazo Piper
        self.sub_arm_status = self.create_subscription(
            String, "/arm/status", self.arm_status_callback, 10
        )

        # Subscripción a Ángulos Pan-Tilt del Cabezal RealSense
        self.sub_pan_tilt = self.create_subscription(
            JointState, "/pan_tilt/joint_states", self.pan_tilt_callback, 10
        )

        # Publicadores ROS 2
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_gripper_cmd = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_joint_cmd = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_pan_tilt_cmd = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)

        # Clientes de Servicios de Manipulación del Brazo
        self.cli_grasp = self.create_client(Trigger, "/arm/grasp_pringles")
        self.cli_release = self.create_client(Trigger, "/arm/release")
        self.cli_home = self.create_client(Trigger, "/arm/home")
        self.cli_pre_grasp = self.create_client(Trigger, "/arm/pre_grasp")

        # Servicio para disparar agarre por ROS 2
        self.srv_trigger = self.create_service(Trigger, "/arm/trigger_grasp", self.trigger_service_cb)

        # Control de Cooldown de Voz
        self.last_out_of_reach_say_time = 0.0

        # Buffers de Imágenes
        self.head_color_frame: Optional[np.ndarray] = None
        self.head_depth_frame: Optional[np.ndarray] = None
        self.wrist_color_frame: Optional[np.ndarray] = None

        # Parámetros Intrínsecos de la RealSense
        self.fx = 604.2
        self.fy = 603.7
        self.cx = 316.6
        self.cy = 253.7
        self.has_intrinsics = False

        # Ángulos del Cabezal Pan-Tilt
        self.head_pan_rad = 0.0
        self.head_tilt_rad = 0.0

        # Solucionador Cinemático y Transformación 3D
        self.kinematics_solver = PiperKinematicsSolver()
        self.target_3d_arm: Optional[Tuple[float, float, float]] = None
        self.target_joints_approach: Optional[List[float]] = None
        self.target_joints_grasp: Optional[List[float]] = None
        self.target_joints_lift: Optional[List[float]] = None
        self.is_target_in_reach = False
        self.grasp_substep = 0
        self.grasp_step_start = 0.0

        # Modelo YOLO para Detección de Objetos
        self.get_logger().info("Cargando modelo YOLO para detección de Pringles...")
        try:
            self.yolo_model = YOLO("yolov8s-worldv2.pt")
            self.yolo_model.set_classes(["pringles can", "can", "bottle", "cup", "cylinder"])
            self.get_logger().info("YOLO-World configurado para 'pringles can'.")
        except Exception as e:
            self.get_logger().warn(f"Fallo cargando YOLO-World ({e}), usando yolov8n base.")
            self.yolo_model = YOLO("yolov8n.pt")

        # Variables de la Máquina de Estados
        self.state = self.STATE_IDLE
        self.state_start_time = time.time()
        self.target_3d_head: Optional[Tuple[float, float, float]] = None  # X, Y, Z en metros ópticos
        self.target_2d_head: Optional[Tuple[int, int, int, int]] = None
        self.wrist_target_2d: Optional[Tuple[int, int, int, int]] = None
        self.confidence = 0.0

        # Telemetría en Vivo del Brazo
        self.arm_is_moving = False
        self.gripper_opening_mm = 69.4
        self.arm_xyz_mm = [0.0, 0.0, 0.0]
        self.arm_joints_deg = [0.0] * 6
        self.grasp_service_sent = False

        # Subtítulos en pantalla
        self.status_message = "Presione [G] o [ESPACIO] para iniciar detección y agarre."

    # --- Callbacks de Cámaras ROS 2 ---
    def head_color_callback(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.head_color_frame = arr
        except Exception:
            pass

    def head_depth_callback(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
            self.head_depth_frame = arr
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
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.wrist_color_frame = arr
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
        """Registra en tiempo real los ángulos de Pan (yaw) y Tilt (pitch) del cabezal."""
        try:
            for name, pos in zip(msg.name, msg.position):
                if "pan" in name:
                    self.head_pan_rad = float(pos)
                elif "tilt" in name:
                    self.head_tilt_rad = float(pos)
        except Exception:
            pass

    def send_arm_joint_goal(
        self,
        joints_deg: List[float],
        duration_s: float = 2.0,
        gripper_mm: Optional[float] = None,
    ):
        """Publica comando articular interpolado a /arm/joint_cmd con verificación de seguridad."""
        safe, reason = is_in_safe_joint_envelope(joints_deg)
        if not safe:
            self.get_logger().error(f"[BLOQUEO DE SEGURIDAD] Comando articular rechazado: {reason}")
            return False

        if self.dry_run:
            self.get_logger().info(
                f"[DRY-RUN] Simulado joint_cmd: {[round(v, 1) for v in joints_deg]}, dur: {duration_s}s, pinza: {gripper_mm}mm"
            )
            return True

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [math.radians(d) for d in joints_deg]
        if gripper_mm is not None:
            js.name.append("gripper")
            js.position.append(gripper_mm)
        js.velocity = [float(duration_s)]
        self.pub_joint_cmd.publish(js)

        if gripper_mm is not None:
            g_msg = Float32()
            g_msg.data = float(gripper_mm)
            self.pub_gripper_cmd.publish(g_msg)

        return True

    def trigger_service_cb(self, request, response):
        if self.state == self.STATE_IDLE:
            self.start_grasp_pipeline()
            response.success = True
            response.message = "Pipeline de agarre iniciado."
        else:
            response.success = False
            response.message = f"Brazo ocupado en estado {self.state}."
        return response

    def say(self, text: str):
        """Sintetiza voz con Piper TTS en el robot."""
        msg = String()
        msg.data = text
        self.pub_say.publish(msg)
        self.status_message = text
        print(f"[Robot]: \"{text}\"")

    def call_arm_service(self, client, name: str):
        """Llama a un servicio de manipulación de manera asíncrona."""
        if self.dry_run:
            print(f"[DRY-RUN] Simulación: Servicio {name} llamado.")
            return
        if not client.service_is_ready():
            self.get_logger().warn(f"Servicio {name} aún no está disponible.")
            return
        req = Trigger.Request()
        future = client.call_async(req)
        self.get_logger().info(f"Comando enviado a servicio {name}...")

    def detect_pringles_head(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """Detecta el pote de Pringles usando YOLO + filtrado geométrico/color."""
        results = self.yolo_model(frame, verbose=False, conf=0.25)
        candidates = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.yolo_model.names.get(cls_id, "").lower()
                conf = float(box.conf[0])
                xyxy = box.xyxy[0].cpu().numpy().astype(int)
                x1, y1, x2, y2 = xyxy
                bw = x2 - x1
                bh = y2 - y1

                aspect = bh / max(1, bw)

                if any(k in cls_name for k in ["pringles", "bottle", "can", "cup", "cylinder"]):
                    score = conf * (1.3 if aspect > 1.2 else 1.0)
                    candidates.append((x1, y1, x2, y2, score))

        if candidates:
            best = max(candidates, key=lambda c: c[4])
            return best

        # Detección de respaldo por color rojo cilíndrico
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 100, 70]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 70]), np.array([180, 255, 255]))
        mask = mask1 | mask2

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1200:
                x, y, w, h = cv2.boundingRect(cnt)
                aspect = h / max(1, w)
                if 1.2 <= aspect <= 3.8:
                    return (x, y, x + w, y + h, 0.65)

        return None

    def detect_pringles_wrist(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """Detecta el pote en la cámara de muñeca (eye-in-hand) para centrado fino."""
        best = self.detect_pringles_head(frame)
        return best

    def calculate_3d_coordinates(
        self, bbox: Tuple[int, int, int, int]
    ) -> Optional[Tuple[float, float, float]]:
        """Desproyecta las coordenadas 2D y el mapa de profundidad a coordenadas 3D (X, Y, Z)."""
        if self.head_depth_frame is None:
            return None

        x1, y1, x2, y2 = bbox
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        d_h, d_w = self.head_depth_frame.shape
        x_min = max(0, cx - 4)
        x_max = min(d_w, cx + 5)
        y_min = max(0, cy - 4)
        y_max = min(d_h, cy + 5)

        depth_patch = self.head_depth_frame[y_min:y_max, x_min:x_max]
        valid_depths = depth_patch[depth_patch > 200]

        if len(valid_depths) == 0:
            return None

        z_mm = float(np.median(valid_depths))
        z_m = z_mm / 1000.0

        x_m = (cx - self.cx) * z_m / self.fx
        y_m = (cy - self.cy) * z_m / self.fy

        return (x_m, y_m, z_m)

    def start_grasp_pipeline(self):
        """Inicia el protocolo de búsqueda y agarre."""
        self.state = self.STATE_SEARCHING
        self.state_start_time = time.time()
        self.grasp_service_sent = False
        self.say("Hello, I am Robot. Searching for the Pringles can with my vision system.")

    def update_state_machine(self, now: float):
        """Máquina de estados finitos coordinada con cinemática 3D y controlador ROS 2."""
        if self.state == self.STATE_IDLE:
            return

        elif self.state == self.STATE_SEARCHING:
            if self.head_color_frame is not None:
                det = self.detect_pringles_head(self.head_color_frame)
                if det:
                    x1, y1, x2, y2, conf = det
                    self.target_2d_head = (x1, y1, x2, y2)
                    self.confidence = conf
                    coords_3d = self.calculate_3d_coordinates((x1, y1, x2, y2))
                    if coords_3d:
                        self.target_3d_head = coords_3d

                        # 1. Transformación 3D: Cámara RealSense (Cabezal Pan-Tilt) -> Base del Brazo Piper
                        x_arm, y_arm, z_arm = transform_camera_to_arm(
                            coords_3d[0], coords_3d[1], coords_3d[2],
                            pan_rad=self.head_pan_rad,
                            tilt_rad=self.head_tilt_rad
                        )
                        self.target_3d_arm = (x_arm, y_arm, z_arm)

                        # 2. Verificación de Área Segura Cartesiana
                        in_box, box_reason = is_in_safe_cartesian_box(x_arm, y_arm, z_arm)
                        self.is_target_in_reach = in_box

                        if not in_box:
                            self.status_message = f"Pote detectado a {coords_3d[2]:.2f}m pero FUERA de alcance: {box_reason}. Coloquelo entre 35 y 55 cm del brazo."
                            if now - self.last_out_of_reach_say_time > 5.0:
                                self.get_logger().warn(f"Pote detectado pero FUERA de zona segura del brazo: {box_reason}")
                                self.say("Pringles can detected, but it is outside the arm workspace. Please place it closer.")
                                self.last_out_of_reach_say_time = now
                            return

                        # 3. Planificación Cinemática Inversa (Aproximación, Ingress y Elevación)
                        yaw_base = math.atan2(y_arm, max(0.01, x_arm))
                        r_xy = math.hypot(x_arm, y_arm)

                        # Puntos de la secuencia de agarre en arm_base_link:
                        # a) Pose de aproximación (standoff radial suave de 6 cm, y 3 cm por encima)
                        r_app = max(0.32, r_xy - 0.06)
                        x_app = r_app * math.cos(yaw_base)
                        y_app = r_app * math.sin(yaw_base)
                        z_app = z_arm + 0.03

                        # b) Pose de agarre (centrado en el pote de Pringles)
                        x_grasp, y_grasp, z_grasp = x_arm, y_arm, z_arm

                        # c) Pose de elevación (8 cm sobre la mesa)
                        x_lift, y_lift, z_lift = x_arm, y_arm, z_arm + 0.08

                        j_app, ok_app = self.kinematics_solver.inverse_kinematics((x_app, y_app, z_app))
                        j_grasp, ok_grasp = self.kinematics_solver.inverse_kinematics((x_grasp, y_grasp, z_grasp))
                        j_lift, ok_lift = self.kinematics_solver.inverse_kinematics((x_lift, y_lift, z_lift))

                        if ok_grasp:
                            self.target_joints_approach = j_app if ok_app else j_grasp
                            self.target_joints_grasp = j_grasp
                            self.target_joints_lift = j_lift if ok_lift else j_grasp

                            self.state = self.STATE_TARGET_LOCKED
                            self.state_start_time = now
                            self.say(
                                f"Target locked! Pringles can located at X {x_arm:.2f}, Y {y_arm:.2f}, Z {z_arm:.2f} meters. Kinematic grasp trajectory computed."
                            )
                            self.get_logger().info(
                                f"[Cinematica] Objetivo fijado: Cam=[{coords_3d[0]:.2f},{coords_3d[1]:.2f},{coords_3d[2]:.2f}]m -> Brazo=[{x_arm:.2f},{y_arm:.2f},{z_arm:.2f}]m"
                            )
                            self.get_logger().info(f"[Cinematica] Joints de agarre calculados: {[round(v, 1) for v in j_grasp]}")
                        else:
                            self.get_logger().warn(f"IK falló para el objetivo de agarre calculado: ({x_grasp:.2f}, {y_grasp:.2f}, {z_grasp:.2f})m")

        elif self.state == self.STATE_TARGET_LOCKED:
            if now - self.state_start_time > 1.0:
                self.state = self.STATE_GRASPING
                self.state_start_time = now
                self.grasp_substep = 1  # 1: Aproximación, 2: Ingress, 3: Clamp, 4: Lift
                self.grasp_step_start = now
                self.grasp_service_sent = True

                # Iniciar Paso 1: Mover a Pose de Aproximación con pinza abierta a 70 mm
                if self.target_joints_approach is not None:
                    self.send_arm_joint_goal(self.target_joints_approach, duration_s=2.5, gripper_mm=70.0)
                else:
                    self.call_arm_service(self.cli_grasp, "/arm/grasp_pringles")

                self.say("Executing approach, grasp and lift trajectory.")

        elif self.state == self.STATE_GRASPING:
            # Inspección continua en la cámara de muñeca para centrado fino
            if self.wrist_color_frame is not None:
                det_w = self.detect_pringles_wrist(self.wrist_color_frame)
                if det_w:
                    self.wrist_target_2d = det_w[:4]

            step_elapsed = now - self.grasp_step_start

            if self.grasp_substep == 1:
                # Esperar a que la aproximación termine (2.5 s)
                if step_elapsed > 2.6:
                    self.grasp_substep = 2
                    self.grasp_step_start = now
                    # Paso 2: Avanzar sobre el pote de Pringles
                    if self.target_joints_grasp is not None:
                        self.send_arm_joint_goal(self.target_joints_grasp, duration_s=1.8, gripper_mm=70.0)
                    self.get_logger().info("[Autonomo] Paso 2: Avanzando pinza sobre el pote de Pringles...")

            elif self.grasp_substep == 2:
                if step_elapsed > 1.9:
                    self.grasp_substep = 3
                    self.grasp_step_start = now
                    # Paso 3: Cerrar pinza para agarre seguro (62 mm para cilindro)
                    if self.dry_run:
                        self.get_logger().info("[DRY-RUN] Simulado: Pinza cerrada a 62 mm con torque.")
                    else:
                        msg_g = Float32()
                        msg_g.data = 62.0
                        self.pub_gripper_cmd.publish(msg_g)
                    self.get_logger().info("[Autonomo] Paso 3: Cerrando pinza (agarre seguro de Pringles)...")

            elif self.grasp_substep == 3:
                if step_elapsed > 1.5:
                    self.grasp_substep = 4
                    self.grasp_step_start = now
                    # Paso 4: Elevar el brazo con el pote agarrado
                    if self.target_joints_lift is not None:
                        self.send_arm_joint_goal(self.target_joints_lift, duration_s=2.0, gripper_mm=62.0)
                    self.get_logger().info("[Autonomo] Paso 4: Elevando pote de Pringles...")

            elif self.grasp_substep == 4:
                if step_elapsed > 2.2:
                    self.state = self.STATE_SUCCESS
                    self.state_start_time = now
                    self.say("Grasp complete! I am Robot, and I have successfully grasped the Pringles can.")

        elif self.state == self.STATE_SUCCESS:
            if now - self.state_start_time > 6.0:
                self.state = self.STATE_IDLE

    def draw_composite_hud(self) -> np.ndarray:
        """Renderiza la vista dual: Cámara Cabezal (Izquierda) + Cámara Muñeca (Derecha) + Telemetría."""
        canvas_w = 1280
        canvas_h = 600
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        feed_w = 620
        feed_h = 440

        # 1. Feed Cámara de Cabezal (RealSense D435)
        if self.head_color_frame is not None:
            head_resized = cv2.resize(self.head_color_frame, (feed_w, feed_h))
            orig_h, orig_w = self.head_color_frame.shape[:2]
            scale_x = feed_w / orig_w
            scale_y = feed_h / orig_h

            if self.target_2d_head is not None:
                x1, y1, x2, y2 = self.target_2d_head
                bx1 = int(x1 * scale_x)
                by1 = int(y1 * scale_y)
                bx2 = int(x2 * scale_x)
                by2 = int(y2 * scale_y)

                # Bounding Box Verde Neón
                cv2.rectangle(head_resized, (bx1, by1), (bx2, by2), (0, 255, 120), 2)
                cx = int((bx1 + bx2) / 2)
                cy = int((by1 + by2) / 2)
                cv2.drawMarker(head_resized, (cx, cy), (0, 255, 120), cv2.MARKER_CROSS, 16, 2)

                label = f"Pringles {self.confidence*100:.0f}%"
                if self.target_3d_head is not None and self.target_3d_arm is not None:
                    xa, ya, za = self.target_3d_arm
                    reach_tag = "[ALCANCE OK]" if self.is_target_in_reach else "[FUERA DE RANGO]"
                    label += f" | Arm:[{xa:.2f},{ya:.2f},{za:.2f}]m {reach_tag}"
                elif self.target_3d_head is not None:
                    x, y, z = self.target_3d_head
                    label += f" | Z:{z:.2f}m"

                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
                cv2.rectangle(head_resized, (bx1, max(0, by1 - 20)), (bx1 + tw + 6, by1), (0, 255, 120), -1)
                cv2.putText(
                    head_resized, label, (bx1 + 3, max(14, by1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (10, 20, 30), 1, cv2.LINE_AA,
                )

            canvas[50 : 50 + feed_h, 10 : 10 + feed_w] = head_resized
        else:
            cv2.putText(
                canvas, "Esperando RealSense D435 (/camera/camera/color/image_raw)...",
                (40, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1,
            )

        # 2. Feed Cámara de Muñeca (Orbbec DaBai Eye-in-Hand)
        if self.wrist_color_frame is not None:
            wrist_resized = cv2.resize(self.wrist_color_frame, (feed_w, feed_h))
            orig_h, orig_w = self.wrist_color_frame.shape[:2]
            scale_x = feed_w / orig_w
            scale_y = feed_h / orig_h

            if self.wrist_target_2d is not None:
                x1, y1, x2, y2 = self.wrist_target_2d
                bx1 = int(x1 * scale_x)
                by1 = int(y1 * scale_y)
                bx2 = int(x2 * scale_x)
                by2 = int(y2 * scale_y)

                cv2.rectangle(wrist_resized, (bx1, by1), (bx2, by2), (0, 200, 255), 2)
                cx = int((bx1 + bx2) / 2)
                cy = int((by1 + by2) / 2)
                cv2.drawMarker(wrist_resized, (cx, cy), (0, 200, 255), cv2.MARKER_TILTED_CROSS, 16, 2)

                cv2.putText(
                    wrist_resized, "Muneca Alineada", (bx1, max(16, by1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1, cv2.LINE_AA,
                )

            # Guías de pinza virtual central
            mid_x = feed_w // 2
            mid_y = feed_h // 2
            cv2.line(wrist_resized, (mid_x - 30, mid_y), (mid_x + 30, mid_y), (0, 120, 255), 1)
            cv2.line(wrist_resized, (mid_x, mid_y - 30), (mid_x, mid_y + 30), (0, 120, 255), 1)

            canvas[50 : 50 + feed_h, 650 : 650 + feed_w] = wrist_resized
        else:
            cv2.putText(
                canvas, "Esperando Orbbec DaBai (/arm_camera/image_raw)...",
                (680, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1,
            )

        # 3. Barra Superior de Títulos
        cv2.rectangle(canvas, (0, 0), (canvas_w, 45), (15, 23, 38), -1)
        cv2.putText(
            canvas, "ROBOT @HOME - DETECCION DUAL-CAMERA Y AGARRE DE PRINGLES",
            (20, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2,
        )

        state_str = self.STATE_NAMES.get(self.state, "DESCONOCIDO")
        state_color = (0, 255, 120) if self.state in [self.STATE_GRASPING, self.STATE_SUCCESS] else (255, 180, 50)
        cv2.putText(
            canvas, f"ESTADO: {state_str}", (canvas_w - 420, 28),
            cv2.FONT_HERSHEY_SIMPLEX, 0.52, state_color, 2,
        )

        # 4. Etiquetas de Cámaras
        cv2.putText(
            canvas, "CAMARA 1: RealSense D435 (Cabezal)", (15, 75),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 220, 255), 1,
        )
        cv2.putText(
            canvas, "CAMARA 2: Orbbec DaBai (Muneca)", (655, 75),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 220, 255), 1,
        )

        # 5. Barra Inferior de Estado y Telemetría
        bar_y = 505
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (18, 25, 40), -1)

        # Mensaje de HRI
        cv2.putText(
            canvas, f"Robot: {self.status_message}", (20, bar_y + 24),
            cv2.FONT_HERSHEY_SIMPLEX, 0.50, (230, 240, 255), 1,
        )

        # Telemetría del Brazo Piper & Cabezal
        pan_deg = math.degrees(self.head_pan_rad)
        tilt_deg = math.degrees(self.head_tilt_rad)
        reach_tag = "ALCANCE SEGURO: OK" if self.is_target_in_reach else "FUERA DE RANGO"
        telemetry_str = f"Pinza: {self.gripper_opening_mm:.1f} mm | Pose: [{self.arm_xyz_mm[0]:.0f}, {self.arm_xyz_mm[1]:.0f}, {self.arm_xyz_mm[2]:.0f}] mm | Cabezal: [{pan_deg:.1f}°, {tilt_deg:.1f}°] | {reach_tag}"
        cv2.putText(
            canvas, telemetry_str, (20, bar_y + 52),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 255, 180), 1,
        )

        # Atajos de Teclado
        cv2.putText(
            canvas, "[W/S] Tilt | [A/D] Pan | [T] Mesa | [G] Agarrar | [R] Soltar | [H] Home | [Q] Salir",
            (canvas_w - 650, bar_y + 52),
            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (160, 180, 205), 1,
        )

        return canvas

    def send_pan_tilt_goal(self, pan_deg: float, tilt_deg: float):
        """Envía comando angular al cabezal Pan-Tilt."""
        v = Vector3()
        v.x = float(pan_deg)
        v.y = float(tilt_deg)
        v.z = 0.0
        self.pub_pan_tilt_cmd.publish(v)
        self.get_logger().info(f"[Cabezal] Comando Pan-Tilt: Pan={pan_deg:.1f}°, Tilt={tilt_deg:.1f}°")

    def run(self):
        window_name = "Robot @Home - Deteccion Dual-Camera y Agarre de Pringles"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 600)

        print("==========================================================================")
        print("   ROBOT @HOME - DETECCION DUAL-CAMERA Y AGARRE DE PRINGLES               ")
        print("--------------------------------------------------------------------------")
        print("   Camara 1 : RealSense D435 (Cabezal - Percepcion 3D Global)             ")
        print("   Camara 2 : Orbbec DaBai (Muneca - Alineacion Fina Eye-in-Hand)         ")
        print("   Brazo    : AgileX Piper 6-DOF Manipulator (ROS 2 @ 50 Hz continuo)     ")
        print("   URDF     : Visualizacion en tiempo real en RViz2 (/joint_states)       ")
        print("--------------------------------------------------------------------------")
        print("   [W] / [S]       : Inclinar cabezal hacia arriba / abajo (+/- 4°)       ")
        print("   [A] / [D]       : Girar cabezal hacia izquierda / derecha (+/- 5°)     ")
        print("   [T]             : Apuntar cabezal a la mesa (Pan 0°, Tilt -18°)        ")
        print("   [G] o [ESPACIO] : Disparar deteccion y secuencia de agarre             ")
        print("   [R]             : Soltar objeto / Abrir pinza                          ")
        print("   [H]             : Retornar brazo a posicion Home                       ")
        print("   [P]             : Mover a pose de aproximacion Pre-Agarre              ")
        print("   [O]             : Abrir pinza manual (70 mm)                           ")
        print("   [C]             : Cerrar pinza manual (45 mm)                          ")
        print("   [Q] o [ESC]     : Salir                                                ")
        print("==========================================================================")

        while rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.02)
            except Exception:
                break

            now = time.time()
            self.update_state_machine(now)

            # Dibujar y mostrar canvas compuesto
            display = self.draw_composite_hud()
            cv2.imshow(window_name, display)

            key = cv2.waitKey(1) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                break
            elif key in [ord("g"), ord("G"), 32]:  # G o ESPACIO
                if self.state == self.STATE_IDLE:
                    self.start_grasp_pipeline()
            elif key in [ord("w"), ord("W")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                self.send_pan_tilt_goal(cur_pan, min(35.0, cur_tilt + 4.0))
            elif key in [ord("s"), ord("S")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                self.send_pan_tilt_goal(cur_pan, max(-28.0, cur_tilt - 4.0))
            elif key in [ord("a"), ord("A")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                self.send_pan_tilt_goal(min(90.0, cur_pan + 5.0), cur_tilt)
            elif key in [ord("d"), ord("D")]:
                cur_pan = math.degrees(self.head_pan_rad)
                cur_tilt = math.degrees(self.head_tilt_rad)
                self.send_pan_tilt_goal(max(-90.0, cur_pan - 5.0), cur_tilt)
            elif key in [ord("t"), ord("T")]:
                # Apuntar directamente a la mesa en el espacio de trabajo del brazo
                self.send_pan_tilt_goal(0.0, -18.0)
            elif key in [ord("r"), ord("R")]:
                print("[*] Liberando objeto / abriendo pinza...")
                self.call_arm_service(self.cli_release, "/arm/release")
            elif key in [ord("h"), ord("H")]:
                print("[*] Retornando a Home...")
                self.call_arm_service(self.cli_home, "/arm/home")
            elif key in [ord("p"), ord("P")]:
                print("[*] Moviendo a Pre-Agarre...")
                self.call_arm_service(self.cli_pre_grasp, "/arm/pre_grasp")
            elif key in [ord("o"), ord("O")]:
                print("[*] Abriendo pinza a 70 mm...")
                msg = Float32()
                msg.data = 70.0
                self.pub_gripper_cmd.publish(msg)
            elif key in [ord("c"), ord("C")]:
                print("[*] Cerrando pinza a 45 mm...")
                msg = Float32()
                msg.data = 45.0
                self.pub_gripper_cmd.publish(msg)

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Deteccion Dual-Camera y Agarre de Pringles para Robot @Home")
    parser.add_argument("--dry-run", action="store_true", help="Modo simulacion sin actuacion fisica de motores")
    args = parser.parse_args()

    rclpy.init()
    node = PringlesGraspNode(dry_run=args.dry_run)
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
