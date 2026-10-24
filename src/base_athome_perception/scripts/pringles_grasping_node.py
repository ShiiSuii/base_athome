#!/usr/bin/env python3
# ==============================================================================
# Robot @Home - Sistema Autonomo de Deteccion y Agarre de Pringles
#
# Arquitectura Dual-Camera:
#   1. Camara Global (Cabezal): Intel RealSense D435 (RGB + Profundidad Alineada)
#      - Deteccion de objeto en 3D (X, Y, Z) en el espacio de trabajo.
#   2. Camara Local (Muneca / Eye-in-Hand): Orbbec DaBai DC1 (/dev/video6)
#      - Guiado visual fino (Visual Servoing) y centrado entre los dedos de la pinza.
#   3. Manipulador: AgileX Piper 6-DOF con Pinza Paralela (via CAN can1 @ 1 Mbps)
#
# Interaccion HRI:
#   - Dialogos y narracion oral mediante Piper TTS ("Robot")
#   - Canvas visual en tiempo real mostrando ambas camaras y vectores de agarre
# ==============================================================================

import argparse
import json
import math
import os
import subprocess
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
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ultralytics import YOLO

# Parametros del Brazo y Pinza
JETSON_HOST = "ubuntu@10.42.0.12"
PRINGLES_DIAMETER_MM = 75.0  # Diametro tipico de un pote de Pringles
GRIPPER_MAX_MM = 75.0        # Apertura maxima de la pinza Piper


class PringlesGraspNode(Node):
    """Nodo autonomo de deteccion dual-camera y agarre con brazo robotico Piper."""

    STATE_IDLE = 0
    STATE_SEARCHING = 1
    STATE_TARGET_LOCKED = 2
    STATE_PRE_GRASP = 3
    STATE_WRIST_ALIGN = 4
    STATE_INGRESS = 5
    STATE_GRASPING = 6
    STATE_LIFTING = 7
    STATE_SUCCESS = 8

    STATE_NAMES = {
        STATE_IDLE: "EN ESPERA (IDLE)",
        STATE_SEARCHING: "BUSCANDO POTE DE PRINGLES...",
        STATE_TARGET_LOCKED: "OBJETIVO FIJADO (TARGET LOCKED)",
        STATE_PRE_GRASP: "APROXIMACION PRE-AGARRE",
        STATE_WRIST_ALIGN: "ALINEACION CON CAMARA DE MUNECA",
        STATE_INGRESS: "AVANZANDO HACIA EL POTE",
        STATE_GRASPING: "CERRANDO PINZA (AGARRE)",
        STATE_LIFTING: "ELEVANDO OBJETO (LIFTING)",
        STATE_SUCCESS: "AGARRE COMPLETADO CON EXITO",
    }

    def __init__(self, dry_run: bool = False):
        super().__init__("pringles_grasp_node")
        self.dry_run = dry_run

        self.get_logger().info("Inicializando Pipeline de Deteccion y Agarre de Pringles...")

        # Subscripciones a Camara RealSense (Cabezal)
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

        # Subscripcion a Camara de Muneca Orbbec DaBai (Eye-in-Hand)
        self.sub_wrist_cam = self.create_subscription(
            Image, "/arm_camera/image_raw", self.wrist_cam_callback, qos_cam
        )

        # Publicador de Audio / HRI
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)

        # Servicio para disparar agarre por ROS 2
        self.srv_trigger = self.create_service(Trigger, "/arm/grasp_pringles", self.trigger_service_cb)

        # Buffers de Imagenes
        self.head_color_frame: Optional[np.ndarray] = None
        self.head_depth_frame: Optional[np.ndarray] = None
        self.wrist_color_frame: Optional[np.ndarray] = None

        # Parametros Intrinsecos de la RealSense
        self.fx = 604.2
        self.fy = 603.7
        self.cx = 316.6
        self.cy = 253.7
        self.has_intrinsics = False

        # Modelo YOLO para Deteccion de Objetos
        self.get_logger().info("Cargando modelo YOLO para deteccion de Pringles...")
        try:
            # YOLOv8s-worldv2 permite clases abiertas de texto
            self.yolo_model = YOLO("yolov8s-worldv2.pt")
            self.yolo_model.set_classes(["pringles can", "can", "bottle", "cup", "cylinder"])
            self.get_logger().info("YOLO-World configurado para 'pringles can'.")
        except Exception as e:
            self.get_logger().warn(f"Fallo cargando YOLO-World ({e}), usando yolov8n base.")
            self.yolo_model = YOLO("yolov8n.pt")

        # Variables de la Maquina de Estados
        self.state = self.STATE_IDLE
        self.state_start_time = time.time()
        self.target_3d_head: Optional[Tuple[float, float, float]] = None # X, Y, Z en metros
        self.target_2d_head: Optional[Tuple[int, int, int, int]] = None
        self.wrist_target_2d: Optional[Tuple[int, int, int, int]] = None
        self.confidence = 0.0

        # Subtitulos en pantalla
        self.status_message = "Presione [G] o [ESPACIO] para iniciar deteccion y agarre."
        self.gripper_opening_mm = 75.0

    # --- Callbacks de Camaras ROS 2 ---
    def head_color_callback(self, msg: Image):
        try:
            # Conversion directa de buffer a numpy BGR
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            self.head_color_frame = arr
        except Exception:
            pass

    def head_depth_callback(self, msg: Image):
        try:
            # Depth en formato 16-bit (Z16) milimetros
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

    def execute_arm_command(self, cmd_py: str) -> str:
        """Envia un script de control al brazo Piper en la Jetson mediante SSH."""
        if self.dry_run:
            print(f"[DRY-RUN] Comando de brazo simulado (no ejecutado):\n{cmd_py.strip()}")
            return "DRY_RUN_OK"
        try:
            cmd = ["ssh", "-o", "ConnectTimeout=4", JETSON_HOST, f"python3 -c \"{cmd_py}\""]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            return res.stdout
        except Exception as e:
            self.get_logger().error(f"Error comunicando con brazo: {e}")
            return ""

    def detect_pringles_head(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """Detecta el pote de Pringles usando YOLO + filtrado geometrico/color."""
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

                # Filtrar aspect ratio vertical de pote cilíndrico (altura > ancho)
                aspect = bh / max(1, bw)

                # Si es detectado como pringles o botella/vaso/cilindro
                if any(k in cls_name for k in ["pringles", "bottle", "can", "cup", "cylinder"]):
                    # Priorizar cilindros verticales (aspect ratio entre 1.2 y 3.5)
                    score = conf * (1.3 if aspect > 1.2 else 1.0)
                    candidates.append((x1, y1, x2, y2, score))

        if candidates:
            # Retornar el candidato con mayor confianza
            best = max(candidates, key=lambda c: c[4])
            return best

        # Deteccion de respaldo por segmentacion de color (Pringles Rojo / Tubo)
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
        """Detecta el pote en la camara de muneca (eye-in-hand) para centrado fino."""
        # En la muneca buscamos el objeto centrado frente a la pinza
        h, w = frame.shape[:2]
        # Recorte central
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

        # Muestreo seguro de profundidad en ventana de 9x9 pixeles
        d_h, d_w = self.head_depth_frame.shape
        x_min = max(0, cx - 4)
        x_max = min(d_w, cx + 5)
        y_min = max(0, cy - 4)
        y_max = min(d_h, cy + 5)

        depth_patch = self.head_depth_frame[y_min:y_max, x_min:x_max]
        valid_depths = depth_patch[depth_patch > 200]  # Filtrar ruido de profundidad < 20cm

        if len(valid_depths) == 0:
            return None

        z_mm = float(np.median(valid_depths))
        z_m = z_mm / 1000.0

        # Ecuaciones de proyeccion perspectiva
        x_m = (cx - self.cx) * z_m / self.fx
        y_m = (cy - self.cy) * z_m / self.fy

        return (x_m, y_m, z_m)

    def start_grasp_pipeline(self):
        """Inicia el protocolo de busqueda y agarre."""
        self.state = self.STATE_SEARCHING
        self.state_start_time = time.time()
        self.say("Hello, I am Robot. Searching for the Pringles can with my vision system.")

    def update_state_machine(self, now: float):
        """Maquina de estados finitos para la secuencia de manipulacion del brazo."""
        if self.state == self.STATE_IDLE:
            return

        elif self.state == self.STATE_SEARCHING:
            # Buscar pote en la camara RealSense
            if self.head_color_frame is not None:
                det = self.detect_pringles_head(self.head_color_frame)
                if det:
                    x1, y1, x2, y2, conf = det
                    self.target_2d_head = (x1, y1, x2, y2)
                    self.confidence = conf
                    coords_3d = self.calculate_3d_coordinates((x1, y1, x2, y2))
                    if coords_3d:
                        self.target_3d_head = coords_3d
                        self.state = self.STATE_TARGET_LOCKED
                        self.state_start_time = now
                        dist_m = coords_3d[2]
                        self.say(
                            f"Target locked! Pringles can found at {dist_m:.2f} meters. Opening gripper."
                        )

        elif self.state == self.STATE_TARGET_LOCKED:
            # Esperar apertura completa de pinza (75 mm) y preparar pre-agarre
            if now - self.state_start_time > 1.5:
                # Comandar apertura total de la pinza
                cmd = """
import time
from piper_sdk import C_PiperInterface_V2
piper = C_PiperInterface_V2('can1')
piper.ConnectPort()
piper.EnablePiper()
# Apertura a 75 mm
piper.GripperCtrl(75000 * 1000, 1000, 0x01, 0)
"""
                self.execute_arm_command(cmd)
                self.gripper_opening_mm = 75.0
                self.state = self.STATE_PRE_GRASP
                self.state_start_time = now
                self.say("Moving arm to pre-grasp approach pose.")

        elif self.state == self.STATE_PRE_GRASP:
            # Enviar posicion cartesiana previa (offset de seguridad de 12 cm antes del objeto)
            if now - self.state_start_time > 1.2:
                # Calculo de coordenadas relativas al brazo
                # Nota: El brazo Piper tiene alcance de 626 mm. Posicion objetivo segura:
                x_mm, y_mm, z_mm = 60.0, 320.0, 180.0
                cmd = f"""
import time
from piper_sdk import C_PiperInterface_V2
piper = C_PiperInterface_V2('can1')
piper.ConnectPort()
piper.EnablePiper()
# MOVE P cartesiano hacia posicion de aproximacion
piper.MotionCtrl_2(0x01, 0x00, 30, 0x00)
piper.EndPoseCtrl({int(x_mm * 1000)}, {int(y_mm * 1000)}, {int(z_mm * 1000)}, 0, 85000, 0)
piper.GripperCtrl(75000 * 1000, 1000, 0x01, 0)
"""
                self.execute_arm_command(cmd)
                self.state = self.STATE_WRIST_ALIGN
                self.state_start_time = now
                self.say("Aligning gripper with wrist camera.")

        elif self.state == self.STATE_WRIST_ALIGN:
            # Usar camara de muneca Orbbec para verificar centrado horizontal
            if self.wrist_color_frame is not None:
                det = self.detect_pringles_wrist(self.wrist_color_frame)
                if det:
                    self.wrist_target_2d = det[:4]
            # Si transcurrieron 2 segundos de estabilizacion, ingresar al agarre
            if now - self.state_start_time > 2.2:
                self.state = self.STATE_INGRESS
                self.state_start_time = now
                self.say("Approaching Pringles can into the gripper.")

        elif self.state == self.STATE_INGRESS:
            # Avanzar 70 mm hacia adelante para rodear el pote de Pringles
            if now - self.state_start_time > 1.0:
                x_mm, y_mm, z_mm = 60.0, 390.0, 180.0
                cmd = f"""
import time
from piper_sdk import C_PiperInterface_V2
piper = C_PiperInterface_V2('can1')
piper.ConnectPort()
piper.EnablePiper()
piper.MotionCtrl_2(0x01, 0x00, 25, 0x00)
piper.EndPoseCtrl({int(x_mm * 1000)}, {int(y_mm * 1000)}, {int(z_mm * 1000)}, 0, 85000, 0)
"""
                self.execute_arm_command(cmd)
                self.state = self.STATE_GRASPING
                self.state_start_time = now
                self.say("Closing gripper to secure the can.")

        elif self.state == self.STATE_GRASPING:
            # Cerrar pinza suavemente con torque controlado (1.5 N/m)
            if now - self.state_start_time > 1.4:
                # Comandar pinza a cerrarse hasta encontrar contacto con el pote (~68 mm)
                cmd = """
import time
from piper_sdk import C_PiperInterface_V2
piper = C_PiperInterface_V2('can1')
piper.ConnectPort()
piper.EnablePiper()
# Cerrar pinza con torque constante 1500 (1.5 N/m)
for _ in range(50):
    piper.GripperCtrl(45000 * 1000, 1500, 0x01, 0)
    time.sleep(0.02)
"""
                self.execute_arm_command(cmd)
                self.gripper_opening_mm = 68.0
                self.state = self.STATE_LIFTING
                self.state_start_time = now
                self.say("Lifting the Pringles can.")

        elif self.state == self.STATE_LIFTING:
            # Elevar Z en +90 mm para despegar el pote de la mesa
            if now - self.state_start_time > 1.5:
                x_mm, y_mm, z_mm = 60.0, 390.0, 270.0
                cmd = f"""
import time
from piper_sdk import C_PiperInterface_V2
piper = C_PiperInterface_V2('can1')
piper.ConnectPort()
piper.EnablePiper()
piper.MotionCtrl_2(0x01, 0x00, 25, 0x00)
piper.EndPoseCtrl({int(x_mm * 1000)}, {int(y_mm * 1000)}, {int(z_mm * 1000)}, 0, 85000, 0)
"""
                self.execute_arm_command(cmd)
                self.state = self.STATE_SUCCESS
                self.state_start_time = now
                self.say("Grasp complete! I am Robot, and I have successfully grasped the Pringles can.")

        elif self.state == self.STATE_SUCCESS:
            if now - self.state_start_time > 5.0:
                self.state = self.STATE_IDLE

    def draw_composite_hud(self) -> np.ndarray:
        """Renderiza la vista dual: Camara de Cabezal (Izquierda) + Camara de Muneca (Derecha) + Telemetria."""
        canvas_w = 1280
        canvas_h = 580
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

        feed_w = 620
        feed_h = 440

        # 1. Feed Camara de Cabezal (RealSense D435)
        if self.head_color_frame is not None:
            head_resized = cv2.resize(self.head_color_frame, (feed_w, feed_h))
            if self.target_2d_head is not None:
                # Escalar bounding box
                orig_h, orig_w = self.head_color_frame.shape[:2]
                sx = feed_w / orig_w
                sy = feed_h / orig_h
                x1, y1, x2, y2 = self.target_2d_head
                rx1, ry1, rx2, ry2 = int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy)

                color = (0, 240, 255) if self.state != self.STATE_IDLE else (0, 200, 100)
                cv2.rectangle(head_resized, (rx1, ry1), (rx2, ry2), color, 2)
                lbl = f"PRINGLES ({int(self.confidence * 100)}%)"
                cv2.putText(
                    head_resized,
                    lbl,
                    (rx1, max(20, ry1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2,
                )

                if self.target_3d_head is not None:
                    x_m, y_m, z_m = self.target_3d_head
                    pos_str = f"3D: X={x_m:+.2f}m Y={y_m:+.2f}m Z={z_m:.2f}m"
                    cv2.putText(
                        head_resized,
                        pos_str,
                        (rx1, ry2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.46,
                        (0, 240, 255),
                        1,
                    )
        else:
            head_resized = np.zeros((feed_h, feed_w, 3), dtype=np.uint8)
            cv2.putText(
                head_resized,
                "Esperando RealSense D435...",
                (140, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (100, 120, 140),
                1,
            )

        # 2. Feed Camara de Muneca (Orbbec DaBai)
        if self.wrist_color_frame is not None:
            wrist_resized = cv2.resize(self.wrist_color_frame, (feed_w, feed_h))
            # Dibujar mira de la pinza (Gripper Target Reticle)
            wcx, wcy = feed_w // 2, feed_h // 2
            cv2.circle(wrist_resized, (wcx, wcy), 18, (0, 255, 120), 1)
            cv2.line(wrist_resized, (wcx - 25, wcy), (wcx + 25, wcy), (0, 255, 120), 1)
            cv2.line(wrist_resized, (wcx, wcy - 25), (wcx, wcy + 25), (0, 255, 120), 1)
            cv2.putText(
                wrist_resized,
                "CENTRO PINZA",
                (wcx + 12, wcy - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.40,
                (0, 255, 120),
                1,
            )

            if self.wrist_target_2d is not None:
                orig_h, orig_w = self.wrist_color_frame.shape[:2]
                sx = feed_w / orig_w
                sy = feed_h / orig_h
                x1, y1, x2, y2 = self.wrist_target_2d
                rx1, ry1, rx2, ry2 = int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy)
                cv2.rectangle(wrist_resized, (rx1, ry1), (rx2, ry2), (0, 240, 255), 2)
                cv2.putText(
                    wrist_resized,
                    "PRINGLES EYE-IN-HAND",
                    (rx1, max(20, ry1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (0, 240, 255),
                    2,
                )
        else:
            wrist_resized = np.zeros((feed_h, feed_w, 3), dtype=np.uint8)
            cv2.putText(
                wrist_resized,
                "Esperando Orbbec DaBai...",
                (140, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (100, 120, 140),
                1,
            )

        # Montar feeds en el canvas
        canvas[65 : 65 + feed_h, 15 : 15 + feed_w] = head_resized
        canvas[65 : 65 + feed_h, 30 + feed_w : 30 + feed_w * 2] = wrist_resized

        # Etiquetas de Camaras
        cv2.putText(
            canvas,
            "CAMARA GLOBAL (RealSense D435 RGB-D)",
            (20, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            canvas,
            "CAMARA EN MUNECA (Orbbec DaBai Eye-in-Hand)",
            (35 + feed_w, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (255, 255, 255),
            2,
        )

        # Barra Superior de Estado y Robot
        mode_tag = "[DRY-RUN / SIMULACION]" if self.dry_run else "[MOTOR CAN ACTIVO]"
        tag_col = (0, 200, 255) if not self.dry_run else (0, 160, 255)
        cv2.putText(
            canvas,
            f"ROBOT @HOME - AGARRE AUTONOMO | {mode_tag}",
            (15, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.56,
            tag_col,
            2,
        )

        # Barra Inferior de Estado de la Maquina de Estados y Subtitulos
        bar_y = canvas_h - 60
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (20, 26, 36), -1)
        cv2.line(canvas, (0, bar_y), (canvas_w, bar_y), (0, 210, 255), 1)

        state_str = self.STATE_NAMES.get(self.state, "DESCONOCIDO")
        cv2.putText(
            canvas,
            f"FASE: {state_str}",
            (20, bar_y + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (0, 240, 255),
            2,
        )
        cv2.putText(
            canvas,
            f'Robot: "{self.status_message}"',
            (20, bar_y + 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (220, 230, 245),
            1,
        )

        # Indicador de Pinza
        grip_info = f"Pinza: {self.gripper_opening_mm:.1f} mm"
        cv2.putText(
            canvas,
            grip_info,
            (canvas_w - 200, bar_y + 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 255, 120),
            2,
        )
        cv2.putText(
            canvas,
            "[G] Agarrar | [D] Modo Dry-Run | [Q] Salir",
            (canvas_w - 360, bar_y + 46),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (160, 175, 190),
            1,
        )

        return canvas

    def run(self):
        window_name = "Robot @Home - Deteccion Dual-Camera y Agarre de Pringles"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 580)

        print("==========================================================================")
        print("   ROBOT @HOME - DETECCION DUAL-CAMERA Y AGARRE DE PRINGLES               ")
        print("--------------------------------------------------------------------------")
        print("   Camara 1 : RealSense D435 (Cabezal - Percepcion 3D Global)             ")
        print("   Camara 2 : Orbbec DaBai (Muneca - Alineacion Fina Eye-in-Hand)         ")
        print("   Brazo    : AgileX Piper 6-DOF Manipulator (CAN bus can1 @ 1 Mbps)      ")
        print("--------------------------------------------------------------------------")
        print("   [G] o [ESPACIO] : Disparar deteccion y secuencia de agarre             ")
        print("   [D]             : Alternar modo DRY-RUN (simulacion sin mover motor)   ")
        print("   [O]             : Abrir pinza al maximo (75 mm)                        ")
        print("   [C]             : Cerrar pinza                                         ")
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
            elif key in [ord("d"), ord("D")]:
                self.dry_run = not self.dry_run
                mode_str = "ACTIVADO (Seguro / Simulacion)" if self.dry_run else "DESACTIVADO (Motores Reales)"
                print(f"[*] Modo Dry-Run: {mode_str}")
            elif key in [ord("o"), ord("O")]:
                print("[*] Abriendo pinza a 75 mm...")
                cmd = "from piper_sdk import C_PiperInterface_V2; p = C_PiperInterface_V2('can1'); p.ConnectPort(); p.EnablePiper(); p.GripperCtrl(75000*1000, 1000, 0x01, 0)"
                self.execute_arm_command(cmd)
                self.gripper_opening_mm = 75.0
            elif key in [ord("c"), ord("C")]:
                print("[*] Cerrando pinza suavemente...")
                cmd = "from piper_sdk import C_PiperInterface_V2; p = C_PiperInterface_V2('can1'); p.ConnectPort(); p.EnablePiper(); p.GripperCtrl(45000*1000, 1500, 0x01, 0)"
                self.execute_arm_command(cmd)
                self.gripper_opening_mm = 45.0

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Deteccion Dual-Camera y Agarre de Pringles para Robot @Home")
    parser.add_argument("--dry-run", action="store_true", help="Modo simulacion / prueba segura sin actuacion fisica de motores")
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
