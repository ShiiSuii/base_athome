#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Robot @Home - Manipulación Autónoma 6-DOF con Cámara Orbbec DaBai
#
# Arquitectura Simplificada (Look-then-Move Determinista):
#   - Cámara DaBai montada 8 cm hacia atrás y perfectamente centrada (x = 320 px).
#   - Detección ultraligera con YOLOv8 (bounding box de la lata/botella).
#   - Parámetro único y directo de ajuste: Ajuste_Distancia_cm (cuánto entra el gripper).
#   - Secuencia limpia de agarre:
#       1. Mover a Pre-Grasp con pinza abierta al máximo (105 mm).
#       2. Medir y alinear lateralmente la muñeca con el centro del cilindro.
#       3. Avance en LÍNEA RECTA PURA hacia adelante (sin desviarse a los lados).
#       4. Cierre de pinza sobre la lata (50 mm).
#       5. Despegue vertical (+3.0 cm) y retracción recta a Pre-Grasp.
# ==============================================================================

import argparse
import math
import os
import subprocess
import sys
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, ColorRGBA, Float32, String
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray

# Entorno de visión
VENV_SITE_PACKAGES = "/home/athome/ros2_vision_venv/lib/python3.12/site-packages"
if os.path.exists(VENV_SITE_PACKAGES) and VENV_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, VENV_SITE_PACKAGES)

from ultralytics import YOLO

# Cinemática oficial del Brazo Piper 6-DOF
from base_athome_perception.piper_kinematics import (
    is_in_safe_joint_envelope,
    is_in_safe_cartesian_box,
    PiperKinematicsSolver,
    SAFE_CARTESIAN_BOX_M,
)

CONFIG_FILE_PATH = "/home/athome/base_athome_ws/src/base_athome_perception/config/arm_poses_and_limits.yaml"

# Posición del hombro respecto a base_link en el URDF
SHOULDER_IN_BASE_LINK_M = np.array([0.155, 0.175, 1.015], dtype=float)

# Parámetros físicos del Gripper Piper
GRIPPER_MAX_OPEN_MM = 105.0
GRIPPER_CLOSE_GRASP_MM = 50.0

# Geometría Óptica de la Cámara Orbbec DaBai (640x480):
# Al estar 8 cm más atrás y perfectamente centrada:
DABAI_CENTER_X_PX = 320.0          # Centro óptico horizontal
DABAI_FOCAL_PX = 485.0             # Longitud focal en píxeles
OBJECT_DIAMETER_M = 0.075          # Diámetro de la lata de Pringles (7.5 cm)
DABAI_GRASP_ZONE_DIST_M = 0.270    # Distancia lente -> centro de los dedos (27.0 cm con los 8cm extra)


def sync_jetson_clock():
    """Sincroniza el reloj del sistema con la Jetson para evitar saltos en TF de RViz."""
    try:
        cur_utc = time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime())
        cmd = (
            f"ssh -o ConnectTimeout=2 -o StrictHostKeyChecking=no ubuntu@10.42.0.12 "
            f"\"echo ubuntu | sudo -S date -u -s '{cur_utc}' >/dev/null 2>&1 || true\""
        )
        subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


class ManipulacionAutonomaNode(Node):
    """Nodo simplificado de manipulación autónoma para el brazo Piper con cámara DaBai."""

    # Fases del sistema
    FASE_MANUAL = 1
    FASE_HOME = 2
    FASE_PREGRASP = 3

    # Estados de la Máquina de Agarre Autónomo
    AUTO_IDLE = 10
    AUTO_PASO_1_MOVER_PREGRASP = 11      # Ir a Pre-Grasp con pinza abierta (105 mm)
    AUTO_PASO_2_ALINEAR_LATERAL = 12     # Centrar la lata en el eje X de la cámara
    AUTO_PASO_3_AVANZAR_RECTO = 13       # Avance lineal recto hacia adelante
    AUTO_PASO_4_CERRAR_PINZA = 14        # Cerrar pinza y despegar de la mesa
    AUTO_PASO_5_RETRAER_PREGRASP = 15    # Retroceso en línea recta a Pre-Grasp
    AUTO_SUCCESS = 16

    FASE_NAMES = {
        FASE_MANUAL: "MODO MANUAL / MONITOREO",
        FASE_HOME: "POSICION HOME",
        FASE_PREGRASP: "PRE-GRASP (CAMARA DABAI LISTA)",
        AUTO_IDLE: "LISTO: PRESIONE [ESPACIO] O [G] PARA AGARRAR",
        AUTO_PASO_1_MOVER_PREGRASP: "PASO 1: YENDO A PRE-GRASP (PINZA 105 MM)",
        AUTO_PASO_2_ALINEAR_LATERAL: "PASO 2: ALINEANDO PINZA CON EL CENTRO DEL OBJETO",
        AUTO_PASO_3_AVANZAR_RECTO: "PASO 3: AVANZANDO EN LINEA RECTA AL OBJETO",
        AUTO_PASO_4_CERRAR_PINZA: "PASO 4: CERRANDO PINZA Y DESPEGANDO DE LA MESA",
        AUTO_PASO_5_RETRAER_PREGRASP: "PASO 5: RETROCEDIENDO A PRE-GRASP CON EL OBJETO",
        AUTO_SUCCESS: "EXITO: OBJETO AGARRADO Y SOSTENIDO EN PRE-GRASP",
    }

    def __init__(self, dry_run: bool = False):
        super().__init__("manipulacion_autonoma_node")
        self.dry_run = dry_run
        self.get_logger().info("Sincronizando reloj con Jetson...")
        sync_jetson_clock()

        self.load_configuration()

        # QoS rápido (profundidad 1)
        qos_fast = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_status = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Suscriptores
        self.sub_head_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.head_color_callback, qos_fast
        )
        self.sub_wrist_cam = self.create_subscription(
            Image, "/arm_camera/image_raw", self.wrist_cam_callback, qos_fast
        )
        self.sub_arm_status = self.create_subscription(
            String, "/arm/status", self.arm_status_callback, qos_status
        )
        self.sub_pan_tilt = self.create_subscription(
            JointState, "/pan_tilt/joint_states", self.pan_tilt_callback, qos_fast
        )

        # Publicadores
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_joint_cmd = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_gripper_cmd = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_pan_tilt_cmd = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)
        self.pub_enable = self.create_publisher(Bool, "/arm/enable", 5)
        self.pub_rviz_markers = self.create_publisher(MarkerArray, "/arm_camera/object_markers", 10)

        # Buffers de imágenes
        self.frame_lock = threading.Lock()
        self.head_color_frame: Optional[np.ndarray] = None
        self.wrist_color_frame: Optional[np.ndarray] = None
        self.wrist_frame_timestamp: float = 0.0
        self.wrist_frame_seq: int = 0

        # Telemetría del brazo Piper
        self.arm_is_moving = False
        self.gripper_opening_mm = GRIPPER_MAX_OPEN_MM
        self.arm_joints_deg = list(self.pose_home)
        self.last_cmd_sent_time: float = 0.0
        self.current_move_duration_s: float = 0.0
        self.fresh_post_stop_dets: int = 0

        # Solucionador cinemático oficial
        self.kinematics_solver = PiperKinematicsSolver()

        # Modelo YOLOv8 ultraligero
        self.get_logger().info("Cargando modelo YOLOv8 ultraligero...")
        try:
            self.yolo_model = YOLO("yolov8n.pt")
        except Exception as e:
            self.get_logger().warn(f"Aviso cargando YOLO ({e}), se usara deteccion cromatica fallback.")
            self.yolo_model = None

        # Variables de percepción de la cámara DaBai
        self.fase_actual = self.FASE_HOME
        self.auto_state = self.AUTO_IDLE
        self.step_start_time = time.time()

        self.target_2d_wrist: Optional[Tuple[int, int, int, int]] = None
        self.dabai_cx: float = DABAI_CENTER_X_PX
        self.dabai_cy: float = 240.0
        self.dabai_width_px: float = 0.0
        self.dist_to_can_m: float = 0.0
        self.lateral_err_m: float = 0.0
        self.dist_fwd_remaining_m: float = 0.0
        self.last_det_time: float = 0.0
        self.det_fps: float = 0.0

        # Guardado de poses durante la trayectoria
        self.joints_aligned: Optional[List[float]] = None
        self.joints_grasp: Optional[List[float]] = None
        self.joints_lift: Optional[List[float]] = None
        self.joints_retract: Optional[List[float]] = None
        self.total_forward_advanced_m: float = 0.0
        self.align_attempts: int = 0

        self._window_name = "Robot @Home - Manipulacion Simplificada (Camara 8cm Atras)"
        self.status_message = f"Listo | Distancia Extra: +{self.distancia_gripper_cm:.1f}cm | [ESPACIO]/[G] Agarrar"

        # Lanzar hilos de ROS 2 y Visión
        self._running = True
        self._ros_spin_thread = threading.Thread(target=self._ros_spin_worker, daemon=True)
        self._ros_spin_thread.start()
        self._vision_thread = threading.Thread(target=self._vision_worker, daemon=True)
        self._vision_thread.start()

    def _ros_spin_worker(self):
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.005)
            except Exception:
                time.sleep(0.005)

    def is_arm_settling(self, now: Optional[float] = None) -> bool:
        t = time.time() if now is None else now
        move_busy = (t - self.last_cmd_sent_time) < (self.current_move_duration_s + 0.40)
        return bool(self.arm_is_moving or move_busy)

    # --------------------------------------------------------------------------
    # CONFIGURACIÓN Y PARÁMETRO DE AJUSTE
    # --------------------------------------------------------------------------
    def load_configuration(self):
        """Carga la configuración y las poses desde el archivo YAML."""
        self.pose_home = [72.09, 176.5, -155.15, 102.62, -31.51, -99.91]
        self.pose_pre_grasp = [42.9, 180.83, -89.01, 60.57, -74.02, -96.89]
        self.distancia_gripper_cm: float = 5.0    # Ajuste de avance extra del gripper hacia el objeto
        self.arm_speed_pct: int = 40              # Velocidad suave del brazo (40%)

        if os.path.exists(CONFIG_FILE_PATH):
            try:
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                    env = data.get("safety_envelope", {})
                    if "dabai_extra_advance_cm" in env:
                        self.distancia_gripper_cm = float(env["dabai_extra_advance_cm"])
                    if "arm_speed_pct" in env:
                        self.arm_speed_pct = int(np.clip(int(env["arm_speed_pct"]), 15, 100))
                    poses = data.get("named_poses", {})
                    if "home" in poses:
                        self.pose_home = list(poses["home"].get("joints_deg", self.pose_home))
                    if "pre_grasp" in poses:
                        self.pose_pre_grasp = list(poses["pre_grasp"].get("joints_deg", self.pose_pre_grasp))
                self.get_logger().info(
                    f"Configuracion cargada: DistanciaGripper=+{self.distancia_gripper_cm:.1f}cm | Vel={self.arm_speed_pct}%"
                )
            except Exception as e:
                self.get_logger().warn(f"Aviso leyendo YAML ({e}), usando valores predeterminados.")

    def save_configuration(self):
        """Guarda la distancia del gripper y la velocidad en el archivo YAML."""
        try:
            data = {}
            if os.path.exists(CONFIG_FILE_PATH):
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
            if "safety_envelope" not in data:
                data["safety_envelope"] = {}
            data["safety_envelope"]["dabai_extra_advance_cm"] = round(float(self.distancia_gripper_cm), 1)
            data["safety_envelope"]["arm_speed_pct"] = int(self.arm_speed_pct)
            with open(CONFIG_FILE_PATH, "w", encoding="utf-8") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f"Guardado en YAML: DistanciaGripper=+{self.distancia_gripper_cm:.1f}cm")
            self.status_message = f"GUARDADO EN YAML: Distancia Extra=+{self.distancia_gripper_cm:.1f}cm | Vel={self.arm_speed_pct}%"
        except Exception as e:
            self.get_logger().error(f"Error guardando en YAML: {e}")

    # --------------------------------------------------------------------------
    # CALLBACKS DE SENSORES
    # --------------------------------------------------------------------------
    def head_color_callback(self, msg: Image):
        try:
            h, w = msg.height, msg.width
            if msg.encoding in ["bgr8", "rgb8"]:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                return
            with self.frame_lock:
                self.head_color_frame = img
        except Exception:
            pass

    def wrist_cam_callback(self, msg: Image):
        try:
            h, w = msg.height, msg.width
            if msg.encoding in ["bgr8", "rgb8"]:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                if msg.encoding == "rgb8":
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                return
            with self.frame_lock:
                self.wrist_color_frame = img
                self.wrist_frame_timestamp = time.time()
                self.wrist_frame_seq += 1
        except Exception:
            pass

    def arm_status_callback(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.arm_is_moving = bool(payload.get("is_moving", False))
            self.gripper_opening_mm = float(payload.get("gripper_opening_mm", self.gripper_opening_mm))
            joints = payload.get("joint_positions_deg", [])
            if len(joints) >= 6:
                self.arm_joints_deg = [float(v) for v in joints[:6]]
        except Exception:
            pass

    def pan_tilt_callback(self, msg: JointState):
        pass

    # --------------------------------------------------------------------------
    # DETECCIÓN SIMPLE Y ROBUSTA DEL OBJETO
    # --------------------------------------------------------------------------
    def detect_object_in_dabai(self, frame: np.ndarray) -> Optional[dict]:
        """
        Detección rápida y limpia de la lata/botella usando YOLOv8 o color HSV.
        Retorna el bounding box, centroide y ancho del cilindro.
        """
        fh, fw = frame.shape[:2]
        best_bbox = None
        best_score = 0.0

        # 1. Inferencia YOLOv8
        if self.yolo_model is not None:
            try:
                results = self.yolo_model.predict(
                    source=frame,
                    imgsz=320,
                    conf=0.28,
                    classes=[39, 41, 75],  # bottle, cup, vase
                    verbose=False,
                    device="cpu",
                )
                for r in results:
                    boxes = r.boxes
                    if boxes is None or len(boxes) == 0:
                        continue
                    for box in boxes:
                        score = float(box.conf[0])
                        xyxy = box.xyxy[0].cpu().numpy()
                        bx1, by1, bx2, by2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
                        bw = bx2 - bx1
                        bh = by2 - by1
                        if bw >= 25 and bh >= 40:
                            if score > best_score:
                                best_score = score
                                best_bbox = (bx1, by1, bx2, by2)
            except Exception:
                pass

        # 2. Detección Fallback por Color HSV (si YOLO no lo ve o no está cargado)
        if best_bbox is None:
            try:
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                # Máscara para colores de tubo de Pringles (rojo / naranja / púrpura / dorado)
                m1 = cv2.inRange(hsv, np.array([0, 80, 70]), np.array([12, 255, 255]))
                m2 = cv2.inRange(hsv, np.array([168, 80, 70]), np.array([180, 255, 255]))
                m3 = cv2.inRange(hsv, np.array([125, 70, 70]), np.array([160, 255, 255]))
                mask = cv2.bitwise_or(m1, m2)
                mask = cv2.bitwise_or(mask, m3)
                # Descartar zona de los dedos en la parte baja
                mask[380:, :] = 0
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cnts:
                    area = cv2.contourArea(c)
                    if area > 800:
                        x, y, w, h = cv2.boundingRect(c)
                        if w >= 25 and h >= 35:
                            score = area / (fw * fh)
                            if score > best_score:
                                best_score = score
                                best_bbox = (x, y, x + w, y + h)
            except Exception:
                pass

        if best_bbox is None:
            return None

        bx1, by1, bx2, by2 = best_bbox
        cx = (bx1 + bx2) / 2.0
        cy = (by1 + by2) / 2.0
        cw = float(max(25, bx2 - bx1))

        return {"bbox": best_bbox, "cx": cx, "cy": cy, "width": cw}

    def _vision_worker(self):
        """Hilo dedicado para procesar la visión sin retrasar el control ROS."""
        last_seq = -1
        while self._running and rclpy.ok():
            with self.frame_lock:
                frame = self.wrist_color_frame.copy() if self.wrist_color_frame is not None else None
                seq = self.wrist_frame_seq
                frame_ts = self.wrist_frame_timestamp

            if frame is None or seq == last_seq:
                time.sleep(0.01)
                continue

            last_seq = seq
            t0 = time.time()
            det = self.detect_object_in_dabai(frame)
            dt = max(0.001, time.time() - t0)
            self.det_fps = 0.8 * self.det_fps + 0.2 * (1.0 / dt) if self.det_fps > 0 else (1.0 / dt)

            now = time.time()
            settling = self.is_arm_settling(now)
            captured_after_stop = frame_ts > (self.last_cmd_sent_time + self.current_move_duration_s + 0.20)

            if det is not None:
                self.target_2d_wrist = det["bbox"]
                self.last_det_time = now

                # Suavizado cuando el brazo está quieto
                if captured_after_stop and not settling:
                    if self.fresh_post_stop_dets == 0:
                        self.dabai_cx = float(det["cx"])
                        self.dabai_cy = float(det["cy"])
                        self.dabai_width_px = float(det["width"])
                    else:
                        self.dabai_cx = 0.3 * self.dabai_cx + 0.7 * float(det["cx"])
                        self.dabai_cy = 0.3 * self.dabai_cy + 0.7 * float(det["cy"])
                        self.dabai_width_px = 0.3 * self.dabai_width_px + 0.7 * float(det["width"])
                    self.fresh_post_stop_dets += 1
                else:
                    self.dabai_cx = float(det["cx"])
                    self.dabai_cy = float(det["cy"])
                    self.dabai_width_px = float(det["width"])

                # Distancia estimada desde la lente de la DaBai hasta la lata
                self.dist_to_can_m = float(
                    np.clip((OBJECT_DIAMETER_M * DABAI_FOCAL_PX) / self.dabai_width_px, 0.15, 0.65)
                )

                # Error lateral respecto al centro óptico exacto (320 px)
                err_px = self.dabai_cx - DABAI_CENTER_X_PX
                self.lateral_err_m = -float((err_px / DABAI_FOCAL_PX) * self.dist_to_can_m)

                # Distancia restante para que la lata quede entre los dedos
                self.dist_fwd_remaining_m = float(max(0.0, self.dist_to_can_m - DABAI_GRASP_ZONE_DIST_M))
            else:
                if now - self.last_det_time > 1.5:
                    self.target_2d_wrist = None
                    self.dabai_width_px = 0.0

    # --------------------------------------------------------------------------
    # COMANDOS DE MOVIMIENTO DEL BRAZO
    # --------------------------------------------------------------------------
    def say(self, text: str):
        msg = String()
        msg.data = text
        self.pub_say.publish(msg)
        self.status_message = text
        print(f"[Robot]: \"{text}\"")

    def send_arm_joint_goal(self, joints_deg: List[float], duration_s: float = 2.0, gripper_mm: Optional[float] = None) -> bool:
        clamped_deg = list(joints_deg[:6])
        clamped_deg[1] = min(clamped_deg[1], 194.5)

        safe, reason = is_in_safe_joint_envelope(clamped_deg)
        if not safe:
            self.get_logger().error(f"[SEGURIDAD] Comando bloqueado: {reason}")
            self.status_message = f"BLOQUEO SEGURIDAD: {reason}"
            return False

        # Escalar duración según velocidad deseada (40% normal)
        spd_scale = 75.0 / max(15.0, float(self.arm_speed_pct))
        scaled_dur = float(np.clip(duration_s * spd_scale, 0.8, 9.5))

        self.last_cmd_sent_time = time.time()
        self.current_move_duration_s = scaled_dur
        self.fresh_post_stop_dets = 0

        if self.dry_run:
            self.get_logger().info(f"[DRY-RUN] joints={clamped_deg}, dur={scaled_dur:.2f}s, pinza={gripper_mm}mm")
            return True

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [math.radians(d) for d in clamped_deg]
        if gripper_mm is not None:
            js.name.append("gripper")
            js.position.append(float(gripper_mm))
        js.velocity = [scaled_dur]
        self.pub_joint_cmd.publish(js)
        return True

    def mover_a_home(self):
        self.fase_actual = self.FASE_HOME
        self.send_arm_joint_goal(self.pose_home, duration_s=2.5, gripper_mm=GRIPPER_MAX_OPEN_MM)
        self.say("Moving to Home position.")

    def mover_a_pre_grasp(self):
        self.fase_actual = self.FASE_PREGRASP
        self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=2.2, gripper_mm=GRIPPER_MAX_OPEN_MM)
        self.say("Moving to Pre-grasp with gripper fully open.")

    def set_arm_torque(self, enable: bool):
        msg = Bool()
        msg.data = bool(enable)
        self.pub_enable.publish(msg)

    # --------------------------------------------------------------------------
    # SECUENCIA DE MANIPULACIÓN AUTÓNOMA DETERMINISTA
    # --------------------------------------------------------------------------
    def start_autonomous_grasp(self):
        """Inicia la rutina autónoma de agarre paso a paso."""
        self.align_attempts = 0
        self.total_forward_advanced_m = 0.0
        self.auto_state = self.AUTO_PASO_1_MOVER_PREGRASP
        self.step_start_time = time.time()
        self.mover_a_pre_grasp()

    def update_autonomous_state_machine(self, now: float):
        if self.auto_state == self.AUTO_IDLE:
            return

        # PASO 1: LLEGAR A PRE-GRASP CON LA PINZA TOTALMENTE ABIERTA (105 MM)
        if self.auto_state == self.AUTO_PASO_1_MOVER_PREGRASP:
            if not self.is_arm_settling(now) and (now - self.step_start_time > 2.5):
                self.auto_state = self.AUTO_PASO_2_ALINEAR_LATERAL
                self.step_start_time = now
                self.fresh_post_stop_dets = 0
                self.say("Pre-grasp reached. Checking alignment with DaBai camera.")

        # PASO 2: ALINEAR CON EL CENTRO DEL CILINDRO SI ESTÁ DESCENTRADO
        elif self.auto_state == self.AUTO_PASO_2_ALINEAR_LATERAL:
            if self.is_arm_settling(now):
                self.status_message = "PASO 2: Esperando estabilizacion del brazo..."
                return

            if self.fresh_post_stop_dets < 2:
                self.status_message = "PASO 2: Leyendo imagen de la camara..."
                if now - self.step_start_time > 6.0 and self.target_2d_wrist is None:
                    self.say("Target object not found. Returning to idle.")
                    self.auto_state = self.AUTO_IDLE
                return

            if self.target_2d_wrist is None:
                self.status_message = "PASO 2: Buscando lata de Pringles frente al gripper..."
                return

            err_px = self.dabai_cx - DABAI_CENTER_X_PX

            # Tolerancia de alineación: +-20 px (~6 mm de margen)
            if abs(err_px) <= 20.0 or self.align_attempts >= 3:
                self.joints_aligned = list(self.arm_joints_deg[:6])
                self.auto_state = self.AUTO_PASO_3_AVANZAR_RECTO
                self.step_start_time = now
                self.say("Object centered. Advancing straight toward object.")
                return

            # Corrección lateral suave
            step_lat_m = float(np.clip(self.lateral_err_m, -0.045, 0.045))
            j_aligned, ok = self.kinematics_solver.move_tool_line(
                list(self.arm_joints_deg[:6]),
                delta_forward_m=0.0,
                delta_lateral_m=step_lat_m,
                delta_up_m=0.0,
                reference_joints_deg=self.pose_pre_grasp,
            )
            self.align_attempts += 1
            if ok:
                self.get_logger().info(f"[Alinear #{self.align_attempts}] Correccion lateral: {step_lat_m*100:+.1f}cm")
                self.send_arm_joint_goal(j_aligned, duration_s=1.2, gripper_mm=GRIPPER_MAX_OPEN_MM)
            else:
                self.auto_state = self.AUTO_PASO_3_AVANZAR_RECTO

        # PASO 3: AVANZAR EN LÍNEA RECTA PURA HACIA ADELANTE
        elif self.auto_state == self.AUTO_PASO_3_AVANZAR_RECTO:
            if self.is_arm_settling(now):
                self.status_message = "PASO 3: Avanzando en linea recta hacia la lata..."
                return

            # Calcular distancia total de avance: Distancia restante + Ajuste del usuario
            fwd_target_m = self.dist_fwd_remaining_m + (self.distancia_gripper_cm / 100.0)
            fwd_clamped_m = float(np.clip(fwd_target_m, 0.04, 0.35))

            self.get_logger().info(
                f"[Avance Recto] Distancia restante={self.dist_fwd_remaining_m*100:.1f}cm + "
                f"Ajuste={self.distancia_gripper_cm:.1f}cm -> Total={fwd_clamped_m*100:.1f}cm"
            )

            current_joints = list(self.arm_joints_deg[:6])
            j_fwd, ok = self.kinematics_solver.move_tool_line(
                current_joints,
                delta_forward_m=fwd_clamped_m,
                delta_lateral_m=0.0,  # CERO desvío lateral: 100% recto
                delta_up_m=0.0,
                reference_joints_deg=self.pose_pre_grasp,
            )

            if ok:
                self.joints_grasp = j_fwd
                self.total_forward_advanced_m = fwd_clamped_m
                self.auto_state = self.AUTO_PASO_4_CERRAR_PINZA
                self.step_start_time = now
                self.send_arm_joint_goal(j_fwd, duration_s=1.8, gripper_mm=GRIPPER_MAX_OPEN_MM)
                self.status_message = f"PASO 3: Avanzando +{fwd_clamped_m*100:.1f}cm directo a la lata..."
            else:
                self.say("Straight forward motion exceeds safe workspace. Stopping.")
                self.auto_state = self.AUTO_IDLE

        # PASO 4: CERRAR PINZA (50 MM) Y ELEVAR +3.0 CM
        elif self.auto_state == self.AUTO_PASO_4_CERRAR_PINZA:
            if self.is_arm_settling(now):
                self.status_message = "PASO 4: Llegando a posicion de agarre..."
                return

            elapsed = now - self.step_start_time
            if elapsed < 1.0:
                # 1. Enviar comando de cierre de pinza a 50 mm
                g_msg = Float32()
                g_msg.data = float(GRIPPER_CLOSE_GRASP_MM)
                self.pub_gripper_cmd.publish(g_msg)
                self.status_message = "PASO 4: Cerrando pinza sobre la lata..."
                return

            if elapsed >= 1.4:
                # 2. Despegar la lata +3.0 cm de la mesa
                current_joints = list(self.arm_joints_deg[:6])
                j_lift, ok_lift = self.kinematics_solver.move_tool_line(
                    current_joints,
                    delta_forward_m=0.0,
                    delta_lateral_m=0.0,
                    delta_up_m=0.030,
                    reference_joints_deg=self.pose_pre_grasp,
                )
                self.joints_lift = j_lift if ok_lift else current_joints
                self.send_arm_joint_goal(self.joints_lift, duration_s=1.2, gripper_mm=GRIPPER_CLOSE_GRASP_MM)

                self.auto_state = self.AUTO_PASO_5_RETRAER_PREGRASP
                self.step_start_time = now
                self.say("Object grasped. Retracting back to pre-grasp.")

        # PASO 5: RETROCESO EN LÍNEA RECTA A PRE-GRASP
        elif self.auto_state == self.AUTO_PASO_5_RETRAER_PREGRASP:
            if self.is_arm_settling(now):
                self.status_message = "PASO 5: Despegando y retrayendo..."
                return

            if now - self.step_start_time > 1.2:
                # Volver suavemente a la pose Pre-Grasp sosteniendo la lata
                self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=1.8, gripper_mm=GRIPPER_CLOSE_GRASP_MM)
                self.auto_state = self.AUTO_SUCCESS
                self.step_start_time = now
                self.say("Grasp complete! Object safely held.")

        elif self.auto_state == self.AUTO_SUCCESS:
            if now - self.step_start_time > 3.0:
                self.auto_state = self.AUTO_IDLE

    # --------------------------------------------------------------------------
    # RENDERIZADO VISUAL DEL HUD
    # --------------------------------------------------------------------------
    def draw_hud(self) -> np.ndarray:
        canvas_w, canvas_h = 1280, 600
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        feed_w, feed_h = 620, 420

        with self.frame_lock:
            h_raw = self.head_color_frame.copy() if self.head_color_frame is not None else None
            w_raw = self.wrist_color_frame.copy() if self.wrist_color_frame is not None else None

        # PANEL IZQUIERDO: Cámara Principal (RealSense D435)
        if h_raw is not None:
            h_img = cv2.resize(h_raw, (feed_w, feed_h))
            canvas[40:40 + feed_h, 15:15 + feed_w] = h_img
        else:
            cv2.rectangle(canvas, (15, 40), (15 + feed_w, 40 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "CAMARA PRINCIPAL (REALSENSE D435)", (130, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(canvas, "1. CAMARA PRINCIPAL (VISTA GENERAL)", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 210, 230), 1)

        # PANEL DERECHO: Cámara de Muñeca Orbbec DaBai (Centrada, 8cm Atrás)
        if w_raw is not None:
            w_img = cv2.resize(w_raw, (feed_w, feed_h))
            orig_h, orig_w = w_raw.shape[:2]
            sx, sy = feed_w / orig_w, feed_h / orig_h

            # Línea central calibrada de la pinza (x = 320 px)
            center_x_scaled = int(DABAI_CENTER_X_PX * sx)
            wcy = int(feed_h / 2)
            cv2.line(w_img, (center_x_scaled, 20), (center_x_scaled, feed_h - 20), (0, 255, 100), 2)
            cv2.drawMarker(w_img, (center_x_scaled, wcy), (0, 255, 100), cv2.MARKER_CROSS, 20, 2)

            # Corredor de tolerancia central (+-20 px)
            tol_px_scaled = int(20.0 * sx)
            cv2.line(w_img, (center_x_scaled - tol_px_scaled, 30), (center_x_scaled - tol_px_scaled, feed_h - 30), (60, 180, 60), 1)
            cv2.line(w_img, (center_x_scaled + tol_px_scaled, 30), (center_x_scaled + tol_px_scaled, feed_h - 30), (60, 180, 60), 1)

            cv2.putText(
                w_img,
                f"CENTRO DEL GRIPPER ({self.det_fps:.0f} FPS | Camara 8cm Atras)",
                (center_x_scaled - 140, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (0, 255, 100),
                1,
            )

            # Bounding box del objeto
            if self.target_2d_wrist is not None:
                x1, y1, x2, y2 = self.target_2d_wrist
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                obj_cx = int(self.dabai_cx * sx)
                obj_cy = int(self.dabai_cy * sy)

                err_px = self.dabai_cx - DABAI_CENTER_X_PX
                is_centered = abs(err_px) <= 20.0
                box_col = (0, 255, 80) if is_centered else (0, 180, 255)

                cv2.rectangle(w_img, (bx1, by1), (bx2, by2), box_col, 2)
                cv2.circle(w_img, (obj_cx, obj_cy), 5, (0, 255, 0), -1)
                cv2.line(w_img, (obj_cx, obj_cy), (center_x_scaled, obj_cy), (0, 255, 255), 2)

                lat_text = f"dLeft={-self.lateral_err_m*100:.1f}cm" if self.lateral_err_m < 0 else f"dRight={self.lateral_err_m*100:.1f}cm"
                if is_centered:
                    lat_text = "CENTRADO OK"

                cv2.putText(
                    w_img,
                    f"Lata Pringles: {lat_text} | Dist={self.dist_to_can_m*100:.1f}cm",
                    (max(8, bx1), max(22, by1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    box_col,
                    2,
                )
                cv2.putText(
                    w_img,
                    f"Faltan={self.dist_fwd_remaining_m*100:.1f}cm + Extra={self.distancia_gripper_cm:.1f}cm",
                    (max(8, bx1), min(feed_h - 10, by2 + 18)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.40,
                    (255, 255, 255),
                    1,
                )

            canvas[40:40 + feed_h, 645:645 + feed_w] = w_img
        else:
            cv2.rectangle(canvas, (645, 40), (645 + feed_w, 40 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM CAMARA ORBBEC DABAI...", (710, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(
            canvas,
            f"2. CAMARA ORBBEC DABAI (Ajuste Distancia=+{self.distancia_gripper_cm:.1f}cm | Vel={self.arm_speed_pct}%)",
            (645, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 255, 140),
            1,
        )

        # PANEL INFERIOR: Telemetría y Controles
        bar_y = 468
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (12, 16, 24), -1)
        cv2.line(canvas, (0, bar_y), (canvas_w, bar_y), (60, 80, 110), 1)

        state_key = self.auto_state if self.auto_state != self.AUTO_IDLE else self.fase_actual
        state_str = self.FASE_NAMES.get(state_key, "LISTO")
        cv2.putText(canvas, f"ESTADO: {state_str}", (20, bar_y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)

        telemetry = (
            f"DaBai ({self.det_fps:.0f}FPS) | Distancia={self.dist_to_can_m*100:.1f}cm | "
            f"Faltan={self.dist_fwd_remaining_m*100:.1f}cm (+{self.distancia_gripper_cm:.1f}cm) | "
            f"Pinza={self.gripper_opening_mm:.0f}mm | {self.status_message}"
        )
        cv2.putText(canvas, telemetry, (20, bar_y + 56), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 220, 240), 1)

        shortcuts1 = (
            f"[ESPACIO]/[G] AGARRAR | [3] Pre-Grasp (105 mm) | [2] Home | [O] Abrir Pinza | [C] Cerrar Pinza | [K] GUARDAR"
        )
        shortcuts2 = (
            f"AJUSTES: Distancia Gripper: +{self.distancia_gripper_cm:.1f}cm (Teclas + / -) | "
            f"Velocidad: {self.arm_speed_pct}% (Teclas 9 / 0) | [Q] Salir"
        )
        cv2.putText(canvas, shortcuts1, (20, bar_y + 86), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (80, 255, 140), 1)
        cv2.putText(canvas, shortcuts2, (20, bar_y + 112), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 230, 255), 1)

        return canvas

    # --------------------------------------------------------------------------
    # TRACKBAR CALLBACKS
    # --------------------------------------------------------------------------
    def _on_dist_slider(self, val_mm: int):
        self.distancia_gripper_cm = float(val_mm) / 10.0

    def _on_speed_slider(self, val_pct: int):
        self.arm_speed_pct = int(np.clip(val_pct, 15, 100))

    # --------------------------------------------------------------------------
    # BUCLE PRINCIPAL
    # --------------------------------------------------------------------------
    def run(self):
        window_name = self._window_name
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 650)

        # Trackbar único de ajuste de distancia del gripper (0 mm a 150 mm)
        cv2.createTrackbar(
            "Distancia_Gripper_mm",
            window_name,
            int(round(self.distancia_gripper_cm * 10.0)),
            150,
            self._on_dist_slider,
        )
        cv2.createTrackbar(
            "Velocidad_%",
            window_name,
            int(round(self.arm_speed_pct)),
            100,
            self._on_speed_slider,
        )

        print("==========================================================================")
        print("   ROBOT @HOME - MANIPULACION AUTONOMA SIMPLIFICADA (CAMARA 8CM ATRAS)    ")
        print("--------------------------------------------------------------------------")
        print("   * [3] o [p]       : Ir a PRE-GRASP con pinza abierta al maximo (105 mm)")
        print("   * [ESPACIO] o [G] : Agarre autonomo en linea recta hacia la lata       ")
        print(f"   * Distancia_Extra : +{self.distancia_gripper_cm:.1f} cm [Slider o teclas + / -]    ")
        print(f"   * Velocidad_%     : {self.arm_speed_pct}% [Slider o teclas 9 / 0]                 ")
        print("   * [O] / [C]       : Abrir (105 mm) / Cerrar (50 mm) pinza              ")
        print("   * [K]             : Guardar configuracion en archivo YAML              ")
        print("   * [2] o [h]       : Ir a posicion HOME                                 ")
        print("   * [Q]             : Salir                                              ")
        print("==========================================================================")

        while rclpy.ok():
            now = time.time()
            self.update_autonomous_state_machine(now)

            canvas = self.draw_hud()
            cv2.imshow(window_name, canvas)

            key = cv2.waitKey(25) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                self._running = False
                break

            elif key in [ord("+"), ord("=")]:
                self.distancia_gripper_cm = min(15.0, round(self.distancia_gripper_cm + 0.5, 1))
                cv2.setTrackbarPos("Distancia_Gripper_mm", window_name, int(round(self.distancia_gripper_cm * 10.0)))
                self.status_message = f"Ajuste Distancia = +{self.distancia_gripper_cm:.1f} cm | [K] Guardar"

            elif key in [ord("-"), ord("_")]:
                self.distancia_gripper_cm = max(0.0, round(self.distancia_gripper_cm - 0.5, 1))
                cv2.setTrackbarPos("Distancia_Gripper_mm", window_name, int(round(self.distancia_gripper_cm * 10.0)))
                self.status_message = f"Ajuste Distancia = +{self.distancia_gripper_cm:.1f} cm | [K] Guardar"

            elif key in [ord("9"), ord("(")]:
                self.arm_speed_pct = max(15, self.arm_speed_pct - 5)
                cv2.setTrackbarPos("Velocidad_%", window_name, int(round(self.arm_speed_pct)))
                self.status_message = f"Velocidad = {self.arm_speed_pct}% | [K] Guardar"

            elif key in [ord("0"), ord(")")]:
                self.arm_speed_pct = min(100, self.arm_speed_pct + 5)
                cv2.setTrackbarPos("Velocidad_%", window_name, int(round(self.arm_speed_pct)))
                self.status_message = f"Velocidad = {self.arm_speed_pct}% | [K] Guardar"

            elif key in [ord("k"), ord("K")]:
                self.save_configuration()

            elif key in [ord("2"), ord("h")]:
                self.auto_state = self.AUTO_IDLE
                self.mover_a_home()

            elif key in [ord("3"), ord("p")]:
                self.auto_state = self.AUTO_IDLE
                self.mover_a_pre_grasp()

            elif key in [ord("o"), ord("O")]:
                g_msg = Float32()
                g_msg.data = float(GRIPPER_MAX_OPEN_MM)
                self.pub_gripper_cmd.publish(g_msg)
                self.status_message = f"Pinza ABIERTA al maximo ({GRIPPER_MAX_OPEN_MM:.0f} mm)."

            elif key in [ord("c"), ord("C")]:
                g_msg = Float32()
                g_msg.data = float(GRIPPER_CLOSE_GRASP_MM)
                self.pub_gripper_cmd.publish(g_msg)
                self.status_message = f"Pinza CERRADA a {GRIPPER_CLOSE_GRASP_MM:.0f} mm."

            elif key in [ord(" "), ord("g"), ord("G")]:
                self.start_autonomous_grasp()

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Manipulacion Autonoma Simplificada Robot @Home")
    parser.add_argument("--dry-run", action="store_true", help="Modo simulacion sin enviar comandos al bus CAN")
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = ManipulacionAutonomaNode(dry_run=args.dry_run)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
