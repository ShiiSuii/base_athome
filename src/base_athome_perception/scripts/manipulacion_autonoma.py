#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Robot @Home - Manipulación por Visual Servoing 6-DOF con Cámara Orbbec DaBai
#
# Mejoras clave:
#   1. Sincronización automática de reloj PC <-> Jetson al iniciar para que RViz
#      muestre siempre el brazo en su posición real en tiempo real (sin error TF).
#   2. Hilo dedicado para ROS 2 (rclpy.spin) + Hilo dedicado para detección YOLO
#      (imgsz=320 a ~17 FPS solo en la cámara DaBai), eliminando el retraso de cámara.
#   3. Control "Mover -> Detener -> Estabilizar Cámara -> Medir":
#      - Movimientos más lentos y suaves (1.20 s por paso).
#      - Espera a que el brazo termine de moverse por completo y a recibir
#        frames nuevos capturados DESPUÉS de detenerse antes de calcular el siguiente
#        paso o actualizar la posición 3D de la lata/botella en RViz.
#   4. Agarre (50 mm) al llegar y Rutina Post-Grasping (volver recto atrás a Pre-Grasp).
#
# Robot: "Robot"
# ==============================================================================

import argparse
import json
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

# Asegurar acceso a ultralytics del venv de visión
VENV_SITE_PACKAGES = "/home/athome/ros2_vision_venv/lib/python3.12/site-packages"
if os.path.exists(VENV_SITE_PACKAGES) and VENV_SITE_PACKAGES not in sys.path:
    sys.path.insert(0, VENV_SITE_PACKAGES)

from ultralytics import YOLO

# Módulo cinemático oficial del Brazo Piper (6-DOF con orientación bloqueada)
from base_athome_perception.piper_kinematics import (
    is_in_safe_joint_envelope,
    is_in_safe_cartesian_box,
    PiperKinematicsSolver,
    SAFE_CARTESIAN_BOX_M,
)

CONFIG_FILE_PATH = "/home/athome/base_athome_ws/src/base_athome_perception/config/arm_poses_and_limits.yaml"

# Posición del hombro izquierdo (arm_base_link) respecto a base_link en el URDF:
SHOULDER_IN_BASE_LINK_M = np.array([0.155, 0.175, 1.015], dtype=float)

# Apertura física máxima del gripper Piper (105 mm) y cierre de agarre firme (50 mm)
GRIPPER_MAX_OPEN_MM = 105.0
GRIPPER_CLOSE_GRASP_MM = 50.0

# Geometría óptica en vivo de la Cámara Orbbec DaBai (640x480):
# En la imagen 640x480 de la DaBai, el dedo izquierdo está en x~185 y el dedo derecho en x~360.
# El centro exacto del hueco entre los dos dedos abiertos es x = 275 px.
DABAI_GRIPPER_GAP_X_PX = 275.0
DABAI_FOCAL_PX = 485.0
OBJECT_DIAMETER_M = 0.075          # Diámetro típico botella / tubo Pringles (7.5 cm)
DABAI_GRASP_ZONE_DIST_M = 0.190    # Distancia lente DaBai -> hueco entre dedos (19.0 cm)
DABAI_ARRIVAL_WIDTH_PX = 172.0     # Ancho en píxeles cuando el objeto ya entró entre los dedos


def sync_jetson_clock():
    """
    Sincroniza el reloj de la Jetson (ubuntu@10.42.0.12) con el reloj del PC local
    para evitar errores de extrapolación TF en RViz2 (Fixed Frame = map).
    """
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
    """Nodo de manipulación guiado en lazo cerrado por la cámara Orbbec DaBai con IK 6-DOF."""

    FASE_1_RANGO = 1
    FASE_2_HOME = 2
    FASE_3_PREGRASP = 3

    # Estados de Visual Servoing con Cámara Orbbec DaBai + Rutina Post-Grasping
    AUTO_IDLE = 10
    AUTO_PASO_B_MOVER_PREGRASP = 12          # Ir a Pre-Grasp con Gripper ABIERTO AL MÁXIMO (105 mm)
    AUTO_PASO_C1_ALINEAR_DABAI = 13          # Centrar el objeto entre los dedos (x=275 px) con Cámara DaBai
    AUTO_PASO_C2_AVANZAR_DABAI = 14          # Avanzar paso a paso con Cámara DaBai hasta que entre en la pinza
    AUTO_PASO_D_AGARRE = 15                  # Cerrar pinza (50 mm) al llegar y despegar +2.5 cm
    AUTO_PASO_E_POSTGRASP_PREGRASP = 16      # RUTINA POST-GRASPING: Volver RECTO HACIA ATRÁS a Pre-Grasp
    AUTO_SUCCESS = 17

    FASE_NAMES = {
        FASE_1_RANGO: "FASE 1: RANGO DE MOVIMIENTOS SEGURO",
        FASE_2_HOME: "FASE 2: POSICION HOME DEL BRAZO",
        FASE_3_PREGRASP: "MODO PRE-GRASP (GRIPPER MAX 105 MM) - CAMARA DABAI LISTA PARA IK",
        AUTO_IDLE: "LISTO: PRESIONE [ESPACIO] O [G] PARA AGARRAR GUIADO POR CAMARA DABAI",
        AUTO_PASO_B_MOVER_PREGRASP: "PASO 1: YENDO A PRE-GRASP CON GRIPPER AL MAXIMO (105 MM)",
        AUTO_PASO_C1_ALINEAR_DABAI: "PASO 2 (CAMARA DABAI): ALINEANDO PAUSADO CON EL OBJETO",
        AUTO_PASO_C2_AVANZAR_DABAI: "PASO 3 (CAMARA DABAI): AVANZANDO PAUSADO HASTA EL OBJETO",
        AUTO_PASO_D_AGARRE: "PASO 4: OBJETO EN LA PINZA -> CERRANDO GRIPPER (50 MM) Y ELEVANDO",
        AUTO_PASO_E_POSTGRASP_PREGRASP: "PASO 5 (POST-GRASPING): VOLVIENDO RECTO ATRAS A PRE-GRASP",
        AUTO_SUCCESS: "POST-GRASPING COMPLETADO: OBJETO EN POSICION PRE-GRASP ([2] Ir a Home)",
    }

    def __init__(self, dry_run: bool = False):
        super().__init__("manipulacion_autonoma_node")
        self.dry_run = dry_run
        self.get_logger().info("Sincronizando reloj con Jetson para TF en RViz2...")
        sync_jetson_clock()

        self.load_configuration()

        # QoS de profundidad 1 en todas las entradas de sensores para CERO atraso de cola
        qos_latest = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_status = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub_head_color = self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.head_color_callback, qos_latest
        )
        self.sub_wrist_cam = self.create_subscription(
            Image, "/arm_camera/image_raw", self.wrist_cam_callback, qos_latest
        )
        self.sub_arm_status = self.create_subscription(
            String, "/arm/status", self.arm_status_callback, qos_status
        )
        self.sub_pan_tilt = self.create_subscription(
            JointState, "/pan_tilt/joint_states", self.pan_tilt_callback, qos_latest
        )

        # Publicadores ROS 2
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_joint_cmd = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_gripper_cmd = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_pan_tilt_cmd = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)
        self.pub_enable = self.create_publisher(Bool, "/arm/enable", 5)
        self.pub_rviz_markers = self.create_publisher(MarkerArray, "/arm_camera/object_markers", 10)

        self.cli_set_enable = self.create_client(SetBool, "/arm/set_enable")

        # Buffers de imagen y sincronización temporal
        self.frame_lock = threading.Lock()
        self.head_color_frame: Optional[np.ndarray] = None
        self.wrist_color_frame: Optional[np.ndarray] = None
        self.wrist_frame_timestamp: float = 0.0
        self.wrist_frame_seq: int = 0

        # Ángulos del cabezal Pan-Tilt
        self.head_pan_rad = 0.0
        self.head_tilt_rad = 0.0
        self.pan_tilt_inverted = True

        # Modo de Enseñanza con la Mano
        self.is_teaching_mode = False
        self.teaching_target: Optional[str] = None

        # Telemetría del brazo Piper
        self.arm_is_moving = False
        self.gripper_opening_mm = GRIPPER_MAX_OPEN_MM
        self.arm_xyz_mm = [0.0, 0.0, 0.0]
        self.arm_joints_deg = list(self.pose_home)

        # Control de tiempos de movimiento del brazo (para saber cuándo está quieto y estable)
        self.last_cmd_sent_time: float = 0.0
        self.current_move_duration_s: float = 0.0
        self.fresh_post_stop_dets: int = 0

        # Solucionador cinemático 6-DOF oficial del brazo Piper
        self.kinematics_solver = PiperKinematicsSolver()

        # Modelo YOLO rápido (imgsz=320 en hilo dedicado para ~17 FPS sin bloquear ROS ni GUI)
        self.get_logger().info("Cargando modelo YOLO rapido para hilo dedicado de camara DaBai...")
        try:
            self.yolo_model = YOLO("yolov8s-worldv2.pt")
            self.yolo_model.set_classes(["bottle", "pringles can", "can", "cup", "cylinder"])
        except Exception:
            self.yolo_model = YOLO("yolov8n.pt")

        # Estado y mediciones en vivo de la Cámara Orbbec DaBai
        self.fase_actual = self.FASE_2_HOME
        self.auto_state = self.AUTO_IDLE
        self.step_start_time = time.time()

        self.target_2d_head: Optional[Tuple[int, int, int, int]] = None
        self.target_2d_wrist: Optional[Tuple[int, int, int, int]] = None
        self.target_3d_arm: Optional[Tuple[float, float, float]] = None

        self.dabai_wcx: float = DABAI_GRIPPER_GAP_X_PX
        self.dabai_wcy: float = 240.0
        self.dabai_bw: float = 0.0
        self.dabai_y2: float = 0.0
        self.dabai_err_px: float = 0.0
        self.wrist_dy_m: float = 0.0      # Desplazamiento lateral (+izq, -der) medido por la DaBai
        self.wrist_dfwd_m: float = 0.0    # Distancia frontal restante hasta quedar entre los dedos
        self.dist_tcp_to_obj_cm: float = 0.0
        self.is_in_reach: bool = True
        self.last_wrist_det_time: float = 0.0
        self.last_marker_pub_time: float = 0.0
        self.det_fps: float = 0.0

        # Estado del Visual Servoing paso a paso con la cámara DaBai
        self.servo_joints: List[float] = list(self.pose_pre_grasp)
        self.joints_aligned: Optional[List[float]] = None
        self.joints_grasp: Optional[List[float]] = None
        self.joints_lift: Optional[List[float]] = None
        self.joints_retract_straight: Optional[List[float]] = None
        self.total_forward_advanced_m: float = 0.0
        self.last_servo_step_time: float = 0.0
        self.align_steps_count: int = 0
        self.advance_steps_count: int = 0
        self.last_valid_bw: float = 0.0

        self._lift_sent = False
        self._retract_line_sent = False
        self._retract_pregrasp_sent = False

        self.status_message = (
            "Guiado por Camara DaBai | [3] Pre-Grasp (105mm) | [ESPACIO]/[G] Alinear, Avanzar y Agarrar"
        )

        # Lanzar hilo dedicado de ROS 2 spin y hilo dedicado de inferencia de cámara DaBai
        self._running = True
        self._ros_spin_thread = threading.Thread(target=self._ros_spin_worker, daemon=True)
        self._ros_spin_thread.start()
        self._vision_thread = threading.Thread(target=self._vision_worker, daemon=True)
        self._vision_thread.start()

    def _ros_spin_worker(self):
        """Procesa todos los callbacks de ROS 2 en tiempo real sin bloqueo."""
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.005)
            except Exception:
                time.sleep(0.005)

    def is_arm_settling(self, now: Optional[float] = None) -> bool:
        """
        Indica si el brazo está actualmente en movimiento o estabilizándose tras un comando.
        Se da un margen extra de 0.45 s después de la duración del movimiento para que la
        cámara DaBai capture una imagen completamente estática.
        """
        t = time.time() if now is None else now
        move_Busy = (t - self.last_cmd_sent_time) < (self.current_move_duration_s + 0.45)
        return bool(self.arm_is_moving or move_Busy)

    # --------------------------------------------------------------------------
    # CONFIGURACIÓN DE POSES (HOME Y PRE-GRASP)
    # --------------------------------------------------------------------------
    def load_configuration(self):
        """Carga las poses Home y Pre-Grasp desde YAML."""
        self.pose_home = [72.09, 176.5, -155.15, 102.62, -31.51, -99.91]
        self.pose_pre_grasp = [42.9, 180.83, -89.01, 60.57, -74.02, -96.89]

        if os.path.exists(CONFIG_FILE_PATH):
            try:
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                    poses = data.get("named_poses", {})
                    if "home" in poses:
                        self.pose_home = list(poses["home"].get("joints_deg", self.pose_home))
                    if "pre_grasp" in poses:
                        self.pose_pre_grasp = list(poses["pre_grasp"].get("joints_deg", self.pose_pre_grasp))
                self.get_logger().info(f"Poses cargadas desde YAML. Pre-Grasp: {self.pose_pre_grasp}")
            except Exception as e:
                self.get_logger().warn(f"Error leyendo YAML ({e}), usando poses por defecto.")

    def save_named_pose(self, pose_name: str, joints_deg: List[float], gripper_mm: float):
        """Guarda la pose enseñada (home o pre_grasp) en YAML."""
        try:
            data = {}
            if os.path.exists(CONFIG_FILE_PATH):
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}

            if "named_poses" not in data:
                data["named_poses"] = {}

            saved_gripper_mm = GRIPPER_MAX_OPEN_MM if pose_name == "pre_grasp" else float(gripper_mm)
            data["named_poses"][pose_name] = {
                "joints_deg": [round(float(v), 2) for v in joints_deg[:6]],
                "gripper_mm": round(saved_gripper_mm, 1),
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
    # CALLBACKS DE SENSORES Y TELEMETRÍA (BAJA LATENCIA)
    # --------------------------------------------------------------------------
    def head_color_callback(self, msg: Image):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            with self.frame_lock:
                self.head_color_frame = frame
        except Exception:
            pass

    def wrist_cam_callback(self, msg: Image):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            with self.frame_lock:
                self.wrist_color_frame = frame
                self.wrist_frame_timestamp = time.time()
                self.wrist_frame_seq += 1
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
    # HILO DEDICADO DE DETECCIÓN RÁPIDA EN CÁMARA ORBBEC DABAI (~17 FPS)
    # --------------------------------------------------------------------------
    def detect_object_in_dabai(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int, float]]:
        """
        Detecta la botella o tubo Pringles en la imagen de la cámara Orbbec DaBai (640x480)
        usando imgsz=320 para máxima velocidad (~55 ms en CPU), descartando los dedos negros abajo.
        """
        results = self.yolo_model(frame, verbose=False, conf=0.14, imgsz=320)
        candidates = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                cls_name = self.yolo_model.names.get(cls_id, "").lower()
                conf = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                bw, bh = max(1, x2 - x1), max(1, y2 - y1)
                aspect = bh / bw

                # Ignorar cajas que estén únicamente en la zona inferior de los dedos negros (y1 > 285)
                if y1 > 285:
                    continue
                if bw < 28 or bh < 60:
                    continue

                if any(k in cls_name for k in ["bottle", "pringles", "can", "cup", "cylinder"]):
                    score = conf * (1.3 if aspect > 1.1 else 1.0)
                    candidates.append((x1, y1, x2, y2, score))

        if candidates:
            return max(candidates, key=lambda c: c[4])

        # Respaldo rápido por color (rojo / morado de Pringles) ignorando la franja inferior de los dedos
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_red1 = cv2.inRange(hsv, np.array([0, 95, 60]), np.array([12, 255, 255]))
        mask_red2 = cv2.inRange(hsv, np.array([165, 95, 60]), np.array([180, 255, 255]))
        mask_purple = cv2.inRange(hsv, np.array([115, 65, 55]), np.array([164, 255, 255]))
        mask = mask_red1 | mask_red2 | mask_purple
        mask[330:, :] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_cnt = None
        best_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1400:
                x, y, w, h = cv2.boundingRect(cnt)
                if y < 265 and 1.05 <= (h / max(1, w)) <= 4.8 and area > best_area:
                    best_area = area
                    best_cnt = (x, y, x + w, y + h, 0.55)

        return best_cnt

    def _vision_worker(self):
        """
        Hilo dedicado a procesar siempre el frame más reciente de la cámara DaBai.
        Solo actualiza la posición 3D en RViz y las mediciones de servoing cuando el brazo
        NO se está moviendo, evitando que la lata/botella se desplace en RViz mientras el brazo viaja.
        """
        last_processed_seq = -1
        while self._running and rclpy.ok():
            with self.frame_lock:
                frame = self.wrist_color_frame.copy() if self.wrist_color_frame is not None else None
                frame_ts = self.wrist_frame_timestamp
                frame_seq = self.wrist_frame_seq
                joints_at_capture = list(self.arm_joints_deg[:6])

            if frame is None or frame_seq == last_processed_seq:
                time.sleep(0.01)
                continue

            last_processed_seq = frame_seq
            t0 = time.time()
            det_w = self.detect_object_in_dabai(frame)
            dt = max(0.001, time.time() - t0)
            self.det_fps = 0.7 * self.det_fps + 0.3 * (1.0 / dt) if self.det_fps > 0 else (1.0 / dt)

            now = time.time()
            settling_now = self.is_arm_settling(now)
            captured_after_stop = frame_ts > (self.last_cmd_sent_time + self.current_move_duration_s + 0.25)

            if det_w is not None:
                wx1, wy1, wx2, wy2, _ = det_w
                self.target_2d_wrist = (wx1, wy1, wx2, wy2)
                self.last_wrist_det_time = now

                raw_wcx = (wx1 + wx2) / 2.0
                raw_wcy = (wy1 + wy2) / 2.0
                raw_bw = float(max(30, wx2 - wx1))
                self.dabai_y2 = float(wy2)

                # Si es el primer frame limpio tras terminar un movimiento, tomar la lectura directa
                # sin mezclarla con la posición anterior al movimiento
                if captured_after_stop and not settling_now:
                    if self.fresh_post_stop_dets == 0 or self.dabai_bw <= 1.0:
                        self.dabai_wcx = raw_wcx
                        self.dabai_wcy = raw_wcy
                        self.dabai_bw = raw_bw
                    else:
                        self.dabai_wcx = 0.35 * self.dabai_wcx + 0.65 * raw_wcx
                        self.dabai_wcy = 0.35 * self.dabai_wcy + 0.65 * raw_wcy
                        self.dabai_bw = 0.35 * self.dabai_bw + 0.65 * raw_bw
                    self.fresh_post_stop_dets += 1
                else:
                    # Durante el movimiento actualizamos el recuadro visual 2D sin contaminar el filtro post-parada
                    self.dabai_wcx = raw_wcx
                    self.dabai_wcy = raw_wcy
                    self.dabai_bw = raw_bw
                    self.fresh_post_stop_dets = 0

                self.last_valid_bw = self.dabai_bw
                self.dabai_err_px = self.dabai_wcx - DABAI_GRIPPER_GAP_X_PX

                # Distancia estimada desde el lente de la DaBai hasta el objeto:
                z_cam_m = float(np.clip((OBJECT_DIAMETER_M * DABAI_FOCAL_PX) / self.dabai_bw, 0.14, 0.52))
                self.wrist_dfwd_m = float(max(0.0, z_cam_m - DABAI_GRASP_ZONE_DIST_M))

                # Desplazamiento lateral necesario (+izquierda, -derecha) para centrar en x = 275 px:
                dy_m = -((self.dabai_wcx - DABAI_GRIPPER_GAP_X_PX) / DABAI_FOCAL_PX) * z_cam_m
                if wx1 <= 6 and self.dabai_wcx < DABAI_GRIPPER_GAP_X_PX:
                    dy_m += 0.015
                elif wx2 >= 634 and self.dabai_wcx > DABAI_GRIPPER_GAP_X_PX:
                    dy_m -= 0.015
                self.wrist_dy_m = float(np.clip(dy_m, -0.14, 0.14))

                # IMPORTANTE PARA RVIZ:
                # Solo recalcular la coordenada 3D de la botella/lata en RViz cuando el brazo está
                # QUIETO y ESTABLE (o si aún no existe). Mientras el brazo se mueve, la lata
                # permanece fija en su lugar real sobre la mesa en RViz!
                if (not settling_now and captured_after_stop) or self.target_3d_arm is None:
                    q_cap_rad = np.array([math.radians(d) for d in joints_at_capture[:6]], dtype=float)
                    cur_pos_mm, cur_R = self.kinematics_solver.forward_kinematics(q_cap_rad)
                    tool_z = cur_R[:, 2]
                    fwd_xy = np.array([tool_z[0], tool_z[1], 0.0], dtype=float)
                    fwd_norm = float(np.linalg.norm(fwd_xy))
                    fwd_unit = (fwd_xy / fwd_norm) if fwd_norm > 1e-4 else np.array([1.0, 0.0, 0.0])
                    left_unit = np.array([-fwd_unit[1], fwd_unit[0], 0.0], dtype=float)

                    obj_pos_m = (cur_pos_mm / 1000.0) + (self.wrist_dfwd_m * fwd_unit) + (self.wrist_dy_m * left_unit)
                    self.target_3d_arm = (float(obj_pos_m[0]), float(obj_pos_m[1]), float(obj_pos_m[2]))
                    in_box, _ = is_in_safe_cartesian_box(*self.target_3d_arm)
                    self.is_in_reach = in_box
            else:
                if now - self.last_wrist_det_time > 1.2:
                    self.target_2d_wrist = None
                    self.dabai_bw = 0.0

    def publish_rviz_3d_markers(self):
        """Publica en RViz (/arm_camera/object_markers) la posición del objeto estática sobre la mesa."""
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

        obj_sh_m = np.array(self.target_3d_arm, dtype=float)
        obj_base_m = SHOULDER_IN_BASE_LINK_M + obj_sh_m

        q_rad = [math.radians(d) for d in self.arm_joints_deg[:6]]
        tcp_sh_mm, _ = self.kinematics_solver.forward_kinematics(q_rad)
        tcp_base_m = SHOULDER_IN_BASE_LINK_M + (tcp_sh_mm / 1000.0)

        self.dist_tcp_to_obj_cm = float(np.linalg.norm(obj_base_m - tcp_base_m) * 100.0)
        col = ColorRGBA(r=0.10, g=0.95, b=0.30, a=0.90)

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
            f"CAMARA DABAI | Faltan: {self.wrist_dfwd_m*100:.1f} cm | "
            f"dY: {self.wrist_dy_m*100:+.1f} cm"
        )
        txt.lifetime = Duration(sec=0, nanosec=500000000)
        ma.markers.append(txt)

        self.pub_rviz_markers.publish(ma)

    # --------------------------------------------------------------------------
    # COMANDOS DE MOVIMIENTO DEL BRAZO PIPER
    # --------------------------------------------------------------------------
    def say(self, text: str):
        clean_ascii = text.encode("ascii", "ignore").decode("ascii")
        msg = String()
        msg.data = clean_ascii
        self.pub_say.publish(msg)
        self.status_message = clean_ascii
        print(f"[Robot]: \"{clean_ascii}\"")

    def send_arm_joint_goal(self, joints_deg: List[float], duration_s: float = 2.0, gripper_mm: Optional[float] = None) -> bool:
        clamped_deg = list(joints_deg[:6])
        clamped_deg[1] = min(clamped_deg[1], 194.5)

        safe, reason = is_in_safe_joint_envelope(clamped_deg)
        if not safe:
            self.get_logger().error(f"[SEGURIDAD] Comando articular bloqueado: {reason}")
            self.status_message = f"BLOQUEO SEGURIDAD: {reason}"
            return False

        # Registrar inicio y duración del movimiento para que el servoing espere a que se detenga
        self.last_cmd_sent_time = time.time()
        self.current_move_duration_s = float(duration_s)
        self.fresh_post_stop_dets = 0

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
        holding_object = self.gripper_opening_mm < 65.0
        g_mm = self.gripper_opening_mm if (keep_gripper or holding_object) else GRIPPER_MAX_OPEN_MM
        self.send_arm_joint_goal(self.pose_home, duration_s=duration_s, gripper_mm=g_mm)
        self.status_message = f"Moviendo a Posicion HOME (Gripper={g_mm:.0f}mm)."

    def mover_a_pre_grasp(self, duration_s: float = 2.2, keep_gripper: bool = False):
        """
        Mueve el brazo a Pre-Grasp.
        Por defecto (keep_gripper=False), abre el GRIPPER AL MÁXIMO (105 mm) y prepara la cámara DaBai.
        """
        if self.is_teaching_mode:
            self.cancel_teaching_mode()
        self.fase_actual = self.FASE_3_PREGRASP
        self.servo_joints = list(self.pose_pre_grasp)
        self.total_forward_advanced_m = 0.0
        g_mm = GRIPPER_CLOSE_GRASP_MM if keep_gripper else GRIPPER_MAX_OPEN_MM
        self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=duration_s, gripper_mm=g_mm)
        if not keep_gripper:
            self.status_message = (
                f"Modo PRE-GRASP activo: Gripper ABIERTO AL MAXIMO ({GRIPPER_MAX_OPEN_MM:.0f} mm) | "
                f"Camara DaBai lista para guiar IK."
            )
        else:
            self.status_message = f"Retorno Post-Grasping a PRE-GRASP manteniendo objeto ({g_mm:.0f} mm)."

    def set_arm_torque(self, enable: bool):
        msg = Bool()
        msg.data = bool(enable)
        self.pub_enable.publish(msg)

    def toggle_teaching_pose(self, target_pose: str):
        if not self.is_teaching_mode:
            self.is_teaching_mode = True
            self.teaching_target = target_pose
            self.set_arm_torque(False)
            self.status_message = f"MODO ENSENANZA [{target_pose.upper()}]: Mueva el brazo con la mano y presione [ENTER] para guardar."
            self.say(f"Teach mode for {target_pose} activated. Move the arm with your hand and press Enter to save.")
        else:
            save_target = self.teaching_target or target_pose
            self.set_arm_torque(True)
            self.is_teaching_mode = False
            self.save_named_pose(save_target, self.arm_joints_deg, self.gripper_opening_mm)
            self.teaching_target = None

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
    # VISUAL SERVOING PAUSADO ("MOVER -> PARAR -> MEDIR") CON CÁMARA ORBBEC DABAI
    # --------------------------------------------------------------------------
    def start_autonomous_grasp(self):
        """
        Inicia el agarre guiado 100% por la cámara Orbbec DaBai con pausas de estabilización:
          1. Si no está en Pre-Grasp, va a Pre-Grasp con el gripper al máximo (105 mm).
          2. Espera a que el brazo esté quieto y la cámara DaBai tenga lectura estable.
          3. Alinea en pasos suaves (1.20 s por paso, máx 4.0 cm) esperando parada completa en cada paso.
          4. Avanza en pasos suaves (1.20 s por paso, 3.5 a 4.5 cm) esperando parada completa en cada paso.
          5. Al llegar entre los dedos, cierra gripper (50 mm), eleva +2.5 cm y vuelve recto atrás a Pre-Grasp.
        """
        self.align_steps_count = 0
        self.advance_steps_count = 0
        self.total_forward_advanced_m = 0.0
        self.last_valid_bw = 0.0
        self._lift_sent = False
        self._retract_line_sent = False
        self._retract_pregrasp_sent = False

        if self.fase_actual == self.FASE_3_PREGRASP:
            self.servo_joints = list(self.arm_joints_deg[:6])
            self.auto_state = self.AUTO_PASO_C1_ALINEAR_DABAI
            self.step_start_time = time.time()
            self.last_servo_step_time = 0.0
            self.fresh_post_stop_dets = 0
            self.say("Guiding kinematics step by step with DaBai arm camera.")
        else:
            self.servo_joints = list(self.pose_pre_grasp)
            self.auto_state = self.AUTO_PASO_B_MOVER_PREGRASP
            self.step_start_time = time.time()
            self.fase_actual = self.FASE_3_PREGRASP
            self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=2.2, gripper_mm=GRIPPER_MAX_OPEN_MM)
            self.say("Moving to pre-grasp with maximum gripper opening.")

    def ejecutar_rutina_post_grasping(self):
        """
        Cierra la pinza sobre el objeto (50 mm), despega +2.5 cm de la mesa y
        retrocede RECTO HACIA ATRÁS hasta la posición Pre-Grasp.
        """
        if self.is_teaching_mode:
            self.cancel_teaching_mode()

        current_grasp_joints = list(self.arm_joints_deg[:6])
        self.joints_grasp = current_grasp_joints

        j_lift, ok_lift = self.kinematics_solver.move_tool_line(
            current_grasp_joints,
            delta_forward_m=0.0,
            delta_lateral_m=0.0,
            delta_up_m=0.025,
            reference_joints_deg=self.pose_pre_grasp,
        )
        self.joints_lift = j_lift if ok_lift else current_grasp_joints

        if self.joints_aligned is not None:
            j_retract, ok_ret = self.kinematics_solver.move_tool_line(
                self.joints_aligned,
                delta_forward_m=0.0,
                delta_lateral_m=0.0,
                delta_up_m=0.020,
                reference_joints_deg=self.pose_pre_grasp,
            )
            self.joints_retract_straight = j_retract if ok_ret else list(self.joints_aligned)
        else:
            back_m = -max(0.12, float(self.total_forward_advanced_m))
            j_retract, ok_ret = self.kinematics_solver.move_tool_line(
                current_grasp_joints,
                delta_forward_m=back_m,
                delta_lateral_m=0.0,
                delta_up_m=0.020,
                reference_joints_deg=self.pose_pre_grasp,
            )
            self.joints_retract_straight = j_retract if ok_ret else list(self.pose_pre_grasp)

        self.auto_state = self.AUTO_PASO_D_AGARRE
        self.step_start_time = time.time()
        self._lift_sent = False
        self._retract_line_sent = False
        self._retract_pregrasp_sent = False

        self.get_logger().info(f"[Agarre + Post-Grasping] Cerrando gripper a {GRIPPER_CLOSE_GRASP_MM} mm...")
        if not self.dry_run:
            g_msg = Float32()
            g_msg.data = float(GRIPPER_CLOSE_GRASP_MM)
            self.pub_gripper_cmd.publish(g_msg)
        self.say("Object reached! Closing gripper and retracting straight back to pre-grasp.")

    def update_autonomous_state_machine(self, now: float):
        if self.auto_state == self.AUTO_IDLE:
            return

        # PASO 1: LLEGAR A PRE-GRASP CON EL GRIPPER ABIERTO AL MÁXIMO (105 MM)
        if self.auto_state == self.AUTO_PASO_B_MOVER_PREGRASP:
            if not self.is_arm_settling(now) and (now - self.step_start_time > 2.5):
                self.servo_joints = list(self.arm_joints_deg[:6])
                self.auto_state = self.AUTO_PASO_C1_ALINEAR_DABAI
                self.step_start_time = now
                self.last_servo_step_time = 0.0
                self.align_steps_count = 0
                self.fresh_post_stop_dets = 0
                self.say("Pre-grasp reached. Aligning gripper fingers with DaBai camera.")

        # PASO 2 (CÁMARA DABAI): CENTRAR EL OBJETO ENTRE LOS DOS DEDOS (x = 275 px) ESPERANDO PARADA EN CADA PASO
        elif self.auto_state == self.AUTO_PASO_C1_ALINEAR_DABAI:
            # 1) Esperar a que el brazo esté 100% detenido tras el paso anterior
            if self.is_arm_settling(now):
                self.status_message = "PASO 2 (DABAI): Esperando que el brazo termine de moverse y estabilice imagen..."
                return

            # 2) Esperar al menos 2 detecciones limpias capturadas DESPUÉS de que el brazo se detuvo
            if self.fresh_post_stop_dets < 2:
                self.status_message = f"PASO 2 (DABAI): Leyendo frame estatico post-movimiento ({self.fresh_post_stop_dets}/2)..."
                if now - self.step_start_time > 7.0 and self.target_2d_wrist is None:
                    self.say("DaBai camera could not see the object. Staying in pre-grasp.")
                    self.auto_state = self.AUTO_IDLE
                return

            if self.target_2d_wrist is None:
                self.status_message = "PASO 2 (CAMARA DABAI): Buscando botella o tubo frente a la pinza..."
                return

            wx1, _, wx2, _, _ = (*self.target_2d_wrist, 0.0)
            is_clipped_edge = (wx1 <= 5 and self.dabai_err_px < 0) or (wx2 >= 634 and self.dabai_err_px > 0)

            # Verificar si el objeto ya está centrado en el hueco entre los dos dedos (|err_px| <= 24 px)
            if (abs(self.dabai_err_px) <= 24.0 and not is_clipped_edge) or (
                self.align_steps_count >= 8 and abs(self.dabai_err_px) <= 48.0
            ):
                self.joints_aligned = list(self.servo_joints)
                self.auto_state = self.AUTO_PASO_C2_AVANZAR_DABAI
                self.step_start_time = now
                self.last_servo_step_time = now
                self.advance_steps_count = 0
                self.get_logger().info(
                    f"[DaBai ALINEADO] Objeto centrado entre los dedos (wcx={self.dabai_wcx:.0f}px, err={self.dabai_err_px:+.0f}px). Iniciando avance..."
                )
                self.say("Aligned between gripper fingers. Advancing step by step.")
                return

            if self.align_steps_count >= 10:
                self.say("Could not center object within safe lateral range. Stopping safely.")
                self.auto_state = self.AUTO_IDLE
                return

            # Paso lateral pausado y suave (máx 4.0 cm por paso, duración 1.20 s)
            step_left_m = float(np.clip(0.65 * self.wrist_dy_m, -0.040, 0.040))
            if abs(step_left_m) < 0.008:
                step_left_m = math.copysign(0.008, self.wrist_dy_m)

            j_next, _ = self.kinematics_solver.move_tool_line(
                self.servo_joints,
                delta_forward_m=0.0,
                delta_lateral_m=step_left_m,
                delta_up_m=0.0,
                reference_joints_deg=self.pose_pre_grasp,
            )
            self.servo_joints = j_next
            self.align_steps_count += 1
            self.last_servo_step_time = now
            self.get_logger().info(
                f"[DaBai Paso Alinear #{self.align_steps_count}] wcx={self.dabai_wcx:.0f}px (meta 275px, err={self.dabai_err_px:+.0f}px) -> "
                f"dLeft={step_left_m*100:+.1f}cm | Joints={j_next}"
            )
            self.status_message = (
                f"CAMARA DABAI ALINEANDO (Paso #{self.align_steps_count}): Moviendo {step_left_m*100:+.1f}cm y pausando para medir..."
            )
            self.send_arm_joint_goal(j_next, duration_s=1.20, gripper_mm=GRIPPER_MAX_OPEN_MM)

        # PASO 3 (CÁMARA DABAI): AVANZAR PAUSADO PASO A PASO HASTA QUE EL OBJETO ENTRE EN LA PINZA
        elif self.auto_state == self.AUTO_PASO_C2_AVANZAR_DABAI:
            # 1) Esperar a que el brazo termine de moverse por completo
            if self.is_arm_settling(now):
                self.status_message = "PASO 3 (DABAI): Esperando parada del brazo para medir distancia restante..."
                return

            # 2) Si ya avanzó bastante y el objeto llenó la vista de cerca, agarrar
            if self.target_2d_wrist is None:
                if self.total_forward_advanced_m >= 0.07 and self.last_valid_bw >= 145.0:
                    self.get_logger().info("[DaBai LLEGADA] Objeto dentro de la pinza. Ejecutando agarre!")
                    self.ejecutar_rutina_post_grasping()
                    return
                if now - self.last_wrist_det_time > 2.5:
                    self.say("Lost sight of object during approach. Stopping safely.")
                    self.auto_state = self.AUTO_IDLE
                return

            # 3) Esperar al menos 2 frames estáticos post-movimiento antes de decidir o dar el siguiente paso
            if self.fresh_post_stop_dets < 2:
                return

            # CONDICIÓN DE LLEGADA VISUAL EN LA CÁMARA DABAI:
            # Solo confirmar llegada después de haber avanzado al menos 1 paso (>= 3.0 cm)
            # (salvo que la botella ya esté literalmente pegada a los dedos con bw >= 195 px).
            has_advanced = self.total_forward_advanced_m >= 0.030
            arrived_by_width = (has_advanced and self.dabai_bw >= DABAI_ARRIVAL_WIDTH_PX) or (self.dabai_bw >= 195.0)
            arrived_by_dist = has_advanced and (self.wrist_dfwd_m <= 0.015)
            reached_max_steps = self.total_forward_advanced_m >= 0.24 or self.advance_steps_count >= 6

            if arrived_by_width or arrived_by_dist or reached_max_steps:
                self.get_logger().info(
                    f"[DaBai LLEGADA CONFIRMADA] bw={self.dabai_bw:.0f}px, faltan={self.wrist_dfwd_m*100:.1f}cm, "
                    f"avance_total={self.total_forward_advanced_m*100:.1f}cm -> AGARRANDO SIN GOLPEAR!"
                )
                self.ejecutar_rutina_post_grasping()
                return

            # Avance corto y pausado (3.5 cm a 4.5 cm por paso, duración 1.20 s) manteniendo centrado entre dedos
            step_fwd_m = float(np.clip(0.60 * max(0.05, self.wrist_dfwd_m), 0.032, 0.045))
            step_left_m = float(np.clip(0.55 * self.wrist_dy_m, -0.022, 0.022))

            j_next, _ = self.kinematics_solver.move_tool_line(
                self.servo_joints,
                delta_forward_m=step_fwd_m,
                delta_lateral_m=step_left_m,
                delta_up_m=0.0,
                reference_joints_deg=self.pose_pre_grasp,
            )
            self.servo_joints = j_next
            self.total_forward_advanced_m += step_fwd_m
            self.advance_steps_count += 1
            self.last_servo_step_time = now

            self.get_logger().info(
                f"[DaBai Avance #{self.advance_steps_count}] +Fwd={step_fwd_m*100:.1f}cm (Total={self.total_forward_advanced_m*100:.1f}cm), "
                f"dLeft={step_left_m*100:+.1f}cm | bw={self.dabai_bw:.0f}px (meta {DABAI_ARRIVAL_WIDTH_PX:.0f}px)"
            )
            self.status_message = (
                f"CAMARA DABAI AVANZANDO (Paso #{self.advance_steps_count}): +{step_fwd_m*100:.1f}cm "
                f"(Total={self.total_forward_advanced_m*100:.1f}cm | Ancho={self.dabai_bw:.0f}/{DABAI_ARRIVAL_WIDTH_PX:.0f}px)"
            )
            self.send_arm_joint_goal(j_next, duration_s=1.20, gripper_mm=GRIPPER_MAX_OPEN_MM)

        # PASO 4: CERRAR PINZA SOBRE EL OBJETO Y DESPEGAR +2.5 CM DE LA MESA
        elif self.auto_state == self.AUTO_PASO_D_AGARRE:
            step_elapsed = now - self.step_start_time
            if step_elapsed > 1.5 and not self._lift_sent:
                self._lift_sent = True
                if self.joints_lift is not None:
                    self.get_logger().info(f"[Paso 4] Despegando objeto +2.5 cm de la mesa: {self.joints_lift}")
                    self.status_message = "PASO 4: Objeto agarrado, despegando +2.5 cm antes de volver recto atras..."
                    self.send_arm_joint_goal(self.joints_lift, duration_s=1.1, gripper_mm=GRIPPER_CLOSE_GRASP_MM)

            if step_elapsed > 2.7:
                self.auto_state = self.AUTO_PASO_E_POSTGRASP_PREGRASP
                self.step_start_time = now
                self._retract_line_sent = False
                self._retract_pregrasp_sent = False
                self.say("Retracting straight back to pre-grasp position.")

        # PASO 5 (RUTINA POST-GRASPING): VOLVER RECTO HACIA ATRÁS HASTA LA POSICIÓN PRE-GRASP
        elif self.auto_state == self.AUTO_PASO_E_POSTGRASP_PREGRASP:
            step_elapsed = now - self.step_start_time

            if not self._retract_line_sent:
                self._retract_line_sent = True
                target_retract = self.joints_retract_straight if self.joints_retract_straight is not None else self.pose_pre_grasp
                self.get_logger().info(f"[Paso 5 - POST-GRASPING] Retrocediendo RECTO HACIA ATRAS: {target_retract}")
                self.status_message = "PASO 5 (POST-GRASPING): Volviendo RECTO HACIA ATRAS hacia Pre-Grasp..."
                self.send_arm_joint_goal(target_retract, duration_s=1.8, gripper_mm=GRIPPER_CLOSE_GRASP_MM)

            if step_elapsed > 1.9 and not self._retract_pregrasp_sent:
                self._retract_pregrasp_sent = True
                self.mover_a_pre_grasp(duration_s=1.4, keep_gripper=True)

            if step_elapsed > 3.4:
                self.auto_state = self.AUTO_SUCCESS
                self.step_start_time = now
                self.say("Post-grasping complete! Object is held in pre-grasp position.")

        elif self.auto_state == self.AUTO_SUCCESS:
            if now - self.step_start_time > 4.0:
                self.auto_state = self.AUTO_IDLE

    # --------------------------------------------------------------------------
    # RENDERIZADO VISUAL DEL HUD (1280 x 600 - FLUIDO A 30 FPS)
    # --------------------------------------------------------------------------
    def draw_hud(self) -> np.ndarray:
        canvas_w, canvas_h = 1280, 600
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        feed_w, feed_h = 620, 420

        with self.frame_lock:
            h_raw = self.head_color_frame.copy() if self.head_color_frame is not None else None
            w_raw = self.wrist_color_frame.copy() if self.wrist_color_frame is not None else None

        # --- PANEL IZQUIERDO: CÁMARA PRINCIPAL (VISTA GENERAL) ---
        if h_raw is not None:
            h_img = cv2.resize(h_raw, (feed_w, feed_h))
            canvas[40:40 + feed_h, 15:15 + feed_w] = h_img
        else:
            cv2.rectangle(canvas, (15, 40), (15 + feed_w, 40 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "CAMARA PRINCIPAL (VISTA GENERAL)", (140, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (120, 140, 160), 1)

        cv2.putText(canvas, "1. CAMARA PRINCIPAL (VISTA GENERAL DE APOYO)", (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 210, 230), 1)

        # --- PANEL DERECHO: CÁMARA ORBBEC DABAI (GUÍA EXPRESA DE LA CINEMÁTICA INVERSA) ---
        if w_raw is not None:
            w_img = cv2.resize(w_raw, (feed_w, feed_h))
            orig_h, orig_w = w_raw.shape[:2]
            sx, sy = feed_w / orig_w, feed_h / orig_h

            gap_cx = int(DABAI_GRIPPER_GAP_X_PX * sx)
            wcy = int(feed_h / 2)
            cv2.line(w_img, (gap_cx, 20), (gap_cx, feed_h - 20), (0, 255, 100), 2)
            cv2.drawMarker(w_img, (gap_cx, wcy), (0, 255, 100), cv2.MARKER_TILTED_CROSS, 24, 2)
            cv2.putText(
                w_img,
                f"HUECO ENTRE DEDOS ({self.det_fps:.0f} FPS)",
                (max(10, gap_cx - 105), 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (0, 255, 100),
                1,
            )

            if self.target_2d_wrist is not None:
                x1, y1, x2, y2 = self.target_2d_wrist
                bx1, by1 = int(x1 * sx), int(y1 * sy)
                bx2, by2 = int(x2 * sx), int(y2 * sy)
                centered_ok = abs(self.dabai_err_px) <= 24.0
                box_col = (0, 255, 80) if centered_ok else (0, 180, 255)
                cv2.rectangle(w_img, (bx1, by1), (bx2, by2), box_col, 2)

                obj_cx = int(((x1 + x2) / 2.0) * sx)
                obj_cy = int(((y1 + y2) / 2.0) * sy)
                cv2.line(w_img, (obj_cx, obj_cy), (gap_cx, obj_cy), (0, 255, 255), 2)
                cv2.circle(w_img, (obj_cx, obj_cy), 5, box_col, -1)

                tag_top = f"dLeft={self.wrist_dy_m*100:+.1f}cm | Faltan={self.wrist_dfwd_m*100:.1f}cm"
                tag_bot = f"Ancho={self.dabai_bw:.0f}/{DABAI_ARRIVAL_WIDTH_PX:.0f}px ({'CENTRADO OK' if centered_ok else 'ALINEANDO'})"
                cv2.putText(w_img, tag_top, (max(8, bx1), max(22, by1 - 24)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, box_col, 2)
                cv2.putText(w_img, tag_bot, (max(8, bx1), max(40, by1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)

            canvas[40:40 + feed_h, 645:645 + feed_w] = w_img
        else:
            cv2.rectangle(canvas, (645, 40), (645 + feed_w, 40 + feed_h), (25, 30, 40), -1)
            cv2.putText(canvas, "ESPERANDO STREAM CAMARA ORBBEC DABAI...", (730, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 140, 160), 1)

        cv2.putText(
            canvas,
            "2. CAMARA ORBBEC DABAI (17 FPS DEDICADO - PAUSA Y ESTABILIZA EN CADA PASO)",
            (645, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (0, 255, 140),
            1,
        )

        # --- PANEL INFERIOR: ESTADO Y CONTROLES SIMPLES ---
        bar_y = 468
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (12, 16, 24), -1)
        cv2.line(canvas, (0, bar_y), (canvas_w, bar_y), (60, 80, 110), 1)

        state_key = self.auto_state if self.auto_state != self.AUTO_IDLE else self.fase_actual
        state_str = self.FASE_NAMES.get(state_key, "LISTO")
        cv2.putText(canvas, f"ESTADO: {state_str}", (20, bar_y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 255), 2)

        dabai_str = (
            f"DaBai ({self.det_fps:.0f}FPS): Faltan {self.wrist_dfwd_m*100:.1f}cm, dLeft={self.wrist_dy_m*100:+.1f}cm, Ancho={self.dabai_bw:.0f}px"
            if self.target_2d_wrist is not None else f"DaBai ({self.det_fps:.0f}FPS): BUSCANDO OBJETO..."
        )
        joints_str = ", ".join([f"{v:+.1f}" for v in self.arm_joints_deg[:6]])
        telemetry_line = f"{dabai_str} | Pinza: {self.gripper_opening_mm:.0f}mm | Joints: [{joints_str}] | {self.status_message}"
        cv2.putText(canvas, telemetry_line, (20, bar_y + 56), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 220, 240), 1)

        shortcuts1 = "[3]/[p] Modo Pre-Grasp (Gripper Abierto al Maximo 105mm) | [ESPACIO]/[G] Agarrar Guiado por Camara DaBai"
        shortcuts2 = "[5]/[B] Rutina Post-Grasping (Cerrar y Volver Recto Atras a Pre-Grasp) | [2]/[h] Home | [R] Abrir 105mm | [C] Cerrar 50mm"
        cv2.putText(canvas, shortcuts1, (20, bar_y + 88), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 255, 140), 1)
        cv2.putText(canvas, shortcuts2, (20, bar_y + 114), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (0, 230, 255), 1)

        return canvas

    # --------------------------------------------------------------------------
    # BUCLE PRINCIPAL INTERACTIVO
    # --------------------------------------------------------------------------
    def run(self):
        window_name = "Robot @Home - Manipulacion Guiada por Camara Orbbec DaBai"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 620)

        print("==========================================================================")
        print("   ROBOT @HOME - CINEMATICA INVERSA GUIADA POR CAMARA ORBBEC DABAI        ")
        print("--------------------------------------------------------------------------")
        print("   * [3] o [p]       : Ir a PRE-GRASP con GRIPPER AL MAXIMO (105 mm)      ")
        print("   * [ESPACIO] o [G] : Agarrar guiado 100% por la camara Orbbec DaBai:    ")
        print("                       1) Centra el objeto pausando en cada paso          ")
        print("                       2) Avanza suave esperando imagen estable post-paso ")
        print("                       3) Se detiene en cuanto el objeto entra en la pinza")
        print("                       4) Cierra el gripper (50 mm) y vuelve recto atras  ")
        print("                          a la posicion de PRE-GRASP                      ")
        print("   * [5] o [B]       : Ejecutar Rutina Post-Grasping manualmente          ")
        print("   * [2] o [h]       : Ir a posicion HOME                                 ")
        print("==========================================================================")

        while rclpy.ok():
            now = time.time()
            if now - self.last_marker_pub_time >= 0.065:
                self.last_marker_pub_time = now
                self.publish_rviz_3d_markers()

            self.update_autonomous_state_machine(now)

            canvas = self.draw_hud()
            cv2.imshow(window_name, canvas)

            key = cv2.waitKey(25) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                self._running = False
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

            elif key == 13:
                if self.is_teaching_mode:
                    self.toggle_teaching_pose(self.teaching_target or "pre_grasp")

            elif key in [ord("5"), ord("b"), ord("B")]:
                self.ejecutar_rutina_post_grasping()

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
    parser = argparse.ArgumentParser(description="Manipulacion Guiada por Camara Orbbec DaBai (IK 6-DOF)")
    parser.add_argument("--dry-run", action="store_true", help="Modo simulacion sin mover motores")
    args = parser.parse_args()

    rclpy.init()
    node = ManipulacionAutonomaNode(dry_run=args.dry_run)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
