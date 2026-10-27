#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Robot @Home - Calibración y Manipulación de Alta Precisión con Marcador ArUco
#
# Flujo de trabajo:
#   1. Pega el marcador ArUco (5.0 cm, ID 0) en el tubo o botella.
#   2. Sujeta el tubo con la pinza cerrada en la posición ideal de agarre.
#   3. Presiona [C] para memorizar la pose de agarre exacta (X, Y, Z milimétricos).
#   4. Coloca el tubo en la mesa y presiona [ESPACIO] o [G]: el brazo irá directo
#      al punto exacto guiado por el ArUco y lo agarrará sin error.
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
from typing import List, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

# Kinematics solver
from base_athome_perception.piper_kinematics import (
    is_in_safe_joint_envelope,
    PiperKinematicsSolver,
)

CONFIG_FILE_PATH = "/home/athome/base_athome_ws/src/base_athome_perception/config/arm_poses_and_limits.yaml"

# Geometría intrínseca cámara Orbbec DaBai DC1 (640x480)
DABAI_FX = 485.0
DABAI_FY = 485.0
DABAI_CX = 320.0
DABAI_CY = 240.0
CAMERA_MATRIX = np.array([[DABAI_FX, 0.0, DABAI_CX],
                          [0.0, DABAI_FY, DABAI_CY],
                          [0.0, 0.0, 1.0]], dtype=np.float32)
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)

GRIPPER_MAX_OPEN_MM = 105.0
GRIPPER_CLOSE_GRASP_MM = 50.0


class ArucoCalibrationNode(Node):
    """Nodo interactivo para calibrar y probar agarre de objetos con marcador ArUco."""

    def __init__(self, dry_run: bool = False):
        super().__init__("aruco_calibration_node")
        self.dry_run = dry_run
        self.load_configuration()

        # Configuración ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # Telemetría y estado
        self.frame_lock = threading.Lock()
        self.wrist_color_frame: Optional[np.ndarray] = None
        self.arm_joints_deg = list(self.pose_home)
        self.gripper_opening_mm = GRIPPER_MAX_OPEN_MM
        self.arm_is_moving = False
        self.last_cmd_time = 0.0
        self.move_duration_s = 2.0

        # Detección ArUco en vivo
        self.detected_id: Optional[int] = None
        self.marker_tvec: Optional[np.ndarray] = None  # [X, Y, Z] en metros
        self.marker_rvec: Optional[np.ndarray] = None
        self.marker_corners: Optional[np.ndarray] = None
        self.marker_center_px: Optional[Tuple[float, float]] = None

        # Pose objetivo calibrada de agarre (relativa al lente DaBai)
        # Por defecto: X=0 cm, Y=+3.5 cm (dedos abajo del lente), Z=19.5 cm (lente a dedos)
        self.calib_target_x_cm: float = 0.0
        self.calib_target_y_cm: float = 3.5
        self.calib_target_z_cm: float = 19.5
        self.calibrated = False

        # Estado del agarre autónomo por ArUco
        self.is_auto_grasping = False
        self.auto_step = 0
        self.auto_step_start_time = 0.0
        self.servo_joints: List[float] = list(self.pose_pre_grasp)
        self.total_advanced_m = 0.0

        self.status_message = "Listo. Coloque el objeto en la pinza y presione [C] para calibrar agarre."

        # Solucionador cinemático
        self.kinematics_solver = PiperKinematicsSolver()

        # ROS 2 Subscripciones y Publicadores
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

        self.sub_cam = self.create_subscription(
            Image, "/arm_camera/image_raw", self.camera_callback, qos_latest
        )
        self.sub_arm_status = self.create_subscription(
            String, "/arm/status", self.arm_status_callback, qos_status
        )

        self.pub_joint_cmd = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_gripper_cmd = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_say = self.create_publisher(String, "/hri/say_something", 5)
        self.pub_markers = self.create_publisher(MarkerArray, "/arm_camera/aruco_markers", 5)

        # Hilo dedicado ROS 2 spin
        self._running = True
        self._spin_thread = threading.Thread(target=self._spin_worker, daemon=True)
        self._spin_thread.start()

    def _spin_worker(self):
        while self._running and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.005)
            except Exception:
                time.sleep(0.005)

    def load_configuration(self):
        """Carga parámetros y poses guardadas desde YAML."""
        self.pose_home = [72.09, 176.5, -155.15, 102.62, -31.51, -99.91]
        self.pose_pre_grasp = [42.9, 180.83, -89.01, 60.57, -74.02, -96.89]
        self.marker_size_mm: float = 50.0  # 50 mm = 5.0 cm
        self.arm_speed_pct: int = 45

        if os.path.exists(CONFIG_FILE_PATH):
            try:
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                    env = data.get("safety_envelope", {})
                    if "arm_speed_pct" in env:
                        self.arm_speed_pct = int(env["arm_speed_pct"])
                    if "aruco_marker_size_mm" in env:
                        self.marker_size_mm = float(env["aruco_marker_size_mm"])
                    if "aruco_target_grasp_cm" in env:
                        tg = env["aruco_target_grasp_cm"]
                        self.calib_target_x_cm = float(tg.get("x", self.calib_target_x_cm))
                        self.calib_target_y_cm = float(tg.get("y", self.calib_target_y_cm))
                        self.calib_target_z_cm = float(tg.get("z", self.calib_target_z_cm))
                        self.calibrated = True

                    poses = data.get("named_poses", {})
                    if "home" in poses:
                        self.pose_home = list(poses["home"].get("joints_deg", self.pose_home))
                    if "pre_grasp" in poses:
                        self.pose_pre_grasp = list(poses["pre_grasp"].get("joints_deg", self.pose_pre_grasp))
                self.get_logger().info(
                    f"Configuracion cargada. ArUco Target: X={self.calib_target_x_cm:.1f}cm, "
                    f"Y={self.calib_target_y_cm:.1f}cm, Z={self.calib_target_z_cm:.1f}cm"
                )
            except Exception as e:
                self.get_logger().warn(f"Error cargando YAML: {e}")

    def save_calibration_to_yaml(self):
        """Guarda la pose de agarre calibrada en YAML."""
        try:
            data = {}
            if os.path.exists(CONFIG_FILE_PATH):
                with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}

            if "safety_envelope" not in data:
                data["safety_envelope"] = {}

            data["safety_envelope"]["aruco_marker_size_mm"] = round(float(self.marker_size_mm), 1)
            data["safety_envelope"]["aruco_target_grasp_cm"] = {
                "x": round(float(self.calib_target_x_cm), 2),
                "y": round(float(self.calib_target_y_cm), 2),
                "z": round(float(self.calib_target_z_cm), 2),
            }
            # También sincronizar offset lateral estimado en mm para el nodo principal
            offset_mm = round(float(self.calib_target_x_cm * 10.0), 1)
            data["safety_envelope"]["dabai_lateral_offset_mm"] = -offset_mm

            with open(CONFIG_FILE_PATH, "w", encoding="utf-8") as f:
                yaml.dump(data, f, default_flow_style=False, sort_keys=False)

            self.status_message = (
                f"CALIBRACION GUARDADA: Target X={self.calib_target_x_cm:+.1f}cm, "
                f"Y={self.calib_target_y_cm:+.1f}cm, Z={self.calib_target_z_cm:.1f}cm"
            )
            self.say("ArUco grasp pose calibrated and saved successfully.")
        except Exception as e:
            self.get_logger().error(f"Error guardando calibracion en YAML: {e}")

    def camera_callback(self, msg: Image):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            with self.frame_lock:
                self.wrist_color_frame = frame
        except Exception:
            pass

    def arm_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.arm_is_moving = bool(data.get("is_moving", False))
            self.gripper_opening_mm = float(data.get("gripper_mm", self.gripper_opening_mm))
            self.arm_joints_deg = list(data.get("joints_deg", self.arm_joints_deg))
        except Exception:
            pass

    def say(self, text: str):
        clean_ascii = text.encode("ascii", "ignore").decode("ascii")
        msg = String()
        msg.data = clean_ascii
        self.pub_say.publish(msg)
        print(f"[Robot]: \"{clean_ascii}\"")

    def send_arm_joint_goal(self, joints_deg: List[float], duration_s: float = 2.0, gripper_mm: Optional[float] = None) -> bool:
        clamped_deg = list(joints_deg[:6])
        clamped_deg[1] = min(clamped_deg[1], 194.5)

        safe, reason = is_in_safe_joint_envelope(clamped_deg)
        if not safe:
            self.get_logger().error(f"Bloqueo seguridad: {reason}")
            self.status_message = f"BLOQUEO SEGURIDAD: {reason}"
            return False

        spd_scale = 75.0 / max(15.0, float(self.arm_speed_pct))
        scaled_dur = float(np.clip(duration_s * spd_scale, 0.8, 9.5))

        self.last_cmd_time = time.time()
        self.move_duration_s = scaled_dur

        if self.dry_run:
            self.get_logger().info(f"[DRY-RUN] joints={clamped_deg}, dur={scaled_dur:.2f}s")
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

    def is_arm_settling(self, now: float) -> bool:
        return bool(self.arm_is_moving or (now - self.last_cmd_time < self.move_duration_s + 0.50))

    def mover_a_pre_grasp(self):
        self.servo_joints = list(self.pose_pre_grasp)
        self.total_advanced_m = 0.0
        self.send_arm_joint_goal(self.pose_pre_grasp, duration_s=2.2, gripper_mm=GRIPPER_MAX_OPEN_MM)
        self.status_message = "Moviendo a PRE-GRASP con pinza abierta al maximo (105 mm)."

    def mover_a_home(self):
        self.send_arm_joint_goal(self.pose_home, duration_s=2.5, gripper_mm=self.gripper_opening_mm)
        self.status_message = "Moviendo a posicion HOME."

    def set_gripper(self, mm: float):
        msg = Float32()
        msg.data = float(mm)
        self.pub_gripper_cmd.publish(msg)

    # --------------------------------------------------------------------------
    # DETECCIÓN DE MARCADORES ARUCO Y ESTIMACIÓN DE POSE 3D
    # --------------------------------------------------------------------------
    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        display = frame.copy()
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, self.aruco_dict, parameters=self.aruco_params
        )

        self.detected_id = None
        self.marker_tvec = None
        self.marker_rvec = None

        if ids is not None and len(ids) > 0:
            target_idx = 0
            # Preferir ID 0 si hay varios
            for i, mid in enumerate(ids):
                if mid[0] == 0:
                    target_idx = i
                    break

            self.detected_id = int(ids[target_idx][0])
            corner_pts = corners[target_idx]
            self.marker_corners = corner_pts

            # Estimación de pose 3D (X, Y, Z en metros)
            marker_len_m = max(0.015, float(self.marker_size_mm) / 1000.0)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corner_pts], marker_len_m, CAMERA_MATRIX, DIST_COEFFS
            )

            rvec = rvecs[0][0]
            tvec = tvecs[0][0]
            self.marker_rvec = rvec
            self.marker_tvec = tvec

            # Dibujar contorno del marcador y ejes 3D
            cv2.aruco.drawDetectedMarkers(display, [corner_pts], np.array([[self.detected_id]]))
            cv2.drawFrameAxes(display, CAMERA_MATRIX, DIST_COEFFS, rvec, tvec, marker_len_m * 0.8, 2)

            # Centro 2D en píxeles
            cx = float(np.mean(corner_pts[0, :, 0]))
            cy = float(np.mean(corner_pts[0, :, 1]))
            self.marker_center_px = (cx, cy)

            x_cm = tvec[0] * 100.0
            y_cm = tvec[1] * 100.0
            z_cm = tvec[2] * 100.0

            dx_cm = x_cm - self.calib_target_x_cm
            dy_cm = y_cm - self.calib_target_y_cm
            dz_cm = z_cm - self.calib_target_z_cm

            # Etiquetas en pantalla
            info_line1 = f"ArUco ID {self.detected_id}: X={x_cm:+.1f}cm, Y={y_cm:+.1f}cm, Z={z_cm:.1f}cm"
            info_line2 = f"Faltan para agarre: dX={dx_cm:+.1f}cm, dY={dy_cm:+.1f}cm, dZ={dz_cm:+.1f}cm"
            cv2.putText(display, info_line1, (int(cx) - 130, int(cy) - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)
            cv2.putText(display, info_line2, (int(cx) - 130, int(cy) - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 80), 1)

        return display

    # --------------------------------------------------------------------------
    # CALIBRACIÓN DE AGARRE IDEAL ("TEACH-BY-HOLDING")
    # --------------------------------------------------------------------------
    def calibrar_pose_actual(self):
        """Memoriza la posición 3D actual del ArUco como la pose perfecta de agarre."""
        if self.marker_tvec is None:
            self.status_message = "ERROR: No se detecta ningun marcador ArUco para calibrar."
            self.say("No ArUco marker detected. Make sure the marker is in view.")
            return

        tvec = self.marker_tvec
        self.calib_target_x_cm = float(tvec[0] * 100.0)
        self.calib_target_y_cm = float(tvec[1] * 100.0)
        self.calib_target_z_cm = float(tvec[2] * 100.0)
        self.calibrated = True

        self.save_calibration_to_yaml()

    # --------------------------------------------------------------------------
    # AGARRE AUTÓNOMO DE PRUEBA CON ARUCO
    # --------------------------------------------------------------------------
    def start_test_grasp(self):
        """Inicia aproximación y agarre paso a paso guiado 100% por el ArUco."""
        if not self.calibrated:
            self.status_message = "ADVERTENCIA: Primero calibre la pose ideal con [C] antes de probar agarre."
            self.say("Please calibrate ideal grasp pose first by pressing C.")
            return

        if self.marker_tvec is None:
            self.status_message = "ERROR: No veo el ArUco en la mesa. Coloque el tubo frente a la camara."
            self.say("I cannot see the ArUco marker on the table.")
            return

        self.is_auto_grasping = True
        self.auto_step = 1
        self.auto_step_start_time = time.time()
        self.total_advanced_m = 0.0
        self.servo_joints = list(self.arm_joints_deg[:6])
        self.set_gripper(GRIPPER_MAX_OPEN_MM)
        self.say("Starting autonomous ArUco grasp approach.")

    def update_auto_grasp_state_machine(self, now: float):
        if not self.is_auto_grasping:
            return

        # Esperar que el brazo termine de moverse
        if self.is_arm_settling(now):
            return

        # Paso 1: ALINEAR LATERALMENTE (dX -> 0)
        if self.auto_step == 1:
            if self.marker_tvec is None:
                if now - self.auto_step_start_time > 3.0:
                    self.status_message = "ArUco perdido durante alineacion. Deteniendo."
                    self.is_auto_grasping = False
                return

            dx_m = float(self.marker_tvec[0] - (self.calib_target_x_cm / 100.0))
            # dX positivo = el objeto está a la derecha de la meta -> mover brazo a la derecha (-delta_lateral)
            # En la cinemática de la herramienta: +delta_lateral mueve el brazo hacia la izquierda
            step_lat_m = float(np.clip(-0.75 * dx_m, -0.040, 0.040))

            if abs(dx_m) <= 0.008:  # Error menor a 8 mm: alineado!
                self.auto_step = 2
                self.auto_step_start_time = now
                self.get_logger().info(f"[ArUco] Alineado lateralmente (dx={dx_m*100:+.1f}cm). Avanzando...")
                self.status_message = "ArUco centrado. Iniciando avance hacia el objeto..."
                return

            j_next, _ = self.kinematics_solver.move_tool_line(
                self.servo_joints,
                delta_forward_m=0.0,
                delta_lateral_m=step_lat_m,
                delta_up_m=0.0,
                reference_joints_deg=self.pose_pre_grasp,
            )
            self.servo_joints = j_next
            self.status_message = f"Alineando ArUco: dX={dx_m*100:+.1f}cm (paso {step_lat_m*100:+.1f}cm)..."
            self.send_arm_joint_goal(j_next, duration_s=1.20, gripper_mm=GRIPPER_MAX_OPEN_MM)

        # Paso 2: AVANZAR DIRECTO AL OBJETO (dZ -> 0)
        elif self.auto_step == 2:
            if self.marker_tvec is None:
                # Si ya avanzó suficiente y quedó muy cerca del lente, cerrar pinza!
                if self.total_advanced_m >= 0.08:
                    self.auto_step = 3
                    self.auto_step_start_time = now
                    return
                if now - self.auto_step_start_time > 3.0:
                    self.status_message = "ArUco perdido durante avance. Deteniendo."
                    self.is_auto_grasping = False
                return

            dz_m = float(self.marker_tvec[2] - (self.calib_target_z_cm / 100.0))
            dx_m = float(self.marker_tvec[0] - (self.calib_target_x_cm / 100.0))

            # Llegada: distancia restante menor a 1.5 cm o límite alcanzado
            if dz_m <= 0.015 or self.total_advanced_m >= 0.25:
                self.auto_step = 3
                self.auto_step_start_time = now
                self.get_logger().info(f"[ArUco] Llegada a meta de agarre (dZ={dz_m*100:.1f}cm). Cerrando pinza!")
                self.status_message = "Llegada completada: Cerrando pinza sobre el objeto..."
                return

            step_fwd_m = float(np.clip(0.65 * max(0.04, dz_m), 0.030, 0.045))
            # Mantener alineado si el error lateral es mayor a 1 cm
            step_lat_m = float(np.clip(-0.50 * dx_m, -0.015, 0.015)) if abs(dx_m) > 0.010 else 0.0

            j_next, _ = self.kinematics_solver.move_tool_line(
                self.servo_joints,
                delta_forward_m=step_fwd_m,
                delta_lateral_m=step_lat_m,
                delta_up_m=0.0,
                reference_joints_deg=self.pose_pre_grasp,
            )
            self.servo_joints = j_next
            self.total_advanced_m += step_fwd_m
            self.status_message = f"Avanzando: Faltan {dz_m*100:.1f}cm (Avanzado={self.total_advanced_m*100:.1f}cm)..."
            self.send_arm_joint_goal(j_next, duration_s=1.20, gripper_mm=GRIPPER_MAX_OPEN_MM)

        # Paso 3: CERRAR PINZA (50 mm) Y ELEVAR
        elif self.auto_step == 3:
            self.set_gripper(GRIPPER_CLOSE_GRASP_MM)
            time.sleep(0.5)
            # Despegar +3 cm de la mesa
            j_lift, ok = self.kinematics_solver.move_tool_line(
                self.servo_joints,
                delta_forward_m=0.0,
                delta_lateral_m=0.0,
                delta_up_m=0.030,
                reference_joints_deg=self.pose_pre_grasp,
            )
            if ok:
                self.send_arm_joint_goal(j_lift, duration_s=1.2, gripper_mm=GRIPPER_CLOSE_GRASP_MM)
            self.auto_step = 4
            self.auto_step_start_time = now
            self.status_message = "Objeto agarrado con ArUco! Despegando de la mesa."
            self.say("Object grasped with high precision ArUco guidance!")

        # Paso 4: RETORNAR A PRE-GRASP
        elif self.auto_step == 4:
            if now - self.auto_step_start_time > 2.0:
                self.mover_a_pre_grasp()
                self.is_auto_grasping = False
                self.status_message = "Rutina completada con exito: Objeto en posicion Pre-Grasp."

    # --------------------------------------------------------------------------
    # RENDERIZADO DEL HUD
    # --------------------------------------------------------------------------
    def draw_hud(self) -> np.ndarray:
        canvas_w, canvas_h = 1000, 680
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        feed_w, feed_h = 640, 480

        with self.frame_lock:
            raw = self.wrist_color_frame.copy() if self.wrist_color_frame is not None else None

        if raw is not None:
            processed = self.process_frame(raw)
            canvas[40:40 + feed_h, 180:180 + feed_w] = processed
        else:
            cv2.rectangle(canvas, (180, 40), (180 + feed_w, 40 + feed_h), (30, 35, 45), -1)
            cv2.putText(canvas, "ESPERANDO CAMARA DABAI (/arm_camera/image_raw)...", (220, 270), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (140, 160, 180), 1)

        # Barra de estado inferior
        bar_y = 535
        cv2.rectangle(canvas, (0, bar_y), (canvas_w, canvas_h), (16, 20, 28), -1)
        cv2.line(canvas, (0, bar_y), (canvas_w, bar_y), (70, 90, 120), 1)

        calib_str = (
            f"TARGET CALIBRADO: X={self.calib_target_x_cm:+.1f}cm, Y={self.calib_target_y_cm:+.1f}cm, Z={self.calib_target_z_cm:.1f}cm"
            if self.calibrated
            else "TARGET CALIBRADO: NO CALIBRADO (Coloque tubo en pinza y presione [C])"
        )
        cv2.putText(canvas, calib_str, (25, bar_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)

        cur_str = (
            f"ArUco ID {self.detected_id} en vivo: X={self.marker_tvec[0]*100:+.1f}cm, "
            f"Y={self.marker_tvec[1]*100:+.1f}cm, Z={self.marker_tvec[2]*100:.1f}cm"
            if self.marker_tvec is not None
            else "ArUco en vivo: BUSCANDO MARCADOR ARUCO ID 0 EN EL CAMPO VISUAL..."
        )
        cv2.putText(canvas, cur_str, (25, bar_y + 56), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 120) if self.marker_tvec is not None else (120, 150, 180), 1)
        cv2.putText(canvas, f"ESTADO: {self.status_message}", (25, bar_y + 84), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (220, 230, 240), 1)

        shortcuts = (
            "[C]/[ENTER] Calibrar Agarre | [ESPACIO]/[G] Probar Agarre | [3] Pre-Grasp | [2] Home | "
            "[R] Abrir Pinza | [X] Cerrar Pinza | [K] Guardar"
        )
        cv2.putText(canvas, shortcuts, (25, bar_y + 114), cv2.FONT_HERSHEY_SIMPLEX, 0.41, (80, 220, 255), 1)

        return canvas

    # --------------------------------------------------------------------------
    # BUCLE PRINCIPAL INTERACTIVO
    # --------------------------------------------------------------------------
    def run(self):
        window_name = "Calibrador ArUco - Agarre de Precision Brazo Piper"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1000, 720)

        def _on_size(val):
            self.marker_size_mm = float(max(15, val))

        def _on_speed(val):
            self.arm_speed_pct = int(np.clip(val, 15, 100))

        cv2.createTrackbar("Tamano_ArUco_mm", window_name, int(round(self.marker_size_mm)), 100, _on_size)
        cv2.createTrackbar("Velocidad_%", window_name, int(round(self.arm_speed_pct)), 100, _on_speed)

        print("==========================================================================")
        print("   CALIBRADOR DE AGARRE CON MARCADOR ARUCO - BRAZO PIPER & ORBBEC DABAI   ")
        print("--------------------------------------------------------------------------")
        print("   1. Sujeta el tubo con la pinza cerrada en la posicion ideal de agarre. ")
        print("   2. Presiona [C] para memorizar la pose de agarre exacta.               ")
        print("   3. Coloca el tubo en la mesa y presiona [ESPACIO] o [G] para agarrar.  ")
        print("   * [3] o [p]       : Ir a Pre-Grasp con pinza abierta al maximo         ")
        print("   * [2] o [h]       : Ir a Home                                          ")
        print("   * [R]             : Abrir pinza al maximo (105 mm)                     ")
        print("   * [X]             : Cerrar pinza sobre el objeto (50 mm)               ")
        print("   * [K]             : Guardar calibracion en YAML                        ")
        print("==========================================================================")

        while rclpy.ok():
            now = time.time()
            self.update_auto_grasp_state_machine(now)

            canvas = self.draw_hud()
            cv2.imshow(window_name, canvas)

            key = cv2.waitKey(25) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                self._running = False
                break

            elif key in [ord("c"), ord("C"), 13]:  # [C] o ENTER
                self.calibrar_pose_actual()

            elif key in [ord("g"), ord("G"), 32]:  # [G] o ESPACIO
                self.start_test_grasp()

            elif key in [ord("3"), ord("p")]:
                self.mover_a_pre_grasp()

            elif key in [ord("2"), ord("h")]:
                self.mover_a_home()

            elif key in [ord("r"), ord("R")]:
                self.set_gripper(GRIPPER_MAX_OPEN_MM)
                self.status_message = f"Pinza abierta a {GRIPPER_MAX_OPEN_MM:.0f} mm."

            elif key in [ord("x"), ord("X")]:
                self.set_gripper(GRIPPER_CLOSE_GRASP_MM)
                self.status_message = f"Pinza cerrada a {GRIPPER_CLOSE_GRASP_MM:.0f} mm."

            elif key in [ord("k"), ord("K")]:
                self.save_calibration_to_yaml()

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Calibracion de Manipulacion con ArUco (Brazo Piper)")
    parser.add_argument("--dry-run", action="store_true", help="Modo simulacion sin mover motores")
    args = parser.parse_args()

    rclpy.init()
    node = ArucoCalibrationNode(dry_run=args.dry_run)
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
