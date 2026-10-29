#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Calibrador Interactivo de Velocidad y Parametros de Trackeo Facial
# Permite ajustar ganancias (KP), velocidad, zona muerta y sentido de giro en vivo.
# Al presionar [S], guarda directamente la configuracion en tracking_params.yaml
# que el nodo de rostros lee y recarga automaticamente en tiempo real.
# ==============================================================================

import argparse
import os
import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml

# Rutas del paquete
current_dir = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.dirname(current_dir)
if pkg_root not in sys.path:
    sys.path.insert(0, pkg_root)

# ROS 2 Imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Vector3
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from base_athome_perception.face_recognizer import FaceRecognizer


CONFIG_PATH = os.path.join(pkg_root, "config", "tracking_params.yaml")


class TrackingCalibrator(Node if ROS2_AVAILABLE else object):
    def __init__(self, topic: str = "/camera/camera/color/image_raw"):
        if ROS2_AVAILABLE:
            super().__init__("tracking_calibrator_node")
            self.bridge = CvBridge()
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.subs_cam = []
            cand_topics = [
                topic,
                "/zed/zed_node/rgb/image_rect_color",
                "/zed/zed_node/left/image_rect_color",
                "/zed/zed_node/rgb/image_raw_color",
                "/zed/zed_node/left/image_raw_color",
                "/zed/rgb/image_rect_color",
                "/zed/left/image_rect_color",
                "/zed/rgb/image_raw",
                "/zed/left/image_raw",
                "/zed/image_raw",
                "/zedm/zed_node/rgb/image_rect_color",
                "/zedm/zed_node/left/image_rect_color",
                "/camera/camera/color/image_raw",
                "/camera/color/image_raw",
            ]
            seen_topics = set()
            for t in cand_topics:
                if t and t not in seen_topics:
                    seen_topics.add(t)
                    self.subs_cam.append(self.create_subscription(Image, t, self.image_callback, qos))
            self.sub = self.subs_cam[0]
            self.pub_pantilt = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 10)
        else:
            self.sub = None
            self.pub_pantilt = None

        self.latest_frame = None
        self.running = True

        # Modelos de deteccion facial
        caffe_proto = os.path.join(pkg_root, "models", "deploy.prototxt")
        caffe_model = os.path.join(pkg_root, "models", "res10_300x300_ssd_iter_140000.caffemodel")
        sface = os.path.join(pkg_root, "models", "face_recognition_sface_2021dec.onnx")
        db_path = os.path.join(pkg_root, "config", "faces_db.pkl")

        self.recognizer = FaceRecognizer(
            caffe_proto_path=caffe_proto,
            caffe_model_path=caffe_model,
            sface_model_path=sface,
            db_path=db_path,
        )

        # Cargar valores actuales de tracking_params.yaml
        self.load_config()

        # Variables de control
        self.target_pan_deg = 0.0
        self.target_tilt_deg = 0.0
        self.last_track_time = 0.0
        self.filtered_face_cx = None
        self.filtered_face_cy = None
        self.save_feedback_timer = 0.0

    def load_config(self):
        """Carga los parametros desde YAML o valores predeterminados."""
        defaults = {
            "kp_pan": 0.025,
            "kp_tilt": 0.018,
            "max_pan_step": 1.2,
            "max_tilt_step": 0.8,
            "deadband_x": 25.0,
            "deadband_y": 25.0,
            "invert_pan": True,
            "invert_tilt": False,
            "filter_alpha": 0.45,
            "max_pan_limit": 50.0,
            "max_tilt_limit": 22.0,
            "min_tilt_limit": -18.0,
        }

        if os.path.exists(CONFIG_PATH):
            try:
                with open(CONFIG_PATH, "r") as f:
                    data = yaml.safe_load(f)
                    t = data.get("tracking", {})
                    defaults.update(t)
            except Exception as e:
                print(f"[Calibrator] Error leyendo {CONFIG_PATH}: {e}")

        self.cfg = defaults

    def save_config(self):
        """Persiste los parametros calibrados directamente en tracking_params.yaml."""
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        try:
            with open(CONFIG_PATH, "w") as f:
                yaml.dump({"tracking": self.cfg}, f, default_flow_style=False, sort_keys=False)
            self.save_feedback_timer = time.time()
            print(f"\n[+] Parametros de trackeo guardados con exito en:\n    {CONFIG_PATH}")
        except Exception as e:
            print(f"[!] Error guardando {CONFIG_PATH}: {e}")

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if cv_img.shape[1] > int(cv_img.shape[0] * 1.9):
                cv_img = cv_img[:, : cv_img.shape[1] // 2]
            self.latest_frame = cv_img
        except Exception:
            pass

    def update_tracking(self, results: list, now: float):
        if not results or self.pub_pantilt is None:
            self.filtered_face_cx = None
            self.filtered_face_cy = None
            return

        target = max(results, key=lambda r: r["bbox"][2] * r["bbox"][3])
        x, y, bw, bh = target["bbox"]

        raw_cx = x + bw / 2.0
        raw_cy = y + bh / 2.0

        alpha = float(self.cfg["filter_alpha"])
        if self.filtered_face_cx is None:
            self.filtered_face_cx = raw_cx
            self.filtered_face_cy = raw_cy
        else:
            self.filtered_face_cx = (1.0 - alpha) * self.filtered_face_cx + alpha * raw_cx
            self.filtered_face_cy = (1.0 - alpha) * self.filtered_face_cy + alpha * raw_cy

        err_x = self.filtered_face_cx - 320.0
        err_y = self.filtered_face_cy - 240.0

        db_x = float(self.cfg["deadband_x"])
        db_y = float(self.cfg["deadband_y"])
        kp_p = float(self.cfg["kp_pan"])
        kp_t = float(self.cfg["kp_tilt"])
        max_p_step = float(self.cfg["max_pan_step"])
        max_t_step = float(self.cfg["max_tilt_step"])
        inv_p = bool(self.cfg["invert_pan"])
        inv_t = bool(self.cfg["invert_tilt"])

        if abs(err_x) > db_x:
            step_pan = np.clip(err_x * kp_p, -max_p_step, max_p_step)
            self.target_pan_deg += (-step_pan if inv_p else step_pan)

        if abs(err_y) > db_y:
            step_tilt = np.clip(err_y * kp_t, -max_t_step, max_t_step)
            self.target_tilt_deg += (step_tilt if inv_t else -step_tilt)

        p_lim = float(self.cfg["max_pan_limit"])
        t_max = float(self.cfg["max_tilt_limit"])
        t_min = float(self.cfg["min_tilt_limit"])

        self.target_pan_deg = float(np.clip(self.target_pan_deg, -p_lim, p_lim))
        self.target_tilt_deg = float(np.clip(self.target_tilt_deg, t_min, t_max))

        # Publicar a 20 Hz
        if now - self.last_track_time >= 0.05:
            self.last_track_time = now
            msg = Vector3()
            msg.x = self.target_pan_deg
            msg.y = self.target_tilt_deg
            msg.z = 0.0
            self.pub_pantilt.publish(msg)

    def center_head(self):
        """Envia la cabeza a 0.0 grados."""
        self.target_pan_deg = 0.0
        self.target_tilt_deg = 0.0
        if self.pub_pantilt:
            msg = Vector3(x=0.0, y=0.0, z=0.0)
            self.pub_pantilt.publish(msg)
        print("[*] Cabezal centrado en (0.0°, 0.0°)")


def nothing(x):
    pass


def main():
    if ROS2_AVAILABLE:
        rclpy.init()
        calibrator = TrackingCalibrator()
    else:
        print("[!] ROS 2 no disponible.")
        return

    win_name = "Calibrador de Trackeo Facial (Base@Home)"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1020, 760)

    # Crear Trackbars interactivos
    cv2.createTrackbar("Vel Pan (KPx1000)", win_name, int(calibrator.cfg["kp_pan"] * 1000), 70, nothing)
    cv2.createTrackbar("Vel Tilt (KPx1000)", win_name, int(calibrator.cfg["kp_tilt"] * 1000), 50, nothing)
    cv2.createTrackbar("Max Paso (x10 deg)", win_name, int(calibrator.cfg["max_pan_step"] * 10), 30, nothing)
    cv2.createTrackbar("Zona Muerta X (px)", win_name, int(calibrator.cfg["deadband_x"]), 70, nothing)
    cv2.createTrackbar("Invertir Pan (0/1)", win_name, 1 if calibrator.cfg["invert_pan"] else 0, 1, nothing)
    cv2.createTrackbar("Invertir Tilt (0/1)", win_name, 1 if calibrator.cfg["invert_tilt"] else 0, 1, nothing)
    cv2.createTrackbar("Filtro Suavizado (%)", win_name, int(calibrator.cfg["filter_alpha"] * 100), 90, nothing)

    print("==================================================")
    print("  CALIBRADOR INTERACTIVO DE TRACKEO FACIAL        ")
    print("--------------------------------------------------")
    print("  Ajusta los sliders en la ventana para calibrar.")
    print("  [S] : Guardar configuracion en tracking_params.yaml")
    print("  [C] : Centrar cabeza en (0°, 0°)")
    print("  [R] : Restablecer valores iniciales")
    print("  [Q] : Salir")
    print("==================================================")

    while rclpy.ok() and calibrator.running:
        try:
            rclpy.spin_once(calibrator, timeout_sec=0.01)
        except Exception:
            break

        frame = calibrator.latest_frame
        if frame is None:
            time.sleep(0.02)
            continue

        now = time.time()

        # Leer valores actuales de los trackbars
        kp_p_val = max(1, cv2.getTrackbarPos("Vel Pan (KPx1000)", win_name)) / 1000.0
        kp_t_val = max(1, cv2.getTrackbarPos("Vel Tilt (KPx1000)", win_name)) / 1000.0
        step_val = max(1, cv2.getTrackbarPos("Max Paso (x10 deg)", win_name)) / 10.0
        db_x_val = max(5, cv2.getTrackbarPos("Zona Muerta X (px)", win_name))
        inv_p_val = bool(cv2.getTrackbarPos("Invertir Pan (0/1)", win_name))
        inv_t_val = bool(cv2.getTrackbarPos("Invertir Tilt (0/1)", win_name))
        alpha_val = max(10, cv2.getTrackbarPos("Filtro Suavizado (%)", win_name)) / 100.0

        calibrator.cfg["kp_pan"] = float(round(kp_p_val, 4))
        calibrator.cfg["kp_tilt"] = float(round(kp_t_val, 4))
        calibrator.cfg["max_pan_step"] = float(round(step_val, 2))
        calibrator.cfg["max_tilt_step"] = float(round(step_val * 0.7, 2))
        calibrator.cfg["deadband_x"] = float(db_x_val)
        calibrator.cfg["deadband_y"] = float(round(db_x_val * 0.8, 1))
        calibrator.cfg["invert_pan"] = inv_p_val
        calibrator.cfg["invert_tilt"] = inv_t_val
        calibrator.cfg["filter_alpha"] = float(round(alpha_val, 2))

        # Deteccion facial
        results = calibrator.recognizer.detect_and_recognize(frame)

        # Aplicar tracking con los parametros calibrados
        calibrator.update_tracking(results, now)

        # Dibujar interfaz de calibracion
        vis = frame.copy()
        h, w = vis.shape[:2]

        # 1. Cruz central y rectangulo de zona muerta (Deadband)
        cx, cy = 320, 240
        dbx = int(calibrator.cfg["deadband_x"])
        dby = int(calibrator.cfg["deadband_y"])
        cv2.rectangle(vis, (cx - dbx, cy - dby), (cx + dbx, cy + dby), (0, 255, 255), 1)
        cv2.putText(vis, f"Deadband ({dbx*2}x{dby*2}px)", (cx - dbx, cy - dby - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 255), 1)

        cv2.line(vis, (cx - 20, cy), (cx + 20, cy), (0, 200, 255), 1)
        cv2.line(vis, (cx, cy - 20), (cx, cy + 20), (0, 200, 255), 1)

        # 2. Dibujar rostro y flecha de correccion
        if results:
            target = max(results, key=lambda r: r["bbox"][2] * r["bbox"][3])
            x, y, bw, bh = target["bbox"]
            fcx, fcy = int(x + bw / 2), int(y + bh / 2)

            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.circle(vis, (fcx, fcy), 6, (0, 0, 255), -1)
            cv2.line(vis, (cx, cy), (fcx, fcy), (0, 165, 255), 2)

            err_x = fcx - cx
            err_y = fcy - cy
            cv2.putText(vis, f"Error: X={err_x:+d}px, Y={err_y:+d}px", (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 1)

        # 3. Barra de estado superior
        cv2.rectangle(vis, (0, 0), (w, 50), (25, 28, 32), -1)
        inv_str = "INV" if calibrator.cfg["invert_pan"] else "NORM"
        status_txt = f"Pan: {calibrator.target_pan_deg:+.1f} deg | Tilt: {calibrator.target_tilt_deg:+.1f} deg | KP_p: {calibrator.cfg['kp_pan']} | Sentido: {inv_str}"
        cv2.putText(vis, status_txt, (15, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 1)
        cv2.putText(vis, "[S] GUARDAR PARAMETROS | [C] CENTRAR | [R] RESET | [Q] SALIR", (15, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 220, 255), 1)

        # 4. Mensaje temporal al guardar
        if now - calibrator.save_feedback_timer < 2.5:
            cv2.rectangle(vis, (100, 180), (w - 100, 240), (0, 180, 50), -1)
            cv2.rectangle(vis, (100, 180), (w - 100, 240), (255, 255, 255), 2)
            cv2.putText(vis, "PARAMETROS GUARDADOS EN EL NODO DE ROSTROS!", (120, 218), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

        cv2.imshow(win_name, vis)

        key = cv2.waitKey(1) & 0xFF
        if key in [ord("q"), ord("Q"), 27]:
            break
        elif key in [ord("s"), ord("S")]:
            calibrator.save_config()
        elif key in [ord("c"), ord("C")]:
            calibrator.center_head()
        elif key in [ord("r"), ord("R")]:
            calibrator.load_config()
            cv2.setTrackbarPos("Vel Pan (KPx1000)", win_name, int(calibrator.cfg["kp_pan"] * 1000))
            cv2.setTrackbarPos("Vel Tilt (KPx1000)", win_name, int(calibrator.cfg["kp_tilt"] * 1000))
            cv2.setTrackbarPos("Max Paso (x10 deg)", win_name, int(calibrator.cfg["max_pan_step"] * 10))
            cv2.setTrackbarPos("Zona Muerta X (px)", win_name, int(calibrator.cfg["deadband_x"]))
            cv2.setTrackbarPos("Invertir Pan (0/1)", win_name, 1 if calibrator.cfg["invert_pan"] else 0)
            cv2.setTrackbarPos("Invertir Tilt (0/1)", win_name, 1 if calibrator.cfg["invert_tilt"] else 0)
            cv2.setTrackbarPos("Filtro Suavizado (%)", win_name, int(calibrator.cfg["filter_alpha"] * 100))
            print("[*] Parametros restablecidos a los valores guardados.")

    cv2.destroyAllWindows()
    try:
        calibrator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
