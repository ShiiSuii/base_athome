#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Prueba Interactiva de Reconocimiento Facial en Tiempo Real
# Subscribidor ROS 2 a la camara RealSense D435 con visualizacion directa en pantalla
# ==============================================================================

import argparse
import os
import sys
import time

import cv2
import numpy as np

# Asegurar que el modulo base_athome_perception sea visible
current_dir = os.path.dirname(os.path.abspath(__file__))
pkg_root = os.path.dirname(current_dir)
if pkg_root not in sys.path:
    sys.path.insert(0, pkg_root)

# Intentar importar ROS 2
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from base_athome_perception.face_recognizer import FaceRecognizer


class FaceRecognitionViewer(Node if ROS2_AVAILABLE else object):
    def __init__(
        self,
        topic: str = "/camera/camera/color/image_raw",
        use_camera_id: int = -1,
        caffe_proto: str = "",
        caffe_model: str = "",
        sface_model: str = "",
        db_path: str = "",
    ):
        if ROS2_AVAILABLE and use_camera_id == -1:
            super().__init__("face_recognition_viewer")
            self.get_logger().info(f"Iniciando visualizador facial sobre topico: {topic}")
            self.bridge = CvBridge()
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.sub = self.create_subscription(Image, topic, self.image_callback, qos)
        else:
            self.sub = None

        self.use_camera_id = use_camera_id
        self.cap = None
        if self.use_camera_id >= 0:
            print(f"Abriendo camara local /dev/video{use_camera_id}...")
            self.cap = cv2.VideoCapture(self.use_camera_id)

        # Inicializar motor facial
        self.recognizer = FaceRecognizer(
            caffe_proto_path=caffe_proto,
            caffe_model_path=caffe_model,
            sface_model_path=sface_model,
            db_path=db_path,
        )

        self.latest_frame = None
        self.fps = 0.0
        self.frame_count = 0
        self.fps_timer = time.time()
        self.running = True

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            if hasattr(self, "get_logger"):
                self.get_logger().error(f"Error convirtiendo imagen: {e}")

    def draw_hud(self, frame: np.ndarray, results: list) -> np.ndarray:
        """Dibuja interfaz grafica profesional sobre la imagen."""
        h, w = frame.shape[:2]
        vis = frame.copy()

        # 1. Barra superior semitransparente
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (w, 55), (20, 24, 30), -1)
        cv2.addWeighted(overlay, 0.75, vis, 0.25, 0, vis)

        # Titulo y estado
        cv2.putText(vis, "BASE@HOME - RECONOCIMIENTO FACIAL", (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        status_text = "EN VIVO"
        cv2.circle(vis, (w - 140, 20), 6, (0, 255, 0), -1)
        cv2.putText(vis, status_text, (w - 125, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.putText(vis, f"FPS: {self.fps:.1f}", (w - 230, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # Rostros conocidos
        known_list = list(self.recognizer.known_faces.keys())
        known_str = f"Registrados ({len(known_list)}): " + (", ".join(known_list[:4]) + ("..." if len(known_list) > 4 else "") if known_list else "Ninguno")
        cv2.putText(vis, known_str, (15, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 200, 220), 1)

        # 2. Bounding boxes y etiquetas de cada rostro
        for res in results:
            x, y, bw, bh = res["bbox"]
            name = res["name"]
            sim = res["similarity"]

            # Color: Verde si es reconocido, Amarillo/Naranja si es desconocido
            if name != "Desconocido":
                color = (0, 230, 70)  # Verde brillante
                label = f"{name} ({int(sim * 100)}%)"
            else:
                color = (0, 165, 255)  # Naranja
                label = f"Desconocido ({int(sim * 100)}%)"

            # Esquinas destacadas estilo HUD
            line_len = int(min(bw, bh) * 0.25)
            thick = 2
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), color, 1)
            cv2.line(vis, (x, y), (x + line_len, y), color, thick + 1)
            cv2.line(vis, (x, y), (x, y + line_len), color, thick + 1)
            cv2.line(vis, (x + bw, y), (x + bw - line_len, y), color, thick + 1)
            cv2.line(vis, (x + bw, y), (x + bw, y + line_len), color, thick + 1)
            cv2.line(vis, (x, y + bh), (x + line_len, y + bh), color, thick + 1)
            cv2.line(vis, (x, y + bh), (x, y + bh - line_len), color, thick + 1)
            cv2.line(vis, (x + bw, y + bh), (x + bw - line_len, y + bh), color, thick + 1)
            cv2.line(vis, (x + bw, y + bh), (x + bw, y + bh - line_len), color, thick + 1)

            # Etiqueta sobre el rostro con fondo solido
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            lbl_y = max(y - 8, th + 8)
            cv2.rectangle(vis, (x, lbl_y - th - 6), (x + tw + 10, lbl_y + 2), color, -1)
            cv2.putText(vis, label, (x + 5, lbl_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (10, 15, 20), 2)

        # 3. Barra inferior de instrucciones
        overlay_bot = vis.copy()
        cv2.rectangle(overlay_bot, (0, h - 35), (w, h), (20, 24, 30), -1)
        cv2.addWeighted(overlay_bot, 0.8, vis, 0.2, 0, vis)
        shortcuts = "[ESPACIO / E] Registrar rostro | [C] Limpiar registro | [Q] Salir"
        cv2.putText(vis, shortcuts, (15, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1)

        return vis

    def run(self):
        window_name = "Base@Home - Reconocimiento Facial en Tiempo Real"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 720)

        print("==================================================")
        print(" Sistema de Reconocimiento Facial Iniciado")
        print(" Presiona [ESPACIO] o [E] para registrar un rostro")
        print(" Presiona [C] para limpiar el registro de nombres")
        print(" Presiona [Q] para salir")
        print("==================================================")

        last_fps_calc = time.time()
        fps_counter = 0

        while self.running:
            if ROS2_AVAILABLE and self.sub:
                rclpy.spin_once(self, timeout_sec=0.01)
                frame = self.latest_frame
            elif self.cap:
                ret, frame = self.cap.read()
                if not ret:
                    time.sleep(0.02)
                    continue
            else:
                frame = None

            if frame is None:
                wait_screen = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(
                    wait_screen,
                    "Esperando se;al de camara RealSense D435...",
                    (50, 240),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 200, 255),
                    2,
                )
                cv2.imshow(window_name, wait_screen)
                key = cv2.waitKey(30) & 0xFF
                if key in [ord("q"), ord("Q"), 27]:
                    break
                continue

            # Inferencia y Deteccion Facial
            results = self.recognizer.detect_and_recognize(frame)

            # Calculo de FPS
            fps_counter += 1
            now = time.time()
            if now - last_fps_calc >= 0.5:
                self.fps = fps_counter / (now - last_fps_calc)
                fps_counter = 0
                last_fps_calc = now

            # Dibujar HUD
            display_frame = self.draw_hud(frame, results)
            cv2.imshow(window_name, display_frame)

            # Manejo de teclado
            key = cv2.waitKey(1) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                print("[*] Saliendo del visualizador...")
                break
            elif key in [ord("e"), ord("E"), 32]:  # E o ESPACIO
                self.enroll_interactive(frame, results)
            elif key in [ord("c"), ord("C")]:
                self.recognizer.clear_database()

        cv2.destroyAllWindows()
        if self.cap:
            self.cap.release()

    def enroll_interactive(self, current_frame: np.ndarray, results: list):
        """Pausa y solicita el nombre para registrar el rostro actual."""
        if not results:
            print("\n[!] No hay ningun rostro detectado frente a la camara.")
            return

        print("\n--------------------------------------------------")
        print(" REGISTRO DE NUEVA PERSONA:")
        default_name = f"Persona_{len(self.recognizer.known_faces) + 1}"
        print(f" Rostro detectado con confianza: {int(results[0]['det_score'] * 100)}%")
        print(f" Ingrese el nombre para esta persona [Enter para '{default_name}']: ", end="", flush=True)

        try:
            import select
            if sys.stdin in select.select([sys.stdin], [], [], 15)[0]:
                user_name = sys.stdin.readline().strip()
            else:
                user_name = ""
        except Exception:
            user_name = input()

        name = user_name if user_name else default_name
        success = self.recognizer.enroll_face(current_frame, name)
        if success:
            print(f"[+] ¡Registrado exitosamente! Reconociendo ahora a: '{name}'")
        print("--------------------------------------------------\n")


def main():
    parser = argparse.ArgumentParser(description="Reconocimiento Facial en Tiempo Real para RoboCup @Home")
    parser.add_argument("--topic", default="/camera/camera/color/image_raw", help="Topico ROS 2 de imagen RGB")
    parser.add_argument("--webcam", type=int, default=-1, help="Usar camara USB local /dev/videoX en lugar de ROS 2")
    args = parser.parse_args()

    pkg_dir = "/home/athome/base_athome_ws/src/base_athome_perception"
    caffe_proto = os.path.join(pkg_dir, "models", "deploy.prototxt")
    caffe_model = os.path.join(pkg_dir, "models", "res10_300x300_ssd_iter_140000.caffemodel")
    sface = os.path.join(pkg_dir, "models", "face_recognition_sface_2021dec.onnx")
    db_path = os.path.join(pkg_dir, "config", "faces_db.pkl")

    if ROS2_AVAILABLE and args.webcam == -1:
        rclpy.init()
        viewer = FaceRecognitionViewer(
            topic=args.topic,
            use_camera_id=-1,
            caffe_proto=caffe_proto,
            caffe_model=caffe_model,
            sface_model=sface,
            db_path=db_path,
        )
        try:
            viewer.run()
        finally:
            viewer.destroy_node()
            rclpy.shutdown()
    else:
        viewer = FaceRecognitionViewer(
            use_camera_id=args.webcam if args.webcam >= 0 else 0,
            caffe_proto=caffe_proto,
            caffe_model=caffe_model,
            sface_model=sface,
            db_path=db_path,
        )
        viewer.run()


if __name__ == "__main__":
    main()
