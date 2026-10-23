#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Sistema Interactivo de Reconocimiento Facial y Registro por Voz
# RoboCup @Home - Interaccion Natural por Audio (Piper TTS EN + Whisper ASR EN)
# Audio Output: Parlante Bluetooth (PipeWire / PulseAudio)
# Audio Input: Microfono Direccional RØDE VideoMic GO II (Jetson Orin Nano)
# ==============================================================================

import argparse
import os
import re
import sys
import threading
import time
from typing import List, Optional

import cv2
import numpy as np

# Asegurar que el modulo base_athome_perception sea visible
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
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from base_athome_perception.face_recognizer import FaceRecognizer
from base_athome_perception.speaker import RoboCupSpeaker


def extract_person_name(text: str) -> str:
    """Extrae de forma robusta el nombre de una persona a partir de la transcripcion oral en ingles."""
    text = text.strip().strip(".!?,")
    if not text:
        return ""

    # Patrones comunes de presentacion en RoboCup @Home
    patterns = [
        r"(?:my name is|i am|i'm|call me|this is|it's|they call me)\s+([A-Za-z]+(?:\s+[A-Za-z]+)?)",
        r"(?:hello|hi|hey),?\s*(?:my name is|i am|i'm)?\s*([A-Za-z]+(?:\s+[A-Za-z]+)?)",
    ]
    for pat in patterns:
        m = re.search(pat, text, re.IGNORECASE)
        if m:
            candidate = m.group(1).strip().title()
            if candidate.lower() not in ["a", "the", "an", "robot", "ready", "here", "nothing", "yes", "no"]:
                return candidate

    # Si es una respuesta corta (1 o 2 palabras, ej: "David" o "Sarah Connor")
    words = [w.strip(".!?,") for w in text.split() if w.strip(".!?,")]
    if 1 <= len(words) <= 2:
        candidate = " ".join(words).title()
        if candidate.lower() not in ["yes", "no", "hello", "hi", "hey", "what", "ok", "okay"]:
            return candidate

    # Respaldo: ultima palabra si es alfabetica
    if words and words[-1].isalpha():
        return words[-1].title()

    return text.title()


class FaceRecognitionViewer(Node if ROS2_AVAILABLE else object):
    """Nodo interactivo de reconocimiento facial y registro por voz para RoboCup @Home."""

    STATE_IDLE = 0
    STATE_PROMPT_NAME = 1
    STATE_WAIT_NAME = 2
    STATE_CONFIRM_NAME = 3

    def __init__(
        self,
        topic: str = "/camera/camera/color/image_raw",
        use_camera_id: int = -1,
        caffe_proto: str = "",
        caffe_model: str = "",
        sface_model: str = "",
        db_path: str = "",
        piper_bin: str = "/home/athome/piper_bin/piper/piper",
        piper_model: str = "/home/athome/piper_bin/voices/en/en_US-lessac-medium.onnx",
    ):
        if ROS2_AVAILABLE and use_camera_id == -1:
            super().__init__("face_recognition_viewer")
            self.get_logger().info(f"Subscribiendo a camara RealSense: {topic}")
            self.bridge = CvBridge()
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.sub = self.create_subscription(Image, topic, self.image_callback, qos)

            # Subscripcion a transcripcion de voz (Whisper desde la Jetson)
            self.sub_voice = self.create_subscription(
                String, "/voice/transcription", self.voice_callback, 10
            )
            self.sub_voice_status = self.create_subscription(
                String, "/voice/status", self.voice_status_callback, 10
            )
        else:
            self.sub = None

        self.use_camera_id = use_camera_id
        self.cap = None
        if self.use_camera_id >= 0:
            print(f"Abriendo camara local /dev/video{use_camera_id}...")
            self.cap = cv2.VideoCapture(self.use_camera_id)

        # Motor de Reconocimiento Facial (ResNet-10 + SFace 128-D)
        self.recognizer = FaceRecognizer(
            caffe_proto_path=caffe_proto,
            caffe_model_path=caffe_model,
            sface_model_path=sface_model,
            db_path=db_path,
        )

        # Sintetizador de Voz Neuronal Piper TTS (Voz en Ingles Natural)
        self.speaker = RoboCupSpeaker(piper_bin=piper_bin, model_path=piper_model)

        # Variables de estado y recepcion
        self.latest_frame = None
        self.fps = 0.0
        self.running = True

        # Estado de interaccion por voz
        self.state = self.STATE_IDLE
        self.current_banner_text = ""
        self.state_start_time = 0.0
        self.collected_features: List[np.ndarray] = []
        self.voice_text: Optional[str] = None
        self.voice_status = "IDLE"
        self.pending_name = ""

        # Auto-Prompting
        self.auto_prompt_enabled = True
        self.unknown_face_start_time = None
        self.last_registered_time = 0.0

    def image_callback(self, msg: Image):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            if hasattr(self, "get_logger"):
                self.get_logger().error(f"Error convirtiendo imagen: {e}")

    def voice_callback(self, msg: String):
        text = msg.data.strip()
        if text:
            if hasattr(self, "get_logger"):
                self.get_logger().info(f"[Whisper Voice Input]: \"{text}\"")
            self.voice_text = text

    def voice_status_callback(self, msg: String):
        self.voice_status = msg.data.strip()

    def start_voice_registration(self, reason: str = "manual"):
        """Inicia el protocolo RoboCup @Home para registrar a una nueva persona hablando."""
        if self.state != self.STATE_IDLE:
            return

        self.state = self.STATE_PROMPT_NAME
        self.state_start_time = time.time()
        self.collected_features = []
        self.voice_text = None
        self.pending_name = ""
        self.current_banner_text = "Hello! Please look at the camera. What is your name?"
        print(f"\n[RoboCup @Home] Iniciando registro por voz ({reason})...")
        print(f"[Robot]: \"{self.current_banner_text}\"")
        self.speaker.speak(self.current_banner_text)

    def draw_hud(self, frame: np.ndarray, results: list) -> np.ndarray:
        """Dibuja interfaz grafica profesional sobre la imagen con soporte interactivo de voz."""
        h, w = frame.shape[:2]
        vis = frame.copy()
        now = time.time()

        # 1. Barra superior principal
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (w, 55), (20, 24, 30), -1)
        cv2.addWeighted(overlay, 0.8, vis, 0.2, 0, vis)

        cv2.putText(vis, "BASE@HOME - RECONOCIMIENTO Y REGISTRO POR VOZ", (15, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (255, 255, 255), 2)
        cv2.circle(vis, (w - 140, 20), 6, (0, 255, 0), -1)
        cv2.putText(vis, "EN VIVO", (w - 125, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
        cv2.putText(vis, f"FPS: {self.fps:.1f}", (w - 230, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (200, 200, 200), 1)

        known_list = list(self.recognizer.known_faces.keys())
        known_str = f"Rostros ({len(known_list)}): " + (", ".join(known_list[:5]) + ("..." if len(known_list) > 5 else "") if known_list else "Ninguno")
        cv2.putText(vis, known_str, (15, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 205, 225), 1)

        # 2. Bounding boxes y etiquetas de rostros
        for res in results:
            x, y, bw, bh = res["bbox"]
            name = res["name"]
            sim = res["similarity"]

            if name != "Desconocido":
                color = (0, 230, 70)  # Verde
                label = f"{name} ({int(sim * 100)}%)"
            else:
                color = (0, 165, 255)  # Naranja
                label = f"Desconocido ({int(sim * 100)}%)"

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

            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            lbl_y = max(y - 8, th + 8)
            cv2.rectangle(vis, (x, lbl_y - th - 6), (x + tw + 10, lbl_y + 2), color, -1)
            cv2.putText(vis, label, (x + 5, lbl_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (10, 15, 20), 2)

        # 3. Banner interactivo de Estado de Voz (RoboCup @Home)
        if self.state != self.STATE_IDLE:
            banner_h = 45
            banner_y = 65
            banner_bg = vis.copy()

            if self.state == self.STATE_PROMPT_NAME:
                # Robot hablando (Cyan)
                b_color = (180, 120, 20)
                icon_txt = "[ROBOT TALKING]"
                msg_txt = self.current_banner_text
            elif self.state == self.STATE_WAIT_NAME:
                # Robot escuchando el microfono (Amarillo/Dorado con parpadeo)
                pulse = int(abs(np.sin(now * 4)) * 50)
                b_color = (0, 160 + pulse, 240)
                elapsed = now - self.state_start_time
                remaining = max(0, int(15.0 - elapsed))
                icon_txt = f"[LISTENING MIC - {remaining}s]"
                if self.voice_status == "LISTENING":
                    msg_txt = "Detecting voice... Speak your name clearly!"
                elif self.voice_status == "PROCESSING":
                    msg_txt = "Transcribing with Whisper AI..."
                else:
                    msg_txt = "Say: 'My name is [Your Name]'"
            elif self.state == self.STATE_CONFIRM_NAME:
                # Confirmacion exitosa (Verde esmeralda)
                b_color = (20, 160, 40)
                icon_txt = "[SUCCESS]"
                msg_txt = self.current_banner_text

            cv2.rectangle(banner_bg, (15, banner_y), (w - 15, banner_y + banner_h), b_color, -1)
            cv2.addWeighted(banner_bg, 0.85, vis, 0.15, 0, vis)
            cv2.rectangle(vis, (15, banner_y), (w - 15, banner_y + banner_h), (255, 255, 255), 1)

            cv2.putText(vis, icon_txt, (25, banner_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(vis, msg_txt, (210, banner_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)

        # 4. Barra inferior de atajos y configuracion
        overlay_bot = vis.copy()
        cv2.rectangle(overlay_bot, (0, h - 38), (w, h), (20, 24, 30), -1)
        cv2.addWeighted(overlay_bot, 0.85, vis, 0.15, 0, vis)

        auto_str = "[A] Auto-Voz: ON" if self.auto_prompt_enabled else "[A] Auto-Voz: OFF"
        shortcuts = f"[V / ESPACIO] Registro por Voz | [T] Escribir | {auto_str} | [C] Borrar DB | [Q] Salir"
        cv2.putText(vis, shortcuts, (15, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (220, 220, 220), 1)

        return vis

    def enroll_by_typing(self, current_frame: np.ndarray, results: list):
        """Metodo de respaldo: Registro manual por teclado."""
        if not results:
            print("\n[!] No hay ningun rostro detectado frente a la camara.")
            return

        print("\n--------------------------------------------------")
        print(" REGISTRO MANUAL POR TECLADO:")
        default_name = f"Persona_{len(self.recognizer.known_faces) + 1}"
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
            msg = f"Nice to meet you, {name}!"
            print(f"[+] Registrado: '{name}'")
            self.speaker.speak(msg)
        print("--------------------------------------------------\n")

    def run(self):
        window_name = "Base@Home - Reconocimiento Facial y Registro por Voz"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 720)

        print("==================================================")
        print("  Base@Home - Sistema de Reconocimiento y Voz")
        print("  - Voz Neuronal: Piper TTS (en_US-lessac-medium)")
        print("  - Salida de Audio: Parlante Bluetooth (PPA 205)")
        print("  - Entrada de Audio: RØDE VideoMic + Whisper AI")
        print("--------------------------------------------------")
        print("  [V] o [ESPACIO] : Iniciar registro oral por voz")
        print("  [T]             : Registro manual por teclado")
        print("  [A]             : Activar/desactivar auto-registro")
        print("  [C]             : Limpiar base de datos de rostros")
        print("  [Q]             : Salir")
        print("==================================================")

        last_fps_calc = time.time()
        fps_counter = 0

        while self.running and (not ROS2_AVAILABLE or rclpy.ok()):
            if ROS2_AVAILABLE and self.sub:
                try:
                    rclpy.spin_once(self, timeout_sec=0.01)
                except Exception:
                    break
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

            now = time.time()

            # Inferencia y Deteccion Facial
            results = self.recognizer.detect_and_recognize(frame)

            # Acumular embeddings del rostro mas grande si estamos en proceso de registro
            if self.state in [self.STATE_PROMPT_NAME, self.STATE_WAIT_NAME]:
                if results and len(self.collected_features) < 10:
                    largest = max(results, key=lambda r: r["bbox"][2] * r["bbox"][3])
                    if largest.get("feature") is not None:
                        self.collected_features.append(largest["feature"])

            # ----------------------------------------------------
            # MAQUINA DE ESTADOS DE REGISTRO POR VOZ (RoboCup @Home)
            # ----------------------------------------------------
            if self.state == self.STATE_IDLE:
                # Comprobar si hay una persona desconocida presente
                has_unknown = any(r["name"] == "Desconocido" for r in results)
                if has_unknown and self.auto_prompt_enabled and (now - self.last_registered_time > 12.0):
                    if self.unknown_face_start_time is None:
                        self.unknown_face_start_time = now
                    elif now - self.unknown_face_start_time >= 2.0:
                        # Ha permanecido frente a la camara por 2 segundos -> Auto-iniciar saludo
                        self.unknown_face_start_time = None
                        self.start_voice_registration(reason="auto-deteccion")
                else:
                    self.unknown_face_start_time = None

            elif self.state == self.STATE_PROMPT_NAME:
                # Esperar a que el robot termine de hablar la pregunta inicial
                if not self.speaker.is_speaking and (now - self.state_start_time > 1.2):
                    self.state = self.STATE_WAIT_NAME
                    self.state_start_time = now
                    self.voice_text = None
                    print("[Robot]: Escuchando respuesta del usuario en el microfono...")

            elif self.state == self.STATE_WAIT_NAME:
                # Esperar transcripcion de Whisper o timeout
                if self.voice_text:
                    user_speech = self.voice_text
                    self.voice_text = None
                    extracted = extract_person_name(user_speech)
                    print(f"[ASR Final]: \"{user_speech}\" -> Nombre extraido: '{extracted}'")

                    if extracted and len(extracted) >= 2:
                        self.pending_name = extracted
                        # Registrar todas las muestras recolectadas
                        if self.collected_features:
                            for feat in self.collected_features:
                                self.recognizer.enroll_feature(feat, self.pending_name)
                        else:
                            self.recognizer.enroll_face(frame, self.pending_name)

                        self.current_banner_text = f"Nice to meet you, {self.pending_name}! I have registered your face."
                        print(f"[Robot]: \"{self.current_banner_text}\"")
                        self.speaker.speak(self.current_banner_text)
                        self.state = self.STATE_CONFIRM_NAME
                        self.state_start_time = now
                    else:
                        print("[ASR]: No se pudo extraer un nombre valido. Reintentando...")
                        self.speaker.speak("I didn't catch your name. What is your name?")
                        self.state_start_time = now

                elif now - self.state_start_time > 15.0:
                    print("[ASR]: Tiempo de espera agotado sin respuesta.")
                    self.speaker.speak("I did not hear you. Let's try again later.")
                    self.state = self.STATE_IDLE
                    self.last_registered_time = now

            elif self.state == self.STATE_CONFIRM_NAME:
                # Esperar a que termine la frase de confirmacion
                if not self.speaker.is_speaking and (now - self.state_start_time > 1.5):
                    self.state = self.STATE_IDLE
                    self.last_registered_time = now
                    self.pending_name = ""
                    self.collected_features = []

            # Calculo de FPS
            fps_counter += 1
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
                print("[*] Saliendo del sistema...")
                break
            elif key in [ord("v"), ord("V"), 32]:  # V o ESPACIO
                self.start_voice_registration(reason="tecla manual")
            elif key in [ord("t"), ord("T")]:
                self.enroll_by_typing(frame, results)
            elif key in [ord("a"), ord("A")]:
                self.auto_prompt_enabled = not self.auto_prompt_enabled
                state_str = "ACTIVADO" if self.auto_prompt_enabled else "DESACTIVADO"
                print(f"[*] Auto-Registro por voz: {state_str}")
            elif key in [ord("c"), ord("C")]:
                self.recognizer.clear_database()
                print("[*] Base de datos facial limpiada.")

        cv2.destroyAllWindows()
        self.speaker.shutdown()
        if self.cap:
            self.cap.release()


def main():
    parser = argparse.ArgumentParser(description="Reconocimiento Facial y Registro por Voz para RoboCup @Home")
    parser.add_argument("--topic", default="/camera/camera/color/image_raw", help="Topico ROS 2 de imagen RGB")
    parser.add_argument("--webcam", type=int, default=-1, help="Usar camara USB local /dev/videoX en lugar de ROS 2")
    args = parser.parse_args()

    pkg_dir = "/home/athome/base_athome_ws/src/base_athome_perception"
    caffe_proto = os.path.join(pkg_dir, "models", "deploy.prototxt")
    caffe_model = os.path.join(pkg_dir, "models", "res10_300x300_ssd_iter_140000.caffemodel")
    sface = os.path.join(pkg_dir, "models", "face_recognition_sface_2021dec.onnx")
    db_path = os.path.join(pkg_dir, "config", "faces_db.pkl")

    piper_bin = "/home/athome/piper_bin/piper/piper"
    piper_voice = "/home/athome/piper_bin/voices/en/en_US-lessac-medium.onnx"

    if ROS2_AVAILABLE and args.webcam == -1:
        rclpy.init()
        viewer = FaceRecognitionViewer(
            topic=args.topic,
            use_camera_id=-1,
            caffe_proto=caffe_proto,
            caffe_model=caffe_model,
            sface_model=sface,
            db_path=db_path,
            piper_bin=piper_bin,
            piper_model=piper_voice,
        )
        try:
            viewer.run()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                viewer.destroy_node()
            except Exception:
                pass
            if rclpy.ok():
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
    else:
        viewer = FaceRecognitionViewer(
            use_camera_id=args.webcam if args.webcam >= 0 else 0,
            caffe_proto=caffe_proto,
            caffe_model=caffe_model,
            sface_model=sface,
            db_path=db_path,
            piper_bin=piper_bin,
            piper_model=piper_voice,
        )
        try:
            viewer.run()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
