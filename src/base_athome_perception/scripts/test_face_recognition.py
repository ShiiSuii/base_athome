#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Reconocimiento Facial, Memoria Biometrica y Trackeo del Rostro
# RoboCup @Home - Interaccion Natural por Audio (Piper TTS EN + Whisper ASR EN)
# Audio Output: Parlante Bluetooth PPA 205 (Intel NUC PipeWire)
# Audio Input: Microfono Direccional RØDE VideoMic GO II (Jetson Orin Nano)
# Hardware Cabezal: Pan-Tilt Dynamixel (Servos ID 1 y 2)
# ==============================================================================

import argparse
import datetime
import json
import os
import re
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

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
    from std_srvs.srv import Trigger
    from geometry_msgs.msg import Vector3
    from cv_bridge import CvBridge
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

from base_athome_perception.face_recognizer import FaceRecognizer
from base_athome_perception.speaker import RoboCupSpeaker


STOP_WORDS = {
    "a", "the", "an", "robot", "ready", "here", "nothing",
    "yes", "no", "si", "hola", "buenas", "hello", "hi", "hey",
    "what", "ok", "okay", "please", "say", "followed", "by",
    "followed by", "name", "your", "my", "is", "am", "me", "llamo",
    "unknown", "desconocido", "person", "persona", "speak", "clearly",
    "it", "its", "this", "that", "there", "who", "you", "your name"
}


def extract_person_name(text: str) -> str:
    """Extrae de forma robusta el nombre de una persona a partir de la transcripcion oral (EN/ES)."""
    text = text.strip().strip(".!?,")
    if not text:
        return ""

    # Limpiar saludos iniciales o muletillas conversacionales
    text_clean = re.sub(
        r"^(?:hello|hi|hey|hola|buenas|good morning|good afternoon)\b[,\s]*",
        "",
        text,
        flags=re.IGNORECASE,
    ).strip()

    # Patrones comunes de presentacion en RoboCup @Home (Ingles y Espanol)
    patterns = [
        r"(?:my name is|i am|i'm|call me|this is|it's|they call me)\s+([A-Za-z]+(?:\s+[A-Za-z]+)?)",
        r"(?:me llamo|mi nombre es|yo soy|soy)\s+([A-Za-z]+(?:\s+[A-Za-z]+)?)",
    ]
    for pat in patterns:
        m = re.search(pat, text_clean, re.IGNORECASE)
        if m:
            candidate = m.group(1).strip().title()
            # Limpiar prefijo accidental 'Followed By'
            candidate = re.sub(r"^(?:Followed By|Followed|By)\s*", "", candidate, flags=re.IGNORECASE).strip()
            if candidate and candidate.lower() not in STOP_WORDS:
                return candidate

    # Filtrar palabras residuales y respuestas directas (ej: 'David')
    cleaned = re.sub(r"^(?:followed by|por favor|please say|say)\s*", "", text_clean, flags=re.IGNORECASE).strip()
    words = [w.strip(".!?,") for w in cleaned.split() if w.strip(".!?,")]
    if 1 <= len(words) <= 2:
        candidate = " ".join(words).title()
        if candidate.lower() not in STOP_WORDS and len(candidate) >= 3:
            return candidate

    if words and words[-1].isalpha() and words[-1].lower() not in STOP_WORDS and len(words[-1]) >= 3:
        return words[-1].title()

    return ""



class FaceRecognitionViewer(Node if ROS2_AVAILABLE else object):
    """Nodo interactivo de reconocimiento facial, memoria biometrica y trackeo para RoboCup @Home."""

    STATE_IDLE = 0
    STATE_PROMPT_NAME = 1
    STATE_WAIT_NAME = 2
    STATE_RECORDING_FACE = 3
    STATE_CONFIRM_NAME = 4

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
            self.get_logger().info(f"Subscribiendo a camara del cabezal (ZED Mini / RealSense): {topic}")
            self.bridge = CvBridge()
            qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
            # Soporte multi-camara: ZED Mini / ZEDm / RealSense / UVC
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

            # Subscripcion a transcripcion de voz (Whisper desde la Jetson)
            self.sub_voice = self.create_subscription(
                String, "/voice/transcription", self.voice_callback, 10
            )
            self.sub_voice_status = self.create_subscription(
                String, "/voice/status", self.voice_status_callback, 10
            )

            # Publicador para trackeo del Pan-Tilt (Dynamixel)
            self.pub_pantilt = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 10)

            # Cliente de servicio para disparo de escucha ASR
            self.cli_trigger = self.create_client(Trigger, "/voice/listen_trigger")

            # --- Interfaces Estandar FBOT@Home HRI ---
            # Topico de entrada: pedir al robot que diga una frase (TTS Piper)
            self.sub_say = self.create_subscription(
                String, "/hri/say_something", self.say_something_callback, 10
            )
            # Topico de salida: streaming del HUD grafico para pantallas HDMI / Foxglove / RViz2
            self.pub_ui_display = self.create_publisher(Image, "/hri/ui_display", 10)
            # Topico de salida: metadatos JSON de personas detectadas y reconocidas
            self.pub_detected_persons = self.create_publisher(String, "/hri/detected_persons", 10)
            # Servicio: reproducir tono de aviso acustico de turn-taking (beep)
            self.srv_play_beep = self.create_service(Trigger, "/hri/play_beep", self.play_beep_service_callback)
            # Servicio: iniciar protocolo guiado de introduccion y registro de persona
            self.srv_introduce = self.create_service(Trigger, "/hri/introduce_person", self.introduce_person_service_callback)
            # Servicio: limpiar memoria biometrica
            self.srv_clear_memory = self.create_service(Trigger, "/hri/clear_memory", self.clear_memory_service_callback)
        else:
            self.sub = None
            self.pub_pantilt = None
            self.cli_trigger = None
            self.sub_say = None
            self.pub_ui_display = None
            self.pub_detected_persons = None
            self.srv_play_beep = None
            self.srv_introduce = None
            self.srv_clear_memory = None

        self.use_camera_id = use_camera_id
        self.cap = None
        if self.use_camera_id >= 0:
            print(f"Abriendo camara local /dev/video{use_camera_id}...")
            self.cap = cv2.VideoCapture(self.use_camera_id)

        # Motor de Memoria Facial Multi-Persona (ResNet-10 + SFace 128-D)
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

        # Maquina de estados de interaccion
        self.state = self.STATE_IDLE
        self.current_banner_text = ""
        self.state_start_time = 0.0
        self.voice_text: Optional[str] = None
        self.voice_status = "IDLE"
        self.pending_name = ""

        # Rutina de grabacion biometrica multi-muestra
        self.recording_samples: List[np.ndarray] = []
        self.best_thumbnail_crop: Optional[np.ndarray] = None
        self.best_sharpness: float = 0.0
        self.last_sample_time: float = 0.0
        self.target_sample_count: int = 10

        # Control de saludos y encuentros recurrentes
        self.greeting_cooldowns: Dict[str, float] = {}
        self.active_banner_override: Optional[Tuple[str, float]] = None

        # Auto-Prompting
        self.auto_prompt_enabled = True
        self.unknown_face_start_time = None
        self.last_registered_time = 0.0

        # Parametros de Trackeo Facial (Cargados desde tracking_params.yaml)
        self.tracking_config_path = os.path.join(pkg_root, "config", "tracking_params.yaml")
        self.last_config_check_time = 0.0
        self.last_config_mtime = 0.0
        self.tracking_cfg = {}
        self.load_tracking_config()

        self.tracking_enabled = True
        self.target_pan_deg = 0.0
        self.target_tilt_deg = 0.0
        self.last_track_time = 0.0
        self.tracked_bbox: Optional[Tuple[int, int, int, int]] = None
        self.filtered_face_cx: Optional[float] = None
        self.filtered_face_cy: Optional[float] = None

    def load_tracking_config(self):
        """Carga parametros de calibracion desde tracking_params.yaml."""
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
        if os.path.exists(self.tracking_config_path):
            try:
                with open(self.tracking_config_path, "r") as f:
                    data = yaml.safe_load(f)
                    t = data.get("tracking", {})
                    defaults.update(t)
                self.last_config_mtime = os.path.getmtime(self.tracking_config_path)
            except Exception as e:
                if hasattr(self, "get_logger"):
                    self.get_logger().warn(f"Error cargando tracking_params.yaml: {e}")
        self.tracking_cfg = defaults

    def check_config_reload(self, now: float):
        """Verifica cada 1.5s si el archivo de calibracion fue actualizado para recargarlo en caliente."""
        if now - self.last_config_check_time < 1.5:
            return
        self.last_config_check_time = now

        if os.path.exists(self.tracking_config_path):
            try:
                mtime = os.path.getmtime(self.tracking_config_path)
                if mtime > self.last_config_mtime:
                    self.load_tracking_config()
                    print("\n[+] [FaceTracking] ¡Parametros actualizados en caliente desde tracking_params.yaml!")
            except Exception:
                pass

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Soporte estéreo Side-by-Side (ej. ZED Mini 2560x720 / 1344x376): recortar ojo izquierdo
            if cv_img.shape[1] > int(cv_img.shape[0] * 1.9):
                cv_img = cv_img[:, : cv_img.shape[1] // 2]
            self.latest_frame = cv_img
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

    def trigger_mic_listening(self):
        """Solicita a whisper_node iniciar grabacion activa inmediatamente."""
        try:
            if hasattr(self, "cli_trigger") and self.cli_trigger and self.cli_trigger.service_is_ready():
                req = Trigger.Request()
                self.cli_trigger.call_async(req)
        except Exception:
            pass

    # --- Callbacks de Interfaces FBOT@Home HRI ---
    def say_something_callback(self, msg: String):
        """Callback de FBOT /hri/say_something: sintetiza voz con Piper TTS y muestra subtitulo."""
        text = msg.data.strip()
        if text:
            if hasattr(self, "get_logger"):
                self.get_logger().info(f"[HRI Say]: \"{text}\"")
            self.speaker.speak(text)
            self.active_banner_override = (text, time.time() + 4.0)

    def play_beep_service_callback(self, request, response):
        """Servicio /hri/play_beep: reproduce el tono acustico turn-taking."""
        self.speaker.play_beep(blocking=False)
        response.success = True
        response.message = "Turn-taking chime beep played."
        return response

    def introduce_person_service_callback(self, request, response):
        """Servicio /hri/introduce_person: inicia la rutina interactiva de presentacion y registro."""
        if self.state == self.STATE_IDLE:
            self.start_voice_registration(reason="hri_service")
            response.success = True
            response.message = "Face introduction routine started."
        else:
            response.success = False
            response.message = f"Robot is currently busy (state: {self.state})."
        return response

    def clear_memory_service_callback(self, request, response):
        """Servicio /hri/clear_memory: borra todos los perfiles de memoria facial."""
        self.recognizer.clear_database()
        response.success = True
        response.message = "Biometric memory cleared."
        return response

    def update_face_tracking(self, results: list, now: float):
        """Calcula y envia comandos reactivos al cabezal usando los parametros calibrados."""
        if not self.tracking_enabled or not results or self.pub_pantilt is None:
            self.tracked_bbox = None
            self.filtered_face_cx = None
            self.filtered_face_cy = None
            return

        # Seleccionar el rostro mas grande (mas cercano)
        target = max(results, key=lambda r: r["bbox"][2] * r["bbox"][3])
        x, y, bw, bh = target["bbox"]
        self.tracked_bbox = (x, y, bw, bh)

        # Centro instantaneo del rostro
        raw_cx = x + bw / 2.0
        raw_cy = y + bh / 2.0

        alpha = float(self.tracking_cfg.get("filter_alpha", 0.45))
        if self.filtered_face_cx is None:
            self.filtered_face_cx = raw_cx
            self.filtered_face_cy = raw_cy
        else:
            self.filtered_face_cx = (1.0 - alpha) * self.filtered_face_cx + alpha * raw_cx
            self.filtered_face_cy = (1.0 - alpha) * self.filtered_face_cy + alpha * raw_cy

        # Error respecto al centro del cuadro (640x480)
        err_x = self.filtered_face_cx - 320.0
        err_y = self.filtered_face_cy - 240.0

        db_x = float(self.tracking_cfg.get("deadband_x", 25.0))
        db_y = float(self.tracking_cfg.get("deadband_y", 25.0))
        kp_p = float(self.tracking_cfg.get("kp_pan", 0.025))
        kp_t = float(self.tracking_cfg.get("kp_tilt", 0.018))
        max_p_step = float(self.tracking_cfg.get("max_pan_step", 1.2))
        max_t_step = float(self.tracking_cfg.get("max_tilt_step", 0.8))
        inv_p = bool(self.tracking_cfg.get("invert_pan", True))
        inv_t = bool(self.tracking_cfg.get("invert_tilt", False))

        if abs(err_x) > db_x:
            step_pan = np.clip(err_x * kp_p, -max_p_step, max_p_step)
            self.target_pan_deg += (-step_pan if inv_p else step_pan)

        if abs(err_y) > db_y:
            step_tilt = np.clip(err_y * kp_t, -max_t_step, max_t_step)
            self.target_tilt_deg += (step_tilt if inv_t else -step_tilt)

        p_lim = float(self.tracking_cfg.get("max_pan_limit", 50.0))
        t_max = float(self.tracking_cfg.get("max_tilt_limit", 22.0))
        t_min = float(self.tracking_cfg.get("min_tilt_limit", -18.0))

        self.target_pan_deg = float(np.clip(self.target_pan_deg, -p_lim, p_lim))
        self.target_tilt_deg = float(np.clip(self.target_tilt_deg, t_min, t_max))

        # Publicar comando a 20 Hz (cada 50 ms)
        if now - self.last_track_time >= 0.05:
            self.last_track_time = now
            msg = Vector3()
            msg.x = self.target_pan_deg
            msg.y = self.target_tilt_deg
            msg.z = 0.0
            self.pub_pantilt.publish(msg)

    def start_voice_registration(self, reason: str = "manual"):
        """Inicia el protocolo interactivo RoboCup @Home para registrar a una nueva persona hablando."""
        if self.state != self.STATE_IDLE:
            return

        self.state = self.STATE_PROMPT_NAME
        self.state_start_time = time.time()
        self.voice_text = None
        self.pending_name = ""
        self.recording_samples = []
        self.best_thumbnail_crop = None
        self.best_sharpness = 0.0

        # Pregunta clara y amigable identificando al robot como "Robot"
        self.current_banner_text = "Hello! I am Robot. What is your name? You can say: My name is, and your name."
        print(f"\n[RoboCup @Home Memory] Iniciando registro guiado ({reason})...")
        print(f"[Robot]: \"{self.current_banner_text}\"")
        self.speaker.speak(self.current_banner_text)

    def start_biometric_recording(self, name: str):
        """Inicia la rutina de grabacion biometrica multi-muestra para el nombre recibido."""
        self.state = self.STATE_RECORDING_FACE
        self.state_start_time = time.time()
        self.pending_name = name
        self.recording_samples = []
        self.best_thumbnail_crop = None
        self.best_sharpness = 0.0
        self.last_sample_time = 0.0

        intro_text = f"Nice to meet you, {name}! I am Robot. Please look at my camera while I record your face."
        self.current_banner_text = intro_text
        print(f"[Robot]: \"{intro_text}\"")
        self.speaker.speak(intro_text)

    def draw_hud(self, frame: np.ndarray, results: list) -> np.ndarray:
        """Renderiza la interfaz profesional: Camara RealSense (640x480) + Panel de Memoria (240x480)."""
        h, w = frame.shape[:2]
        now = time.time()

        # Canvas total: 880x480 (640 para la camara en vivo, 240 para el panel de memoria)
        sidebar_w = 240
        canvas = np.zeros((h, w + sidebar_w, 3), dtype=np.uint8)

        # 1. Copiar fotograma de la camara
        vis = frame.copy()

        # Barra superior de la camara
        overlay = vis.copy()
        cv2.rectangle(overlay, (0, 0), (w, 55), (20, 24, 30), -1)
        cv2.addWeighted(overlay, 0.82, vis, 0.18, 0, vis)

        cv2.putText(vis, "ROBOT @HOME - HRI SYSTEM", (15, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        cv2.circle(vis, (w - 110, 20), 5, (0, 255, 0), -1)
        cv2.putText(vis, "EN VIVO", (w - 98, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 2)
        cv2.putText(vis, f"FPS: {self.fps:.1f}", (w - 195, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (200, 200, 200), 1)

        track_info = f"Head: Pan {self.target_pan_deg:+.1f} deg | Tilt {self.target_tilt_deg:+.1f} deg"
        cv2.putText(vis, track_info, (15, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (170, 200, 230), 1)

        # Nombres de rostros visibles en este fotograma
        visible_names = set()

        # Bounding boxes y etiquetas biometricas
        for res in results:
            x, y, bw, bh = res["bbox"]
            name = res["name"]
            sim = res["similarity"]
            q_valid = res.get("quality_valid", True)
            q_score = res.get("quality_score", 0.0)

            if name != "Desconocido":
                visible_names.add(name.lower())

            is_tracked = (self.tracked_bbox is not None and (x, y, bw, bh) == self.tracked_bbox and self.tracking_enabled)

            # Color segun estado
            if self.state == self.STATE_RECORDING_FACE and is_tracked:
                color = (0, 240, 255)  # Cyan para grabacion biometrica
                label = f"GRABANDO: {self.pending_name} [{len(self.recording_samples)}/{self.target_sample_count}]"
            elif name != "Desconocido":
                color = (0, 230, 70)  # Verde para conocido
                label = f"{name} ({int(sim * 100)}%)"
            else:
                color = (0, 165, 255)  # Naranja para desconocido
                label = f"Desconocido ({int(sim * 100)}%)"

            # Dibujar rectangulo con esquinas reforzadas
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

            # Efecto laser animado durante la rutina de grabacion biometrica
            if self.state == self.STATE_RECORDING_FACE and is_tracked:
                scan_y = int(y + (np.sin(now * 8.0) * 0.5 + 0.5) * bh)
                cv2.line(vis, (x, scan_y), (x + bw, scan_y), (0, 240, 255), 2)
                # Calidad visual
                q_text = f"Nitidez: {q_score:.0f}"
                cv2.putText(vis, q_text, (x, y + bh + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 240, 255), 1)

            # Etiqueta sobre el rostro
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 1)
            lbl_y = max(y - 8, th + 8)
            cv2.rectangle(vis, (x, lbl_y - th - 6), (x + tw + 10, lbl_y + 2), color, -1)
            cv2.putText(vis, label, (x + 5, lbl_y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (10, 15, 20), 2)

            # Reticulo de tracking
            if is_tracked and self.state != self.STATE_RECORDING_FACE:
                cx, cy = int(x + bw / 2), int(y + bh / 2)
                cv2.circle(vis, (cx, cy), 12, (0, 240, 255), 1)
                cv2.line(vis, (cx - 16, cy), (cx + 16, cy), (0, 240, 255), 1)
                cv2.line(vis, (cx, cy - 16), (cx, cy + 16), (0, 240, 255), 1)
                cv2.putText(vis, "[TRACKING]", (x, y + bh + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 240, 255), 1)

        # Banner interactivo de Estado y Dialogo
        banner_y = 65
        banner_h = 44

        # Comprobar override de saludo temporal
        if self.active_banner_override and now < self.active_banner_override[1]:
            b_color = (0, 180, 80)
            icon_txt = "[WELCOME]"
            msg_txt = self.active_banner_override[0]
            banner_bg = vis.copy()
            cv2.rectangle(banner_bg, (15, banner_y), (w - 15, banner_y + banner_h), b_color, -1)
            cv2.addWeighted(banner_bg, 0.85, vis, 0.15, 0, vis)
            cv2.rectangle(vis, (15, banner_y), (w - 15, banner_y + banner_h), (255, 255, 255), 1)
            cv2.putText(vis, icon_txt, (25, banner_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
            cv2.putText(vis, msg_txt, (140, banner_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

        elif self.state != self.STATE_IDLE:
            banner_bg = vis.copy()

            if self.state == self.STATE_PROMPT_NAME:
                b_color = (180, 120, 20)
                icon_txt = "[ROBOT TALKING]"
                msg_txt = self.current_banner_text
            elif self.state == self.STATE_WAIT_NAME:
                pulse = int(abs(np.sin(now * 4)) * 50)
                b_color = (0, 160 + pulse, 240)
                elapsed = now - self.state_start_time
                remaining = max(0, int(8.0 - elapsed))
                icon_txt = f"[LISTENING MIC - {remaining}s]"
                if self.voice_status == "LISTENING":
                    msg_txt = "Voice detected! Transcribing..."
                elif self.voice_status == "PROCESSING":
                    msg_txt = "Transcribing with Whisper AI..."
                else:
                    msg_txt = "Speak: 'My name is [Name]' / 'Me llamo [Nombre]'"
            elif self.state == self.STATE_RECORDING_FACE:
                b_color = (0, 140, 200)
                progress_pct = int((len(self.recording_samples) / max(1, self.target_sample_count)) * 100)
                icon_txt = f"[RECORDING {progress_pct}%]"
                msg_txt = f"Capturing biometric samples for {self.pending_name} ({len(self.recording_samples)}/{self.target_sample_count})"
            elif self.state == self.STATE_CONFIRM_NAME:
                b_color = (20, 160, 40)
                icon_txt = "[SUCCESS]"
                msg_txt = self.current_banner_text

            cv2.rectangle(banner_bg, (15, banner_y), (w - 15, banner_y + banner_h), b_color, -1)
            cv2.addWeighted(banner_bg, 0.85, vis, 0.15, 0, vis)
            cv2.rectangle(vis, (15, banner_y), (w - 15, banner_y + banner_h), (255, 255, 255), 1)
            cv2.putText(vis, icon_txt, (25, banner_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
            cv2.putText(vis, msg_txt, (185, banner_y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)

            # Tarjeta guiada de instruccion para el usuario
            if self.state in [self.STATE_PROMPT_NAME, self.STATE_WAIT_NAME]:
                card_w, card_h = 600, 56
                card_x = (w - card_w) // 2
                card_y = h - 105
                card_bg = vis.copy()
                cv2.rectangle(card_bg, (card_x, card_y), (card_x + card_w, card_y + card_h), (35, 28, 15), -1)
                cv2.addWeighted(card_bg, 0.85, vis, 0.15, 0, vis)
                cv2.rectangle(vis, (card_x, card_y), (card_x + card_w, card_y + card_h), (0, 200, 255), 2)

                cv2.putText(vis, "COMO RESPONDER (RØDE Directional Microphone):", (card_x + 12, card_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (0, 220, 255), 2)
                cv2.putText(vis, '>>> "My name is [Name]" o "Me llamo [Nombre]" (Ej: "My name is John") <<<', (card_x + 12, card_y + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)

            elif self.state == self.STATE_RECORDING_FACE:
                card_w, card_h = 600, 56
                card_x = (w - card_w) // 2
                card_y = h - 105
                card_bg = vis.copy()
                cv2.rectangle(card_bg, (card_x, card_y), (card_x + card_w, card_y + card_h), (20, 40, 50), -1)
                cv2.addWeighted(card_bg, 0.85, vis, 0.15, 0, vis)
                cv2.rectangle(vis, (card_x, card_y), (card_x + card_w, card_y + card_h), (0, 240, 255), 2)

                # Barra de progreso visual
                p_width = int((card_w - 24) * (len(self.recording_samples) / max(1, self.target_sample_count)))
                cv2.rectangle(vis, (card_x + 12, card_y + 26), (card_x + 12 + p_width, card_y + 46), (0, 220, 100), -1)
                cv2.rectangle(vis, (card_x + 12, card_y + 26), (card_x + card_w - 12, card_y + 46), (150, 150, 150), 1)

                cv2.putText(vis, f"GRABANDO MUESTRAS BIOMETRICAS: {len(self.recording_samples)} / {self.target_sample_count}...", (card_x + 12, card_y + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 240, 255), 2)

        # Barra inferior de atajos de la camara
        overlay_bot = vis.copy()
        cv2.rectangle(overlay_bot, (0, h - 36), (w, h), (20, 24, 30), -1)
        cv2.addWeighted(overlay_bot, 0.85, vis, 0.15, 0, vis)

        auto_str = "Auto: ON" if self.auto_prompt_enabled else "Auto: OFF"
        track_str = "Track: ON" if self.tracking_enabled else "Track: OFF"
        mic_str = f"Mic: {self.voice_status}"
        shortcuts = f"[V] Registro | {mic_str} | [T] Teclado | [H] {track_str} | [A] {auto_str} | [C] Borrar BD | [Q] Salir"
        cv2.putText(vis, shortcuts, (12, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1)

        # Pegar el area de la camara en el canvas
        canvas[0:h, 0:w] = vis

        # -------------------------------------------------------------
        # 2. PANEL LATERAL: MEMORIA FACIAL MULTI-PERSONA (RoboCup @Home)
        # -------------------------------------------------------------
        side_x = w
        cv2.rectangle(canvas, (side_x, 0), (side_x + sidebar_w, h), (24, 28, 35), -1)
        # Linea divisoria vertical
        cv2.line(canvas, (side_x, 0), (side_x, h), (50, 60, 75), 2)

        # Cabecera de la memoria
        cv2.rectangle(canvas, (side_x, 0), (side_x + sidebar_w, 55), (32, 38, 48), -1)
        cv2.putText(canvas, "MEMORIA FACIAL", (side_x + 14, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)

        profiles = self.recognizer.get_known_profiles()
        count_str = f"@Home: {len(profiles)} personas"
        cv2.putText(canvas, count_str, (side_x + 14, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 220, 255), 1)

        if not profiles:
            # Estado vacio
            cv2.rectangle(canvas, (side_x + 15, 120), (side_x + sidebar_w - 15, 240), (30, 35, 44), -1)
            cv2.rectangle(canvas, (side_x + 15, 120), (side_x + sidebar_w - 15, 240), (45, 55, 70), 1)
            cv2.putText(canvas, "Memoria Vacia", (side_x + 40, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (180, 190, 205), 1)
            cv2.putText(canvas, "Hable o presione [V]", (side_x + 28, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (130, 140, 155), 1)
            cv2.putText(canvas, "para registrar personas.", (side_x + 26, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (130, 140, 155), 1)
        else:
            # Ordenar perfiles por ultimo avistamiento descendente
            sorted_profiles = sorted(profiles, key=lambda p: p["last_seen"], reverse=True)
            # Mostrar hasta 4 perfiles principales
            display_count = min(4, len(sorted_profiles))

            for idx in range(display_count):
                prof = sorted_profiles[idx]
                card_y = 65 + idx * 95
                card_h = 86
                card_x = side_x + 10
                card_w = sidebar_w - 20

                p_name = prof["name"]
                is_active = p_name.lower() in visible_names

                # Colores segun presencia activa
                if is_active:
                    card_border = (0, 230, 70)
                    card_bg = (25, 45, 32)
                    status_lbl = "ACTIVO"
                    status_col = (0, 230, 70)
                else:
                    card_border = (50, 60, 75)
                    card_bg = (32, 38, 48)
                    elapsed_s = now - prof["last_seen"]
                    if elapsed_s < 60:
                        status_lbl = "Reciente"
                    elif elapsed_s < 3600:
                        status_lbl = f"{int(elapsed_s // 60)}m"
                    else:
                        status_lbl = f"{int(elapsed_s // 3600)}h"
                    status_col = (150, 165, 180)

                cv2.rectangle(canvas, (card_x, card_y), (card_x + card_w, card_y + card_h), card_bg, -1)
                cv2.rectangle(canvas, (card_x, card_y), (card_x + card_w, card_y + card_h), card_border, 1)

                # Miniatura del rostro
                thumb_box_x = card_x + 8
                thumb_box_y = card_y + 12
                thumb_size = 62

                if prof["thumbnail_img"] is not None and prof["thumbnail_img"].size > 0:
                    try:
                        th_resized = cv2.resize(prof["thumbnail_img"], (thumb_size, thumb_size))
                        canvas[thumb_box_y : thumb_box_y + thumb_size, thumb_box_x : thumb_box_x + thumb_size] = th_resized
                        cv2.rectangle(canvas, (thumb_box_x, thumb_box_y), (thumb_box_x + thumb_size, thumb_box_y + thumb_size), card_border, 1)
                    except Exception:
                        pass
                else:
                    # Icono / iniciales de respaldo
                    cv2.rectangle(canvas, (thumb_box_x, thumb_box_y), (thumb_box_x + thumb_size, thumb_box_y + thumb_size), (45, 55, 70), -1)
                    initials = p_name[:2].upper()
                    cv2.putText(canvas, initials, (thumb_box_x + 14, thumb_box_y + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (220, 220, 220), 2)

                # Informacion de la persona
                info_x = thumb_box_x + thumb_size + 10
                # Nombre
                display_name = (p_name[:12] + "..") if len(p_name) > 13 else p_name
                cv2.putText(canvas, display_name, (info_x, card_y + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (255, 255, 255), 2)

                # Badge de estado (Activo / Tiempo)
                cv2.putText(canvas, status_lbl, (card_x + card_w - 55, card_y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.38, status_col, 1)

                # Metricas: visitas y muestras
                cv2.putText(canvas, f"Visitas: {prof['sighting_count']}", (info_x, card_y + 48), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 195, 210), 1)
                cv2.putText(canvas, f"Muestras: {prof['samples_count']}", (info_x, card_y + 68), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 210, 240), 1)

            # Si hay mas perfiles que los visibles
            if len(sorted_profiles) > display_count:
                extra = len(sorted_profiles) - display_count
                cv2.putText(canvas, f"+ {extra} persona(s) mas en memoria", (side_x + 35, 452), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (140, 155, 175), 1)

        # Pie del panel de memoria
        cv2.line(canvas, (side_x, h - 28), (side_x + sidebar_w, h - 28), (40, 48, 60), 1)
        cv2.putText(canvas, "ROBOT HRI | SFace 128-D | @Home", (side_x + 14, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (130, 145, 160), 1)

        return canvas

    def enroll_by_typing(self, current_frame: np.ndarray, results: list):
        """Metodo de respaldo: Registro manual por teclado con fotos y biometricos."""
        if not results:
            print("\n[!] No hay ningun rostro detectado frente a la camara.")
            return

        print("\n--------------------------------------------------")
        print(" REGISTRO MANUAL POR TECLADO (MEMORIA ROBOCUP @HOME):")
        default_name = f"Guest_{len(self.recognizer.profiles) + 1}"
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
        self.start_biometric_recording(name)
        print(f"[+] Iniciando grabacion biometrica para '{name}'...")
        print("--------------------------------------------------\n")

    def run(self):
        window_name = "Robot @Home - Human-Robot Interaction (HRI)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 980, 535)

        print("==================================================")
        print("  Base@Home - Reconocimiento Facial y Memoria Multi-Persona")
        print("  - Inspirado en RoboCup @Home (Receptionist / Guest)")
        print("  - Memoria Biometrica: SFace 128-D + Perfiles JSON")
        print("  - Voz Neuronal: Piper TTS (en_US-lessac-medium)")
        print("  - Microfono: RØDE VideoMic GO II (+20 dB Calibrado)")
        print("  - Cabezal: Pan-Tilt Dynamixel (Trackeo Reactivo)")
        print("--------------------------------------------------")
        print("  [V] o [ESPACIO] : Iniciar registro oral por voz")
        print("  [H]             : Activar/desactivar trackeo del rostro")
        print("  [A]             : Activar/desactivar auto-saludo y registro")
        print("  [T]             : Registro manual por teclado")
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
                    "Esperando senal de camara (ZED Mini / Head Camera)...",
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

            # Recarga en caliente si tracking_params.yaml fue editado con el calibrador
            self.check_config_reload(now)

            # Inferencia y Deteccion Facial
            results = self.recognizer.detect_and_recognize(frame)

            # Actualizar trackeo del rostro con el cabezal Pan-Tilt
            self.update_face_tracking(results, now)

            # ----------------------------------------------------
            # MAQUINA DE ESTADOS Y RUTINA DE MEMORIA (RoboCup @Home)
            # ----------------------------------------------------
            if self.state == self.STATE_IDLE:
                # 1. Reconocimiento de personas conocidas y saludo amigable
                for res in results:
                    name = res["name"]
                    if name != "Desconocido":
                        # Registrar avistamiento / encuentro
                        is_new_sighting = self.recognizer.update_sighting(name, min_interval_sec=25.0)

                        # Saludar si no se ha saludado recientemente a esta persona
                        last_greet = self.greeting_cooldowns.get(name.lower(), 0.0)
                        if (is_new_sighting or (now - last_greet > 35.0)) and not self.speaker.is_speaking:
                            self.greeting_cooldowns[name.lower()] = now
                            # Obtener perfil para personalizar el saludo
                            prof = None
                            for p in self.recognizer.profiles.values():
                                if p.name.lower() == name.lower():
                                    prof = p
                                    break
                            seen_count = prof.sighting_count if prof else 1
                            if seen_count > 1:
                                greeting = f"Welcome back, {name}! I am Robot, good to see you again."
                            else:
                                greeting = f"Hello {name}! I am Robot, nice to see you."

                            print(f"[RoboCup @Home Memory]: \"{greeting}\"")
                            self.speaker.speak(greeting)
                            self.active_banner_override = (greeting, now + 3.5)
                            break

                # 2. Auto-deteccion de personas desconocidas para iniciar registro
                has_unknown = any(r["name"] == "Desconocido" for r in results)
                has_known = any(r["name"] != "Desconocido" for r in results)

                if has_unknown and not has_known and self.auto_prompt_enabled and (now - self.last_registered_time > 12.0):
                    if self.unknown_face_start_time is None:
                        self.unknown_face_start_time = now
                    elif now - self.unknown_face_start_time >= 1.8:
                        self.unknown_face_start_time = None
                        self.start_voice_registration(reason="auto-deteccion")
                else:
                    self.unknown_face_start_time = None

            elif self.state == self.STATE_PROMPT_NAME:
                # Transicion inmediata apenas el robot termine de hablar la pregunta
                if not self.speaker.is_speaking and (now - self.state_start_time > 0.2):
                    # Tono de aviso acustico de turn-taking (FBOT @Home pattern)
                    self.speaker.play_beep(blocking=True)
                    self.state = self.STATE_WAIT_NAME
                    self.state_start_time = time.time()
                    self.voice_text = None
                    self.trigger_mic_listening()
                    print("[Robot]: Escuchando respuesta del usuario en el microfono RØDE...")

            elif self.state == self.STATE_WAIT_NAME:
                # Esperar transcripcion de Whisper (hasta 8.0s)
                if self.voice_text:
                    user_speech = self.voice_text
                    self.voice_text = None
                    extracted = extract_person_name(user_speech)
                    print(f"[ASR Final]: \"{user_speech}\" -> Nombre extraido: '{extracted}'")

                    if extracted and len(extracted) >= 2:
                        self.pending_name = extracted
                        # Iniciar la rutina guiada de grabacion biometrica
                        self.start_biometric_recording(self.pending_name)
                    else:
                        print("[ASR]: No se pudo extraer un nombre valido. Reintentando...")
                        prompt_retry = "I am Robot, and I didn't catch your name. Please say: My name is, and your name."
                        self.current_banner_text = prompt_retry
                        self.speaker.speak(prompt_retry)
                        self.state = self.STATE_PROMPT_NAME
                        self.state_start_time = now

                elif now - self.state_start_time > 8.0:
                    print("[ASR]: Tiempo de espera finalizado (sin respuesta).")
                    self.speaker.speak("I am Robot. I did not hear you, let's try again later.")
                    self.state = self.STATE_IDLE
                    self.last_registered_time = now

            elif self.state == self.STATE_RECORDING_FACE:
                # RUTINA DE GRABACION BIOMETRICA MULTI-MUESTRA
                if results:
                    target = max(results, key=lambda r: r["bbox"][2] * r["bbox"][3])
                    feat = target.get("feature")
                    q_valid = target.get("quality_valid", False)
                    q_score = target.get("quality_score", 0.0)

                    # Capturar muestra a una tasa maxima de ~7 Hz (cada 140ms) para garantizar variabilidad
                    if feat is not None and q_valid and (now - self.last_sample_time >= 0.14):
                        self.last_sample_time = now
                        self.recording_samples.append(feat)

                        # Guardar el recorte mas nitido para la foto de perfil
                        if q_score > self.best_sharpness or self.best_thumbnail_crop is None:
                            x, y, bw, bh = target["bbox"]
                            mx, my = int(bw * 0.12), int(bh * 0.12)
                            y1, y2 = max(0, y - my), min(frame.shape[0], y + bh + my)
                            x1, x2 = max(0, x - mx), min(frame.shape[1], x + bw + mx)
                            self.best_thumbnail_crop = frame[y1:y2, x1:x2].copy()
                            self.best_sharpness = q_score

                # Verificar condicion de finalizacion
                elapsed = now - self.state_start_time
                if len(self.recording_samples) >= self.target_sample_count or (elapsed > 4.5 and len(self.recording_samples) >= 4):
                    # Persistir en memoria biometrica
                    pid, is_new = self.recognizer.enroll_person(
                        name=self.pending_name,
                        features=self.recording_samples,
                        thumbnail_img=self.best_thumbnail_crop,
                        attributes={"role": "guest", "enrolled_via": "voice_interaction"}
                    )
                    success_msg = f"Thank you, {self.pending_name}! I am Robot, and your face is now saved in my memory."
                    print(f"[RoboCup @Home Memory]: Guardado '{self.pending_name}' [ID: {pid}] con {len(self.recording_samples)} muestras.")
                    self.speaker.speak(success_msg)
                    self.current_banner_text = success_msg
                    self.state = self.STATE_CONFIRM_NAME
                    self.state_start_time = now

                elif elapsed > 6.5 and len(self.recording_samples) < 4:
                    print("[RoboCup @Home Memory]: Muestras insuficientes o rostro perdido.")
                    self.speaker.speak("I am Robot. I lost sight of your face, let's try again.")
                    self.state = self.STATE_IDLE
                    self.last_registered_time = now

            elif self.state == self.STATE_CONFIRM_NAME:
                # Esperar a que el robot termine de hablar el agradecimiento
                if not self.speaker.is_speaking and (now - self.state_start_time > 0.4):
                    self.state = self.STATE_IDLE
                    self.last_registered_time = now
                    self.pending_name = ""
                    self.recording_samples = []
                    self.best_thumbnail_crop = None

            # Calculo de FPS
            fps_counter += 1
            if now - last_fps_calc >= 0.5:
                self.fps = fps_counter / (now - last_fps_calc)
                fps_counter = 0
                last_fps_calc = now

            # Dibujar interfaz HUD profesional con panel lateral de memoria
            display_frame = self.draw_hud(frame, results)
            cv2.imshow(window_name, display_frame)

            # Publicar HUD grafico en topico ROS 2 (FBOT@Home /hri/ui_display)
            if ROS2_AVAILABLE and hasattr(self, "pub_ui_display") and self.pub_ui_display is not None:
                try:
                    ui_msg = self.bridge.cv2_to_imgmsg(display_frame, encoding="bgr8")
                    self.pub_ui_display.publish(ui_msg)
                except Exception:
                    pass

            # Publicar metadatos JSON de rostros detectados (FBOT@Home /hri/detected_persons)
            if ROS2_AVAILABLE and hasattr(self, "pub_detected_persons") and self.pub_detected_persons is not None:
                try:
                    persons_list = []
                    for r in results:
                        persons_list.append({
                            "name": r["name"],
                            "similarity": round(float(r["similarity"]), 3),
                            "bbox": [int(v) for v in r["bbox"]],
                            "quality_score": round(float(r.get("quality_score", 0.0)), 1),
                        })
                    msg_detected = String()
                    msg_detected.data = json.dumps(persons_list)
                    self.pub_detected_persons.publish(msg_detected)
                except Exception:
                    pass

            # Manejo de teclado
            key = cv2.waitKey(1) & 0xFF
            if key in [ord("q"), ord("Q"), 27]:
                print("[*] Saliendo del sistema...")
                break
            elif key in [ord("v"), ord("V"), 32]:  # V o ESPACIO
                self.start_voice_registration(reason="tecla manual")
            elif key in [ord("h"), ord("H")]:
                self.tracking_enabled = not self.tracking_enabled
                state_str = "ACTIVADO" if self.tracking_enabled else "DESACTIVADO"
                print(f"[*] Trackeo Facial Pan-Tilt: {state_str}")
            elif key in [ord("t"), ord("T")]:
                self.enroll_by_typing(frame, results)
            elif key in [ord("a"), ord("A")]:
                self.auto_prompt_enabled = not self.auto_prompt_enabled
                state_str = "ACTIVADO" if self.auto_prompt_enabled else "DESACTIVADO"
                print(f"[*] Auto-Registro por voz: {state_str}")
            elif key in [ord("c"), ord("C")]:
                self.recognizer.clear_database()
                print("[*] Memoria facial borrada por el usuario.")

        cv2.destroyAllWindows()
        self.speaker.shutdown()
        if self.cap:
            self.cap.release()


def main():
    parser = argparse.ArgumentParser(description="Reconocimiento Facial, Memoria Biometrica y Trackeo para RoboCup @Home")
    parser.add_argument("--topic", default="/camera/camera/color/image_raw", help="Topico ROS 2 de imagen RGB")
    parser.add_argument("--webcam", type=int, default=-1, help="Usar camara USB local /dev/videoX en lugar de ROS 2")
    args = parser.parse_args()

    pkg_dir = "/home/athome/base_athome_ws/src/base_athome_perception"
    caffe_proto = os.path.join(pkg_dir, "models", "deploy.prototxt")
    caffe_model = os.path.join(pkg_dir, "models", "res10_300x300_ssd_iter_140000.caffemodel")
    sface = os.path.join(pkg_dir, "models", "face_recognition_sface_2021dec.onnx")
    db_path = os.path.join(pkg_dir, "config", "faces_database")

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
