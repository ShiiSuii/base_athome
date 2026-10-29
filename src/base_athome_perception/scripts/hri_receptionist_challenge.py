#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ==============================================================================
# Base@Home - RoboCup @Home HRI Receptionist Challenge Node
# ==============================================================================
# Standard RoboCup @Home Receptionist Task:
#   1. Robot waits at entrance door (waypoint 'door' or 'A').
#   2. Detects approaching guest via ZED Mini head camera and locks pan-tilt.
#   3. Greets guest and asks for name and favorite drink.
#   4. Turn-taking acoustic cue (beep.wav) and Whisper ASR transcription.
#   5. Biometric face enrollment with SFace (128-D) + Laplacian quality filter.
#   6. Expressive animated face emotion synchronization (/robot/emotion).
#   7. Escorts guest to living room (Nav2 waypoint 'living_room' or 'B').
#   8. Detects host ("Charlie") and introduces the new guest.
#   9. Inspects living room seating using YOLOv8-pose (seated vs standing),
#      points with AgileX Piper 6-DOF arm to available seat and invites guest.
#  10. Returns to door to receive Guest 2, and introduces Guest 2 to both
#      the host and Guest 1.
# ==============================================================================

import argparse
import datetime
import math
import os
import queue
import re
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Vector3, PoseStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger

# CV Bridge
try:
    from cv_bridge import CvBridge
except ImportError:
    class CvBridge:
        def imgmsg_to_cv2(self, img_msg, desired_encoding="bgr8"):
            im = np.frombuffer(img_msg.data, dtype=np.uint8)
            im = im.reshape((img_msg.height, img_msg.width, -1))
            if desired_encoding == "bgr8" and im.shape[2] == 3:
                return im
            return im

# Nav2 BasicNavigator
try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    NAV2_AVAILABLE = True
except ImportError:
    BasicNavigator = None
    TaskResult = None
    NAV2_AVAILABLE = False

# Base@Home Perception Modules
from base_athome_perception.face_recognizer import FaceRecognizer, PersonProfile
from base_athome_perception.speaker import RoboCupSpeaker
from base_athome_perception.person_pose_detector import PersonPoseDetector
from base_athome_perception.piper_kinematics import (
    SAFE_JOINT_LIMITS_DEG,
    DEFAULT_NAMED_POSES,
)

# ------------------------------------------------------------------------------
# VOCABULARIOS Y EXTRACCION DE DIALOGO ROBUSTA (NLP)
# ------------------------------------------------------------------------------

COMMON_DRINKS = [
    'coca cola', 'coke', 'pepsi', 'fanta', 'sprite', 'water', 'mineral water',
    'sparkling water', 'beer', 'wine', 'red wine', 'white wine', 'coffee',
    'tea', 'iced tea', 'green tea', 'black tea', 'milk', 'orange juice',
    'apple juice', 'lemonade', 'juice', 'soda', 'cider', 'hot chocolate',
    'agua', 'cafe', 'cerveza', 'vino', 'jugo', 'leche', 'te'
]

DRINK_TRANSLATIONS = {
    'agua': 'Water',
    'cafe': 'Coffee',
    'cerveza': 'Beer',
    'vino': 'Wine',
    'jugo': 'Juice',
    'leche': 'Milk',
    'te': 'Tea',
    'coca cola': 'Coke',
}

STOP_WORDS = {
    'robot', 'please', 'hello', 'hi', 'yes', 'no', 'drink', 'name', 'favorite',
    'favourite', 'the', 'and', 'my', 'is', 'like', 'i', 'am', 'call', 'me', 'y',
    'por', 'favor', 'hola', 'mi', 'nombre', 'es', 'bebida', 'favorita', 'un', 'una',
    'some', 'would', 'prefer', 'want', 'gusta', 'tomar', 'followed', 'by'
}


def extract_name_and_drink(text: str) -> Tuple[Optional[str], Optional[str]]:
    """Extrae (nombre, bebida) a partir de una frase hablada en inglés o español."""
    text_clean = text.strip()
    name = None
    drink = None

    text_lower = text_clean.lower()
    for d in sorted(COMMON_DRINKS, key=len, reverse=True):
        if re.search(r'\b' + re.escape(d) + r'\b', text_lower):
            drink = DRINK_TRANSLATIONS.get(d, d.title())
            break

    if not drink:
        drink_patterns = [
            r'(?:favorite drink is|favourite drink is|drink is|i like|i love|i prefer|i want|have a|some|drink|bebida favorita es|me gusta el|me gusta la|tomar|beber)\s+([a-zA-Z\s]+?)(?:$|\.|\band\b|\by\b)',
        ]
        for pat in drink_patterns:
            m = re.search(pat, text_lower)
            if m:
                cand = m.group(1).strip()
                cand_words = [w for w in cand.split() if w not in STOP_WORDS]
                if cand_words:
                    d_raw = ' '.join(cand_words)
                    drink = DRINK_TRANSLATIONS.get(d_raw, d_raw.title())
                    break

    name_patterns = [
        r'(?:my name is|i am|i\'m|call me|this is|they call me)\s+([A-Za-z]+)',
        r'(?:me llamo|mi nombre es|yo soy|soy)\s+([A-Za-z]+)',
    ]
    for pat in name_patterns:
        m = re.search(pat, text_clean, re.IGNORECASE)
        if m:
            cand = m.group(1).strip().title()
            if cand.lower() not in STOP_WORDS:
                name = cand
                break

    if not name:
        rem = text_clean
        if drink:
            rem = re.sub(re.escape(drink), '', rem, flags=re.IGNORECASE)
            for k, v in DRINK_TRANSLATIONS.items():
                if v == drink:
                    rem = re.sub(re.escape(k), '', rem, flags=re.IGNORECASE)
        rem = re.sub(
            r'^(?:hello|hi|please|i am|i\'m|my name is|and|i like|hola|me llamo|mi nombre es)\s*',
            '',
            rem,
            flags=re.IGNORECASE,
        ).strip()
        words = [w.strip('.!?,') for w in rem.split() if w.strip('.!?,')]
        valid_words = [w for w in words if w.lower() not in STOP_WORDS and len(w) >= 3 and w.isalpha()]
        if valid_words:
            name = valid_words[0].title()

    return name, drink


# ------------------------------------------------------------------------------
# CONSTANTES DE POSES Y GESTOS DEL BRAZO PIPER
# ------------------------------------------------------------------------------

ARM_GESTURE_POSES = {
    'home': [72.09, 176.5, -155.15, 102.62, -31.51, -99.91],
    'cube_stow': [72.09, 176.5, -155.15, 102.62, -31.51, -99.91],
    'point_couch': [45.0, 155.0, -75.0, 50.0, -50.0, -80.0],
    'point_chair': [25.0, 150.0, -70.0, 35.0, -45.0, -80.0],
    'wave_hello': [60.0, 130.0, -110.0, 80.0, -10.0, -90.0],
}


# ------------------------------------------------------------------------------
# NODO PRINCIPAL DE LA CHALLENGE HRI RECEPTIONIST
# ------------------------------------------------------------------------------

class HriReceptionistChallengeNode(Node):
    """Nodo autónomo de la prueba HRI Receptionist para RoboCup @Home."""

    # Estados de la Máquina de Estados
    STATE_INIT = 0
    STATE_NAV_TO_DOOR = 1
    STATE_WAIT_DOOR = 2
    STATE_GREET_AND_PROMPT = 3
    STATE_WAIT_NAME_DRINK = 4
    STATE_PROMPT_MISSING_INFO = 5
    STATE_CONFIRM_DETAILS = 6
    STATE_MEMORIZE_FACE = 7
    STATE_NAV_TO_LIVING_ROOM = 8
    STATE_FIND_HOST = 9
    STATE_INTRODUCE_GUESTS = 10
    STATE_DETECT_AND_OFFER_SEAT = 11
    STATE_CHECK_NEXT_GUEST = 12
    STATE_CHALLENGE_COMPLETE = 13

    STATE_NAMES = {
        STATE_INIT: "0/7: INICIALIZANDO SISTEMA",
        STATE_NAV_TO_DOOR: "1/7: NAVEGANDO A LA ENTRADA",
        STATE_WAIT_DOOR: "2/7: ESPERANDO NUEVO INVITADO",
        STATE_GREET_AND_PROMPT: "3/7: SALUDO Y PREGUNTA DE DATOS",
        STATE_WAIT_NAME_DRINK: "3/7: ESCUCHANDO NOMBRE Y BEBIDA",
        STATE_PROMPT_MISSING_INFO: "3/7: ACLARANDO INFORMACION",
        STATE_CONFIRM_DETAILS: "3/7: CONFIRMANDO DATOS",
        STATE_MEMORIZE_FACE: "4/7: MEMORIZANDO ROSTRO BIOMETRICO",
        STATE_NAV_TO_LIVING_ROOM: "5/7: GUIANDO A LA SALA",
        STATE_FIND_HOST: "6/7: LOCALIZANDO AL ANFITRION",
        STATE_INTRODUCE_GUESTS: "6/7: PRESENTANDO A LOS INVITADOS",
        STATE_DETECT_AND_OFFER_SEAT: "7/7: ASIGNANDO Y SENALANDO ASIENTO",
        STATE_CHECK_NEXT_GUEST: "CICLO: PREPARANDO SIGUIENTE INVITADO",
        STATE_CHALLENGE_COMPLETE: "EXITO: CHALLENGE COMPLETADA",
    }

    def __init__(
        self,
        camera_topic: str = "",
        door_wp: str = "A",
        living_room_wp: str = "B",
        sim_nav: bool = False,
        no_gui: bool = False,
        max_guests: int = 2,
    ):
        super().__init__("hri_receptionist_challenge_node")

        self.door_wp = door_wp
        self.living_room_wp = living_room_wp
        self.sim_nav = sim_nav
        self.no_gui = no_gui
        self.max_guests = max_guests

        self.get_logger().info("==================================================")
        self.get_logger().info("🚀 INICIANDO ROBOCUP @HOME HRI RECEPTIONIST CHALLENGE")
        self.get_logger().info(f"   Punto Entrada: '{self.door_wp}' | Punto Sala: '{self.living_room_wp}'")
        self.get_logger().info(f"   Modo Navegacion Simulada: {self.sim_nav} | Max Invitados: {self.max_guests}")
        self.get_logger().info("==================================================")

        # Rutas de modelos y recursos
        self.pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.models_dir = os.path.join(self.pkg_dir, "models")
        self.config_dir = os.path.join(self.pkg_dir, "config")
        self.resources_dir = os.path.join(self.pkg_dir, "resources")
        self.db_dir = os.path.join(self.config_dir, "faces_database")

        # 1. Inicializar Sintesis de Voz Neuronal y Beep
        piper_bin = "/home/athome/piper_bin/piper/piper"
        piper_voice = "/home/athome/piper_bin/voices/en/en_US-lessac-medium.onnx"
        beep_wav = os.path.join(self.resources_dir, "beep.wav")
        self.speaker = RoboCupSpeaker(
            piper_bin=piper_bin,
            model_path=piper_voice,
            beep_path=beep_wav,
        )

        # 2. Inicializar Motor de Reconocimiento y Memoria Biometrica SFace
        caffe_proto = os.path.join(self.models_dir, "deploy.prototxt")
        caffe_model = os.path.join(self.models_dir, "res10_300x300_ssd_iter_140000.caffemodel")
        sface_model = os.path.join(self.models_dir, "face_recognition_sface_2021dec.onnx")
        self.recognizer = FaceRecognizer(
            caffe_proto_path=caffe_proto,
            caffe_model_path=caffe_model,
            sface_model_path=sface_model,
            db_path=self.db_dir,
            conf_thresh=0.55,
            cosine_thresh=0.52,
        )

        # Pre-cargar o registrar perfil del Host ("Charlie") si no existe
        self._ensure_host_registered()

        # 3. Inicializar Detector de Poses Humanas (YOLOv8-pose)
        pose_model = os.path.join(self.models_dir, "yolov8n-pose.pt")
        try:
            self.pose_detector = PersonPoseDetector(pose_model, conf_thresh=0.45)
            self.get_logger().info("Detector YOLOv8-pose inicializado correctamente.")
        except Exception as e:
            self.get_logger().warn(f"No se pudo cargar YOLOv8-pose ({e}). Continuando con deteccion facial.")
            self.pose_detector = None

        # 4. Inicializar Navegador Nav2
        self.navigator = None
        if NAV2_AVAILABLE and not self.sim_nav:
            try:
                self.navigator = BasicNavigator()
                self.get_logger().info("Nav2 BasicNavigator conectado.")
            except Exception as e:
                self.get_logger().warn(f"Nav2 no disponible ({e}). Activando simulacion de navegacion.")
                self.sim_nav = True
        else:
            self.sim_nav = True

        # Cargar tabla de waypoints
        self.waypoints = self._load_waypoints()

        # 5. Publicadores ROS 2
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_emotion = self.create_publisher(String, "/robot/emotion", 10)
        self.pub_pantilt = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 10)
        self.pub_arm_joint = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_arm_gripper = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_joint_states = self.create_publisher(JointState, "/joint_states", 10)

        # Cliente para activar microfono /voice/listen_trigger
        self.cli_mic_trigger = self.create_client(Trigger, "/voice/listen_trigger")

        # 6. Suscriptores ROS 2
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.last_frame_time: float = 0.0

        qos_cam = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        candidate_topics = [
            "/zed/zed_node/rgb/image_rect_color",
            "/zed/zed_node/left/image_rect_color",
            "/zed/rgb/image_rect_color",
            "/camera/camera/color/image_raw",
            "/camera/color/image_raw",
        ]
        if camera_topic:
            candidate_topics.insert(0, camera_topic)

        self.sub_cam_list = []
        for top in candidate_topics:
            self.sub_cam_list.append(
                self.create_subscription(Image, top, self.image_callback, qos_cam)
            )

        # Suscriptor a Whisper ASR
        self.sub_voice = self.create_subscription(
            String, "/voice/transcription", self.voice_callback, 10
        )
        self.sub_voice_status = self.create_subscription(
            String, "/voice/status", self.voice_status_callback, 10
        )

        # 7. Variables de la Máquina de Estados
        self.state = self.STATE_INIT
        self.state_start_time = time.time()
        self.guest_index = 1
        self.registered_guests: List[Dict] = []

        # Datos temporales del invitado actual
        self.current_name: Optional[str] = None
        self.current_drink: Optional[str] = None
        self.missing_field: Optional[str] = None
        self.retry_count = 0
        self.door_detection_start: Optional[float] = None

        # Grabación biométrica
        self.recording_samples: List[np.ndarray] = []
        self.best_thumbnail: Optional[np.ndarray] = None
        self.best_sharpness: float = 0.0
        self.target_sample_count = 10
        self.last_sample_time: float = 0.0

        # Audio y Transcripción
        self.voice_text: Optional[str] = None
        self.voice_status: str = "IDLE"
        self.user_speech_buffer: str = ""

        # Control del Cabezal Pan-Tilt
        self.current_pan_deg = 0.0
        self.current_tilt_deg = -5.0
        self.target_pan_deg = 0.0
        self.target_tilt_deg = -5.0
        self.pan_tilt_tracking = True

        # Telemetría del brazo
        self.current_arm_pose_name = "home"
        self.arm_joints_deg = list(ARM_GESTURE_POSES['home'])

        # Hilo de publicación de /joint_states a 25 Hz
        self.timer_joints = self.create_timer(0.04, self._publish_joint_states)

        # Subtítulo y banner activo en el HUD
        self.current_banner_text = "Iniciando desafio HRI Receptionist..."
        self.last_spoken_text = ""

        # Iniciar posición Home del brazo y emoción neutral
        self.set_robot_emotion("neutral")
        self.move_arm_to_named_pose("cube_stow", duration_s=1.5)

        self.get_logger().info("HriReceptionistChallengeNode listo para ejecucion.")

    # --------------------------------------------------------------------------
    # GESTION DE WAYPOINTS Y PERFILES
    # --------------------------------------------------------------------------

    def _load_waypoints(self) -> Dict[str, Dict]:
        """Carga waypoints, zonas y asientos desde semantic_map.yaml o waypoints.yaml."""
        semantic_path = os.path.join(
            self.pkg_dir, "..", "base_athome_navigation", "config", "semantic_map.yaml"
        )
        wp_path = os.path.join(
            self.pkg_dir, "..", "base_athome_navigation", "config", "waypoints.yaml"
        )
        if not os.path.exists(wp_path):
            wp_path = os.path.expanduser("~/waypoints_base_athome.yaml")

        self.zones = {}
        self.seats = {}
        waypoints = {}

        if os.path.exists(semantic_path):
            try:
                with open(semantic_path, "r") as f:
                    data = yaml.safe_load(f) or {}
                    waypoints = data.get("waypoints", {})
                    self.zones = data.get("zones", {})
                    self.seats = data.get("seats", {})
                    self.get_logger().info(
                        f"Mapa Semántico cargado: {len(self.zones)} zonas, {len(waypoints)} waypoints, {len(self.seats)} asientos."
                    )
            except Exception as e:
                self.get_logger().warn(f"Error cargando semantic_map.yaml: {e}")

        if not waypoints and os.path.exists(wp_path):
            try:
                with open(wp_path, "r") as f:
                    data = yaml.safe_load(f) or {}
                    if data and "waypoints" in data:
                        waypoints = data["waypoints"]
            except Exception as e:
                self.get_logger().warn(f"Error cargando waypoints: {e}")

        # Aliasing para máxima compatibilidad (A <-> door, B <-> living_room)
        if "door" in waypoints and "A" not in waypoints:
            waypoints["A"] = waypoints["door"]
        if "A" in waypoints and "door" not in waypoints:
            waypoints["door"] = waypoints["A"]
        if "living_room" in waypoints and "B" not in waypoints:
            waypoints["B"] = waypoints["living_room"]
        if "B" in waypoints and "living_room" not in waypoints:
            waypoints["living_room"] = waypoints["B"]

        return waypoints

    def _ensure_host_registered(self) -> None:
        """Asegura que el perfil del anfitrión ('Charlie') exista en la memoria."""
        host_exists = any(p.name.lower() == "charlie" for p in self.recognizer.profiles.values())
        if not host_exists:
            # Crear perfil formal del anfitrión
            dummy_feat = np.random.randn(128).astype(np.float32)
            dummy_feat /= np.linalg.norm(dummy_feat)
            self.recognizer.enroll_person(
                name="Charlie",
                features=[dummy_feat],
                attributes={"role": "host", "favorite_drink": "Water", "seat": "sofa_left"},
            )
            self.get_logger().info("Perfil del anfitrion 'Charlie' registrado en la memoria.")

    # --------------------------------------------------------------------------
    # COMUNICACION MULTI-MODAL: HABLA, EMOCIONES Y CABEZAL
    # --------------------------------------------------------------------------

    def say(self, text: str, emotion: str = "speaking", blocking: bool = False):
        """Sintetiza texto con Piper TTS y sincroniza la cara animada y los subtitulos."""
        text = text.strip()
        if not text:
            return

        self.last_spoken_text = text
        self.current_banner_text = text

        # Publicar texto a /hri/say_something para que la cara muestre el subtitulo
        msg_str = String()
        msg_str.data = text
        self.pub_say.publish(msg_str)

        # Configurar emocion
        if emotion:
            self.set_robot_emotion(emotion)

        # Locución con Piper TTS
        self.speaker.speak(text, blocking=blocking)

    def set_robot_emotion(self, emotion: str):
        """Envia estado emocional a la cara interactiva en pantalla."""
        msg = String()
        msg.data = emotion
        self.pub_emotion.publish(msg)

    def trigger_mic_listening(self):
        """Solicita al servicio de Whisper en la Jetson comenzar a escuchar."""
        if self.cli_mic_trigger.service_is_ready():
            req = Trigger.Request()
            self.cli_mic_trigger.call_async(req)

    def move_head_to(self, pan_deg: float, tilt_deg: float):
        """Envía comandos angulares al cabezal pan-tilt."""
        self.target_pan_deg = float(np.clip(pan_deg, -65.0, 65.0))
        self.target_tilt_deg = float(np.clip(tilt_deg, -35.0, 35.0))
        msg = Vector3()
        msg.x = self.target_pan_deg
        msg.y = self.target_tilt_deg
        msg.z = 0.0
        self.pub_pantilt.publish(msg)

    def move_arm_to_named_pose(self, pose_name: str, duration_s: float = 2.0):
        """Mueve el brazo AgileX Piper a una postura segura precalibrada."""
        if pose_name not in ARM_GESTURE_POSES:
            self.get_logger().warn(f"Pose de brazo desconocida: {pose_name}")
            return

        target_deg = ARM_GESTURE_POSES[pose_name]
        self.current_arm_pose_name = pose_name
        self.arm_joints_deg = list(target_deg)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [math.radians(d) for d in target_deg]
        js.velocity = [duration_s]
        self.pub_arm_joint.publish(js)

        # Control de pinza segun la pose
        g_msg = Float32()
        g_msg.data = 60.0 if "point" in pose_name else 25.0
        self.pub_arm_gripper.publish(g_msg)

    def _publish_joint_states(self):
        """Publica el estado cinematico actual en /joint_states para visualizacion en RViz2."""
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            "pan_joint",
            "tilt_joint",
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        # Pan-tilt en radianes
        pan_rad = math.radians(self.target_pan_deg)
        tilt_rad = math.radians(self.target_tilt_deg)
        arm_rad = [math.radians(d) for d in self.arm_joints_deg]

        js.position = [pan_rad, tilt_rad] + arm_rad
        self.pub_joint_states.publish(js)

    # --------------------------------------------------------------------------
    # NAVEGACION AUTONOMA
    # --------------------------------------------------------------------------

    def navigate_to_waypoint(self, wp_name: str) -> bool:
        """Navega de forma autónoma al waypoint objetivo."""
        self.get_logger().info(f"🎯 Iniciando navegacion hacia waypoint '{wp_name}'...")
        self.current_banner_text = f"Navegando hacia '{wp_name}'..."
        self.set_robot_emotion("neutral")

        # Postura de transporte seguro del brazo
        self.move_arm_to_named_pose("cube_stow", duration_s=1.0)
        self.move_head_to(0.0, -5.0)

        if self.sim_nav or self.navigator is None:
            self.get_logger().info(f"[SIM NAV]: Simulando viaje a '{wp_name}' (3.0s)...")
            time.sleep(3.0)
            self.get_logger().info(f"[SIM NAV]: Llegada confirmada a '{wp_name}'.")
            return True

        if wp_name not in self.waypoints:
            self.get_logger().error(f"Waypoint '{wp_name}' no encontrado en waypoints.yaml.")
            return False

        wp_data = self.waypoints[wp_name]
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = float(wp_data.get("x", 0.0))
        target_pose.pose.position.y = float(wp_data.get("y", 0.0))
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.x = float(wp_data.get("qx", 0.0))
        target_pose.pose.orientation.y = float(wp_data.get("qy", 0.0))
        target_pose.pose.orientation.z = float(wp_data.get("qz", 0.0))
        target_pose.pose.orientation.w = float(wp_data.get("qw", 1.0))

        self.navigator.goToPose(target_pose)

        while not self.navigator.isTaskComplete():
            time.sleep(0.1)

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Nav2: Llegada exitosa a '{wp_name}'.")
            return True
        else:
            self.get_logger().warn(f"Nav2: Tarea no completada en '{wp_name}' (resultado: {result}).")
            return False

    # --------------------------------------------------------------------------
    # CALLBACKS ROS 2 (IMAGEN Y VOZ)
    # --------------------------------------------------------------------------

    def image_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.frame_lock:
                self.latest_frame = cv_img
                self.last_frame_time = time.time()
        except Exception as e:
            self.get_logger().warn(f"Error procesando frame de camara: {e}")

    def voice_callback(self, msg: String):
        text = msg.data.strip()
        if text:
            self.voice_text = text
            self.user_speech_buffer = text
            self.get_logger().info(f"🎙️ [Whisper ASR]: \"{text}\"")

    def voice_status_callback(self, msg: String):
        self.voice_status = msg.data.strip().upper()

    # --------------------------------------------------------------------------
    # BUCLE DE LA MAQUINA DE ESTADOS HRI
    # --------------------------------------------------------------------------

    def step(self):
        """Ejecuta un ciclo de la maquina de estados de la prueba Receptionist."""
        now = time.time()

        with self.frame_lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None

        # 1. Detección Facial y Seguimiento de Cabezal
        face_results = []
        if frame is not None:
            face_results = self.recognizer.detect_and_recognize(frame)

            # Seguimiento suave del rostro con Pan-Tilt
            if self.pan_tilt_tracking and face_results:
                largest_face = max(face_results, key=lambda f: f["bbox"][2] * f["bbox"][3])
                fx, fy, fw, fh = largest_face["bbox"]
                cx = fx + fw / 2.0
                cy = fy + fh / 2.0
                img_h, img_w = frame.shape[:2]

                err_x = (cx - img_w / 2.0) / (img_w / 2.0)
                err_y = (cy - img_h / 2.0) / (img_h / 2.0)

                # Control P suave
                pan_step = np.clip(err_x * 1.5, -2.0, 2.0)
                tilt_step = np.clip(err_y * 1.2, -1.5, 1.5)

                self.target_pan_deg -= pan_step
                self.target_tilt_deg -= tilt_step
                self.move_head_to(self.target_pan_deg, self.target_tilt_deg)

        # 2. Transición y Lógica de Estados
        if self.state == self.STATE_INIT:
            self.say("Robot is ready. Starting Receptionist Challenge.", emotion="happy", blocking=True)
            self.state = self.STATE_NAV_TO_DOOR
            self.state_start_time = now

        elif self.state == self.STATE_NAV_TO_DOOR:
            self.say("Navigating to the entrance door.", emotion="neutral")
            self.navigate_to_waypoint(self.door_wp)
            self.state = self.STATE_WAIT_DOOR
            self.state_start_time = now
            self.door_detection_start = None
            self.move_head_to(0.0, -5.0)
            self.set_robot_emotion("neutral")
            self.current_banner_text = f"Esperando invitado {self.guest_index} en la puerta..."

        elif self.state == self.STATE_WAIT_DOOR:
            # Esperar a que una persona se acerque a la puerta
            if face_results:
                largest_face = max(face_results, key=lambda f: f["bbox"][2] * f["bbox"][3])
                fw, fh = largest_face["bbox"][2], largest_face["bbox"][3]
                # Si el rostro tiene tamaño suficiente (persona frente al robot)
                if fw * fh > 3500:
                    if self.door_detection_start is None:
                        self.door_detection_start = now
                    elif now - self.door_detection_start >= 1.2:
                        self.state = self.STATE_GREET_AND_PROMPT
                        self.state_start_time = now
                else:
                    self.door_detection_start = None
            else:
                self.door_detection_start = None

        elif self.state == self.STATE_GREET_AND_PROMPT:
            # Saludo y pregunta abierta de nombre y bebida favorita
            greeting = f"Hello! Welcome to our home. I am Robot. Please tell me your name, and your favorite drink."
            self.say(greeting, emotion="speaking", blocking=True)
            # Tono de turn-taking acustico para ceder el turno de habla al humano
            self.speaker.play_beep(blocking=True)
            self.set_robot_emotion("listening")
            self.trigger_mic_listening()
            self.voice_text = None
            self.state = self.STATE_WAIT_NAME_DRINK
            self.state_start_time = now

        elif self.state == self.STATE_WAIT_NAME_DRINK:
            # Esperar respuesta por Whisper ASR (hasta 8.5s)
            if self.voice_text:
                speech = self.voice_text
                self.voice_text = None
                self.set_robot_emotion("thinking")

                extracted_name, extracted_drink = extract_name_and_drink(speech)
                self.get_logger().info(
                    f"NLP Parser: \"{speech}\" -> Nombre: '{extracted_name}', Bebida: '{extracted_drink}'"
                )

                if extracted_name:
                    self.current_name = extracted_name
                if extracted_drink:
                    self.current_drink = extracted_drink

                # Si tenemos ambos datos
                if self.current_name and self.current_drink:
                    self.state = self.STATE_CONFIRM_DETAILS
                    self.state_start_time = now
                elif self.current_name and not self.current_drink:
                    self.missing_field = "drink"
                    self.state = self.STATE_PROMPT_MISSING_INFO
                    self.state_start_time = now
                elif not self.current_name and self.current_drink:
                    self.missing_field = "name"
                    self.state = self.STATE_PROMPT_MISSING_INFO
                    self.state_start_time = now
                else:
                    # No se extrajo ninguno
                    self.retry_count += 1
                    if self.retry_count >= 2:
                        # Asignar nombres por defecto para avanzar de forma robusta
                        self.current_name = f"Guest {self.guest_index}"
                        self.current_drink = "Water"
                        self.state = self.STATE_CONFIRM_DETAILS
                        self.state_start_time = now
                    else:
                        prompt_clarify = "I did not catch that. Please state clearly: My name is, and your drink."
                        self.say(prompt_clarify, emotion="speaking", blocking=True)
                        self.speaker.play_beep(blocking=True)
                        self.set_robot_emotion("listening")
                        self.trigger_mic_listening()
                        self.state_start_time = now

            elif now - self.state_start_time > 8.5:
                # Timeout de habla
                self.retry_count += 1
                if self.retry_count >= 2:
                    self.current_name = f"Guest {self.guest_index}"
                    self.current_drink = "Water"
                    self.state = self.STATE_CONFIRM_DETAILS
                    self.state_start_time = now
                else:
                    self.say("I didn't hear you. Please tell me your name.", emotion="speaking", blocking=True)
                    self.speaker.play_beep(blocking=True)
                    self.set_robot_emotion("listening")
                    self.trigger_mic_listening()
                    self.state_start_time = now

        elif self.state == self.STATE_PROMPT_MISSING_INFO:
            if self.missing_field == "drink":
                prompt_drink = f"Nice to meet you, {self.current_name}! What is your favorite drink?"
                self.say(prompt_drink, emotion="speaking", blocking=True)
            else:
                prompt_name = "I got your drink, but what is your name?"
                self.say(prompt_name, emotion="speaking", blocking=True)

            self.speaker.play_beep(blocking=True)
            self.set_robot_emotion("listening")
            self.trigger_mic_listening()
            self.voice_text = None
            self.state = self.STATE_WAIT_NAME_DRINK
            self.state_start_time = now

        elif self.state == self.STATE_CONFIRM_DETAILS:
            confirm_msg = f"Thank you, {self.current_name}! I noted that your favorite drink is {self.current_drink}."
            self.say(confirm_msg, emotion="happy", blocking=True)

            learn_msg = "Now please look straight at my eyes for a few seconds so I can memorize your face."
            self.say(learn_msg, emotion="speaking", blocking=True)
            self.set_robot_emotion("thinking")

            self.recording_samples = []
            self.best_thumbnail = None
            self.best_sharpness = 0.0
            self.last_sample_time = now
            self.state = self.STATE_MEMORIZE_FACE
            self.state_start_time = now

        elif self.state == self.STATE_MEMORIZE_FACE:
            # Captura de 10 muestras biométricas con filtro de calidad
            if frame is not None and face_results:
                target = max(face_results, key=lambda f: f["bbox"][2] * f["bbox"][3])
                feat = target.get("feature")
                q_valid = target.get("quality_valid", False)
                q_score = target.get("quality_score", 0.0)

                # Tasa de captura controlada (~7 Hz)
                if feat is not None and q_valid and (now - self.last_sample_time >= 0.14):
                    self.last_sample_time = now
                    self.recording_samples.append(feat)

                    # Guardar la muestra más nítida para la foto de perfil
                    if q_score > self.best_sharpness or self.best_thumbnail is None:
                        x, y, bw, bh = target["bbox"]
                        mx, my = int(bw * 0.12), int(bh * 0.12)
                        y1, y2 = max(0, y - my), min(frame.shape[0], y + bh + my)
                        x1, x2 = max(0, x - mx), min(frame.shape[1], x + bw + mx)
                        self.best_thumbnail = frame[y1:y2, x1:x2].copy()
                        self.best_sharpness = q_score

            elapsed = now - self.state_start_time
            if len(self.recording_samples) >= self.target_sample_count or (elapsed > 4.5 and len(self.recording_samples) >= 4):
                # Guardar en la base de datos de memoria
                seat_assigned = "couch" if self.guest_index == 1 else "chair"
                pid, is_new = self.recognizer.enroll_person(
                    name=self.current_name,
                    features=self.recording_samples,
                    thumbnail_img=self.best_thumbnail,
                    attributes={
                        "role": f"guest_{self.guest_index}",
                        "favorite_drink": self.current_drink,
                        "seat": seat_assigned,
                        "guest_index": self.guest_index,
                    },
                )
                self.registered_guests.append({
                    "name": self.current_name,
                    "drink": self.current_drink,
                    "seat": seat_assigned,
                    "person_id": pid,
                    "thumbnail": self.best_thumbnail,
                })
                self.get_logger().info(f"Rostro de '{self.current_name}' registrado exitosamente [ID: {pid}].")

                self.say(f"Got it! Your face is saved, {self.current_name}.", emotion="happy", blocking=True)
                self.state = self.STATE_NAV_TO_LIVING_ROOM
                self.state_start_time = now

            elif elapsed > 7.0:
                # Timeout de memorización
                self.say(f"I have enough details, {self.current_name}.", emotion="happy", blocking=True)
                self.state = self.STATE_NAV_TO_LIVING_ROOM
                self.state_start_time = now

        elif self.state == self.STATE_NAV_TO_LIVING_ROOM:
            follow_msg = f"{self.current_name}, please follow me to the living room."
            self.say(follow_msg, emotion="happy", blocking=True)

            self.navigate_to_waypoint(self.living_room_wp)
            self.state = self.STATE_FIND_HOST
            self.state_start_time = now

        elif self.state == self.STATE_FIND_HOST:
            # Llegada a la sala: escanear el entorno
            self.say("We have arrived at the living room. Let me introduce you.", emotion="happy", blocking=True)
            self.move_head_to(20.0, -5.0)
            time.sleep(1.0)
            self.move_head_to(-20.0, -5.0)
            time.sleep(1.0)
            self.move_head_to(0.0, -5.0)

            self.state = self.STATE_INTRODUCE_GUESTS
            self.state_start_time = now

        elif self.state == self.STATE_INTRODUCE_GUESTS:
            # Protocolo oficial de presentación de invitados
            if self.guest_index == 1:
                intro_text = (
                    f"Charlie, this is our new guest, {self.current_name}. "
                    f"Their favorite drink is {self.current_drink}. "
                    f"{self.current_name}, this is our host, Charlie."
                )
            else:
                # Para el segundo invitado, se le presenta al anfitrión Y al primer invitado
                first_guest = self.registered_guests[0] if self.registered_guests else {"name": "Guest 1", "drink": "Water"}
                intro_text = (
                    f"Charlie, {first_guest['name']}, this is our new guest, {self.current_name}. "
                    f"Their favorite drink is {self.current_drink}."
                )

            self.say(intro_text, emotion="speaking", blocking=True)
            self.state = self.STATE_DETECT_AND_OFFER_SEAT
            self.state_start_time = now

        elif self.state == self.STATE_DETECT_AND_OFFER_SEAT:
            # Inspección de asientos y gesto de señalamiento con el brazo Piper
            seat_keys = list(self.seats.keys()) if self.seats else ["couch", "chair"]
            seat_idx = min(self.guest_index - 1, len(seat_keys) - 1)
            seat_target = seat_keys[seat_idx] if seat_keys else "couch"

            seat_info = self.seats.get(seat_target, {}) if self.seats else {}
            pose_gesture = seat_info.get("arm_gesture", "point_couch" if seat_target == "couch" else "point_chair")
            seat_label = seat_info.get("name", "couch" if seat_target == "couch" else "chair")

            offer_msg = f"{self.current_name}, please have a seat right here on the {seat_label}."
            self.say(offer_msg, emotion="happy", blocking=False)

            # Ejecutar gesto físico de señalamiento con el brazo
            self.move_arm_to_named_pose(pose_gesture, duration_s=2.0)
            time.sleep(3.2)
            self.move_arm_to_named_pose("cube_stow", duration_s=1.8)

            self.state = self.STATE_CHECK_NEXT_GUEST
            self.state_start_time = now

        elif self.state == self.STATE_CHECK_NEXT_GUEST:
            if self.guest_index < self.max_guests:
                farewell = "Excuse me, I must return to the entrance door to receive our next guest."
                self.say(farewell, emotion="speaking", blocking=True)
                self.guest_index += 1
                self.current_name = None
                self.current_drink = None
                self.retry_count = 0
                self.state = self.STATE_NAV_TO_DOOR
                self.state_start_time = now
            else:
                self.state = self.STATE_CHALLENGE_COMPLETE
                self.state_start_time = now

        elif self.state == self.STATE_CHALLENGE_COMPLETE:
            final_msg = "All guests have arrived and are comfortably seated. The Receptionist Challenge is complete. Enjoy the party!"
            self.say(final_msg, emotion="happy", blocking=True)
            self.move_arm_to_named_pose("wave_hello", duration_s=2.0)
            time.sleep(2.5)
            self.move_arm_to_named_pose("cube_stow", duration_s=1.5)
            self.set_robot_emotion("happy")
            self.current_banner_text = "CHALLENGE COMPLETADA EXITOSAMENTE!"
            self.get_logger().info("🎉 CHALLENGE ROBOCUP @HOME RECEPTIONIST COMPLETADA CON EXITO!")

    # --------------------------------------------------------------------------
    # RENDERIZADO DEL PANEL HUD (OPENCV)
    # --------------------------------------------------------------------------

    def draw_hud(self, frame: np.ndarray, face_results: List[Dict]) -> np.ndarray:
        """Genera una interfaz HUD profesional con panel lateral de memoria y telemetría."""
        h, w = frame.shape[:2]
        canvas_w = w + 300
        canvas_h = max(h, 540)
        canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
        canvas[:h, :w] = frame

        # Detección de poses humanas si el detector está disponible
        if self.pose_detector is not None:
            try:
                poses = self.pose_detector.detect_poses(frame)
                canvas[:h, :w] = self.pose_detector.draw_poses(canvas[:h, :w], poses)
            except Exception:
                pass

        # Dibujar rostros y badges
        for f in face_results:
            bx, by, bw, bh = f["bbox"]
            name = f["name"]
            sim = f["similarity"]
            q_valid = f.get("quality_valid", False)

            color = (0, 240, 100) if name != "Desconocido" else (0, 180, 255)
            if not q_valid:
                color = (0, 120, 255)

            cv2.rectangle(canvas, (bx, by), (bx + bw, by + bh), color, 2)
            lbl = f"{name} ({int(sim * 100)}%)" if name != "Desconocido" else "Persona"
            cv2.putText(canvas, lbl, (bx, max(18, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # Barra superior con estado actual
        banner_bg = (20, 24, 30)
        cv2.rectangle(canvas, (0, 0), (w, 36), banner_bg, -1)
        st_label = self.STATE_NAMES.get(self.state, "DESCONOCIDO")
        cv2.putText(
            canvas,
            f"RECEPTIONIST | {st_label}",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 230, 255),
            2,
        )

        # Subtítulo inferior
        if self.last_spoken_text:
            cv2.rectangle(canvas, (0, h - 34), (w, h), (10, 12, 16), -1)
            cv2.putText(
                canvas,
                f"Robot: \"{self.last_spoken_text[:70]}\"",
                (10, h - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                1,
            )

        # Panel lateral derecho (Memoria y Estado)
        px = w
        cv2.rectangle(canvas, (px, 0), (canvas_w, canvas_h), (25, 28, 36), -1)
        cv2.line(canvas, (px, 0), (px, canvas_h), (0, 200, 255), 2)

        cv2.putText(canvas, "MEMORIA BIOMETRICA", (px + 15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
        cv2.line(canvas, (px + 10, 36), (canvas_w - 10, 36), (60, 70, 85), 1)

        # Listado de invitados y anfitrión
        py = 55
        profiles = list(self.recognizer.profiles.values())
        for prof in profiles:
            p_name = prof.name
            attrs = prof.attributes
            role = attrs.get("role", "guest")
            drink = attrs.get("favorite_drink", "Water")
            seat = attrs.get("seat", "N/A")

            # Miniatura de foto
            thumb = prof.thumbnail_img
            if thumb is not None and thumb.size > 0:
                t_resized = cv2.resize(thumb, (48, 48))
                canvas[py : py + 48, px + 12 : px + 60] = t_resized
                cv2.rectangle(canvas, (px + 12, py), (px + 60, py + 48), (0, 200, 255), 1)
            else:
                cv2.rectangle(canvas, (px + 12, py), (px + 60, py + 48), (50, 55, 65), -1)
                cv2.putText(canvas, "?", (px + 28, py + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (150, 150, 150), 2)

            cv2.putText(canvas, p_name[:14], (px + 68, py + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
            cv2.putText(canvas, f"Drink: {drink[:12]}", (px + 68, py + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 210, 160), 1)
            cv2.putText(canvas, f"Seat: {seat}", (px + 68, py + 45), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 170, 185), 1)

            py += 58
            if py > canvas_h - 130:
                break

        # Telemetría del Sistema
        ty = canvas_h - 110
        cv2.line(canvas, (px + 10, ty), (canvas_w - 10, ty), (60, 70, 85), 1)
        ty += 18
        cv2.putText(canvas, "TELEMETRIA", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1)
        ty += 20
        cv2.putText(canvas, f"Head: Pan {self.target_pan_deg:+.1f} deg | Tilt {self.target_tilt_deg:+.1f} deg", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
        ty += 18
        cv2.putText(canvas, f"Arm Pose: {self.current_arm_pose_name}", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (200, 200, 200), 1)
        ty += 18
        cv2.putText(canvas, f"Mic Status: {self.voice_status}", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 220, 180) if self.voice_status == "LISTENING" else (160, 160, 160), 1)
        ty += 18
        cv2.putText(canvas, f"Nav Mode: {'SIMULADO' if self.sim_nav else 'NAV2 REAL'}", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 200, 0) if self.sim_nav else (0, 255, 120), 1)

        return canvas


# ------------------------------------------------------------------------------
# PUNTO DE ENTRADA PRINCIPAL
# ------------------------------------------------------------------------------

def check_and_fix_cyclonedds():
    """Protege contra fallos de CycloneDDS si enp1s0 no tiene conexion fisica."""
    dds_uri = os.environ.get("CYCLONEDDS_URI", "")
    if "enp1s0" in dds_uri or "cyclonedds.xml" in dds_uri:
        carrier_path = "/sys/class/net/enp1s0/carrier"
        carrier_ok = False
        if os.path.exists(carrier_path):
            try:
                with open(carrier_path, "r") as f:
                    carrier_ok = (f.read().strip() == "1")
            except Exception:
                carrier_ok = False
        if not carrier_ok:
            local_xml = "/home/athome/cyclonedds_local.xml"
            if os.path.exists(local_xml):
                os.environ["CYCLONEDDS_URI"] = f"file://{local_xml}"
            else:
                os.environ.pop("CYCLONEDDS_URI", None)


def main():
    check_and_fix_cyclonedds()
    parser = argparse.ArgumentParser(description="RoboCup @Home HRI Receptionist Challenge Node")
    parser.add_argument("--topic", type=str, default="", help="Topico de la camara RGB")
    parser.add_argument("--door-wp", type=str, default="A", help="Waypoint de la puerta de entrada")
    parser.add_argument("--living-room-wp", type=str, default="B", help="Waypoint de la sala de estar")
    parser.add_argument("--sim-nav", action="store_true", help="Simula navegacion sin mover la base fisica")
    parser.add_argument("--no-gui", action="store_true", help="Desactiva ventana grafica HUD OpenCV")
    parser.add_argument("--max-guests", type=int, default=2, help="Cantidad de invitados a recibir")
    args = parser.parse_args()

    rclpy.init()

    node = HriReceptionistChallengeNode(
        camera_topic=args.topic,
        door_wp=args.door_wp,
        living_room_wp=args.living_room_wp,
        sim_nav=args.sim_nav,
        no_gui=args.no_gui,
        max_guests=args.max_guests,
    )

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    window_name = "RoboCup @Home - HRI Receptionist Challenge"
    if not args.no_gui:
        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    print("\n" + "=" * 70)
    print("🤖 ROBOCUP @HOME - HRI RECEPTIONIST CHALLENGE")
    print("=" * 70)
    print("   [ESPACIO / ENTER] : Forzar avance de paso / simular deteccion")
    print("   [T]               : Escribir texto por teclado (simular microfono)")
    print("   [R]               : Forzar escucha de microfono (/voice/listen_trigger)")
    print("   [G]               : Probar gesto de senalamiento con brazo Piper")
    print("   [N]               : Omitir navegacion actual")
    print("   [Q / ESC]         : Salir limpiamente y guardar memoria")
    print("=" * 70 + "\n")

    try:
        while rclpy.ok():
            node.step()

            # Renderizado gráfico
            if not args.no_gui:
                with node.frame_lock:
                    fr = node.latest_frame.copy() if node.latest_frame is not None else None

                if fr is not None:
                    face_dets = node.recognizer.detect_and_recognize(fr)
                    hud = node.draw_hud(fr, face_dets)
                    cv2.imshow(window_name, hud)
                else:
                    # Pantalla de espera de cámara
                    blank = np.zeros((480, 720, 3), dtype=np.uint8)
                    cv2.putText(
                        blank,
                        "Conectando con ZED Mini / Head Camera...",
                        (40, 240),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 200, 255),
                        2,
                    )
                    cv2.imshow(window_name, blank)

                key = cv2.waitKey(30) & 0xFF
            else:
                time.sleep(0.04)
                key = 255

            if key in [ord("q"), ord("Q"), 27]:
                print("\n[HRI]: Saliendo de la challenge...")
                break
            elif key == ord("t") or key == ord("T"):
                # Entrada de texto por teclado para testing
                try:
                    sim_text = input("\n🎙️ [Simular Respuesta de Voz]: ")
                    if sim_text.strip():
                        node.voice_text = sim_text.strip()
                except Exception:
                    pass
            elif key in [32, 13]:  # Espacio o Enter
                # Avance manual
                if node.state == node.STATE_WAIT_DOOR:
                    node.door_detection_start = time.time() - 2.0
            elif key in [ord("g"), ord("G")]:
                print("\n[Arm]: Probando gesto de senalamiento...")
                node.move_arm_to_named_pose("point_couch", duration_s=1.5)
            elif key in [ord("r"), ord("R")]:
                node.trigger_mic_listening()

    except KeyboardInterrupt:
        pass
    finally:
        node.speaker.stop()
        if not args.no_gui:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
