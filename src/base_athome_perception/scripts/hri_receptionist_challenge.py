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
# Optional --wait-doorbell waits up to --doorbell-timeout-s (default 30 s) before
# each trip to the door. --start-doorbell-audio also enables --wait-doorbell and
# starts a companion publisher using --doorbell-audio-device (PortAudio default
# if empty). Otherwise --doorbell-audio-topic needs an external live PCM source:
# std_msgs/Int16MultiArray, mono 16 kHz, <=1600 samples per message. Timeout continues without
# claiming detection. ROS parameters have the same names with underscores.
# Guest 1's trusted torso/legs colors and garment types are saved and described
# to Guest 2 in one sentence (<=20 words); unsupported attributes are omitted.
# ==============================================================================

import argparse
import datetime
import json
import math
import os

os.environ.setdefault("YOLO_OFFLINE", "1")  # ultralytics: nunca auto-instalar paquetes en el venv
import queue
import re
import subprocess
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

# ROS 2 Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Vector3, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import String, Float32, Bool, Int16MultiArray
from std_srvs.srv import SetBool, Trigger

# CV Bridge
try:
    from cv_bridge import CvBridge
except ImportError:
    class CvBridge:
        def imgmsg_to_cv2(self, img_msg, desired_encoding="bgr8"):
            im = np.frombuffer(img_msg.data, dtype=np.uint8)
            im = im.reshape((img_msg.height, img_msg.width, -1))
            if desired_encoding == "bgr8" and img_msg.encoding == "rgb8":
                return cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
            return im.copy()

# Nav2 BasicNavigator
try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    NAV2_AVAILABLE = True
except ImportError:
    BasicNavigator = None
    TaskResult = None
    ActionClient = None
    NavigateToPose = None
    NAV2_AVAILABLE = False

# Base@Home Perception Modules
from base_athome_perception.face_recognizer import DEFAULT_DB_DIR, FaceRecognizer, PersonProfile
from base_athome_perception.guest_vocabulary import GuestVocabulary
from base_athome_perception.guest_input_server import GuestInputServer
from base_athome_perception import seat_occupancy as seatocc
from base_athome_perception import scene_analysis as scene
from base_athome_perception.speaker import RoboCupSpeaker
from base_athome_perception.doorbell_detector import DoorbellDetector
from base_athome_perception.receptionist_attributes import (
    describe_guest_attributes, extract_guest_attributes, prefer_attributes,
)
from base_athome_perception.receptionist_doorbell import (
    doorbell_wait_status, validate_doorbell_timeout,
)
try:
    from base_athome_perception.person_pose_detector import PersonPoseDetector
except ImportError:
    PersonPoseDetector = None
from base_athome_perception.piper_kinematics import (
    SAFE_JOINT_LIMITS_DEG,
    DEFAULT_NAMED_POSES,
)

# ------------------------------------------------------------------------------
# VOCABULARIOS Y EXTRACCION DE DIALOGO ROBUSTA (NLP)
# ------------------------------------------------------------------------------

from base_athome_perception.guest_dialog import (COMMON_DRINKS, DRINK_TRANSLATIONS, STOP_WORDS,  # noqa: F401
                                                    extract_name_and_drink, parse_yes_no)
from base_athome_perception.voice_client import VoiceClient, VoiceResult

try:
    from base_athome_skills.door_detector import (
        DoorDetectorConfig,
        DoorOpenDetector,
        window_distance,
    )
except ImportError:
    _ws_skills = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "base_athome_skills",
    )
    if os.path.isdir(_ws_skills) and _ws_skills not in sys.path:
        sys.path.insert(0, _ws_skills)
    from base_athome_skills.door_detector import (
        DoorDetectorConfig,
        DoorOpenDetector,
        window_distance,
    )


def is_start_speech(text: str) -> bool:
    """Verifica si el texto corresponde a una señal verbal de inicio (Regla 3.6.8)."""
    if not text:
        return False
    t = text.lower().strip()
    if re.search(r"\b(no|not|don't|dont|stop|wait|cancel|espera)\b", t):
        return False
    pattern = r"\b(start|begin|go|ready|arranca|arrancar|empeza|empezar|inicia|iniciar)\b"
    return bool(re.search(pattern, t))


# Ruta del ejecutable en la linea de comandos del driver real (evita coincidir con shells
# que solo mencionan el nombre, p.ej. un "pgrep realsense2_camera_node" en otra terminal)
RS_PROC_PATTERN = r"^/opt/ros/[a-z]+/lib/realsense2_camera/realsense2_camera_node"
# Escucha: whisper_node espera hasta LISTEN_START_TIMEOUT_S a que la persona empiece a hablar y corta
# solo ~1 s despues de que termina (ya no hay ventana fija). Cada pedido termina con un resultado.
LISTEN_START_TIMEOUT_S = 8.0
YES_NO_PROMPT = "Answer yes or no. Yes. No. Yes, that is correct. No, that is wrong."

# ------------------------------------------------------------------------------
# CONSTANTES DE POSES Y GESTOS DEL BRAZO PIPER
# ------------------------------------------------------------------------------

ARM_GESTURE_POSES = {
    # Reposo real medido por CAN (2026-10-30). Los gestos 'point_*'/'wave' son del montaje anterior:
    # volver a ensenarlos antes de usar --arm-gestures.
    'home': [86.97, 172.87, -162.63, -99.89, -0.68, 44.98],
    'cube_stow': [86.97, 172.87, -162.63, -99.89, -0.68, 44.98],
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
    STATE_SCAN_SCENE = 14
    STATE_WAIT_SEATED = 15
    STATE_DONE = 16  # terminal: antes COMPLETE se re-ejecutaba en bucle (repetia el discurso final)
    STATE_ASK_CONFIRM = 17     # "Your name is X and you like Y. Is that correct?"
    STATE_WAIT_CONFIRM = 18
    STATE_WAIT_DOORBELL = 19

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
        STATE_SCAN_SCENE: "6/7: ANALIZANDO LA SALA CON LA CABEZA",
        STATE_WAIT_SEATED: "7/7: ESPERANDO QUE EL INVITADO SE SIENTE",
        STATE_DONE: "7/7: TERMINADO",
        STATE_ASK_CONFIRM: "3/7: VERIFICANDO LO QUE ENTENDI",
        STATE_WAIT_CONFIRM: "3/7: ESCUCHANDO SI / NO",
        STATE_WAIT_DOORBELL: "ESPERANDO TIMBRE ANTES DE IR A LA PUERTA",
    }
    # Estados en los que la cabeza sigue la cara del interlocutor
    TRACKING_STATES = {STATE_WAIT_DOOR, STATE_GREET_AND_PROMPT, STATE_WAIT_NAME_DRINK, STATE_PROMPT_MISSING_INFO,
                       STATE_CONFIRM_DETAILS, STATE_MEMORIZE_FACE, STATE_ASK_CONFIRM, STATE_WAIT_CONFIRM}
    # Seguimiento LEVE (por defecto) solo mientras se registra al invitado: mirar a quien habla aunque sea alto
    REGISTRATION_STATES = {STATE_GREET_AND_PROMPT, STATE_WAIT_NAME_DRINK, STATE_PROMPT_MISSING_INFO,
                           STATE_CONFIRM_DETAILS, STATE_MEMORIZE_FACE, STATE_ASK_CONFIRM, STATE_WAIT_CONFIRM}

    def __init__(
        self,
        camera_topic: str = "",
        door_wp: str = "A",
        living_room_wp: str = "B",
        sim_nav: bool = False,
        no_gui: bool = False,
        wait_start: bool = False,
        max_guests: int = 2,
        arm_gestures: bool = False,
        track_faces: bool = False,
        gentle_tracking: bool = True,
        door_config: Optional[Any] = None,
        scan_topic: str = "/scan",
        wait_doorbell: bool = False,
        doorbell_timeout_s: float = 30.0,
        doorbell_audio_topic: str = "/hri/doorbell/audio",
    ):
        super().__init__("hri_receptionist_challenge_node")

        self.door_wp = door_wp
        self.living_room_wp = living_room_wp
        self.sim_nav = sim_nav
        self.no_gui = no_gui
        self.wait_start = wait_start      # esperar inicio (puerta, voz o ESPACIO)
        self.start_confirmed = False
        self._last_voice_listen_attempt = 0.0
        self.door_detector = DoorOpenDetector(cfg=door_config or DoorDetectorConfig())
        self.scan_topic = scan_topic
        self.wait_doorbell = self.declare_parameter("wait_doorbell", wait_doorbell).value
        self.doorbell_timeout_s = validate_doorbell_timeout(
            self.declare_parameter("doorbell_timeout_s", doorbell_timeout_s).value)
        self.doorbell_audio_topic = self.declare_parameter(
            "doorbell_audio_topic", doorbell_audio_topic).value
        self.doorbell_lock = threading.Lock()
        self.doorbell_started_at = None
        self.doorbell_detected_at = None
        self.doorbell_detector = DoorbellDetector(armed=False)
        if self.wait_doorbell:
            # New input contract: mono PCM int16, 16000 Hz, <=1600 samples/message.
            # Volatile sensor QoS avoids replaying historical audio on subscription.
            self.create_subscription(Int16MultiArray, self.doorbell_audio_topic,
                                     self._doorbell_audio_callback, qos_profile_sensor_data)
        self._pt_retest_requested = False
        self.max_guests = max_guests
        self.arm_gestures = arm_gestures
        # Estado MEDIDO del brazo (/arm/status del controlador en la Jetson)
        self.arm_feedback_ok = False
        self.arm_enabled = False

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
        # Biometria fuera del repo (no versionar caras). Migra la base vieja una vez.
        self.db_dir = os.environ.get("ATHOME_FACES_DB", DEFAULT_DB_DIR)
        if os.environ.get("ATHOME_SIM") != "1":
            self._migrate_legacy_db(os.path.join(self.config_dir, "faces_database"), self.db_dir)
        self.session_id = f"receptionist_{datetime.datetime.now():%Y%m%d_%H%M%S}"

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
            session_id=self.session_id,  # invitados = memoria temporal de esta sesion
        )
        self.get_logger().info(
            f"Memoria facial: {self.db_dir} | detector {'YuNet+alineacion' if self.recognizer.aligned else 'SSD sin alinear'}"
            f" | umbral {self.recognizer.cosine_thresh:.2f} | sesion {self.session_id}")
        self.vocab = GuestVocabulary()
        # Alternativa al microfono: pantalla tactil/teclado (http://127.0.0.1:8090) -> /hri/guest_input
        self.guest_inputs: "queue.Queue[dict]" = queue.Queue()
        self.pub_guest_input = self.create_publisher(String, "/hri/guest_input", 10)
        self.create_subscription(String, "/hri/guest_input", self._guest_input_cb, 10)
        try:
            self.guest_screen = GuestInputServer(self.vocab.names, self.vocab.drinks,
                                                 lambda d: self.pub_guest_input.publish(String(data=json.dumps(d))))
            self.get_logger().info(f"Pantalla del invitado: http://127.0.0.1:{self.guest_screen.port}")
        except OSError as e:
            self.guest_screen = None
            self.get_logger().warn(f"Pantalla del invitado no disponible: {e}")
        self.get_logger().info(f"Vocabulario de invitados: {len(self.vocab.names)} nombres, "
                               f"{len(self.vocab.drinks)} bebidas ({self.vocab.source})")

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
        # El pan-tilt arranca sin torque (auto_torque=false): se habilita al primer movimiento.
        self.cli_pantilt_torque = self.create_client(SetBool, "/pan_tilt/set_torque")
        # Realimentacion del pan-tilt para la revision de arranque (posicion MEDIDA, no la comandada)
        self.pt_connected = None
        self.pt_meas = None        # (pan_deg, tilt_deg, time)
        self.pt_selftest = None    # resultado de pan_tilt_self_test()
        self.create_subscription(Bool, "/pan_tilt/connected", self._pt_connected_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/pan_tilt/joint_states", self._pt_joint_cb, 10)
        self._pantilt_torque_requested = False
        # Pose del robot (solo lectura de AMCL) para orientar la cabeza hacia los asientos.
        self.robot_pose = None  # (x, y, yaw) en frame map
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 10)
        self.head_pan_sign = 1.0  # +1: pan positivo = izquierda (REP-103); nodo corregido el 2026-11-04 (pan_direction +1)
        self.fallback_guest_pan_deg = [25.0, -25.0]  # si no hay pose: invitado 1 / invitado 2
        self.pub_arm_joint = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_arm_gripper = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_torso_joint_states = self.create_publisher(JointState, "/torso/joint_states", 10)
        # torso_lift_joint tambien en /joint_states: el robot_state_publisher de la navegacion no usa
        # joint_state_publisher y sin este joint no hay TF de cabeza, camara ni brazo. Unico dueno del torso
        # hasta que exista un nodo del elevador.
        self.pub_joint_states_torso = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_arm_joint_states_cmd = self.create_publisher(JointState, "/arm/joint_states_cmd", 10)
        self.pub_voice_mute = self.create_publisher(Bool, "/voice/mute", 10)
        self._mute_lock = threading.Lock()
        self._mute_refs = 0
        self.pub_voice_prompt = self.create_publisher(String, "/voice/initial_prompt", 10)

        # Cliente para activar microfono /voice/listen_trigger
        self.cli_mic_trigger = self.create_client(Trigger, "/voice/listen_trigger")
        # Escucha con id y resultado explicito (ok / no_speech / offline ...): ver voice_client.py
        self.voice = VoiceClient(self, start_timeout=LISTEN_START_TIMEOUT_S)
        self.listen_id: Optional[str] = None
        self.voice_failures = 0         # escuchas seguidas sin entender nada
        self.confirm_tries = 0
        self.ask_confirmation = os.environ.get("ATHOME_HRI_CONFIRM") == "1"
        self.from_screen = False
        self.last_heard_text = ""
        # Boton/servicio para vaciar la memoria temporal de personas
        self.srv_clear_memory = self.create_service(Trigger, "/hri/clear_memory", self._srv_clear_memory)
        self.clear_memory_requested = False
        self.clear_button_rect = None  # (x1, y1, x2, y2) en la ventana HUD
        # Sesgar Whisper hacia nombres/bebidas oficiales (latched para nodos que arranquen despues)
        self.timer_prompt = self.create_timer(5.0, self._publish_voice_prompt)

        # 6. Suscriptores ROS 2
        self.bridge = CvBridge()
        self.frame_lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.last_frame_time: float = 0.0

        # Best effort + depth 1: nunca se acumulan frames viejos ni se frena al publicador
        qos_cam = qos_profile_sensor_data

        candidate_topics = [
            "/camera/camera/color/image_raw",
            "/camera/color/image_raw",
            "/zed/zed_node/rgb/image_rect_color",
            "/zed/zed_node/left/image_rect_color",
            "/zed/rgb/image_rect_color",
        ]
        if camera_topic:
            candidate_topics.insert(0, camera_topic)
        unique_topics = list(dict.fromkeys(candidate_topics))
        self.camera_topic_name = unique_topics[0]

        self.sub_cam_list = []
        for top in unique_topics:
            self.sub_cam_list.append(
                self.create_subscription(Image, top, self.image_callback, qos_cam)
            )

        self.create_subscription(String, "/arm/status", self._arm_status_cb, 10)

        # Profundidad alineada + intrinsecos: ubicar personas en el mapa (ocupacion de asientos)
        self.depth_lock = threading.Lock()
        self.latest_depth_m: Optional[np.ndarray] = None
        self.cam_fx_cx: Optional[Tuple[float, float]] = None
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw",
                                 self._depth_callback, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info",
                                 self._caminfo_callback, qos_profile_sensor_data)

        # Suscriptor a Whisper ASR
        self.sub_voice = self.create_subscription(
            String, "/voice/transcription", self.voice_callback, 10
        )
        self.sub_voice_status = self.create_subscription(
            String, "/voice/status", self.voice_status_callback, 10
        )

        # Suscriptor a Hokuyo LaserScan para detección de puerta abierta (Regla 3.6.8)
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data
        )

        # 7. Variables de la Máquina de Estados
        self.state = self.STATE_INIT
        self.state_start_time = time.time()
        self.guest_index = 1
        self.registered_guests: List[Dict] = []

        # Datos temporales del invitado actual
        self.current_name: Optional[str] = None
        self.current_drink: Optional[str] = None
        self.current_attributes: Dict = {}
        self.missing_field: Optional[str] = None
        self.retry_count = 0
        self.door_detection_start: Optional[float] = None
        self.last_person_seen_time: Optional[float] = None
        self.last_door_prompt_time: float = 0.0
        self.force_advance: bool = False

        # Grabación biométrica
        self.recording_samples: List[np.ndarray] = []
        self.samples_synthetic = False
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
        # Seguimiento de caras con el pan-tilt: desactivado (era demasiado agresivo); la cabeza mira al
        # frente durante la conversacion. Se puede activar con --track-faces.
        self.pan_tilt_tracking = track_faces
        self._gentle_requested = gentle_tracking
        self.gentle_tracking = gentle_tracking
        self.gentle_tracker = scene.GentleFaceTracker()

        # Telemetría del brazo
        self.current_arm_pose_name = "home"
        self.arm_joints_deg = list(ARM_GESTURE_POSES['home'])

        # Hilo de publicación de /joint_states a 25 Hz
        self.timer_joints = self.create_timer(0.04, self._publish_joint_states)

        # Subtítulo y banner activo en el HUD
        self.current_banner_text = "Iniciando pre-vuelo y sincronizacion de sensores..."
        self.last_spoken_text = ""

        # Pre-vuelo y sincronización inicial de subsistemas
        self.last_camera_wait_log: float = 0.0
        self.nav2_verified: bool = False
        self.preflight_complete: bool = False
        self.session_file: str = os.path.join(self.db_dir, "receptionist_session.json")
        self.session_start_time: float = time.time()
        self.session_start_str: str = datetime.datetime.fromtimestamp(self.session_start_time).strftime("%Y-%m-%d %H:%M:%S")
        self._init_session_storage()

        # Iniciar posición Home del brazo y emoción neutral
        self.set_robot_emotion("neutral")

        # Estado de la GUI OpenCV
        self.window_name = "RoboCup @Home - HRI Receptionist Challenge"
        self.request_shutdown = False
        self.current_face_dets: List[Dict] = []
        self.current_poses: List[Dict] = []
        self.frame_count = 0
        self.last_face_time = 0.0
        self.last_pose_time = 0.0
        self.cam_proc: Optional[subprocess.Popen] = None
        self.cam_last_restart = 0.0
        self._ensure_camera_running()
        self.timer_cam_watchdog = self.create_timer(1.0, self._camera_watchdog)

        # Reconocimiento facial en un hilo propio: la ventana sigue a la tasa de la camara
        self.face_lock = threading.Lock()
        self.scanning = False
        self.furniture_model = None
        self.scene_objects: List = []
        self.seat_bearing_deg: Optional[float] = None
        self.seated_hits = 0
        self.seat_reminded = False
        self.face_dets_stamp = 0.0
        self.visual_attributes = {}
        self.visual_attributes_stamp = 0.0
        self.last_tracked_stamp = 0.0
        self.face_worker_stop = threading.Event()
        self.face_worker = threading.Thread(target=self._face_worker_loop, daemon=True)
        self.face_worker.start()

        self.get_logger().info("HriReceptionistChallengeNode inicializado. Esperando sensores y pre-vuelo...")

    @staticmethod
    def _rs_service_installed() -> bool:
        return subprocess.run(["systemctl", "--user", "cat", "realsense-nuc"],
                              capture_output=True).returncode == 0

    def _ensure_camera_running(self):
        """Verifica si la RealSense D435 está conectada y si el driver está activo; si no, lo inicia.

        Si existe el servicio realsense-nuc (driver unico supervisado), se usa ese.
        En simulacion (ATHOME_SIM=1) las camaras las publica sim_robot: no se arranca nada.
        """
        if os.environ.get("ATHOME_SIM") == "1":
            return
        if self._rs_service_installed():
            if subprocess.run(["systemctl", "--user", "is-active", "--quiet", "realsense-nuc"]).returncode != 0:
                self.get_logger().info("📷 Iniciando servicio realsense-nuc...")
                subprocess.run(["systemctl", "--user", "start", "realsense-nuc"])
                self.cam_last_restart = time.time()
            return
        try:
            out = subprocess.check_output(["pgrep", "-f", RS_PROC_PATTERN], text=True)
            if out.strip():
                return
        except Exception:
            pass

        try:
            lsusb_out = subprocess.check_output(["lsusb"], text=True)
            if "8086:0b07" in lsusb_out or "RealSense" in lsusb_out:
                self.get_logger().info("📷 RealSense D435 detectada en NUC por USB. Iniciando driver automáticamente...")
                cmd = (
                    "bash -c 'source /opt/ros/jazzy/setup.bash && "
                    "source /home/athome/base_athome_ws/install/setup.bash 2>/dev/null && "
                    "exec ros2 launch base_athome_bringup realsense_nuc.launch.py'"
                )
                log = open("/tmp/realsense_nuc.log", "ab")
                self.cam_proc = subprocess.Popen(cmd, shell=True, start_new_session=True,
                                                 stdout=log, stderr=subprocess.STDOUT)
                self.cam_last_restart = time.time()
        except Exception as e:
            self.get_logger().warn(f"No se pudo autoiniciar realsense2_camera_node: {e}")

    def _camera_watchdog(self):
        """Si la camara deja de mandar frames: relanza el driver (como mucho cada 20 s)."""
        now = time.time()
        ref = self.last_frame_time if self.last_frame_time > 0.0 else self.session_start_time + 4.0
        if now - ref < 4.0:
            return
        if now - self.cam_last_restart < 20.0:
            return
        running = subprocess.run(["pgrep", "-f", RS_PROC_PATTERN], capture_output=True).returncode == 0
        gap = now - ref
        if running and gap < 15.0:
            return  # puede ser un hipo: esperar un poco antes de reiniciar
        self.get_logger().warn(f"📷 Camara sin frames hace {gap:.1f}s (driver {'colgado' if running else 'caido'}). Reiniciando driver...")
        self.cam_last_restart = now
        if self._rs_service_installed():
            subprocess.run(["systemctl", "--user", "restart", "realsense-nuc"])
            return
        if running:
            subprocess.run(["pkill", "-f", RS_PROC_PATTERN])
            time.sleep(2.0)
        self._ensure_camera_running()

    def _face_worker_loop(self):
        """Procesa el ultimo frame disponible (~10 Hz) sin bloquear la GUI ni la maquina de estados."""
        last_count = -1
        while not self.face_worker_stop.is_set():
            with self.frame_lock:
                frame = self.latest_frame
                count = self.frame_count
            if frame is None or count == last_count:
                time.sleep(0.02)
                continue
            last_count = count
            t0 = time.time()
            try:
                dets = self.recognizer.detect_and_recognize(frame)
                poses = None
                attributes = None
                if self.pose_detector is not None and t0 - self.last_pose_time >= 0.28:
                    self.last_pose_time = t0
                    poses = self.pose_detector.detect_poses(frame)
                    if self.guest_index == 1 and self.state in self.REGISTRATION_STATES:
                        attributes = extract_guest_attributes(frame, poses, dets)
                with self.face_lock:
                    self.current_face_dets = dets
                    self.face_dets_stamp = t0
                    if poses is not None:
                        self.current_poses = poses
                    if attributes is not None:
                        self.visual_attributes = attributes
                        self.visual_attributes_stamp = t0
            except Exception as e:
                self.get_logger().warn(f"Error en reconocimiento facial: {e}", throttle_duration_sec=5.0)
            time.sleep(max(0.0, 0.1 - (time.time() - t0)))

    def _publish_voice_prompt(self):
        self.pub_voice_prompt.publish(String(data=self.vocab.whisper_prompt()))

    def _srv_clear_memory(self, req, resp):
        self.clear_memory_requested = True
        resp.success = True
        resp.message = "Memoria temporal de personas: limpieza solicitada."
        return resp

    def clear_temporary_memory(self):
        """Vacia invitados de la memoria (perfiles no persistentes) y reinicia la prueba al invitado 1."""
        n = self.recognizer.clear_session()
        self.registered_guests = []
        self.guest_index = 1
        self.current_name = None
        self.current_drink = None
        self.current_attributes = {}
        # Worker observations predating the next registration are rejected by step().
        self.recording_samples = []
        self.best_thumbnail = None
        self._init_session_storage()
        self.current_banner_text = f"Memoria temporal limpia ({n} personas borradas)"
        self.get_logger().info(f"🧹 Memoria temporal de personas limpiada: {n} perfiles.")

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.clear_button_rect is not None:
            x1, y1, x2, y2 = self.clear_button_rect
            if x1 <= x <= x2 and y1 <= y <= y2:
                self.clear_memory_requested = True

    @staticmethod
    def _migrate_legacy_db(old_dir: str, new_dir: str) -> None:
        if os.path.isfile(os.path.join(new_dir, "profiles.json")) or not os.path.isfile(os.path.join(old_dir, "profiles.json")):
            return
        import shutil
        os.makedirs(new_dir, exist_ok=True)
        for item in ("profiles.json", "features.pkl", "thumbnails"):
            src = os.path.join(old_dir, item)
            if os.path.isdir(src):
                shutil.copytree(src, os.path.join(new_dir, item), dirs_exist_ok=True)
            elif os.path.isfile(src):
                shutil.copy2(src, new_dir)
        print(f"[HRI] Memoria facial migrada de {old_dir} a {new_dir}")

    def destroy_node(self):
        # El driver de la RealSense queda corriendo a proposito: relanzarlo en cada corrida
        # era una causa de cortes (USB re-enumerando). Para cerrarlo: pkill -f realsense2_camera_node
        if hasattr(self, "face_worker_stop"):
            self.face_worker_stop.set()
        worker = getattr(self, "face_worker", None)
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=5.0)
        super().destroy_node()

    # --------------------------------------------------------------------------
    # GESTION DE WAYPOINTS Y PERFILES
    # --------------------------------------------------------------------------

    def _load_waypoints(self) -> Dict[str, Dict]:
        """Carga waypoints, zonas y asientos desde semantic_map.yaml o waypoints.yaml."""
        semantic_path = os.path.join(
            self.pkg_dir, "..", "base_athome_navigation", "config", "semantic_map.yaml"
        )
        if os.environ.get("ATHOME_SIM") == "1":
            semantic_path = os.environ.get("ATHOME_SIM_SEMANTIC_MAP") or semantic_path
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
        host_exists = any(p.name.lower() == "charlie" for p in self.recognizer.session_profiles())
        if not host_exists:
            # Crear perfil formal del anfitrión
            dummy_feat = np.random.randn(1, 128).astype(np.float32)
            dummy_feat /= np.linalg.norm(dummy_feat)
            self.recognizer.enroll_person(
                name="Charlie",
                features=[dummy_feat],
                attributes={"role": "host", "favorite_drink": "Water", "seat": "sofa_left", "synthetic": True},
                persistent=True,
            )
            self.get_logger().info("Perfil del anfitrion 'Charlie' registrado en la memoria.")

    def _init_session_storage(self) -> None:
        """Inicializa el archivo de registro persistente de la sesión de invitados."""
        os.makedirs(self.db_dir, exist_ok=True)
        os.makedirs(os.path.join(self.db_dir, "thumbnails"), exist_ok=True)
        initial_session = {
            "session_id": f"receptionist_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}",
            "started_at": self.session_start_str,
            "status": "RUNNING",
            "host": {
                "name": "Charlie",
                "role": "host",
                "seat_zone": "living_room"
            },
            "guests_count": 0,
            "guests": []
        }
        try:
            with open(self.session_file, "w", encoding="utf-8") as f:
                json.dump(initial_session, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"✔ [Almacenamiento]: Registro de sesión inicializado en {self.session_file}")
        except Exception as e:
            self.get_logger().warn(f"No se pudo inicializar receptionist_session.json: {e}")

    def _update_session_storage(self, guest_data: Dict) -> None:
        """Actualiza en tiempo real los datos del invitado en receptionist_session.json."""
        if not hasattr(self, "session_file") or not os.path.exists(self.session_file):
            self._init_session_storage()

        try:
            with open(self.session_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            data["guests_count"] = len(self.registered_guests)
            updated = False
            for i, g in enumerate(data.get("guests", [])):
                if g.get("guest_index") == guest_data.get("guest_index"):
                    data["guests"][i] = guest_data
                    updated = True
                    break
            if not updated:
                data.setdefault("guests", []).append(guest_data)

            with open(self.session_file, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"✔ [Almacenamiento]: Datos de '{guest_data.get('name')}' guardados en {self.session_file}")
        except Exception as e:
            self.get_logger().warn(f"Error actualizando receptionist_session.json: {e}")

    def _finalize_session_storage(self) -> None:
        """Marca la sesión de Receptionist como completada exitosamente."""
        if not hasattr(self, "session_file") or not os.path.exists(self.session_file):
            return
        try:
            with open(self.session_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            data["status"] = "COMPLETED"
            data["finished_at"] = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            data["total_guests_served"] = len(self.registered_guests)
            with open(self.session_file, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"🎉 [Almacenamiento]: Sesión completada y guardada en {self.session_file}")
        except Exception as e:
            self.get_logger().warn(f"Error finalizando sesión: {e}")

    def _check_nav2_ready(self, timeout_sec: float = 2.0) -> bool:
        """Verifica si el servidor de acción NavigateToPose de Nav2 está activo."""
        if self.navigator is None:
            return False
        try:
            if hasattr(self.navigator, '_action_client') and self.navigator._action_client:
                return self.navigator._action_client.server_is_ready()
            return True
        except Exception as e:
            self.get_logger().debug(f"Error comprobando servidor Nav2: {e}")
            return False

    # --------------------------------------------------------------------------
    # COMUNICACION MULTI-MODAL: HABLA, EMOCIONES Y CABEZAL
    # --------------------------------------------------------------------------

    # --------------------------------------------------------------------------
    # GESTION DE INTERFAZ GRAFICA (OPENCV HUD SIN CONGELAMIENTO)
    # --------------------------------------------------------------------------

    def pump_gui(self, delay_ms: int = 15) -> int:
        """Renderiza el cuadro en vivo de la camara RealSense con HUD y bombea eventos OpenCV a ~40-60 FPS."""
        if self.no_gui or self.request_shutdown:
            time.sleep(delay_ms / 1000.0)
            return 255

        with self.frame_lock:
            fr = self.latest_frame.copy() if self.latest_frame is not None else None

        if fr is not None:
            hud = self.draw_hud(fr, self.current_face_dets)
            cv2.imshow(self.window_name, hud)
        else:
            self._draw_waiting_screen()

        # Pausa de 5ms para ceder el GIL de Python y permitir que el hilo de ROS reciba frames continuamente
        time.sleep(0.005)
        key = cv2.waitKey(max(1, delay_ms)) & 0xFF
        if key != 255:
            self.handle_key(key)
        return key

    def sleep_gui(self, duration_s: float):
        """Espera de tiempo activa que mantiene la camara y la ventana OpenCV vivas y actualizandose."""
        t_end = time.time() + duration_s
        while time.time() < t_end and rclpy.ok() and not self.request_shutdown:
            self.pump_gui(20)

    def _draw_waiting_screen(self):
        """Pantalla visual de prevuelo mientras se conecta el stream de la camara."""
        blank = np.zeros((540, 840, 3), dtype=np.uint8)
        cv2.rectangle(blank, (20, 20), (820, 520), (35, 45, 55), 2)
        cv2.putText(blank, "ROBOCUP @HOME - PRE-VUELO RECEPTIONIST", (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 210, 255), 2)
        cv2.line(blank, (40, 75), (800, 75), (50, 65, 80), 1)

        cv2.putText(blank, "Verificando y sincronizando subsistemas requeridos:", (40, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (220, 220, 220), 1)
        cv2.putText(blank, f"[...] Camara RealSense D435 : Conectando a {self.camera_topic_name}...", (40, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 170, 255), 1)

        nav_str = "Modo Simulacion Activo" if self.sim_nav else "Esperando / Verificando Nav2..."
        cv2.putText(blank, f"[*] Navegacion Nav2   : {nav_str}", (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
        cv2.putText(blank, f"[v] Registro Biometrico: Listo ({os.path.basename(self.session_file)})", (40, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 240, 140), 1)
        cv2.putText(blank, f"[v] Modulo de Voz      : {self.voice_status}", (40, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 240, 140), 1)
        cv2.putText(blank, f"[v] Mapa Semantico     : {len(self.zones)} zonas | {len(self.waypoints)} waypoints | {len(self.seats)} asientos", (40, 320), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 240, 140), 1)

        cv2.line(blank, (40, 420), (800, 420), (50, 65, 80), 1)
        cv2.putText(blank, "ATENCION: Verifique que la camara RealSense D435 este transmitiendo.", (40, 455), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (255, 200, 0), 1)
        cv2.putText(blank, "Atajo de pruebas: Presione [ESPACIO] para generar frame sintetico y avanzar.", (40, 485), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (140, 160, 180), 1)
        cv2.imshow(self.window_name, blank)

    def handle_key(self, key: int):
        """Gestiona atajos de teclado interactivos para pruebas y depuracion."""
        if key in [ord("q"), ord("Q"), 27]:
            print("\n[HRI]: Saliendo de la challenge...")
            self._finalize_session_storage()
            self.request_shutdown = True
        elif key in [ord("p"), ord("P")] and self.state == self.STATE_INIT and self.preflight_complete:
            self._pt_retest_requested = True     # volver a revisar la cabeza antes de arrancar
        elif key in [32, 13]:  # Espacio o Enter
            print(f"\n[Manual]: Tecla [ESPACIO/ENTER] presionada en estado '{self.STATE_NAMES.get(self.state, self.state)}'. Forzando avance...")
            self.force_advance = True
            if self.state == self.STATE_INIT and self.wait_start:
                self.force_advance = False       # que no saltee el primer paso de la mision
                if self.preflight_complete:
                    self.start_confirmed = True
                else:
                    print("   [Manual]: Todavia esta cargando; espera el aviso para arrancar.")
            elif self.state == self.STATE_INIT:
                if self.latest_frame is None:
                    with self.frame_lock:
                        self.latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)
                        self.last_frame_time = time.time()
            elif self.state == self.STATE_WAIT_DOOR:
                self.state = self.STATE_GREET_AND_PROMPT
                self.state_start_time = time.time()
                self.door_detection_start = None
            elif self.state == self.STATE_WAIT_NAME_DRINK:
                print("   [Manual]: Asignando datos de demo por teclado: 'Alex' y 'Lemonade'")
                self.current_name = "Alex" if self.guest_index == 1 else "Taylor"
                self.current_drink = "Lemonade" if self.guest_index == 1 else "Coffee"
                self.voice.cancel(self.listen_id)
                self.listen_id = None
                self.state = self.STATE_CONFIRM_DETAILS
                self.state_start_time = time.time()
            elif self.state == self.STATE_MEMORIZE_FACE:
                if not self.recording_samples and getattr(self, "fallback_samples", None):
                    self.recording_samples = list(self.fallback_samples)
                    self.get_logger().warn(f"Rostro guardado con {len(self.recording_samples)} muestras de baja calidad")
                if not self.recording_samples:
                    self.get_logger().error("No pude ver el rostro del invitado: perfil SIN biometria (no se reconocera luego)")
                    dummy_f = np.random.randn(1, 128).astype(np.float32)
                    dummy_f /= np.linalg.norm(dummy_f)
                    self.recording_samples.append(dummy_f)
                    self.samples_synthetic = True
                self.state_start_time = time.time() - 10.0
        elif key in [ord("t"), ord("T")]:
            def _read_text():  # en otro hilo: input() no debe congelar la camara
                try:
                    sim_text = input("\n🎙️ [Simular Respuesta de Voz]: ")
                    if sim_text.strip():
                        if self.voice.inject(sim_text.strip()) is None:
                            self.voice_text = sim_text.strip()
                        self.get_logger().info(f"🎙️ [Manual Texto]: '{self.voice_text}'")
                except Exception:
                    pass
            threading.Thread(target=_read_text, daemon=True).start()
        elif key in [ord("m"), ord("M")]:
            print("\n[Memoria]: Limpiando memoria temporal de personas...")
            self.clear_memory_requested = True
        elif key in [ord("g"), ord("G")]:
            print("\n[Arm]: Probando gesto de senalamiento...")
            self.move_arm_to_named_pose("point_couch", duration_s=1.5)
        elif key in [ord("r"), ord("R")]:
            print("\n[Mic]: Forzando escucha de microfono...")
            self.start_listening(beep=False)
        elif key in [ord("n"), ord("N")]:
            print("\n[Nav]: Omitiendo navegacion en curso...")
            if self.navigator and not self.sim_nav:
                self.navigator.cancelTask()

    # --------------------------------------------------------------------------
    # COMUNICACION MULTI-MODAL: HABLA, EMOCIONES Y CABEZAL
    # --------------------------------------------------------------------------

    def say(self, text: str, emotion: str = "speaking", blocking: bool = False):
        """Sintetiza texto con Piper TTS y sincroniza la cara animada y los subtitulos sin congelar la GUI."""
        text = text.strip()
        if not text:
            return

        # 1. Silenciar microfono en la Jetson para evitar auto-escucha (anti-echo).
        # Contador: con frases encoladas, el unmute llega recien al terminar la ultima.
        with self._mute_lock:
            self._mute_refs += 1
        if hasattr(self, "pub_voice_mute"):
            self.pub_voice_mute.publish(Bool(data=True))

        self.last_spoken_text = text
        self.current_banner_text = text
        if getattr(self, "guest_screen", None) is not None:
            self.guest_screen.question = text

        # Publicar texto a /hri/say_something para que la cara muestre el subtitulo
        msg_str = String()
        msg_str.data = text
        self.pub_say.publish(msg_str)

        # Configurar emocion
        if emotion:
            self.set_robot_emotion(emotion)

        done_event = threading.Event()

        def _on_done():
            if done_event.is_set():
                return
            done_event.set()
            with self._mute_lock:
                self._mute_refs = max(0, self._mute_refs - 1)
                last = self._mute_refs == 0
            if last and rclpy.ok() and hasattr(self, "pub_voice_mute"):
                try:
                    self.pub_voice_mute.publish(Bool(data=False))
                except Exception:
                    pass

        # Locución con Piper TTS
        try:
            self.speaker.speak(text, blocking=False, on_done=_on_done)
            if blocking:
                # Bombea la GUI continuamente para que el video de la camara nunca se freeze mientras habla
                while not done_event.is_set() and rclpy.ok() and not self.request_shutdown:
                    self.pump_gui(20)
        except Exception as e:
            self.get_logger().warn(f"Error en say(): {e}")
            _on_done()

    def set_robot_emotion(self, emotion: str):
        """Envia estado emocional a la cara interactiva en pantalla."""
        if not rclpy.ok():
            return
        try:
            msg = String()
            msg.data = emotion
            self.pub_emotion.publish(msg)
        except Exception:
            pass

    def trigger_mic_listening(self):
        """Solicita al servicio de Whisper en la Jetson comenzar a escuchar."""
        if not rclpy.ok():
            return
        # Nunca escuchar con el micro silenciado: re-enviar unmute (por si se perdio uno)
        if self._mute_refs == 0:
            self.pub_voice_mute.publish(Bool(data=False))
        try:
            if not self.cli_mic_trigger.service_is_ready():
                self.cli_mic_trigger.wait_for_service(timeout_sec=0.5)
            if self.cli_mic_trigger.service_is_ready():
                self.cli_mic_trigger.call_async(Trigger.Request())
            else:
                # Sin servicio igual se escucha: whisper_node detecta la voz por VAD
                self.get_logger().warn("/voice/listen_trigger no disponible: escucha solo por deteccion de voz",
                                       throttle_duration_sec=10.0)
        except Exception as e:
            self.get_logger().warn(f"Error disparando escucha: {e}", throttle_duration_sec=10.0)

    def start_listening(self, prompt: Optional[str] = None, beep: bool = True):
        """Beep (antes de abrir el micro: no debe grabarse) y pedido de escucha con id."""
        if self._mute_refs == 0:
            self.pub_voice_mute.publish(Bool(data=False))  # por si se perdio un unmute
        if beep:
            self.speaker.play_beep(blocking=True)
        self.set_robot_emotion("listening")
        self.listen_id = self.voice.request(prompt=prompt or self.vocab.whisper_prompt())
        self.state_start_time = time.time()

    def _ask_again(self, text: str):
        self.say(text, emotion="speaking", blocking=True)
        self.start_listening()

    def _voice_problem_hint(self) -> str:
        # El robot no tiene pantalla tactil al frente: no se ofrece tocar la pantalla por voz.
        return ""

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.robot_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def _pt_connected_cb(self, msg):
        self.pt_connected = bool(msg.data)

    def _pt_joint_cb(self, msg: JointState):
        try:
            names = list(msg.name)
            pan = math.degrees(msg.position[names.index("pan_joint")])
            tilt = math.degrees(msg.position[names.index("tilt_joint")])
        except (ValueError, IndexError):
            return
        self.pt_meas = (pan, tilt, time.time())

    def _wait_head_at(self, pan: float, tilt: float, tol_deg: float = 3.0, timeout_s: float = 2.5):
        """Espera que la posicion MEDIDA llegue al objetivo. Devuelve (ok, error_max_grados)."""
        t_end = time.time() + timeout_s
        err = None
        while time.time() < t_end:
            m = self.pt_meas
            if m is not None and time.time() - m[2] < 1.0:
                err = max(abs(m[0] - pan), abs(m[1] - tilt))
                if err <= tol_deg:
                    return True, err
            self.sleep_gui(0.05)
        return False, err

    def pan_tilt_self_test(self) -> Dict:
        """Revision del pan-tilt al arrancar: conexion, feedback, torque y una prueba corta de movimiento
        verificada con la posicion MEDIDA (incluye el sentido de giro). Mueve la cabeza unos grados."""
        checks = []

        def add(name, ok, detail=""):
            checks.append((name, bool(ok), detail))
            line = f"{'✔' if ok else '✘'} [Pan-tilt] {name}{': ' + detail if detail else ''}"
            # rclpy no permite cambiar la severidad en un mismo punto de llamada: una llamada por severidad
            if ok:
                self.get_logger().info(line)
            else:
                self.get_logger().error(line)

        t_end = time.time() + 3.0
        while self.pt_connected is None and time.time() < t_end:
            self.sleep_gui(0.05)
        add("conectado (/pan_tilt/connected)", self.pt_connected is True,
            "sin mensaje" if self.pt_connected is None else ("" if self.pt_connected else "servos desconectados (U2D2/12 V)"))
        fresh = self.pt_meas is not None and time.time() - self.pt_meas[2] < 1.0
        add("realimentacion (/pan_tilt/joint_states)", fresh, "" if fresh else "no llegan posiciones medidas")
        torque_ok, msg = False, "servicio /pan_tilt/set_torque no disponible"
        if self.cli_pantilt_torque.wait_for_service(timeout_sec=2.0):
            fut = self.cli_pantilt_torque.call_async(SetBool.Request(data=True))
            t_end = time.time() + 3.0
            while not fut.done() and time.time() < t_end:
                self.sleep_gui(0.05)
            if fut.done() and fut.result() is not None:
                torque_ok, msg = fut.result().success, fut.result().message
                self._pantilt_torque_requested = torque_ok
            else:
                msg = "sin respuesta del servicio"
        add("torque", torque_ok, msg if not torque_ok else "")
        moves_ok, worst = False, None
        if self.pt_connected and fresh and torque_ok:
            moves_ok, worst = True, 0.0
            for pan, tilt in ((0.0, 0.0), (10.0, 0.0), (-10.0, 0.0), (0.0, 8.0), (0.0, -8.0), (0.0, 0.0)):
                self.move_head_smooth(pan, tilt, speed_dps=40.0)
                ok, err = self._wait_head_at(pan, tilt)
                worst = max(worst, err if err is not None else 99.0)
                if not ok:
                    moves_ok = False
                    add(f"movimiento a pan {pan:+.0f} / tilt {tilt:+.0f}", False,
                        f"medido lejos del objetivo (error {err if err is not None else float('nan'):.1f} grados)")
                    break
            if moves_ok:
                add("movimiento y sentido de giro", True, f"error maximo {worst:.1f} grados")
        else:
            add("movimiento", False, "omitido (falla conexion/feedback/torque)")
        result = dict(ok=all(c[1] for c in checks), checks=checks)
        self.pt_selftest = result
        if result["ok"]:
            self.gentle_tracking = self._gentle_requested   # por si una revision anterior lo habia apagado
        if not result["ok"]:
            self.gentle_tracking = False   # sin cabeza confiable no se sigue la cara
            self.get_logger().error("Pan-tilt con fallas: seguimiento de rostro DESACTIVADO; la camara queda donde este")
        return result

    def _ready_banner(self) -> str:
        """Estado en vivo mientras se espera inicio: puerta, voz o ESPACIO."""
        head = "OK" if (self.pt_selftest or {}).get("ok") and getattr(self, "pt_connected", False) else "FALLA (P revisa)"
        mic = "OK" if self.voice.online() else "SIN RESPUESTA"
        nav = "simulada" if self.sim_nav else "OK"
        door_st = self.door_detector.state if hasattr(self, "door_detector") else "?"
        return f"LISTO: Puerta ({door_st}) | Voz ({mic}) | ESPACIO | Nav {nav} | Pan-tilt {head}"

    def _ensure_pantilt_torque(self):
        if self._pantilt_torque_requested or not self.cli_pantilt_torque.service_is_ready():
            return
        self._pantilt_torque_requested = True
        self.cli_pantilt_torque.call_async(SetBool.Request(data=True))
        self.get_logger().info("Torque del pan-tilt solicitado (/pan_tilt/set_torque)")

    def _head_pan_to_seat(self, seat_key: str) -> Optional[float]:
        """Angulo de pan (grados) para mirar un asiento del mapa semantico, o None."""
        seat = self.seats.get(seat_key) if self.seats else None
        if not seat or self.robot_pose is None or "x" not in seat or "y" not in seat:
            return None
        rx, ry, ryaw = self.robot_pose
        bearing = math.atan2(float(seat["y"]) - ry, float(seat["x"]) - rx) - ryaw
        bearing = math.atan2(math.sin(bearing), math.cos(bearing))
        return self.head_pan_sign * math.degrees(bearing)

    def _look_at_guest(self, guest_idx: int):
        guest = self.registered_guests[guest_idx]
        pan = self._head_pan_to_seat(guest.get("seat", ""))
        if pan is None:
            pan = self.fallback_guest_pan_deg[min(guest_idx, len(self.fallback_guest_pan_deg) - 1)]
        self.move_head_to(pan, -5.0)
        self.sleep_gui(0.8)

    def _introduce_guests_to_each_other(self):
        """Reglamento 2026.2 §5.1: mirando a un invitado, decir nombre y bebida del OTRO (para ambos)."""
        if len(self.registered_guests) < 2:
            return
        g1, g2 = self.registered_guests[0], self.registered_guests[1]
        for listener_idx, other in ((1, g1), (0, g2)):
            listener = self.registered_guests[listener_idx]
            self._look_at_guest(listener_idx)
            self.say(f"{listener['name']}, this is {other['name']}. "
                     f"{other['name']}'s favorite drink is {other['drink']}.",
                     emotion="speaking", blocking=True)

    # Barrido de la sala (ajustable): mas angulos, dos inclinaciones y movimiento lento para tener contexto.
    SCAN_PANS_DEG = (-60.0, -30.0, 0.0, 30.0, 60.0)   # move_head_to limita el pan a +-65
    SCAN_TILTS_DEG = (-12.0, -28.0)     # -12: personas paradas; -28: personas SENTADAS
    SCAN_DWELL_S = 0.8                  # espera en cada vista para que lleguen detecciones nuevas
    SEAT_TILT_DEG = -22.0               # mirar a los asientos mas abajo
    HEAD_SCAN_SPEED_DPS = 40.0          # velocidad de la cabeza al barrer (antes saltaba de golpe); ~15 s en total

    def move_head_smooth(self, pan_deg: float, tilt_deg: float, speed_dps: float = None):
        """Lleva la cabeza a (pan, tilt) a velocidad limitada, interpolando el objetivo (bloqueante)."""
        speed = float(speed_dps or self.HEAD_SCAN_SPEED_DPS)
        goal = (float(np.clip(pan_deg, -90.0, 90.0)), float(np.clip(tilt_deg, -35.0, 35.0)))
        for p, t in scene.head_steps((self.target_pan_deg, self.target_tilt_deg), goal, speed, rate_hz=20.0):
            self.move_head_to(p, t)
            self.sleep_gui(0.05)

    def move_head_to(self, pan_deg: float, tilt_deg: float):
        """Envía comandos angulares al cabezal pan-tilt."""
        self._ensure_pantilt_torque()
        self.target_pan_deg = float(np.clip(pan_deg, -65.0, 65.0))
        self.target_tilt_deg = float(np.clip(tilt_deg, -35.0, 35.0))
        if not rclpy.ok():
            return
        try:
            msg = Vector3()
            msg.x = self.target_pan_deg
            msg.y = self.target_tilt_deg
            msg.z = 0.0
            self.pub_pantilt.publish(msg)
        except Exception:
            pass

    def move_arm_to_named_pose(self, pose_name: str, duration_s: float = 2.0):
        """Mueve el brazo AgileX Piper a una postura segura precalibrada."""
        if pose_name not in ARM_GESTURE_POSES:
            self.get_logger().warn(f"Pose de brazo desconocida: {pose_name}")
            return
        # El brazo NO se manda a poses fijas salvo que se pida (--arm-gestures) y haya
        # realimentacion real: las poses grabadas no corresponden a como esta montado hoy.
        if not self.arm_gestures:
            self.get_logger().info(f"[Brazo] gesto '{pose_name}' omitido (brazo pasivo; usar --arm-gestures)",
                                   throttle_duration_sec=30.0)
            return
        if not (self.arm_feedback_ok and self.arm_enabled):
            self.get_logger().warn(f"[Brazo] gesto '{pose_name}' omitido: brazo sin realimentacion o deshabilitado",
                                   throttle_duration_sec=10.0)
            return

        target_deg = ARM_GESTURE_POSES[pose_name]
        self.current_arm_pose_name = pose_name

        if not rclpy.ok():
            return
        try:
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
        except Exception:
            pass

    def _publish_joint_states(self):
        """Publica torso_lift_joint en /torso/joint_states y pose comandada del brazo en /arm/joint_states_cmd."""
        try:
            now = self.get_clock().now().to_msg()

            # 1. Torso lift joint -> /torso/joint_states (un solo dueño por joint)
            js_torso = JointState()
            js_torso.header.stamp = now
            js_torso.name = ["torso_lift_joint"]
            js_torso.position = [0.0]
            js_torso.velocity = [0.0]
            js_torso.effort = [0.0]
            if hasattr(self, 'pub_torso_joint_states') and self.pub_torso_joint_states:
                self.pub_torso_joint_states.publish(js_torso)
                if getattr(self, 'pub_joint_states_torso', None):
                    self.pub_joint_states_torso.publish(js_torso)

            # 2. Brazo (pose de gesto comandada) -> SOLO a /arm/joint_states_cmd (nunca /joint_states)
            js_arm = JointState()
            js_arm.header.stamp = now
            js_arm.name = [f"joint{i}" for i in range(1, 7)]
            arm_rad = [math.radians(d) for d in self.arm_joints_deg]
            js_arm.position = arm_rad
            js_arm.velocity = [0.0] * len(arm_rad)
            js_arm.effort = [0.0] * len(arm_rad)
            if hasattr(self, 'pub_arm_joint_states_cmd') and self.pub_arm_joint_states_cmd:
                self.pub_arm_joint_states_cmd.publish(js_arm)
        except Exception:
            pass

    # --------------------------------------------------------------------------
    # NAVEGACION AUTONOMA
    # --------------------------------------------------------------------------

    def navigate_to_waypoint(self, wp_name: str) -> bool:
        """Navega de forma autónoma al waypoint objetivo bombeando activamente la GUI."""
        self.get_logger().info(f"🎯 Iniciando navegacion hacia waypoint '{wp_name}'...")
        self.current_banner_text = f"Navegando hacia '{wp_name}'..."
        self.set_robot_emotion("neutral")

        # Postura de transporte seguro del brazo
        self.move_arm_to_named_pose("cube_stow", duration_s=1.0)
        self.move_head_to(0.0, -5.0)

        if self.sim_nav or self.navigator is None:
            self.get_logger().info(f"[SIM NAV]: Simulando viaje a '{wp_name}' (3.0s)...")
            self.sleep_gui(3.0)
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

        while not self.navigator.isTaskComplete() and rclpy.ok() and not self.request_shutdown:
            self.pump_gui(20)

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
                self.frame_count += 1
        except Exception as e:
            self.get_logger().warn(f"Error procesando frame de camara: {e}")

    def _guest_input_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(d, dict) and d.get("type") in ("name", "drink", "answer"):
            self.guest_inputs.put(d)
            self.get_logger().info(f"🖐 [Pantalla]: {d['type']} = {d['value']}")

    def _arm_status_cb(self, msg: String):
        """Posicion real del brazo: con realimentacion, la visualizacion sigue al brazo (aunque se mueva a mano)."""
        try:
            st = json.loads(msg.data)
        except Exception:
            return
        self.arm_feedback_ok = bool(st.get("feedback_ok", False))
        self.arm_enabled = bool(st.get("enabled", False))
        joints = st.get("joints_deg", [])
        if self.arm_feedback_ok and len(joints) >= 6:
            self.arm_joints_deg = [float(v) for v in joints[:6]]
            self.current_arm_pose_name = "medida"

    def _depth_callback(self, msg: Image):
        try:
            if msg.encoding in ("16UC1", "mono16"):
                d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                d = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
            else:
                return
            with self.depth_lock:
                self.latest_depth_m = d
        except Exception:
            pass

    def _caminfo_callback(self, msg: CameraInfo):
        self.cam_fx_cx = (float(msg.k[0]), float(msg.k[2]))

    def _fx(self, img_w: int) -> float:
        """Focal en pixeles (camera_info o FOV horizontal de la D435 color, ~69 grados)."""
        if self.cam_fx_cx:
            return self.cam_fx_cx[0]
        return img_w / (2.0 * math.tan(math.radians(34.5)))

    def _load_furniture_model(self) -> bool:
        """Carga YOLOv8n y hace la primera prediccion (~4 s sin refrescar la ventana): se hace en el pre-vuelo,
        no en medio del barrido."""
        if self.furniture_model is None:
            try:
                from ultralytics import YOLO
                self.furniture_model = YOLO(os.path.join(self.models_dir, "yolov8n.pt"))
                self.furniture_model.predict(np.zeros((480, 640, 3), np.uint8), conf=0.35, verbose=False, device="cpu")
            except Exception as e:
                self.get_logger().warn(f"Detector de muebles no disponible: {e}", throttle_duration_sec=30.0)
                self.furniture_model = False
        return bool(self.furniture_model)

    def _detect_furniture(self, frame: np.ndarray) -> List[Tuple[str, List[float], float]]:
        """Sillas, sofas y bancos con YOLOv8n (COCO)."""
        if not self._load_furniture_model():
            return []
        names = {13: "bench", 56: "chair", 57: "couch"}
        out = []
        res = self.furniture_model.predict(frame, classes=list(names), conf=0.35, verbose=False, device="cpu")[0]
        for b in res.boxes:
            out.append((names[int(b.cls)], [float(v) for v in b.xyxy[0].tolist()], float(b.conf)))
        return out

    def analyze_scene(self) -> None:
        """Barre la sala con el pan-tilt y arma la lista de personas/muebles vistos (self.scene_objects)."""
        self.scanning = True
        objs: List[scene.SeenObject] = []
        try:
            for pan, tilt in scene.scan_plan(self.SCAN_PANS_DEG, self.SCAN_TILTS_DEG):
                self.move_head_smooth(pan, tilt)
                self.sleep_gui(self.SCAN_DWELL_S)  # dejar que lleguen detecciones nuevas en este angulo
                with self.frame_lock:
                    frame = self.latest_frame.copy() if self.latest_frame is not None else None
                if frame is None:
                    continue
                with self.face_lock:
                    poses = list(self.current_poses or [])
                    faces = list(self.current_face_dets or [])
                img_w = frame.shape[1]
                fx_px, cx_px = self._fx(img_w), (self.cam_fx_cx[1] if self.cam_fx_cx else img_w / 2.0)
                person_boxes = [list(map(float, d["bbox"][:4])) for d in poses]
                for d, box in zip(poses, person_boxes):
                    bearing, width = scene.bbox_bearing_width(box, pan, fx_px, cx_px, self.head_pan_sign)
                    name = None
                    for f in faces:  # cara reconocida dentro de la caja de la persona
                        fx0, fy0, fw, fh = f["bbox"]
                        if f.get("person_id") and box[0] <= fx0 + fw / 2 <= box[2] and box[1] <= fy0 + fh / 2 <= box[3]:
                            name = f["name"]
                    objs.append(scene.SeenObject("person", bearing, width,
                                                 seated=str(d.get("posture", "")).upper().startswith("SENTAD"), name=name))
                for kind, box, _ in self._detect_furniture(frame):
                    bearing, width = scene.bbox_bearing_width(box, pan, fx_px, cx_px, self.head_pan_sign)
                    objs.append(scene.SeenObject(kind, bearing, width, occupants=scene.occupants_in_image(box, person_boxes)))
        finally:
            self.scanning = False
        self.scene_objects = scene.merge_views(objs)
        # El anfitrion no tiene cara registrada: con el primer invitado, la unica persona sin nombre es el host
        # (preferir personas SENTADAS: el invitado que acompana al robot suele estar parado y, si su cara
        # no se reconocio en el barrido, antes se lo presentaba como "Charlie").
        unnamed = [o for o in self.scene_objects if o.kind == "person" and not o.name]
        seated = [o for o in unnamed if o.seated]
        if self.guest_index == 1 and len(seated) == 1:
            seated[0].name = "Charlie"
        elif self.guest_index == 1 and len(unnamed) == 1 and not any(o.seated for o in self.scene_objects if o.kind == "person"):
            unnamed[0].name = "Charlie"
            self.get_logger().warn("Anfitrion asignado a la unica persona sin nombre (no vi a nadie sentado)")
        summary = [(o.kind, round(o.bearing_deg), o.name or "", o.occupants) for o in self.scene_objects]
        self.get_logger().info(f"🔎 Escena: {summary}")

    def scan_seats(self) -> Tuple[Dict[str, int], bool]:
        """Mira cada asiento del mapa semantico y cuenta personas en el (ocupacion, uso_profundidad)."""
        persons: List[Tuple[float, float]] = []
        fallback: Dict[str, int] = {}
        used_depth = False
        for key in self.seats:
            pan = self._head_pan_to_seat(key)
            if pan is not None:
                self.move_head_smooth(pan, self.SEAT_TILT_DEG)
            self.sleep_gui(1.6)  # dejar que el detector de poses procese varios frames nuevos
            with self.face_lock:
                poses = list(self.current_poses or [])
            with self.depth_lock:
                depth = self.latest_depth_m
            img_w = self.latest_frame.shape[1] if self.latest_frame is not None else 640
            if depth is not None and self.cam_fx_cx and self.robot_pose is not None and pan is not None:
                fx, cx = self.cam_fx_cx
                for det in poses:
                    z = seatocc.depth_at_bbox(depth, det["bbox"])
                    if z is not None:
                        u = (det["bbox"][0] + det["bbox"][2]) / 2.0
                        persons.append(seatocc.person_map_position(
                            u, z, fx, cx, self.robot_pose, math.radians(pan), self.head_pan_sign))
                        used_depth = True
            fallback[key] = seatocc.image_fallback_occupied(
                [d["bbox"] for d in poses], [d.get("posture", "") for d in poses], img_w)
        if used_depth:
            occ = seatocc.seat_occupancy(self.seats, seatocc.merge_nearby(persons))
        else:
            occ = fallback
        self.get_logger().info(f"🪑 Ocupacion de asientos ({'profundidad+mapa' if used_depth else 'imagen'}): {occ}")
        return occ, used_depth

    def voice_callback(self, msg: String):
        text = msg.data.strip()
        if text:
            self.voice_text = text
            self.user_speech_buffer = text
            self.get_logger().info(f"🎙️ [Whisper ASR]: \"{text}\"")

    def voice_status_callback(self, msg: String):
        self.voice_status = msg.data.strip().upper()

    def _voice_service_ready(self) -> bool:
        try:
            return bool(self.voice._new_protocol_available() or self.cli_mic_trigger.service_is_ready())
        except Exception:
            return False

    def _scan_callback(self, msg: LaserScan):
        if not (self.wait_start and not self.start_confirmed):
            return
        d = window_distance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max, self.door_detector.cfg
        )
        state = self.door_detector.update(d)
        if state == "OPEN" and not self.start_confirmed:
            if self.preflight_complete:
                self.get_logger().info("🚪 [Inicio]: Puerta abierta detectada por láser (/scan). Arrancando...")
                self.start_confirmed = True

    # --------------------------------------------------------------------------
    # BUCLE DE LA MAQUINA DE ESTADOS HRI
    # --------------------------------------------------------------------------

    def _prepare_door_arrival(self):
        """Start an optional bounded wait, separately for each arriving guest."""
        if not getattr(self, "wait_doorbell", False):
            self.state = self.STATE_NAV_TO_DOOR
            return
        with self.doorbell_lock:
            self.doorbell_detector.reset()
            self.doorbell_detected_at = None
            self.doorbell_started_at = time.monotonic()
            self.state = self.STATE_WAIT_DOORBELL
            self.doorbell_detector.arm()
        self.current_banner_text = "Esperando timbre (con tiempo limite)..."

    def _doorbell_audio_callback(self, msg):
        """Process small live PCM blocks only during the armed waiting window."""
        with self.doorbell_lock:
            now = time.monotonic()
            status = doorbell_wait_status(self.wait_doorbell, self.doorbell_started_at,
                                         now, self.doorbell_timeout_s,
                                         self.doorbell_detected_at)
            if (self.state != self.STATE_WAIT_DOORBELL or status != "waiting"
                    or not 0 < len(msg.data) <= 1600):
                return
            if self.doorbell_detector.process(msg.data).detected:
                self.doorbell_detected_at = now
                self.doorbell_detector.disarm()

    def _step_doorbell_wait(self):
        with self.doorbell_lock:
            status = doorbell_wait_status(True, self.doorbell_started_at,
                                         time.monotonic(), self.doorbell_timeout_s,
                                         self.doorbell_detected_at)
            if status == "waiting":
                return
            self.doorbell_detector.disarm()
            self.state = self.STATE_NAV_TO_DOOR
        if status == "timeout":
            self.get_logger().warn("Timbre: tiempo agotado; continuo hacia la puerta sin deteccion.")
        else:
            self.get_logger().info("Timbre detectado: voy a recibir al invitado.")
        self.state_start_time = time.time()

    def step(self):
        """Ejecuta un ciclo de la maquina de estados de la prueba Receptionist."""
        now = time.time()

        with self.frame_lock:
            frame = self.latest_frame.copy() if self.latest_frame is not None else None

        # 1. Detección Facial (la hace _face_worker_loop en otro hilo) y seguimiento de cabezal
        if self.clear_memory_requested:
            self.clear_memory_requested = False
            self.clear_temporary_memory()
        with self.face_lock:
            face_results = list(self.current_face_dets)
            dets_stamp = self.face_dets_stamp
        if frame is not None:
            # Seguimiento suave del rostro con Pan-Tilt: un paso por deteccion nueva
            # (antes se aplicaba el mismo error en cada ciclo de ~50 Hz y sobreoscilaba)
            new_dets = dets_stamp != self.last_tracked_stamp
            self.last_tracked_stamp = dets_stamp
            if (self.pan_tilt_tracking and not self.scanning and self.state in self.TRACKING_STATES
                    and face_results and new_dets):
                largest_face = max(face_results, key=lambda f: f["bbox"][2] * f["bbox"][3])
                bx, by, bw, bh = largest_face["bbox"]
                img_h, img_w = frame.shape[:2]
                fx_px = self._fx(img_w)
                dpan, dtilt = scene.face_tracking_step(bx + bw / 2.0, by + bh / 2.0, img_w, img_h, fx_px, fx_px,
                                                      gain=0.3, deadband_deg=4.0, max_step_deg=3.0)
                if dpan or dtilt:
                    self.move_head_to(self.target_pan_deg + self.head_pan_sign * dpan,
                                      self.target_tilt_deg + dtilt)
            elif (self.gentle_tracking and not self.pan_tilt_tracking and not self.scanning
                  and self.state in self.REGISTRATION_STATES and face_results and new_dets):
                largest_face = max(face_results, key=lambda f: f["bbox"][2] * f["bbox"][3])
                bx, by, bw, bh = largest_face["bbox"]
                img_h, img_w = frame.shape[:2]
                fx_px = self._fx(img_w)
                # el pan del tracker va en la convencion de la imagen (izquierda +): se corrige con head_pan_sign
                tgt = self.gentle_tracker.update(bx + bw / 2.0, by + bh / 2.0, img_w, img_h, fx_px, fx_px,
                                                 self.head_pan_sign * self.target_pan_deg, self.target_tilt_deg,
                                                 time.time())
                if tgt is not None:
                    self.move_head_to(self.head_pan_sign * tgt[0], tgt[1])
            elif self.state not in self.REGISTRATION_STATES:
                self.gentle_tracker.reset()

        # 2. Transición y Lógica de Estados
        if self.state == self.STATE_INIT:
            # 1. Espera activa de cámara
            if self.latest_frame is None:
                if now - self.last_camera_wait_log > 2.5:
                    self.last_camera_wait_log = now
                    self.get_logger().info(
                        f"⏳ [Pre-vuelo]: Esperando transmisión de cámara en '{self.camera_topic_name}'..."
                    )
                self.current_banner_text = "PRE-VUELO: Esperando conexion de camara RealSense D435..."
                return

            # 2. Comprobación y sincronización de Nav2
            if not self.nav2_verified:
                if not self.sim_nav:
                    self.get_logger().info("🔍 [Pre-vuelo]: Verificando servidor de navegacion Nav2...")
                    nav2_ok = self._check_nav2_ready(timeout_sec=2.0)
                    if not nav2_ok:
                        self.get_logger().warn(
                            "⚠ [Pre-vuelo]: Servidor Nav2 (navigate_to_pose) no activo o no lanzado. "
                            "Cambiando automaticamente a Modo Navegacion Simulada para no bloquear la prueba."
                        )
                        self.sim_nav = True
                    else:
                        self.get_logger().info("✔ [Pre-vuelo]: Servidor Nav2 (navigate_to_pose) listo y respondiendo.")
                self.nav2_verified = True

            if self.furniture_model is None:
                self.current_banner_text = "Cargando el detector de muebles..."
                self.pump_gui(1)
                self._load_furniture_model()
            if self.pt_selftest is None:
                self.current_banner_text = "Revisando el pan-tilt de la camara..."
                self.pan_tilt_self_test()
                if not self.pt_selftest["ok"]:
                    self.say("Warning: my head camera motors are not working correctly. "
                             "I will keep the camera fixed.", emotion="thinking", blocking=True)
            frame_shape = self.latest_frame.shape if self.latest_frame is not None else (0, 0, 0)
            if self.preflight_complete and self.wait_start and not self.start_confirmed:
                if self._pt_retest_requested:
                    self._pt_retest_requested = False
                    self.pan_tilt_self_test()

                # A. Arranque por apertura de puerta (/scan via door_detector)
                if self.door_detector.state == "OPEN":
                    self.get_logger().info("🚪 [Inicio]: Puerta abierta detectada por láser (/scan). Arrancando...")
                    self.start_confirmed = True

                # B. Arranque por palabra hablada (transcripcion directa o manual)
                if not self.start_confirmed and self.voice_text:
                    v_heard = self.voice_text
                    self.voice_text = None
                    if is_start_speech(v_heard):
                        self.get_logger().info(f"🎙️ [Inicio]: Palabra de inicio detectada: '{v_heard}'. Arrancando...")
                        self.start_confirmed = True

                # C. Arranque por cliente de voz (VoiceClient poll)
                if not self.start_confirmed:
                    if self.listen_id is not None:
                        res = self.voice.poll(self.listen_id)
                        if res is not None:
                            self.listen_id = None
                            if res.ok and is_start_speech(res.text):
                                self.get_logger().info(f"🎙️ [Inicio]: Palabra de inicio detectada por VoiceClient: '{res.text}'. Arrancando...")
                                self.start_confirmed = True
                    # Reabrir escucha si termino sin confirmar y el microfono esta disponible
                    if self.listen_id is None and not self.start_confirmed:
                        if now - self._last_voice_listen_attempt > 1.5:
                            self._last_voice_listen_attempt = now
                            if self.voice.online() or self._voice_service_ready():
                                try:
                                    self.listen_id = self.voice.request(prompt="start, robot start, begin, go")
                                except Exception:
                                    pass

                if not self.start_confirmed:
                    self.current_banner_text = self._ready_banner()
                    return

            if self.preflight_complete:
                if self.listen_id is not None:
                    self.voice.cancel(self.listen_id)
                    self.listen_id = None
                self.voice_text = None
                self.say("Starting Receptionist Challenge.", emotion="happy", blocking=True)
                self._prepare_door_arrival()
                self.state_start_time = time.time()
                return
            print("\n" + "=" * 70)
            print("🚀 [PRE-VUELO RECEPTIONIST COMPLETADO - SUBSISTEMAS LISTOS]")
            print("=" * 70)
            print(f"  [✔] Cámara RealSense D435: Stream OK ({frame_shape[1]}x{frame_shape[0]} px)")
            print(f"  [✔] Memoria & Almacen    : {self.session_file}")
            print(f"  [✔] Reconocimiento Facial: SFace (128-d) + YuNet inicializados")
            print(f"  [✔] Navegación           : {'SIMULADA (Segura sin base)' if self.sim_nav else 'Nav2 REAL'}")
            print(f"  [✔] Audio & Síntesis     : Piper TTS + Whisper ASR")
            print(f"  [{'✔' if self.furniture_model else '✘'}] Detector de muebles  : YOLOv8n (COCO)")
            print(f"  [✔] Waypoints Semánticos : Entrada '{self.door_wp}' | Sala '{self.living_room_wp}'")
            for name, ok, detail in (self.pt_selftest or {}).get("checks", []):
                print(f"  [{'✔' if ok else '✘'}] Pan-tilt: {name}{' - ' + detail if detail else ''}")
            print("=" * 70 + "\n")

            self.preflight_complete = True
            if self.wait_start:
                print("👉 Todo cargado. Listo para arrancar: abre la puerta, di 'start' o presiona [ESPACIO]. "
                      "[P] vuelve a revisar el pan-tilt, [Q] sale.\n")
                self.say("All systems online. Ready to start.", emotion="happy", blocking=True)
                self.current_banner_text = self._ready_banner()
                if self.voice.online() or self._voice_service_ready():
                    try:
                        self.listen_id = self.voice.request(prompt="start, robot start, begin, go")
                        self._last_voice_listen_attempt = time.time()
                    except Exception:
                        pass
                return
            self.say("All systems online. Camera and perception ready. Starting Receptionist Challenge.", emotion="happy", blocking=True)
            self._prepare_door_arrival()
            self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_WAIT_DOORBELL:
            self._step_doorbell_wait()

        elif self.state == self.STATE_NAV_TO_DOOR:
            self.say("Navigating to the entrance door.", emotion="neutral", blocking=True)
            self.navigate_to_waypoint(self.door_wp)
            # Limpieza rigurosa de audio al llegar a la puerta para no registrar ruido o eco de navegacion
            self.voice_text = None
            self.user_speech_buffer = ""
            self.door_detection_start = None
            self.last_person_seen_time = None
            # Centrar la camara al llegar a la puerta (facilita el seguimiento del invitado): movimiento suave,
            # reenvio por si el primer comando llego con el torque apagado (el nodo lo descarta), y tracker limpio.
            self._ensure_pantilt_torque()
            self.move_head_smooth(0.0, 0.0, speed_dps=40.0)
            self.sleep_gui(0.3)
            self.move_head_to(0.0, 0.0)
            self.gentle_tracker.reset()
            self.set_robot_emotion("neutral")
            self.current_banner_text = f"Esperando invitado {self.guest_index} en la puerta..."
            self.state = self.STATE_WAIT_DOOR
            self.state_start_time = time.time()

        elif self.state == self.STATE_WAIT_DOOR:
            # 1. Avance manual forzado vía teclado [ESPACIO / ENTER]
            if self.force_advance:
                self.force_advance = False
                self.get_logger().info("⚡ [Manual]: Forzando avance de puerta -> Saludo.")
                self.state = self.STATE_GREET_AND_PROMPT
                self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo
                self.door_detection_start = None
                return

            # 2. Centrado suave del cabezal si no hay seguimiento activo
            if not face_results and (abs(self.target_pan_deg) > 3.0 or abs(self.target_tilt_deg) > 3.0):
                self.move_head_to(0.0, 0.0)

            # 3. Detección Multi-Modal de Persona frente al robot
            person_detected = False
            detect_reason = ""

            # A. Rostro detectado (umbral realista >= 500 px², visible desde 2.5m)
            if face_results:
                largest_face = max(face_results, key=lambda f: f["bbox"][2] * f["bbox"][3])
                fw, fh = largest_face["bbox"][2], largest_face["bbox"][3]
                if fw * fh >= 500:
                    person_detected = True
                    detect_reason = f"Rostro visible ({fw}x{fh} px)"

            # B. Detección de cuerpo humano con YOLOv8-pose (por si la persona no mira directo a la cámara)
            if not person_detected and self.pose_detector is not None and frame is not None:
                try:
                    poses = self.pose_detector.detect_poses(frame)
                    if poses and len(poses) > 0:
                        person_detected = True
                        detect_reason = f"Cuerpo/Silueta detectado (YOLO: {len(poses)} personas)"
                except Exception:
                    pass

            # C. Presencia por voz (SOLO si dice palabras de saludo explícitas)
            if not person_detected and self.voice_text:
                v_clean = self.voice_text.lower().strip()
                door_keywords = ["hello", "hi", "hey", "robot", "here", "door", "open", "i'm here", "im here", "hola", "buenas"]
                if any(k in v_clean for k in door_keywords) or len(v_clean.split()) >= 3:
                    person_detected = True
                    detect_reason = f"Voz de saludo detectada: '{self.voice_text}'"
                # Consumir para evitar falsos positivos acumulados
                self.voice_text = None

            # 4. Debounce y confirmación de presencia (1.0s continuo de presencia)
            if person_detected:
                self.last_person_seen_time = now
                if self.door_detection_start is None:
                    self.door_detection_start = now
                    self.get_logger().info(f"👀 [Puerta]: {detect_reason}. Confirmando presencia...")
                elif now - self.door_detection_start >= 1.0:
                    self.get_logger().info("✔ [Puerta]: Invitado confirmado. Iniciando saludo y bienvenida.")
                    self.state = self.STATE_GREET_AND_PROMPT
                    self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo
                    self.door_detection_start = None
                    return
            else:
                # Tolerancia de 1.0s para no reiniciar el contador si hay un parpadeo de frame
                if self.last_person_seen_time is None or (now - self.last_person_seen_time > 1.0):
                    self.door_detection_start = None

            # 5. Recordatorio cortés si espera más de 20 segundos sin nadie
            if (now - self.state_start_time > 20.0) and (now - self.last_door_prompt_time > 20.0):
                self.last_door_prompt_time = now
                self.say("Hello! I am waiting for the guest at the door. Please step forward.", emotion="speaking", blocking=False)

        elif self.state == self.STATE_GREET_AND_PROMPT:
            self.current_attributes = {}
            # Saludo y pregunta abierta de nombre y bebida favorita
            # 1) quien es el robot  2) que tiene que decir la persona  3) cuando (despues del beep)
            self.move_head_to(0.0, 0.0)  # mirar al frente (sin seguimiento)
            self.voice_text = None
            self.say("Hello! Welcome to our home. I am Robot, the receptionist of this house.",
                     emotion="speaking", blocking=True)
            self.say("After the beep, please say: My name is, your name, and my favorite drink is, your drink.",
                     emotion="speaking", blocking=True)
            self.voice_text = None
            self.user_speech_buffer = ""
            self.retry_count = 0
            self.voice_failures = 0
            self.confirm_tries = 0
            self.from_screen = False
            self.state = self.STATE_WAIT_NAME_DRINK
            # Tono de turn-taking acustico para ceder el turno de habla al humano + escucha con id
            self.start_listening()

        elif self.state == self.STATE_WAIT_NAME_DRINK:
            # Respuestas por la pantalla tactil (alternativa al microfono): se aceptan siempre
            while not self.guest_inputs.empty():
                d = self.guest_inputs.get()
                if d["type"] == "name":
                    self.current_name = self.vocab.match_name(d["value"]) or d["value"].title()
                elif d["type"] == "drink":
                    self.current_drink = self.vocab.match_drink(d["value"]) or d["value"].title()
                self.from_screen = True
            if self.current_name and self.current_drink and self.from_screen:
                self.voice.cancel(self.listen_id)
                self.listen_id = None
                self.state = self.STATE_CONFIRM_DETAILS   # tocado en pantalla: no hace falta preguntar si/no
                self.state_start_time = time.time()
                return
            res = self.voice.poll(self.listen_id)
            if res is None and self.listen_id is None and self.voice_text:  # [T] sin pedido abierto
                res, self.voice_text = VoiceResult("manual", "ok", text=self.voice_text, source="keyboard"), None
            if res is None:
                if self.listen_id is None and now - self.state_start_time > 2.0:
                    self.start_listening()          # p.ej. tras un [ESPACIO] o un reinicio: volver a escuchar
                return
            self.listen_id = None
            self.voice_text = None
            if res.ok:
                speech = res.text
                self.set_robot_emotion("thinking")
                extracted_name, extracted_drink = extract_name_and_drink(speech, self.vocab)
                self.get_logger().info(
                    f"NLP Parser: \"{speech}\" -> Nombre: '{extracted_name}', Bebida: '{extracted_drink}'"
                )
                self.last_heard_text = speech
                if extracted_name:
                    self.current_name = extracted_name
                if extracted_drink:
                    self.current_drink = extracted_drink

                if self.current_name and self.current_drink:
                    self.voice_failures = 0
                    # RuleBook: entender nombre y bebida sin preguntas de confirmacion suma puntos. La frase siguiente
                    # es una afirmacion, permitida. ATHOME_HRI_CONFIRM=1 vuelve a preguntar, a costa de esos puntos.
                    self.state = self.STATE_ASK_CONFIRM if self.ask_confirmation else self.STATE_CONFIRM_DETAILS
                    self.state_start_time = time.time()
                elif self.current_name or self.current_drink:
                    self.voice_failures = 0
                    self.missing_field = "drink" if self.current_name else "name"
                    self.state = self.STATE_PROMPT_MISSING_INFO
                    self.state_start_time = time.time()
                else:
                    # Hablo pero no se extrajo nada: pedir aclaracion sin inventar datos
                    self.voice_failures += 1
                    self.retry_count += 1
                    self._ask_again("Sorry, I did not understand. After the beep, please say: "
                                    "My name is, your name, and my favorite drink is, your drink."
                                    + self._voice_problem_hint())
            elif res.status in ("offline", "error"):
                self.voice_failures += 1
                self.retry_count += 1
                self.get_logger().error(f"🎤 Microfono/ASR no disponible ({res.status}: {res.reason}). "
                                        "Revisar Jetson: systemctl status jetson-head; ros2 topic echo /voice/status")
                self.say("I am having trouble with my microphone. One moment please.",
                         emotion="speaking", blocking=True)
                self.sleep_gui(2.0)
                self.start_listening(beep=False)   # si el ASR vuelve, se sigue por voz
            else:
                # no_speech / rejected / timeout: no se escucho una frase; NO avanzar, volver a preguntar
                self.voice_failures += 1
                self.retry_count += 1
                self.get_logger().info(f"⏳ [Audio]: sin respuesta ({res.status} {res.reason}), "
                                       f"intento {self.retry_count}. [T] escribe por teclado.")
                if self.retry_count % 2 == 1:
                    prompt_wait = ("I could not hear you. Please come a little closer and, after the beep, say: "
                                   "My name is, your name, and my favorite drink is, your drink.")
                else:
                    prompt_wait = "After the beep, please tell me your name and your favorite drink."
                self._ask_again(prompt_wait + self._voice_problem_hint())

        elif self.state == self.STATE_PROMPT_MISSING_INFO:
            if self.missing_field == "drink":
                prompt = (f"Nice to meet you, {self.current_name}! After the beep, please say: "
                          f"my favorite drink is, your drink.")
            else:
                prompt = f"I got your drink, {self.current_drink}. After the beep, please say: my name is, your name."
            self.say(prompt, emotion="speaking", blocking=True)
            self.voice_text = None
            self.user_speech_buffer = ""
            self.state = self.STATE_WAIT_NAME_DRINK
            self.start_listening()

        elif self.state == self.STATE_ASK_CONFIRM:
            # Verificar lo entendido: un nombre o bebida mal escuchados cuestan puntos en todo el resto
            self.confirm_tries += 1
            self.say(f"Let me check. Your name is {self.current_name}, and your favorite drink is "
                     f"{self.current_drink}. Is that correct? Please say yes or no after the beep.",
                     emotion="speaking", blocking=True)
            self.state = self.STATE_WAIT_CONFIRM
            self.start_listening(prompt=YES_NO_PROMPT)

        elif self.state == self.STATE_WAIT_CONFIRM:
            answer = None
            while not self.guest_inputs.empty():
                d = self.guest_inputs.get()
                if d["type"] == "answer":
                    answer = parse_yes_no(d["value"])
                elif d["type"] in ("name", "drink"):  # corrigio en la pantalla
                    self.guest_inputs.put(d)
                    self.voice.cancel(self.listen_id)
                    self.listen_id = None
                    self.state = self.STATE_WAIT_NAME_DRINK
                    self.state_start_time = time.time()
                    return
            res = None
            if answer is None:
                res = self.voice.poll(self.listen_id)
                if res is None:
                    return
                if res.ok:
                    answer = parse_yes_no(res.text)
                    self.get_logger().info(f"Confirmacion: \"{res.text}\" -> {answer}")
            self.voice.cancel(self.listen_id)
            self.listen_id = None
            if answer is True or (answer is None and self.confirm_tries >= 2):
                if answer is None:
                    self.get_logger().warn("Sin si/no claro tras 2 intentos: sigo con lo entendido")
                self.state = self.STATE_CONFIRM_DETAILS
                self.state_start_time = time.time()
            elif answer is False:
                # Volver a preguntar ambos datos (mas rapido y claro que preguntar cual estaba mal)
                self.current_name = None
                self.current_drink = None
                self.confirm_tries = 0
                self.say("Sorry about that. Let's try again.", emotion="speaking", blocking=True)
                self._ask_again("After the beep, please say slowly: My name is, your name, and my favorite "
                                "drink is, your drink.")
                self.state = self.STATE_WAIT_NAME_DRINK
            else:
                self.state = self.STATE_ASK_CONFIRM
                self.state_start_time = time.time()

        elif self.state == self.STATE_CONFIRM_DETAILS:
            confirm_msg = f"Thank you, {self.current_name}! I noted that your favorite drink is {self.current_drink}."
            self.say(confirm_msg, emotion="happy", blocking=True)

            learn_msg = "Now please look straight at my eyes for a few seconds so I can memorize your face."
            self.say(learn_msg, emotion="speaking", blocking=True)
            self.set_robot_emotion("thinking")

            self.recording_samples = []
            self.fallback_samples = []  # rostros detectados pero chicos/borrosos: mejor que un vector aleatorio
            self.asked_closer = False
            self.samples_synthetic = False
            self.best_thumbnail = None
            self.best_sharpness = 0.0
            self.last_sample_time = time.time()
            self.state = self.STATE_MEMORIZE_FACE
            self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_MEMORIZE_FACE:
            if self.guest_index == 1:
                with self.face_lock:
                    if (self.visual_attributes_stamp >= self.state_start_time
                            and 0 <= now - self.visual_attributes_stamp <= 1.0):
                        self.current_attributes = prefer_attributes(
                            self.current_attributes, self.visual_attributes)
            # Captura de 10 muestras biométricas con filtro de calidad
            if frame is not None and face_results:
                target = max(face_results, key=lambda f: f["bbox"][2] * f["bbox"][3])
                feat = target.get("feature")
                q_valid = target.get("quality_valid", False)
                q_score = target.get("quality_score", 0.0)

                # Tasa de captura controlada (~7 Hz)
                same_person = True
                if feat is not None and self.recording_samples:
                    same_person = self.recognizer.similarity(feat, self.recording_samples[0]) >= 0.30
                if feat is not None and not q_valid and len(self.fallback_samples) < 10:
                    self.fallback_samples.append(feat)
                if feat is not None and q_valid and same_person and (now - self.last_sample_time >= 0.14):
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
                # Selección dinámica del asiento desde el mapa semántico
                seat_keys = list(self.seats.keys()) if self.seats else []
                if seat_keys:
                    seat_idx = min(self.guest_index - 1, len(seat_keys) - 1)
                    seat_assigned = seat_keys[seat_idx]
                else:
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
                        "synthetic": self.samples_synthetic,
                        "visual_attributes": self.current_attributes,
                    },
                )

                guest_record = {
                    "guest_index": self.guest_index,
                    "person_id": pid,
                    "name": self.current_name,
                    "favorite_drink": self.current_drink,
                    "assigned_seat": seat_assigned,
                    "biometric_samples": len(self.recording_samples),
                    "best_sharpness": round(float(self.best_sharpness), 2),
                    "registered_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "thumbnail_path": self.recognizer.profiles[pid].thumbnail_file if pid in self.recognizer.profiles else "",
                    "visual_attributes": self.current_attributes,
                }
                self.registered_guests.append({
                    "name": self.current_name,
                    "drink": self.current_drink,
                    "seat": seat_assigned,
                    "person_id": pid,
                    "thumbnail": self.best_thumbnail,
                    "visual_attributes": self.current_attributes,
                })
                # Persistencia inmediata en receptionist_session.json
                self._update_session_storage(guest_record)

                print("\n" + "-" * 60)
                print(f"👤 NUEVO INVITADO REGISTRADO EN MEMORIA Y DISCO:")
                print(f"   Nombre        : {self.current_name}")
                print(f"   Bebida Fav.   : {self.current_drink}")
                print(f"   Asiento Asign.: {seat_assigned}")
                print(f"   Muestras Face : {len(self.recording_samples)} descriptores (128-d)")
                print(f"   Archivo Sesion: {self.session_file}")
                print("-" * 60 + "\n")

                self.say(f"Got it! Your face is saved, {self.current_name}.", emotion="happy", blocking=True)
                self.state = self.STATE_NAV_TO_LIVING_ROOM
                self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

            elif elapsed > 3.5 and not self.recording_samples and not self.asked_closer:
                # Nada valido todavia (rostro chico = lejos, o borroso): pedir que se acerque y dar mas tiempo
                self.asked_closer = True
                why = face_results[0].get("quality_reason", "") if face_results else "no face"
                self.get_logger().warn(f"Memorizar rostro: sin muestras validas ({why}); pido que se acerque")
                self.say("Please come a little closer and look at my camera.", emotion="speaking", blocking=True)
                self.state_start_time = time.time() - 2.0

            elif elapsed > 7.0:
                # Timeout de memorización: garantizar almacenamiento incluso con pocas muestras
                seat_keys = list(self.seats.keys()) if self.seats else []
                seat_assigned = seat_keys[min(self.guest_index - 1, len(seat_keys) - 1)] if seat_keys else ("couch" if self.guest_index == 1 else "chair")
                if not self.recording_samples:
                    dummy_f = np.random.randn(1, 128).astype(np.float32)
                    dummy_f /= np.linalg.norm(dummy_f)
                    self.recording_samples.append(dummy_f)
                    self.samples_synthetic = True

                pid, is_new = self.recognizer.enroll_person(
                    name=self.current_name,
                    features=self.recording_samples,
                    thumbnail_img=self.best_thumbnail,
                    attributes={
                        "role": f"guest_{self.guest_index}",
                        "favorite_drink": self.current_drink,
                        "seat": seat_assigned,
                        "guest_index": self.guest_index,
                        "synthetic": self.samples_synthetic,
                        "visual_attributes": self.current_attributes,
                    },
                )
                guest_record = {
                    "guest_index": self.guest_index,
                    "person_id": pid,
                    "name": self.current_name,
                    "favorite_drink": self.current_drink,
                    "assigned_seat": seat_assigned,
                    "biometric_samples": len(self.recording_samples),
                    "best_sharpness": round(float(self.best_sharpness), 2),
                    "registered_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "thumbnail_path": "",
                    "visual_attributes": self.current_attributes,
                }
                self.registered_guests.append({
                    "name": self.current_name,
                    "drink": self.current_drink,
                    "seat": seat_assigned,
                    "person_id": pid,
                    "thumbnail": self.best_thumbnail,
                    "visual_attributes": self.current_attributes,
                })
                self._update_session_storage(guest_record)

                self.say(f"I have enough details, {self.current_name}.", emotion="happy", blocking=True)
                self.state = self.STATE_NAV_TO_LIVING_ROOM
                self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_NAV_TO_LIVING_ROOM:
            follow_msg = f"{self.current_name}, please follow me to the living room."
            self.say(follow_msg, emotion="happy", blocking=True)

            self.navigate_to_waypoint(self.living_room_wp)
            self.state = self.STATE_SCAN_SCENE
            self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_SCAN_SCENE:
            self.say("We have arrived. Let me take a look around.", emotion="thinking", blocking=True)
            self.current_banner_text = "Analizando la sala: personas y asientos..."
            self.analyze_scene()
            host = next((o for o in self.scene_objects if o.kind == "person" and o.name == "Charlie"), None)
            if host is not None:  # mirar al anfitrion para presentarlo (sentado: un poco mas abajo)
                self.move_head_smooth(self.head_pan_sign * host.bearing_deg, -15.0 if host.seated else -5.0)
            self.state = self.STATE_INTRODUCE_GUESTS
            self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_FIND_HOST:
            # Llegada a la sala: escanear el entorno
            self.say("We have arrived at the living room. Let me introduce you.", emotion="happy", blocking=True)
            self.move_head_to(20.0, -5.0)
            self.sleep_gui(1.0)
            self.move_head_to(-20.0, -5.0)
            self.sleep_gui(1.0)
            self.move_head_to(0.0, -5.0)

            self.state = self.STATE_INTRODUCE_GUESTS
            self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

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
            if self.guest_index == 2 and self.registered_guests:
                description = describe_guest_attributes(
                    self.registered_guests[0].get("visual_attributes", {}))
                if description:
                    self.say(description, emotion="speaking", blocking=True)
            self.state = self.STATE_DETECT_AND_OFFER_SEAT
            self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_DETECT_AND_OFFER_SEAT:
            # Inspección de asientos: mirar cada uno, contar personas y ofrecer uno con lugar libre
            seat_keys = list(self.seats.keys()) if self.seats else ["couch", "chair"]
            seat_target = None
            if self.seats:
                self.current_banner_text = "Revisando que asientos estan libres..."
                occ, used_depth = self.scan_seats()
                reserved = {}
                if not used_depth:  # sin ubicar personas en el mapa: contar tambien a quienes ya sentamos
                    for g in self.registered_guests[:-1]:
                        reserved[g.get("seat")] = reserved.get(g.get("seat"), 0) + 1
                seat_target = seatocc.choose_free_seat(self.seats, occ, reserved)
            visual_seat = scene.choose_seat(self.scene_objects)
            self.seat_bearing_deg = None
            if seat_target is not None and self._head_pan_to_seat(seat_target) is not None:
                self.seat_bearing_deg = self.head_pan_sign * self._head_pan_to_seat(seat_target)
            elif visual_seat is not None:
                self.seat_bearing_deg = visual_seat.bearing_deg
                seat_target = seat_target or visual_seat.kind
            if seat_target is None:
                seat_idx = min(self.guest_index - 1, len(seat_keys) - 1)
                seat_target = seat_keys[seat_idx] if seat_keys else "couch"
            # Registrar el asiento real del invitado (antes era un valor provisional por indice)
            if self.registered_guests:
                self.registered_guests[-1]["seat"] = seat_target
                pid = self.registered_guests[-1].get("person_id")
                if pid in self.recognizer.profiles:
                    self.recognizer.profiles[pid].attributes["seat"] = seat_target
                    self.recognizer.save_metadata_only()
            pan = self._head_pan_to_seat(seat_target)
            if pan is not None:
                self.move_head_to(pan, -10.0)  # mirar el asiento mientras lo ofrece

            seat_info = self.seats.get(seat_target, {}) if self.seats else {}
            pose_gesture = seat_info.get("arm_gesture", "point_couch" if seat_target == "couch" else "point_chair")
            seat_label = seat_info.get("name", "couch" if seat_target == "couch" else "chair")

            if self.seat_bearing_deg is not None:
                self.move_head_to(self.head_pan_sign * self.seat_bearing_deg, -10.0)
            if self.scene_objects and (visual_seat is not None or not self.seats):
                offer_msg = scene.describe_scene(self.scene_objects, self.current_name, visual_seat)
            elif self.seat_bearing_deg is not None:
                offer_msg = (f"{self.current_name}, the {seat_label} {scene.direction_words(self.seat_bearing_deg)} "
                             f"is free. Please take a seat there.")
            else:
                offer_msg = f"{self.current_name}, please have a seat on the {seat_label}."
            self.say(offer_msg, emotion="happy", blocking=False)

            # Ejecutar gesto físico de señalamiento con el brazo
            self.move_arm_to_named_pose(pose_gesture, duration_s=2.0)
            self.sleep_gui(3.2)
            self.move_arm_to_named_pose("cube_stow", duration_s=1.8)
            self.seated_hits = 0
            self.seat_reminded = False
            self.state = self.STATE_WAIT_SEATED
            self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_WAIT_SEATED:
            # No volver a la puerta hasta ver al invitado sentado en el asiento ofrecido
            elapsed = now - self.state_start_time
            bearing = self.seat_bearing_deg
            if bearing is not None and abs(self.target_pan_deg - self.head_pan_sign * bearing) > 3.0:
                self.move_head_to(self.head_pan_sign * bearing, -10.0)
            with self.face_lock:
                poses = list(self.current_poses or [])
            seated = False
            if frame is not None and poses:
                img_w = frame.shape[1]
                fx_px, cx_px = self._fx(img_w), (self.cam_fx_cx[1] if self.cam_fx_cx else img_w / 2.0)
                bearings = [scene.bbox_bearing_width(d["bbox"], self.target_pan_deg, fx_px, cx_px, self.head_pan_sign)[0]
                            for d in poses]
                postures = [d.get("posture", "") for d in poses]
                seated = scene.person_seated_at(bearing, bearings, postures) if bearing is not None else \
                    any(str(p).upper().startswith("SENTAD") for p in postures)
            if seated and now - self.last_pose_time < 1.0:
                self.seated_hits += 1
            elif not seated:
                self.seated_hits = 0
            if self.seated_hits >= 3:
                self.say(f"Great, {self.current_name}. Make yourself comfortable.", emotion="happy", blocking=True)
                if self.guest_index >= 2:
                    self._introduce_guests_to_each_other()
                self.state = self.STATE_CHECK_NEXT_GUEST
                self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo
            elif elapsed > 20.0 and not self.seat_reminded:
                self.seat_reminded = True
                where = scene.direction_words(bearing) if bearing is not None else "here"
                self.say(f"{self.current_name}, please sit down on the free seat {where}.", emotion="speaking", blocking=True)
            elif elapsed > 45.0 or self.force_advance:
                self.force_advance = False
                self.get_logger().warn("No vi al invitado sentarse a tiempo; continuo con el siguiente.")
                if self.guest_index >= 2:
                    self._introduce_guests_to_each_other()
                self.state = self.STATE_CHECK_NEXT_GUEST
                self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_CHECK_NEXT_GUEST:
            if self.guest_index < self.max_guests:
                farewell = "Excuse me, I must return to the entrance door to receive our next guest."
                self.say(farewell, emotion="speaking", blocking=True)
                self.guest_index += 1
                self.current_name = None
                self.current_drink = None
                self.current_attributes = {}
                self.retry_count = 0
                self.voice_text = None
                self.user_speech_buffer = ""
                self.voice.cancel(self.listen_id)
                self.listen_id = None
                self._prepare_door_arrival()
                self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo
            else:
                self.state = self.STATE_CHALLENGE_COMPLETE
                self.state_start_time = time.time()  # tras los say() bloqueantes, no el 'now' del inicio del ciclo

        elif self.state == self.STATE_CHALLENGE_COMPLETE:
            final_msg = "All guests have arrived and are comfortably seated. The Receptionist Challenge is complete. Enjoy the party!"
            self.say(final_msg, emotion="happy", blocking=True)
            self.move_arm_to_named_pose("wave_hello", duration_s=2.0)
            self.sleep_gui(2.5)
            self.move_arm_to_named_pose("cube_stow", duration_s=1.5)
            self.set_robot_emotion("happy")
            self.current_banner_text = "CHALLENGE COMPLETADA EXITOSAMENTE!"
            self._finalize_session_storage()
            self.get_logger().info("🎉 CHALLENGE ROBOCUP @HOME RECEPTIONIST COMPLETADA CON EXITO!")
            self.state = self.STATE_DONE
            self.state_start_time = time.time()

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

        # Detección de poses humanas desde cache (sin inferencia síncrona aquí para no congelar la GUI)
        if self.pose_detector is not None and self.current_poses:
            try:
                canvas[:h, :w] = self.pose_detector.draw_poses(canvas[:h, :w], self.current_poses)
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

        cv2.putText(canvas, "MEMORIA", (px + 15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 255), 2)
        cv2.line(canvas, (px + 10, 36), (canvas_w - 10, 36), (60, 70, 85), 1)

        # Listado de invitados y anfitrión
        py = 55
        # Boton "limpiar memoria temporal" (click, tecla M o servicio /hri/clear_memory)
        bx1, by1, bx2, by2 = px + 150, 12, canvas_w - 10, 34
        cv2.rectangle(canvas, (bx1, by1), (bx2, by2), (40, 40, 160), -1)
        cv2.rectangle(canvas, (bx1, by1), (bx2, by2), (90, 90, 255), 1)
        cv2.putText(canvas, "LIMPIAR [M]", (bx1 + 12, by2 - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        self.clear_button_rect = (bx1, by1, bx2, by2)
        profiles = self.recognizer.session_profiles()
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
        ty = canvas_h - 130
        cv2.line(canvas, (px + 10, ty), (canvas_w - 10, ty), (60, 70, 85), 1)
        ty += 18
        cv2.putText(canvas, "TELEMETRIA", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1)
        ty += 18
        now = time.time()
        time_since_frame = now - self.last_frame_time if self.last_frame_time > 0 else 999.0
        if time_since_frame > 1.0:
            cam_color = (0, 0, 255)
            cam_str = f"Camara: CONGELADA ({time_since_frame:.1f}s sin datos)"
            cv2.rectangle(canvas, (10, 42), (430, 72), (0, 0, 180), -1)
            cv2.putText(canvas, f"ALERTA: CAMARA SIN DATOS ({time_since_frame:.1f}s)", (15, 63), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2)
        else:
            cam_color = (0, 255, 120)
            cam_str = f"Camara: RealSense D435 (#{self.frame_count} - VIVO)"
        cv2.putText(canvas, cam_str, (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.38, cam_color, 1)
        ty += 18
        cv2.putText(canvas, f"Head: Pan {self.target_pan_deg:+.1f} deg | Tilt {self.target_tilt_deg:+.1f} deg", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
        ty += 18
        cv2.putText(canvas, f"Arm Pose: {self.current_arm_pose_name}", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (200, 200, 200), 1)
        ty += 18
        cv2.putText(canvas, f"Mic Status: {self.voice_status}", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 180) if self.voice_status == "LISTENING" else (160, 160, 160), 1)
        ty += 18
        cv2.putText(canvas, f"Nav Mode: {'SIMULADO' if self.sim_nav else 'NAV2 REAL'}", (px + 15, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 200, 0) if self.sim_nav else (0, 255, 120), 1)

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
    parser.add_argument("--no-wait", action="store_true",
                        help="Arranca solo al terminar de cargar, sin esperar apertura de puerta, voz ni ESPACIO")
    parser.add_argument("--max-guests", type=int, default=2, help="Cantidad de invitados a recibir")
    parser.add_argument("--wait-doorbell", action="store_true",
                        help="Esperar timbre antes de ir a la puerta, para cada invitado")
    parser.add_argument("--start-doorbell-audio", action="store_true",
                        help="Lanzar publicador de microfono y activar --wait-doorbell")
    parser.add_argument("--doorbell-audio-device", default="",
                        help="Nombre o indice sounddevice; vacio usa el microfono predeterminado")
    parser.add_argument("--doorbell-timeout-s", type=float, default=30.0,
                        help="Tope de espera del timbre; al vencer continua hacia la puerta")
    parser.add_argument("--doorbell-audio-topic", default="/hri/doorbell/audio",
                        help="Entrada Int16MultiArray: PCM mono 16 kHz, bloques <=1600 muestras")
    parser.add_argument("--no-gentle-tracking", action="store_true",
                        help="Desactivar el seguimiento leve del rostro durante el registro del invitado")
    parser.add_argument("--track-faces", action="store_true",
                        help="Seguir la cara con el pan-tilt (por defecto la cabeza queda al frente)")
    parser.add_argument("--arm-gestures", action="store_true",
                        help="Permitir gestos del brazo (solo con realimentacion CAN y brazo habilitado)")
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()

    node = HriReceptionistChallengeNode(
        camera_topic=args.topic,
        door_wp=args.door_wp,
        living_room_wp=args.living_room_wp,
        sim_nav=args.sim_nav,
        no_gui=args.no_gui,
        wait_start=not args.no_wait,
        max_guests=args.max_guests,
        arm_gestures=args.arm_gestures,
        track_faces=args.track_faces,
        gentle_tracking=not args.no_gentle_tracking,
        wait_doorbell=args.wait_doorbell or args.start_doorbell_audio,
        doorbell_timeout_s=args.doorbell_timeout_s,
        doorbell_audio_topic=args.doorbell_audio_topic,
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    audio_node = None
    if args.start_doorbell_audio and node.wait_doorbell:
        from rclpy.parameter import Parameter
        from base_athome_perception.doorbell_audio_publisher import DoorbellAudioPublisher

        audio_node = DoorbellAudioPublisher(parameter_overrides=[
            Parameter("device", value=args.doorbell_audio_device),
            Parameter("topic", value=node.doorbell_audio_topic),
        ])
        executor.add_node(audio_node)

    def _safe_spin():
        try:
            executor.spin()
        except Exception:
            pass

    ros_thread = threading.Thread(target=_safe_spin, daemon=True)
    ros_thread.start()

    if not args.no_gui:
        cv2.namedWindow(node.window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(node.window_name, node.on_mouse)

    print("\n" + "=" * 70)
    print("🤖 ROBOCUP @HOME - HRI RECEPTIONIST CHALLENGE")
    print("=" * 70)
    print("   [Apertura puerta / 'start'] : Arrancar por láser (/scan) o comando verbal (Regla 3.6.8)")
    print("   [ESPACIO / ENTER]           : Atajo de laboratorio para arrancar; despues, forzar avance de paso")
    print("   [P]               : Volver a revisar el pan-tilt (antes de arrancar)")
    print("   [T]               : Escribir texto por teclado (simular microfono)")
    print("   [R]               : Forzar escucha de microfono (/voice/listen_trigger)")
    print("   [G]               : Probar gesto de senalamiento con brazo Piper")
    print("   [N]               : Omitir navegacion actual")
    print("   [M] / boton LIMPIAR : Limpiar memoria temporal de personas (invitados)")
    print("   [Q / ESC]         : Salir limpiamente y guardar memoria")
    print("=" * 70 + "\n")

    try:
        while rclpy.ok() and not node.request_shutdown:
            node.step()
            node.pump_gui(20)
    except KeyboardInterrupt:
        pass
    finally:
        node.speaker.stop()
        if not args.no_gui:
            cv2.destroyAllWindows()
        executor.shutdown()
        if audio_node is not None:
            audio_node.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
