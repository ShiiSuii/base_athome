#!/usr/bin/env python3
"""Verificador previo de competencia para RoboCup@Home OPL (HOME-93).

SOLO LECTURA: no mueve actuadores ni publica comandos.
Evalua antes de ingresar a la pista: nodos y topicos clave, mapa cargado,
poses enseñadas, camaras, microfono/ASR, salida de audio/TTS, lugares y objetos.
Imprime lista de comprobacion manual final (E-stop, peso, dimensiones, etc.).

Sale con rc 0 solo si todas las comprobaciones automaticas estan OK.
Uso:
    python3 src/base_athome_perception/scripts/revisar_competencia.py
    python3 src/base_athome_perception/scripts/revisar_competencia.py --offline
"""

import argparse
import importlib.util
import json
import math
import os
import re
import subprocess
import sys
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, "..", "models")
SEMANTIC_MAP = os.path.join(HERE, "..", "..", "base_athome_navigation", "config", "semantic_map.yaml")
WAYPOINTS_PATH = os.path.join(HERE, "..", "..", "base_athome_navigation", "config", "waypoints.yaml")
OBJECTS_PATH = os.path.join(HERE, "..", "..", "..", "tools", "gpsr_parser", "data", "objects", "objects.md")
NAMES_PATH = os.path.join(HERE, "..", "..", "..", "tools", "gpsr_parser", "data", "names", "names.md")
ARM_CONFIG_PATH = os.path.join(HERE, "..", "config", "arm_poses_and_limits.yaml")
PIPER_BIN = "/home/athome/piper_bin/piper/piper"
PIPER_MODEL = "/home/athome/piper_bin/voices/en/en_US-lessac-medium.onnx"
BEEP_PATH = os.path.join(HERE, "..", "resources", "beep.wav")
DEFAULT_MAP_YAML = "/home/athome/mapa_base_athome.yaml"
DEFAULT_MAP_PGM = "/home/athome/mapa_base_athome.pgm"
WAIT_S = 6.0

TOPICS = (
    ("/scan", LaserScan, "lidar Hokuyo (/scan)", "revisar el Hokuyo, alimentacion 12V y cable de red"),
    ("/odom", Odometry, "odometria de base (/odom)", "revisar driver hoverboard y ros2 launch base_athome_bringup base.launch.py"),
    ("/pan_tilt/joint_states", JointState, "cabeza pan-tilt (/pan_tilt/joint_states)", "calibrar_pantilt: revisar alimentacion 12V y cable USB del U2D2"),
    ("/camera/camera/color/image_raw", Image, "RealSense color (/camera/camera/color/image_raw)", "reconectar RealSense al puerto USB 3 azul de la NUC"),
    ("/camera/camera/aligned_depth_to_color/image_raw", Image, "RealSense profundidad alineada", "reconectar RealSense al puerto USB 3 azul de la NUC"),
    ("/camera/camera/color/camera_info", CameraInfo, "RealSense calibracion (camera_info)", "revisar nodo realsense_nuc.launch.py"),
    ("/arm_camera/image_raw", Image, "camara de muneca DaBai (/arm_camera/image_raw)", "revisar la DaBai y el servicio jetson-head en la Jetson"),
    ("/arm/joint_states", JointState, "articulaciones del brazo Piper (/arm/joint_states)", "energizar el Piper; revisar bus CAN en la Jetson"),
    ("/torso/joint_states", JointState, "altura del elevador (/torso/joint_states)", "falta torso_state: abrir commander"),
    ("/map", OccupancyGrid, "mapa de navegacion Nav2 (/map)", "recargar mapa con src/base_athome_skills/scripts/nav_map_sync.sh"),
)

FRAMES = (
    ("map", "base_footprint", "ubicacion en el mapa (AMCL / TF)", "fijar pose inicial estimada en RViz o reiniciar AMCL"),
    ("base_footprint", "camera_color_optical_frame", "TF de la cabeza RealSense", "falta joint_state_publisher o datos del pan_tilt"),
    ("base_footprint", "arm_camera_optical_frame", "TF de la muneca DaBai", "falta joint_state_publisher o /arm/joint_states"),
)

REQUIRED_POSES = (
    ("home", "pose home (transito seguro)", "ensenar pose home con ensenar_pregrasp.py o piper_teach_recorder.py"),
    ("pre_grasp", "pose pre_grasp (aproximacion mesa)", "ensenar pose pre_grasp con ensenar_pregrasp.py"),
    ("post_grasp", "pose post_grasp (retiro seguro)", "ensenar pose post_grasp con ensenar_pregrasp.py"),
    ("pre_grasp_floor", "pose pre_grasp_floor (piso / basura)", "ensenar pose pre_grasp_floor con ensenar_pregrasp.py"),
)


# ==============================================================================
# 1. PARSERS Y FUNCIONES AUXILIARES PURAS
# ==============================================================================

def parse_names_markdown(text: str) -> List[str]:
    """Extrae la lista de nombres del markdown oficial de competencia."""
    names: List[str] = []
    for line in text.splitlines():
        m = re.match(r"^\|\s*([A-Za-z][A-Za-z _'-]*?)\s*\|", line)
        if m:
            val = m.group(1).strip()
            if val.lower() not in ("names", "name", "---", ":---:", ":---"):
                names.append(val)
    return names


def parse_objects_markdown(text: str) -> Dict[str, List[str]]:
    """Extrae categorias y objetos del markdown oficial de competencia."""
    out: Dict[str, List[str]] = {}
    current_cat = "general"
    for line in text.splitlines():
        l = line.strip()
        if l.startswith("# Class"):
            m = re.match(r"# Class\s+([A-Za-z0-9_]+)", l)
            if m:
                current_cat = m.group(1).strip()
                out.setdefault(current_cat, [])
            continue
        m = re.match(r"^\|\s*([a-z0-9_]+)\s*\|", l)
        if m:
            obj = m.group(1).strip()
            if obj not in ("objectname", "---", ":---:", ":---"):
                out.setdefault(current_cat, []).append(obj)
    return out


def parse_pactl_sinks(text: str) -> List[str]:
    """Parsea las lineas de salida de 'pactl list sinks short'."""
    sinks: List[str] = []
    for line in text.splitlines():
        parts = line.strip().split()
        if len(parts) >= 2:
            sinks.append(parts[1])
    return sinks


def get_system_pactl_sinks() -> List[str]:
    """Consulta dispositivos de salida de audio del sistema (PipeWire/PulseAudio)."""
    try:
        res = subprocess.run(["pactl", "list", "sinks", "short"], stdout=subprocess.PIPE, text=True, timeout=2)
        return parse_pactl_sinks(res.stdout)
    except Exception:
        return []


def read_file_safe(path: str) -> str:
    """Lee un archivo de texto de forma segura; vacio si no existe o falla."""
    if not path or not os.path.exists(path):
        return ""
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read()
    except Exception:
        return ""


def read_yaml_safe(path: str) -> dict:
    """Lee un archivo YAML de forma segura; dict vacio si falla."""
    text = read_file_safe(path)
    if not text:
        return {}
    try:
        return yaml.safe_load(text) or {}
    except Exception:
        return {}


# ==============================================================================
# 2. LOGICA DE EVALUACION PURA (TESTEABLE SIN ROS)
# ==============================================================================

def evaluar_mapa(map_yaml_path: str, map_image_path: str, live_map_seen: Optional[bool] = None) -> List[Tuple[bool, str, str]]:
    """Evalua los archivos de mapa en disco y opcionalmente el estado live del topico /map."""
    res: List[Tuple[bool, str, str]] = []
    yaml_name = os.path.basename(map_yaml_path) if map_yaml_path else "mapa.yaml"

    if not map_yaml_path or not os.path.exists(map_yaml_path):
        res.append((False, f"archivo YAML del mapa ({yaml_name})", "guardar mapa con scripts/guardar_mapa_2d.sh"))
    else:
        cfg = read_yaml_safe(map_yaml_path)
        if "resolution" in cfg and "origin" in cfg:
            res.append((True, f"archivo YAML del mapa ({yaml_name}, res={cfg['resolution']}m)", ""))
        else:
            res.append((False, f"archivo YAML del mapa ({yaml_name})", "archivo de mapa incompleto: falta resolution u origin"))

    img_name = os.path.basename(map_image_path) if map_image_path else "mapa.pgm"
    if not map_image_path or not os.path.exists(map_image_path):
        res.append((False, f"archivo de imagen del mapa ({img_name})", "verificar imagen .pgm o .png del mapa en disco"))
    else:
        size = os.path.getsize(map_image_path)
        if size > 1024:
            res.append((True, f"archivo de imagen del mapa ({img_name}, {size // 1024} KB)", ""))
        else:
            res.append((False, f"archivo de imagen del mapa ({img_name})", "archivo de imagen de mapa vacio o corrupto"))

    if live_map_seen is not None:
        if live_map_seen:
            res.append((True, "topico /map activo en Nav2", ""))
        else:
            res.append((False, "topico /map activo en Nav2", "recargar mapa con src/base_athome_skills/scripts/nav_map_sync.sh"))

    return res


def evaluar_poses(pose_cfg: dict, solver: Optional[Any] = None) -> List[Tuple[bool, str, str]]:
    """Evalua que existan las poses clave y que sus angulos esten dentro del safety_envelope."""
    res: List[Tuple[bool, str, str]] = []
    named_poses = pose_cfg.get("named_poses") or pose_cfg.get("poses") or {}
    safety = pose_cfg.get("safety_envelope") or {}
    limits = safety.get("joint_limits_deg") or {}

    for pose_key, label, fix in REQUIRED_POSES:
        pose = named_poses.get(pose_key)
        if not pose:
            res.append((False, label, fix))
            continue

        joints = pose.get("joints_deg")
        if not isinstance(joints, (list, tuple)) or len(joints) != 6:
            count = len(joints) if isinstance(joints, (list, tuple)) else 0
            res.append((False, f"{label} ({count} articulaciones)", f"re-ensenar pose con exactamente 6 articulaciones: {fix}"))
            continue

        violations = []
        for idx, val in enumerate(joints):
            jname = f"joint{idx + 1}"
            if jname in limits:
                jmin, jmax = limits[jname]
                if val < (jmin - 0.05) or val > (jmax + 0.05):
                    violations.append(f"{jname}={val:.1f}deg fuera de [{jmin}, {jmax}]")

        if violations:
            res.append((False, f"{label} (limites excedidos)", "; ".join(violations) + f": {fix}"))
        else:
            res.append((True, label, ""))

    return res


def evaluar_camaras(seen_topics: Dict[str, Any], models_dir: str) -> List[Tuple[bool, str, str]]:
    """Evalua estado de camaras (RealSense y DaBai de muñeca) y pesos de modelos de vision."""
    res: List[Tuple[bool, str, str]] = []

    cam_topics = [
        ("/camera/camera/color/image_raw", "camara RealSense color (/camera/camera/color/image_raw)", "reconectar RealSense al puerto USB 3 azul de la NUC"),
        ("/camera/camera/aligned_depth_to_color/image_raw", "camara RealSense profundidad alineada", "reconectar RealSense al puerto USB 3 azul de la NUC"),
        ("/camera/camera/color/camera_info", "camara RealSense calibracion (camera_info)", "revisar nodo realsense_nuc.launch.py"),
        ("/arm_camera/image_raw", "camara de muneca DaBai (/arm_camera/image_raw)", "revisar la DaBai y el servicio jetson-head en la Jetson"),
    ]
    for topic, label, fix in cam_topics:
        res.append((topic in seen_topics, label, fix))

    for name in ("yolov8s-worldv2.pt", "yolov8n-pose.pt"):
        path = os.path.join(models_dir, name)
        ok = os.path.exists(path) and os.path.getsize(path) > 1000000
        res.append((ok, f"pesos de vision {name}", "correr scripts/download_models.sh"))

    return res


def evaluar_audio_tts(piper_bin: str, model_path: str, beep_path: str, pactl_sinks: Optional[List[str]] = None) -> List[Tuple[bool, str, str]]:
    """Evalua motor TTS Piper, modelo de voz, beep de turn-taking y sinks de audio."""
    res: List[Tuple[bool, str, str]] = []

    bin_ok = bool(piper_bin and os.path.exists(piper_bin) and os.access(piper_bin, os.X_OK))
    res.append((bin_ok, "motor TTS neuronal Piper", f"instalar binario de piper en {piper_bin}"))

    model_ok = bool(model_path and os.path.exists(model_path) and os.path.getsize(model_path) > 1000000)
    res.append((model_ok, "modelo de voz en_US-lessac-medium", f"descargar modelo ONNX en {model_path}"))

    beep_ok = bool(beep_path and os.path.exists(beep_path) and os.path.getsize(beep_path) > 500)
    res.append((beep_ok, "audio turn-taking (beep.wav)", f"verificar beep.wav en {beep_path}"))

    if pactl_sinks is not None:
        if not pactl_sinks:
            res.append((False, "salida de audio del sistema (PipeWire/PulseAudio)", "iniciar PipeWire/PulseAudio o conectar parlante: pactl list sinks"))
        else:
            all_bt = all("bluez_output" in s for s in pactl_sinks)
            if all_bt:
                res.append((False, "salida de audio cableada/RF para jueces (Setup.tex:112)", "conectar salida cableada jack 3.5 mm / transmisor RF (actualmente solo Bluetooth)"))
            else:
                res.append((True, f"salida de audio cableada/interna disponible ({len(pactl_sinks)} sinks)", ""))

    return res


def evaluar_microfono_asr(asr_status: Optional[str], topics_seen: Dict[str, Any], names_path: str) -> List[Tuple[bool, str, str]]:
    """Evalua microfono, servicio ASR y archivo de nombres de competencia."""
    res: List[Tuple[bool, str, str]] = []

    if asr_status is None:
        res.append((False, "nodo ASR de microfono (/voice/status)", "iniciar nodo audio_asr en la Jetson (jetson-head.service)"))
    elif asr_status == "NO_MIC":
        res.append((False, "microfono conectado en Jetson", "conectar microfono direccional RØDE al puerto USB de la Jetson y reiniciar ASR"))
    else:
        res.append((True, f"microfono y ASR activos (estado: {asr_status})", ""))

    has_result = "/voice/result" in topics_seen or "/voice/transcription" in topics_seen
    if has_result:
        res.append((True, "canal de transcripcion de voz (/voice/result)", ""))

    names_text = read_file_safe(names_path)
    if not names_text:
        res.append((False, "nombres oficiales de competencia (names.md)", f"cargar nombres del TC en {names_path}"))
    else:
        names = parse_names_markdown(names_text)
        if len(names) >= 1:
            res.append((True, f"nombres oficiales cargados ({len(names)} nombres para Whisper)", ""))
        else:
            res.append((False, "nombres oficiales de competencia (names.md)", "el archivo names.md no contiene nombres validos"))

    return res


def evaluar_lugares(semantic_map_data: dict, waypoints_data: dict) -> List[Tuple[bool, str, str]]:
    """Evalua waypoints y zonas semanticas requeridas para la arena."""
    res: List[Tuple[bool, str, str]] = []

    wps: Dict[str, Any] = dict(waypoints_data.get("waypoints") or {})
    wps.update(semantic_map_data.get("waypoints") or {})

    has_door = any(k in wps for k in ("door", "entrance", "start", "entry"))
    if has_door:
        res.append((True, "punto de arranque de la arena ('door'/'entrance')", ""))
    else:
        res.append((False, "punto de arranque de la arena ('door'/'entrance')", "editor_mapa: marcar waypoint 'door' o 'entrance' y guardar"))

    has_exit = any(k in wps for k in ("exit", "salida", "exit_door"))
    if has_exit:
        res.append((True, "punto de salida de la arena ('exit')", ""))
    else:
        res.append((False, "punto de salida de la arena ('exit')", "editor_mapa: marcar waypoint 'exit' para salida reglamentaria en 3 min"))

    loc_count = len(wps)
    if loc_count >= 3:
        res.append((True, f"waypoints de navegacion cargados ({loc_count} ubicaciones)", ""))
    else:
        res.append((False, "waypoints de navegacion cargados", f"solo hay {loc_count} waypoints; definir al menos 3 salas/mesas con editor_mapa"))

    zones = semantic_map_data.get("zones") or {}
    if zones:
        res.append((True, f"zonas semanticas delimitadas ({len(zones)} zonas)", ""))
    else:
        res.append((False, "zonas semanticas delimitadas", "marcar zonas con editor_mapa en semantic_map.yaml"))

    return res


def evaluar_objetos(objects_path: str, semantic_map_data: Optional[dict] = None) -> List[Tuple[bool, str, str]]:
    """Evalua lista de objetos y categorias oficiales para competencia."""
    res: List[Tuple[bool, str, str]] = []

    text = read_file_safe(objects_path)
    if not text:
        res.append((False, "archivo de objetos de competencia (objects.md)", f"cargar objetos del TC en {objects_path}"))
    else:
        cat_objs = parse_objects_markdown(text)
        total = sum(len(objs) for objs in cat_objs.values())
        if total >= 30:
            res.append((True, f"objetos oficiales cargados ({total} objetos en {len(cat_objs)} categorias)", ""))
        elif total > 0:
            res.append((True, f"objetos cargados ({total} objetos en {len(cat_objs)} categorias; min 30 recomendado)", ""))
        else:
            res.append((False, "objetos oficiales de competencia (objects.md)", "objects.md no contiene objetos validos"))

    return res


def evaluar_topicos_y_hardware(
    seen_topics: Dict[str, Any],
    tf_status: Dict[Tuple[str, str], bool],
    arm_status: Optional[dict],
    torso_status: Optional[dict],
    amcl_received: bool,
) -> List[Tuple[bool, str, str]]:
    """Evalua topicos de hardware, conectividad y transformadas TF."""
    res: List[Tuple[bool, str, str]] = []

    for topic, _, label, fix in TOPICS:
        res.append((topic in seen_topics, label, fix))

    arm = arm_status or {}
    res.append((bool(arm.get("feedback_ok")), "brazo Piper con feedback CAN", "energizar el Piper; revisar bus CAN en la Jetson"))

    torso = torso_status or {}
    res.append((bool(torso.get("connected")), "elevador conectado", "revisar conexion USB del ESP32 del elevador"))
    res.append((bool(torso.get("calibrated")), "elevador calibrado", "calibrar_elevador"))

    res.append((amcl_received, "AMCL publica la pose (/amcl_pose)", "dar la pose inicial en RViz; revisar Nav2/AMCL"))

    for parent, child, label, fix in FRAMES:
        can_tf = tf_status.get((parent, child), False)
        res.append((can_tf, label, fix))

    return res


def obtener_checklist_manual() -> List[Dict[str, str]]:
    """Devuelve la lista estructurada de verificaciones fisicas manuales obligatorias."""
    return [
        {
            "code": "E-STOP",
            "rule": "Setup.tex:104, Procedure.tex:6",
            "title": "Boton de Parada de Emergencia (E-Stop)",
            "desc": "Pulsador hongo rojo visible (UNICO boton rojo del robot). Corte bipolar que desenergiza simultaneamente base hoverboard, brazo Piper y elevador.",
            "cmd": "Probar pulsado con robot en soporte: verificar desenergizacion total instantanea.",
        },
        {
            "code": "DIMENSIONES",
            "rule": "Robots.tex:21-35",
            "title": "Dimensiones y Peso Reglamentarios",
            "desc": "Ancho total <= 70 cm, Altura <= 200 cm, Peso registrado en balanza de boxes.",
            "cmd": "Medir ancho y alto con cinta metrica; pesar el robot en balanza antes de la inspeccion.",
        },
        {
            "code": "INICIO_MANUAL",
            "rule": "Procedure.tex:100-108",
            "title": "Senal Manual de Inicio Homologada",
            "desc": "Pulsador fisico START verde/azul o sensor de contacto. PROHIBIDO terminantemente pulsar [ESPACIO] o [ENTER] en teclado.",
            "cmd": "Verificar que la mision inicie con pulsador fisico o comando de voz sin tocar teclado.",
        },
        {
            "code": "APARIENCIA",
            "rule": "Setup.tex:117",
            "title": "Seguridad Mecanica y Apariencia Exterior",
            "desc": "Cero cables colgando sueltos, sin cinta adhesiva expuesta (duct tape), sin rebabas ni aristas cortantes (-50 pts si desprolijo).",
            "cmd": "Asegurar cableado con precintos organizadores y retirar cinta provisoria.",
        },
        {
            "code": "RED_ARENANETWORK",
            "rule": "Scenario.tex:179-198",
            "title": "Conexion de Red y Prohibicion de Hotspots",
            "desc": "Conectado exclusivamente a la VLAN ArenaNetwork_<TeamID>. CERO puntos de acceso / hotspots emitidos por NUC o Jetson.",
            "cmd": "nmcli dev wifi list && nmcli dev status",
        },
        {
            "code": "SALIDA_AUDIO_RF",
            "rule": "Setup.tex:112",
            "title": "Salida de Audio Cableada / Transmisor RF",
            "desc": "Conector jack 3.5 mm dual o transmisor inalambrico RF probado con la consola de megafonia del estadio.",
            "cmd": "paplay /home/athome/piper_bin/voices/en/test.wav",
        },
        {
            "code": "ALIMENTACION",
            "rule": "Seguridad operativa",
            "title": "Baterias y Nivel de Carga",
            "desc": "Bateria de traccion 10S >41.5 V, bateria de computo >11.8 V. Cargador 42V desenchufado y guardado en boxes.",
            "cmd": "Medir voltaje en conector XT60 con multimetro y retirar cargador de red.",
        },
    ]


def imprimir_checklist_manual() -> None:
    """Imprime el checklist de comprobacion manual previo a la Robot Inspection."""
    print("\n" + "=" * 80)
    print("LISTA DE COMPROBACION MANUAL PREVIA A COMPETENCIA (Robot Inspection & Safety)")
    print("=" * 80)
    for item in obtener_checklist_manual():
        print(f"[ ] {item['title']} [{item['code']}] (Regla: {item['rule']}):")
        print(f"    {item['desc']}")
        print(f"    -> Comprobacion: {item['cmd']}\n")


# ==============================================================================
# 3. NODO PROBE DE SOLO LECTURA (ROS 2)
# ==============================================================================

class Probe(Node):
    """Nodo temporal de solo lectura para capturar telemetria sin emitir comandos."""

    def __init__(self):
        super().__init__("revisar_competencia")
        self.seen: Dict[str, float] = {}
        self.arm: Optional[dict] = None
        self.torso: Optional[dict] = None
        self.asr_status: Optional[str] = None
        self.amcl: Optional[PoseWithCovarianceStamped] = None
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        for topic, kind, _, _ in TOPICS:
            self.create_subscription(kind, topic, lambda _m, t=topic: self.seen.__setitem__(t, time.time()), qos_profile_sensor_data)

        self.create_subscription(String, "/arm/status", self._on_arm_status, 10)
        self.create_subscription(String, "/torso/status", self._on_torso_status, 10)
        self.create_subscription(String, "/voice/status", self._on_voice_status, 10)
        self.create_subscription(String, "/voice/result", lambda _m: self.seen.__setitem__("/voice/result", time.time()), 10)
        self.create_subscription(String, "/voice/transcription", lambda _m: self.seen.__setitem__("/voice/transcription", time.time()), 10)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: setattr(self, "amcl", m), latched)

    def _on_arm_status(self, m: String):
        try:
            self.arm = json.loads(m.data)
        except Exception:
            pass

    def _on_torso_status(self, m: String):
        try:
            self.torso = json.loads(m.data)
        except Exception:
            pass

    def _on_voice_status(self, m: String):
        self.asr_status = m.data.strip().upper()
        self.seen["/voice/status"] = time.time()


# ==============================================================================
# 4. FLUJO PRINCIPAL DE VERIFICACION
# ==============================================================================

def check_files(out: Callable[[bool, str, str], None], custom_paths: Optional[dict] = None) -> None:
    """Verifica archivos, configuraciones y modelos en disco de forma offline."""
    paths = {
        "map_yaml": DEFAULT_MAP_YAML,
        "map_pgm": DEFAULT_MAP_PGM,
        "arm_config": ARM_CONFIG_PATH,
        "semantic_map": SEMANTIC_MAP,
        "waypoints": WAYPOINTS_PATH,
        "objects": OBJECTS_PATH,
        "names": NAMES_PATH,
        "piper_bin": PIPER_BIN,
        "piper_model": PIPER_MODEL,
        "beep": BEEP_PATH,
        "models_dir": MODELS,
    }
    if custom_paths:
        paths.update(custom_paths)

    sem_data = read_yaml_safe(paths["semantic_map"])
    map_info = sem_data.get("map_info") or {}
    map_yaml = map_info.get("yaml_file") or paths["map_yaml"]
    map_pgm = map_info.get("image_file") or paths["map_pgm"]

    # 1. Mapa en archivos
    for ok, label, fix in evaluar_mapa(map_yaml, map_pgm):
        out(ok, label, fix)

    # 2. Poses del brazo
    arm_cfg = read_yaml_safe(paths["arm_config"])
    for ok, label, fix in evaluar_poses(arm_cfg):
        out(ok, label, fix)

    # 3. Modelos de vision
    for ok, label, fix in evaluar_camaras(seen_topics={}, models_dir=paths["models_dir"]):
        if "pesos de vision" in label:
            out(ok, label, fix)

    # 4. Audio TTS y speaker
    sinks = get_system_pactl_sinks()
    for ok, label, fix in evaluar_audio_tts(paths["piper_bin"], paths["piper_model"], paths["beep"], pactl_sinks=sinks):
        out(ok, label, fix)

    # 5. Microfono y nombres
    for ok, label, fix in evaluar_microfono_asr(asr_status="OFFLINE_CHECK", topics_seen={}, names_path=paths["names"]):
        if "nombres" in label:
            out(ok, label, fix)

    # 6. Lugares semanticos
    wps_data = read_yaml_safe(paths["waypoints"])
    for ok, label, fix in evaluar_lugares(sem_data, wps_data):
        out(ok, label, fix)

    # 7. Objetos de competencia
    for ok, label, fix in evaluar_objetos(paths["objects"], sem_data):
        out(ok, label, fix)


def check_live(out: Callable[[bool, str, str], None], wait_seconds: float = WAIT_S) -> None:
    """Verifica en vivo por suscripcion pasiva (ROS 2) sin publicar ningun comando."""
    rclpy.init()
    node = Probe()
    end = time.time() + wait_seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    tf_status: Dict[Tuple[str, str], bool] = {}
    for parent, child, _, _ in FRAMES:
        can_tf = False
        try:
            can_tf = node.tf_buf.can_transform(parent, child, rclpy.time.Time())
        except Exception:
            can_tf = False
        tf_status[(parent, child)] = can_tf

    # Evaluacion de topicos de hardware y TFs
    for ok, label, fix in evaluar_topicos_y_hardware(
        seen_topics=node.seen,
        tf_status=tf_status,
        arm_status=node.arm,
        torso_status=node.torso,
        amcl_received=(node.amcl is not None),
    ):
        out(ok, label, fix)

    # Evaluacion de streams de camaras
    for ok, label, fix in evaluar_camaras(seen_topics=node.seen, models_dir=MODELS):
        if "pesos de vision" not in label:
            out(ok, label, fix)

    # Evaluacion en vivo de ASR
    for ok, label, fix in evaluar_microfono_asr(asr_status=node.asr_status, topics_seen=node.seen, names_path=NAMES_PATH):
        if "nombres" not in label:
            out(ok, label, fix)

    # Estado de orquestador de mision / commander
    cmd_subs = node.count_subscribers("/commander/command")
    mission_subs = node.count_subscribers("/mission/run")
    bag_subs = node.count_subscribers("/bag_finder/request")
    has_orchestrator = (cmd_subs > 0 or mission_subs > 0 or bag_subs > 0)
    out(has_orchestrator, "orquestador de mision activo (commander/mission)", "abrir commander: ros2 run base_athome_skills commander")

    node.destroy_node()
    rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(description="Verificador previo de competencia RoboCup@Home (solo lectura)")
    parser.add_argument("--offline", action="store_true", help="Omitir la prueba de topicos en vivo de ROS 2")
    parser.add_argument("--timeout", type=float, default=WAIT_S, help="Segundos de escucha para el sondeo en vivo")
    args = parser.parse_args()

    bad: List[str] = []

    def out(ok: bool, label: str, fix: str) -> None:
        print(f"[{'OK' if ok else 'FALTA'}] {label}" + ("" if ok else f"\n        -> {fix}"))
        if not ok:
            bad.append(label)

    print("=== BASE@HOME — VERIFICADOR PREVIO DE COMPETENCIA (SOLO LECTURA) ===\n")
    print("--- [1/2] Verificacion de Archivos, Modelos y Poses ---")
    check_files(out)

    if not args.offline:
        print(f"\n--- [2/2] Verificacion en Vivo de Topicos y Hardware ({args.timeout:.1f} s) ---")
        check_live(out, wait_seconds=args.timeout)
    else:
        print("\n--- [2/2] Verificacion en Vivo Omitida (--offline) ---")

    imprimir_checklist_manual()

    if not bad:
        print("\nTodo listo en software y hardware para competir.")
        return 0
    else:
        print(f"\nFaltan {len(bad)} cosas en verificacion automatica: no ingresar a la arena todavia.")
        return 1


if __name__ == "__main__":
    np.set_printoptions(precision=3)
    sys.exit(main())
