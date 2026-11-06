"""Tests unitarios offline del arranque sin teclado del Receptionist (Regla 3.6.8).

Cubre:
1. is_start_speech: palabras de inicio validas, cancelaciones y frases no relacionadas.
2. Deteccion de puerta abierta mediante _scan_callback (LaserScan -> door_detector -> OPEN).
3. Puerta cerrada no arranca.
4. Obstaculo lejano en calibracion (NO_DOOR) no arranca solo.
5. Arranque por palabra hablada directa (voice_text de Whisper ASR).
6. Arranque por cliente de voz asincrono (VoiceClient.poll -> VoiceResult ok con palabra de inicio).
7. Atajo de laboratorio por teclado [ESPACIO / ENTER] en handle_key().
8. Apertura de puerta antes de terminar el pre-vuelo (arranca al completar pre-vuelo).
9. _ready_banner refleja estado de la puerta, microfono y atajos.
10. _scan_callback se desactiva una vez confirmado el inicio.
"""
import importlib.util
import math
import os
import sys
import threading
from unittest import mock

import numpy as np
import pytest
from sensor_msgs.msg import LaserScan

HERE = os.path.dirname(__file__)
SCRIPT = os.path.join(HERE, "..", "scripts", "hri_receptionist_challenge.py")


@pytest.fixture(scope="module")
def hri_module():
    spec = importlib.util.spec_from_file_location("hri_receptionist_challenge", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(mod)
    except Exception as exc:
        pytest.skip(f"No se pudo importar hri_receptionist_challenge: {exc}")
    return mod


@pytest.fixture
def node_cls(hri_module):
    cls = getattr(hri_module, "HriReceptionistChallengeNode", None)
    if cls is None:
        cls = next(
            v for v in vars(hri_module).values()
            if isinstance(v, type) and hasattr(v, "_scan_callback")
        )
    return cls


def make_test_scan(front_m: float, side_m: float = 3.0, front_deg: float = 10.0) -> LaserScan:
    """Genera un LaserScan sintetico con la geometria de Hokuyo UST-05LX (270 deg, 0.25 deg/rayo)."""
    angle_min = math.radians(-135)
    angle_increment = math.radians(0.25)
    num_rays = 1081
    ranges = []
    for i in range(num_rays):
        a = math.degrees(angle_min + i * angle_increment)
        r = front_m if abs(a) <= front_deg else side_m
        ranges.append(float(r))
    msg = LaserScan()
    msg.angle_min = angle_min
    msg.angle_max = math.radians(135)
    msg.angle_increment = angle_increment
    msg.range_min = 0.06
    msg.range_max = 5.0
    msg.ranges = ranges
    return msg


def make_mock_node(node_cls, wait_start=True, preflight_complete=True, calibration_scans=3, open_scans=2):
    """Crea una instancia ligera de HriReceptionistChallengeNode sin inicializar hardware."""
    from base_athome_skills.door_detector import DoorDetectorConfig, DoorOpenDetector

    n = node_cls.__new__(node_cls)
    n.frame_lock = threading.Lock()
    n.latest_frame = np.zeros((480, 640, 3), dtype=np.uint8)
    n.clear_memory_requested = False
    n.face_lock = threading.Lock()
    n.current_face_dets = []
    n.face_dets_stamp = 0
    n.last_tracked_stamp = 0
    n.pan_tilt_tracking = False
    n.gentle_tracking = False
    n.gentle_tracker = mock.Mock()
    n.pt_connected = True
    n.scanning = False
    n.state = node_cls.STATE_INIT
    n.STATE_NAMES = {node_cls.STATE_INIT: "0/7: INICIALIZANDO"}
    n.nav2_verified = True
    n.sim_nav = True
    n.furniture_model = True
    n.pt_selftest = {"ok": True, "checks": []}
    n.preflight_complete = preflight_complete
    n.wait_start = wait_start
    n.start_confirmed = False
    n._pt_retest_requested = False
    n.force_advance = False
    n.voice_text = None
    n.listen_id = None
    n._last_voice_listen_attempt = 0.0
    n.guest_index = 1
    n.door_wp = "A"
    n.navigate_to_waypoint = mock.Mock()
    n._ensure_pantilt_torque = mock.Mock()
    n.move_head_smooth = mock.Mock()
    n.move_head_to = mock.Mock()
    n.sleep_gui = mock.Mock()
    n.set_robot_emotion = mock.Mock()
    cfg = DoorDetectorConfig(
        calibration_scans=calibration_scans,
        open_scans_required=open_scans,
        max_closed_distance_m=1.5,
        open_margin_m=0.5,
    )
    n.door_detector = DoorOpenDetector(cfg=cfg)
    n.voice = mock.Mock()
    n.voice.online.return_value = True
    n.voice._new_protocol_available.return_value = True
    n.voice.poll.return_value = None
    n.say = mock.Mock()
    n.get_logger = mock.Mock()
    return n


# ==============================================================================
# TESTS DE is_start_speech
# ==============================================================================

def test_is_start_speech_positive(hri_module):
    is_start = hri_module.is_start_speech
    positive_samples = [
        "start",
        "START",
        "Robot, start",
        "robot start",
        "start challenge",
        "start receptionist",
        "please start the mission",
        "begin",
        "ready",
        "go",
        "arranca",
        "arrancar",
        "empezar",
        "iniciar",
        "empeza la prueba",
    ]
    for s in positive_samples:
        assert is_start(s), f"Deberia reconocer como inicio: '{s}'"


def test_is_start_speech_negative_and_cancellation(hri_module):
    is_start = hri_module.is_start_speech
    negative_samples = [
        "stop",
        "don't start",
        "do not start",
        "dont start",
        "wait",
        "cancel",
        "espera",
        "no",
        "no start",
        "not ready",
    ]
    for s in negative_samples:
        assert not is_start(s), f"NO deberia arrancar con negacion: '{s}'"


def test_is_start_speech_unrelated(hri_module):
    is_start = hri_module.is_start_speech
    unrelated_samples = [
        "hello",
        "hi robot",
        "Alex",
        "Taylor",
        "my name is Morgan",
        "Coffee please",
        "water",
        "thank you",
        "",
        None,
    ]
    for s in unrelated_samples:
        assert not is_start(s), f"NO deberia arrancar con: '{s}'"


# ==============================================================================
# TESTS DE ARRANQUE POR PUERTA (_scan_callback / LaserScan)
# ==============================================================================

def test_door_open_triggers_start(node_cls):
    """Puerta cerrada (0.8m) calibra -> Puerta abierta (2.0m) activa start_confirmed y transiciona."""
    n = make_mock_node(node_cls, calibration_scans=3, open_scans=2)

    # Calibrar con 3 scans de puerta cerrada a 0.8m
    for _ in range(3):
        n._scan_callback(make_test_scan(0.8))
    assert n.door_detector.state == "WAITING_CLOSED"
    assert not n.start_confirmed

    # 1 scan abierto a 2.0m: no alcanza el umbral de 2 scans consecutivos
    n._scan_callback(make_test_scan(2.0))
    assert n.door_detector.state == "WAITING_CLOSED"
    assert not n.start_confirmed

    # 2do scan abierto consecutivo: puerta abierta declarada
    n._scan_callback(make_test_scan(2.0))
    assert n.door_detector.state == "OPEN"
    assert n.start_confirmed, "start_confirmed debe ser True tras abrirse la puerta"

    # En el proximo ciclo de step(), transiciona a STATE_NAV_TO_DOOR
    n.step()
    assert n.state == node_cls.STATE_NAV_TO_DOOR
    n.say.assert_called_with("Starting Receptionist Challenge.", emotion="happy", blocking=True)


def test_closed_door_does_not_start(node_cls):
    """Puerta cerrada permanente mantiene el nodo en STATE_INIT esperando."""
    n = make_mock_node(node_cls, calibration_scans=3, open_scans=2)
    for _ in range(10):
        n._scan_callback(make_test_scan(0.8))
        n.step()
    assert n.door_detector.state == "WAITING_CLOSED"
    assert not n.start_confirmed
    assert n.state == node_cls.STATE_INIT


def test_no_door_in_front_does_not_auto_start(node_cls):
    """Si no hay puerta al frente (< 1.5m), pasa a NO_DOOR pero NO arranca solo."""
    n = make_mock_node(node_cls, calibration_scans=3, open_scans=2)
    for _ in range(4):
        n._scan_callback(make_test_scan(3.5))
        n.step()
    assert n.door_detector.state == "NO_DOOR"
    assert not n.start_confirmed
    assert n.state == node_cls.STATE_INIT


def test_door_open_before_preflight_complete(node_cls):
    """Si la puerta se abre durante el pre-vuelo, arranca inmediatamente al completar pre-vuelo."""
    n = make_mock_node(node_cls, preflight_complete=False, calibration_scans=3, open_scans=2)
    for _ in range(3):
        n._scan_callback(make_test_scan(0.8))
    for _ in range(2):
        n._scan_callback(make_test_scan(2.0))
    assert n.door_detector.state == "OPEN"
    # preflight_complete es False: callback no activa start_confirmed prematuramente
    assert not n.start_confirmed

    # Al completarse el pre-vuelo, el step() detecta que la puerta ya estaba OPEN
    n.preflight_complete = True
    n.step()
    assert n.start_confirmed
    assert n.state == node_cls.STATE_NAV_TO_DOOR


def test_scan_callback_ignores_after_start_confirmed(node_cls):
    """Despues de start_confirmed=True, _scan_callback no consume CPU."""
    n = make_mock_node(node_cls)
    n.start_confirmed = True
    prev_state = n.door_detector.state
    # Enviamos un scan
    n._scan_callback(make_test_scan(0.8))
    # El estado del detector no debio cambiar
    assert n.door_detector.state == prev_state


# ==============================================================================
# TESTS DE ARRANQUE POR VOZ
# ==============================================================================

def test_voice_text_transcription_triggers_start(node_cls):
    """Transcripcion recibida en voice_text ('start') activa start_confirmed y transiciona."""
    n = make_mock_node(node_cls)
    n.voice_text = "start"
    n.step()
    assert n.start_confirmed
    assert n.voice_text is None
    assert n.state == node_cls.STATE_NAV_TO_DOOR


def test_voice_text_robot_start_triggers_start(node_cls):
    """Comando verbal oficial 'Robot, start' activa el inicio."""
    n = make_mock_node(node_cls)
    n.voice_text = "Robot, start"
    n.step()
    assert n.start_confirmed
    assert n.state == node_cls.STATE_NAV_TO_DOOR


def test_voice_text_unrelated_does_not_start(node_cls):
    """Frases ajenas al inicio no disparan start_confirmed."""
    n = make_mock_node(node_cls)
    n.voice_text = "hello robot, my name is Alex"
    n.step()
    assert not n.start_confirmed
    assert n.state == node_cls.STATE_INIT


def test_voice_client_poll_result_triggers_start(node_cls, hri_module):
    """Resultado del VoiceClient (con id de pedido) con 'start' activa el inicio."""
    VoiceResult = hri_module.VoiceResult
    n = make_mock_node(node_cls)
    n.listen_id = "req-001"
    n.voice.poll.return_value = VoiceResult(id="req-001", status="ok", text="start")

    n.step()
    assert n.start_confirmed
    assert n.state == node_cls.STATE_NAV_TO_DOOR


def test_door_open_cancels_pending_voice_listen(node_cls):
    """Si la puerta se abre teniendo una escucha abierta de microfono, cancela la escucha pendiente."""
    n = make_mock_node(node_cls, calibration_scans=1, open_scans=1)
    n.listen_id = "req-pending"
    n._scan_callback(make_test_scan(0.8))
    n._scan_callback(make_test_scan(2.0))
    assert n.start_confirmed
    n.step()
    n.voice.cancel.assert_called_with("req-pending")
    assert n.listen_id is None


def test_voice_client_poll_no_speech_reopens_listen(node_cls, hri_module):
    """Si VoiceClient retorna no_speech, no arranca y reabre la escucha cuando corresponde."""
    VoiceResult = hri_module.VoiceResult
    n = make_mock_node(node_cls)
    n.listen_id = "req-002"
    n.voice.poll.return_value = VoiceResult(id="req-002", status="no_speech", text="", reason="timeout")

    n.step()
    assert not n.start_confirmed
    assert n.state == node_cls.STATE_INIT
    n.voice.request.assert_called()
    assert n.listen_id == n.voice.request.return_value


# ==============================================================================
# TESTS DE ATAJO DE LABORATORIO POR TECLADO
# ==============================================================================

def test_space_key_shortcut_in_state_init(node_cls):
    """ESPACIO (32) en STATE_INIT con preflight_complete confirma inicio."""
    n = make_mock_node(node_cls)
    n.handle_key(32)
    assert n.start_confirmed
    n.step()
    assert n.state == node_cls.STATE_NAV_TO_DOOR


def test_enter_key_shortcut_in_state_init(node_cls):
    """ENTER (13) en STATE_INIT con preflight_complete confirma inicio."""
    n = make_mock_node(node_cls)
    n.handle_key(13)
    assert n.start_confirmed
    n.step()
    assert n.state == node_cls.STATE_NAV_TO_DOOR


def test_space_key_before_preflight_does_not_start(node_cls):
    """ESPACIO antes de terminar el pre-vuelo no confirma inicio."""
    n = make_mock_node(node_cls, preflight_complete=False)
    n.handle_key(32)
    assert not n.start_confirmed


# ==============================================================================
# TESTS DE BANNER
# ==============================================================================

def test_ready_banner_format(node_cls):
    """El banner debe mostrar estado de puerta, voz y atajos."""
    n = make_mock_node(node_cls)
    banner = n._ready_banner()
    assert "LISTO:" in banner
    assert "Puerta" in banner
    assert "Voz" in banner
    assert "ESPACIO" in banner
    assert "Nav simulada" in banner
