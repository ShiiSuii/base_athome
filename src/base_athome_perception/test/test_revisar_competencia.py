#!/usr/bin/env python3
"""Tests unitarios para el verificador de competencia (HOME-93).

Evalua funciones puras de logica sin requerir hardware de robot ni simulador.
Se ejecuta con:
    ROS_DOMAIN_ID=77 pytest src/base_athome_perception/test/test_revisar_competencia.py
"""

import importlib.util
import os
import tempfile
from pathlib import Path
from typing import List, Tuple

import pytest
import yaml

# Carga dinamica del script bajo prueba
SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "revisar_competencia.py"
spec = importlib.util.spec_from_file_location("revisar_competencia", SCRIPT_PATH)
rc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rc)


# ==============================================================================
# 1. PRUEBAS DE PARSERS
# ==============================================================================

def test_parse_names_markdown():
    sample = """
## Names
| Names |
| ------------ |
| Adel         |
| Angel        |
| Axel         |
| Charlie      |
"""
    names = rc.parse_names_markdown(sample)
    assert names == ["Adel", "Angel", "Axel", "Charlie"]


def test_parse_names_markdown_empty_and_headers():
    assert rc.parse_names_markdown("") == []
    assert rc.parse_names_markdown("| name |\n| --- |\n") == []


def test_parse_objects_markdown():
    sample = """
# Class drinks (drink)

| Objectname               |  Image                   |
:-------------------------:|:-------------------------:
| juice_pack | ![](known_objects/drinks!drink/juice_pack.jpg) |
| cola | ![](known_objects/drinks!drink/cola.jpg) |

# Class toys (toy)

| Objectname               |  Image                   |
:-------------------------:|:-------------------------:
| tennis_ball | ![](known_objects/toys!toy/tennis_ball.png) |
"""
    objs = rc.parse_objects_markdown(sample)
    assert "drinks" in objs
    assert objs["drinks"] == ["juice_pack", "cola"]
    assert "toys" in objs
    assert objs["toys"] == ["tennis_ball"]


def test_parse_objects_markdown_empty():
    assert rc.parse_objects_markdown("") == {}


def test_parse_pactl_sinks():
    sample = """
479	alsa_output.pci-0000_00_1f.3.hdmi-stereo	PipeWire	s32le 2ch 48000Hz	SUSPENDED
480	bluez_output.00_11_22_33_44_55.1	PipeWire	s16le 2ch 44100Hz	RUNNING
"""
    sinks = rc.parse_pactl_sinks(sample)
    assert len(sinks) == 2
    assert sinks[0] == "alsa_output.pci-0000_00_1f.3.hdmi-stereo"
    assert sinks[1] == "bluez_output.00_11_22_33_44_55.1"


def test_parse_pactl_sinks_empty():
    assert rc.parse_pactl_sinks("") == []


# ==============================================================================
# 2. PRUEBAS DE EVALUACION DE MAPA
# ==============================================================================

def test_evaluar_mapa_valid(tmp_path):
    yaml_file = tmp_path / "mapa.yaml"
    pgm_file = tmp_path / "mapa.pgm"

    yaml_file.write_text(yaml.dump({"resolution": 0.05, "origin": [-3.0, -9.0, 0.0], "image": "mapa.pgm"}))
    pgm_file.write_bytes(b"P5\n10 10\n255\n" + b"\x00" * 2000)

    res = rc.evaluar_mapa(str(yaml_file), str(pgm_file), live_map_seen=True)
    assert all(ok for ok, _, _ in res)
    assert any("0.05" in label for _, label, _ in res)
    assert any("topico /map activo" in label for _, label, _ in res)


def test_evaluar_mapa_missing_files(tmp_path):
    missing_yaml = str(tmp_path / "inexistente.yaml")
    missing_pgm = str(tmp_path / "inexistente.pgm")

    res = rc.evaluar_mapa(missing_yaml, missing_pgm, live_map_seen=False)
    assert not any(ok for ok, _, _ in res)
    assert any("guardar mapa" in fix for _, _, fix in res)
    assert any("nav_map_sync" in fix for _, _, fix in res)


def test_evaluar_mapa_malformed_yaml(tmp_path):
    yaml_file = tmp_path / "corrupto.yaml"
    pgm_file = tmp_path / "mapa.pgm"
    yaml_file.write_text("algo_invalido: true\n")
    pgm_file.write_bytes(b"P5\n10 10\n255\n" + b"\x00" * 2000)

    res = rc.evaluar_mapa(str(yaml_file), str(pgm_file))
    yaml_check = [r for r in res if "archivo YAML" in r[1]][0]
    assert not yaml_check[0]
    assert "falta resolution u origin" in yaml_check[2]


# ==============================================================================
# 3. PRUEBAS DE EVALUACION DE POSES DEL BRAZO
# ==============================================================================

def test_evaluar_poses_complete_and_valid():
    cfg = {
        "safety_envelope": {
            "joint_limits_deg": {
                "joint1": [-154.0, 154.0],
                "joint2": [0.0, 195.0],
                "joint3": [-175.0, 0.0],
                "joint4": [-110.0, 110.0],
                "joint5": [-80.0, 80.0],
                "joint6": [-180.0, 180.0],
            }
        },
        "named_poses": {
            "home": {"joints_deg": [73.87, 169.72, -138.02, 26.69, -15.7, -117.17]},
            "pre_grasp": {"joints_deg": [48.06, 122.69, -51.16, 62.39, -61.1, -111.96]},
            "post_grasp": {"joints_deg": [-21.72, 99.31, -33.18, -7.58, 31.57, -102.11]},
            "pre_grasp_floor": {"joints_deg": [45.87, 153.29, -80.81, 15.96, -56.47, -103.98]},
        },
    }
    res = rc.evaluar_poses(cfg)
    assert len(res) == 4
    assert all(ok for ok, _, _ in res)


def test_evaluar_poses_missing_pose():
    cfg = {
        "safety_envelope": {"joint_limits_deg": {}},
        "named_poses": {
            "home": {"joints_deg": [0, 0, 0, 0, 0, 0]},
            "pre_grasp": {"joints_deg": [0, 0, 0, 0, 0, 0]},
            # Faltan post_grasp y pre_grasp_floor
        },
    }
    res = rc.evaluar_poses(cfg)
    missing = [r for r in res if not r[0]]
    assert len(missing) == 2
    assert any("post_grasp" in r[1] for r in missing)
    assert any("pre_grasp_floor" in r[1] for r in missing)


def test_evaluar_poses_joint_limit_violation():
    cfg = {
        "safety_envelope": {
            "joint_limits_deg": {
                "joint2": [0.0, 195.0],
            }
        },
        "named_poses": {
            "home": {"joints_deg": [0.0, 210.0, 0.0, 0.0, 0.0, 0.0]},  # joint2 excede 195
            "pre_grasp": {"joints_deg": [0, 0, 0, 0, 0, 0]},
            "post_grasp": {"joints_deg": [0, 0, 0, 0, 0, 0]},
            "pre_grasp_floor": {"joints_deg": [0, 0, 0, 0, 0, 0]},
        },
    }
    res = rc.evaluar_poses(cfg)
    home_check = [r for r in res if "home" in r[1]][0]
    assert not home_check[0]
    assert "joint2" in home_check[2]
    assert "fuera de" in home_check[2]


def test_evaluar_poses_wrong_joint_count():
    cfg = {
        "named_poses": {
            "home": {"joints_deg": [0.0, 10.0, 20.0]},  # solo 3 articulaciones
            "pre_grasp": {"joints_deg": [0, 0, 0, 0, 0, 0]},
            "post_grasp": {"joints_deg": [0, 0, 0, 0, 0, 0]},
            "pre_grasp_floor": {"joints_deg": [0, 0, 0, 0, 0, 0]},
        }
    }
    res = rc.evaluar_poses(cfg)
    home_check = [r for r in res if "home" in r[1]][0]
    assert not home_check[0]
    assert "3 articulaciones" in home_check[1]


# ==============================================================================
# 4. PRUEBAS DE EVALUACION DE CAMARAS Y MODELOS DE VISION
# ==============================================================================

def test_evaluar_camaras_all_ok(tmp_path):
    seen = {
        "/camera/camera/color/image_raw": 1.0,
        "/camera/camera/aligned_depth_to_color/image_raw": 1.0,
        "/camera/camera/color/camera_info": 1.0,
        "/arm_camera/image_raw": 1.0,
    }
    # Crear pesos sinteticos > 1MB
    w1 = tmp_path / "yolov8s-worldv2.pt"
    w2 = tmp_path / "yolov8n-pose.pt"
    w1.write_bytes(b"\x00" * 1500000)
    w2.write_bytes(b"\x00" * 1500000)

    res = rc.evaluar_camaras(seen_topics=seen, models_dir=str(tmp_path))
    assert all(ok for ok, _, _ in res)


def test_evaluar_camaras_missing_topics_and_weights(tmp_path):
    seen = {}
    res = rc.evaluar_camaras(seen_topics=seen, models_dir=str(tmp_path))
    assert not any(ok for ok, _, _ in res)
    assert any("reconectar RealSense" in fix for _, _, fix in res)
    assert any("revisar la DaBai" in fix for _, _, fix in res)
    assert any("download_models.sh" in fix for _, _, fix in res)


# ==============================================================================
# 5. PRUEBAS DE AUDIO, TTS Y PARLANTE
# ==============================================================================

def test_evaluar_audio_tts_ok(tmp_path):
    piper_bin = tmp_path / "piper"
    piper_bin.write_bytes(b"#!/bin/sh\nexit 0\n")
    piper_bin.chmod(0o755)

    piper_model = tmp_path / "model.onnx"
    piper_model.write_bytes(b"\x00" * 2000000)

    beep_file = tmp_path / "beep.wav"
    beep_file.write_bytes(b"\x00" * 1000)

    sinks = ["alsa_output.pci-0000_00_1f.3.hdmi-stereo"]
    res = rc.evaluar_audio_tts(str(piper_bin), str(piper_model), str(beep_file), pactl_sinks=sinks)
    assert all(ok for ok, _, _ in res)


def test_evaluar_audio_tts_missing_components(tmp_path):
    res = rc.evaluar_audio_tts(
        piper_bin=str(tmp_path / "no_piper"),
        model_path=str(tmp_path / "no_model.onnx"),
        beep_path=str(tmp_path / "no_beep.wav"),
        pactl_sinks=[],
    )
    assert not any(ok for ok, _, _ in res)
    assert any("instalar binario de piper" in fix for _, _, fix in res)
    assert any("descargar modelo ONNX" in fix for _, _, fix in res)
    assert any("pactl list sinks" in fix for _, _, fix in res)


def test_evaluar_audio_tts_bluetooth_warning_rule(tmp_path):
    piper_bin = tmp_path / "piper"
    piper_bin.write_bytes(b"bin")
    piper_bin.chmod(0o755)
    piper_model = tmp_path / "model.onnx"
    piper_model.write_bytes(b"\x00" * 1500000)
    beep_file = tmp_path / "beep.wav"
    beep_file.write_bytes(b"\x00" * 1000)

    # Solo hay sink Bluetooth: incumple Setup.tex:112
    sinks = ["bluez_output.00_11_22_33_44_55.1"]
    res = rc.evaluar_audio_tts(str(piper_bin), str(piper_model), str(beep_file), pactl_sinks=sinks)
    sink_check = [r for r in res if "Setup.tex:112" in r[1]][0]
    assert not sink_check[0]
    assert "jack 3.5 mm" in sink_check[2]


# ==============================================================================
# 6. PRUEBAS DE MICROFONO Y ASR
# ==============================================================================

def test_evaluar_microfono_asr_ok(tmp_path):
    names_file = tmp_path / "names.md"
    names_file.write_text("## Names\n| Names |\n| --- |\n| Jane |\n| Robin |\n")

    seen = {"/voice/result": 1.0}
    res = rc.evaluar_microfono_asr(asr_status="IDLE", topics_seen=seen, names_path=str(names_file))
    assert all(ok for ok, _, _ in res)
    assert any("IDLE" in label for _, label, _ in res)


def test_evaluar_microfono_asr_no_mic(tmp_path):
    names_file = tmp_path / "names.md"
    names_file.write_text("## Names\n| Names |\n| --- |\n| Jane |\n")

    res = rc.evaluar_microfono_asr(asr_status="NO_MIC", topics_seen={}, names_path=str(names_file))
    mic_check = [r for r in res if "microfono conectado" in r[1]][0]
    assert not mic_check[0]
    assert "conectar microfono" in mic_check[2]


def test_evaluar_microfono_asr_offline(tmp_path):
    names_file = tmp_path / "names.md"
    names_file.write_text("## Names\n| Names |\n| --- |\n| Jane |\n")

    res = rc.evaluar_microfono_asr(asr_status=None, topics_seen={}, names_path=str(names_file))
    status_check = [r for r in res if "/voice/status" in r[1]][0]
    assert not status_check[0]
    assert "jetson-head.service" in status_check[2]


def test_evaluar_microfono_asr_missing_names(tmp_path):
    res = rc.evaluar_microfono_asr(asr_status="IDLE", topics_seen={}, names_path=str(tmp_path / "vacio.md"))
    names_check = [r for r in res if "nombres oficiales" in r[1]][0]
    assert not names_check[0]


# ==============================================================================
# 7. PRUEBAS DE LUGARES SEMANTICOS Y OBJETOS
# ==============================================================================

def test_evaluar_lugares_complete():
    sem_data = {
        "zones": {"living_room": {}, "entrance": {}},
        "waypoints": {"living_room": {"x": 1.0, "y": 2.0}},
    }
    wps_data = {
        "waypoints": {
            "door": {"x": 0.0, "y": 0.0},
            "exit": {"x": 5.0, "y": 5.0},
            "kitchen": {"x": 3.0, "y": 4.0},
        }
    }
    res = rc.evaluar_lugares(sem_data, wps_data)
    assert all(ok for ok, _, _ in res)
    assert any("door" in label for _, label, _ in res)
    assert any("exit" in label for _, label, _ in res)
    assert any("4 ubicaciones" in label for _, label, _ in res)


def test_evaluar_lugares_missing_door_and_exit():
    sem_data = {"zones": {}}
    wps_data = {"waypoints": {"room_a": {}, "room_b": {}}}
    res = rc.evaluar_lugares(sem_data, wps_data)
    door_check = [r for r in res if "door" in r[1]][0]
    exit_check = [r for r in res if "exit" in r[1]][0]
    assert not door_check[0]
    assert not exit_check[0]
    assert any("marcar waypoint 'door'" in door_check[2] for _ in [1])
    assert any("marcar waypoint 'exit'" in exit_check[2] for _ in [1])


def test_evaluar_objetos_valid(tmp_path):
    objs_file = tmp_path / "objects.md"
    objs_file.write_text("""
# Class drinks (drink)
| Objectname | Image |
| cola | img |
| milk | img |

# Class snacks (snack)
| Objectname | Image |
| chips | img |
""")
    res = rc.evaluar_objetos(str(objs_file))
    assert all(ok for ok, _, _ in res)
    assert any("3 objetos en 2 categorias" in label for _, label, _ in res)


def test_evaluar_objetos_missing(tmp_path):
    missing_file = str(tmp_path / "no_existe.md")
    res = rc.evaluar_objetos(missing_file)
    assert not res[0][0]
    assert "cargar objetos del TC" in res[0][2]


# ==============================================================================
# 8. PRUEBAS DE TOPICOS Y HARDWARE EN VIVO
# ==============================================================================

def test_evaluar_topicos_y_hardware_all_good():
    seen = {topic: 1.0 for topic, _, _, _ in rc.TOPICS}
    tfs = {(parent, child): True for parent, child, _, _ in rc.FRAMES}
    arm_st = {"feedback_ok": True}
    torso_st = {"connected": True, "calibrated": True}

    res = rc.evaluar_topicos_y_hardware(
        seen_topics=seen,
        tf_status=tfs,
        arm_status=arm_st,
        torso_status=torso_st,
        amcl_received=True,
    )
    assert all(ok for ok, _, _ in res)


def test_evaluar_topicos_y_hardware_failures():
    seen = {}
    tfs = {}
    arm_st = {"feedback_ok": False}
    torso_st = {"connected": False, "calibrated": False}

    res = rc.evaluar_topicos_y_hardware(
        seen_topics=seen,
        tf_status=tfs,
        arm_status=arm_st,
        torso_status=torso_st,
        amcl_received=False,
    )
    assert not any(ok for ok, _, _ in res)
    assert any("revisar el Hokuyo" in fix for _, _, fix in res)
    assert any("energizar el Piper" in fix for _, _, fix in res)
    assert any("calibrar_elevador" in fix for _, _, fix in res)
    assert any("dar la pose inicial en RViz" in fix for _, _, fix in res)


# ==============================================================================
# 9. PRUEBAS DE LISTA MANUAL Y INTEGRACION
# ==============================================================================

def test_obtener_checklist_manual_structure():
    items = rc.obtener_checklist_manual()
    assert len(items) >= 7

    codes = [item["code"] for item in items]
    assert "E-STOP" in codes
    assert "DIMENSIONES" in codes
    assert "INICIO_MANUAL" in codes
    assert "APARIENCIA" in codes
    assert "RED_ARENANETWORK" in codes
    assert "SALIDA_AUDIO_RF" in codes
    assert "ALIMENTACION" in codes

    for item in items:
        assert item["rule"]
        assert item["title"]
        assert item["desc"]
        assert item["cmd"]
        assert len(item["cmd"].splitlines()) == 1  # Un comando simple de una sola linea


def test_check_files_integration_with_temp_environment(tmp_path):
    # Crear un entorno sintetico completo para verificar la ejecucion sin excepciones
    map_yaml = tmp_path / "map.yaml"
    map_yaml.write_text(yaml.dump({"resolution": 0.05, "origin": [0, 0, 0], "image": "map.pgm"}))
    map_pgm = tmp_path / "map.pgm"
    map_pgm.write_bytes(b"P5\n10 10\n255\n" + b"\x00" * 2000)

    arm_cfg = tmp_path / "arm.yaml"
    arm_cfg.write_text(yaml.dump({
        "safety_envelope": {"joint_limits_deg": {"joint1": [-154, 154], "joint2": [0, 195]}},
        "named_poses": {
            "home": {"joints_deg": [0, 10, 0, 0, 0, 0]},
            "pre_grasp": {"joints_deg": [0, 10, 0, 0, 0, 0]},
            "post_grasp": {"joints_deg": [0, 10, 0, 0, 0, 0]},
            "pre_grasp_floor": {"joints_deg": [0, 10, 0, 0, 0, 0]},
        }
    }))

    sem_map = tmp_path / "sem.yaml"
    sem_map.write_text(yaml.dump({"zones": {"kitchen": {}}, "waypoints": {"door": {"x": 0, "y": 0}}}))

    wps = tmp_path / "wps.yaml"
    wps.write_text(yaml.dump({"waypoints": {"exit": {"x": 1, "y": 1}, "living_room": {"x": 2, "y": 2}}}))

    names = tmp_path / "names.md"
    names.write_text("| names |\n| --- |\n| Alex |\n")

    objs = tmp_path / "objects.md"
    objs.write_text("# Class food\n| apple |\n")

    models_dir = tmp_path / "models"
    models_dir.mkdir()
    (models_dir / "yolov8s-worldv2.pt").write_bytes(b"\x00" * 1500000)
    (models_dir / "yolov8n-pose.pt").write_bytes(b"\x00" * 1500000)

    piper_bin = tmp_path / "piper"
    piper_bin.write_bytes(b"bin")
    piper_bin.chmod(0o755)
    piper_model = tmp_path / "voice.onnx"
    piper_model.write_bytes(b"\x00" * 1500000)
    beep = tmp_path / "beep.wav"
    beep.write_bytes(b"\x00" * 1000)

    custom = {
        "map_yaml": str(map_yaml),
        "map_pgm": str(map_pgm),
        "arm_config": str(arm_cfg),
        "semantic_map": str(sem_map),
        "waypoints": str(wps),
        "objects": str(objs),
        "names": str(names),
        "piper_bin": str(piper_bin),
        "piper_model": str(piper_model),
        "beep": str(beep),
        "models_dir": str(models_dir),
    }

    records: List[Tuple[bool, str, str]] = []
    rc.check_files(lambda ok, label, fix: records.append((ok, label, fix)), custom_paths=custom)
    assert len(records) > 0
    # En este entorno sintetico, todo debe ser OK
    assert all(ok for ok, _, _ in records)
