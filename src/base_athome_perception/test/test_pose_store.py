"""Tests unitarios offline para pose_store."""

import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from base_athome_perception.pose_store import (  # noqa: E402
    FALLBACK_JOINT_LIMITS,
    get_named_pose,
    load_joint_limits,
    load_named_poses,
    load_pose_config,
    save_named_pose,
    validate_joints,
)

SAMPLE_YAML_CONTENT = """
safety_envelope:
  joint_limits_deg:
    joint1: [-154.0, 154.0]
    joint2: [0.0, 195.0]
    joint3: [-175.0, 0.0]
    joint4: [-110.0, 110.0]
    joint5: [-80.0, 80.0]
    joint6: [-180.0, 180.0]
  cartesian_limits_m:
    x: [0.15, 0.72]
    y: [-0.48, 0.35]
    z: [-0.48, 0.35]
  grasp_forward_cm: 35.0

named_poses:
  home:
    joints_deg: [86.97, 172.87, -162.63, -99.89, -0.68, 44.98]
    gripper_mm: 24.6
    timestamp: '2026-10-30 14:07:00'
  pre_grasp:
    joints_deg: [58.1, 169.44, -68.31, 68.15, -74.94, -79.41]
    gripper_mm: 105.0
    timestamp: '2026-10-27 15:27:12'
"""


@pytest.fixture
def temp_yaml_file(tmp_path: Path) -> Path:
    """Crea un archivo arm_poses_and_limits.yaml temporal para pruebas."""
    file_path = tmp_path / "arm_poses_and_limits.yaml"
    file_path.write_text(SAMPLE_YAML_CONTENT.strip(), encoding="utf-8")
    return file_path


def test_load_pose_config(temp_yaml_file: Path):
    """Verifica que la configuracion YAML completa se cargue correctamente."""
    data = load_pose_config(temp_yaml_file)
    assert "safety_envelope" in data
    assert "named_poses" in data
    assert data["safety_envelope"]["grasp_forward_cm"] == 35.0


def test_load_named_poses(temp_yaml_file: Path):
    """Verifica la carga especifica del bloque named_poses."""
    poses = load_named_poses(temp_yaml_file)
    assert "home" in poses
    assert "pre_grasp" in poses
    assert poses["home"]["gripper_mm"] == 24.6
    assert len(poses["home"]["joints_deg"]) == 6


def test_get_named_pose(temp_yaml_file: Path):
    """Verifica obtencion de pose individual o None si no existe."""
    home = get_named_pose("home", path=temp_yaml_file)
    assert home is not None
    assert home["gripper_mm"] == 24.6

    missing = get_named_pose("non_existent", path=temp_yaml_file)
    assert missing is None


def test_load_joint_limits(temp_yaml_file: Path):
    """Verifica la extraccion de limites articulares."""
    limits = load_joint_limits(temp_yaml_file)
    assert limits["joint1"] == (-154.0, 154.0)
    assert limits["joint2"] == (0.0, 195.0)
    assert limits["joint3"] == (-175.0, 0.0)

    # Fallback si el archivo no existe
    fallback = load_joint_limits(Path("/tmp/non_existent_file_limits.yaml"))
    assert fallback == FALLBACK_JOINT_LIMITS


def test_validate_joints_valid():
    """Valida configuraciones articulares dentro de los limites permitidos."""
    valid_joints = [86.97, 172.87, -162.63, -99.89, -0.68, 44.98]
    res = validate_joints(valid_joints, limits=FALLBACK_JOINT_LIMITS)
    assert res.valid is True
    assert bool(res) is True
    assert res.error is None


def test_validate_joints_wrong_length():
    """Detecta error si la cantidad de articulaciones es distinta de 6."""
    res = validate_joints([10.0, 20.0, 30.0], limits=FALLBACK_JOINT_LIMITS)
    assert res.valid is False
    assert "exactamente 6" in res.error

    res_none = validate_joints(None)
    assert res_none.valid is False


def test_validate_joints_out_of_limits():
    """Detecta violacion de limites articulares y devuelve error estructurado."""
    # joint2 maximo es 195.0, probamos con 200.0
    bad_joints = [80.0, 200.0, -100.0, 0.0, 0.0, 0.0]
    res = validate_joints(bad_joints, limits=FALLBACK_JOINT_LIMITS)
    assert res.valid is False
    assert res.joint == "joint2"
    assert res.joint_index == 1
    assert res.value == 200.0
    assert res.limits == (0.0, 195.0)
    assert "joint2 fuera de limites" in res.error

    # joint3 limites son [-175.0, 0.0], probamos con +10.0
    bad_joint3 = [0.0, 50.0, 10.0, 0.0, 0.0, 0.0]
    res3 = validate_joints(bad_joint3, limits=FALLBACK_JOINT_LIMITS)
    assert res3.valid is False
    assert res3.joint == "joint3"


def test_save_named_pose_rejects_out_of_bounds(temp_yaml_file: Path):
    """Si una articulacion viola los limites, no guarda ni altera el archivo."""
    bad_joints = [0.0, 220.0, 0.0, 0.0, 0.0, 0.0]  # joint2 > 195.0
    res = save_named_pose("test_pose", bad_joints, path=temp_yaml_file)

    assert not res.valid
    assert "joint2" in res.error

    # Asegurar que el archivo no fue modificado con la pose invalida
    poses = load_named_poses(temp_yaml_file)
    assert "test_pose" not in poses


def test_save_named_pose_creates_previous_and_preserves_safety(
    temp_yaml_file: Path,
):
    """Sobrescribir una pose guarda la version anterior como <nombre>_previous y preserva safety_envelope."""
    initial_home = get_named_pose("home", path=temp_yaml_file)
    assert initial_home is not None

    new_joints = [85.0, 170.0, -160.0, -95.0, 0.0, 40.0]
    new_gripper = 55.5
    res = save_named_pose(
        "home",
        new_joints,
        gripper_mm=new_gripper,
        path=temp_yaml_file,
        timestamp="2026-10-30T15:00:00",
    )

    assert res.valid is True
    assert res.details["previous_saved"] == "home_previous"

    # Verificar contenido final
    cfg = load_pose_config(temp_yaml_file)

    # 1. safety_envelope se mantuvo integro
    assert "safety_envelope" in cfg
    assert cfg["safety_envelope"]["grasp_forward_cm"] == 35.0
    assert cfg["safety_envelope"]["joint_limits_deg"]["joint1"] == [-154.0, 154.0]

    # 2. La nueva pose esta guardada
    home = cfg["named_poses"]["home"]
    assert home["joints_deg"] == new_joints
    assert home["gripper_mm"] == 55.5
    assert home["timestamp"] == "2026-10-30T15:00:00"

    # 3. La pose previa existe con los datos originales
    prev = cfg["named_poses"]["home_previous"]
    assert prev["joints_deg"] == initial_home["joints_deg"]
    assert prev["gripper_mm"] == initial_home["gripper_mm"]


def test_save_named_pose_atomic_and_no_tmp_leak(tmp_path: Path):
    """Verifica que la escritura sea atomica y no deje archivos temporales colgados."""
    yaml_path = tmp_path / "fresh_poses.yaml"
    joints = [0.0, 100.0, -50.0, 0.0, 0.0, 0.0]

    res = save_named_pose("post_grasp", joints, gripper_mm=30.0, path=yaml_path)
    assert res.valid is True
    assert yaml_path.exists()

    # Verificar que no queden archivos .tmp en el directorio
    tmp_files = list(tmp_path.glob("*.tmp"))
    assert len(tmp_files) == 0

    # Cargar y verificar
    data = load_named_poses(yaml_path)
    assert "post_grasp" in data
    assert data["post_grasp"]["gripper_mm"] == 30.0
