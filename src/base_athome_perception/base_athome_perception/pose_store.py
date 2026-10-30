"""Almacenamiento y validacion de poses del brazo en formato YAML.

Funciones puras para leer y escribir named_poses preservando el resto de la
configuracion (safety_envelope, cartesian_limits_m, etc.), validando limites
articulares, versionando poses previas con el sufijo `_previous` y garantizando
escritura atomica en disco.
"""

from __future__ import annotations

import os
import tempfile
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import yaml

DEFAULT_CONFIG_PATH = (
    Path(__file__).resolve().parent.parent / "config" / "arm_poses_and_limits.yaml"
)

# Limites de reserva en caso de que no se encuentre el archivo de configuracion
FALLBACK_JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "joint1": (-154.0, 154.0),
    "joint2": (0.0, 195.0),
    "joint3": (-175.0, 0.0),
    "joint4": (-110.0, 110.0),
    "joint5": (-80.0, 80.0),
    "joint6": (-180.0, 180.0),
}


@dataclass
class ValidationResult:
    """Resultado estructurado de validacion o guardado de pose."""

    valid: bool
    error: Optional[str] = None
    joint: Optional[str] = None
    joint_index: Optional[int] = None
    value: Optional[float] = None
    limits: Optional[Tuple[float, float]] = None
    details: Dict[str, Any] = field(default_factory=dict)

    def __bool__(self) -> bool:
        return self.valid

    def __getitem__(self, item: str) -> Any:
        return getattr(self, item)

    def get(self, item: str, default: Any = None) -> Any:
        return getattr(self, item, default)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def load_pose_config(path: Union[str, Path, None] = None) -> Dict[str, Any]:
    """Carga el archivo completo YAML de configuracion de poses y limites.

    Si el archivo no existe o esta vacio, devuelve un diccionario vacio.
    """
    cfg_path = Path(path or DEFAULT_CONFIG_PATH)
    if not cfg_path.exists():
        return {}

    with open(cfg_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        return {}
    return data


def load_named_poses(
    path: Union[str, Path, None] = None,
) -> Dict[str, Dict[str, Any]]:
    """Devuelve el diccionario de poses nombradas registradas."""
    config = load_pose_config(path)
    named = config.get("named_poses", {})
    if isinstance(named, dict):
        return named
    return {}


def get_named_pose(
    name: str, path: Union[str, Path, None] = None
) -> Optional[Dict[str, Any]]:
    """Devuelve la pose nombrada o None si no existe."""
    poses = load_named_poses(path)
    pose = poses.get(name)
    if isinstance(pose, dict):
        return pose
    return None


def load_joint_limits(
    path: Union[str, Path, None] = None,
) -> Dict[str, Tuple[float, float]]:
    """Carga los limites articulares en grados desde safety_envelope."""
    config = load_pose_config(path)
    raw_limits = (
        config.get("safety_envelope", {}).get("joint_limits_deg")
        if isinstance(config.get("safety_envelope"), dict)
        else None
    )

    limits: Dict[str, Tuple[float, float]] = {}
    if isinstance(raw_limits, dict):
        for i in range(1, 7):
            k = f"joint{i}"
            v = raw_limits.get(k)
            if (
                isinstance(v, (list, tuple))
                and len(v) >= 2
                and isinstance(v[0], (int, float))
                and isinstance(v[1], (int, float))
            ):
                limits[k] = (float(v[0]), float(v[1]))
            else:
                limits[k] = FALLBACK_JOINT_LIMITS[k]
        return limits

    return dict(FALLBACK_JOINT_LIMITS)


def validate_joints(
    joints_deg: Sequence[Union[int, float]],
    limits: Optional[Dict[str, Tuple[float, float]]] = None,
    path: Union[str, Path, None] = None,
) -> ValidationResult:
    """Valida que haya exactamente 6 articulaciones y que esten en rango.

    Devuelve un ValidationResult estructurado.
    """
    if joints_deg is None:
        return ValidationResult(
            valid=False,
            error="La lista de articulaciones no puede ser None.",
        )

    try:
        joints_list: List[float] = [float(j) for j in joints_deg]
    except (TypeError, ValueError) as exc:
        return ValidationResult(
            valid=False,
            error=f"Valores de articulacion no numericos: {exc}",
        )

    if len(joints_list) != 6:
        return ValidationResult(
            valid=False,
            error=(
                f"Se requieren exactamente 6 articulaciones, se recibieron "
                f"{len(joints_list)}."
            ),
            details={"received_count": len(joints_list)},
        )

    if limits is None:
        limits = load_joint_limits(path)

    for idx, val in enumerate(joints_list):
        joint_key = f"joint{idx + 1}"
        joint_range = limits.get(joint_key, FALLBACK_JOINT_LIMITS.get(joint_key))
        if joint_range is None:
            continue
        min_deg, max_deg = joint_range
        # Permitimos una minima tolerancia de 1e-4 para evitar errores de precision float
        if val < (min_deg - 1e-4) or val > (max_deg + 1e-4):
            err_msg = (
                f"{joint_key} fuera de limites: valor {val:.2f} deg excede "
                f"[{min_deg:.1f}, {max_deg:.1f}] deg."
            )
            return ValidationResult(
                valid=False,
                error=err_msg,
                joint=joint_key,
                joint_index=idx,
                value=val,
                limits=joint_range,
                details={
                    "joint": joint_key,
                    "joint_index": idx,
                    "value": val,
                    "min": min_deg,
                    "max": max_deg,
                },
            )

    return ValidationResult(valid=True)


def save_named_pose(
    name: str,
    joints_deg: Sequence[Union[int, float]],
    gripper_mm: Union[int, float] = 0.0,
    path: Union[str, Path, None] = None,
    timestamp: Optional[str] = None,
    source: Optional[str] = None,
) -> ValidationResult:
    """Guarda una pose nombrada en el archivo YAML de forma atomica y validada.

    - Valida que joints_deg este dentro de joint_limits_deg.
    - Si la pose ya existia, la respalda como `<nombre>_previous`.
    - Genera timestamp ISO si no se especifica.
    - Preserva intactos los demas campos del archivo (safety_envelope, etc.).
    - Escribe a un archivo temporal en el mismo directorio y reemplaza con os.replace.
    """
    clean_name = str(name).strip()
    if not clean_name:
        return ValidationResult(
            valid=False,
            error="El nombre de la pose no puede estar vacio.",
        )

    target_path = Path(path or DEFAULT_CONFIG_PATH).resolve()

    # Cargar configuracion actual
    config = load_pose_config(target_path)
    if not config:
        config = {
            "safety_envelope": {
                "joint_limits_deg": {
                    k: [v[0], v[1]] for k, v in FALLBACK_JOINT_LIMITS.items()
                }
            },
            "named_poses": {},
        }

    # Validar articulaciones
    limits = load_joint_limits(target_path)
    val_res = validate_joints(joints_deg, limits=limits)
    if not val_res.valid:
        return val_res

    # Asegurar bloque named_poses
    named_poses = config.setdefault("named_poses", {})
    if not isinstance(named_poses, dict):
        named_poses = {}
        config["named_poses"] = named_poses

    # Respaldar pose previa si existia
    previous_saved: Optional[str] = None
    if clean_name in named_poses and isinstance(named_poses[clean_name], dict):
        prev_key = f"{clean_name}_previous"
        named_poses[prev_key] = dict(named_poses[clean_name])
        previous_saved = prev_key

    # Formatear entrada
    iso_ts = timestamp or datetime.now().isoformat(timespec="seconds")
    new_entry: Dict[str, Any] = {
        "joints_deg": [round(float(j), 2) for j in joints_deg],
        "gripper_mm": round(float(gripper_mm), 1),
        "timestamp": iso_ts,
    }
    if source is not None:
        new_entry["source"] = source

    named_poses[clean_name] = new_entry

    # Escritura atomica
    target_path.parent.mkdir(parents=True, exist_ok=True)
    temp_dir = target_path.parent
    fd, tmp_file = tempfile.mkstemp(
        dir=temp_dir, prefix="arm_poses_", suffix=".yaml.tmp"
    )
    try:
        with open(fd, "w", encoding="utf-8") as f:
            yaml.safe_dump(config, f, sort_keys=False, allow_unicode=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_file, str(target_path))
    finally:
        if os.path.exists(tmp_file):
            try:
                os.remove(tmp_file)
            except OSError:
                pass

    return ValidationResult(
        valid=True,
        details={
            "name": clean_name,
            "timestamp": iso_ts,
            "previous_saved": previous_saved,
            "path": str(target_path),
        },
    )
