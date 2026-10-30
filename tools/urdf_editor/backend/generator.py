"""Validacion de parametros, generacion de URDF en copia temporal y exportacion de candidatos.

Nada de este modulo escribe dentro de `src/`: el preview trabaja sobre una copia en
un directorio temporal y el export solo escribe en `candidates/`.
"""
from __future__ import annotations

import difflib
import math
import os
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from datetime import datetime, timezone
from pathlib import Path

from . import xacro_props
from .settings import Settings

_LABEL_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
_PKG_RE = re.compile(r"^package://([A-Za-z0-9_]+)/(.+)$")


class ParamError(ValueError):
    """Error de validacion de entrada (se traduce a HTTP 4xx)."""

    def __init__(self, errors: list[str]):
        super().__init__("; ".join(errors))
        self.errors = errors


class GenerationError(RuntimeError):
    """xacro/check_urdf fallaron o no estan disponibles."""


# ---------------------------------------------------------------- parametros

def list_params(settings: Settings) -> list[dict]:
    schema = settings.load_schema()
    props = xacro_props.parse_props(settings.props_path.read_text(encoding="utf-8"))
    out = []
    for p in props.values():
        spec = schema["editable"].get(p.name)
        editable = spec is not None and p.kind == "numeric"
        item = {
            "name": p.name,
            "raw": p.raw,
            "kind": p.kind,
            "value": p.number,
            "editable": editable,
            "protected": p.name in schema["protected"],
        }
        if p.name in schema["protected"]:
            item["protected_reason"] = schema["protected"][p.name]
        if editable:
            item.update({k: spec[k] for k in ("group", "label", "unit", "min", "max", "step")})
            if "warning" in spec:
                item["warning"] = spec["warning"]
        out.append(item)
    return out


def validate_values(settings: Settings, values: object) -> dict[str, float]:
    """Valida el mapa nombre->numero recibido del navegador. Nunca evalua expresiones."""
    if not isinstance(values, dict):
        raise ParamError(["'values' debe ser un objeto {nombre: numero}"])
    schema = settings.load_schema()
    props = xacro_props.parse_props(settings.props_path.read_text(encoding="utf-8"))
    errors: list[str] = []
    clean: dict[str, float] = {}
    for name, value in values.items():
        if name in schema["protected"]:
            errors.append(f"{name}: propiedad protegida, no editable desde el editor")
            continue
        spec = schema["editable"].get(name)
        if spec is None:
            errors.append(f"{name}: propiedad no incluida en la whitelist editable")
            continue
        prop = props.get(name)
        if prop is None or prop.kind != "numeric":
            errors.append(f"{name}: no es una propiedad numerica en properties.xacro")
            continue
        # bool es subclase de int: rechazarlo explicitamente. Strings tampoco se aceptan.
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            errors.append(f"{name}: el valor debe ser numerico (recibido {type(value).__name__})")
            continue
        value = float(value)
        if not math.isfinite(value):
            errors.append(f"{name}: el valor debe ser finito")
            continue
        if not (spec["min"] <= value <= spec["max"]):
            errors.append(f"{name}: {value} fuera de rango [{spec['min']}, {spec['max']}] {spec['unit']}")
            continue
        clean[name] = value
    if errors:
        raise ParamError(errors)
    return clean


# ---------------------------------------------------------------- generacion

def tool_env(settings: Settings) -> dict[str, str]:
    """Entorno para xacro/check_urdf. Si ROS no esta sourceado, agrega el prefijo de Jazzy."""
    env = os.environ.copy()
    if shutil.which("xacro", path=env.get("PATH")) is None:
        prefix = settings.ros_prefix
        py = next(iter(sorted(prefix.glob("lib/python3*/site-packages"))), None)

        def prepend(key: str, value: Path | None) -> None:
            if value is not None:
                env[key] = os.pathsep.join(filter(None, [str(value), env.get(key, "")]))

        prepend("PATH", prefix / "bin")
        prepend("LD_LIBRARY_PATH", prefix / "lib")
        prepend("PYTHONPATH", py)
    return env


def _run(cmd: list[str], settings: Settings, cwd: Path) -> subprocess.CompletedProcess:
    env = tool_env(settings)
    exe = shutil.which(cmd[0], path=env.get("PATH"))
    if exe is None:
        raise GenerationError(f"'{cmd[0]}' no encontrado; instalar ros-jazzy-xacro / liburdfdom-tools")
    try:
        return subprocess.run([exe, *cmd[1:]], cwd=cwd, env=env, capture_output=True, text=True,
                              timeout=settings.subprocess_timeout_s, check=False)
    except subprocess.TimeoutExpired as exc:
        raise GenerationError(f"{cmd[0]} excedio {settings.subprocess_timeout_s}s") from exc


def run_xacro(settings: Settings, urdf_dir: Path, out: Path) -> str:
    proc = _run(["xacro", str(urdf_dir / settings.entry_rel), "-o", str(out)], settings, urdf_dir)
    if proc.returncode != 0:
        raise GenerationError(f"xacro fallo (rc={proc.returncode}): {proc.stderr.strip()}")
    return proc.stderr.strip()


def summarize_urdf(settings: Settings, urdf_text: str) -> dict:
    root = ET.fromstring(urdf_text)
    links = [l.get("name") for l in root.findall("link")]
    joints = [{"name": j.get("name"), "type": j.get("type"),
               "parent": j.find("parent").get("link"), "child": j.find("child").get("link")}
              for j in root.findall("joint")]
    meshes: dict[str, dict] = {}
    for m in root.iter("mesh"):
        fn = m.get("filename", "")
        if fn in meshes:
            continue
        info = {"filename": fn, "found": False, "url": None}
        match = _PKG_RE.match(fn)
        pkg_dir = settings.package_dir(match.group(1)) if match else None
        if pkg_dir is not None:
            path = (pkg_dir / match.group(2)).resolve()
            if path.is_relative_to(pkg_dir) and path.is_file():
                info.update(found=True, url=f"/meshes/{match.group(1)}/{match.group(2)}")
        meshes[fn] = info
    return {"links": links, "joints": joints, "meshes": list(meshes.values())}


def props_diff(original: str, candidate: str, rel: str) -> str:
    return "".join(difflib.unified_diff(
        original.splitlines(keepends=True), candidate.splitlines(keepends=True),
        fromfile=f"a/{rel}", tofile=f"b/{rel} (preview)"))


def preview(settings: Settings, values: dict[str, float]) -> dict:
    """`values` debe venir de validate_values()."""
    original = settings.props_path.read_text(encoding="utf-8")
    candidate = xacro_props.apply_values(original, values)
    warnings: list[str] = []

    with tempfile.TemporaryDirectory(prefix="urdf_editor_") as tmp:
        tmp_urdf = Path(tmp) / "urdf"
        # symlinks=True: no seguir enlaces hacia fuera del arbol al copiar.
        shutil.copytree(settings.urdf_dir, tmp_urdf, symlinks=True)
        (tmp_urdf / settings.props_rel).write_text(candidate, encoding="utf-8")
        out = Path(tmp) / "preview.urdf"
        xacro_stderr = run_xacro(settings, tmp_urdf, out)
        if xacro_stderr:
            warnings.append(f"xacro: {xacro_stderr}")
        urdf_text = out.read_text(encoding="utf-8")
        check = _run(["check_urdf", str(out)], settings, Path(tmp))

    if "${" in urdf_text:
        warnings.append("El URDF generado contiene expresiones ${...} sin resolver")
    check_ok = check.returncode == 0
    if not check_ok:
        warnings.append(f"check_urdf fallo: {check.stderr.strip() or check.stdout.strip()}")
    summary = summarize_urdf(settings, urdf_text)
    for m in summary["meshes"]:
        if not m["found"]:
            warnings.append(f"Mesh no disponible localmente: {m['filename']} (se mostrara placeholder)")
    if values:
        warnings.append("Datos inerciales (masa/inercia) NO se recalculan: pendientes de recalcular/medir.")
        warnings.append("Footprint/costmap de Nav2 NO cambian con este preview.")
    for name in values:
        spec = settings.load_schema()["editable"][name]
        if "warning" in spec:
            warnings.append(f"{name}: {spec['warning']}")

    return {
        "urdf": urdf_text,
        "check_urdf": {"ok": check_ok, "output": (check.stdout + check.stderr).strip()},
        "warnings": warnings,
        "diff": props_diff(original, candidate, f"{settings.urdf_rel}/{settings.props_rel}"),
        "changed": {k: xacro_props.format_number(v) for k, v in values.items()},
        **summary,
    }


# ---------------------------------------------------------------- exportacion

def export_candidate(settings: Settings, values: dict[str, float], label: str | None) -> dict:
    if label is not None and not _LABEL_RE.match(label):
        raise ParamError(["label invalido: solo [A-Za-z0-9_-], 1..40 caracteres"])
    result = preview(settings, values)  # no exportar algo que xacro no genera
    if not result["check_urdf"]["ok"]:
        raise GenerationError("check_urdf fallo; no se exporta el candidato")

    cand_dir = settings.candidates_dir
    if cand_dir.is_symlink():
        raise GenerationError("candidates/ no puede ser un enlace simbolico")
    cand_dir.mkdir(parents=True, exist_ok=True)
    cand_dir = cand_dir.resolve()

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    stem = f"properties_{stamp}" + (f"_{label}" if label else "")
    written = []
    original = settings.props_path.read_text(encoding="utf-8")
    contents = {
        f"{stem}.xacro": xacro_props.apply_values(original, values),
        f"{stem}.diff": result["diff"],
    }
    for name, text in contents.items():
        path = (cand_dir / name).resolve()
        if path.parent != cand_dir:
            raise ParamError(["ruta de exportacion fuera de candidates/"])
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
        with os.fdopen(os.open(path, flags, 0o644), "w", encoding="utf-8") as fh:
            fh.write(text)
        repo = settings.repo_root.resolve()
        written.append(str(path.relative_to(repo)) if path.is_relative_to(repo) else str(path))
    return {"files": written, "diff": result["diff"], "warnings": result["warnings"]}
