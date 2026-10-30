"""Rutas y configuracion del editor. Todo relativo al checkout; sin rutas absolutas del robot."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

TOOL_DIR = Path(__file__).resolve().parent.parent
REPO_ROOT = TOOL_DIR.parent.parent

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765
MAX_BODY_BYTES = 16 * 1024


@dataclass(frozen=True)
class Settings:
    repo_root: Path = REPO_ROOT
    urdf_rel: str = "src/hoverboard_base/urdf"
    entry_rel: str = "robot.urdf.xacro"           # relativo a urdf_rel
    props_rel: str = "xacro/properties.xacro"     # relativo a urdf_rel
    # package:// -> directorio local (solo lectura). Relativo a repo_root.
    packages: dict[str, str] = field(default_factory=lambda: {"piper_description": "src/piper_description"})
    candidates_dir: Path = TOOL_DIR / "candidates"
    schema_path: Path = TOOL_DIR / "config" / "editable_params.json"
    static_dir: Path = TOOL_DIR / "static"
    ros_prefix: Path = Path("/opt/ros/jazzy")
    subprocess_timeout_s: float = 30.0

    @property
    def urdf_dir(self) -> Path:
        return self.repo_root / self.urdf_rel

    @property
    def props_path(self) -> Path:
        return self.urdf_dir / self.props_rel

    def package_dir(self, name: str) -> Path | None:
        rel = self.packages.get(name)
        return (self.repo_root / rel).resolve() if rel else None

    def load_schema(self) -> dict:
        return json.loads(self.schema_path.read_text(encoding="utf-8"))
