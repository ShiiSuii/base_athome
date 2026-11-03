"""Lugares semanticos (solo lectura) desde semantic_map.yaml y waypoints.yaml (capa de aplicacion)."""
import math
import os
from typing import Dict, Optional, Tuple

import yaml

Place = Tuple[float, float, float]  # x, y (marco map), yaw (rad)


def _waypoints(path: str) -> Dict[str, dict]:
    if not path or not os.path.exists(path):
        return {}
    with open(path) as f:
        return dict((yaml.safe_load(f) or {}).get("waypoints") or {})


def load_places(semantic_map: str, waypoints: str = "") -> Dict[str, Place]:
    """Waypoints de semantic_map.yaml; los de waypoints.yaml completan los que falten.

    Tambien agrega `object:<clave>` con el punto de aproximacion de cada objeto del mapa si lo tiene.
    """
    out: Dict[str, Place] = {}
    raw = _waypoints(semantic_map)
    for k, v in _waypoints(waypoints).items():
        raw.setdefault(k, v)
    for name, wp in raw.items():
        try:
            yaw = math.radians(float(wp.get("yaw_deg", 0.0)))
            if "yaw_deg" not in wp and "qz" in wp:
                yaw = 2.0 * math.atan2(float(wp["qz"]), float(wp.get("qw", 1.0)))
            out[str(name)] = (float(wp["x"]), float(wp["y"]), yaw)
        except (KeyError, TypeError, ValueError):
            continue
    if semantic_map and os.path.exists(semantic_map):
        with open(semantic_map) as f:
            objects = (yaml.safe_load(f) or {}).get("objects") or {}
        for key, obj in objects.items():
            app = (obj or {}).get("approach") or {}
            if "x" in app and "y" in app:
                out[f"object:{key}"] = (float(app["x"]), float(app["y"]),
                                        math.radians(float(app.get("yaw_deg", 0.0))))
    return out


def resolve(places: Dict[str, Place], name: str) -> Optional[Place]:
    """Busca por nombre exacto o normalizado ('Living Room' -> 'living_room')."""
    if name in places:
        return places[name]
    norm = str(name).strip().lower().replace(" ", "_").replace("-", "_")
    for k, v in places.items():
        if k.lower() == norm:
            return v
    return None


def home_place(places: Dict[str, Place]) -> Optional[Place]:
    return resolve(places, "home") or resolve(places, "default")
