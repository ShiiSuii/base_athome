"""Lugares semanticos (solo lectura) desde semantic_map.yaml y waypoints.yaml (capa de aplicacion)."""
import math
import os
from typing import Any, Dict, List, Optional, Tuple

import yaml

Place = Tuple[float, float, float]  # x, y (marco map), yaw (rad)


def _waypoints(path: str) -> Dict[str, dict]:
    if not path or not os.path.exists(path):
        return {}
    with open(path) as f:
        return dict((yaml.safe_load(f) or {}).get("waypoints") or {})


def load_places(semantic_map: str, waypoints: str = "") -> Dict[str, Place]:
    """Waypoints de semantic_map.yaml; los de waypoints.yaml completan los que falten.

    Tambien agrega `object:<clave>` con el punto de aproximacion de cada objeto del mapa si lo tiene, y cada
    mueble sin waypoint propio con el punto del objeto de menor numero que tenga encima.
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
        for obj in load_objects(semantic_map):
            if obj["on"] and not obj["problem"] and resolve(out, obj["on"]) is None:
                out[obj["on"]] = out[obj["place"]]
    return out


def _read_map(semantic_map: str) -> Dict[str, Any]:
    if not semantic_map or not os.path.exists(semantic_map):
        return {}
    with open(semantic_map) as f:
        return yaml.safe_load(f) or {}


def point_in_polygon(x: float, y: float, polygon: List[List[float]]) -> bool:
    inside = False
    n = len(polygon or [])
    for i in range(n):
        xi, yi = float(polygon[i][0]), float(polygon[i][1])
        xj, yj = float(polygon[i - 1][0]), float(polygon[i - 1][1])
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi) + xi:
            inside = not inside
    return inside


def load_tables(semantic_map: str) -> Dict[str, Dict[str, str]]:
    """Muebles del mapa: {id: {name, type}}."""
    furniture = _read_map(semantic_map).get("furniture") or {}
    return {str(k): {"name": str((v or {}).get("name", k)), "type": str((v or {}).get("type", ""))}
            for k, v in furniture.items() if isinstance(v, dict)}


def load_objects(semantic_map: str) -> List[Dict[str, Any]]:
    """Objetos del mapa en orden numerico. `problem` != "": no se puede ir a buscarlo (y dice por que)."""
    data = _read_map(semantic_map)
    furniture = data.get("furniture") or {}
    out = []
    for key, obj in (data.get("objects") or {}).items():
        if not isinstance(obj, dict) or "number" not in obj:
            continue
        cls, name, app = str(obj.get("class", "other")), str(obj.get("name", key)), obj.get("approach") or {}
        problem = ""
        if "x" not in app or "y" not in app:
            problem = "no tiene punto de aproximacion"
        else:
            for fid, f in furniture.items():
                if isinstance(f, dict) and point_in_polygon(float(app["x"]), float(app["y"]), f.get("polygon")):
                    problem = f"su punto de aproximacion cae dentro de '{fid}': volve a colocarlo en el editor"
                    break
        out.append(dict(number=int(obj["number"]), key=str(key), name=name, cls=cls, on=str(obj.get("on") or ""),
                        label=name if cls == "other" else cls,   # etiqueta que tiene que ver el detector
                        place=f"object:{key}" if not problem or "x" in app else "", problem=problem))
    return sorted(out, key=lambda o: o["number"])


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
