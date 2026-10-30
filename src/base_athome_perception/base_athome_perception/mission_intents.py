#!/usr/bin/env python3
"""Interpreta pedidos hablados (ingles o espanol) en una lista ordenada de pasos de mision.

Pasos posibles:
  ("go_to", waypoint)         ir a un waypoint del mapa (door, living_room, ...)
  ("receive_guest", None)     recibir a la persona en la puerta (nombre, bebida, cara)
  ("fetch", object_key)       ir al objeto del mapa y agarrarlo
  ("guide", waypoint)         guiar a la persona hasta un lugar ("follow me")
  ("place", None)             soltar el objeto que lleva
  ("go_home", None)           volver al lugar por defecto
Ejemplos:
  "go to the door and receive the guest"         -> go_to door, receive_guest
  "take the pringles from the table"             -> fetch obj_1
  "take the guest to the living room and leave the pringles there" -> guide living_room, place
  "do the mission" / "haz la mision"            -> mision completa de demostracion
"""
import re
from typing import Dict, List, Optional, Tuple

Step = Tuple[str, Optional[str]]

WAYPOINT_WORDS = {
    "door": ["door", "entrance", "puerta", "entrada"],
    "living_room": ["living room", "livingroom", "sala de estar", "sala", "living"],
    "kitchen": ["kitchen", "cocina"],
    "bedroom": ["bedroom", "dormitorio", "cuarto"],
}
RECEIVE = ["receive", "welcome", "greet", "guest", "visitor", "recibi", "recibe", "recibir", "invitado", "visita", "persona"]
FETCH = ["take", "grab", "pick", "bring", "fetch", "get", "toma", "tomá", "agarra", "agarrá", "trae", "traé", "busca", "buscá"]
GUIDE = ["guide", "lead", "take the guest", "take the person", "accompany", "show", "lleva a la persona", "llevá a la persona",
         "guia", "guiá", "acompaña", "acompañá", "lleva al invitado", "llevá al invitado"]
PLACE = ["leave", "drop", "release", "put", "place", "deliver", "suelta", "soltá", "deja", "dejá", "pon", "poné", "entrega"]
HOME = ["go home", "go back", "return", "vuelve", "volvé", "regresa", "a casa", "base"]
FULL_MISSION = ["do the mission", "start the mission", "full mission", "haz la mision", "hace la mision", "hacé la misión",
                "haz la misión", "empeza la mision", "empezá la misión", "mision completa", "misión completa"]


def _norm(text: str) -> str:
    t = text.lower()
    for a, b in (("á", "a"), ("é", "e"), ("í", "i"), ("ó", "o"), ("ú", "u")):
        t = t.replace(a, b)
    return re.sub(r"\s+", " ", t)


def _has(t: str, words) -> bool:
    return any(re.search(r"\b" + re.escape(_norm(w)) + r"\b", t) for w in words)


def _first_pos(t: str, words) -> int:
    pos = [m.start() for w in words for m in [re.search(r"\b" + re.escape(_norm(w)) + r"\b", t)] if m]
    return min(pos) if pos else -1


def find_waypoint(t: str, available: Optional[List[str]] = None) -> Optional[str]:
    best = None
    for key, words in WAYPOINT_WORDS.items():
        p = _first_pos(t, words)
        if p >= 0 and (best is None or p < best[0]):
            best = (p, key)
    if best and (available is None or best[1] in available):
        return best[1]
    for key in available or []:  # nombres propios del mapa ("waypoint B", "mesa")
        if _has(t, [key.replace("_", " ")]):
            return key
    return None


def find_object(t: str, objects: Dict[str, Dict]) -> Optional[str]:
    """Objeto del mapa por nombre ("pringles"), clase ("can") o numero ("object 1", "objeto 1", "number 1")."""
    m = re.search(r"\b(?:object|objeto|number|numero|item)\s*(?:number|numero)?\s*(\d+)\b", t)
    if m:
        n = int(m.group(1))
        for key, o in objects.items():
            if int(o.get("number", -1)) == n:
                return key
    for key, o in objects.items():
        if _has(t, [str(o.get("name", key))]):
            return key
    for key, o in objects.items():
        if o.get("class") and _has(t, [str(o["class"])]):
            return key
    return None


def full_mission(objects: Dict[str, Dict]) -> List[Step]:
    obj = next(iter(sorted(objects, key=lambda k: objects[k].get("number", 99))), None)
    steps: List[Step] = [("go_to", "door"), ("receive_guest", None)]
    if obj:
        steps.append(("fetch", obj))
    steps += [("guide", "living_room")]
    if obj:
        steps.append(("place", None))
    steps.append(("go_home", None))
    return steps


def parse_commands(text: str, objects: Optional[Dict[str, Dict]] = None,
                   waypoints: Optional[List[str]] = None) -> List[Step]:
    objects = objects or {}
    t = _norm(text)
    if _has(t, FULL_MISSION):
        return full_mission(objects)
    # Dividir en clausulas por conectores para respetar el orden pedido
    clauses = [c.strip() for c in re.split(r"\b(?:and then|then|and|after that|luego|despues|y despues|y luego|y)\b|[,.;]", t)
               if c and c.strip()]
    steps: List[Step] = []
    for c in clauses:
        wp = find_waypoint(c, waypoints)
        obj = find_object(c, objects)
        if _has(c, GUIDE) and wp:
            steps.append(("guide", wp))
        elif _has(c, PLACE) and (obj or "there" in c or "alli" in c or "ahi" in c or not _has(c, FETCH)):
            steps.append(("place", None))
        elif _has(c, FETCH) and obj:
            steps.append(("fetch", obj))
        elif _has(c, RECEIVE) and not wp:
            if not steps or steps[-1] != ("go_to", "door"):
                steps.append(("go_to", "door"))
            steps.append(("receive_guest", None))
        elif _has(c, HOME):
            steps.append(("go_home", None))
        elif wp:
            steps.append(("go_to", wp))
    # quitar repetidos consecutivos
    out: List[Step] = []
    for s in steps:
        if not out or out[-1] != s:
            out.append(s)
    return out


def describe(steps: List[Step], objects: Optional[Dict[str, Dict]] = None) -> str:
    objects = objects or {}
    words = []
    for kind, arg in steps:
        name = (arg or "").replace("_", " ")
        if kind == "go_to":
            words.append(f"go to the {name}")
        elif kind == "receive_guest":
            words.append("receive the guest")
        elif kind == "fetch":
            words.append(f"take the {objects.get(arg, {}).get('name', name)}")
        elif kind == "guide":
            words.append(f"guide the guest to the {name}")
        elif kind == "place":
            words.append("leave the object there")
        elif kind == "go_home":
            words.append("go back to my place")
    if not words:
        return ""
    return ", ".join(words[:-1]) + (" and " if len(words) > 1 else "") + words[-1]
