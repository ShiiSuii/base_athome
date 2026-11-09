"""Lo que el commander dice por voz. Puro y en ingles: el ASR (base.en) y la voz (Piper en_US) son en ingles."""
import re
from typing import Any, Dict, List, Optional


def _place_name(place: str, objects: List[Dict[str, Any]]) -> str:
    if place.startswith("object:"):
        key = place[len("object:"):]
        for o in objects:
            if o["key"] == key:
                return f"object {o['number']}, the {o['name']}"
        return "the object"
    return "the " + place.replace("_", " ")


def plan_to_speech(plan: Dict[str, Any], objects: Optional[List[Dict[str, Any]]] = None) -> str:
    """Resumen hablado del plan, para confirmarlo antes de moverse."""
    objects = objects or []
    parts = []
    for s in plan.get("steps", []):
        skill, args = s["skill"], s.get("args", {})
        if skill == "navigate_to":
            phrase = f"go to {_place_name(str(args.get('place')), objects)}"
        elif skill == "return_home":
            phrase = "go back home"
        elif skill == "detect_object":
            phrase = f"look for the {args.get('object')}"
        elif skill == "pick_object":
            phrase = "pick it up" if args.get("use_bag_target") else f"pick up the {args.get('object')}"
        elif skill == "place_object":
            phrase = "put it down"
        elif skill == "approach_person":
            phrase = "come to you"
        elif skill == "find_pointed_bag":
            phrase = "find the bag you point at"
        elif skill == "approach_bag":
            phrase = "go to the bag"
        elif skill in ("set_torso", "dock_bag", "remember_bag"):
            continue                          # detalle interno, como la pose del brazo
        elif skill == "speak" and str(args.get("text", "")).startswith("Please point"):
            continue
        elif skill == "speak":
            phrase = f"say: {args.get('text')}"
        elif skill == "dock_front":
            phrase = f"get close to the {args.get('target', 'thing')} in front of me"
        elif skill == "describe_view":
            phrase = "look and tell you what I see"
        elif skill == "wait":
            phrase = f"wait {args.get('seconds')} seconds"
        elif skill == "listen":
            phrase = "listen"
        else:
            continue                          # arm_pose es un detalle interno
        state = s.get("if", {}).get("state")
        parts.append(("if I cannot find it, " if state == "FAILED" else "") + phrase)
    if not parts:
        return "I have nothing to do."
    text = parts[0] if len(parts) == 1 else ", ".join(parts[:-1]) + ", and " + parts[-1]
    return f"I will {text}."


def error_to_speech(errors: List[str]) -> str:
    """Pasa el primer error del compilador (en castellano, para la web) a una frase hablada."""
    e = errors[0] if errors else ""
    m = re.search(r"no conozco el lugar '([^']+)'", e)
    if m:
        return f"I do not know the place {m.group(1)}."
    m = re.search(r"no hay objeto numero (\w+)", e)
    if m:
        return f"There is no object number {m.group(1)} on the map."
    if "deci el numero" in e:
        return "There is more than one on the map. Please tell me the object number."
    if "varias mesas" in e:
        return "There are several tables. Please tell me which one."
    if "agregale un waypoint" in e:
        return "I do not have a place to stand at that table."
    if "punto de basura" in e:
        return "I do not know where to leave the trash. Please add the trash point on the map."
    if "cae dentro" in e or "no tiene punto" in e:
        return "I cannot reach that object. Its position on the map must be fixed."
    if "no esta implementado" in e:
        return "I cannot do that yet."
    if "ya tengo" in e:
        return "My gripper is already holding something."
    if "no tengo nada" in e:
        return "I am not holding anything."
    return "Sorry, I did not understand."


def asr_prompt(places: List[str], objects: List[Dict[str, Any]]) -> str:
    """Vocabulario para sesgar a Whisper hacia lo que el robot puede hacer."""
    names = [p.replace("_", " ") for p in places if not p.startswith("object:")]
    things = sorted({o["name"] for o in objects} | {o["cls"] for o in objects if o["cls"] != "other"})
    text = "Go to the kitchen, pick up object 1, find the bottle, bring it to the table, take out the trash, go back home, stop."
    if names:
        text += " Places: " + ", ".join(names) + "."
    if things:
        text += " Objects: " + ", ".join(things) + "."
    return text


def needs_confirmation(plan: Dict[str, Any], registry) -> bool:
    """Solo se pide "yes" si algun paso puede mover el robot. Una skill desconocida cuenta como que mueve."""
    for s in plan.get("steps", []) + plan.get("recovery", []):
        spec = registry.get(s.get("skill", ""))
        if spec is None or spec.moves_hardware:
            return True
    return False
