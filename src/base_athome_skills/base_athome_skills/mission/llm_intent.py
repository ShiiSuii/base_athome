"""Traducir con una LLM local lo que el compilador no entendio. Sin ROS. La LLM solo propone: su salida esta limitada por
una gramatica a nuestras acciones, lugares y objetos, y despues pasa por el mismo compilador y validador que cualquier orden.
Nunca mueve el robot: la confirmacion por voz sigue siendo obligatoria.

Medido en la NUC el 2026-11-06 con Qwen2.5 1.5B Q4: 1.9 GiB, 0.4 s de mediana y 5 s de maximo; frases de reserva: 14/16
aciertos y 0/8 ordenes falsas. Son pocas frases: no prueban que no invente una orden.
"""
import json
import urllib.error
import urllib.request
from typing import Any, Dict, List, Sequence

SYSTEM = ("You read one sentence said near a home robot. Output the robot actions as JSON, or [] when the sentence is NOT a "
          "direct request to the robot to do something now. Statements, opinions, thanks, past events, negations and "
          "questions about the world are not requests: output []. Actions: go_to place; pick object; deliver_to_me; "
          "follow_me; guide place (lead a person there); describe (look and say what the robot sees now).")
SHOTS = (("go to the shelf", '[{"a": "go_to", "x": "shelf"}]'),
         ("bring me a cup from the table", '[{"a": "go_to", "x": "table"}, {"a": "pick", "x": "cup"}, {"a": "deliver_to_me"}]'),
         ("please lead her to the entrance", '[{"a": "guide", "x": "entrance"}]'),
         ("follow me please", '[{"a": "follow_me"}]'),
         ("hey robot what's going on around you", '[{"a": "describe"}]'),
         ("the table is very old", "[]"), ("I love this shelf", "[]"), ("thank you", "[]"),
         ("yesterday I went to the entrance", "[]"), ("never touch the cup", "[]"))
URL = "http://127.0.0.1:8771/intent"
TIMEOUT_S = 12.0


def _alternatives(names: Sequence[str]) -> str:
    clean = sorted({str(n).strip().lower().replace('"', "") for n in names if str(n).strip()})
    return " | ".join('"\\"%s\\""' % n for n in clean)


def grammar(places: Sequence[str], objects: Sequence[str]) -> str:
    """GBNF: solo acciones conocidas, con lugares y objetos de esta lista."""
    rules = ['root ::= "[]" | "[" act (", " act)* "]"', "act ::= " + " | ".join(
        (["go", "gd"] if places else []) + (["pk"] if objects else []) + ["dv", "fl", "ds"])]
    if places:
        rules += ['go ::= "{\\"a\\": \\"go_to\\", \\"x\\": " place "}"', 'gd ::= "{\\"a\\": \\"guide\\", \\"x\\": " place "}"',
                  "place ::= " + _alternatives(places)]
    if objects:
        rules += ['pk ::= "{\\"a\\": \\"pick\\", \\"x\\": " obj "}"', "obj ::= " + _alternatives(objects)]
    rules += ['ds ::= "{\\"a\\": \\"describe\\"}"', 'dv ::= "{\\"a\\": \\"deliver_to_me\\"}"', 'fl ::= "{\\"a\\": \\"follow_me\\"}"']
    return "\n".join(rules)


def messages(text: str) -> List[Dict[str, str]]:
    out = [dict(role="system", content=SYSTEM)]
    for user, answer in SHOTS:
        out += [dict(role="user", content=user), dict(role="assistant", content=answer)]
    return out + [dict(role="user", content=text)]


def to_actions(reply: str) -> List[Dict[str, Any]]:
    """Respuesta de la LLM a acciones en el formato del parser de GPSR. [] si no es una orden o no se entiende."""
    try:
        items = json.loads(reply)
    except (TypeError, ValueError):
        return []
    if not isinstance(items, list):
        return []
    out: List[Dict[str, Any]] = []
    for it in items:
        name, arg = (it.get("a"), it.get("x")) if isinstance(it, dict) else (None, None)
        if name == "go_to" and arg:
            out.append(dict(action="go_to", params=dict(location=arg)))
        elif name == "guide" and arg:
            out.append(dict(action="guide", params=dict(location=arg)))
        elif name == "pick" and arg:
            out.append(dict(action="pick", params=dict(object=arg)))
        elif name == "deliver_to_me":
            out.append(dict(action="deliver_to", params=dict(person="me")))
        elif name == "describe":
            out.append(dict(action="describe", params={}))
        elif name == "follow_me":
            out.append(dict(action="follow", params=dict(person="me")))
        else:
            return []          # una parte que no se entiende invalida todo: nunca ejecutar media orden
    return out


def ask(text: str, places: Sequence[str], objects: Sequence[str], url: str = URL, timeout_s: float = TIMEOUT_S) -> List[Dict[str, Any]]:
    """Pregunta al servidor local. [] si no esta corriendo o tarda: la LLM es opcional."""
    body = json.dumps(dict(messages=messages(text), grammar=grammar(places, objects))).encode()
    req = urllib.request.Request(url, body, {"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as res:
            return to_actions(json.load(res).get("reply", ""))
    except (OSError, ValueError, urllib.error.URLError):
        return []
