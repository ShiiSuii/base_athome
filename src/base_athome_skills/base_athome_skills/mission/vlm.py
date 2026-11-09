"""Cliente local del VLM (Visual Language Model). Sin ROS.

Permite consultar al servidor local (scripts/vlm_server.py en 127.0.0.1:8772) pasando
una imagen (JPEG en bytes o array OpenCV) y una pregunta en lenguaje natural.
Si el servidor no esta corriendo o tarda demasiado, devuelve None sin lanzar excepcion.
"""
import base64
import json
import urllib.error
import urllib.request
from typing import Any, Optional

URL = "http://127.0.0.1:8772"
TIMEOUT_S = 15.0
# Medido en la NUC el 2026-11-09 con SmolVLM 500M Q8: 835 MiB y unos 5 s por respuesta. Solo con 4 imagenes de prueba.
ROBOT_TIMEOUT_S = 25.0
DEFAULT_QUESTION = "Describe what you see in one short sentence."


def encode_image_bytes(image_bytes: bytes) -> str:
    """Codifica los bytes JPEG en base64 ASCII."""
    return base64.b64encode(image_bytes).decode("ascii")


def ask_vlm(image_bytes: bytes, question: str, url: str = URL, timeout_s: float = TIMEOUT_S) -> Optional[str]:
    """Consulta al servidor local de VLM con una imagen en bytes y una pregunta.

    Devuelve la respuesta en texto o None si el servidor no responde o falla.
    """
    if not image_bytes or not question:
        return None
    try:
        payload = json.dumps(dict(image=encode_image_bytes(image_bytes), question=question.strip())).encode("utf-8")
        req = urllib.request.Request(url, payload, {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout_s) as res:
            data = json.load(res)
            ans = data.get("answer")
            return str(ans).strip() if ans is not None else None
    except (OSError, ValueError, urllib.error.URLError, TimeoutError):
        return None


def ask_vlm_cv(frame: Any, question: str, url: str = URL, timeout_s: float = TIMEOUT_S) -> Optional[str]:
    """Variante para imagenes de OpenCV (numpy ndarray). Codifica a JPEG antes de enviar."""
    if frame is None:
        return None
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            return None
        return ask_vlm(buf.tobytes(), question, url=url, timeout_s=timeout_s)
    except Exception:
        return None


def short_answer(text: str, max_chars: int = 200) -> str:
    """Hasta dos oraciones: el robot lo dice en voz alta."""
    text = " ".join(str(text or "").split())
    parts = [p for p in text.replace("!", ".").replace("?", ".").split(".") if p.strip()]
    out = ". ".join(p.strip() for p in parts[:2])
    out = (out + ".") if out else ""
    return out[:max_chars]


def count_question(params: dict) -> str:
    """Pregunta para contar, desde los parametros de la gramatica oficial de GPSR."""
    target = str(params.get("target") or "objects")
    extra = []
    if params.get("clothing"):
        extra.append(f"wearing {params['clothing']}")
    if params.get("gesture"):
        extra.append(str(params["gesture"]))
    if params.get("posture"):
        extra.append(str(params["posture"]))
    what = " ".join([target] + extra)
    return f"How many {what} do you see? Answer with one number."


def describe_question(params: dict) -> str:
    """Pregunta para describir un objeto ("the largest drink") o, si no hay datos, lo que se ve."""
    prop, target = str(params.get("property") or "").strip(), str(params.get("target") or "").strip()
    if prop and target:
        return f"Which is the {prop} {target} you see? Answer with its name only."
    return DEFAULT_QUESTION
