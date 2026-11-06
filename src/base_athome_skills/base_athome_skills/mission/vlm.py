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
