"""Exclusion de reproduccion local entre TTS, beeps y keepalive (sin ROS)."""

from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import tempfile


@contextmanager
def playback_lock(blocking=True, path=None):
    """El keepalive usa blocking=False; TTS y beeps esperan el fin del audio anterior."""
    if path is None:
        runtime_dir = os.environ.get("XDG_RUNTIME_DIR") or tempfile.gettempdir()
        path = Path(runtime_dir) / f"base_athome_audio_{os.getuid()}.lock"
    with open(path, "a") as stream:
        try:
            fcntl.flock(stream, fcntl.LOCK_EX | (0 if blocking else fcntl.LOCK_NB))
        except BlockingIOError:
            yield False
            return
        try:
            yield True
        finally:
            fcntl.flock(stream, fcntl.LOCK_UN)
