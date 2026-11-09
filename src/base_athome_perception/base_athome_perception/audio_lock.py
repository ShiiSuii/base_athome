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


AUDIO_SINK_UUID = "0000110b-0000-1000-8000-00805f9b34fb"   # A2DP Audio Sink: un parlante


def paired_speakers(run_fn=None):
    """MACs de los dispositivos Bluetooth emparejados que son parlantes (Audio Sink)."""
    import subprocess
    run_fn = run_fn or subprocess.run
    out = run_fn(["bluetoothctl", "devices", "Paired"], capture_output=True, text=True, timeout=3).stdout
    macs = [line.split()[1] for line in out.splitlines() if line.startswith("Device ") and len(line.split()) > 1]
    speakers = []
    for mac in macs:
        info = run_fn(["bluetoothctl", "info", mac], capture_output=True, text=True, timeout=3).stdout
        if AUDIO_SINK_UUID in info.lower():
            speakers.append((mac, "Connected: yes" in info))
    return speakers


def reconnect_bluetooth_speaker(run_fn=None):
    """Si el parlante emparejado se desconecto (se apaga solo por inactividad), pide reconectarlo. True si quedo conectado."""
    import subprocess
    run_fn = run_fn or subprocess.run
    try:
        speakers = paired_speakers(run_fn)
        if any(connected for _, connected in speakers):
            return True
        for mac, _ in speakers:
            res = run_fn(["bluetoothctl", "connect", mac], capture_output=True, text=True, timeout=10)
            if "Connection successful" in (res.stdout or ""):
                return True
    except (OSError, subprocess.SubprocessError):
        pass
    return False
