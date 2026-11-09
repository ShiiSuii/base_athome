#!/usr/bin/env python3
"""Bip breve cada 5 s para que el parlante Bluetooth no se apague; si se desconecto, lo reconecta. No compite con el TTS."""

import argparse
import json
import math
import os
import signal
import struct
import subprocess
import threading
import time

from base_athome_perception.audio_lock import playback_lock, reconnect_bluetooth_speaker


def quiet_tone(rate=16000):
    """PCM s16le mono, 440 Hz al 6 % de amplitud (bip suave pero audible: al 0.3 % no se oia), con rampas de 10 ms."""
    samples = int(rate * 0.15)
    ramp = rate * 0.01
    return b"".join(struct.pack("<h", round(
        32767 * 0.06 * min(1.0, i / ramp, (samples - 1 - i) / ramp)
        * math.sin(math.tau * 440 * i / rate))) for i in range(samples))


def idle_sink(run_fn=subprocess.run):
    """Bluetooth primero, luego predeterminado; None si ocupado o estado desconocido."""
    res = run_fn(["pactl", "--format=json", "list", "sinks"],
                 capture_output=True, text=True, check=True, timeout=1.0)
    sinks = json.loads(res.stdout)
    target = next((s for s in sinks if "bluez_output" in s["name"]), None)
    if target is None and reconnect_bluetooth_speaker():     # se apago o se desconecto: volver a conectarlo
        sinks = json.loads(run_fn(["pactl", "--format=json", "list", "sinks"],
                                  capture_output=True, text=True, check=True, timeout=1.0).stdout)
        target = next((s for s in sinks if "bluez_output" in s["name"]), None)
    if target is None:
        default = run_fn(["pactl", "get-default-sink"], capture_output=True,
                         text=True, check=True, timeout=1.0).stdout.strip()
        target = next((s for s in sinks if s["name"] == default), None)
    if target is None or target.get("state", "").upper() not in ("IDLE", "SUSPENDED"):
        return None
    return target["name"]


def play_if_idle(run_fn=subprocess.run, lock_fn=playback_lock):
    # El lock cubre tambien la sintesis TTS, incluso antes de que el sink pase a RUNNING.
    with lock_fn(blocking=False) as acquired:
        if not acquired:
            return False
        sink = idle_sink(run_fn)
        if sink is None:
            return False
        run_fn(["paplay", "--device", sink, "--raw", "--rate=16000",
                "--channels=1", "--format=s16le"], input=quiet_tone(),
               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True, timeout=5.0)   # un parlante recien despierto tarda
        return True


class AudioKeepalive:
    def __init__(self, interval_s=5.0, clock_fn=time.monotonic, play_fn=play_if_idle):
        if not math.isfinite(interval_s) or interval_s <= 0:
            raise ValueError("interval_s must be positive and finite")
        self.interval_s = interval_s
        self.clock_fn = clock_fn
        self.play_fn = play_fn
        self.next_due = clock_fn() + interval_s

    def tick(self):
        now = self.clock_fn()
        if now < self.next_due:
            return False
        self.next_due = now + self.interval_s  # Si estaba ocupado, saltear sin reintentos rapidos.
        try:
            return self.play_fn()
        except (OSError, subprocess.SubprocessError, ValueError, KeyError, TypeError) as exc:
            print(f"[audio_keepalive] ciclo omitido: {exc}", flush=True)
            return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interval", type=float,
                        default=os.environ.get("AUDIO_KEEPALIVE_INTERVAL_S", "5"),
                        help="Intervalo en segundos (default: 5, pedido del usuario: el parlante Bluetooth se apagaba; AUDIO_KEEPALIVE_INTERVAL_S)")
    args = parser.parse_args()
    try:
        keepalive = AudioKeepalive(args.interval)
    except ValueError as exc:
        parser.error(str(exc))
    stopped = threading.Event()
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
        signal.signal(sig, lambda *_: stopped.set())
    while not stopped.wait(max(0.0, keepalive.next_due - keepalive.clock_fn())):
        keepalive.tick()


if __name__ == "__main__":
    main()
