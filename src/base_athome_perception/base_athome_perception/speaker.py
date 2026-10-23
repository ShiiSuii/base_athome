#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Sintesis de Voz Neuronal en Tiempo Real (RoboCup @Home)
# Utiliza Piper TTS con modelo neural en ingles (en_US-lessac-medium)
# Reproduccion de baja latencia a traves del parlante Bluetooth (PipeWire / PulseAudio)
# ==============================================================================

import os
import queue
import subprocess
import threading
import time
from typing import Optional, Callable


class RoboCupSpeaker:
    """Motor de Text-to-Speech neuronal de alta calidad y voz natural en ingles."""

    def __init__(
        self,
        piper_bin: str = "/home/athome/piper_bin/piper/piper",
        model_path: str = "/home/athome/piper_bin/voices/en/en_US-lessac-medium.onnx",
        sample_rate: int = 22050,
    ):
        self.piper_bin = piper_bin
        self.model_path = model_path
        self.sample_rate = sample_rate

        self._queue = queue.Queue()
        self._is_speaking = False
        self._current_process: Optional[subprocess.Popen] = None
        self._running = True

        # Hilo de reproduccion continua
        self._worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker_thread.start()

    @property
    def is_speaking(self) -> bool:
        return self._is_speaking

    def speak(self, text: str, blocking: bool = False, on_done: Optional[Callable] = None) -> None:
        """Envia un texto para ser sintetizado y reproducido en ingles natural."""
        text = text.strip()
        if not text:
            return

        done_event = threading.Event() if blocking else None

        def done_callback():
            if on_done:
                on_done()
            if done_event:
                done_event.set()

        self._queue.put((text, done_callback))

        if blocking and done_event:
            done_event.wait()

    def stop(self) -> None:
        """Detiene inmediatamente cualquier locucion en curso y limpia la cola."""
        while not self._queue.empty():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                break

        if self._current_process and self._current_process.poll() is None:
            try:
                self._current_process.terminate()
            except Exception:
                pass
        self._is_speaking = False

    def _worker_loop(self) -> None:
        while self._running:
            try:
                text, done_cb = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            self._is_speaking = True
            try:
                # Sanitizar texto para bash
                clean_text = text.replace('"', '\\"').replace("`", "").replace("$", "")
                cmd = (
                    f'echo "{clean_text}" | '
                    f'"{self.piper_bin}" --model "{self.model_path}" --output-raw 2>/dev/null | '
                    f'paplay --raw --rate {self.sample_rate} --channels 1 --format s16le 2>/dev/null'
                )

                self._current_process = subprocess.Popen(
                    cmd,
                    shell=True,
                    executable="/bin/bash",
                )
                self._current_process.wait()
            except Exception as e:
                print(f"[RoboCupSpeaker] Error en reproduccion TTS: {e}")
            finally:
                self._is_speaking = False
                self._current_process = None
                try:
                    done_cb()
                except Exception:
                    pass

    def shutdown(self) -> None:
        self._running = False
        self.stop()
