#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Sintesis de Voz Neuronal y Audio Player para HRI (RoboCup @Home)
# Utiliza Piper TTS con modelo neural en ingles (en_US-lessac-medium)
# Soporte para turn-taking acustico (beep.wav) y reproduccion de audio WAV
# Reproduccion a traves del parlante Bluetooth (PipeWire / PulseAudio)
# ==============================================================================

import os
import queue
import subprocess
import tempfile
import threading
import time
import wave
from typing import Callable, Optional

from base_athome_perception.audio_lock import playback_lock


def prepend_wav_silence(path: str, seconds: float = 0.8) -> None:
    """Warm up the audio stream before speech, within the same playback."""
    with wave.open(path, "rb") as src:
        params = src.getparams()
        audio = src.readframes(src.getnframes())
    silent_sample = b"\x80" if params.sampwidth == 1 else b"\x00" * params.sampwidth
    silence = silent_sample * params.nchannels * round(params.framerate * seconds)
    with wave.open(path, "wb") as dst:
        dst.setparams(params)
        dst.writeframes(silence + audio)


class RoboCupSpeaker:
    """Motor de Text-to-Speech neuronal y reproductor de audio para HRI (RoboCup @Home)."""

    def __init__(
        self,
        piper_bin: str = "/home/athome/piper_bin/piper/piper",
        model_path: str = "/home/athome/piper_bin/voices/en/en_US-lessac-medium.onnx",
        sample_rate: int = 22050,
        beep_path: Optional[str] = None,
    ):
        self.piper_bin = piper_bin
        self.model_path = model_path
        self.sample_rate = sample_rate

        # Ruta predeterminada del tono de aviso acústico (beep de escucha)
        if beep_path and os.path.exists(beep_path):
            self.beep_path = beep_path
        else:
            pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.beep_path = os.path.join(pkg_root, "resources", "beep.wav")

        self._queue = queue.Queue()
        self._is_speaking = False
        self._current_process: Optional[subprocess.Popen] = None
        self._running = True
        self._stop_generation = 0
        self._process_lock = threading.Lock()

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

    def play_audio_file(self, wav_path: str, blocking: bool = False) -> None:
        """Reproduce un archivo WAV directamente al parlante Bluetooth."""
        if not os.path.exists(wav_path):
            return

        def _play():
            try:
                with playback_lock():
                    bt_sink = self.ensure_bluetooth_sink()
                    sink_flag = ["-d", bt_sink] if bt_sink else []
                    subprocess.run(["paplay"] + sink_flag + [wav_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception as e:
                print(f"[RoboCupSpeaker] Error reproduciendo audio: {e}")

        if blocking:
            _play()
        else:
            threading.Thread(target=_play, daemon=True).start()

    def play_beep(self, blocking: bool = True) -> None:
        """Reproduce el tono acustico (beep) de turn-taking para avisar al humano que el robot escucha."""
        if os.path.exists(self.beep_path):
            self.play_audio_file(self.beep_path, blocking=blocking)

    def stop(self) -> None:
        """Detiene inmediatamente cualquier locucion en curso y limpia la cola."""
        while not self._queue.empty():
            try:
                _, done_cb = self._queue.get_nowait()
            except queue.Empty:
                break
            try:
                done_cb()  # p.ej. des-silenciar el microfono
            except Exception:
                pass

        with self._process_lock:
            self._stop_generation += 1
            if self._current_process and self._current_process.poll() is None:
                try:
                    self._current_process.terminate()
                except Exception:
                    pass
        self._is_speaking = False

    @staticmethod
    def ensure_bluetooth_sink() -> Optional[str]:
        """Detecta y activa el parlante Bluetooth como sink predeterminado de audio; si se desconecto, lo reconecta."""
        try:
            from base_athome_perception.audio_lock import reconnect_bluetooth_speaker
            res = subprocess.run(["pactl", "list", "sinks", "short"], stdout=subprocess.PIPE, text=True, timeout=1)
            # Reconectar solo si se eligio el parlante Bluetooth (AUDIO_SPEAKER=bluetooth). Con cable no se intenta:
            # bloquearia cada frase hasta 10 s y podria mandar la voz al Bluetooth.
            if ("bluez_output" not in res.stdout and os.environ.get("AUDIO_SPEAKER", "auto") == "bluetooth"
                    and reconnect_bluetooth_speaker()):
                time.sleep(2.0)     # PipeWire tarda en crear el sink despues de conectar
            res = subprocess.run(["pactl", "list", "sinks", "short"], stdout=subprocess.PIPE, text=True, timeout=1)
            for line in res.stdout.splitlines():
                parts = line.split()
                if len(parts) >= 2 and "bluez_output" in parts[1]:
                    bt_sink = parts[1]
                    subprocess.run(["pactl", "set-default-sink", bt_sink], stderr=subprocess.DEVNULL)
                    return bt_sink
        except Exception:
            pass
        return None

    def _worker_loop(self) -> None:
        while self._running:
            try:
                text, done_cb = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue

            self._is_speaking = True
            wav_path = None
            generation = self._stop_generation
            try:
                with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as wav:
                    wav_path = wav.name
                # Finish synthesis before acquiring the playback lock: Bluetooth
                # must receive a complete WAV without inference-induced underruns.
                with self._process_lock:
                    if generation != self._stop_generation:
                        continue
                    self._current_process = subprocess.Popen(
                        [self.piper_bin, "--model", self.model_path, "--output_file", wav_path],
                        stdin=subprocess.PIPE, text=True,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    )
                self._current_process.communicate(input=text)
                if self._current_process.returncode != 0:
                    continue
                prepend_wav_silence(wav_path)
                with playback_lock():
                    bt_sink = self.ensure_bluetooth_sink()
                    sink_flag = ["-d", bt_sink] if bt_sink else []
                    with self._process_lock:
                        if generation != self._stop_generation:
                            continue
                        self._current_process = subprocess.Popen(
                            ["paplay"] + sink_flag + [wav_path],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        )
                    self._current_process.wait()
            except Exception as e:
                print(f"[RoboCupSpeaker] Error en reproduccion TTS: {e}")

            finally:
                if wav_path is not None:
                    os.unlink(wav_path)
                self._is_speaking = False
                self._current_process = None
                try:
                    done_cb()
                except Exception:
                    pass

    def shutdown(self) -> None:
        self._running = False
        self.stop()
