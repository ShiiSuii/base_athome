#!/usr/bin/env python3
"""Nucleo de escucha del ASR (sin ROS, testeable): VAD Silero en streaming, deteccion de fin de
habla (endpointing) y filtros de texto.

Por que existe (problemas medidos en el robot, 2026-10-30 / 2026-11-03):
  * Ventana fija de 6 s: si el invitado terminaba en 2 s el robot igual esperaba 6 s; si empezaba a
    hablar tarde (o hablaba largo) la frase se cortaba o se perdia.
  * Compuerta "voz si y solo si WebRTC VAD + energia > umbral": con el invitado lejos o hablando bajo
    la ventana terminaba "sin voz" y el audio se tiraba sin transcribir (log: "Ventana de escucha
    terminada sin voz").
  * El pedido de escucha y el texto no tenian identificador: una transcripcion vieja podia
    contestar una pregunta nueva, y si no habia voz el cliente solo se enteraba por timeout.

Este modulo resuelve el "cuando grabar": espera el comienzo de la voz hasta `start_timeout`, graba
mientras la persona habla y corta `end_silence` segundos despues de que termina (como fbot_hri:
silencio de cierre ~1.2 s). Silero (incluido en faster-whisper) corre a 0.34 ms por bloque de 32 ms
en la Jetson Orin Nano (medido), mucho mas robusto que WebRTC+energia frente a ruido.
"""
import re
from collections import deque
from typing import Dict, List, Optional

import numpy as np

SAMPLE_RATE = 16000
VAD_CHUNK = 512                     # Silero v5: bloques de 512 muestras (32 ms) a 16 kHz
CHUNK_SEC = VAD_CHUNK / SAMPLE_RATE


class SileroStream:
    """Silero VAD en streaming (mantiene el estado h/c y el contexto entre bloques)."""

    def __init__(self, session=None):
        if session is None:
            from faster_whisper.vad import get_vad_model
            session = get_vad_model().session
        self.session = session
        self.reset()

    def reset(self):
        self.h = np.zeros((1, 1, 128), np.float32)
        self.c = np.zeros((1, 1, 128), np.float32)
        self.ctx = np.zeros((1, 64), np.float32)

    def __call__(self, chunk: np.ndarray) -> float:
        x = chunk.astype(np.float32).reshape(1, VAD_CHUNK)
        out, self.h, self.c = self.session.run(
            None, {"input": np.concatenate([self.ctx, x], 1), "h": self.h, "c": self.c})
        self.ctx = x[:, -64:]
        return float(np.squeeze(out))


class EnergyVAD:
    """Respaldo si Silero no carga: probabilidad aproximada por energia relativa al ruido de fondo."""

    def __init__(self):
        self.floor = 0.01

    def reset(self):
        pass

    def __call__(self, chunk: np.ndarray) -> float:
        rms = float(np.sqrt(np.mean(chunk.astype(np.float32) ** 2)))
        if rms < self.floor * 2.0:
            self.floor = 0.98 * self.floor + 0.02 * rms
        return float(np.clip((rms / max(self.floor, 1e-4) - 2.0) / 3.0, 0.0, 1.0))


class Endpointer:
    """Maquina de estados de un pedido de escucha.

    feed(chunk, prob, muted) con bloques de 32 ms; devuelve None mientras sigue, o un dict
    {'status': 'ok'|'no_speech', 'audio': np.ndarray float32, 'speech_sec', 'wait_sec', 'reason'}.

    - WAIT: espera voz hasta start_timeout (el reloj NO corre mientras el robot habla: muted).
      La voz "empieza" con `onset_chunks` bloques seguidos con prob >= on_thr (evita clics/golpes).
    - SPEECH: graba; termina tras end_silence s seguidos con prob < off_thr, o en max_speech.
    - Los bloques con el robot hablando (muted) se reemplazan por silencio: el pedido NO se cancela.
    - Voz debil (invitado lejos, Silero duda): si la energia del bloque supera `weak_snr` veces el
      ruido de fondo (`floor`, lo estima el nodo) y Silero da al menos `weak_prob`, cuenta como voz.
      El resultado sale con weak=True para transcribir con un VAD mas permisivo.
    - pre_roll: se antepone el audio reciente (no silenciado) para no perder el comienzo de la frase
      si la persona arranco justo antes del pedido.
    """

    def __init__(self, start_timeout=8.0, end_silence=1.1, max_speech=15.0, min_speech=0.25,
                 on_thr=0.5, off_thr=0.35, onset_chunks=3, pre_roll: Optional[List[np.ndarray]] = None,
                 weak_prob=0.12, weak_snr=4.0):
        self.start_timeout = float(start_timeout)
        self.end_silence = float(end_silence)
        self.max_speech = float(max_speech)
        self.min_speech = float(min_speech)
        self.on_thr, self.off_thr = float(on_thr), float(off_thr)
        self.onset_chunks = int(onset_chunks)
        self.weak_prob, self.weak_snr = float(weak_prob), float(weak_snr)
        self.weak = False
        self.frames: List[np.ndarray] = []
        self.pending: deque = deque(maxlen=max(1, int(0.5 / CHUNK_SEC)))  # audio previo al onset
        if pre_roll:
            self.pending.extend(pre_roll)
        self.state = "WAIT"
        self.wait_t = 0.0          # tiempo esperando voz (sin contar mute)
        self.speech_t = 0.0        # tiempo grabando desde el onset
        self.voiced_t = 0.0        # tiempo con voz efectiva
        self.silence_t = 0.0
        self.run = 0
        self.peak = 0.0
        self.max_prob = 0.0        # diagnostico: si 'no_speech' con max_prob alto, revisar umbrales
        self.max_rms = 0.0

    def feed(self, chunk: np.ndarray, prob: float, muted: bool = False, floor: Optional[float] = None
             ) -> Optional[Dict]:
        if muted:
            chunk = np.zeros_like(chunk)
            prob = 0.0
        else:
            rms = float(np.sqrt(np.mean(chunk ** 2))) if len(chunk) else 0.0
            self.max_prob = max(self.max_prob, prob)
            self.max_rms = max(self.max_rms, rms)
            if floor and prob < self.on_thr and prob >= self.weak_prob and rms >= self.weak_snr * floor:
                prob = self.on_thr          # voz debil pero clara sobre el ruido
                self.weak = True
        if self.state == "WAIT":
            if not muted:
                self.wait_t += CHUNK_SEC
            self.pending.append(chunk)
            self.run = self.run + 1 if prob >= self.on_thr else 0
            if self.run >= self.onset_chunks:
                self.state = "SPEECH"
                self.frames = list(self.pending)
                self.pending.clear()
                self.voiced_t = self.run * CHUNK_SEC
                self.speech_t = self.voiced_t
                return None
            if self.wait_t >= self.start_timeout:
                self.state = "DONE"
                return {"status": "no_speech", "audio": np.zeros(0, np.float32), "speech_sec": 0.0,
                        "wait_sec": self.wait_t, "reason": "start_timeout", "peak": 0.0}
            return None

        if self.state != "SPEECH":
            return None
        self.frames.append(chunk)
        self.speech_t += CHUNK_SEC
        self.peak = max(self.peak, float(np.max(np.abs(chunk))) if len(chunk) else 0.0)
        if prob >= self.off_thr:
            self.silence_t = 0.0
            if prob >= self.on_thr:
                self.voiced_t += CHUNK_SEC
        else:
            self.silence_t += CHUNK_SEC
        reason = None
        if self.silence_t >= self.end_silence:
            reason = "end_silence"
        elif self.speech_t >= self.max_speech:
            reason = "max_speech"
        if reason is None:
            return None
        self.state = "DONE"
        if self.voiced_t < self.min_speech:
            return {"status": "no_speech", "audio": np.zeros(0, np.float32), "speech_sec": self.voiced_t,
                    "wait_sec": self.wait_t, "reason": "too_short", "peak": self.peak}
        # Recortar la cola de silencio (deja 0.3 s): menos audio para Whisper
        keep_tail = int(max(0.0, self.silence_t - 0.3) / CHUNK_SEC)
        frames = self.frames[:len(self.frames) - keep_tail] if keep_tail else self.frames
        return {"status": "ok", "audio": np.concatenate(frames).astype(np.float32),
                "speech_sec": self.voiced_t, "wait_sec": self.wait_t, "reason": reason, "peak": self.peak,
                "weak": self.weak}


# ---------------------------------------------------------------------------- filtros de texto
HALLUCINATIONS = {
    'you', 'thank you', 'thank you.', 'thanks for watching', 'thanks for watching!', 'thank you for watching',
    'bye', 'bye.', 'bye-bye', '.', '..', '...', 'uh', 'um', 'hmm', 'mm', 'oh',
    'subtitles by the amara.org community', 'www.mooji.org', 'please subscribe',
}
PROMPT_COMMON = {'and', 'the', 'my', 'is', 'name', 'favorite', 'drink', 'a', 'to', 'i', 'me'}


def collapse_repeats(text: str, max_rep: int = 2) -> str:
    """Corta las repeticiones en bucle de Whisper ("Robin, Robin, Robin, ...", frases duplicadas)."""
    words = text.split()
    for n in range(1, 9):                        # n-gramas de 1 a 8 palabras
        out, i = [], 0
        while i < len(words):
            gram = [w.strip('.,!?').lower() for w in words[i:i + n]]
            j, rep = i + n, 1
            while j + n <= len(words) and [w.strip('.,!?').lower() for w in words[j:j + n]] == gram:
                rep += 1
                j += n
            if rep > max_rep and len(gram) == n:
                out.extend(words[i:i + n])
                i = j
            else:
                out.append(words[i])
                i += 1
        words = out
    return ' '.join(words)


def is_prompt_echo(text: str, prompt: str) -> bool:
    """Con audio sin voz Whisper "lee" su initial_prompt (lista de nombres y bebidas)."""
    prompt_words = set(re.findall(r'[a-z]+', prompt.lower())) - PROMPT_COMMON
    heard = [w for w in re.findall(r'[a-z]+', text.lower()) if w not in PROMPT_COMMON]
    if len(heard) < 5:
        return False
    in_prompt = [w for w in heard if w in prompt_words]
    return len(in_prompt) / len(heard) > 0.85 and len(set(in_prompt)) >= 5


def clean_transcript(text: str, prompt: str = '') -> (str, Optional[str]):
    """Devuelve (texto_limpio, motivo_descarte | None)."""
    text = collapse_repeats(re.sub(r'\s+', ' ', text or '').strip())
    norm = text.lower().strip()
    if not text:
        return '', 'empty'
    if norm in HALLUCINATIONS or norm.strip('.!? ') in HALLUCINATIONS:
        return text, 'hallucination'
    if (text.startswith('[') and text.endswith(']')) or (text.startswith('(') and text.endswith(')')):
        return text, 'non_speech_tag'
    if not re.search(r'[a-zA-ZÀ-ſ]{2,}', text) and norm.strip('.!? ') not in ('i', 'a'):
        return text, 'no_words'
    if prompt and is_prompt_echo(text, prompt):
        return text, 'prompt_echo'
    return text, None
