#!/usr/bin/env python3
"""
Nodo ASR (Whisper + WebRTC VAD) del cabezal Jetson para RoboCup @Home.

Adaptado de las ideas de fbot_hri (fbot_speech/speech_recognizer.py):
  * initial_prompt con nombres/bebidas comunes (mejora el reconocimiento de nombres).
  * ventana de escucha con timeout, silencio de cierre mas largo y duracion minima.
  * descarte de alucinaciones tipicas de Whisper sobre ruido.

Topicos / servicios:
  PUB  /voice/transcription  (std_msgs/String)   texto reconocido
  PUB  /voice/status         (std_msgs/String)   IDLE | LISTENING | PROCESSING | MUTED | NO_MIC
  PUB  /voice/mic_level      (std_msgs/Float32)  RMS 0..1 (salud del microfono)
  SUB  /voice/mute           (std_msgs/Bool)     true mientras el robot habla (anti-eco)
  SUB  /voice/initial_prompt (std_msgs/String)   cambia el vocabulario esperado en caliente
  SRV  /voice/listen_trigger (std_srvs/Trigger)  abre una ventana de escucha forzada
"""
import re
import time
import collections
import threading
import numpy as np
import sounddevice as sd
import webrtcvad

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger

from faster_whisper import WhisperModel


DEFAULT_PROMPT = (
    "Hello, my name is James. Names: James Michael Robert John David William Richard "
    "Joseph Thomas Christopher Mary Patricia Jennifer Linda Elizabeth Barbara Susan "
    "Jessica Karen Sarah Charlie Alex Adam Axel Amelia Angel Ava Charlotte Hunter Max "
    "Olivia Paris Robin Skyler. Drinks: coke, orange juice, milk, water, tea, coffee, "
    "lemonade, beer, wine, juice."
)

# Alucinaciones frecuentes de Whisper cuando solo hay ruido/silencio.
HALLUCINATIONS = {
    'you', 'thank you', 'thank you.', 'thanks for watching', 'thanks for watching!',
    'bye', 'bye.', 'bye-bye', '.', '..', '...', 'uh', 'um', 'hmm', 'mm',
    'subtitles by the amara.org community', 'www.mooji.org',
}


class WhisperASRNode(Node):
    def __init__(self):
        super().__init__('whisper_asr_node')

        # ---------------- Parametros ----------------
        self.declare_parameter('model_size', 'tiny')
        self.declare_parameter('language', 'auto')
        self.declare_parameter('energy_threshold', 0.02)
        self.declare_parameter('vad_mode', 2)
        self.declare_parameter('silence_duration_sec', 1.0)
        self.declare_parameter('max_utterance_sec', 9.0)
        self.declare_parameter('min_utterance_sec', 0.5)
        self.declare_parameter('trigger_wait_sec', 7.0)
        self.declare_parameter('unmute_guard_sec', 0.45)
        self.declare_parameter('beam_size', 3)
        self.declare_parameter('no_speech_threshold', 0.6)
        self.declare_parameter('min_avg_logprob', -1.1)
        self.declare_parameter('mic_name_hint', 'rode')
        self.declare_parameter('initial_prompt', DEFAULT_PROMPT)

        gp = self.get_parameter
        self.model_size = gp('model_size').get_parameter_value().string_value
        self.language = gp('language').get_parameter_value().string_value
        self.energy_threshold = gp('energy_threshold').get_parameter_value().double_value
        self.vad_mode = gp('vad_mode').get_parameter_value().integer_value
        self.silence_duration = gp('silence_duration_sec').get_parameter_value().double_value
        self.max_utterance_sec = gp('max_utterance_sec').get_parameter_value().double_value
        self.min_utterance_sec = gp('min_utterance_sec').get_parameter_value().double_value
        self.trigger_wait_sec = gp('trigger_wait_sec').get_parameter_value().double_value
        self.unmute_guard_sec = gp('unmute_guard_sec').get_parameter_value().double_value
        self.beam_size = gp('beam_size').get_parameter_value().integer_value
        self.no_speech_threshold = gp('no_speech_threshold').get_parameter_value().double_value
        self.min_avg_logprob = gp('min_avg_logprob').get_parameter_value().double_value
        self.mic_name_hint = gp('mic_name_hint').get_parameter_value().string_value.lower()
        self.initial_prompt = gp('initial_prompt').get_parameter_value().string_value

        self.get_logger().info(f'Loading Whisper model: {self.model_size} (lang: {self.language})...')
        self.model = WhisperModel(self.model_size, device='cpu', compute_type='int8')
        self.get_logger().info('Whisper model loaded successfully!')

        # ---------------- ROS ----------------
        self.pub_transcription = self.create_publisher(String, '/voice/transcription', 10)
        self.pub_status = self.create_publisher(String, '/voice/status', 10)
        self.pub_level = self.create_publisher(Float32, '/voice/mic_level', 10)
        self.create_subscription(Bool, '/voice/mute', self.mute_callback, 10)
        self.create_subscription(String, '/voice/initial_prompt', self.prompt_callback, 10)
        self.srv_trigger = self.create_service(Trigger, '/voice/listen_trigger', self.handle_listen_trigger)

        # ---------------- Audio ----------------
        # VAD y Whisper a 16 kHz, frames de 30 ms = 480 muestras.
        self.target_rate = 16000
        self.frame_duration_ms = 30
        self.frame_size = int(self.target_rate * (self.frame_duration_ms / 1000.0))
        # El RODE VideoMic GO II opera nativamente a 48 kHz.
        self.capture_rate = 48000
        self.capture_blocksize = int(self.frame_size * 3)  # 1440 muestras = 30 ms @ 48 kHz

        self.vad = webrtcvad.Vad(self.vad_mode)
        self.is_running = True
        self.current_status = 'IDLE'
        self.manual_trigger_requested = False

        self.muted = False
        self.last_mute_active = 0.0
        self.guard_until = 0.0
        self.last_rms = 0.0
        self.ambient_rms = 0.04

        self.audio_thread = threading.Thread(target=self.audio_loop, daemon=True)
        self.audio_thread.start()

        self.status_timer = self.create_timer(0.5, self.publish_heartbeat)
        self.level_timer = self.create_timer(0.2, self.publish_level)

    # ------------------------------------------------------------------
    # ROS helpers
    # ------------------------------------------------------------------
    def publish_heartbeat(self):
        msg = String()
        msg.data = 'MUTED' if self.muted else self.current_status
        self.pub_status.publish(msg)

    def publish_level(self):
        msg = Float32()
        msg.data = float(self.last_rms)
        self.pub_level.publish(msg)

    def set_status(self, status):
        self.current_status = status
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)

    def mute_callback(self, msg: Bool):
        was_muted = self.muted
        self.muted = bool(msg.data)
        now = time.time()
        if self.muted:
            self.last_mute_active = now
        elif was_muted:
            # Cola de reverberacion de la voz del robot
            self.guard_until = now + self.unmute_guard_sec

    def prompt_callback(self, msg: String):
        text = msg.data.strip()
        self.initial_prompt = text if text else DEFAULT_PROMPT
        self.get_logger().info(f'initial_prompt actualizado ({len(self.initial_prompt)} chars)')

    def handle_listen_trigger(self, request, response):
        self.manual_trigger_requested = True
        response.success = True
        response.message = f'Triggered recording. Current status: {self.current_status}'
        self.get_logger().info('External listen trigger received!')
        return response

    # ------------------------------------------------------------------
    # Audio
    # ------------------------------------------------------------------
    def find_microphone_device(self):
        try:
            try:
                sd._terminate()
                sd._initialize()
            except Exception:
                pass
            devices = sd.query_devices()
            for i, d in enumerate(devices):
                name = d['name'].lower()
                ch = d['max_input_channels']
                if ('videomic' in name or 'rode' in name or 'røde' in name or self.mic_name_hint in name) and ch > 0:
                    self.get_logger().info(f'Found microphone: [{i}] {d["name"]}', throttle_duration_sec=30.0)
                    return i
            for i, d in enumerate(devices):
                name = d['name'].lower()
                ch = d['max_input_channels']
                if 'usb' in name and ch > 0:
                    self.get_logger().info(f'Found USB microphone: [{i}] {d["name"]}', throttle_duration_sec=30.0)
                    return i
        except Exception as e:
            self.get_logger().warn(f'Error querying audio devices: {e}')
        return None

    @staticmethod
    def to_16k(chunk_48k: np.ndarray) -> np.ndarray:
        """48 kHz -> 16 kHz con filtro anti-alias promediado (boxcar filter)."""
        n = (len(chunk_48k) // 3) * 3
        return chunk_48k[:n].astype(np.float32).reshape(-1, 3).mean(axis=1).astype(np.int16)

    def audio_loop(self):
        ring_buffer = collections.deque(maxlen=int(300 / self.frame_duration_ms))
        triggered = False
        heard_speech = False
        voiced_frames = []
        silence_start_time = None
        trigger_start_time = None

        audio_queue = collections.deque(maxlen=400)

        def callback(indata, frames, time_info, status):
            audio_queue.append(indata.copy())

        def reset_state():
            nonlocal triggered, heard_speech, voiced_frames, silence_start_time, trigger_start_time
            triggered = False
            heard_speech = False
            voiced_frames = []
            silence_start_time = None
            trigger_start_time = None
            ring_buffer.clear()

        while rclpy.ok() and self.is_running:
            mic_device = self.find_microphone_device()
            if mic_device is None:
                self.set_status('NO_MIC')
                self.get_logger().warn('No se encontro microfono USB. Reintentando en 2s...', throttle_duration_sec=10.0)
                time.sleep(2.0)
                continue
            try:
                self.get_logger().info(f'Opening audio stream on device {mic_device} @ {self.capture_rate}Hz...')
                with sd.InputStream(
                    device=mic_device,
                    samplerate=self.capture_rate,
                    channels=1,
                    dtype='int16',
                    callback=callback,
                    blocksize=self.capture_blocksize,
                ):
                    self.set_status('IDLE')
                    self.get_logger().info('Listening for speech (48kHz -> 16kHz con anti-alias)...')

                    while rclpy.ok() and self.is_running:
                        if not audio_queue:
                            time.sleep(0.005)
                            continue

                        chunk_48k = audio_queue.popleft().flatten()
                        chunk = self.to_16k(chunk_48k)
                        if len(chunk) != self.frame_size:
                            continue
                        pcm_bytes = chunk.tobytes()

                        rms = float(np.sqrt(np.mean(chunk.astype(np.float32) ** 2)) / 32768.0)
                        self.last_rms = rms
                        now = time.time()

                        # --- Anti-eco: el robot esta hablando ---
                        if self.muted:
                            self.last_mute_active = now
                        if self.muted or now < self.guard_until:
                            if triggered or voiced_frames:
                                reset_state()
                            if self.current_status != 'IDLE':
                                self.current_status = 'IDLE'
                            continue

                        is_vad_speech = self.vad.is_speech(pcm_bytes, self.target_rate)

                        if not triggered:
                            self.ambient_rms = 0.96 * self.ambient_rms + 0.04 * rms
                            speech_threshold = max(self.energy_threshold, self.ambient_rms * 1.30 + 0.012)
                        else:
                            speech_threshold = max(self.energy_threshold, self.ambient_rms * 1.15 + 0.008)

                        is_speech = is_vad_speech and (rms >= speech_threshold)

                        # --- Disparo manual (/voice/listen_trigger) ---
                        if self.manual_trigger_requested and not triggered:
                            self.manual_trigger_requested = False
                            triggered = True
                            heard_speech = False
                            trigger_start_time = now
                            silence_start_time = None
                            voiced_frames = []
                            self.set_status('LISTENING')
                            self.get_logger().info('Triggered listening...')
                            ring_buffer.clear()

                        if not triggered:
                            ring_buffer.append((chunk, is_speech))
                            num_voiced = len([f for f, speech in ring_buffer if speech])
                            if num_voiced >= 0.5 * ring_buffer.maxlen:
                                triggered = True
                                heard_speech = True
                                trigger_start_time = now
                                self.set_status('LISTENING')
                                self.get_logger().info(f'Voice detected! (RMS: {rms:.4f} > {speech_threshold:.4f}) Recording...')
                                for f, s in ring_buffer:
                                    voiced_frames.append(f)
                                ring_buffer.clear()
                                silence_start_time = None
                            continue

                        # ---- Grabando ----
                        voiced_frames.append(chunk)
                        duration = now - trigger_start_time

                        if is_speech:
                            heard_speech = True
                            silence_start_time = None
                            is_silent = False
                        else:
                            if silence_start_time is None:
                                silence_start_time = now
                            is_silent = (now - silence_start_time >= self.silence_duration)

                        reached_max = duration >= self.max_utterance_sec

                        # Disparo manual sin que el humano haya hablado aun: esperar hasta trigger_wait_sec
                        if not heard_speech:
                            if duration >= self.trigger_wait_sec:
                                self.get_logger().info('Ventana de escucha agotada sin voz.')
                                reset_state()
                                self.set_status('IDLE')
                            continue

                        if (is_silent and heard_speech) or reached_max:
                            reason = 'Max duration' if reached_max else 'Silence'
                            self.set_status('PROCESSING')
                            self.get_logger().info(f'End of utterance ({reason}, total {duration:.2f}s). Transcribing...')

                            audio_int16 = np.concatenate(voiced_frames) if voiced_frames else np.array([], dtype=np.int16)
                            t_start = trigger_start_time
                            reset_state()

                            if len(audio_int16) >= int(self.min_utterance_sec * self.target_rate):
                                audio_float = audio_int16.astype(np.float32) / 32768.0
                                threading.Thread(
                                    target=self.transcribe_worker, args=(audio_float, t_start), daemon=True
                                ).start()
                            else:
                                self.set_status('IDLE')

            except Exception as e:
                self.set_status('NO_MIC')
                self.get_logger().warn(f'InputStream exception: {e}. Retrying in 2s...', throttle_duration_sec=3.0)
                time.sleep(2.0)

    # ------------------------------------------------------------------
    # Transcripcion
    # ------------------------------------------------------------------
    def transcribe_worker(self, audio_data, t_start):
        t0 = time.time()
        try:
            lang = None if self.language.lower() in ['auto', 'none', '', 'multi', 'bi'] else self.language
            if self.model_size.endswith('.en'):
                lang = 'en'

            segments, info = self.model.transcribe(
                audio_data,
                language=lang,
                initial_prompt=self.initial_prompt,
                beam_size=self.beam_size,
                vad_filter=False,
                condition_on_previous_text=False,
                no_speech_threshold=self.no_speech_threshold,
            )
            segments = list(segments)

            # Normaliza idiomas raros auto-detectados a es/en
            if lang is None and info.language not in ['en', 'es']:
                target_lang = 'es' if info.language in ['pt', 'it', 'gl', 'ca'] else 'en'
                segments, info = self.model.transcribe(
                    audio_data, language=target_lang, initial_prompt=self.initial_prompt,
                    beam_size=self.beam_size, vad_filter=False, condition_on_previous_text=False,
                )
                segments = list(segments)

            # Filtro de calidad por segmento (ruido -> basura)
            good = []
            for seg in segments:
                if seg.no_speech_prob > self.no_speech_threshold and seg.avg_logprob < self.min_avg_logprob:
                    continue
                if seg.avg_logprob < -1.6:
                    continue
                good.append(seg.text)
            text = ' '.join(good).strip()
            dt = time.time() - t0

            norm = re.sub(r'\s+', ' ', text.lower()).strip()
            is_junk = (
                not text
                or norm in HALLUCINATIONS
                or not re.search(r'[a-zA-Z\u00c0-\u017f]{2,}', text)
                or (text.startswith('[') and text.endswith(']'))
                or (text.startswith('(') and text.endswith(')'))
            )

            # El robot hablo durante esta frase: es eco de su propia voz
            echoed = self.last_mute_active >= t_start

            if is_junk:
                self.get_logger().info(f'Descartado (ruido/alucinacion) [{dt:.2f}s]: "{text}"')
            elif echoed:
                self.get_logger().info(f'Descartado (eco del robot) [{dt:.2f}s]: "{text}"')
            else:
                self.get_logger().info(
                    f'Transcribed ({dt:.2f}s) [{info.language} prob={info.language_probability:.2f}]: "{text}"')
                msg = String()
                msg.data = text
                self.pub_transcription.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')
        finally:
            self.set_status('IDLE')


def main(args=None):
    rclpy.init(args=args)
    node = WhisperASRNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.is_running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
