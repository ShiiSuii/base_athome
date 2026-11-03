#!/usr/bin/env python3
"""
Nodo ASR (faster-whisper + Silero VAD en streaming) del cabezal Jetson para RoboCup @Home.

Escucha POR PEDIDO con identificador y fin de habla automatico (ver asr_core.py):
  1. El cliente pide escuchar (topico /voice/listen_request con JSON, o el servicio Trigger).
  2. El nodo espera a que la persona empiece a hablar (hasta start_timeout, por defecto 8 s),
     graba mientras habla y corta ~1.1 s despues de que termina (no una ventana fija de 6 s).
  3. SIEMPRE publica un resultado en /voice/result para ese id: 'ok' con el texto, o
     'no_speech' / 'rejected' / 'cancelled' / 'error'. El cliente nunca tiene que adivinar por timeout.

Topicos / servicios:
  SUB  /voice/listen_request (std_msgs/String)  JSON {"id", "prompt", "start_timeout", "end_silence",
                                                "max_speech"}; un pedido nuevo reemplaza al anterior
  SRV  /voice/listen_trigger (std_srvs/Trigger)  pedido con parametros por defecto; message = JSON {"id"}
  SUB  /voice/cancel         (std_msgs/String)   id a cancelar ("" = el activo)
  PUB  /voice/result         (std_msgs/String)   JSON {"id", "event": listening|speech_start|speech_end|result,
                                                "status", "text", "reason", "speech_sec", "wait_sec", "asr_sec"}
  PUB  /voice/transcription  (std_msgs/String)   texto de cada resultado 'ok' (compatibilidad)
  PUB  /voice/status         (std_msgs/String)   IDLE | LISTENING | PROCESSING | MUTED | NO_MIC
  PUB  /voice/mic_level      (std_msgs/Float32)  RMS 0..1 (salud del microfono)
  SUB  /voice/mute           (std_msgs/Bool)     true mientras el robot habla: esos bloques se anulan,
                                                pero el pedido de escucha sigue abierto
  SUB  /voice/initial_prompt (std_msgs/String)   vocabulario por defecto (cada pedido puede traer el suyo)
"""
import json
import wave
import queue
import re
import subprocess
import threading
import time
from collections import deque

import numpy as np
import sounddevice as sd

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32
from std_srvs.srv import Trigger

from faster_whisper import WhisperModel

from audio_asr.asr_core import (CHUNK_SEC, SAMPLE_RATE, VAD_CHUNK, Endpointer, EnergyVAD, SileroStream,
                                clean_transcript)

DEFAULT_PROMPT = (
    "Guests introduce themselves at a party. Names: Adel, Angel, Axel, Charlie, Jane, Jules, Morgan, "
    "Paris, Robin, Simone. Drinks: juice pack, cola, milk, orange juice, tropical juice, red wine, iced tea."
)


class WhisperASRNode(Node):
    def __init__(self):
        super().__init__('whisper_asr_node')
        p = self.declare_parameter
        p('model_size', 'base.en')
        p('language', 'en')
        p('beam_size', 3)
        p('cpu_threads', 4)
        p('start_timeout_sec', 8.0)        # espera maxima a que la persona EMPIECE a hablar
        p('end_silence_sec', 1.1)          # silencio que cierra la frase (fbot: 1.2, FRIDA: 1.5)
        p('max_speech_sec', 15.0)
        p('vad_on_threshold', 0.5)
        p('vad_off_threshold', 0.35)
        p('continuous', False)             # True: transcribe toda voz sin pedido (demos)
        p('unmute_guard_sec', 0.35)        # cola de reverberacion del parlante
        p('max_mute_sec', 25.0)            # failsafe: ninguna frase del robot dura tanto
        p('mic_gain_percent', 100)
        p('auto_gain', True)               # baja la ganancia si la voz satura (clipping)
        p('no_speech_threshold', 0.6)
        p('min_avg_logprob', -1.0)
        p('mic_name_hint', 'rode')
        p('initial_prompt', DEFAULT_PROMPT)
        # parametros viejos (ventana fija): se aceptan para no romper launch files, ya no se usan
        for old, val in (('energy_threshold', 0.02), ('vad_mode', 2), ('silence_duration_sec', 1.0),
                         ('max_utterance_sec', 9.0), ('min_utterance_sec', 0.5), ('trigger_wait_sec', 7.0),
                         ('listen_window_sec', 6.0), ('max_ambient_rms', 0.15)):
            p(old, val)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.model_size = g('model_size')
        self.language = g('language')
        self.beam_size = int(g('beam_size'))
        self.defaults = {'start_timeout': float(g('start_timeout_sec')),
                         'end_silence': float(g('end_silence_sec')),
                         'max_speech': float(g('max_speech_sec'))}
        self.on_thr, self.off_thr = float(g('vad_on_threshold')), float(g('vad_off_threshold'))
        self.continuous = bool(g('continuous'))
        self.unmute_guard_sec = float(g('unmute_guard_sec'))
        self.max_mute_sec = float(g('max_mute_sec'))
        self.gain_percent = int(g('mic_gain_percent'))
        self.auto_gain = bool(g('auto_gain'))
        self.no_speech_threshold = float(g('no_speech_threshold'))
        self.min_avg_logprob = float(g('min_avg_logprob'))
        self.mic_name_hint = str(g('mic_name_hint')).lower()
        self.initial_prompt = str(g('initial_prompt'))

        self.get_logger().info(f'Cargando Whisper {self.model_size} (idioma {self.language})...')
        self.model = WhisperModel(self.model_size, device='cpu', compute_type='int8',
                                  cpu_threads=int(g('cpu_threads')))
        try:
            self.vad = SileroStream()
            self.vad_name = 'silero'
        except Exception as e:  # sin onnxruntime: respaldo por energia (peor, pero no deja sordo al robot)
            self.get_logger().error(f'Silero VAD no disponible ({e}): uso VAD por energia')
            self.vad = EnergyVAD()
            self.vad_name = 'energia'
        self._warmup()

        # ---------------- ROS ----------------
        self.pub_transcription = self.create_publisher(String, '/voice/transcription', 10)
        self.pub_result = self.create_publisher(String, '/voice/result', 20)
        self.pub_status = self.create_publisher(String, '/voice/status', 10)
        self.pub_level = self.create_publisher(Float32, '/voice/mic_level', 10)
        self.create_subscription(Bool, '/voice/mute', self.mute_callback, 10)
        self.create_subscription(String, '/voice/initial_prompt', self.prompt_callback, 10)
        self.create_subscription(String, '/voice/listen_request', self.request_callback, 10)
        self.create_subscription(String, '/voice/cancel', self.cancel_callback, 10)
        # Prueba reproducible: mezcla un WAV (ruta en la Jetson) con el microfono EN VIVO, como si una
        # persona hablara delante del robot. Lo usa asr_benchmark --inject. No reemplaza al microfono.
        self.create_subscription(String, '/voice/debug_inject', self.inject_callback, 10)
        self.inject = None                 # np.ndarray 16 kHz pendiente de mezclar
        self.create_service(Trigger, '/voice/listen_trigger', self.handle_listen_trigger)

        # ---------------- estado ----------------
        self.lock = threading.Lock()
        self.next_request = None           # pedido recibido, lo toma el hilo de audio
        self.cancel_id = None
        self.active = None                 # {'req': dict, 'ep': Endpointer, 'started': bool}
        self.trigger_count = 0
        self.asr_queue: "queue.Queue" = queue.Queue()
        self.processing = False
        self.muted = False
        self.mute_since = 0.0
        self.guard_until = 0.0
        self.last_rms = 0.0
        self.mic_ok = False
        self.is_running = True
        self.noise_floor = 0.01            # RMS del ruido de fondo (se adapta solo sin voz)

        threading.Thread(target=self.audio_loop, daemon=True).start()
        threading.Thread(target=self.asr_worker, daemon=True).start()
        self.create_timer(0.5, self.publish_status)
        self.create_timer(0.2, lambda: self.pub_level.publish(Float32(data=float(self.last_rms))))
        self.create_timer(30.0, self.set_gain)  # el USB o pulseaudio pueden cambiarla
        self.get_logger().info(f'ASR listo: VAD {self.vad_name}, fin de habla {self.defaults["end_silence"]} s, '
                               f'espera de voz {self.defaults["start_timeout"]} s')

    def _warmup(self):
        """La primera decodificacion es lenta (carga de pesos/caches): hacerla al arrancar."""
        try:
            t = time.time()
            list(self.model.transcribe(np.zeros(SAMPLE_RATE, np.float32), language=self._lang(), beam_size=1)[0])
            self.get_logger().info(f'Whisper precalentado en {time.time() - t:.1f} s')
        except Exception as e:
            self.get_logger().warn(f'Precalentamiento fallo: {e}')

    def _lang(self):
        if self.model_size.endswith('.en'):
            return 'en'
        return None if self.language.lower() in ('auto', 'none', '', 'multi', 'bi') else self.language

    # ------------------------------------------------------------------ ROS callbacks
    def publish_status(self):
        if not self.mic_ok:
            s = 'NO_MIC'
        elif self.active is not None and self.active['req']['id']:
            s = 'LISTENING'
        elif self.processing or not self.asr_queue.empty():
            s = 'PROCESSING'
        elif self.muted:
            s = 'MUTED'
        else:
            s = 'IDLE'
        self.pub_status.publish(String(data=s))

    def emit(self, rid, event, **kw):
        if not rid and event != 'result':
            return
        d = {'id': rid, 'event': event, 't': round(time.time(), 3)}
        d.update(kw)
        self.pub_result.publish(String(data=json.dumps(d)))

    def mute_callback(self, msg: Bool):
        now = time.time()
        if msg.data and not self.muted:
            self.mute_since = now
        if not msg.data and self.muted:
            self.guard_until = now + self.unmute_guard_sec
        self.muted = bool(msg.data)

    def prompt_callback(self, msg: String):
        text = msg.data.strip() or DEFAULT_PROMPT
        if text != self.initial_prompt:
            self.initial_prompt = text
            self.get_logger().info(f'initial_prompt por defecto actualizado ({len(text)} chars)')

    def _queue_request(self, req: dict):
        with self.lock:
            self.next_request = req
        self.get_logger().info(f'Pedido de escucha {req["id"]} (espera {req["start_timeout"]:.0f} s, '
                               f'cierre {req["end_silence"]:.1f} s)')

    def request_callback(self, msg: String):
        try:
            d = json.loads(msg.data) if msg.data.strip().startswith('{') else {}
        except ValueError:
            d = {}
        self.trigger_count += 1
        req = dict(self.defaults)
        for k in ('start_timeout', 'end_silence', 'max_speech'):
            if k in d:
                try:
                    req[k] = float(d[k])
                except (TypeError, ValueError):
                    pass
        req['id'] = str(d.get('id') or f'r{self.trigger_count}')
        req['prompt'] = str(d.get('prompt') or '')
        self._queue_request(req)

    def handle_listen_trigger(self, request, response):
        self.trigger_count += 1
        req = dict(self.defaults, id=f't{self.trigger_count}', prompt='')
        self._queue_request(req)
        response.success = True
        response.message = json.dumps({'id': req['id']})
        return response

    def cancel_callback(self, msg: String):
        with self.lock:
            self.cancel_id = msg.data.strip() or '*'

    # ------------------------------------------------------------------ microfono
    def _card(self):
        try:
            with open('/proc/asound/cards', encoding='utf-8', errors='ignore') as f:
                m = re.search(r'^\s*(\d+)\s+\[[^\]]*\]:.*(?:RØDE|RODE|VideoMic)', f.read(), re.M | re.I)
            return m.group(1) if m else None
        except OSError:
            return None

    def set_gain(self):
        card = self._card()
        if card is None:
            return
        try:
            subprocess.run(['amixer', '-c', card, 'sset', 'Mic', f'{self.gain_percent}%', 'cap'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=3)
            if getattr(self, '_gain_logged', None) != (card, self.gain_percent):
                self._gain_logged = (card, self.gain_percent)
                self.get_logger().info(f'Ganancia del microfono (tarjeta {card}) en {self.gain_percent}%')
        except Exception as e:
            self.get_logger().warn(f'No se pudo fijar la ganancia: {e}')

    def _adapt_gain(self, peak):
        """Saturacion = Whisper entiende mal. Baja 10 % si satura; vuelve a subir si la voz llega debil."""
        if not self.auto_gain:
            return
        new = self.gain_percent
        if peak >= 0.97:
            new = max(60, self.gain_percent - 10)
        elif peak < 0.08:
            new = min(100, self.gain_percent + 10)
        if new != self.gain_percent:
            self.get_logger().warn(f'Pico de voz {peak:.2f}: ganancia {self.gain_percent}% -> {new}%')
            self.gain_percent = new
            self.set_gain()

    def find_microphone_device(self):
        try:
            try:
                sd._terminate()
                sd._initialize()
            except Exception:
                pass
            devices = sd.query_devices()
            for want in (('videomic', 'rode', 'røde', self.mic_name_hint), ('usb',)):
                for i, d in enumerate(devices):
                    if d['max_input_channels'] > 0 and any(w in d['name'].lower() for w in want):
                        self.get_logger().info(f'Microfono: [{i}] {d["name"]}', throttle_duration_sec=30.0)
                        return i
        except Exception as e:
            self.get_logger().warn(f'Error buscando microfono: {e}')
        return None

    # ------------------------------------------------------------------ audio
    def audio_loop(self):
        capture_rate, decim = 48000, 3                 # el RØDE VideoMic GO II trabaja a 48 kHz
        q = deque(maxlen=600)                          # ~19 s de margen si el hilo se atrasa
        stats = {'last': time.time(), 'overflows': 0}
        preroll = deque(maxlen=int(0.6 / CHUNK_SEC))   # (chunk, prob, muted) recientes

        def callback(indata, frames, time_info, status):
            stats['last'] = time.time()
            if status and status.input_overflow:
                stats['overflows'] += 1
            q.append(indata[:, 0].copy())

        while rclpy.ok() and self.is_running:
            dev = self.find_microphone_device()
            if dev is None:
                self.mic_ok = False
                self.get_logger().warn('Sin microfono USB; reintento en 2 s', throttle_duration_sec=10.0)
                self._fail_active('no_mic')
                time.sleep(2.0)
                continue
            try:
                self.set_gain()
                stats['last'] = time.time()
                with sd.InputStream(device=dev, samplerate=capture_rate, channels=1, dtype='int16',
                                    callback=callback, blocksize=VAD_CHUNK * decim):
                    self.mic_ok = True
                    self.vad.reset()
                    self.get_logger().info(f'Captura abierta en el dispositivo {dev} @ {capture_rate} Hz')
                    while rclpy.ok() and self.is_running:
                        if not q:
                            if time.time() - stats['last'] > 2.0:
                                self.get_logger().warn('Sin audio del microfono por >2 s: reabro la captura')
                                break
                            time.sleep(0.004)
                            continue
                        if stats['overflows']:
                            self.get_logger().warn(f"Overflow de captura x{stats['overflows']}",
                                                   throttle_duration_sec=30.0)
                            stats['overflows'] = 0
                        raw = q.popleft().astype(np.float32) / 32768.0
                        n = (len(raw) // decim) * decim
                        chunk = raw[:n].reshape(-1, decim).mean(axis=1)
                        if len(chunk) != VAD_CHUNK:
                            continue
                        self.process_chunk(chunk, preroll)
            except Exception as e:
                self.mic_ok = False
                self.get_logger().warn(f'Error de captura: {e}; reintento en 2 s', throttle_duration_sec=5.0)
                time.sleep(2.0)

    def _fail_active(self, reason):
        with self.lock:
            act, self.active = self.active, None
        if act:
            self.emit(act['req']['id'], 'result', status='error', text='', reason=reason)

    def _take_requests(self, now, preroll):
        """Aplica pedidos y cancelaciones recibidos; devuelve los resultados a emitir por el cambio."""
        events = []
        with self.lock:
            req, self.next_request = self.next_request, None
            cancel, self.cancel_id = self.cancel_id, None
            old = self.active
            if old and cancel in ('*', old['req']['id']):
                self.active = None
                events.append((old['req']['id'], 'cancel'))
            elif old and req is not None:
                events.append((old['req']['id'], 'replaced'))
            if req is not None:
                recent = list(preroll)[-int(0.3 / CHUNK_SEC):]
                speaking = any(pr >= self.on_thr for _, pr, m in recent if not m)
                pre = [c for c, _, m in preroll if not m] if speaking else []
                ep = Endpointer(req['start_timeout'], req['end_silence'], req['max_speech'],
                                on_thr=self.on_thr, off_thr=self.off_thr, pre_roll=pre)
                self.active = {'req': req, 'ep': ep, 'started': False}
            elif self.active is None and self.continuous:
                ep = Endpointer(1e9, self.defaults['end_silence'], self.defaults['max_speech'],
                                on_thr=self.on_thr, off_thr=self.off_thr)
                self.active = {'req': dict(self.defaults, id='', prompt=''), 'ep': ep, 'started': False}
        for rid, why in events:
            if rid:
                self.emit(rid, 'result', status='cancelled', text='', reason=why)
        if req is not None:
            self.emit(req['id'], 'listening')

    def inject_callback(self, msg: String):
        try:
            d = json.loads(msg.data) if msg.data.strip().startswith('{') else {'path': msg.data.strip()}
            with wave.open(d['path']) as w:
                sr, ch = w.getframerate(), w.getnchannels()
                a = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768.0
            if ch > 1:
                a = a.reshape(-1, ch).mean(axis=1)
            idx = np.arange(0, len(a), sr / float(SAMPLE_RATE)).astype(int)
            self.inject = a[idx[idx < len(a)]] * float(d.get('gain', 1.0))
            self.get_logger().info(f'Inyeccion de prueba: {d["path"]} ({len(self.inject) / SAMPLE_RATE:.1f} s, '
                                   f'ganancia {d.get("gain", 1.0)})')
        except Exception as e:
            self.get_logger().warn(f'debug_inject invalido: {e}')

    def process_chunk(self, chunk, preroll):
        now = time.time()
        inj = self.inject
        if inj is not None:
            part = inj[:VAD_CHUNK]
            chunk = chunk.copy()
            chunk[:len(part)] += part
            self.inject = inj[VAD_CHUNK:] if len(inj) > VAD_CHUNK else None
        self.last_rms = float(np.sqrt(np.mean(chunk ** 2)))
        if self.muted and now - self.mute_since > self.max_mute_sec:
            self.get_logger().warn(f'Mute activo hace >{self.max_mute_sec:.0f} s: asumo que se perdio el unmute')
            self.muted = False
            self.guard_until = now + self.unmute_guard_sec
        muted = self.muted or now < self.guard_until
        prob = 0.0 if muted else self.vad(chunk)
        if not muted and prob < 0.1 and self.last_rms < 3.0 * self.noise_floor + 0.002:
            self.noise_floor = 0.99 * self.noise_floor + 0.01 * self.last_rms
        preroll.append((chunk, prob, muted))
        self._take_requests(now, preroll)

        act = self.active
        if act is None:
            return
        out = act['ep'].feed(chunk, prob, muted, floor=self.noise_floor)
        rid = act['req']['id']
        if not act['started'] and act['ep'].state == 'SPEECH':
            act['started'] = True
            self.emit(rid, 'speech_start')
            if rid:
                self.get_logger().info(f'[{rid}] voz detectada')
        if out is None:
            return
        with self.lock:
            if self.active is act:
                self.active = None
        if out['status'] != 'ok':
            if rid:
                self.get_logger().info(f'[{rid}] sin voz ({out["reason"]}, espera {out["wait_sec"]:.1f} s, '
                                       f'VAD max {act["ep"].max_prob:.2f}, RMS max {act["ep"].max_rms:.3f}, '
                                       f'ruido {self.noise_floor:.3f})')
                self.emit(rid, 'result', status='no_speech', text='', reason=out['reason'],
                          wait_sec=round(out['wait_sec'], 2), speech_sec=round(out['speech_sec'], 2))
            return
        self.emit(rid, 'speech_end', speech_sec=round(out['speech_sec'], 2))
        self.get_logger().info(f'[{rid}] fin de habla ({out["reason"]}): {len(out["audio"]) / SAMPLE_RATE:.1f} s '
                               f'de audio, pico {out["peak"]:.2f}{", voz debil" if out.get("weak") else ""}, '
                               f'ruido {self.noise_floor:.3f}')
        self._adapt_gain(out['peak'])
        self.asr_queue.put((act['req'], out, time.time()))

    # ------------------------------------------------------------------ transcripcion
    def asr_worker(self):
        while rclpy.ok() and self.is_running:
            try:
                req, out, t_end = self.asr_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            self.processing = True
            try:
                self.transcribe(req, out, t_end)
            except Exception as e:
                self.get_logger().error(f'Error de transcripcion: {e}')
                if req['id']:
                    self.emit(req['id'], 'result', status='error', text='', reason=str(e)[:120])
            finally:
                self.processing = False

    def transcribe(self, req, out, t_end):
        prompt = req.get('prompt') or self.initial_prompt
        t0 = time.time()
        segments, info = self.model.transcribe(
            out['audio'], language=self._lang(), initial_prompt=prompt, beam_size=self.beam_size,
            vad_filter=True, vad_parameters={'min_silence_duration_ms': 500, 'speech_pad_ms': 300,
                                             'threshold': 0.2 if out.get('weak') else 0.4},
            condition_on_previous_text=False, no_speech_threshold=self.no_speech_threshold,
            temperature=0.0, without_timestamps=True, max_new_tokens=96)
        good, logprobs = [], []
        for seg in segments:
            if seg.no_speech_prob > self.no_speech_threshold and seg.avg_logprob < self.min_avg_logprob:
                continue
            if seg.avg_logprob < -1.5:
                continue
            good.append(seg.text)
            logprobs.append(seg.avg_logprob)
        text, reject = clean_transcript(' '.join(good), prompt)
        asr_sec = time.time() - t0
        rid = req['id']
        meta = dict(speech_sec=round(out['speech_sec'], 2), wait_sec=round(out['wait_sec'], 2),
                    asr_sec=round(asr_sec, 2), latency_sec=round(time.time() - t_end, 2),
                    avg_logprob=round(float(np.mean(logprobs)), 3) if logprobs else None)
        if reject:
            self.get_logger().info(f'[{rid}] descartado ({reject}) [{asr_sec:.2f} s]: "{text}"')
            if rid:
                self.emit(rid, 'result', status='rejected', text='', raw=text, reason=reject, **meta)
            return
        self.get_logger().info(f'[{rid}] "{text}" (ASR {asr_sec:.2f} s)')
        if rid:
            self.emit(rid, 'result', status='ok', text=text, **meta)
        self.pub_transcription.publish(String(data=text))


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
