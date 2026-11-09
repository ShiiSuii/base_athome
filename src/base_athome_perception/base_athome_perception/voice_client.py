#!/usr/bin/env python3
"""Cliente unico del microfono (ASR de la Jetson) para todas las apps de la NUC.

Uso (no bloqueante, para maquinas de estado):
    self.voice = VoiceClient(node)
    rid = self.voice.request(prompt=vocab.whisper_prompt())   # despues del beep
    ...
    res = self.voice.poll(rid)          # None mientras escucha/transcribe
    if res: res.status in ('ok', 'no_speech', 'rejected', 'cancelled', 'error', 'timeout', 'offline')

Uso bloqueante (hilos propios): res = self.voice.listen(prompt=..., timeout=...)

Por que: antes cada app disparaba /voice/listen_trigger y tomaba "el proximo texto" que llegara a
/voice/transcription con un timeout fijo (ventana + 5 s). Si no habia voz la app se enteraba por
timeout; un texto atrasado contestaba la pregunta siguiente; y si la Jetson estaba caida el robot
decia "I could not hear you" para siempre. Ahora cada pedido tiene id y SIEMPRE termina con un
resultado explicito. Si el nodo de la Jetson es viejo (sin /voice/result) se usa el modo anterior.
"""
import itertools
import json
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Dict, Optional

from std_msgs.msg import String
from std_srvs.srv import Trigger


@dataclass
class VoiceResult:
    id: str
    status: str                 # ok | no_speech | rejected | cancelled | error | timeout | offline
    text: str = ""
    reason: str = ""
    source: str = "voice"       # voice | keyboard | screen
    info: Dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        return self.status == "ok" and bool(self.text.strip())


class VoiceClient:
    def __init__(self, node, start_timeout: float = 8.0, end_silence: float = 1.1, max_speech: float = 15.0):
        self.node = node
        self.log = node.get_logger()
        self.defaults = dict(start_timeout=start_timeout, end_silence=end_silence, max_speech=max_speech)
        self._lock = threading.Lock()
        self._results: Dict[str, VoiceResult] = {}
        self._pending: Dict[str, Dict] = {}     # id -> {'t0', 'deadline', 'legacy', 'speech'}
        self._ids = itertools.count(1)
        self._prefix = f"{os.getpid() % 10000}-"
        self.status = "UNKNOWN"
        self.status_time = 0.0
        self.new_protocol_seen = False
        self.on_event = None                    # callback(event_dict) opcional (cara, HUD)
        self.pub_request = node.create_publisher(String, "/voice/listen_request", 10)
        self.pub_cancel = node.create_publisher(String, "/voice/cancel", 10)
        self.cli_trigger = node.create_client(Trigger, "/voice/listen_trigger")
        node.create_subscription(String, "/voice/result", self._on_result, 20)
        node.create_subscription(String, "/voice/transcription", self._on_legacy_text, 10)
        node.create_subscription(String, "/voice/status", self._on_status, 10)

    # ------------------------------------------------------------------ estado
    def online(self) -> bool:
        """El nodo ASR publica /voice/status cada 0.5 s: si calla >3 s, la Jetson/ASR no esta."""
        return time.time() - self.status_time < 3.0 and self.status != "NO_MIC"

    def _on_status(self, msg: String):
        self.status = msg.data.strip().upper()
        self.status_time = time.time()

    # ------------------------------------------------------------------ pedidos
    def request(self, prompt: str = "", quiet: bool = False, **kw) -> str:
        """Abre una escucha (llamar DESPUES del beep). Devuelve el id. quiet: no loguear escuchas sin voz (monitor de "stop")."""
        rid = f"{self._prefix}{next(self._ids)}"
        p = dict(self.defaults)
        p.update({k: float(v) for k, v in kw.items() if k in p and v is not None})
        now = time.time()
        # margen: espera de voz + frase maxima + transcripcion (~2 s) + red
        deadline = now + p["start_timeout"] + p["max_speech"] + 8.0
        legacy = not self._new_protocol_available()
        with self._lock:
            for old in list(self._pending):      # un pedido nuevo reemplaza a los anteriores
                self._finish(old, VoiceResult(old, "cancelled", reason="replaced"))
            self._pending[rid] = {"t0": now, "deadline": deadline, "legacy": legacy, "speech": False, "quiet": quiet}
        if legacy:
            self._legacy_trigger(rid)
        else:
            self.pub_request.publish(String(data=json.dumps(dict(p, id=rid, prompt=prompt or ""))))
        if not quiet:
            self.log.info(f"🎤 Escuchando [{rid}]{' (modo compatible)' if legacy else ''}")
        return rid

    def _new_protocol_available(self) -> bool:
        if self.new_protocol_seen:
            return True
        try:
            return self.node.count_subscribers("/voice/listen_request") > 0
        except Exception:
            return False

    def _legacy_trigger(self, rid: str):
        if self.cli_trigger.service_is_ready() or self.cli_trigger.wait_for_service(timeout_sec=0.5):
            self.cli_trigger.call_async(Trigger.Request())
        else:
            self.log.warn("ASR sin /voice/listen_trigger ni /voice/listen_request (Jetson apagada o caida)")

    def cancel(self, rid: Optional[str] = None):
        with self._lock:
            ids = [rid] if rid else list(self._pending)
            for i in ids:
                if i in self._pending:
                    self._finish(i, VoiceResult(i, "cancelled", reason="client"))
        self.pub_cancel.publish(String(data=rid or ""))

    def inject(self, text: str, source: str = "keyboard") -> Optional[str]:
        """Respuesta escrita (teclado/pantalla) para el pedido abierto: mismo camino que la voz."""
        with self._lock:
            if not self._pending:
                return None
            rid = list(self._pending)[-1]
            self._finish(rid, VoiceResult(rid, "ok", text=text.strip(), source=source))
        self.pub_cancel.publish(String(data=rid))
        return rid

    def is_pending(self, rid: Optional[str]) -> bool:
        with self._lock:
            return bool(rid) and rid in self._pending

    def speech_started(self, rid: Optional[str]) -> bool:
        with self._lock:
            return bool(rid) and self._pending.get(rid, {}).get("speech", False)

    def poll(self, rid: Optional[str]) -> Optional[VoiceResult]:
        if not rid:
            return None
        now = time.time()
        with self._lock:
            if rid in self._results:
                return self._results.pop(rid)
            pend = self._pending.get(rid)
            if pend is None:
                return None
            age = now - pend["t0"]
            # Sin heartbeat del ASR: no tiene sentido esperar el deadline completo
            if age > 1.5 and not self.online():
                self._finish(rid, VoiceResult(rid, "offline", reason=f"status={self.status}"))
            elif pend["legacy"] and not pend["speech"] and age > self.defaults["start_timeout"] + 7.0:
                self._finish(rid, VoiceResult(rid, "no_speech", reason="legacy_timeout"))
            elif now > pend["deadline"]:
                self._finish(rid, VoiceResult(rid, "timeout"))
            return self._results.pop(rid, None)

    def listen(self, prompt: str = "", timeout: Optional[float] = None, **kw) -> VoiceResult:
        """Bloqueante (no usar desde un callback del executor)."""
        rid = self.request(prompt, **kw)
        t_end = time.time() + (timeout or 1e9)
        while time.time() < t_end:
            res = self.poll(rid)
            if res is not None:
                return res
            time.sleep(0.05)
        self.cancel(rid)
        return VoiceResult(rid, "timeout")

    # ------------------------------------------------------------------ callbacks
    def _finish(self, rid: str, res: VoiceResult):
        """Con self._lock tomado."""
        pend = self._pending.pop(rid, None)
        if pend is not None:
            res.info.setdefault("client_sec", round(time.time() - pend["t0"], 2))
            self._results[rid] = res
            if len(self._results) > 50:
                self._results.pop(next(iter(self._results)))
            if res.status != "cancelled" and not (pend.get("quiet") and res.status != "ok"):
                self.log.info(f"🎤 [{rid}] {res.status}{': ' + repr(res.text) if res.text else ''}"
                              f"{' (' + res.reason + ')' if res.reason else ''}")

    def _on_result(self, msg: String):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self.new_protocol_seen = True
        rid = str(d.get("id", ""))
        if self.on_event:
            try:
                self.on_event(d)
            except Exception:
                pass
        with self._lock:
            if rid not in self._pending:
                return
            ev = d.get("event")
            if ev == "speech_start":
                self._pending[rid]["speech"] = True
                # la persona habla: dar tiempo a que termine la frase y se transcriba
                self._pending[rid]["deadline"] = max(self._pending[rid]["deadline"],
                                                     time.time() + self.defaults["max_speech"] + 8.0)
            elif ev == "result":
                info = {k: d[k] for k in ("speech_sec", "wait_sec", "asr_sec", "latency_sec", "avg_logprob", "raw")
                        if k in d}
                self._finish(rid, VoiceResult(rid, str(d.get("status", "error")), text=str(d.get("text", "")),
                                              reason=str(d.get("reason", "")), info=info))

    def _on_legacy_text(self, msg: String):
        text = msg.data.strip()
        if not text:
            return
        with self._lock:
            legacy = [r for r, p in self._pending.items() if p["legacy"]]
            if legacy:
                self._finish(legacy[-1], VoiceResult(legacy[-1], "ok", text=text, reason="legacy"))
