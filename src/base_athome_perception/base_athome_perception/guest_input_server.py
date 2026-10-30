#!/usr/bin/env python3
"""Pantalla tactil / teclado para el invitado: alternativa al microfono.

Sirve una pagina web (http://127.0.0.1:8090 en la pantalla del robot, o desde un celular en la
misma red) con:
  - la pregunta actual del robot (topico /hri/question),
  - botones grandes con los NOMBRES y BEBIDAS oficiales de la competencia,
  - un campo para escribir si no esta en la lista, y botones SI / NO para confirmar.
Cada toque se publica en /hri/guest_input (std_msgs/String, JSON):
  {"type": "name", "value": "Robin"} | {"type": "drink", "value": "Iced Tea"} | {"type": "answer", "value": "yes"}
El receptionist y el secuenciador lo aceptan igual que la voz.
"""
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable, List

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Robot - Bienvenido</title>
<style>
body{font-family:sans-serif;background:#10141c;color:#eee;margin:0;padding:18px}
h1{font-size:30px;margin:6px 0 14px;color:#4fd1ff} h2{font-size:22px;margin:18px 0 8px}
.q{font-size:26px;background:#1d2533;padding:14px;border-radius:12px;min-height:40px}
.grid{display:flex;flex-wrap:wrap;gap:10px}
button{font-size:24px;padding:16px 22px;border:0;border-radius:12px;background:#2b6cb0;color:#fff;cursor:pointer}
button.d{background:#2f855a} button.y{background:#38a169} button.n{background:#c53030}
input{font-size:24px;padding:12px;border-radius:10px;border:0;width:60%}
.ok{color:#9ae6b4;font-size:22px;margin-top:12px;min-height:28px}
</style></head><body>
<h1>Hello! Tap your answer</h1>
<div class="q" id="q">...</div>
<h2>Your name</h2><div class="grid" id="names"></div>
<p><input id="nm" placeholder="Other name"><button onclick="send('name',nm.value)">OK</button></p>
<h2>Your favorite drink</h2><div class="grid" id="drinks"></div>
<p><input id="dr" placeholder="Other drink"><button class="d" onclick="send('drink',dr.value)">OK</button></p>
<h2>Confirm</h2><div class="grid"><button class="y" onclick="send('answer','yes')">YES</button>
<button class="n" onclick="send('answer','no')">NO</button></div>
<div class="ok" id="ok"></div>
<script>
const NAMES=%NAMES%, DRINKS=%DRINKS%;
function mk(id,list,kind,cls){const g=document.getElementById(id);list.forEach(v=>{const b=document.createElement('button');
 b.textContent=v;if(cls)b.className=cls;b.onclick=()=>send(kind,v);g.appendChild(b);});}
mk('names',NAMES,'name');mk('drinks',DRINKS,'drink','d');
function send(type,value){if(!value)return;fetch('/input',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify({type:type,value:value})}).then(()=>{document.getElementById('ok').textContent='Sent: '+value;});}
setInterval(()=>fetch('/question').then(r=>r.text()).then(t=>{document.getElementById('q').textContent=t||'...';}),700);
</script></body></html>"""


class GuestInputServer:
    def __init__(self, names: List[str], drinks: List[str], on_input: Callable[[dict], None],
                 host: str = "0.0.0.0", port: int = 8090):
        self.question = ""
        self.on_input = on_input
        page = PAGE.replace("%NAMES%", json.dumps(names)).replace("%DRINKS%", json.dumps(drinks)).encode("utf-8")
        server_ref = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body: bytes, ctype="text/html; charset=utf-8"):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.startswith("/question"):
                    self._send(200, server_ref.question.encode("utf-8"), "text/plain; charset=utf-8")
                else:
                    self._send(200, page)

            def do_POST(self):
                if not self.path.startswith("/input"):
                    self._send(404, b"")
                    return
                try:
                    n = int(self.headers.get("Content-Length", "0"))
                    data = json.loads(self.rfile.read(n) or b"{}")
                    if data.get("type") in ("name", "drink", "answer") and str(data.get("value", "")).strip():
                        server_ref.on_input({"type": data["type"], "value": str(data["value"]).strip()[:40]})
                        self._send(200, b"ok", "text/plain")
                        return
                except (ValueError, json.JSONDecodeError):
                    pass
                self._send(400, b"bad request", "text/plain")

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def stop(self):
        self.httpd.shutdown()
