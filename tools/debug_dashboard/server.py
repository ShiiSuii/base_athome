#!/usr/bin/env python3
"""Tablero de debug del robot en 127.0.0.1:8780 (alias `debug`). Solo muestra: no da ordenes al robot."""
import json
import hashlib
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import estado

HERE = Path(__file__).resolve().parent
PORT = int(os.environ.get("DEBUG_PORT", "8780"))
lock = threading.Lock()
cache = dict(fp=None, data=None)


def state():
    with lock:
        fp = estado.fingerprint()
        if fp != cache["fp"]:
            cache.update(fp=fp, data=estado.compute())
    page = hashlib.sha1((HERE / "index.html").read_bytes()).hexdigest()
    return dict(cache["data"], vivo=estado.robot_mode(), hora=time.strftime("%H:%M:%S"), pagina=page)


def restart_on_change():
    """Relanza el tablero si cambia su propio codigo, para no quedar mostrando logica vieja."""
    files = [HERE / "server.py", HERE / "estado.py"]
    before = [f.stat().st_mtime for f in files]
    while True:
        time.sleep(2)
        if [f.stat().st_mtime for f in files] != before:
            os.execv(sys.executable, [sys.executable, str(HERE / "server.py")])


class H(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def _send(self, body, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/grafo"):
            # El grafo de graphify: se regenera con `graphify update .` despues de cada cambio de codigo.
            graph = estado.WS / "graphify-out" / "graph.html"
            body = graph.read_bytes() if graph.exists() else "Falta el grafo: correr graphify update .".encode()
            self._send(body, "text/html; charset=utf-8")
        elif self.path.startswith("/api/state"):
            self._send(json.dumps(state()).encode(), "application/json")
        else:
            # El estado va en la pagina para que se vea completa desde el primer dibujo.
            first = json.dumps(state()).replace("<", "\\u003c")
            page = (HERE / "index.html").read_text(encoding="utf-8").replace("/*ESTADO*/null", first)
            self._send(page.encode(), "text/html; charset=utf-8")


if __name__ == "__main__":
    os.nice(10)  # Nunca le quita CPU al robot.
    threading.Thread(target=restart_on_change, daemon=True).start()
    print(f"Tablero: http://127.0.0.1:{PORT}", flush=True)
    ThreadingHTTPServer(("127.0.0.1", PORT), H).serve_forever()
