#!/usr/bin/env python3
"""Servidor local de la LLM, en su propio entorno (~/.local/share/llm-venv), solo en 127.0.0.1. Carga el modelo una vez.
La gramatica y el mensaje llegan en cada pedido: ver mission/llm_intent.py. No conoce ROS ni el robot."""
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock

from llama_cpp import Llama, LlamaGrammar

MODEL = os.environ.get("ATHOME_LLM_MODEL", os.path.expanduser("~/.local/share/base_athome/models/qwen2.5-1.5b-q4.gguf"))
llm = Llama(model_path=MODEL, n_ctx=2048, n_threads=int(os.environ.get("ATHOME_LLM_THREADS", "4")), verbose=False)
lock = Lock()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_POST(self):
        req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        with lock:
            res = llm.create_chat_completion(messages=req["messages"], grammar=LlamaGrammar.from_string(req["grammar"], verbose=False),
                                             max_tokens=80, temperature=0.0)
        body = json.dumps(dict(reply=res["choices"][0]["message"]["content"])).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(body)


print("llm_server listo", flush=True)
ThreadingHTTPServer(("127.0.0.1", 8771), Handler).serve_forever()
