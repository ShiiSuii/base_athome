#!/usr/bin/env python3
"""Servidor local del VLM, en el entorno de la LLM (~/.local/share/llm-venv), solo en 127.0.0.1. No conoce ROS ni el robot.
Recibe una imagen JPEG y una pregunta, y responde texto: ver mission/vlm.py."""
import base64
import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Lock

from llama_cpp import Llama
from llama_cpp.llama_chat_format import MTMDChatHandler

MODELS = os.path.expanduser("~/.local/share/base_athome/models")
MODEL = os.environ.get("ATHOME_VLM_MODEL", os.path.join(MODELS, "SmolVLM-500M-Instruct-Q8_0.gguf"))
MMPROJ = os.environ.get("ATHOME_VLM_MMPROJ", os.path.join(MODELS, "mmproj-SmolVLM-500M-Instruct-Q8_0.gguf"))
THREADS = int(os.environ.get("ATHOME_VLM_THREADS", "4"))


class SmolVLMHandler(MTMDChatHandler):
    CHAT_FORMAT = ("<|im_start|>{% for message in messages %}{{ message['role'] | capitalize }}:"
                   "{% if message['content'] is string %}{{ message['content'] }}{% else %}"
                   "{% for part in message['content'] %}{% if part['type'] == 'text' %}{{ part['text'] }}"
                   "{% else %}{{ part['image_url'] if part['image_url'] is string else part['image_url']['url'] }}{% endif %}"
                   "{% endfor %}{% endif %}<end_of_utterance>\n{% endfor %}{% if add_generation_prompt %}Assistant:{% endif %}")


llm = Llama(model_path=MODEL, chat_handler=SmolVLMHandler(clip_model_path=MMPROJ, verbose=False, use_gpu=False),
            n_ctx=4096, n_threads=THREADS, verbose=False)
lock = Lock()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def do_POST(self):
        req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
        url = "data:image/jpeg;base64," + req["image"]
        content = [dict(type="image_url", image_url=dict(url=url)), dict(type="text", text=req["question"])]
        with lock:
            res = llm.create_chat_completion(messages=[dict(role="user", content=content)], max_tokens=60, temperature=0.0,
                                             stop=["<end_of_utterance>"])
        body = json.dumps(dict(answer=res["choices"][0]["message"]["content"])).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(body)


if __name__ == "__main__":
    print("vlm_server listo", flush=True)
    ThreadingHTTPServer(("127.0.0.1", 8772), Handler).serve_forever()
