#!/usr/bin/env python3
"""Nodo `commander`: pedidos en lenguaje natural -> plan validado -> mission_executor. Con pagina web local.

  sub  /commander/command  std_msgs/String  pedido en texto ("anda a la cocina y busca una botella")
  pub  /commander/result   std_msgs/String  JSON {text, ok, steps[], errors[], cancel, executed}
  pub  /mission/run        std_msgs/String  plan (lo ejecuta mission_executor; /mission/status informa el avance)
  cli  /mission/cancel     std_srvs/Trigger
  web  http://127.0.0.1:8770                escribir el pedido, ver el plan, ejecutar, cancelar, ver el avance
  voz  voice:=true                          pedido por microfono -> dice el plan -> "yes" lo ejecuta, "stop" cancela

No mueve nada por si mismo y no reemplaza las guardas del ejecutor (mode:=real exige supervised:=true alli).
auto_execute:=false (por defecto en la web) muestra el plan y espera confirmacion; por topico ejecuta si auto_execute:=true.
Se niega a correr en el dominio 10 salvo real:=true.
"""
import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .command_parser import compile_command
from .node import _share_config
from .places import load_objects, load_places, load_tables
from .plan import parse_plan, validate_plan
from .skills import default_registry
from .voice_dialog import asr_prompt, error_to_speech, plan_to_speech

PORT = 8770
PAGE = r"""<!doctype html><html lang="es"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Commander</title><style>
:root{--bg:#14181d;--fg:#e8ecf1;--mut:#93a1b1;--card:#1e242c;--line:#323b46;--ok:#3fb27f;--err:#e5606a;--run:#e0a93b;--acc:#4a9eff}
*{box-sizing:border-box}body{font:16px system-ui,sans-serif;background:var(--bg);color:var(--fg);margin:0 auto;padding:16px;max-width:860px}
h1{font-size:22px;margin:4px 0 12px}textarea{width:100%;font:17px system-ui;padding:12px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--fg);min-height:76px}
button{font:600 15px system-ui;padding:10px 16px;border:0;border-radius:8px;margin:8px 8px 0 0;cursor:pointer;background:var(--line);color:var(--fg)}
button.go{background:var(--acc);color:#07111f}button.stop{background:var(--err);color:#fff}button:disabled{opacity:.4;cursor:default}
button.it{font-size:14px;padding:5px 9px;margin:4px 4px 0 0}button.bad{outline:1px solid var(--err)}.card{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px;margin-top:14px}.mut{color:var(--mut);font-size:14px}.err{color:var(--err)}
ol{margin:6px 0 0;padding-left:24px}li{padding:3px 0}.st{font-size:12px;font-weight:700;padding:2px 7px;border-radius:5px;margin-left:6px;background:var(--line)}
.SUCCEEDED{background:var(--ok);color:#06140d}.FAILED,.CANCELED{background:var(--err);color:#fff}.RUNNING{background:var(--run);color:#1b1300}
</style></head><body><h1>Commander del robot</h1><div class="mut" id="mode"></div>
<textarea id="t" placeholder="Agarra el objeto 1 y llevalo a la cocina"></textarea>
<div><button id="p">Ver plan</button><button id="x" class="go" disabled>Ejecutar</button><button id="c" class="stop">Cancelar mision</button></div>
<div class="card"><b>En el mapa</b> <span class="mut">(toca para agregar al pedido)</span><div id="items" class="mut">-</div></div>
<div class="card" id="plan"><span class="mut">Escribi un pedido y toca "Ver plan".</span></div>
<div class="card"><b>Mision</b> <span id="ms" class="st">-</span><div class="mut" id="mm"></div><ol id="steps"></ol></div>
<script>
const $=i=>document.getElementById(i);let ready=false;
const esc=s=>String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
async function items(){const j=await(await fetch('/api/items')).json();
 const o=j.objects.map(x=>'<button class="it'+(x.problem?' bad':'')+'" data-t="objeto '+x.number+'" title="'+esc(x.problem)+'">#'+x.number+' '+esc(x.name)+' <small>'+esc(x.cls)+(x.on?' en '+esc(x.on):'')+'</small></button>').join('');
 const p=j.places.map(x=>'<button class="it" data-t="'+esc(x)+'">'+esc(x)+'</button>').join('');
 $('items').innerHTML='<div>Objetos: '+(o||'ninguno')+'</div><div>Lugares: '+(p||'ninguno')+'</div>';}
$('items').onclick=e=>{const b=e.target.closest('button');if(!b)return;const t=$('t');t.value=(t.value?t.value.trimEnd()+' ':'')+b.dataset.t;t.focus();};
items();setInterval(items,5000);
async function api(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});return r.json();}
$('p').onclick=async()=>{const j=await api('/api/plan',{text:$('t').value});ready=j.ok;$('x').disabled=!ready;
 $('plan').innerHTML=j.cancel?'<span class="mut">Eso es un pedido de cancelar: usa el boton rojo.</span>':j.ok?'<b>Plan</b><ol>'+j.steps.map(s=>'<li>'+esc(s)+'</li>').join('')+'</ol>':'<b class="err">No lo puedo hacer</b><ul>'+j.errors.map(e=>'<li class="err">'+esc(e)+'</li>').join('')+'</ul>';};
$('x').onclick=async()=>{const j=await api('/api/run',{text:$('t').value});if(!j.ok)alert((j.errors||[]).join('\n'));};
$('c').onclick=()=>api('/api/cancel');
async function poll(){try{const j=await(await fetch('/api/status')).json();$('mode').textContent=j.info;const s=j.mission||{};
 $('ms').textContent=s.state||'-';$('ms').className='st '+(s.state||'');$('mm').textContent=(s.name||'')+' '+(s.message||'');
 $('steps').innerHTML=(s.steps||[]).map(x=>'<li>'+esc(x.id+' '+JSON.stringify(x.args))+'<span class="st '+x.state+'">'+x.state+'</span> <span class="mut">'+esc(x.message||'')+'</span></li>').join('');}catch(e){}
 setTimeout(poll,500);}poll();
</script></body></html>"""


class Commander(Node):
    def __init__(self):
        super().__init__("commander")
        self.declare_parameter("semantic_map", "")
        self.declare_parameter("waypoints", "")
        self.declare_parameter("auto_execute", False)
        self.declare_parameter("real", False)
        self.declare_parameter("voice", False)
        self.declare_parameter("port", PORT)
        self.declare_parameter("tts", True)          # false: no usa el parlante (pruebas)
        self.voice_state = "apagada"
        self.semantic_map = self.get_parameter("semantic_map").value or _share_config("semantic_map.yaml")
        self.waypoints = self.get_parameter("waypoints").value or _share_config("waypoints.yaml")
        self.registry = default_registry()
        self.mission = {}
        self.pub_run = self.create_publisher(String, "/mission/run", 5)
        self.pub_result = self.create_publisher(String, "/commander/result", 5)
        self.pub_say = self.create_publisher(String, "/hri/say_something", 5)
        self.pub_mute = self.create_publisher(Bool, "/voice/mute", 5)
        self._speaker = None
        self.cli_cancel = self.create_client(Trigger, "/mission/cancel")
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/mission/status", self._status, latched)
        self.create_subscription(String, "/commander/command", self._command, 5)
        self._start_web()
        if bool(self.get_parameter("voice").value):
            threading.Thread(target=self._voice_loop, daemon=True).start()
        self.get_logger().info(f"commander listo: http://127.0.0.1:{self.httpd.server_port} | lugares: {sorted(self.places())}")

    def places(self):
        return load_places(self.semantic_map, self.waypoints)      # se relee: el mapa se edita con el editor

    def _status(self, msg):
        try:
            self.mission = json.loads(msg.data)
        except ValueError:
            pass

    def compile(self, text):
        """Pedido -> resultado (dict). Valida contra el registro y el mapa con las mismas reglas del ejecutor."""
        places = self.places()
        c = compile_command(text, places, objects=load_objects(self.semantic_map), tables=load_tables(self.semantic_map))
        errors = list(c.errors)
        plan = None
        if c.ok:
            plan = c.plan
            errors = validate_plan(parse_plan(plan), self.registry, places)
        return dict(text=text, ok=c.ok and not errors, steps=c.steps_text, errors=errors, cancel=c.cancel, plan=plan,
                    executed=False)

    def items(self):
        """Lo que se puede pedir: objetos en orden numerico y lugares."""
        places = sorted(k for k in self.places() if not k.startswith("object:"))
        return dict(objects=load_objects(self.semantic_map), places=places)

    def run(self, res):
        if self.pub_run.get_subscription_count() == 0:
            res["ok"] = False
            res["errors"] = ["mission_executor no esta corriendo: no se envio nada"]
            return res
        if self.mission.get("state") == "RUNNING":
            res["ok"] = False
            res["errors"] = ["ya hay una mision en curso: cancelala primero"]
            return res
        self.pub_run.publish(String(data=json.dumps(res["plan"])))
        res["executed"] = True
        self.get_logger().info(f"Plan enviado: {res['steps']}")
        return res

    def cancel(self):
        if not self.cli_cancel.service_is_ready():
            return False
        self.cli_cancel.call_async(Trigger.Request())
        return True

    def _command(self, msg):
        res = self.compile(msg.data)
        if res["cancel"]:
            res["executed"] = self.cancel()
        elif res["ok"] and bool(self.get_parameter("auto_execute").value):
            res = self.run(res)
        self.pub_result.publish(String(data=json.dumps({k: v for k, v in res.items() if k != "plan"})))

    # ------------------------------------------------------------------ voz
    def _say(self, text):
        self.get_logger().info(f"Digo: {text}")
        self.pub_say.publish(String(data=text))
        if self._speaker is None:
            return
        self.pub_mute.publish(Bool(data=True))      # que el ASR no se escuche a si mismo
        try:
            self._speaker.speak(text, blocking=True)
        finally:
            self.pub_mute.publish(Bool(data=False))

    def _hear(self, voice, state, **kw):
        """Abre una escucha y devuelve el texto, o "" si no hubo voz."""
        self.voice_state = state
        places = self.places()
        res = voice.listen(prompt=asr_prompt(sorted(places), load_objects(self.semantic_map)), **kw)
        if res.ok:
            self.voice_state = f"oi: {res.text}"
            self.get_logger().info(f"Oi: {res.text}")
            return res.text
        if res.status in ("offline", "error"):
            self.voice_state = f"microfono no disponible ({res.status})"
            time.sleep(2.0)
        return ""

    def _voice_loop(self):
        from base_athome_perception.guest_dialog import parse_yes_no
        from base_athome_perception.voice_client import VoiceClient
        if bool(self.get_parameter("tts").value):
            from base_athome_perception.speaker import RoboCupSpeaker
            self._speaker = RoboCupSpeaker()
        voice = VoiceClient(self, start_timeout=30.0)
        self.voice_state = "esperando el microfono"
        while rclpy.ok() and not voice.online():
            time.sleep(0.5)
        self._say("Commander ready. Tell me what to do.")
        while rclpy.ok():
            running = self.mission.get("state") == "RUNNING"
            text = self._hear(voice, "mision en curso: escucho 'stop'" if running else "escuchando un pedido")
            if not text:
                continue
            res = self.compile(text)
            self.pub_result.publish(String(data=json.dumps({k: v for k, v in res.items() if k != "plan"})))
            if res["cancel"]:
                self.cancel()
                self._say("Stopping." if running else "There is nothing to stop.")
            elif self.mission.get("state") == "RUNNING":
                self._say("I am busy. Say stop to cancel.")
            elif not res["ok"]:
                self._say(error_to_speech(res["errors"]))
            else:
                self._say(plan_to_speech(res["plan"], load_objects(self.semantic_map)) + " Should I do it?")
                answer = self._hear(voice, "esperando yes o no", start_timeout=10.0)
                if parse_yes_no(answer) is True:
                    res = self.run(res)
                    self._say("Okay." if res["executed"] else "I cannot start the mission right now.")
                else:
                    self._say("Okay, I will not do it.")

    def _start_web(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, body, ctype="application/json"):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.startswith("/api/status"):
                    info = f"dominio {os.environ.get('ROS_DOMAIN_ID', '0')} | ejecutor {'conectado' if node.pub_run.get_subscription_count() else 'NO conectado'} | modo {node.mission.get('mode', '?')} | voz: {node.voice_state}"
                    return self._send(json.dumps(dict(info=info, mission=node.mission)).encode())
                if self.path.startswith("/api/items"):
                    return self._send(json.dumps(node.items()).encode())
                return self._send(PAGE.encode(), "text/html; charset=utf-8")

            def do_POST(self):
                try:
                    d = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                except ValueError:
                    d = {}
                if self.path == "/api/cancel":
                    return self._send(json.dumps(dict(ok=node.cancel())).encode())
                res = node.compile(str(d.get("text", "")))
                if self.path == "/api/run" and res["ok"]:
                    res = node.run(res)
                return self._send(json.dumps({k: v for k, v in res.items() if k != "plan"}).encode())

        self.httpd = ThreadingHTTPServer(("127.0.0.1", int(self.get_parameter("port").value)), H)   # solo local
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()


def main(args=None):
    argv = sys.argv if args is None else args
    if os.environ.get("ROS_DOMAIN_ID", "0") == "10" and "real:=true" not in " ".join(argv):
        print("[commander] dominio 10 (robot real) sin real:=true: rechazado. Usar ROS_DOMAIN_ID=42")
        raise SystemExit(2)
    rclpy.init(args=args)
    node = Commander()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
