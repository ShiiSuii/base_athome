#!/usr/bin/env python3
"""Teleoperacion del Piper con UN control del Meta Quest 2 conectado por USB a la Jetson.

Flujo: navegador del Quest (WebXR) -> http://localhost:8765 (adb reverse por el cable, sin WiFi)
-> este nodo -> /arm/joint_trajectory y /arm/gripper_cmd -> piper_arm_controller (limites, feedback).
Mapeo y seguridad: quest_teleop_core.py (grip = hombre muerto, joystick = par de articulaciones,
A/X = cambia par, gatillo = pinza; 20 grados/s, timeout 0,3 s). El nodo NUNCA habilita el brazo.

  python3 quest_teleop_node.py [--ros-args -p hand:=right -p max_speed_dps:=20.0 -p open_browser:=true]
"""
import json
import socket
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import Float32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import quest_cartesian_core as qc
from quest_teleop_core import MODE_NAMES, ControllerSample, TeleopState, step

PORT = 8765
PAGE_VERSION = 5   # subir al cambiar la pagina: el nodo avisa si el visor tiene una version vieja abierta
DEG = 3.141592653589793 / 180.0

PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Piper Quest</title><style>body{font-family:sans-serif;background:#111;color:#eee;margin:16px}
button{font-size:28px;padding:18px 28px;border-radius:12px;border:0;background:#2a7;color:#fff}
#s{margin-top:16px;font-size:20px;white-space:pre}</style></head><body>
<h2>Teleoperacion Piper <small>(v PAGE_VERSION)</small></h2><p>Control HAND_LABEL. MODO_TXT</p><button id="b">Empezar</button><div id="s">listo</div>
<script>
const st=document.getElementById('s');let busy=false,last=0,xr=false;
function post(d){return fetch('/state',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)})
 .then(r=>r.json()).then(j=>{st.textContent=j.reason+'\nmodo: '+j.mode;}).catch(e=>{st.textContent='sin conexion '+e});}
function send(d){if(busy)return;busy=true;d.v=PAGE_VERSION;d.xr=true;post(d).finally(()=>{busy=false});}
setInterval(()=>{if(!xr)post({v:PAGE_VERSION,hello:true,xr:false,webxr:!!navigator.xr});},1000);   // latido: pagina abierta
document.getElementById('b').onclick=async()=>{
 try{
 if(!navigator.xr){post({v:PAGE_VERSION,error:'WebXR no disponible'});st.textContent='WebXR no disponible';return;}
 // passthrough (ver la habitacion y el robot) si el visor lo permite; si no, VR
 let mode='immersive-ar';
 if(!(await navigator.xr.isSessionSupported('immersive-ar').catch(()=>false)))mode='immersive-vr';
 const session=await navigator.xr.requestSession(mode,{optionalFeatures:['local-floor']});
 post({v:PAGE_VERSION,hello:true,xr:false,info:'sesion '+mode});
 const canvas=document.createElement('canvas');const gl=canvas.getContext('webgl',{xrCompatible:true});
 await gl.makeXRCompatible();session.updateRenderState({baseLayer:new XRWebGLLayer(session,gl)});
 const ref=await session.requestReferenceSpace('local');xr=true;
 const loop=(t,frame)=>{session.requestAnimationFrame(loop);
  const layer=session.renderState.baseLayer;gl.bindFramebuffer(gl.FRAMEBUFFER,layer.framebuffer);
  let src=null;const all=[...session.inputSources].filter(s=>s.gamepad);
  src=all.find(s=>s.handedness===HAND)||all[0]||null;          // el configurado o cualquiera
  let grip=false;
  if(src){const g=src.gamepad;grip=g.buttons.length>1&&g.buttons[1].value>0.5;
   if(t-last>33){last=t;const ps=frame.getPose(src.gripSpace||src.targetRaySpace,ref);
    const pose=ps?{p:[ps.transform.position.x,ps.transform.position.y,ps.transform.position.z],
     q:[ps.transform.orientation.x,ps.transform.orientation.y,ps.transform.orientation.z,ps.transform.orientation.w]}:null;
    send({hand:src.handedness,axes:Array.from(g.axes),buttons:g.buttons.map(b=>b.value),pose:pose});}}
  else if(t-last>500){last=t;send({hand:'ninguno',axes:[],buttons:[],pose:null});}
  // en AR se ve la habitacion: solo un tinte verde con el grip; en VR gris/verde
  if(mode==='immersive-ar'){if(grip)gl.clearColor(0.0,0.6,0.1,0.25);else gl.clearColor(0,0,0,0);}
  else{if(grip)gl.clearColor(0.05,0.35,0.1,1);else gl.clearColor(0.12,0.12,0.12,1);}
  gl.clear(gl.COLOR_BUFFER_BIT);};
 session.requestAnimationFrame(loop);session.addEventListener('end',()=>{xr=false;st.textContent='sesion VR terminada'});
 }catch(e){post({v:PAGE_VERSION,error:String(e)});st.textContent='error: '+e;}};
</script></body></html>"""


class QuestTeleop(Node):
    def __init__(self):
        super().__init__("quest_teleop")
        self.declare_parameter("hand", "right")
        self.declare_parameter("max_speed_dps", 20.0)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("open_browser", True)
        self.declare_parameter("arm_status_timeout_s", 0.5)
        self.declare_parameter("mode", "cartesian")          # cartesian (pose 6D del control) | joint
        self.declare_parameter("scale", 1.0)                 # escala de la traslacion mano -> pinza
        self.declare_parameter("max_joint_dps", 45.0)        # velocidad maxima por articulacion (modo cartesiano)
        self.hand = str(self.get_parameter("hand").value)
        self.max_speed = float(self.get_parameter("max_speed_dps").value)
        self.dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.lock = threading.Lock()
        self.sample = None
        self.arm = {}
        self.state = TeleopState()
        self.cstate = qc.CartesianState()
        self.pose = None
        self.pose_t = 0.0
        self.page_version = None
        self.page_seen_t = 0.0
        self.page_xr = False
        self.page_error = None
        self.post_count = 0
        self.page_hand = None
        self.mode = str(self.get_parameter("mode").value)
        self.pub_target = self.create_publisher(PoseStamped, "/quest_teleop/target_pose", 10)
        self.pub_cmd = self.create_publisher(JointState, "/quest_teleop/command", 10)
        self.last_grip_sent = None
        self.pub_traj = self.create_publisher(JointTrajectory, "/arm/joint_trajectory", 2)
        self.pub_grip = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.pub_status = self.create_publisher(String, "/quest_teleop/status", 5)
        self.create_subscription(String, "/arm/status", self._arm_cb, 10)
        self.create_timer(self.dt, self._tick)
        self._start_http()
        self._setup_adb()

    # ------------------------------------------------------------------ entrada
    def _arm_cb(self, msg):
        try:
            self.arm = json.loads(msg.data)
            self.arm["_t"] = time.monotonic()
        except ValueError:
            pass

    def _start_http(self):
        node = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, fmt, *a):
                if self.command == "GET":   # cada carga de la pagina queda en el log (diagnostico)
                    node.get_logger().info(f"HTTP {self.client_address[0]} {fmt % a}")

            def do_GET(self):
                page = PAGE.replace("PAGE_VERSION", str(PAGE_VERSION))
                txt = ("Mantener GRIP: la pinza copia el movimiento de tu mano (posicion y giro). Soltar = frena; volver a "
                       "agarrar para seguir desde otra posicion. X = activar/desactivar el giro. Gatillo = pinza."
                       if node.mode == "cartesian" else
                       "Mantener GRIP para mover. Joystick = articulaciones. X/A = cambiar par. Gatillo = pinza.")
                body = page.replace("MODO_TXT", txt).replace("HAND_LABEL", "izquierdo" if node.hand == "left" else "derecho") \
                    .replace("HAND", json.dumps(node.hand)).encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                node.post_count += 1
                raw = b""
                try:
                    raw = self.rfile.read(int(self.headers.get("Content-Length", 0)))
                    d = json.loads(raw)
                    with node.lock:
                        node.page_version = d.get("v")
                        node.page_seen_t = time.monotonic()
                        # Cualquier version de la pagina sirve si manda botones (y la pose para el modo cartesiano):
                        # las pestanas viejas quedan abiertas en el Quest y no conviene obligar a recargar.
                        node.page_xr = bool(d.get("xr")) or "buttons" in d
                        if d.get("error"):
                            node.page_error = str(d["error"])[:200]
                            node.get_logger().error(f"Pagina del visor: {node.page_error}")
                        if not node.page_xr:
                            raise ValueError("latido")           # pagina abierta, todavia sin sesion VR
                        node.page_hand = d.get("hand")
                        node.sample = ControllerSample(t=time.monotonic(), axes=d.get("axes", []),
                                                       buttons=d.get("buttons", []))
                        pz = d.get("pose")
                        if isinstance(pz, dict) and "p" in pz and "q" in pz:
                            node.pose, node.pose_t = (pz["p"], pz["q"]), time.monotonic()
                        # sin pose (pagina vieja o control sin seguimiento): NO se borra la ultima pose valida
                except (ValueError, TypeError) as e:
                    if str(e) not in ("latido", "pagina vieja"):
                        node.get_logger().warn(f"POST invalido ({e}): {raw[:120]!r}", throttle_duration_sec=2.0)
                except Exception as e:  # noqa: BLE001
                    node.get_logger().error(f"POST error {type(e).__name__}: {e}")
                with node.lock:
                    out = {"reason": (node.cstate.reason if node.mode == "cartesian" else node.state.reason),
                           "mode": ("cartesiano" + ("" if node.cstate.use_rot else " (solo posicion)")
                                    if node.mode == "cartesian" else MODE_NAMES[node.state.mode]),
                           "arm_enabled": bool(node.arm.get("enabled"))}
                body = json.dumps(out).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        # Solo localhost, en IPv4 Y en IPv6: el adb del host reenvia "localhost" y puede resolver a ::1
        # (con solo 127.0.0.1 el navegador del Quest conectaba y la conexion se cortaba: pagina en blanco).
        class H6(ThreadingHTTPServer):
            address_family = socket.AF_INET6
        self.httpd = ThreadingHTTPServer(("127.0.0.1", PORT), H)
        self.servers = [self.httpd]
        try:
            self.servers.append(H6(("::1", PORT), H))
        except OSError as e:
            self.get_logger().warn(f"sin servidor IPv6 ::1 ({e})")
        for srv in self.servers:
            threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.get_logger().info(f"Pagina de teleoperacion en http://127.0.0.1:{PORT} (solo localhost)")

    def _setup_adb(self):
        try:
            r = subprocess.run(["adb", "reverse", f"tcp:{PORT}", f"tcp:{PORT}"], capture_output=True, text=True, timeout=8)
            if r.returncode != 0:
                self.get_logger().warn(f"adb reverse fallo ({r.stderr.strip()}): ¿Quest conectado y autorizado?")
                return
            self.get_logger().info("adb reverse OK: el Quest ve http://localhost:8765")
            if bool(self.get_parameter("open_browser").value):
                subprocess.run(["adb", "shell", "am", "start", "-a", "android.intent.action.VIEW",
                                "-d", f"http://localhost:{PORT}", "com.oculus.browser"],
                               capture_output=True, text=True, timeout=8)
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().warn(f"adb no disponible: {e}")

    # ------------------------------------------------------------------ ciclo
    def _tick(self):
        now = time.monotonic()
        fresh_arm = self.arm.get("feedback_ok") and \
            now - self.arm.get("_t", 0) < float(self.get_parameter("arm_status_timeout_s").value)
        measured = [float(v) for v in self.arm.get("joints_deg", [])][:6] if fresh_arm else None
        with self.lock:
            sample = self.sample
            pose = self.pose
        if self.mode == "cartesian":
            self._tick_cartesian(now, sample, pose, measured, fresh_arm)
            return
        with self.lock:
            st = step(self.state, sample, measured, now, self.dt, max_speed_dps=self.max_speed)
        if not self.arm.get("enabled") and st.active:
            st.reason = "brazo DESHABILITADO: habilitarlo con /arm/set_enable (no se envian comandos)"
            st.active = False
        for line in st.log:
            self.get_logger().info(line)
        st.log.clear()
        deadman = sample is not None and now - sample.t < 0.3 and len(sample.buttons) > 1 and sample.buttons[1] > 0.5
        if st.active and st.target_deg is not None and self.arm.get("enabled"):
            msg = JointTrajectory()
            msg.joint_names = [f"joint{i}" for i in range(1, 7)]
            p = JointTrajectoryPoint()
            p.positions = [v * DEG for v in st.target_deg]
            p.time_from_start = Duration(sec=0, nanosec=int(self.dt * 3 * 1e9))   # horizonte corto: movimiento continuo
            msg.points = [p]
            self.pub_traj.publish(msg)
        if deadman and self.arm.get("enabled") and (self.last_grip_sent is None or abs(st.gripper_mm - self.last_grip_sent) > 3.0):
            self.pub_grip.publish(Float32(data=float(st.gripper_mm)))
            self.last_grip_sent = st.gripper_mm
        self.pub_status.publish(String(data=json.dumps(dict(
            reason=st.reason, mode=MODE_NAMES[st.mode], active=st.active, deadman=bool(deadman),
            arm_enabled=bool(self.arm.get("enabled")), feedback_ok=bool(fresh_arm),
            headset_age_s=None if sample is None else round(now - sample.t, 2)))))


def _publish_traj(node, q_deg):
    msg = JointTrajectory()
    msg.joint_names = [f"joint{i}" for i in range(1, 7)]
    p = JointTrajectoryPoint()
    p.positions = [v * DEG for v in q_deg]
    p.time_from_start = Duration(sec=0, nanosec=int(node.dt * 3 * 1e9))
    msg.points = [p]
    node.pub_traj.publish(msg)


def _tick_cartesian(self, now, sample, pose, measured, fresh_arm):
    age = 99.0 if sample is None else now - sample.t
    buttons = sample.buttons if sample is not None else []
    if pose is not None and now - self.pose_t > 0.3:
        pose = None                     # pose vieja: equivale a perder el seguimiento del control
    t_ik = time.monotonic()
    with self.lock:
        st = qc.cart_step(self.cstate, pose, buttons, measured, age, self.dt,
                          scale=float(self.get_parameter("scale").value),
                          max_joint_dps=float(self.get_parameter("max_joint_dps").value))
    for line in st.log:
        self.get_logger().info(line)
    st.log.clear()
    enabled = bool(self.arm.get("enabled"))
    page_alive = now - self.page_seen_t < 2.0
    if not page_alive:
        st.reason = (f"la pagina NO esta abierta en el visor: abrir http://localhost:{PORT} en el navegador del Quest")
    elif not self.page_xr:
        st.reason = "pagina abierta: tocar 'Entrar en VR'" + (f" (error: {self.page_error})" if self.page_error else "")
    elif self.page_hand == "ninguno":
        st.reason = "en VR pero no se ve ningun control: encender/mover el control"
    if sample is not None and age < 0.3 and pose is None:
        st.reason = "llegan botones pero NO la pose del control (¿control apagado o fuera de vista?)"
    if not enabled and st.engaged:
        st.reason = "brazo DESHABILITADO: habilitarlo con /arm/set_enable (no se envian comandos)"
    if st.engaged and st.q_cmd is not None and enabled:
        _publish_traj(self, st.q_cmd)
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [v * DEG for v in st.q_cmd]
        self.pub_cmd.publish(js)
        if st.target_p is not None:
            ps = PoseStamped()
            ps.header.stamp, ps.header.frame_id = js.header.stamp, "arm_base_link_robot_axes"
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) / 1000.0 for v in st.target_p)
            ps.pose.orientation.w = 1.0
            x, y, z, w = qc.R_to_quat(st.target_R)
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = x, y, z, w
            self.pub_target.publish(ps)
    deadman = st.engaged
    if deadman and enabled and (self.last_grip_sent is None or abs(st.gripper_mm - self.last_grip_sent) > 3.0):
        self.pub_grip.publish(Float32(data=float(st.gripper_mm)))
        self.last_grip_sent = st.gripper_mm
    self.pub_status.publish(String(data=json.dumps(dict(
        reason=st.reason, mode="cartesiano", rotation=st.use_rot, engaged=st.engaged, arm_enabled=enabled,
        feedback_ok=bool(fresh_arm), pose_received=pose is not None, page_version=self.page_version,
        page_alive=page_alive, xr=self.page_xr, posts=self.post_count, hand=self.page_hand, page_error=self.page_error, err_mm=round(st.err_mm, 1),
        err_deg=round(st.err_deg, 1), headset_age_s=None if sample is None else round(age, 2),
        ik_ms=round((time.monotonic() - t_ik) * 1000.0, 1)))))


QuestTeleop._tick_cartesian = _tick_cartesian


def main():
    rclpy.init()
    node = QuestTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for srv in node.servers:
            srv.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
