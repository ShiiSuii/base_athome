#!/usr/bin/env python3
"""Nodo ROS 2 `mission_executor`: recibe un plan estructurado, lo valida y lo ejecuta paso a paso.

Interfaces (contrato, ver README del paquete):
  sub  /mission/run     std_msgs/String  plan JSON/YAML {"name", "steps": [{id, skill, args, retries, timeout_s, on_failure}]}
  srv  /mission/cancel  std_srvs/Trigger cancela la mision en curso (cancela el goal de Nav2 si lo hay)
  pub  /mission/status  std_msgs/String  JSON de estado (transient_local, en cada cambio y a 1 Hz)
  pub  /mission/skills  std_msgs/String  JSON con el registro de habilidades (transient_local)
  pub  /hri/say_something                texto que se dice (igual que el secuenciador)

Seguridad: mode:=sim exige ROS_DOMAIN_ID != 10 (nunca manda goals al robot real). mode:=real exige
supervised:=true (decision humana, AGENTS.md §3) porque navigate_* mueve la base.
Uso:  ROS_DOMAIN_ID=42 ros2 run base_athome_skills mission_executor --ros-args -p mode:=sim
"""
import json
import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .executor import MissionExecutor, SkillContext
from .places import home_place, load_places, resolve
from .plan import PlanError, parse_plan, validate_plan
from .skills import ERR_CANCELED, ERR_PRECONDITION, SkillResult, default_registry

try:
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover
    NavigateToPose = None

NAV_SUCCEEDED, NAV_CANCELED, NAV_ABORTED = 4, 5, 6


def _share_config(name: str) -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("base_athome_navigation"), "config", name)
    except Exception:
        return ""


class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__("mission_executor")
        self.declare_parameter("mode", "sim")
        self.declare_parameter("supervised", False)
        self.declare_parameter("semantic_map", "")
        self.declare_parameter("waypoints", "")
        self.declare_parameter("nav_server_timeout_s", 5.0)
        # Manipulacion en el robot real: ademas de supervised, hace falta pedirla explicitamente.
        self.declare_parameter("allow_manipulation_real", False)
        self.mode = str(self.get_parameter("mode").value)
        domain = os.environ.get("ROS_DOMAIN_ID", "0")
        if self.mode not in ("sim", "real"):
            raise SystemExit(f"mode debe ser sim o real (recibido {self.mode!r})")
        if self.mode == "sim" and domain == "10":
            raise SystemExit("mode:=sim en el dominio 10 (robot real) RECHAZADO: usar ROS_DOMAIN_ID=42")
        if self.mode == "real" and not bool(self.get_parameter("supervised").value):
            raise SystemExit("mode:=real exige supervised:=true (supervision humana; mueve la base)")
        self.semantic_map = self.get_parameter("semantic_map").value or _share_config("semantic_map.yaml")
        self.waypoints_file = self.get_parameter("waypoints").value or _share_config("waypoints.yaml")
        self.places = load_places(self.semantic_map, self.waypoints_file)

        self.cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_status = self.create_publisher(String, "/mission/status", latched)
        self.pub_skills = self.create_publisher(String, "/mission/skills", latched)
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_mute = self.create_publisher(Bool, "/voice/mute", 10)
        self.create_subscription(String, "/mission/run", self._on_run, 5, callback_group=self.cb)
        self.create_service(Trigger, "/mission/cancel", self._on_cancel, callback_group=self.cb)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cb) \
            if NavigateToPose else None
        self._speaker = None
        self._voice = None
        self._manip = None
        self._manip_lock = threading.Lock()
        self.ros_executor = None  # lo asigna main(): la manipulacion se agrega como nodo del mismo proceso

        self.registry = default_registry()
        handlers = dict(navigate_to=self.h_navigate_to, navigate_sequence=self.h_navigate_sequence,
                        return_home=self.h_return_home, speak=self.h_speak, listen=self.h_listen, wait=self.h_wait,
                        detect_object=self.h_detect, pick_object=self.h_pick, place_object=self.h_place,
                        arm_pose=self.h_arm_pose)
        self.executor_core = MissionExecutor(self.registry, handlers, mode=self.mode, on_update=self._publish)
        self._thread: Optional[threading.Thread] = None
        self.pub_skills.publish(String(data=json.dumps(self.registry.describe())))
        self._publish(self.executor_core.snapshot())
        self.create_timer(1.0, lambda: self._publish(self.executor_core.snapshot()))
        self.get_logger().info(f"mission_executor listo (modo {self.mode}, dominio {domain}); "
                               f"{len(self.places)} lugares: {sorted(self.places)}")

    # ------------------------------------------------------------------ interfaz
    def _publish(self, status: dict):
        self.pub_status.publish(String(data=json.dumps(status)))

    def _reject(self, name: str, errors):
        st = self.executor_core.status
        st.name, st.state, st.message = name, "FAILED", "plan rechazado: " + "; ".join(errors)
        st.steps, st.started_at, st.finished_at = [], time.time(), time.time()
        self.executor_core.log("error", st.message)

    def _on_run(self, msg: String):
        if self.executor_core.running:
            self.get_logger().warn("Ya hay una mision en curso: plan ignorado (cancelar primero)")
            self.executor_core.log("warn", "plan nuevo ignorado: hay una mision en curso")
            return
        try:
            plan = parse_plan(msg.data)
        except PlanError as e:
            self._reject("?", [str(e)])
            return
        self.places = load_places(self.semantic_map, self.waypoints_file)  # releer por si se edito
        errors = validate_plan(plan, self.registry, self.places)
        if errors:
            self.get_logger().error(f"Plan '{plan.name}' rechazado: {errors}")
            self._reject(plan.name, errors)
            return
        self._thread = threading.Thread(target=self.executor_core.run, args=(plan,), daemon=True)
        self._thread.start()

    def _on_cancel(self, _req, resp):
        resp.success = self.executor_core.cancel()
        resp.message = "cancelacion pedida" if resp.success else "no hay mision en curso"
        return resp

    # ------------------------------------------------------------------ navegacion
    def _navigate(self, ctx: SkillContext, name: str, place) -> SkillResult:
        if self.nav is None:
            return SkillResult.failure("nav_unavailable", "nav2_msgs no disponible")
        if not self.nav.wait_for_server(timeout_sec=min(float(self.get_parameter("nav_server_timeout_s").value),
                                                        max(0.1, ctx.remaining()))):
            return SkillResult.failure("nav_unavailable", "la accion navigate_to_pose no responde")
        x, y, yaw = place
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        import math
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        ctx.log("info", f"navigate_to '{name}' -> ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} grados)")
        fut = self.nav.send_goal_async(goal)
        while not fut.done():
            if ctx.cancel.is_set():
                return SkillResult.failure(ERR_CANCELED, "cancelado antes de aceptar el goal")
            time.sleep(0.02)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return SkillResult.failure("goal_rejected", f"Nav2 rechazo el goal a '{name}'")
        res = handle.get_result_async()
        while not res.done():
            if ctx.cancel.is_set():
                handle.cancel_goal_async()
                # Esperar a que el goal termine de verdad: un reintento no debe superponerse al goal viejo.
                t_end = time.time() + 5.0
                while not res.done() and time.time() < t_end:
                    time.sleep(0.02)
                if not res.done():
                    return SkillResult.failure(ERR_CANCELED, f"navegacion a '{name}': cancelacion pedida, "
                                                             "Nav2 no confirmo el fin del goal en 5 s")
                return SkillResult.failure(ERR_CANCELED, f"navegacion a '{name}' cancelada (goal cancelado)")
            time.sleep(0.05)
        status = res.result().status
        if status == NAV_SUCCEEDED:
            return SkillResult.success(f"llegue a '{name}'", place=name)
        if status == NAV_CANCELED:
            return SkillResult.failure("nav_canceled", f"Nav2 cancelo la navegacion a '{name}'")
        return SkillResult.failure("nav_failed", f"Nav2 no pudo llegar a '{name}' (status {status})")

    def h_navigate_to(self, ctx: SkillContext) -> SkillResult:
        name = str(ctx.args["place"])
        place = resolve(self.places, name)
        if place is None:
            return SkillResult.failure("unknown_place", f"'{name}' no esta en el mapa semantico")
        return self._navigate(ctx, name, place)

    def h_navigate_sequence(self, ctx: SkillContext) -> SkillResult:
        reached = []
        for name in ctx.args["places"]:
            place = resolve(self.places, str(name))
            if place is None:
                return SkillResult.failure("unknown_place", f"'{name}' no esta en el mapa semantico", reached=reached)
            r = self._navigate(ctx, str(name), place)
            if not r.ok:
                r.data["reached"] = reached
                r.message = f"{r.message} (alcanzados: {reached})"
                return r
            reached.append(str(name))
        return SkillResult.success(f"recorrido completo: {reached}", reached=reached)

    def h_return_home(self, ctx: SkillContext) -> SkillResult:
        place = home_place(self.places)
        if place is None:
            return SkillResult.failure(ERR_PRECONDITION, "no hay waypoint 'home' ni 'default'")
        return self._navigate(ctx, "home", place)

    # ------------------------------------------------------------------ HRI
    def h_speak(self, ctx: SkillContext) -> SkillResult:
        text = str(ctx.args["text"])
        self.pub_say.publish(String(data=text))
        ctx.log("info", f"speak: {text}")
        if self.mode == "sim":
            ctx.sleep(min(3.0, 0.05 * len(text)))
            return SkillResult.success("dicho (simulado)")
        try:
            if self._speaker is None:
                from base_athome_perception.speaker import RoboCupSpeaker
                self._speaker = RoboCupSpeaker()
            self.pub_mute.publish(Bool(data=True))
            done = threading.Event()
            self._speaker.speak(text, blocking=False, on_done=done.set)
            while not done.wait(0.1):
                if ctx.cancel.is_set():
                    return SkillResult.failure(ERR_CANCELED, "habla cancelada")
            return SkillResult.success("dicho")
        except Exception as e:  # noqa: BLE001
            return SkillResult.failure("tts_failed", f"{type(e).__name__}: {e}")
        finally:
            self.pub_mute.publish(Bool(data=False))

    def h_listen(self, ctx: SkillContext) -> SkillResult:
        try:
            if self._voice is None:
                from base_athome_perception.voice_client import VoiceClient
                self._voice = VoiceClient(self)
        except Exception as e:  # noqa: BLE001
            return SkillResult.failure("asr_unavailable", f"{type(e).__name__}: {e}")
        if ctx.args.get("prompt"):
            r = self.h_speak(SkillContext({"text": ctx.args["prompt"]}, ctx.cancel, ctx.deadline, ctx.log))
            if not r.ok:
                return r
        rid = self._voice.request(prompt="")
        t_end = time.time() + min(float(ctx.args.get("listen_timeout_s", 15.0)), ctx.remaining())
        while time.time() < t_end:
            if ctx.cancel.is_set():
                self._voice.cancel(rid)
                return SkillResult.failure(ERR_CANCELED, "escucha cancelada")
            res = self._voice.poll(rid)
            if res is not None:
                if not res.ok:
                    return SkillResult.failure("no_speech", f"ASR: {res.status}")
                expect = [str(w).lower() for w in (ctx.args.get("expect") or [])]
                if expect and not any(w in res.text.lower() for w in expect):
                    return SkillResult.failure("unexpected_answer", f"oi '{res.text}'", text=res.text)
                return SkillResult.success(f"oi '{res.text}'", text=res.text)
            time.sleep(0.05)
        self._voice.cancel(rid)
        return SkillResult.failure("no_speech", "sin respuesta a tiempo")

    # ------------------------------------------------------------------ manipulacion
    def _manipulation(self):
        """Nodo de manipulacion (base_athome_perception.arm_manipulation) creado al primer uso."""
        if self.mode == "real" and not bool(self.get_parameter("allow_manipulation_real").value):
            return None, "manipulacion real deshabilitada (allow_manipulation_real:=false)"
        with self._manip_lock:
            if self._manip is None:
                try:
                    from base_athome_perception.arm_manipulation import Manipulacion
                except Exception as e:  # noqa: BLE001
                    return None, f"no se pudo cargar arm_manipulation: {type(e).__name__}: {e}"
                self._manip = Manipulacion()
                if self.ros_executor is not None:
                    self.ros_executor.add_node(self._manip)
                t_end = time.time() + 45.0  # move_group puede tardar en arrancar
                while time.time() < t_end and not self._manip.moveit.available():
                    time.sleep(0.2)
        return self._manip, None

    def _with_manip(self, ctx: SkillContext, fn, error: str) -> SkillResult:
        m, why = self._manipulation()
        if m is None:
            return SkillResult.failure("manipulation_unavailable", why)
        stop = threading.Event()

        def watch():  # la cancelacion de la mision corta el movimiento (abort -> cancel del goal FJT)
            while not stop.is_set():
                if ctx.cancel.is_set():
                    m.abort = True
                    return
                time.sleep(0.05)
        threading.Thread(target=watch, daemon=True).start()
        try:
            out = fn(m)
        finally:
            stop.set()
        ok, msg = out[0], out[1]
        data = out[2] if len(out) > 2 else {}
        if ctx.cancel.is_set():
            return SkillResult.failure(ERR_CANCELED, f"cancelado ({msg})", **data)
        return SkillResult.success(msg, **data) if ok else SkillResult.failure(error, msg, **data)

    def h_arm_pose(self, ctx: SkillContext) -> SkillResult:
        pose = str(ctx.args["pose"])
        return self._with_manip(ctx, lambda m: m.go_to_blocking(pose), "arm_failed")

    def h_detect(self, ctx: SkillContext) -> SkillResult:
        label = str(ctx.args["object"])
        t = float(ctx.args.get("detect_timeout_s", 8.0))
        return self._with_manip(ctx, lambda m: m.detect_blocking(label, timeout_s=min(t, ctx.remaining())),
                                "not_found")

    def h_pick(self, ctx: SkillContext) -> SkillResult:
        label = str(ctx.args["object"])
        r = self._with_manip(ctx, lambda m: m.pick_blocking(label), "grasp_failed")
        if not r.ok and "agarre vacio" in r.message:
            r.error = "grasp_empty"
        elif not r.ok and r.message.startswith("no veo"):
            r.error = "not_found"
        return r

    def h_place(self, ctx: SkillContext) -> SkillResult:
        return self._with_manip(ctx, lambda m: m.place_blocking(), "place_failed")

    def h_wait(self, ctx: SkillContext) -> SkillResult:
        if ctx.sleep(float(ctx.args["seconds"])):
            return SkillResult.success(f"espere {ctx.args['seconds']} s")
        return SkillResult.failure(ERR_CANCELED, "espera cancelada")


def preflight(argv, environ) -> Optional[str]:
    """Rechazo ANTES de rclpy.init (no unirse al dominio 10 salvo modo real pedido explicitamente)."""
    if environ.get("ROS_DOMAIN_ID", "0") == "10" and "mode:=real" not in " ".join(argv):
        return "dominio 10 (robot real) sin mode:=real: RECHAZADO antes de unirse al dominio. Usar ROS_DOMAIN_ID=42"
    return None


def main(args=None):
    import sys
    reason = preflight(sys.argv if args is None else args, os.environ)
    if reason:
        print(f"[mission_executor] {reason}")
        raise SystemExit(2)
    rclpy.init(args=args)
    try:
        node = MissionExecutorNode()
    except SystemExit as e:
        print(f"[mission_executor] {e}")
        rclpy.shutdown()
        raise
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(node)
    node.ros_executor = ex
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.executor_core.cancel("nodo detenido")
        if node._manip is not None:
            node._manip.stop_move_group()
            node._manip.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
