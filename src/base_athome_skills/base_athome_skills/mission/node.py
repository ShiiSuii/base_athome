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
import math
import os
import threading
import time
import uuid
from typing import Optional

import numpy as np

import rclpy
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Trigger

from . import bag_task
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
DOCK_TURN_RPS = 0.3
DOCK_SPEED_MPS = 0.08      # tramo final hasta la bolsa
TORSO_TOLERANCE_M = 0.02


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

        # Bolsa senalada: pedido al detector, elevador y tramo final (bag_task.py)
        self.bag_target = None                # (x, y, z) en map, medido por la cabeza
        self.bag_memory = None                # (x, y, z) en base_link, ya junto a la bolsa: la base no se mueve mas
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.robot_pose = None                # (x, y, yaw) en map, de AMCL
        self.odom_xy = None
        self.front_range = None
        self.torso_status = {}
        self.bag_results = {}
        self.pub_torso = self.create_publisher(Float64, "/torso/goal", 5)
        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 5)
        self.pub_bag_req = self.create_publisher(String, "/bag_finder/request", 5)
        latched_pose = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._amcl, latched_pose)
        self.create_subscription(Odometry, "/odom", self._odom, 10)
        self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
        self.create_subscription(String, "/torso/status", self._torso_status, 10)
        self.create_subscription(String, "/bag_finder/result", self._bag_result, 10)

        self.registry = default_registry()
        handlers = dict(navigate_to=self.h_navigate_to, navigate_sequence=self.h_navigate_sequence,
                        return_home=self.h_return_home, speak=self.h_speak, listen=self.h_listen, wait=self.h_wait,
                        detect_object=self.h_detect, pick_object=self.h_pick, receive_bag=self.h_receive_bag, give_object=self.h_give, place_object=self.h_place,
                        arm_pose=self.h_arm_pose, find_pointed_bag=self.h_find_pointed_bag, approach_person=self.h_approach_person,
                        approach_bag=self.h_approach_bag, dock_bag=self.h_dock_bag, remember_bag=self.h_remember_bag, set_torso=self.h_set_torso)
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
        plane_z, memory = None, None
        if ctx.args.get("use_bag_target") and self.bag_target is not None:
            plane_z = self.bag_target[2]     # la cabeza da la altura; la camara de muneca, la posicion
            memory = self.bag_memory
        # con la bolsa no se va a post_grasp: quedaria pegada al cuerpo; viaja adelante, en la pose de piso
        retreat = not ctx.args.get("use_bag_target")
        r = self._with_manip(ctx, lambda m: m.pick_blocking(label, plane_z, retreat=retreat, memory_xyz=memory), "grasp_failed")
        if not r.ok and "agarre vacio" in r.message:
            r.error = "grasp_empty"
        elif not r.ok and r.message.startswith("no veo"):
            r.error = "not_found"
        return r

    def h_receive_bag(self, ctx: SkillContext) -> SkillResult:
        r = self._with_manip(ctx, lambda m: m.receive_blocking(), "grasp_empty")
        if not r.ok and "vacio" in r.message:
            r.error = "grasp_empty"
        return r

    def h_give(self, ctx: SkillContext) -> SkillResult:
        return self._with_manip(ctx, lambda m: m.give_blocking(), "place_failed")

    def h_place(self, ctx: SkillContext) -> SkillResult:
        return self._with_manip(ctx, lambda m: m.place_blocking(), "place_failed")

    # ------------------------------------------------------------------ bolsa senalada
    def _pose_now(self):
        """(x, y, yaw) en map en este instante, por TF. AMCL solo publica cuando la base avanzo lo suficiente: su ultimo
        mensaje puede estar atrasado decimetros, y ese atraso se volvia error de agarre al convertir la bolsa a la base."""
        try:
            t = self.tf_buf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001 - sin TF queda lo ultimo de AMCL
            return self.robot_pose
        q = t.rotation
        return t.translation.x, t.translation.y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _amcl(self, msg):
        p = msg.pose.pose
        self.robot_pose = (p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w))

    def _odom(self, msg):
        self.odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _scan(self, msg):
        """Lo mas cercano adelante (+-20 grados) o None."""
        best = None
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= math.radians(20.0) and msg.range_min < r < msg.range_max and (best is None or r < best):
                best = r
        self.front_range = best

    def _torso_status(self, msg):
        try:
            self.torso_status = json.loads(msg.data)
            self.torso_status["_t"] = time.time()
        except ValueError:
            pass

    def _bag_result(self, msg):
        try:
            d = json.loads(msg.data)
            self.bag_results[str(d.get("id"))] = d
        except ValueError:
            pass

    def _ask_finder(self, ctx: SkillContext, mode: str, timeout: float, **extra):
        """Pedido a bag_finder. Devuelve (respuesta, None) o (None, SkillResult de error)."""
        if self.pub_bag_req.get_subscription_count() == 0:
            return None, SkillResult.failure("finder_unavailable", "bag_finder no esta corriendo")
        rid = uuid.uuid4().hex[:8]
        self.pub_bag_req.publish(String(data=json.dumps(dict(id=rid, mode=mode, timeout_s=timeout, **extra))))
        end = time.time() + timeout + 3.0
        while time.time() < end:
            if ctx.cancel.is_set():
                return None, SkillResult.failure(ERR_CANCELED, "busqueda cancelada")
            res = self.bag_results.pop(rid, None)
            if res is not None:
                if res.get("ok"):
                    return res, None
                return None, SkillResult.failure(str(res.get("error", "bag_not_found")), str(res.get("message", "")))
            time.sleep(0.05)
        return None, SkillResult.failure("finder_unavailable", "bag_finder no respondio")

    def h_approach_person(self, ctx: SkillContext) -> SkillResult:
        # Avisar antes de moverse: sin elevador calibrado la mision fallaria recien junto a la bolsa
        st = self.torso_status
        if time.time() - st.get("_t", 0.0) > 2.0 or not st.get("connected"):
            return SkillResult.failure("torso_unavailable", "el elevador no responde (torso_state)")
        if not st.get("calibrated"):
            return SkillResult.failure("torso_not_calibrated", "elevador sin calibrar: correr calibrar_elevador")
        res, err = self._ask_finder(ctx, "locate_person", min(float(ctx.args.get("find_timeout_s", 30.0)), ctx.remaining()))
        if err is not None:
            return err
        if self.robot_pose is None:
            return SkillResult.failure("no_person", "no se la pose del robot")
        pose = bag_task.person_stand_pose(res["person"], self._pose_now())
        if pose is None:
            return SkillResult.success("la persona ya esta cerca", person=res["person"])
        return self._navigate(ctx, "hacia la persona", pose)

    def h_find_pointed_bag(self, ctx: SkillContext) -> SkillResult:
        self.bag_target, self.bag_memory = None, None
        res, err = self._ask_finder(ctx, "pointed_bag", min(float(ctx.args.get("find_timeout_s", 30.0)), ctx.remaining()))
        if err is not None:
            phrase = bag_task.correction_phrase(err.message)
            if phrase:          # decirle que corregir: el paso se reintenta
                self.pub_say.publish(String(data=phrase))
                ctx.log("info", f"correccion: {phrase}")
            return err
        self.bag_target = tuple(float(v) for v in res["bag"])
        return SkillResult.success(f"bolsa en ({self.bag_target[0]:.2f}, {self.bag_target[1]:.2f}, "
                                   f"{self.bag_target[2]:.2f})", bag=list(self.bag_target))

    def h_remember_bag(self, ctx: SkillContext) -> SkillResult:
        """Ya junto a la bolsa y con el elevador arriba: guarda donde esta. Sin medicion de la muneca, queda la de la cabeza."""
        if self.bag_target is None or self.robot_pose is None:
            return SkillResult.failure("no_bag_target", "no hay bolsa ubicada")
        head = list(bag_task.object_in_base(self.bag_target, self._pose_now()))
        head[2] -= bag_task.GROUND_TO_BASE_LINK_M
        z = self.bag_target[2]
        res = self._with_manip(ctx, lambda m: m.locate_bag_blocking(z, head), "bag_not_found")
        if ctx.cancel.is_set():
            return res
        if res.ok and res.data.get("base_link_xyz"):
            self.bag_memory = tuple(res.data["base_link_xyz"])
            return SkillResult.success(f"bolsa guardada segun la muneca: {[round(v, 3) for v in self.bag_memory]} "
                                       f"(la cabeza decia {[round(v, 3) for v in head]})", bag=list(self.bag_memory))
        self.bag_memory = tuple(head)
        return SkillResult.success(f"la muneca no la vio ({res.message}); guardo lo de la cabeza: "
                                   f"{[round(v, 3) for v in head]}", bag=head)

    def _floor_grasp(self):
        """Geometria de la pose de piso ensenada, leida del archivo en cada pedido. (geometria, motivo de rechazo)."""
        m, why = self._manipulation()
        if m is None:
            return None, why
        m.reload_poses()
        tcp_mm, R = m.kin.forward_kinematics([math.radians(v) for v in m.poses["pre_grasp_floor"]["joints_deg"]])
        tcp = np.asarray(m.shoulder_in_base()) - [0.0, 0.0, m.torso_pos or 0.0] + np.asarray(tcp_mm) / 1000.0
        grasp = bag_task.floor_grasp_from_tcp(tcp, R[:, 2])
        grasp = bag_task.with_reach_margin(grasp)
        clearance = bag_task.base_clearance(grasp=grasp)
        if clearance < bag_task.MIN_BASE_CLEARANCE_M:
            return None, (f"con la pose de piso, una bolsa quedaria a {clearance * 100:.0f} cm del chasis: "
                          "ensenarla mas adelante")
        return grasp, ""

    def _stop_base(self):
        self.pub_cmd_vel.publish(Twist())

    def h_approach_bag(self, ctx: SkillContext) -> SkillResult:
        if self.bag_target is None or self.robot_pose is None:
            return SkillResult.failure("no_bag_target", "no hay bolsa ubicada o no se la pose del robot")
        grasp, why = self._floor_grasp()
        if grasp is None:
            return SkillResult.failure("pose_too_close", why)
        back = bag_task.DOCK_DISTANCE_M + bag_task.BAG_REFINE_BACK_M
        pose = bag_task.stand_pose(self.bag_target, self._pose_now(), extra_back_m=back, grasp=grasp)
        res = self._navigate(ctx, "lejos de la bolsa", pose)
        if not res.ok:
            return res
        # Ya cerca, la cabeza mide mejor. Sin esta medicion sigue con la de lejos: la muneca es la siguiente oportunidad.
        again, err = self._ask_finder(ctx, "refine_bag", min(8.0, ctx.remaining()), near=list(self.bag_target[:2]))
        note = f"no pude volver a medirla: {err.message}" if err is not None else ""
        if err is None:
            moved = math.hypot(again["bag"][0] - self.bag_target[0], again["bag"][1] - self.bag_target[1])
            self.bag_target = tuple(float(v) for v in again["bag"])
            note = f"medida de nuevo, corrijo {moved * 100:.0f} cm"
        pose = bag_task.stand_pose(self.bag_target, self._pose_now(), extra_back_m=bag_task.DOCK_DISTANCE_M, grasp=grasp)
        res = self._navigate(ctx, "al inicio del tramo final", pose)
        return SkillResult.success(f"{res.message}; {note}", bag=list(self.bag_target)) if res.ok else res

    def h_dock_bag(self, ctx: SkillContext) -> SkillResult:
        if self.odom_xy is None:
            return SkillResult.failure("blocked", "sin odometria para el tramo final")
        if self.bag_target is None:
            return SkillResult.failure("no_bag_target", "no hay bolsa ubicada")
        grasp, why = self._floor_grasp()
        if grasp is None:
            return SkillResult.failure("pose_too_close", why)
        start = self.odom_xy
        end = time.time() + bag_task.DOCK_MAX_TRAVEL_M / DOCK_SPEED_MPS * 2.0 + 10.0
        try:
            while time.time() < end:
                if ctx.cancel.is_set():
                    return SkillResult.failure(ERR_CANCELED, "acercamiento cancelado")
                done = math.hypot(self.odom_xy[0] - start[0], self.odom_xy[1] - start[1])
                action, amount = bag_task.dock_step(bag_task.object_in_base(self.bag_target, self._pose_now()), grasp)
                if action == "done":
                    return SkillResult.success(f"junto a la bolsa (tramo final {done:.2f} m)")
                if action == "off":
                    return SkillResult.failure("blocked", f"la bolsa quedo {amount * 100:.0f} cm a un costado: no la alcanzo")
                if done > bag_task.DOCK_MAX_TRAVEL_M:
                    return SkillResult.failure("blocked", f"el tramo final ya lleva {done:.2f} m sin llegar a la bolsa: paro")
                msg = Twist()
                if action == "turn":
                    msg.angular.z = max(-DOCK_TURN_RPS, min(DOCK_TURN_RPS, 1.5 * amount))
                else:
                    stop = bag_task.dock_stop_range_with_arm(grasp, self.bag_target[2])
                    if bag_task.dock_verdict(0.0, self.front_range, stop_range_m=stop) == "blocked":
                        return SkillResult.failure("blocked", f"algo a {self.front_range:.2f} m adelante: paro")
                    msg.linear.x = min(DOCK_SPEED_MPS, max(0.03, amount))
                self.pub_cmd_vel.publish(msg)
                time.sleep(0.05)
            return SkillResult.failure("blocked", "el tramo final no termino a tiempo")
        finally:
            self._stop_base()

    def h_set_torso(self, ctx: SkillContext) -> SkillResult:
        if self.mode == "real" and not bool(self.get_parameter("allow_manipulation_real").value):
            return SkillResult.failure("torso_unavailable", "mover el elevador exige allow_manipulation_real:=true")
        if ctx.args.get("for_bag"):
            if self.bag_target is None:
                return SkillResult.failure("no_bag_target", "no hay bolsa ubicada")
            grasp, why = self._floor_grasp()
            if grasp is None:
                return SkillResult.failure("pose_too_close", why)
            target = bag_task.torso_for_object(bag_task.grasp_height(self.bag_target[2]), grasp=grasp)
        else:
            target = float(ctx.args.get("target_m", 0.0))
        st = self.torso_status
        if time.time() - st.get("_t", 0.0) > 2.0 or not st.get("connected"):
            return SkillResult.failure("torso_unavailable", "torso_state no informa o el elevador no responde")
        if not st.get("calibrated"):
            return SkillResult.failure("torso_not_calibrated", "elevador sin calibrar: mandar CALIBRATE")
        self.pub_torso.publish(Float64(data=float(target)))
        started = time.time()
        while time.time() - started < ctx.remaining():
            if ctx.cancel.is_set():
                self.pub_torso.publish(Float64(data=float("nan")))     # parar
                return SkillResult.failure(ERR_CANCELED, "elevador detenido")
            st = self.torso_status
            if time.time() - st.get("_t", 0.0) > 2.0:
                return SkillResult.failure("torso_unavailable", "torso_state dejo de informar")
            pos = st.get("position_m")
            if time.time() - started > 0.5 and pos is not None and not st.get("moving"):
                if abs(pos - target) <= TORSO_TOLERANCE_M:
                    return SkillResult.success(f"elevador en {pos:+.3f} m", position_m=pos)
                if time.time() - started > 3.0:
                    return SkillResult.failure("torso_timeout", f"elevador quieto en {pos:+.3f} m, objetivo {target:+.3f} m")
            time.sleep(0.05)
        self.pub_torso.publish(Float64(data=float("nan")))
        return SkillResult.failure("torso_timeout", "el elevador no llego a tiempo")

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
