#!/usr/bin/env python3
"""Nodo ROS 2 para la prueba Restaurant (RoboCup@Home 2026.2).

Conecta la maquina de estados RestaurantSession con:
  - Nav2 (navigate_to_pose ActionClient)
  - /bag_finder/request y /bag_finder/result (deteccion de clientes con mano levantada)
  - VoiceClient (ASR neuronal de la Jetson)
  - RoboCupSpeaker y /hri/say_something (TTS neuronal Piper)
  - Pan-tilt Dynamixel (/pan_tilt/target_degrees)
  - LLM local (ask_order en 127.0.0.1:8771)
  - VLM local (ask_vlm / verify_item_on_tray en 127.0.0.1:8772)
  - /restaurant/state (monitoreo del estado JSON en vivo)

Brazo opcional: arm:=true y allow_motion:=true, controlador ya habilitado por el
humano, feedback fresco y cuatro poses calibrated: true. Las poses iniciales en
restaurant_arm_poses.yaml NO habilitan movimiento. En LAB_SUPERVISADO usar
``python3 -m base_athome_perception.restaurant_arm_teach``: GRABAR/GUARDAR del
TeachPanel llama save_pose con articulaciones medidas, solo al archivo extra.
Ensenar pregrasp frente al objeto, postgrasp retirado, carry dentro del chasis y
handover frente al cliente; validar cada recorrido antes de marcar calibrated:
true. No se modifica arm_poses_and_limits.yaml. La entrega conserva la pinza
cerrada hasta release_fn; no habilita torque ni mueve el elevador.
Docking opt-in: restaurant_dock:=true (RESTAURANT_DOCK=1 en el script), exige
arm/allow_motion. Selecciona pick por vision y retrocede antes de Nav2; requiere
scan trasero dedicado y fresco ANTES de acercarse. Real: autorizacion Nivel C.
"""

import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Vector3
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty, String
try:  # slam_toolbox solo hace falta para pausar el SLAM al girar en el lugar
    from slam_toolbox.srv import Pause as SlamPause
except ImportError:  # pragma: no cover - sin slam_toolbox el giro sigue sin pausa
    SlamPause = None
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from base_athome_perception.speaker import RoboCupSpeaker
from base_athome_perception.voice_client import VoiceClient
from base_athome_perception import pose_store
from base_athome_perception.restaurant_arm import (
    RestaurantArm, create_restaurant_manipulation, merge_restaurant_poses,
)

from .restaurant_docking import RestaurantDocking
from .mission import restaurant as rest
from .mission import vlm
from .mission.restaurant_navigation import NavigationStopMonitor, RestaurantNavigation
from .mission.restaurant_session import RestaurantSession, path_length
from .restaurant_interfaces import (
    RestaurantFaces, head_angles, make_face_recognizer, person_map_point,
    supported_kwargs, transform_point,
)


class RestaurantNode(Node):
    """Nodo principal de ejecucion de Restaurant."""

    def __init__(self):
        super().__init__("restaurant_node")
        self.cb_group = ReentrantCallbackGroup()

        # Parametros
        self.declare_parameter("auto_start", False)
        self.declare_parameter("sim_table_retreat", False)
        self._sim_table_retreat = bool(self.get_parameter("sim_table_retreat").value)
        if self._sim_table_retreat and not (
                os.environ.get("ATHOME_SIM") == "1" and os.environ.get("ROS_DOMAIN_ID") == "42"):
            raise ValueError("sim_table_retreat requires simulation in domain 42")
        self._needs_table_retreat = False
        self._previous_state = None
        self.declare_parameter("menu", list(rest.DEFAULT_MENU))
        self.declare_parameter("timeout_s", 900.0)
        self.declare_parameter("tray", True)
        self.declare_parameter("restaurant_dock", False)
        self._dock_enabled = bool(self.get_parameter("restaurant_dock").value)
        self.declare_parameter("arm", False)
        self.declare_parameter("allow_motion", False)
        self.declare_parameter("restaurant_arm_poses_file", "")
        self.declare_parameter("face_height_m", 1.2)  # sentado; face_z medido tiene prioridad
        self.declare_parameter("max_customers", 2)
        self.declare_parameter("max_no_progress_rounds", 3)
        self.declare_parameter("standoff_dist", 0.8)  # 10/11: un poco mas cerca del cliente (antes 0.9)
        self.declare_parameter("delivery_wait_s", 7.0)
        self.declare_parameter("batch_orders", True)
        self.declare_parameter("navigation_timeout_s", 90.0)
        self.navigation_timeout_s = float(self.get_parameter("navigation_timeout_s").value)
        if not math.isfinite(self.navigation_timeout_s) or self.navigation_timeout_s <= 0:
            raise ValueError("navigation_timeout_s must be positive and finite")
        self.declare_parameter("spin_timeout_s", 40.0)
        self.spin_timeout_s = float(self.get_parameter("spin_timeout_s").value)
        if not math.isfinite(self.spin_timeout_s) or self.spin_timeout_s <= 0:
            raise ValueError("spin_timeout_s must be positive and finite")
        self._nav_aborted = False
        self._costmap = None
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_costmap,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=self.cb_group,
        )

        auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value
        self._exit_when_done = auto_start
        menu_param = self.get_parameter("menu").get_parameter_value().string_array_value or list(rest.DEFAULT_MENU)
        timeout_s = self.get_parameter("timeout_s").get_parameter_value().double_value
        use_tray = self.get_parameter("tray").get_parameter_value().bool_value
        max_cust = self.get_parameter("max_customers").get_parameter_value().integer_value
        standoff = self.get_parameter("standoff_dist").get_parameter_value().double_value
        delivery_wait_s = float(self.get_parameter("delivery_wait_s").value)
        batch_orders = self.get_parameter("batch_orders").get_parameter_value().bool_value

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cb_group)
        self._navigation = RestaurantNavigation(self.nav_client, warn_fn=self.get_logger().warn)
        self.spin_client = ActionClient(self, Spin, "spin", callback_group=self.cb_group)
        self._spinning = RestaurantNavigation(self.spin_client, warn_fn=self.get_logger().warn)
        self.backup_client = ActionClient(self, BackUp, "backup", callback_group=self.cb_group)
        self._backing = RestaurantNavigation(self.backup_client, warn_fn=self.get_logger().warn)
        # Pausa del SLAM durante giros en el lugar (decision del usuario 10/11); sin slam_toolbox no hace nada.
        self._slam_pause_cli = (self.create_client(SlamPause, "/slam_toolbox/pause_new_measurements",
                                                   callback_group=self.cb_group) if SlamPause else None)
        self._slam_params_cli = self.create_client(GetParameters, "/slam_toolbox/get_parameters",
                                                   callback_group=self.cb_group)
        self._slam_paused_assumed = False
        self._slam_pause_depth = 0
        # Solo consulta al planner (no mueve el robot): valida que exista camino antes de mandar el goal.
        self.plan_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose", callback_group=self.cb_group)

        # Deteccion de comensales (/bag_finder)
        self.pub_bag_request = self.create_publisher(String, "/bag_finder/request", 5)
        self.sub_bag_result = self.create_subscription(
            String, "/bag_finder/result", self._on_bag_result, 10, callback_group=self.cb_group
        )
        self._bag_pending: Dict[str, Tuple[threading.Event, Dict[str, Any]]] = {}
        self._bag_lock = threading.Lock()

        # Voz y audio
        self.voice = VoiceClient(self)
        try:
            self.speaker = RoboCupSpeaker()
        except Exception as e:
            self.get_logger().warn(f"RoboCupSpeaker no disponible: {e}")
            self.speaker = None

        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_head = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)
        self._audio_ok = self.speaker is not None
        self._face_aiming = False
        self._head_feedback = None
        self.create_subscription(JointState, "/pan_tilt/joint_states", self._on_head_feedback,
                                 qos_profile_sensor_data, callback_group=self.cb_group)

        self.ros_executor = None  # main agrega la unica Manipulacion a este executor, como mission/node.py
        self._manip = None
        self._manip_lock = threading.Lock()
        perception_share = Path(get_package_share_directory("base_athome_perception"))
        extra_poses = (self.get_parameter("restaurant_arm_poses_file").value
                       or str(perception_share / "config/restaurant_arm_poses.yaml"))
        try:
            poses = merge_restaurant_poses(pose_store.load_named_poses(), extra_poses)
        except Exception as exc:
            self.get_logger().warn(f"Restaurant: poses invalidas, solo bandeja ({exc})")
            poses = {}
        self._arm = RestaurantArm(
            self._manipulation, poses, self._arm_permitted,
            unsafe_fn=lambda: self.session.stop(), warn_fn=self.get_logger().warn,
        )
        self._faces = RestaurantFaces(
            lambda db: make_face_recognizer(perception_share / "models", db),
            self._aim_at_person, self._face_frame,
            active_fn=self._session_active, warn_fn=self.get_logger().warn,
        )

        # Monitoreo de estado
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, "/restaurant/state", latched_qos)

        # Camara para verificacion visual con VLM
        self._latest_image_bytes: Optional[bytes] = None
        self._latest_face_frame = None
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self._on_image_raw,
            qos_profile_sensor_data,
            callback_group=self.cb_group,
        )

        # Triggers de inicio
        self.create_subscription(Empty, "/restaurant/start", lambda _: self.start_mission(), 5, callback_group=self.cb_group)

        # Instanciar sesion
        session_kwargs = dict(
            navigate_fn=self._navigate,
            get_costmap_fn=lambda: self._costmap,
            nav_aborted_fn=lambda: self._nav_aborted,
            speak_fn=self._speak,
            listen_fn=self._listen,
            detect_customers_fn=self._detect_customers,
            ask_order_fn=self._ask_order,
            ask_vlm_fn=vlm.ask_vlm,
            get_pose_fn=self._get_pose,
            cancel_nav_fn=self._cancel_nav,
            point_head_fn=self._point_head,
            get_camera_image_fn=self._get_camera_image,
            on_state_change_fn=self._on_session_state_change,
            menu=menu_param,
            max_time_s=timeout_s,
            standoff_dist=standoff,
            delivery_wait_s=delivery_wait_s,
            use_tray=use_tray,
            max_customers=max_cust,
            max_no_progress_rounds=self.get_parameter("max_no_progress_rounds").value,
            plan_fn=self._plan,
            batch_orders=batch_orders,
            spin_fn=self._spin,
            detect_barman_fn=self._detect_barman,
            detect_people_fn=self._detect_people,
            scan_turn_fn=self._scan_turn,
            get_odom_yaw_fn=self._odom_yaw,
            track_person_fn=self._track_person,
            stop_tracking_fn=self._stop_tracking,
        )
        # El otro worker agrega estos parametros en paralelo. Versiones anteriores
        # reciben solo los que aceptan; arm=false deja pick_item_fn=None (bandeja).
        arm_enabled = bool(self.get_parameter("arm").value)
        if self._dock_enabled and not (arm_enabled and self.get_parameter("allow_motion").value):
            raise ValueError("restaurant_dock requires arm and allow_motion")
        self._docking = RestaurantDocking(self, self._session_active, self._arm_permitted) if self._dock_enabled else None
        session_kwargs.update(supported_kwargs(RestaurantSession, dict(
            enroll_face_fn=self._enroll_face,
            identify_face_fn=self._identify_face,
            detect_item_fn=self._detect_item,
            pick_item_fn=self._arm.pick_item if arm_enabled else None,
            dock_fn=(lambda: self._docking.run()) if self._dock_enabled else None,
            undock_fn=(lambda: self._docking.run(reverse=True)) if self._dock_enabled else None,
            present_gripper_fn=self._arm.present_gripper if arm_enabled and not self._dock_enabled else None,
            grip_item_fn=self._arm.grip_item if arm_enabled else None,
            handover_fn=self._arm.handover if arm_enabled else None,
            release_fn=self._arm.release if arm_enabled else None,
            stow_fn=self._arm.stow if arm_enabled else None,
        )))
        self.session = RestaurantSession(**session_kwargs)

        self._head_customer_xy = None
        self._person_tracking_xy = None
        self.create_timer(0.5, self._refresh_customer_tracking, callback_group=self.cb_group)
        self._session_thread: Optional[threading.Thread] = None
        self._session_lock = threading.Lock()

        self.get_logger().info("Nodo Restaurant inicializado listo.")

        if auto_start:
            self.get_logger().info("auto_start=True: iniciando mision inmediatamente.")
            self.start_mission()
        elif sys.stdin.isatty():
            t = threading.Thread(target=self._wait_terminal_key, daemon=True)
            t.start()

    def _wait_terminal_key(self):
        print("\n=======================================================", flush=True)
        print("  RESTAURANT CHALLENGE: Esperando ESPACIO o ENTER para iniciar...", flush=True)
        print("=======================================================\n", flush=True)
        try:
            sys.stdin.readline()
            self.start_mission()
        except Exception:
            pass

    def start_mission(self) -> None:
        """Inicia la ejecucion de la sesion en un hilo secundario."""
        with self._session_lock:
            if self._session_thread and self._session_thread.is_alive():
                self.get_logger().warn("La sesion ya esta en ejecucion.")
                return
            if not self._navigation.reset() or not self._spinning.reset() or not self._backing.reset():
                self.get_logger().error("Hay una operacion Nav2 pendiente de resultado terminal; inicio rechazado.")
                return
            self.get_logger().info("Arrancando mision Restaurant...")
            self._session_thread = threading.Thread(target=self._run_session, daemon=True)
            self._session_thread.start()

    def _run_session(self):
        try:
            # Cada prueba comienza sin perfiles/embeddings de la anterior.
            self._faces.reset()
            ok = self.session.run()
            self.get_logger().info(f"Mision Restaurant terminada. Resultado: {ok}")
        except Exception as e:
            self._cancel_nav()
            self.get_logger().error(f"Error inesperado en sesion Restaurant: {e}")
        finally:
            if self._exit_when_done:     # prueba automatica: al terminar se cierra el nodo y con el la sesion
                rclpy.try_shutdown()

    # ------------------------------------------------------------------ Callbacks e Interfaces
    def _on_costmap(self, msg: OccupancyGrid) -> None:
        if msg.header.frame_id == "map":
            self._costmap = msg

    def _navigate(self, x: float, y: float, yaw: float, *, deadline_s: float) -> bool:
        """Navega en map, vigilando stop y ambos plazos hasta el resultado terminal."""
        self._nav_aborted = False
        retreat_needed = getattr(self, "_needs_table_retreat", False)
        if retreat_needed and not self._back_away_from_table():
            self.session.stop()
            return False
        if retreat_needed and math.hypot(x, y) < 0.5:
            if not self._navigate_goal(1.79, -0.10, -math.pi / 2, deadline_s=deadline_s):
                return False
        self.get_logger().info(f"Nav2 -> x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        ok = self._navigate_goal(x, y, yaw, deadline_s=deadline_s)
        if ok and retreat_needed:
            self._needs_table_retreat = False
        return ok

    def _navigate_goal(self, x: float, y: float, yaw: float, *, deadline_s: float) -> bool:
        """Ejecuta un NavigateToPose con monitor de stop para un tramo."""

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = float(qz)
        goal_msg.pose.pose.orientation.w = float(qw)

        monitor = NavigationStopMonitor(self.voice, self.session.stop)
        try:
            ok = self._navigation.navigate(
                goal_msg, deadline_s=deadline_s, timeout_s=self.navigation_timeout_s,
                stop_requested_fn=lambda: self.session.stopped, monitor_fn=monitor.poll,
            )
            self._nav_aborted = self._navigation.last_status == GoalStatus.STATUS_ABORTED
            return ok
        finally:
            monitor.close()

    # ------------------------------------------------------------------ SLAM en giros
    # Prueba real 10/11: girando en el lugar (0,4 rad/s) slam_toolbox movio al robot 5,5 m en el mapa;
    # despues Nav2 lo creia dentro de un obstaculo. Decision del usuario: pausar las mediciones nuevas
    # del SLAM durante cada giro (la odometria de ruedas lo maneja bien) y reanudarlas siempre al final.
    def _slam_call(self, client, request, timeout_s=2.0):
        if client is None or not client.wait_for_service(timeout_sec=0.5):
            return None
        fut = client.call_async(request)
        end = time.monotonic() + timeout_s
        while not fut.done() and time.monotonic() < end:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def _slam_paused_state(self):
        """True/False segun el parametro paused_new_measurements; None si no se puede leer."""
        req = GetParameters.Request()
        req.names = ["paused_new_measurements"]
        res = self._slam_call(getattr(self, "_slam_params_cli", None), req)
        if res is None or not res.values or res.values[0].type != 1:  # 1 = PARAMETER_BOOL
            return None
        return bool(res.values[0].bool_value)

    def _set_slam_paused(self, want: bool) -> bool:
        if getattr(self, "_slam_pause_cli", None) is None:
            return False
        state = self._slam_paused_state()
        if state is None:
            state = self._slam_paused_assumed
        if state == want:
            return True
        if self._slam_call(self._slam_pause_cli, SlamPause.Request()) is None:
            self.get_logger().warn(f"No pude {'pausar' if want else 'reanudar'} el SLAM (servicio no responde).")
            return False
        self._slam_paused_assumed = want
        check = self._slam_paused_state()
        if check is not None and check != want:  # alterno de mas: volver a alternar una vez
            self._slam_call(self._slam_pause_cli, SlamPause.Request())
            self._slam_paused_assumed = not check
        self.get_logger().info("SLAM " + ("pausado para girar en el lugar." if want else "reanudado."))
        return True

    def _slam_pause_begin(self):
        self._slam_pause_depth = getattr(self, "_slam_pause_depth", 0) + 1
        if self._slam_pause_depth == 1:
            self._set_slam_paused(True)

    def _slam_pause_end(self):
        self._slam_pause_depth = max(0, getattr(self, "_slam_pause_depth", 1) - 1)
        if self._slam_pause_depth == 0:
            self._set_slam_paused(False)

    def _spin(self, angle_rad: float) -> bool:
        """Spin relativo con colisiones de Nav2, plazo acotado y el mismo stop que navegar; SLAM pausado."""
        if self.session.stopped or self.session.remaining_s() <= 0:
            return False
        timeout_s = min(self.spin_timeout_s, self.session.remaining_s())
        goal = Spin.Goal()
        goal.target_yaw = float(angle_rad)
        goal.time_allowance = Duration(seconds=timeout_s).to_msg()
        monitor = NavigationStopMonitor(self.voice, self.session.stop)
        pause_end = getattr(self, "_slam_pause_end", None)
        if getattr(self, "_slam_pause_begin", None):
            self._slam_pause_begin()
        try:
            return self._spinning.navigate(
                goal, deadline_s=self.session.deadline_s, timeout_s=timeout_s,
                stop_requested_fn=lambda: self.session.stopped, monitor_fn=monitor.poll,
            )
        finally:
            monitor.close()
            if pause_end:
                pause_end()

    def _back_away_from_table(self) -> bool:
        """Ensayo opt-in de simulacion; BackUp conserva el chequeo de colision de Nav2."""
        self._speak("I will move backwards a little before leaving the table.")
        if self.session.stopped or self.session.remaining_s() <= 0:
            return False
        timeout_s = min(30.0, self.session.remaining_s())
        goal = BackUp.Goal()
        goal.target.x = -0.45
        goal.speed = 0.05
        goal.time_allowance = Duration(seconds=timeout_s).to_msg()
        monitor = NavigationStopMonitor(self.voice, self.session.stop)
        try:
            return self._backing.navigate(
                goal, deadline_s=self.session.deadline_s, timeout_s=timeout_s,
                stop_requested_fn=lambda: self.session.stopped, monitor_fn=monitor.poll)
        finally:
            monitor.close()

    def _plan(self, pose: Tuple[float, float, float], timeout_s: float = 2.0) -> Optional[float]:
        """Largo del plan de Nav2 desde la pose actual, None sin camino; lanza si no hay planner."""
        if not self.plan_client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError("compute_path_to_pose no disponible")
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(pose[0])
        goal.goal.pose.position.y = float(pose[1])
        goal.goal.pose.orientation.z = math.sin(pose[2] / 2.0)
        goal.goal.pose.orientation.w = math.cos(pose[2] / 2.0)
        goal.use_start = False
        done = threading.Event()
        send = self.plan_client.send_goal_async(goal)
        send.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            raise RuntimeError("compute_path_to_pose no respondio")
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("compute_path_to_pose rechazo el pedido")
        done.clear()
        res_future = handle.get_result_async()
        res_future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            handle.cancel_goal_async()
            raise RuntimeError("compute_path_to_pose sin resultado")
        res = res_future.result()
        poses = res.result.path.poses if res is not None else []
        ok = (res is not None and res.status == GoalStatus.STATUS_SUCCEEDED
              and res.result.error_code == 0 and len(poses) > 0)
        length = path_length([(p.pose.position.x, p.pose.position.y) for p in poses]) if ok else None
        self.get_logger().info(
            f"Plan a x={pose[0]:.2f}, y={pose[1]:.2f}: "
            + (f"{length:.2f} m" if ok else f"sin camino (error {res.result.error_code if res else '?'})"))
        return length

    def _cancel_nav(self) -> None:
        """Marca stop incluso antes de recibir un handle; _navigate espera su fin."""
        if getattr(self, "_docking", None) is not None:
            self._docking.cancel()
        self._navigation.cancel()
        self._spinning.cancel()
        backing = getattr(self, "_backing", None)
        if backing is not None:
            backing.cancel()

    def _ask_order(self, text: str, menu: List[str]) -> List[str]:
        remaining = self.session.remaining_s()
        if self.session.stopped or remaining <= 0:
            return []
        return rest.ask_order(text, menu, timeout_s=min(rest.LLM_TIMEOUT_S, remaining))

    def _speak(self, text: str) -> None:
        """Bloquea hasta terminar el audio; si falla, impide el movimiento anunciado."""
        self.get_logger().info(f"🗣️ Robot: '{text}'")
        self.pub_say.publish(String(data=text))
        if self.speaker:
            try:
                self.speaker.speak(text, blocking=True)
            except Exception as e:
                self._audio_ok = False
                self.session.stop()
                self.get_logger().warn(f"Error reproduciendo con RoboCupSpeaker: {e}")
        else:
            self._audio_ok = False
            self.session.stop()

    def _listen(self, timeout_s: float = 15.0) -> Optional[str]:
        """Escucha la respuesta del interlocutor con VoiceClient."""
        timeout_s = min(timeout_s, self.session.remaining_s())
        if self.session.stopped or timeout_s <= 0:
            return ""
        self.get_logger().info(f"Escuchando comensal (timeout={timeout_s}s)...")
        try:
            res = self.voice.listen(prompt="What would you like to order?", timeout=timeout_s)
            if res and res.ok:
                self.get_logger().info(f"Comensal: '{res.text}'")
                return res.text
        except Exception as e:
            self.get_logger().warn(f"Error en escucha VoiceClient: {e}")
        return ""

    def _detect_people(self, timeout_s: float, min_frames: int):
        """Todas las personas que ve la camara al frente, con posicion en el mapa (sin mano levantada)."""
        req_id = f"rest_people_{int(time.time() * 1000)}"
        ev = threading.Event()
        with self._bag_lock:
            self._bag_pending[req_id] = (ev, {})
        self.pub_bag_request.publish(String(data=json.dumps({
            "id": req_id, "mode": "people_here", "view_s": 2.0,
            "timeout_s": float(timeout_s), "min_frames": int(min_frames),
        })))
        ev.wait(timeout=max(0.0, min(timeout_s + 5.0, self.session.remaining_s())))
        with self._bag_lock:
            _, res_data = self._bag_pending.pop(req_id, (None, {}))
        if not res_data or not res_data.get("ok", False):
            self.get_logger().warn(f"Sin respuesta de personas de bag_finder ({res_data.get('error', 'timeout')}).")
            return []
        people = res_data.get("customers", [])
        for p in people:
            self.get_logger().info(f"Persona: xy={p.get('xy')} range_m={p.get('range_m')}")
        return people

    SCAN_TURN_MAX_S = 45.0

    def _scan_turn(self, on_first_step=None):
        """Vuelta continua de 360 grados con Nav2 Spin mientras bag_finder detecta (people_here largo).

        Con on_first_step: gira 30 grados, llama (p. ej. avisar por voz) y completa la vuelta.
        Devuelve (personas, giro_ok). Cada persona trae heading_deg en odom.
        """
        req_id = f"rest_scan_{int(time.time() * 1000)}"
        ev = threading.Event()
        with self._bag_lock:
            self._bag_pending[req_id] = (ev, {})
        # La busqueda de clientes contiene dos acciones Spin y una locucion
        # bloqueante entre ambas. El detector debe seguir activo hasta el stop_scan,
        # tambien si la simulacion tarda en completar los giros.
        scan_limit = self.SCAN_TURN_MAX_S
        if on_first_step is not None:
            scan_limit += self.spin_timeout_s + 15.0
        budget = min(scan_limit, max(1.0, self.session.remaining_s()))
        self.pub_bag_request.publish(String(data=json.dumps({
            "id": req_id, "mode": "people_here", "view_s": budget, "timeout_s": budget + 2.0, "min_frames": 2,
        })))
        time.sleep(1.5)  # bag_finder lleva la cabeza al frente y la deja quieta antes de girar
        self._slam_pause_begin()
        try:
            ok = self._scan_turn_spins(on_first_step)
        finally:
            self._slam_pause_end()
        self.pub_bag_request.publish(String(data=json.dumps({"id": req_id, "mode": "stop_scan"})))
        ev.wait(timeout=5.0)
        with self._bag_lock:
            _, res = self._bag_pending.pop(req_id, (None, {}))
        people = res.get("customers", []) if res and res.get("ok") else []
        if not ok:
            self.get_logger().warn("Vuelta incompleta: Nav2 no dejo girar; se busca con la cabeza.")
        for p in people:
            self.get_logger().info(f"Vuelta: persona xy={p.get('xy')} range_m={p.get('range_m')} "
                                   f"heading={p.get('heading_deg')} mano={p.get('raised_frames')}")
        return people, ok

    def _scan_turn_spins(self, on_first_step):
        if on_first_step is None:
            ok = self._spin(2.0 * math.pi)  # Nav2 corta si ve colision (p. ej. pegado a la barra)
        else:
            ok = self._spin(math.pi / 6)
            if ok and not self.session.stopped:
                on_first_step()
            ok = ok and not self.session.stopped and self._spin(2.0 * math.pi - math.pi / 6)
        return ok

    def _odom_yaw(self):
        """Yaw de base_footprint en odom (rad): odometria de ruedas, sin saltos de SLAM."""
        try:
            q = self.tf_buffer.lookup_transform("odom", "base_footprint", rclpy.time.Time()).transform.rotation
            return 2.0 * math.atan2(q.z, q.w)
        except Exception:
            return None

    def _detect_barman(self, timeout_s: float, min_frames: int):
        return self._detect_customers(timeout_s, min_frames, pans=[0.0], stop_on_first=True)

    def _detect_customers(self, timeout_s: float = 45.0, min_frames: int = 3,
                          *, pans=None, stop_on_first: bool = False):
        """Pide a /bag_finder detectar clientes con la mano alzada."""
        req_id = f"rest_cust_{int(time.time() * 1000)}"
        ev = threading.Event()
        with self._bag_lock:
            self._bag_pending[req_id] = (ev, {})

        req_data = json.dumps({
            "id": req_id,
            "mode": "raised_hands",
            "pans": [-90.0, 90.0, 0.0] if pans is None else pans,
            "head_dps": 12.0,
            "view_s": 2.5,
            "stop_on_first": stop_on_first,
            "timeout_s": float(timeout_s),
            "min_frames": int(min_frames),
        })
        self.pub_bag_request.publish(String(data=req_data))

        ev.wait(timeout=min(timeout_s + 5.0, self.session.remaining_s()))
        with self._bag_lock:
            _, res_data = self._bag_pending.pop(req_id, (None, {}))

        if not res_data or not res_data.get("ok", False):
            self.get_logger().warn(f"Sin respuesta de clientes de bag_finder ({res_data.get('error', 'timeout')}).")
            return res_data or {"ok": False, "error": "timeout", "customers": []}

        customers = res_data.get("customers", [])
        for c in customers:
            self.get_logger().info(f"Cliente detectado: xy={c.get('xy')} range_m={c.get('range_m')}")
        return customers

    def _on_bag_result(self, msg: String):
        try:
            data = json.loads(msg.data)
            rid = str(data.get("id", ""))
            with self._bag_lock:
                if rid in self._bag_pending:
                    ev, _ = self._bag_pending[rid]
                    self._bag_pending[rid] = (ev, data)
                    ev.set()
        except Exception:
            pass

    def _get_pose(self) -> Optional[Tuple[float, float, float]]:
        """Consulta TF sin bloquear; la sesion espera una pose valida con plazo."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            quaternion = t.transform.rotation
            values = (tx, ty, quaternion.x, quaternion.y, qz, qw)
            if not all(math.isfinite(v) for v in values) or sum(v * v for v in values[2:]) < 1e-12:
                return None
            yaw = 2.0 * math.atan2(qz, qw)
            return (float(tx), float(ty), float(yaw))
        except Exception:
            return None

    def _point_head(self, pan_deg: float, tilt_deg: float) -> None:
        """Mueve la cabeza pan-tilt para apuntar al cliente."""
        msg = Vector3()
        msg.x = float(pan_deg)
        msg.y = float(tilt_deg)
        msg.z = 0.0
        self.pub_head.publish(msg)

    def _on_image_raw(self, msg: Image):
        try:
            import cv2
            from cv_bridge import CvBridge

            bridge = CvBridge()
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._latest_face_frame = (cv_img.copy(), time.monotonic())
            ok, buf = cv2.imencode(".jpg", cv_img)
            if ok:
                self._latest_image_bytes = buf.tobytes()
        except Exception:
            pass

    def _get_camera_image(self) -> Optional[bytes]:
        return self._latest_image_bytes

    def _session_active(self):
        session = getattr(self, "session", None)
        return session is not None and not session.stopped and session.remaining_s() > 0

    def _arm_permitted(self):
        return (bool(self.get_parameter("arm").value)
                and bool(self.get_parameter("allow_motion").value)
                and self._audio_ok and self._session_active())

    def _manipulation(self):
        """Una instancia headless, sin competir con otro nodo de manipulacion activo."""
        with self._manip_lock:
            if self._manip is None:
                if self.ros_executor is None or not self._arm_permitted():
                    return None
                if any(name in {"manipulacion_simple", "manipulacion_autonoma_node", "mission_executor"}
                       for name in self.get_node_names()):
                    self.get_logger().warn("Otro ejecutor de brazo esta activo; Restaurant usara la bandeja.")
                    return None
                self._manip = create_restaurant_manipulation(self._arm.poses, self._arm_permitted)
                self.ros_executor.add_node(self._manip)
                # Solo espera feedback; no habilita, no mueve y no cambia limites.
                end = time.monotonic() + min(3.0, self.session.remaining_s())
                while self._session_active() and time.monotonic() < end and not self._manip.motion_ready():
                    time.sleep(0.05)
            return self._manip

    def _detect_item(self, item: str, timeout_s: float) -> bool:
        if not self._session_active():
            return False
        if self._dock_enabled and not self._arm.present_gripper():
            # Wrist camera must face the counter before asking vision to detect.
            return False
        return self._arm.detect_item(item, min(timeout_s, self.session.remaining_s()))

    def _face_frame(self):
        return self._latest_face_frame

    def _with_face_target(self, fn):
        self._face_aiming = True
        try:
            self._refresh_customer_tracking()
            return fn()
        finally:
            self._face_aiming = False

    def _enroll_face(self, label: str, person: Optional[dict] = None) -> bool:
        ok = self._with_face_target(lambda: self._faces.enroll(label, person))
        logger = getattr(self, "get_logger", None)
        if logger is not None and ok:
            logger().info(f"Cara guardada: {label}")
        elif logger is not None:
            why = ("sin audio (no se apunta la cabeza)" if not getattr(self, "_audio_ok", True)
                   else "no vi una sola cara centrada y nitida")
            logger().warn(f"Cara NO guardada para {label}: {why}")
        return ok

    def _identify_face(self, person: dict) -> Optional[str]:
        label = self._with_face_target(lambda: self._faces.identify(person))
        logger = getattr(self, "get_logger", None)
        if logger is not None:
            logger().info(f"Cara en xy={person.get('xy') if isinstance(person, dict) else None}: {label or 'desconocida'}")
        return label



    def _on_head_feedback(self, msg):
        joints = dict(zip(msg.name, msg.position))
        if all(name in joints and math.isfinite(joints[name]) for name in ("pan_joint", "tilt_joint")):
            self._head_feedback = (math.degrees(joints["pan_joint"]),
                                   math.degrees(joints["tilt_joint"]), time.monotonic())

    def _aim_at_person(self, person) -> bool:
        """map -> pan_tilt_base_link por TF completo; confirma apuntado con feedback."""
        if not self._audio_ok or not self._session_active():
            return False
        point = person_map_point(person, float(self.get_parameter("face_height_m").value))
        if point is None:
            return False
        try:
            tf = self.tf_buffer.lookup_transform("pan_tilt_base_link", "map", rclpy.time.Time())
            angles = head_angles(transform_point(point, tf.transform))
            if angles is None:
                return False
        except Exception:
            return False
        was_aiming = self._face_aiming
        self._face_aiming = True
        try:
            self._refresh_customer_tracking()  # cancelar lease de bag_finder mientras apuntamos
            sent = time.monotonic()
            self._point_head(*angles)
            end = sent + min(8.0, self.session.remaining_s())
            while self._session_active() and time.monotonic() < end:
                feedback = self._head_feedback
                if (feedback and feedback[2] > sent and time.monotonic() - feedback[2] < 0.5
                        and max(abs(feedback[i] - angles[i]) for i in (0, 1)) < 3.0):
                    return True
                time.sleep(0.05)
            return False
        finally:
            self._face_aiming = was_aiming

    def _track_person(self, person: dict) -> None:
        point = person_map_point(person)
        self._person_tracking_xy = list(point[:2]) if point is not None else None
        self._refresh_customer_tracking()

    def _stop_tracking(self) -> None:
        self._person_tracking_xy = None
        self._head_customer_xy = None
        self._refresh_customer_tracking()

    def _refresh_customer_tracking(self):
        """Lease corto: si Restaurant termina o se cae, la cabeza deja de seguir."""
        xy = getattr(self, "_person_tracking_xy", None) or self._head_customer_xy
        if self.session.stopped or self.session.remaining_s() <= 0:
            xy = None
        if getattr(self, "_face_aiming", False):
            xy = None
        self.pub_bag_request.publish(String(data=json.dumps({
            "mode": "track_customer", "xy": xy,
        })))

    def _on_session_state_change(self, state_dict: Dict[str, Any]) -> None:
        """Publica el estado actual en /restaurant/state."""
        state = state_dict.get("state")
        if getattr(self, "_sim_table_retreat", False):
            if state == "STARTING":
                self._needs_table_retreat = False
            elif (state in {"APPROACHING_CUSTOMER", "RETURNING_TO_BAR", "RETURNING_TO_BAR_FINAL"}
                  and self._previous_state in {"CONFIRMING_ORDER", "CHECKING_ORDER", "TAKING_ORDER", "WAITING_CUSTOMER"}):
                self._needs_table_retreat = True
            self._previous_state = state
        active = state in {"APPROACHING_CUSTOMER", "GREETING", "TAKING_ORDER", "CONFIRMING_ORDER", "DELIVERING_TO_CUSTOMER"}
        customers = self.session.customers
        idx = state_dict.get("customer_idx", 0)
        self._head_customer_xy = (customers[idx].get("xy") if active and 0 <= idx < len(customers) else None)
        self._refresh_customer_tracking()
        msg = String(data=json.dumps(state_dict))
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RestaurantNode()
    executor = MultiThreadedExecutor()
    node.ros_executor = executor
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.session.stop()
        if node._manip is not None:
            node._manip.abort = True
            node._manip.stop_move_group()
            executor.remove_node(node._manip)
            node._manip.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
