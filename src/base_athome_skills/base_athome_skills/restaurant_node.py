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
"""

import json
import math
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Vector3
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from base_athome_perception.speaker import RoboCupSpeaker
from base_athome_perception.voice_client import VoiceClient

from .mission import restaurant as rest
from .mission import vlm
from .mission.restaurant_navigation import NavigationStopMonitor, RestaurantNavigation
from .mission.restaurant_session import RestaurantSession


class RestaurantNode(Node):
    """Nodo principal de ejecucion de Restaurant."""

    def __init__(self):
        super().__init__("restaurant_node")
        self.cb_group = ReentrantCallbackGroup()

        # Parametros
        self.declare_parameter("auto_start", False)
        self.declare_parameter("menu", list(rest.DEFAULT_MENU))
        self.declare_parameter("timeout_s", 900.0)
        self.declare_parameter("tray", True)
        self.declare_parameter("max_customers", 2)
        self.declare_parameter("standoff_dist", 0.9)
        self.declare_parameter("navigation_timeout_s", 90.0)
        self.navigation_timeout_s = float(self.get_parameter("navigation_timeout_s").value)
        if not math.isfinite(self.navigation_timeout_s) or self.navigation_timeout_s <= 0:
            raise ValueError("navigation_timeout_s must be positive and finite")
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

        # TF Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=self.cb_group)
        self._navigation = RestaurantNavigation(self.nav_client, warn_fn=self.get_logger().warn)

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

        # Monitoreo de estado
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, "/restaurant/state", latched_qos)

        # Camara para verificacion visual con VLM
        self._latest_image_bytes: Optional[bytes] = None
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
        self.session = RestaurantSession(
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
            use_tray=use_tray,
            max_customers=max_cust,
        )

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
            if not self._navigation.reset():
                self.get_logger().error("Hay una operacion Nav2 pendiente de resultado terminal; inicio rechazado.")
                return
            self.get_logger().info("Arrancando mision Restaurant...")
            self._session_thread = threading.Thread(target=self._run_session, daemon=True)
            self._session_thread.start()

    def _run_session(self):
        try:
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
        self.get_logger().info(f"Nav2 -> x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

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

    def _cancel_nav(self) -> None:
        """Marca stop incluso antes de recibir un handle; _navigate espera su fin."""
        self._navigation.cancel()

    def _ask_order(self, text: str, menu: List[str]) -> List[str]:
        remaining = self.session.remaining_s()
        if self.session.stopped or remaining <= 0:
            return []
        return rest.ask_order(text, menu, timeout_s=min(rest.LLM_TIMEOUT_S, remaining))

    def _speak(self, text: str) -> None:
        """Emite voz con Piper TTS y publica en /hri/say_something."""
        self.get_logger().info(f"🗣️ Robot: '{text}'")
        self.pub_say.publish(String(data=text))
        if self.speaker:
            try:
                self.speaker.speak(text, blocking=True)
            except Exception as e:
                self.get_logger().warn(f"Error reproduciendo con RoboCupSpeaker: {e}")

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

    def _detect_customers(self, timeout_s: float = 45.0, min_frames: int = 3) -> List[Dict[str, Any]]:
        """Pide a /bag_finder detectar clientes con la mano alzada."""
        req_id = f"rest_cust_{int(time.time() * 1000)}"
        ev = threading.Event()
        with self._bag_lock:
            self._bag_pending[req_id] = (ev, {})

        req_data = json.dumps({
            "id": req_id,
            "mode": "raised_hands",
            "timeout_s": float(timeout_s),
            "min_frames": int(min_frames),
        })
        self.pub_bag_request.publish(String(data=req_data))

        ev.wait(timeout=min(timeout_s + 5.0, self.session.remaining_s()))
        with self._bag_lock:
            _, res_data = self._bag_pending.pop(req_id, (None, {}))

        if not res_data or not res_data.get("ok", False):
            self.get_logger().warn(f"Sin respuesta de clientes de bag_finder ({res_data.get('error', 'timeout')}).")
            return []

        customers = res_data.get("customers", [])
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
            ok, buf = cv2.imencode(".jpg", cv_img)
            if ok:
                self._latest_image_bytes = buf.tobytes()
        except Exception:
            pass

    def _get_camera_image(self) -> Optional[bytes]:
        return self._latest_image_bytes

    def _on_session_state_change(self, state_dict: Dict[str, Any]) -> None:
        """Publica el estado actual en /restaurant/state."""
        msg = String(data=json.dumps(state_dict))
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RestaurantNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.session.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
