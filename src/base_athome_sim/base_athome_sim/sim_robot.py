#!/usr/bin/env python3
"""Robot simulado Base@Home: mismos topicos/servicios que la Jetson (Piper, pan-tilt, whisper),
la RealSense de la NUC, la camara de muneca y Nav2 (navigate_to_pose, /amcl_pose, /scan, TF).

Las apps reales (receptionist, manipulacion, mision) corren SIN CAMBIOS contra este nodo con
ATHOME_SIM=1 y un ROS_DOMAIN_ID distinto del robot (por defecto 42, ver alias `sim_env`).

Control del "mundo" (alias en scripts/aliases/nuc_aliases.sh):
  /sim/speech  (String)  lo que dice el invitado; solo se escucha con la ventana de escucha
                         abierta, igual que whisper_node en modo bajo demanda    -> sim_decir "..."
  /sim/guest   (String)  "door" (invitado en la puerta, mirando al robot) | "none" | "next"
  /sim/auto_guest (Bool) responde solo cada vez que el robot abre la ventana de escucha
"""
import glob
import json
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped, Twist, Vector3
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
try:
    from control_msgs.action import FollowJointTrajectory
except ImportError:  # pragma: no cover
    FollowJointTrajectory = None
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import SetBool, Trigger
from base_athome_sim import sim_scene
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from base_athome_sim import gz_model
from base_athome_sim.sim_core import REST_DEG, SimArm, SimBase, SimMicrophone, SimPanTilt

D2R = math.pi / 180.0
WS = os.path.expanduser("~/base_athome_ws")
ARM_CONFIG = os.path.join(WS, "src/base_athome_perception/config/arm_poses_and_limits.yaml")
DEFAULT_FACES = ["/opt/ros/jazzy/share/rqt_image_view/resource/lena.png", "/usr/share/pixmaps/faces/surfer.jpg"]
AUTO_LINES = ["My name is Robin and my favorite drink is iced tea", "yes",
              "My name is Charlie and I like orange juice", "yes"]


def yaw_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def to_image(arr, encoding, stamp, frame):
    m = Image()
    m.header.stamp, m.header.frame_id = stamp, frame
    m.height, m.width = arr.shape[:2]
    m.encoding = encoding
    m.step = arr.strides[0]
    m.data = arr.tobytes()
    return m


class SimRobot(Node):
    BAG_KNOT_BELOW_TOP_M, BAG_KNOT_MM = 0.04, 10.0

    def __init__(self):
        super().__init__("sim_robot")
        p = self.declare_parameter
        p("map_yaml", os.path.expanduser("~/mapa_base_athome.yaml"))
        p("start_pose", [0.0, 0.0, 0.0])          # x, y, yaw (si hay waypoint 'home' se usa ese)
        p("object_xyz", [0.39, -0.11, 0.92])      # base_footprint: ~12 cm de avance desde la PRE-GRASP actual (el limite de IK esta en ~14-15 cm)
        p("object_diameter_m", 0.075)              # = object_diameter_m de la calibracion de muneca; la pinza se detiene aqui
        p("guest_distance_m", 1.2)
        p("faces", [""])                           # imagenes de rostros (vacio = publicas del sistema)
        p("auto_guest", False)
        p("camera_rate_hz", 15.0)
        p("nav_speed", 0.5)
        p("gazebo", False)                         # True: base, lidar y camaras vienen de gz sim
        gp = lambda n: self.get_parameter(n).value

        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.arm, self.pt = SimArm(), SimPanTilt()
        self.mic = SimMicrophone()
        self.auto_guest = bool(gp("auto_guest"))
        self.auto_idx = 0
        self.listen_count = 0
        self.prompt = ""
        wp = self._waypoints().get("home")
        sx, sy, syaw = (wp["x"], wp["y"], math.radians(wp.get("yaw_deg", 0))) if wp else gp("start_pose")
        self.base = SimBase(sx, sy, syaw, speed=float(gp("nav_speed")))
        self.cmd_vel = (0.0, 0.0, 0.0)
        self.obj = list(gp("object_xyz"))
        if os.environ.get("ATHOME_SIM_OBJECT_XYZ"):  # pruebas de alcance: "x,y,z" en base_footprint
            self.obj = [float(v) for v in os.environ["ATHOME_SIM_OBJECT_XYZ"].split(",")]
        self.obj_attached = False
        self.gripper_was_closed = True
        # Bolsa fija en el mapa (ATHOME_SIM_BAG="x,y,z", z = la manija): sin esto el objeto sigue al robot
        self.bag_map = [float(v) for v in os.environ["ATHOME_SIM_BAG"].split(",")] if os.environ.get("ATHOME_SIM_BAG") else None
        self.person_xy = [float(v) for v in os.environ.get("ATHOME_SIM_PERSON", "0.5,1.8").split(",")]
        self.person_pointing = False
        self.person_points = os.environ.get("ATHOME_SIM_PERSON_POINTS", "1") == "1"   # 0: nunca hace el gesto
        self.point_err = float(os.environ.get("ATHOME_SIM_POINT_ERR_M", "0.15"))     # la gente no apunta con precision
        self.person_sway = float(os.environ.get("ATHOME_SIM_PERSON_SWAY_M", "0.0"))
        self.bag2 = [float(v) for v in os.environ["ATHOME_SIM_BAG2"].split(",")] if os.environ.get("ATHOME_SIM_BAG2") else None
        self.wrist_tilt_err = math.radians(float(os.environ.get("ATHOME_SIM_WRIST_TILT_ERR_DEG", "0")))   # montaje real distinto del modelo
        self.head_pix_err = float(os.environ.get("ATHOME_SIM_DEPTH_NOISE_MM", "5"))
        self.wrist_ghost = os.environ.get("ATHOME_SIM_WRIST_GHOST", "1") == "1"       # falso positivo sobre la garra
        self.rng = np.random.default_rng(0)
        self.torso_pos, self.torso_goal = 0.0, 0.0
        self.bag_delivered = False
        self.pub_torso_js = self.create_publisher(JointState, "/torso/joint_states", 10)
        self.pub_torso_st = self.create_publisher(String, "/torso/status", 10)
        self.pub_people = self.create_publisher(String, "/sim/people", 5)
        self.guest_present = False
        self.guest_idx = 0
        self.guest_d = float(gp("guest_distance_m"))
        faces = [f for f in gp("faces") if f] or sorted(glob.glob(os.path.expanduser("~/sim_faces/*.jpg"))) or DEFAULT_FACES
        self.faces = [im for im in (cv2.imread(f) for f in faces) if im is not None]
        self.get_logger().info(f"Rostros simulados: {len(self.faces)}")
        calib = {}
        try:
            with open(ARM_CONFIG, encoding="utf-8") as f:
                calib = (yaml.safe_load(f) or {}).get("wrist_camera_calibration") or {}
        except OSError:
            pass
        self.wfx = float(calib.get("focal_px", 485.0))
        self.wcx, self.wcy = float(calib.get("cx_px", 320.0)), float(calib.get("cy_px", 240.0))
        self.gz = bool(gp("gazebo"))
        self.map = None if self.gz else self._load_map(gp("map_yaml"))

        # --- Publicadores (mismos nombres que el robot real) ---
        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_arm_js = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.pub_pt_js = self.create_publisher(JointState, "/pan_tilt/joint_states", 10)
        self.pub_arm_status = self.create_publisher(String, "/arm/status", 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pt_conn = self.create_publisher(Bool, "/pan_tilt/connected", latched)
        self.pub_voice = self.create_publisher(String, "/voice/transcription", 10)
        self.pub_vresult = self.create_publisher(String, "/voice/result", 20)
        self.pub_vstatus = self.create_publisher(String, "/voice/status", 10)
        self.pub_level = self.create_publisher(Float32, "/voice/mic_level", 10)
        self.pub_rgb = self.create_publisher(Image, "/camera/camera/color/image_raw", qos_profile_sensor_data)
        self.pub_depth = self.create_publisher(Image, "/camera/camera/aligned_depth_to_color/image_raw", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/camera/color/camera_info", qos_profile_sensor_data)
        self.pub_wrist = self.create_publisher(Image, "/arm_camera/image_raw", qos_profile_sensor_data)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.pub_amcl = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", latched)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.pub_map = self.create_publisher(OccupancyGrid, "/map", latched)
        self.pub_world = self.create_publisher(MarkerArray, "/sim/world", 5)
        self.pub_pt_conn.publish(Bool(data=True))

        # --- Suscripciones / servicios ---
        sub = lambda t, n, f, q=10: self.create_subscription(t, n, f, q, callback_group=self.cb)
        sub(JointState, "/arm/joint_cmd", self._joint_cmd, 5)
        sub(JointTrajectory, "/arm/joint_trajectory", self._trajectory, 2)
        sub(Float32, "/arm/gripper_cmd", lambda m: self.arm.gripper(m.data), 5)
        sub(Bool, "/arm/enable", lambda m: self.arm.set_enable(m.data), 5)
        sub(Vector3, "/pan_tilt/target_degrees", lambda m: setattr(self.pt, "target", (m.x, m.y)))
        sub(JointState, "/pan_tilt/command", self._pt_cmd)
        sub(Bool, "/voice/mute", lambda m: setattr(self.mic, "muted", bool(m.data)))
        sub(String, "/voice/initial_prompt", lambda m: setattr(self, "prompt", m.data))
        sub(String, "/sim/speech", self._speech)
        sub(String, "/sim/guest", self._guest)
        sub(Bool, "/sim/auto_guest", lambda m: setattr(self, "auto_guest", bool(m.data)))
        sub(PoseStamped, "/goal_pose", self._goal_pose)  # "2D Goal Pose" de RViz, como bt_navigator
        sub(Twist, "/cmd_vel", lambda m: setattr(self, "cmd_vel", (m.linear.x, m.angular.z, time.time())))
        sub(Float64, "/torso/goal", self._torso_goal)
        sub(String, "/hri/say_something", self._heard_robot)
        srv = lambda t, n, f: self.create_service(t, n, f, callback_group=self.cb)
        srv(SetBool, "/arm/set_enable", self._srv_enable)
        srv(SetBool, "/pan_tilt/set_torque", self._srv_torque)
        srv(Trigger, "/voice/listen_trigger", self._srv_listen)
        sub(String, "/voice/listen_request", self._listen_request)
        sub(String, "/voice/cancel", lambda m: self._voice_cancelled(self.mic.cancel(m.data.strip()), "cancel"))
        srv(Trigger, "/arm/home", self._srv_home)
        if FollowJointTrajectory is not None:  # misma interfaz que piper_arm_controller (HOME-45)
            ActionServer(self, FollowJointTrajectory, "/arm/follow_joint_trajectory", execute_callback=self._fjt_execute,
                         goal_callback=lambda g: GoalResponse.ACCEPT if self.arm.enabled else GoalResponse.REJECT,
                         cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=self.cb)
        ActionServer(self, NavigateToPose, "navigate_to_pose", execute_callback=self._nav_execute,
                     goal_callback=lambda g: GoalResponse.ACCEPT, cancel_callback=lambda g: CancelResponse.ACCEPT,
                     callback_group=self.cb)

        self.tf = TransformBroadcaster(self)
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = "map", "odom"
        st.transform.rotation.w = 1.0
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform(st)
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.last = time.time()
        self.create_timer(0.02, self._tick)
        if self.gz:
            self._setup_gazebo()
        else:
            self.create_timer(1.0 / (min(float(gp("camera_rate_hz")), 5.0) if self.bag_map is not None else float(gp("camera_rate_hz"))),
                              self._cameras)       # dibujar la escena es caro: no dejar sin tiempo a la base y al elevador
            self.create_timer(0.1, self._scan)
        self.create_timer(0.5, self._slow)
        self.create_timer(0.05, self._fast)
        self.create_timer(0.05, self._arm_status)
        if self.map is not None:
            self.pub_map.publish(self.map["msg"])
        self.get_logger().info("Robot simulado listo (Piper, pan-tilt, voz, camaras, Nav2 falso).")

    # ------------------------------------------------------------------ mundo
    @staticmethod
    def _waypoints():
        wps = {}
        for f in ("semantic_map.yaml", "waypoints.yaml"):
            try:
                with open(os.path.join(WS, "src/base_athome_navigation/config", f), encoding="utf-8") as fh:
                    for k, v in ((yaml.safe_load(fh) or {}).get("waypoints") or {}).items():
                        if isinstance(v, dict) and "x" in v:
                            wps.setdefault(k, v)
            except OSError:
                pass
        return wps

    def _load_map(self, path):
        try:
            with open(path, encoding="utf-8") as f:
                info = yaml.safe_load(f)
            img_path = info["image"] if os.path.isabs(info["image"]) else os.path.join(os.path.dirname(path), info["image"])
            img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
            res, origin = float(info["resolution"]), info["origin"]
            occ_th = float(info.get("occupied_thresh", 0.65))
            occ = (255 - img.astype(np.float32)) / 255.0 > occ_th   # negate=0
            occ = np.flipud(occ)                                     # fila 0 = y minima
            msg = OccupancyGrid()
            msg.header.frame_id = "map"
            msg.info.resolution = res
            msg.info.width, msg.info.height = img.shape[1], img.shape[0]
            msg.info.origin.position.x, msg.info.origin.position.y = float(origin[0]), float(origin[1])
            msg.info.origin.orientation.w = 1.0
            free = (255 - np.flipud(img).astype(np.float32)) / 255.0 < float(info.get("free_thresh", 0.25))
            grid = np.full(occ.shape, -1, np.int8)
            grid[free], grid[occ] = 0, 100
            msg.data = grid.flatten().tolist()
            self.get_logger().info(f"Mapa cargado: {path}")
            return {"occ": occ, "res": res, "ox": float(origin[0]), "oy": float(origin[1]), "msg": msg}
        except Exception as e:  # sin mapa: /scan vacio, todo lo demas funciona
            self.get_logger().warn(f"Sin mapa para /scan ({e})")
            return None

    def _guest(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ("door", "present", "si", "yes"):
            self.guest_present = True
        elif cmd in ("none", "absent", "no", "seated", "sentado"):
            self.guest_present = False
        elif cmd == "next":
            self.guest_idx += 1
            self.guest_present = True
        self.get_logger().info(f"Invitado #{self.guest_idx + 1}: {'presente' if self.guest_present else 'ausente'}")

    def _goal_pose(self, msg):
        q = msg.pose.orientation
        self.base.set_goal(msg.pose.position.x, msg.pose.position.y, 2 * math.atan2(q.z, q.w))

    def _speech(self, msg):
        ok = self.mic.hear(msg.data)
        self.get_logger().info(f"Invitado dice: '{msg.data}' -> {'ESCUCHADO' if ok else 'PERDIDO (el robot no estaba escuchando)'}")

    # ------------------------------------------------------------------ callbacks reales
    def _joint_cmd(self, msg):
        if len(msg.position) < 6:
            return
        is_rad = all(abs(v) <= 3.5 for v in msg.position[:6])
        tgt = [v / D2R if is_rad else v for v in msg.position[:6]]
        dur = max(0.6, min(10.0, msg.velocity[0])) if len(msg.velocity) and msg.velocity[0] > 0 else 2.5
        with self.lock:
            self.arm.joint_cmd(tgt, dur)
            if len(msg.position) >= 7:
                g = msg.position[6]
                self.arm.gripper(g * 1000.0 if g < 1.0 else g)

    def _trajectory(self, msg):
        try:
            idx = [msg.joint_names.index(f"joint{i}") for i in range(1, 7)]
        except ValueError:
            return
        times, pts = [], []
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if times and t <= times[-1]:
                continue
            times.append(t)
            pts.append([pt.positions[i] / D2R for i in idx])
        if pts:
            with self.lock:
                self.arm.trajectory(times, pts)

    def _fjt_execute(self, goal_handle):
        R = FollowJointTrajectory.Result
        res = R()
        msg = goal_handle.request.trajectory
        try:
            idx = [list(msg.joint_names).index(f"joint{i}") for i in range(1, 7)]
        except ValueError:
            res.error_code, res.error_string = R.INVALID_JOINTS, "faltan joint1..joint6"
            goal_handle.abort()
            return res
        times, pts = [], []
        for pt in msg.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            if times and t <= times[-1]:
                continue
            times.append(t)
            pts.append([pt.positions[i] / D2R for i in idx])
        if not pts:
            res.error_code, res.error_string = R.INVALID_GOAL, "sin puntos"
            goal_handle.abort()
            return res
        with self.lock:
            self.arm.trajectory(times, pts)
        t_end = time.time() + times[-1] + 2.0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                with self.lock:
                    self.arm.joint_cmd(list(self.arm.q), 0.6)
                goal_handle.canceled()
                return res
            if not self.arm.enabled:
                res.error_code, res.error_string = R.PATH_TOLERANCE_VIOLATED, "sim: brazo deshabilitado"
                goal_handle.abort()
                return res
            with self.lock:
                moving = self.arm.moving
            if not moving:
                dev = max(abs(a - b) for a, b in zip(self.arm.q, pts[-1]))
                if dev <= 2.0:
                    res.error_code, res.error_string = R.SUCCESSFUL, f"sim: llegada verificada ({dev:.2f} grados)"
                    goal_handle.succeed()
                else:
                    res.error_code, res.error_string = R.GOAL_TOLERANCE_VIOLATED, f"sim: desvio {dev:.2f} grados"
                    goal_handle.abort()
                return res
            if time.time() > t_end:
                res.error_code, res.error_string = R.GOAL_TOLERANCE_VIOLATED, "sim: timeout"
                goal_handle.abort()
                return res
            time.sleep(0.05)
        return res

    def _pt_cmd(self, msg):
        pan, tilt = self.pt.target
        for n, v in zip(msg.name, msg.position):
            if "pan" in n:
                pan = v / D2R
            elif "tilt" in n:
                tilt = v / D2R
        self.pt.target = (pan, tilt)

    def _srv_enable(self, req, res):
        self.arm.set_enable(req.data)
        res.success, res.message = True, f"sim: brazo {'habilitado' if req.data else 'libre'}"
        return res

    def _srv_torque(self, req, res):
        self.pt.torque = bool(req.data)
        res.success, res.message = True, f"sim: torque pan-tilt {req.data}"
        return res

    def _srv_home(self, req, res):
        self.arm.joint_cmd(REST_DEG, 3.0)
        res.success, res.message = True, "sim: home"
        return res

    def _voice_event(self, rid, event, **kw):
        d = {"id": rid, "event": event, "t": round(time.time(), 3), "sim": True}
        d.update(kw)
        self.pub_vresult.publish(String(data=json.dumps(d)))

    def _voice_cancelled(self, rid, why):
        if rid:
            self._voice_event(rid, "result", status="cancelled", text="", reason=why)

    def _open_listen(self, rid, prompt="", start_timeout=None, end_silence=None):
        self._voice_cancelled(self.mic.request(rid, start_timeout, end_silence), "replaced")
        self._voice_event(rid, "listening")
        self.get_logger().info(f"Escucha {rid} abierta (espera {start_timeout or self.mic.start_timeout:.0f} s; "
                               f"prompt: '{(prompt or self.prompt)[:50]}')")
        if self.auto_guest:
            line = AUTO_LINES[self.auto_idx % len(AUTO_LINES)]
            self.auto_idx += 1
            threading.Timer(1.0, lambda: self._speech(String(data=line))).start()

    def _listen_request(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            d = {}
        self.listen_count += 1
        self._open_listen(str(d.get("id") or f"r{self.listen_count}"), d.get("prompt", ""),
                          d.get("start_timeout"), d.get("end_silence"))

    def _srv_listen(self, req, res):
        self.listen_count += 1
        rid = f"t{self.listen_count}"
        self._open_listen(rid)
        res.success, res.message = True, json.dumps({"id": rid})
        return res

    async def _nav_execute(self, goal_handle):
        p = goal_handle.request.pose.pose
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        self.base.set_goal(p.position.x, p.position.y, yaw)
        mine = self.base.goal  # identidad del objetivo de ESTE goal (un goal nuevo lo reemplaza)
        self.get_logger().info(f"Nav2 (sim): ir a ({p.position.x:.2f}, {p.position.y:.2f})")
        fb = NavigateToPose.Feedback()
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                if self.base.goal is mine:  # no pisar el objetivo de un goal posterior
                    self.base.goal = None
                goal_handle.canceled()
                return NavigateToPose.Result()
            if self.base.goal is not mine and self.base.goal is not None:
                goal_handle.abort()  # reemplazado por otro goal (como el preempt de Nav2)
                return NavigateToPose.Result()
            if self.base.goal is None:
                break
            fb.distance_remaining = float(math.hypot(p.position.x - self.base.x, p.position.y - self.base.y))
            fb.current_pose.header.frame_id = "map"
            fb.current_pose.pose.position.x, fb.current_pose.pose.position.y = self.base.x, self.base.y
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        goal_handle.succeed()
        return NavigateToPose.Result()

    # ------------------------------------------------------------------ bucles
    def _tick(self):
        now = time.time()
        dt, self.last = now - self.last, now
        with self.lock:
            self.arm.update(now)
        self.pt.update(dt)
        step = 0.10 * dt       # elevador simulado, 10 cm/s
        self.torso_pos += max(-step, min(step, self.torso_goal - self.torso_pos))
        if self.gz:
            self._gz_tick(now)
        elif self.base.goal is not None:
            self.base.update(dt)
        elif now - self.cmd_vel[2] < 0.5:  # teleop directo (sin objetivo de Nav2)
            v, w = self.cmd_vel[0], self.cmd_vel[1]
            self.base.yaw += w * dt
            self.base.x += v * dt * math.cos(self.base.yaw)
            self.base.y += v * dt * math.sin(self.base.yaw)
        stamp = self.get_clock().now().to_msg()
        q = [v * D2R for v in self.arm.q]
        half = min(0.035, max(0.0, self.arm.gripper_mm / 2000.0))
        arm = JointState(name=[f"joint{i}" for i in range(1, 9)], position=q + [half, -half])
        arm.header.stamp = stamp
        pt = JointState(name=["pan_joint", "tilt_joint"], position=[self.pt.pan * D2R, self.pt.tilt * D2R])
        pt.header.stamp = stamp
        full = JointState(name=arm.name + pt.name + ["torso_lift_joint"], position=list(arm.position) + list(pt.position) + [self.torso_pos])
        full.header.stamp = stamp
        self.pub_js.publish(full)
        self.pub_arm_js.publish(arm)
        self.pub_pt_js.publish(pt)
        if self.gz:
            for j, v in zip(full.name, full.position):
                self.gz_joint_pubs[j].publish(Float64(data=float(v)))
        t = TransformStamped()
        t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, "odom", "base_footprint"
        t.transform.translation.x, t.transform.translation.y = self.base.x, self.base.y
        _, _, t.transform.rotation.z, t.transform.rotation.w = yaw_quat(self.base.yaw)
        self.tf.sendTransform(t)
        for rid, status, text in self.mic.due(now):
            self._voice_event(rid, "result", status=status, text=text,
                              reason="" if status == "ok" else "start_timeout")
            if status == "ok":
                self.pub_voice.publish(String(data=text))
            self.get_logger().info(f"/voice/result [{rid}] {status} '{text}'")

    # ------------------------------------------------------------------ Gazebo
    def _setup_gazebo(self):
        """gz sim aporta fisica de la base, lidar y camaras; este nodo sigue siendo el 'firmware'
        (Piper, pan-tilt, voz, Nav2 falso) y reenvia los sensores con los nombres del robot real."""
        self.gz_joint_pubs = {j: self.create_publisher(Float64, gz_model.joint_cmd_topic(j), 10)
                              for j in gz_model.POSITION_JOINTS}
        self.gz_cmd = self.create_publisher(Twist, f"/model/{gz_model.MODEL}/cmd_vel", 10)
        sub = lambda t, n, f: self.create_subscription(t, n, f, qos_profile_sensor_data, callback_group=self.cb)
        sub(Odometry, f"/model/{gz_model.MODEL}/odometry", self._gz_odom)
        sub(LaserScan, "/sim/gz/scan", self._gz_scan)
        sub(Image, "/sim/gz/head/image", self._gz_head)
        sub(Image, "/sim/gz/head/depth_image", self._gz_depth)
        sub(CameraInfo, "/sim/gz/head/camera_info", self._gz_info)
        sub(Image, "/sim/gz/wrist", self._gz_wrist)
        self.get_logger().info("Modo Gazebo: base/lidar/camaras desde gz sim")

    def _gz_tick(self, now):
        v = w = 0.0
        g = self.base.goal
        if g is not None:
            dx, dy = g[0] - self.base.x, g[1] - self.base.y
            d = math.hypot(dx, dy)
            wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
            if d > 0.08:
                err = wrap(math.atan2(dy, dx) - self.base.yaw)
                w = max(-1.0, min(1.0, 2.0 * err))
                v = min(self.base.speed, 0.8 * d) if abs(err) < 0.5 else 0.0
            else:
                err = wrap(g[2] - self.base.yaw)
                w = max(-1.0, min(1.0, 2.0 * err))
                if abs(err) < 0.05:
                    self.base.goal, w = None, 0.0
        elif now - self.cmd_vel[2] < 0.5:
            v, w = self.cmd_vel[0], self.cmd_vel[1]
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = float(v), float(w)
        self.gz_cmd.publish(cmd)

    def _gz_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.base.x, self.base.y = p.x, p.y
        self.base.yaw = 2 * math.atan2(q.z, q.w)
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = "odom", "base_footprint"
        self.pub_odom.publish(m)

    def _restamp(self, m, frame):
        m.header.stamp, m.header.frame_id = self.get_clock().now().to_msg(), frame
        return m

    def _gz_scan(self, m):
        self.pub_scan.publish(self._restamp(m, "laser"))

    def _gz_info(self, m):
        self.pub_info.publish(self._restamp(m, "camera_color_optical_frame"))

    def _gz_wrist(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else img
        self.pub_wrist.publish(to_image(np.ascontiguousarray(bgr), "bgr8", self.get_clock().now().to_msg(), "arm_camera_optical_frame"))

    def _gz_head(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if m.encoding == "rgb8" else img.copy()
        self._overlay_guest(bgr, None)
        self.pub_rgb.publish(to_image(bgr, "bgr8", self.get_clock().now().to_msg(), "camera_color_optical_frame"))

    def _gz_depth(self, m):
        d = np.frombuffer(m.data, np.float32).reshape(m.height, m.width)
        mm = np.nan_to_num(d * 1000.0, nan=0.0, posinf=0.0, neginf=0.0).clip(0, 65535).astype(np.uint16)
        self._overlay_guest(None, mm)
        self.pub_depth.publish(to_image(mm, "16UC1", self.get_clock().now().to_msg(), "camera_color_optical_frame"))

    def _arm_status(self):
        # 20 Hz como el controlador real (50 Hz): a 2 Hz los consumidores lo veian "viejo" (teleop sin feedback)
        self.pub_arm_status.publish(String(data=json.dumps({
            "is_moving": self.arm.moving, "enabled": self.arm.enabled, "feedback_ok": True,
            "gripper_mm": round(self.arm.gripper_mm, 2), "xyz_mm": [0.0, 0.0, 0.0],
            "joints_deg": [round(v, 2) for v in self.arm.q], "sim": True})))

    def _slow(self):
        stamp = self.get_clock().now().to_msg()
        st = ("LISTENING" if self.mic.active else ("PROCESSING" if self.mic.pending else
                                                   ("MUTED" if self.mic.muted else "IDLE")))
        self.pub_vstatus.publish(String(data=st))
        self.pub_level.publish(Float32(data=0.02 if st == "LISTENING" else 0.0))
        a = PoseWithCovarianceStamped()
        a.header.stamp, a.header.frame_id = stamp, "map"
        a.pose.pose.position.x, a.pose.pose.position.y = self.base.x, self.base.y
        _, _, a.pose.pose.orientation.z, a.pose.pose.orientation.w = yaw_quat(self.base.yaw)
        self.pub_amcl.publish(a)
        self._update_object(stamp)
        self._publish_scene(stamp)

    def _fast(self):
        """Odometria y elevador a 20 Hz: el tramo final hasta la bolsa se controla con la odometria."""
        stamp = self.get_clock().now().to_msg()
        if not self.gz:
            o = Odometry()
            o.header.stamp, o.header.frame_id, o.child_frame_id = stamp, "odom", "base_footprint"
            o.pose.pose.position.x, o.pose.pose.position.y = self.base.x, self.base.y
            _, _, o.pose.pose.orientation.z, o.pose.pose.orientation.w = yaw_quat(self.base.yaw)
            self.pub_odom.publish(o)
        js = JointState(name=["torso_lift_joint"], position=[self.torso_pos])
        js.header.stamp = stamp
        self.pub_torso_js.publish(js)
        moving = abs(self.torso_goal - self.torso_pos) > 1e-3
        self.pub_torso_st.publish(String(data=json.dumps(dict(connected=True, fresh=True, calibrated=True,
                                                              position_m=round(self.torso_pos, 4), moving=moving))))

    def _torso_goal(self, msg):
        if math.isnan(msg.data):
            self.torso_goal = self.torso_pos
        else:
            self.torso_goal = max(-0.59, min(0.0, float(msg.data)))
            self.get_logger().info(f"Elevador (sim) hacia {self.torso_goal:+.2f} m")

    def _heard_robot(self, msg):
        """La persona simulada obedece: senala cuando el robot se lo pide."""
        if "point" in msg.data.lower() and self.person_points:
            self.person_pointing = True

    def _person(self):
        """La persona ahora: mira al robot y, si corresponde, apunta a la bolsa con el error de una persona real."""
        if self.bag_map is None:
            return None
        xy = list(self.person_xy)
        if self.person_sway:
            xy[1] += self.person_sway * math.sin(2.0 * math.pi * time.time() / 6.0)
        target = None
        if self.person_pointing and not self.obj_attached and not self.bag_delivered:
            target = (self.bag_map[0] + self.point_err, self.bag_map[1] - self.point_err)
        return sim_scene.make_person(xy, (self.base.x, self.base.y), target)

    def _world_faces(self, person):
        faces = sim_scene.person_faces(person) if person else []
        if self.bag_map is not None:
            if self.obj_attached:
                c, s = math.cos(self.base.yaw), math.sin(self.base.yaw)
                xy = (self.base.x + c * self.obj[0] - s * self.obj[1], self.base.y + s * self.obj[0] + c * self.obj[1])
                faces += sim_scene.bag_faces(xy, self.obj[2], center_z=self.obj[2] - 0.15)
            else:
                faces += sim_scene.bag_faces(self.bag_map[:2], self.bag_map[2])
        if self.bag2 is not None:
            faces += sim_scene.bag_faces(self.bag2[:2], self.bag2[2])
        return faces

    def _publish_scene(self, stamp):
        """Persona, gesto y punto de basura, para verlos en RViz."""
        if self.bag_map is None:
            return
        marks = []

        def mark(mid, kind, frame="map"):
            m = Marker()
            m.header.stamp, m.header.frame_id = stamp, frame
            m.ns, m.id, m.type, m.action = "sim_bag_scene", mid, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.01   # RViz marca como error una escala 0, aun al borrar
            marks.append(m)
            return m
        px, py = self._person()["xy"]
        body = mark(1, Marker.CYLINDER)
        body.pose.position.x, body.pose.position.y, body.pose.position.z = px, py, 0.8
        body.scale.x, body.scale.y, body.scale.z = 0.35, 0.35, 1.6
        body.color.r, body.color.g, body.color.b, body.color.a = 0.2, 0.5, 0.9, 0.35
        person = self._person()
        if self.person_pointing and not self.obj_attached and not self.bag_delivered:
            shoulder = person["keypoints"][6]
            ray = mark(2, Marker.LINE_STRIP)
            ray.scale.x = 0.03
            ray.color.r, ray.color.g, ray.color.a = 1.0, 0.9, 1.0
            ray.points = [Point(x=shoulder[0], y=shoulder[1], z=shoulder[2]),
                          Point(x=self.bag_map[0] + self.point_err, y=self.bag_map[1] - self.point_err, z=0.0)]
        else:
            gone = mark(2, Marker.LINE_STRIP)
            gone.action = Marker.DELETE
        for waypoint, wid in (("trash_drop", 3),):
            wp = self._waypoints().get(waypoint)
            if wp:
                t = mark(wid, Marker.CUBE)
                t.pose.position.x, t.pose.position.y, t.pose.position.z = float(wp["x"]) + 0.4, float(wp["y"]), 0.02
                t.scale.x, t.scale.y, t.scale.z = 0.8, 0.8, 0.04
                t.color.g, t.color.r, t.color.a = 0.8, 0.2, 0.6
                label = mark(wid + 1, Marker.TEXT_VIEW_FACING)
                label.pose.position.x, label.pose.position.y, label.pose.position.z = float(wp["x"]), float(wp["y"]), 0.5
                label.scale.z = 0.25
                label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
                label.text = "BASURA"
        self.pub_world.publish(MarkerArray(markers=marks))

    def _lookup(self, target, source):
        try:
            tr = self.tf_buf.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None
        t, r = tr.transform.translation, tr.transform.rotation
        x, y, z, w = r.x, r.y, r.z, r.w
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return R, np.array([t.x, t.y, t.z])

    def _update_object(self, stamp):
        """Agarre simulado: si la pinza se cierra con el TCP a <6 cm del objeto, el objeto la sigue."""
        tcp = self._lookup("base_footprint", "tcp_link")
        if tcp is not None:
            # Una bolsa se toma por el nudo, debajo de su parte mas alta, y lo que queda entre los dedos es fino
            target = np.array(self.obj) - ([0.0, 0.0, self.BAG_KNOT_BELOW_TOP_M] if self.bag_map is not None else 0.0)
            d = float(np.linalg.norm(tcp[1] - target))
            dia_mm = self.BAG_KNOT_MM if self.bag_map is not None else 1000.0 * float(self.get_parameter("object_diameter_m").value)
            # Objeto entre los dedos (TCP a < 6 cm): la pinza no puede cerrar por debajo de su diametro
            self.arm.grip_block_mm = dia_mm if (d < 0.06 or self.obj_attached) else None
            closing = self.arm.gripper_mm <= dia_mm + 2.0
            if closing and not self.gripper_was_closed and self.bag_map is not None:     # la verdad, para medir el error
                self.get_logger().info(f"Pinza cerrada: TCP {np.round(tcp[1], 3).tolist()}, objeto "
                                       f"{np.round(self.obj, 3).tolist()}, a {d * 100:.1f} cm")
            self.gripper_was_closed = closing
            if not self.obj_attached and self.arm.gripper_mm <= dia_mm + 2.0 and d < 0.06:
                self.obj_attached = True
                self.get_logger().info("Objeto AGARRADO (sim)")
            elif self.obj_attached and self.arm.gripper_mm > dia_mm + 10.0:
                self.obj_attached = False
                self.get_logger().info("Objeto SOLTADO (sim)")
                if self.bag_map is not None:    # la bolsa cae al piso donde se solto, parada sobre su base
                    c, s = math.cos(self.base.yaw), math.sin(self.base.yaw)
                    self.bag_map = [self.base.x + c * self.obj[0] - s * self.obj[1],
                                    self.base.y + s * self.obj[0] + c * self.obj[1], 0.30]
                    self.bag_delivered = True
                    self.get_logger().info(f"Bolsa en el piso (sim): ({self.bag_map[0]:.2f}, {self.bag_map[1]:.2f})")
            if self.obj_attached:
                self.obj = (tcp[1] + ([0.0, 0.0, self.BAG_KNOT_BELOW_TOP_M] if self.bag_map is not None else 0.0)).tolist()
        if self.bag_map is not None and not self.obj_attached:
            c, s = math.cos(self.base.yaw), math.sin(self.base.yaw)
            dx, dy = self.bag_map[0] - self.base.x, self.bag_map[1] - self.base.y
            self.obj = [c * dx + s * dy, -s * dx + c * dy, self.bag_map[2]]
        m = Marker()
        m.header.stamp, m.header.frame_id = stamp, "base_footprint"
        m.ns, m.id, m.type, m.action = "sim_object", 0, Marker.CYLINDER, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = self.obj
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = float(self.get_parameter("object_diameter_m").value)
        m.scale.z = 0.23
        m.color.r, m.color.g, m.color.b, m.color.a = 0.85, 0.1, 0.85, 1.0
        if self.bag_map is not None:     # bolsa: bulto debajo de la manija, que es el punto que se agarra
            m.type = Marker.CUBE
            m.pose.position.z = self.obj[2] - 0.15
            m.scale.x, m.scale.y, m.scale.z = 0.28, 0.28, 0.30
            m.color.r, m.color.g, m.color.b = 0.05, 0.05, 0.05
        self.pub_world.publish(MarkerArray(markers=[m]))

    def _overlay_guest(self, rgb, depth, W=640, H=480, f=615.0):
        """Dibuja al invitado (rostro real de una imagen + cuerpo) donde lo veria la RealSense segun
        el pan-tilt. Se usa igual sobre la imagen sintetica o sobre el render de Gazebo."""
        if not (self.guest_present and self.faces):
            return
        face = self.faces[self.guest_idx % len(self.faces)]
        u = W / 2 + f * math.tan(self.pt.pan * D2R)           # pan + izquierda: el invitado se ve a la derecha
        v = H / 2 + f * math.tan((self.pt.tilt - 5.0) * D2R)  # rostro ~5 grados arriba del eje
        size = int(f * 0.45 / self.guest_d)                    # el rostro ocupa ~1/3 de la foto: cara real ~15 cm
        if not (20 < size < H and -size < u < W + size):
            return
        fi = cv2.resize(face, (size, int(size * face.shape[0] / face.shape[1])))
        x0, y0 = int(u - fi.shape[1] / 2), int(v - fi.shape[0] / 2)
        xa, ya, xb, yb = max(0, x0), max(0, y0), min(W, x0 + fi.shape[1]), min(H, y0 + fi.shape[0])
        if xb <= xa or yb <= ya:
            return
        bx0, bx1 = max(0, xa - size // 4), min(W, xb + size // 4)
        if rgb is not None:
            rgb[ya:yb, xa:xb] = fi[ya - y0:yb - y0, xa - x0:xb - x0]
            rgb[yb:H, bx0:bx1] = (90, 60, 40)
        if depth is not None:
            depth[ya:H, bx0:bx1] = int(self.guest_d * 1000)

    def _cameras(self):
        stamp = self.get_clock().now().to_msg()
        # --- RealSense de la cabeza (640x480, fx ~615) ---
        W, H, f = 640, 480, 615.0
        person = self._person()
        head = self._lookup("camera_color_optical_frame", "map") if self.bag_map is not None else None
        if head is not None:     # la escena sale de la geometria: la percepcion real corre en lazo cerrado
            self.pub_people.publish(String(data=json.dumps(dict(people=[person] if person else []))))
            rgb, depth = sim_scene.render(head[0], head[1], (f, f, W / 2, H / 2), W, H, self._world_faces(person))
            if self.head_pix_err:
                noise = self.rng.normal(0.0, self.head_pix_err, depth.shape)
                depth = np.where(depth > 0, np.clip(depth + noise, 1, 65535), 0).astype(np.uint16)
        else:
            rgb = np.full((H, W, 3), (70, 75, 80), np.uint8)
            rgb[int(H * 0.62):] = (60, 90, 120)  # piso
            depth = np.full((H, W), 3000, np.uint16)
        self._overlay_guest(rgb, depth)
        self.pub_rgb.publish(to_image(rgb, "bgr8", stamp, "camera_color_optical_frame"))
        self.pub_depth.publish(to_image(depth, "16UC1", stamp, "camera_color_optical_frame"))
        info = CameraInfo(width=W, height=H, distortion_model="plumb_bob")
        info.header.stamp, info.header.frame_id = stamp, "camera_color_optical_frame"
        info.k = [f, 0.0, W / 2, 0.0, f, H / 2, 0.0, 0.0, 1.0]
        info.p = [f, 0.0, W / 2, 0.0, 0.0, f, H / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = [0.0] * 5
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.pub_info.publish(info)
        # --- Camara de muneca: lata magenta proyectada con la pose real del brazo (TF) ---
        wimg = np.full((480, 640, 3), (150, 160, 165), np.uint8)
        cam = self._lookup("arm_camera_optical_frame", "base_footprint")
        wmap = self._lookup("arm_camera_optical_frame", "map") if self.bag_map is not None else None
        if wmap is not None:
            Rcw, tcw = wmap
            if self.wrist_tilt_err:   # el montaje real no es el del modelo: la imagen sale de una camara corrida
                a = self.wrist_tilt_err
                Rcw = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]]) @ Rcw
            K = (self.wfx, self.wfx, self.wcx, self.wcy)
            wimg, _ = sim_scene.render(Rcw, tcw, K, 640, 480, self._world_faces(person))
            for poly in (((0, 480), (0, 395), (105, 362), (120, 480)), ((640, 480), (640, 395), (535, 362), (520, 480))):
                cv2.fillConvexPoly(wimg, np.array(poly, np.int32), (25, 25, 25))     # dedos de la garra, como en el robot real
            if self.wrist_ghost:      # falso positivo real: una caja sobre la garra (imagen del robot, 2026-11-05)
                cv2.rectangle(wimg, (30, 400), (90, 470), sim_scene.BAG_BGR, -1)
        elif cam is not None and not self.obj_attached:
            pc = cam[0] @ np.array(self.obj) + cam[1]
            if pc[2] > 0.05:
                u, v = self.wcx + self.wfx * pc[0] / pc[2], self.wcy + self.wfx * pc[1] / pc[2]
                hw, hh = self.wfx * 0.075 / pc[2] / 2, self.wfx * 0.23 / pc[2] / 2
                cv2.rectangle(wimg, (int(u - hw), int(v - hh)), (int(u + hw), int(v + hh)), (200, 30, 200), -1)
        self.pub_wrist.publish(to_image(wimg, "bgr8", stamp, "arm_camera_optical_frame"))

    def _scan(self):
        s = LaserScan()
        s.header.stamp, s.header.frame_id = self.get_clock().now().to_msg(), "laser"
        s.angle_min, s.angle_max, s.angle_increment = -2.356, 2.356, 0.5 * D2R
        s.range_min, s.range_max, s.scan_time = 0.06, 5.0, 0.1
        n = int(round((s.angle_max - s.angle_min) / s.angle_increment)) + 1
        if self.map is None:
            s.ranges = [float("inf")] * n
        else:
            mp = self.map
            ang = self.base.yaw + s.angle_min + np.arange(n) * s.angle_increment
            rs = np.arange(0.06, 5.0, mp["res"] / 2)
            xs = self.base.x + np.outer(np.cos(ang), rs)
            ys = self.base.y + np.outer(np.sin(ang), rs)
            ci = ((xs - mp["ox"]) / mp["res"]).astype(int)
            ri = ((ys - mp["oy"]) / mp["res"]).astype(int)
            h, w = mp["occ"].shape
            inside = (ci >= 0) & (ci < w) & (ri >= 0) & (ri < h)
            hit = np.zeros_like(inside)
            hit[inside] = mp["occ"][ri[inside], ci[inside]]
            first = np.where(hit.any(axis=1), hit.argmax(axis=1), -1)
            s.ranges = [float(rs[k]) if k >= 0 else float("inf") for k in first]
        self.pub_scan.publish(s)


def main():
    rclpy.init()
    node = SimRobot()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
