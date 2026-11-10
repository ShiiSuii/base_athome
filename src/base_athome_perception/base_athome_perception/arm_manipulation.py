"""Nucleo de la manipulacion Base@Home (usado por scripts/manipulacion_simple.py y por el
secuenciador de misiones). Tambien se puede usar sin ventana (headless).

Manipulacion Base@Home (version simple, 2026-10-30).

Una sola ventana: video de la camara de MUNECA (no la de la cabeza) + panel de texto.

  [4][5][6]  ensenar SOLO HOME / PRE-GRASP / POST-GRASP (brazo libre, [ENTER] guarda y bloquea)
  [T]        ensenar poses: el brazo queda LIBRE; llevalo a HOME y [ENTER], luego a
             PRE-GRASP y [ENTER], luego a POST-GRASP y [ENTER]. Al final se bloquea.
  [ESC]      cancelar la ensenanza (bloquea el brazo) / salir si no hay ensenanza
  [1][2][3]  ir a HOME / PRE-GRASP / POST-GRASP guardadas
  [O] [C]    abrir / cerrar pinza
  [A]        alinear la pinza con el objeto (opcional, manual)
  [G]        agarrar con cinematica inversa (MoveIt arranca solo con este script)
  [M]        calibrar distancia con la cinta (Medido_cm = centro de la pinza -> cara del objeto)
  Sliders:   Avance_max_cm (tope del avance hacia el objeto), Ajuste_cm (-10..+10 sobre el avance),
             Velocidad_% (5..100 de la velocidad maxima; avance final y subida a la mitad). [K] guarda.
  RViz:      MarkerArray /arm_camera/object_markers (y /manipulacion/objeto), frame base_link:
             cilindro verde = objeto, esfera/flecha naranja = donde quedara el centro de la pinza
  [Q]        salir

El brazo lo maneja piper_arm_controller (Jetson): /arm/status (medido), /arm/set_enable,
/arm/joint_cmd, /arm/gripper_cmd. Las poses se guardan en config/arm_poses_and_limits.yaml
(pose_store: valida limites y guarda la anterior como <pose>_previous).
"""
import json
import math
import os
import signal
import subprocess
import threading
import time
from typing import List, Optional

os.environ.setdefault("YOLO_OFFLINE", "1")  # ultralytics: nunca auto-instalar paquetes

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import ColorRGBA, Float32, String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_srvs.srv import SetBool
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
try:  # HOME-45: ejecucion con resultado verificado por el controlador (real o simulado)
    from control_msgs.action import FollowJointTrajectory
except ImportError:  # pragma: no cover
    FollowJointTrajectory = None

from base_athome_perception import bag_vision, pose_store
from base_athome_skills.mission import bag_task
from base_athome_perception import wrist_geometry as wg
from base_athome_perception.moveit_bridge import MoveItBridge, decimate, pose_from, quat_from_matrix
from base_athome_perception.piper_kinematics import PiperKinematicsSolver
from base_athome_perception.teach_wizard import LABELS, POSES, TeachWizard
from base_athome_perception.wrist_detector import TargetTracker, make_bag_detector, make_detector

CONFIG = os.path.join(os.path.dirname(os.path.realpath(__file__)), "..", "config", "arm_poses_and_limits.yaml")
if os.environ.get("ATHOME_SIM") == "1":  # en simulacion no pisar la calibracion del robot real
    _SIM_CONFIG = os.path.expanduser("~/.local/share/base_athome/sim/arm_poses_and_limits.yaml")
    if not os.path.exists(_SIM_CONFIG):
        import shutil
        os.makedirs(os.path.dirname(_SIM_CONFIG), exist_ok=True)
        shutil.copy(os.environ.get("ATHOME_SIM_POSES") or CONFIG, _SIM_CONFIG)
    CONFIG = _SIM_CONFIG
FLOOR_POSE = "pre_grasp_floor"     # se ensena con ensenar_pregrasp; si no hay, la calculada
FLOOR_PREGRASP_DEG = [-15.4, 161.1, -112.4, 83.9, 78.0, -50.9]
SHOULDER_IN_BASE = np.array([0.005, 0.175, 1.065])  # arm_base_link en base_link (URDF) con el elevador ARRIBA
# (torso_lift_joint=0). Con el elevador abajo z baja en |torso_lift_joint| (prismatico vertical): ver
# shoulder_in_base(). La altura MEDIDA llega por /torso/joint_states (base_athome_bringup torso_state).
GRIPPER_OPEN_MM, GRIPPER_CLOSE_MM = 105.0, 50.0
# Una bolsa se toma por el nudo o el asa (2-15 mm comprimidos): con el cierre de 50 mm de los objetos rigidos se escapa.
GRIPPER_CLOSE_BAG_MM = 0.0
BAG_EMPTY_MAX_MM = 2.0      # cerro casi del todo: sin material entre los dedos. Falta confirmar con esfuerzo en el robot real
WINDOW = "Manipulacion Base@Home"


class Manipulacion(Node):
    # Juego de 3 poses que ensena la ventana ([T], [4-6], [1-3]); Restaurant usa otro.
    TEACH_POSES = POSES
    TEACH_LABELS: dict = {}
    TEACH_HINTS: dict = {}
    OPEN_GRIPPER_POSES = ("pre_grasp",)

    def __init__(self):
        super().__init__("manipulacion_simple")
        self.lock = threading.Lock()
        self.joints_deg: List[float] = [0.0] * 6
        self.gripper_mm = 0.0
        self.feedback_ok = False
        self.enabled = False
        self.frame: Optional[np.ndarray] = None
        self.status = ""
        self.busy = False
        self.abort = False

        self.create_subscription(String, "/arm/status", self._status_cb, 10)
        self.torso_pos: Optional[float] = 0.0 if os.environ.get("ATHOME_SIM") == "1" else None  # sim: fijo arriba
        self.torso_t = time.time() if self.torso_pos is not None else None
        self.create_subscription(JointState, "/torso/joint_states", self._torso_cb, 10)
        self.create_subscription(Image, "/arm_camera/image_raw", self._image_cb, qos_profile_sensor_data)
        self.pub_joint = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_traj = self.create_publisher(JointTrajectory, "/arm/joint_trajectory", 2)
        self.pub_gripper = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.cli_enable = self.create_client(SetBool, "/arm/set_enable")
        self.fjt = ActionClient(self, FollowJointTrajectory, "/arm/follow_joint_trajectory") \
            if FollowJointTrajectory is not None else None
        self.last_exec_msg = ""

        self.poses = {}
        self.reload_poses()
        self.calib = wg.WristCalib.from_dict(pose_store.load_pose_config(CONFIG).get("wrist_camera_calibration"))
        self.measured_cm = 24.0
        self.detection = None  # (bbox, z_est)
        env = pose_store.load_pose_config(CONFIG).get("safety_envelope", {})
        self.max_advance_cm = float(env.get("max_advance_cm", 30.0))
        self.advance_adjust_cm = float(env.get("advance_adjust_cm", 0.0))
        # Velocidad de los movimientos (% de la maxima de MoveIt / joint_limits). Slider Velocidad_%.
        self.speed_pct = float(np.clip(env.get("arm_speed_pct", 30), 5, 100))
        self.align_before_grasp = bool(env.get("align_before_grasp", False))  # desactivado: [G] como antes
        self.object_hold_s = float(env.get("object_hold_s", 8.0))
        models = os.path.expanduser("~/base_athome_ws/src/base_athome_perception/models")
        self.detector = make_detector(models)
        self.models_dir = models
        self.bag_detector = None      # se carga al primer pedido de una bolsa
        self.bag_vision = False
        self.plane_z = None
        self.bag_expected = None      # donde deberia estar la bolsa (base_link): descarta detecciones lejos de ahi
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.tracker = TargetTracker()
        self.det_label = ""
        self.obj_base = None  # centro del objeto en base_link, deteccion EN VIVO (None si se perdio)
        # Objetivo FIJADO: ultima posicion del objeto en base_link; se conserva object_hold_s aunque la
        # deteccion parpadee. Es lo que se dibuja en RViz y adonde va el agarre (un solo calculo).
        self.target = None    # dict(obj, z, box, label, t, q_deg)
        # Tambien en /arm_camera/object_markers: ese topico ya esta en las configuraciones de RViz
        self.pub_markers = self.create_publisher(MarkerArray, "/manipulacion/objeto", 5)
        self.pub_markers_compat = self.create_publisher(MarkerArray, "/arm_camera/object_markers", 5)
        self.create_timer(0.1, self._publish_markers)
        self.kin = PiperKinematicsSolver()
        self.moveit = MoveItBridge(self, torso_provider=lambda: self.torso_pos if self.torso_known() else None)
        self.move_group_proc = None
        threading.Thread(target=self._ensure_move_group, daemon=True).start()
        self.wizard = TeachWizard(self.set_torque, self.save_pose, self.TEACH_POSES, self.TEACH_LABELS, self.TEACH_HINTS)
        threading.Thread(target=self._vision_loop, daemon=True).start()

    # ---------------------------------------------------------------- ROS
    def _status_cb(self, msg: String):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.feedback_ok = bool(st.get("feedback_ok", False))
            self.enabled = bool(st.get("enabled", False))
            if self.feedback_ok:
                self.joints_deg = [float(v) for v in st.get("joints_deg", self.joints_deg)[:6]]
                self.gripper_mm = float(st.get("gripper_mm", self.gripper_mm))

    def _torso_cb(self, msg: JointState):
        if "torso_lift_joint" in msg.name:
            self.torso_pos = float(msg.position[list(msg.name).index("torso_lift_joint")])
            self.torso_t = time.time()

    def torso_known(self) -> bool:
        return self.torso_pos is not None and (os.environ.get("ATHOME_SIM") == "1"
                                               or time.time() - (self.torso_t or 0.0) < 2.0)

    def shoulder_in_base(self) -> np.ndarray:
        """arm_base_link en base_link con la altura del elevador MEDIDA (o asumida arriba si no se sabe)."""
        z = self.torso_pos if self.torso_known() else 0.0
        return SHOULDER_IN_BASE + np.array([0.0, 0.0, z])

    def _image_cb(self, msg: Image):
        if msg.encoding not in ("bgr8", "rgb8"):
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        with self.lock:
            self.frame = img.copy()

    def set_torque(self, on: bool) -> bool:
        """Habilita (bloquea en la posicion actual) o libera el brazo, via servicio (hay respuesta)."""
        if not self.cli_enable.wait_for_service(timeout_sec=2.0):
            self.status = "Servicio /arm/set_enable no disponible (¿jetson-head corriendo?)."
            return False
        fut = self.cli_enable.call_async(SetBool.Request(data=on))
        t_end = time.time() + 3.0
        while not fut.done() and time.time() < t_end:
            time.sleep(0.02)
        ok = fut.done() and fut.result().success
        self.status = ("Brazo BLOQUEADO." if on else "Brazo LIBRE (sin torque).") if ok else \
            f"No se pudo {'bloquear' if on else 'liberar'} el brazo: {fut.result().message if fut.done() else 'timeout'}"
        return ok

    def save_pose(self, name: str, joints_deg, gripper_mm: float) -> Optional[str]:
        if name in self.OPEN_GRIPPER_POSES:
            gripper_mm = GRIPPER_OPEN_MM  # el pre-agarre siempre con la pinza abierta
        res = pose_store.save_named_pose(name, joints_deg, gripper_mm, path=CONFIG, source="teach_wizard")
        if not res:
            return res.get("error") or "pose invalida"
        self.reload_poses()
        self.get_logger().info(f"Pose {name} guardada: {[round(v, 1) for v in joints_deg]}")
        return None

    def reload_poses(self):
        self.poses = {n: p for n, p in pose_store.load_named_poses(CONFIG).items() if n in POSES or n == FLOOR_POSE}
        # Sin pose ensenada, la calculada.
        self.poses.setdefault(FLOOR_POSE, dict(joints_deg=list(FLOOR_PREGRASP_DEG), gripper_mm=GRIPPER_OPEN_MM))

    # ------------------------------------------------------------ movimiento
    def send_joints(self, q_deg, duration_s: float):
        js = JointState()
        js.name = [f"joint{i}" for i in range(1, 7)]
        js.position = [math.radians(float(v)) for v in q_deg]
        js.velocity = [float(duration_s)]
        self.pub_joint.publish(js)

    def gripper(self, mm: float):
        self.pub_gripper.publish(Float32(data=float(mm)))

    def go_to(self, name: str):
        """Va a una pose guardada con un plan sin colisiones (MoveIt) o, si no esta, en linea articular lenta."""
        if self.busy or self.wizard.active:
            return
        pose = self.poses.get(name)
        if not pose:
            self.status = f"{self.wizard.labels.get(name, name)} no esta guardada. Usa [T] para ensenarla."
            return
        if not self.feedback_ok:
            self.status = "Sin datos reales del brazo: no lo muevo."
            return
        threading.Thread(target=self._go_worker, args=(name, pose), daemon=True).start()

    def _go_worker(self, name, pose):
        self.busy, self.abort = True, False
        try:
            if not self.enabled and not self.set_torque(True):
                return
            target = [float(v) for v in pose["joints_deg"]]
            plan = None
            if self.moveit.available():
                plan = self.moveit.plan_to_joints([math.radians(v) for v in self.joints_deg],
                                                  [math.radians(v) for v in target], vel_scale=self.vel_scale())
            if plan:
                self.status = f"Yendo a {self.wizard.labels.get(name, name)} (MoveIt)..."
                self._execute(plan)
            else:
                dmax = max(abs(a - b) for a, b in zip(target, self.joints_deg))
                dps = 60.0 * self.vel_scale()  # 100 % = 60 grados/s, 30 % = 18 grados/s
                dur = float(np.clip(dmax / max(dps, 3.0), 1.0, 20.0))
                self.status = f"Yendo a {self.wizard.labels.get(name, name)} ({dur:.1f} s)..."
                self.send_joints(target, dur)
                time.sleep(dur + 0.3)
            self.gripper(pose.get("gripper_mm", GRIPPER_OPEN_MM))
            if not self.abort:
                self.status = f"En {self.wizard.labels.get(name, name)}."
        finally:
            self.busy = False

    def _ensure_move_group(self):
        """MoveIt dentro del mismo comando: si no hay move_group corriendo, lo lanza y lo cierra al salir."""
        if self.moveit.cli_ik.wait_for_service(timeout_sec=1.5):
            return
        self.status = "Iniciando MoveIt..."
        env = dict(os.environ)
        log = open(f"/tmp/move_group_manipulacion_dom{os.environ.get('ROS_DOMAIN_ID', '0')}.log", "w")  # sim y real separados
        self.move_group_proc = subprocess.Popen(
            ["ros2", "launch", "base_athome_moveit_config", "move_group.launch.py"],
            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        ok = self.moveit.cli_ik.wait_for_service(timeout_sec=40.0)
        self.status = "MoveIt listo." if ok else f"MoveIt no arranco (ver /tmp/move_group_manipulacion_dom{os.environ.get('ROS_DOMAIN_ID', '0')}.log)."

    def stop_move_group(self):
        if self.move_group_proc is not None and self.move_group_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.move_group_proc.pid), signal.SIGINT)
                self.move_group_proc.wait(timeout=5)
            except Exception:
                pass

    def _execute(self, points) -> bool:
        """Ejecuta la trayectoria COMPLETA. Con la accion follow_joint_trajectory el resultado lo confirma
        el controlador con el feedback MEDIDO; sin ella (controlador viejo) se publica en el topico y se
        espera por tiempo."""
        pts = decimate(points, min_dt=0.05)
        if len(pts) < 2:
            return True
        msg = JointTrajectory()
        msg.joint_names = [f"joint{i}" for i in range(1, 7)]
        for q, t in pts[1:]:
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            p.time_from_start = Duration(sec=int(t), nanosec=int((t % 1.0) * 1e9))
            msg.points.append(p)
        if self.fjt is not None and self.fjt.wait_for_server(timeout_sec=0.5):
            return self._execute_fjt(msg, pts[-1][1])
        if getattr(self, "require_feedback_action", False):
            self.last_exec_msg = "sin accion follow_joint_trajectory: movimiento rechazado"
            return False
        self.pub_traj.publish(msg)
        t_end = time.time() + pts[-1][1] + 0.5
        while time.time() < t_end:
            if self.abort:
                # detener: mantener la posicion actual medida
                self.send_joints(list(self.joints_deg), 0.6)
                self.status = "Movimiento cancelado."
                return False
            time.sleep(0.05)
        self.last_exec_msg = "ejecutado por tiempo (sin accion follow_joint_trajectory)"
        return True

    def _execute_fjt(self, traj: JointTrajectory, duration_s: float) -> bool:
        goal = FollowJointTrajectory.Goal(trajectory=traj)
        fut = self.fjt.send_goal_async(goal)
        t_end = time.time() + 3.0
        while not fut.done() and time.time() < t_end:
            time.sleep(0.02)
        gh = fut.result() if fut.done() else None
        if gh is None or not gh.accepted:
            self.last_exec_msg = "el controlador rechazo la trayectoria (¿deshabilitado o sin feedback?)"
            self.status = f"Movimiento: {self.last_exec_msg}"
            return False
        res = gh.get_result_async()
        t_end = time.time() + duration_s + 10.0
        while not res.done():
            if self.abort or time.time() > t_end:
                cf = gh.cancel_goal_async()
                t_c = time.time() + 3.0
                while not res.done() and time.time() < t_c:
                    time.sleep(0.02)
                self.last_exec_msg = "cancelado" if self.abort else "sin resultado a tiempo (cancelado)"
                self.status = f"Movimiento {self.last_exec_msg}."
                return False
            time.sleep(0.02)
        r = res.result().result
        self.last_exec_msg = r.error_string
        ok = r.error_code == FollowJointTrajectory.Result.SUCCESSFUL and res.result().status == 4
        if not ok:
            self.status = f"Movimiento fallido: {r.error_string}"
        return ok

    # ------------------------------------------------------------ API bloqueante (misiones)
    def wait_feedback(self, timeout_s: float = 3.0) -> bool:
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            if self.feedback_ok:
                return True
            time.sleep(0.05)
        return False

    def go_to_blocking(self, name: str):
        """Va a una pose guardada y devuelve (ok, mensaje). La llegada la confirma el controlador."""
        pose = self.poses.get(name)
        if not pose:
            return False, f"la pose '{name}' no esta guardada"
        if not self.wait_feedback():
            return False, "sin realimentacion del brazo"
        if not self.enabled and not self.set_torque(True):
            return False, f"no se pudo habilitar el brazo: {self.status}"
        target = [float(v) for v in pose["joints_deg"]]
        plan = None
        if self.moveit.available():
            plan = self.moveit.plan_to_joints([math.radians(v) for v in self.joints_deg],
                                              [math.radians(v) for v in target], vel_scale=self.vel_scale())
            if plan is None:
                return False, f"MoveIt no encontro un plan sin colisiones hacia '{name}'"
        else:
            return False, "MoveIt no disponible: no se mueve el brazo sin plan de colisiones"
        if not self._execute(plan):
            return False, f"no llego a '{name}': {self.last_exec_msg}"
        self.gripper(pose.get("gripper_mm", GRIPPER_OPEN_MM))
        time.sleep(1.4)
        return True, f"en '{name}' ({self.last_exec_msg})"

    def detect_blocking(self, label: str = "", timeout_s: float = 5.0):
        """Espera una deteccion fresca (opcionalmente que la etiqueta contenga `label`).
        Devuelve (ok, mensaje, datos) con caja, distancia estimada y centro en base_link."""
        t_end = time.time() + timeout_s
        wants = [w.strip() for w in (label or "").lower().split("|") if w.strip()]
        want = wants[0] if len(wants) == 1 else "/".join(wants)
        while time.time() < t_end and not self.abort:
            tg = self.held_target()
            lab = (tg or {}).get("label", "")
            if tg is not None and (not wants or any(w in lab.lower() for w in wants) or lab.lower() in want):
                held = self.detection is None
                data = dict(label=lab, box=[round(float(v), 1) for v in tg["box"]], distance_m=round(float(tg["z"]), 3),
                            base_link_xyz=[round(float(v), 3) for v in tg["obj"]], held=held)
                return True, f"veo '{lab}' a {tg['z'] * 100:.0f} cm" + (" (retenido)" if held else ""), data
            time.sleep(0.05)
        return False, f"no veo {'un ' + want if want else 'ningun objeto'}", {}

    def locate_bag_blocking(self, plane_z: float, expected_xyz=None, timeout_s: float = 8.0):
        """Mide la bolsa con la camara de muneca, sin mover nada. (ok, mensaje, datos con base_link_xyz)."""
        ok, why = self._start_bag_vision(plane_z, expected_xyz)
        if not ok:
            return False, why, {}
        try:
            ok, msg, data = self.detect_blocking("bag", timeout_s=timeout_s)
            if not ok:
                return False, msg, {}
            return True, "bolsa medida con la camara de muneca", dict(base_link_xyz=[float(v) for v in self.target["obj"]])
        finally:
            self.plane_z, self.bag_vision, self.bag_expected = None, False, None

    def _start_bag_vision(self, plane_z: float, expected_xyz=None):
        """Pasa la camara de muneca al detector de bolsas y a medir sobre el plano z (altura vista por la cabeza)."""
        if self.bag_detector is None:
            self.bag_detector = make_bag_detector(self.models_dir)
        if hasattr(self.bag_detector, "available") and not self.bag_detector.available():
            return False, f"faltan los pesos del detector de bolsas: {self.bag_detector.weights}"
        self.tracker = type(self.tracker)()      # sin arrastrar lo que veia antes
        self.target, self.detection = None, None
        self.plane_z, self.bag_vision = float(plane_z), True
        self.bag_expected = None if expected_xyz is None else [float(v) for v in expected_xyz]
        return True, ""

    def _bag_candidates(self, dets, size):
        """Mide cada deteccion con bag_vision y deja solo la que corresponde a la bolsa. (dets, punto base_link o None)."""
        try:
            cam = self.tf_buf.lookup_transform("base_footprint", "arm_camera_optical_frame", rclpy.time.Time())
            ground = self.tf_buf.lookup_transform("base_footprint", "base_link", rclpy.time.Time())
        except Exception as e:  # noqa: BLE001 - sin TF no hay medicion: no se agarra a ciegas
            self.get_logger().warn(f"Sin TF de la camara de muneca: {e}", throttle_duration_sec=5.0)
            return [], None
        t, q = cam.transform.translation, cam.transform.rotation
        R = bag_vision.quat_to_R(q)
        K = (self.calib.focal_px, self.calib.focal_px, self.calib.cx_px, self.calib.cy_px)
        offset = ground.transform.translation
        expected = None if self.bag_expected is None else (self.bag_expected[0] + offset.x, self.bag_expected[1] + offset.y)
        measures = [bag_vision.wrist_measure(d[0], (t.x, t.y, t.z), R, K, self.plane_z, size) for d in dets]
        best, why = bag_vision.choose_nearest(measures, expected)
        if best is None:
            if dets:
                self.get_logger().warn(f"Bolsa de la muneca descartada: {why}", throttle_duration_sec=3.0)
            return [], None
        xy = measures[best].xy
        return [dets[best]], np.array([xy[0] - offset.x, xy[1] - offset.y, self.plane_z - offset.z])

    def pick_blocking(self, label: str = "", plane_z=None, retreat: bool = True, memory_xyz=None):
        """Agarre completo con confirmacion por el ancho MEDIDO de la pinza. (ok, mensaje, datos).
        `plane_z`: altura del objeto, medida por la camara de la cabeza. La camara de muneca lo ubica con el detector
        de bolsas y el rayo del pixel cortado con esa altura; el tamano de la caja no sirve sin diametro conocido."""
        self.abort = False
        if not self.torso_known():
            return False, "altura del elevador sin medir (/torso/joint_states): no se calcula un agarre a ciegas", {}
        if plane_z is not None:
            ok, why = self._start_bag_vision(plane_z, memory_xyz)
            if not ok:
                return False, why, {}
        try:
            ok, msg, data = self.detect_blocking(label, timeout_s=3.0 if memory_xyz is not None else 8.0 if plane_z is not None else 5.0)
            if not ok and memory_xyz is None:
                return False, msg, data
            fixed = None
            if not ok:     # de cerca puede dejar de verla: la base no se movio, sigue donde la medi
                self.get_logger().warn(f"{msg}: agarro con la posicion guardada {[round(float(v), 3) for v in memory_xyz]}")
                # bag_vision sigue activa: con ella apagada el detector general escribiria su objeto en self.target
                fixed = [float(v) for v in memory_xyz]
                data = dict(source="memoria", base_link_xyz=[round(v, 3) for v in fixed])
            if plane_z is None:
                ok, why = self._grasp_core()
            else:
                offset = bag_task.GROUND_TO_BASE_LINK_M
                ok, why = self._grasp_core(close_mm=GRIPPER_CLOSE_BAG_MM, grasp_z=bag_task.grasp_height(plane_z) - offset,
                                           fixed_xyz=fixed)
        finally:
            self.plane_z, self.bag_vision, self.bag_expected = None, False, None
        if not ok:
            return False, why, data
        time.sleep(0.3)
        width = float(self.gripper_mm)
        data["gripper_mm"] = round(width, 1)
        if width <= (GRIPPER_CLOSE_MM + 5.0 if plane_z is None else BAG_EMPTY_MAX_MM):
            self.gripper(GRIPPER_OPEN_MM)
            return False, f"agarre vacio: la pinza cerro hasta {width:.0f} mm (sin objeto)", data
        if retreat and "post_grasp" in self.poses:
            ok2, msg2 = self.go_to_blocking("post_grasp")
            if not ok2:
                return False, f"objeto tomado pero no pude retirarme: {msg2}", data
        return True, f"objeto tomado (pinza {width:.0f} mm)", data

    def receive_blocking(self, wait_s: float = 5.0):
        """Plan B: la persona pone la bolsa en la pinza. Abre, espera, cierra y confirma por ancho. No mueve el brazo."""
        self.abort = False
        if not self.enabled and not self.set_torque(True):
            return False, f"no se pudo habilitar el brazo ({self.status})", {}
        self.gripper(GRIPPER_OPEN_MM)
        end = time.time() + wait_s
        while time.time() < end:
            if self.abort:
                return False, "cancelado", {}
            time.sleep(0.1)
        self.gripper(GRIPPER_CLOSE_BAG_MM)
        time.sleep(2.0)
        width = float(self.gripper_mm)
        if width <= BAG_EMPTY_MAX_MM:
            self.gripper(GRIPPER_OPEN_MM)
            return False, f"agarre vacio: la pinza cerro hasta {width:.0f} mm (sin bolsa)", dict(gripper_mm=round(width, 1))
        return True, f"bolsa recibida (pinza {width:.0f} mm)", dict(gripper_mm=round(width, 1))

    def give_blocking(self, wait_s: float = 8.0):
        """Entrega: espera, abre la pinza y confirma por ancho. El brazo no se mueve. (ok, mensaje, datos)."""
        self.abort = False
        end = time.time() + wait_s
        while time.time() < end:
            if self.abort:
                return False, "cancelado", {}
            time.sleep(0.1)
        self.gripper(GRIPPER_OPEN_MM)
        time.sleep(2.0)
        if self.gripper_mm < GRIPPER_OPEN_MM - 10.0:
            return False, f"la pinza no abrio ({self.gripper_mm:.0f} mm)", {}
        return True, "objeto entregado", {}

    def place_blocking(self):
        """Coloca adelante (PRE-GRASP), abre la pinza, verifica y vuelve a HOME. (ok, mensaje)."""
        self.abort = False
        ok, msg = self.go_to_blocking("pre_grasp")   # pre_grasp abre la pinza al llegar
        if not ok:
            return False, msg
        time.sleep(0.5)
        if self.gripper_mm < GRIPPER_OPEN_MM - 10.0:
            return False, f"la pinza no abrio (mide {self.gripper_mm:.0f} mm)"
        ok, msg = self.go_to_blocking("home")
        return ok, ("objeto colocado; " + msg) if ok else f"colocado pero no volvi a HOME: {msg}"

    # ---------------------------------------------------------------- vision
    def _vision_loop(self):
        while rclpy.ok():
            with self.lock:
                frame = None if self.frame is None else self.frame.copy()
            if frame is None:
                time.sleep(0.05)
                continue
            try:
                dets = (self.bag_detector if self.bag_vision else self.detector).detect(frame)
                bag_point = None
                if self.bag_vision:
                    dets, bag_point = self._bag_candidates(dets, (frame.shape[1], frame.shape[0]))
            except Exception as e:
                self.get_logger().warn(f"Detector: {e}", throttle_duration_sec=10.0)
                time.sleep(0.5)
                continue
            tr = self.tracker.update(dets)
            if tr is None:
                self.detection = None
                self.obj_base = None
            else:
                box, width = tr
                z = wg.size_distance(width, self.calib)
                with self.lock:
                    q_det = list(self.joints_deg)
                if self.bag_vision:
                    if bag_point is None:      # el tracker sostiene la caja vieja, pero sin medicion no hay objetivo
                        self.detection = None
                        time.sleep(0.05)
                        continue
                    obj = bag_point
                else:
                    obj = self._object_point(box, z, q_det)
                self.detection = (box, z)
                self.det_label = f"{self.tracker.label} {self.tracker.conf:.2f} ({self.detector.source})"
                self.obj_base = obj
                self.target = dict(obj=obj, z=z, box=box, label=self.tracker.label or "", t=time.time(), q_deg=q_det)
            time.sleep(0.03)

    def vel_scale(self) -> float:
        return float(np.clip(self.speed_pct / 100.0, 0.05, 1.0))

    def used_advance(self, z: float) -> float:
        """Avance real a usar (m): calibrado + ajuste, nunca mas que el tope."""
        adv = wg.advance_needed(z, self.calib) + self.advance_adjust_cm / 100.0
        return float(np.clip(adv, 0.0, self.max_advance_cm / 100.0))

    def _object_point(self, box, z, q_deg):
        """Centro del objeto en base_link con las articulaciones del MOMENTO de la deteccion.
        Mismo calculo para RViz y para el agarre (avance calibrado + ajuste, SIN el tope)."""
        tcp_mm, R = self.kin.forward_kinematics(np.array([math.radians(v) for v in q_deg]))
        x1, y1, x2, y2 = box
        left, up = wg.lateral_offsets((x1 + x2) / 2, (y1 + y2) / 2, z, self.calib)
        adv = max(0.0, wg.advance_needed(z, self.calib) + self.advance_adjust_cm / 100.0)
        return self.shoulder_in_base() + wg.object_center(tcp_mm / 1000.0, R, adv, left, up, self.calib)

    def held_target(self):
        """Objetivo fijado si es reciente (en vivo o retenido < object_hold_s); si no, None."""
        tg = self.target
        if tg is None or time.time() - tg["t"] > self.object_hold_s:
            return None
        return tg

    def _object_in_base(self, box, z):
        with self.lock:
            q = list(self.joints_deg)
        tcp_mm, R = self.kin.forward_kinematics(np.array([math.radians(v) for v in q]))
        x1, y1, x2, y2 = box
        left, up = wg.lateral_offsets((x1 + x2) / 2, (y1 + y2) / 2, z, self.calib)
        obj = wg.object_center(tcp_mm / 1000.0, R, wg.advance_needed(z, self.calib), left, up, self.calib)
        return self.shoulder_in_base() + obj

    def _publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        def mk(mid, mtype):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = now
            m.ns = "arm_target"
            m.id = mid
            m.type = mtype
            m.action = Marker.ADD if self.held_target() is not None else Marker.DELETE
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.01   # RViz marca como error una escala 0, aun al borrar
            m.lifetime.nanosec = 500_000_000
            return m
        cyl, grasp, arrow, txt = mk(0, Marker.CYLINDER), mk(1, Marker.SPHERE), mk(2, Marker.ARROW), mk(3, Marker.TEXT_VIEW_FACING)
        tg = self.held_target()
        if tg is not None:
            o = tg["obj"]
            live = self.detection is not None
            cyl.pose.position.x, cyl.pose.position.y, cyl.pose.position.z = map(float, o)
            cyl.scale.x = cyl.scale.y = self.calib.object_diameter_m
            cyl.scale.z = 0.20
            # verde = deteccion en vivo; amarillo = posicion RETENIDA (la deteccion parpadeo)
            cyl.color = ColorRGBA(r=0.1, g=0.9, b=0.3, a=0.85) if live else ColorRGBA(r=1.0, g=0.85, b=0.1, a=0.85)
            with self.lock:
                q = list(self.joints_deg)
            tcp_mm, R = self.kin.forward_kinematics(np.array([math.radians(v) for v in q]))
            tcp = self.shoulder_in_base() + tcp_mm / 1000.0
            app = R[:, 2] / np.linalg.norm(R[:, 2])
            gc = tcp - self.calib.grasp_center_back_m * app
            need = float(np.dot(np.asarray(o) - gc, app))           # avance necesario hasta el objeto fijado
            stop = gc + min(max(0.0, need), self.max_advance_cm / 100.0) * app  # donde quedara el centro de la pinza
            grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z = map(float, stop)
            grasp.scale.x = grasp.scale.y = grasp.scale.z = 0.025
            grasp.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.95)
            arrow.points = [Point(x=float(gc[0]), y=float(gc[1]), z=float(gc[2])),
                            Point(x=float(stop[0]), y=float(stop[1]), z=float(stop[2]))]
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.008, 0.016, 0.02
            arrow.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9)
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = float(o[0]), float(o[1]), float(o[2]) + 0.15
            txt.scale.z = 0.04
            txt.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            txt.text = (f"avance {need * 100:.0f} cm (tope {self.max_advance_cm:.0f})"
                        + ("" if live else f" RETENIDO {time.time() - tg['t']:.1f} s")
                        + (" FUERA DE TOPE" if need * 100 > self.max_advance_cm else ""))
        ma.markers = [cyl, grasp, arrow, txt]
        self.pub_markers.publish(ma)
        self.pub_markers_compat.publish(ma)

    def set_advance_limits(self, max_cm=None, adjust_cm=None):
        if max_cm is not None:
            self.max_advance_cm = float(max_cm)
        if adjust_cm is not None:
            self.advance_adjust_cm = float(adjust_cm)

    def save_advance_limits(self):
        import yaml
        cfg = pose_store.load_pose_config(CONFIG)
        env = cfg.setdefault("safety_envelope", {})
        env["max_advance_cm"] = round(self.max_advance_cm, 1)
        env["advance_adjust_cm"] = round(self.advance_adjust_cm, 1)
        env["arm_speed_pct"] = int(round(self.speed_pct))
        tmp = CONFIG + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
        os.replace(tmp, CONFIG)
        self.status = (f"Guardado: tope {self.max_advance_cm:.0f} cm, ajuste {self.advance_adjust_cm:+.0f} cm, "
                       f"velocidad {self.speed_pct:.0f} %.")

    def calibrate(self):
        if self.detection is None:
            self.status = "Calibrar: no veo el objeto."
            return
        z = self.detection[1]
        wg.add_measurement(self.calib, z, self.measured_cm / 100.0)
        cfg = pose_store.load_pose_config(CONFIG)
        cfg["wrist_camera_calibration"] = self.calib.to_dict()
        import yaml
        tmp = CONFIG + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            yaml.safe_dump(cfg, f, sort_keys=False, allow_unicode=True)
        os.replace(tmp, CONFIG)
        self.status = f"Calibrado: avance {wg.advance_needed(z, self.calib) * 100:.1f} cm para {self.measured_cm:.0f} cm medidos."

    # ---------------------------------------------------------------- agarre
    def grasp(self):
        if self.busy or self.wizard.active:
            return
        if self.detection is None:
            self.status = "Agarre: no veo el objeto en la camara de muneca."
            return
        if not self.moveit.available():
            self.status = "Agarre: MoveIt todavia no esta listo (arranca solo, esperar unos segundos)."
            return
        threading.Thread(target=self._grasp_worker, daemon=True).start()

    def _fresh_detection(self, after: float, timeout_s: float = 2.5):
        """Espera una deteccion tomada DESPUES de `after` (brazo quieto) y la devuelve."""
        t_end = time.time() + timeout_s
        while time.time() < t_end and not self.abort:
            if self.detection is not None and self.tracker.last_seen > after:
                return self.detection
            time.sleep(0.05)
        return None

    def align_with_object(self, max_iter: int = 5) -> bool:
        """Centra el objeto en el eje de la pinza con pequenos movimientos laterales/verticales.

        En cada vuelta: deteccion nueva con el brazo quieto -> desvio izquierda/arriba -> camino
        cartesiano corto en el plano perpendicular al avance (misma orientacion, sin acercarse).
        Termina cuando el desvio es < 0.8 cm lateral y < 1.2 cm vertical.
        """
        if not self.enabled and not self.set_torque(True):
            return False
        t_ref = time.time()
        for i in range(1, max_iter + 1):
            det = self._fresh_detection(after=t_ref + 0.3)
            if det is None:
                self.status = "Alineacion: perdi el objeto de vista."
                return False
            (x1, y1, x2, y2), z = det
            left, up = wg.lateral_offsets((x1 + x2) / 2, (y1 + y2) / 2, z, self.calib)
            if wg.is_aligned(left, up):
                self.status = f"Alineado con el objeto ({i - 1} correcciones)."
                return True
            q_now = [math.radians(v) for v in self.joints_deg]
            tcp_mm, R = self.kin.forward_kinematics(np.array(q_now))
            # corregir el 80 % del desvio por vuelta: converge sin pasarse por el ruido de la deteccion
            new_tcp = self.shoulder_in_base() + tcp_mm / 1000.0 + 0.8 * wg.alignment_offset(R, left, up)
            self.status = f"Alineando ({i}/{max_iter}): lateral {left * 100:+.1f} cm, vertical {up * 100:+.1f} cm"
            path = self.moveit.cartesian_path(q_now, [pose_from(new_tcp, quat_from_matrix(R.tolist()))],
                                              vel_scale=self.vel_scale() / 2, min_fraction=0.9)
            if not path:
                self.status = "Alineacion: no puedo moverme lateralmente desde aca."
                return False
            if not self._execute(path):
                return False
            time.sleep(0.4)  # dejar que se asiente la imagen
            t_ref = time.time()
        self.status = "Alineacion: no converge, sigo con la mejor posicion."
        return True

    def align_only(self):
        if self.busy or self.wizard.active:
            return
        if self.detection is None or not self.moveit.available():
            self.status = "Alinear: necesito ver el objeto y MoveIt listo."
            return

        def run():
            self.busy, self.abort = True, False
            try:
                self.align_with_object()
            finally:
                self.busy = False
        threading.Thread(target=run, daemon=True).start()

    def _grasp_worker(self):
        self.busy, self.abort = True, False
        try:
            # Como antes de la alineacion: usa la deteccion actual. Alinear es opcional con [A].
            if self.align_before_grasp and not self.align_with_object():
                return
            ok, why = self._grasp_core()
            self.status = "Agarre: listo." if ok else why
            if ok and "post_grasp" in self.poses:
                self.busy = False
                self.go_to("post_grasp")
        finally:
            self.busy = False

    @staticmethod
    def _axis_angle(axis, ang):
        axis = np.asarray(axis, float) / np.linalg.norm(axis)
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K

    def _approach_candidates(self, approach):
        """[(inclinacion_grados, R_extra)]: identidad primero y luego inclinaciones crecientes del eje de la
        pinza alrededor de los ejes perpendiculares (vertical del mundo y horizontal lateral)."""
        a = np.asarray(approach, float) / np.linalg.norm(approach)
        side = np.cross([0.0, 0.0, 1.0], a)
        side = side / np.linalg.norm(side) if np.linalg.norm(side) > 1e-3 else np.array([0.0, 1.0, 0.0])
        up = np.cross(a, side)
        out = [(0.0, np.eye(3))]
        for deg in (8.0, 16.0, 24.0, 30.0):
            r = math.radians(deg)
            for ax in (side, -side, up, -up):   # side primero: inclina la punta hacia abajo (objetos sobre mesa)
                out.append((deg, self._axis_angle(ax, r)))
        return out

    def _grasp_core(self, close_mm: float = GRIPPER_CLOSE_MM, grasp_z=None, fixed_xyz=None):
        """Aproximacion + avance + cierre + subida. (ok, motivo). No va a POST-GRASP.
        `grasp_z`: altura de agarre en base_link, si no es la del objeto detectado. `fixed_xyz`: punto guardado, en lugar de
        lo ultimo que vio la camara."""
        tg = dict(obj=fixed_xyz, t=time.time()) if fixed_xyz is not None else self.held_target()
        if tg is None:
            return False, "Agarre: no veo el objeto (ni hay una posicion retenida reciente)."
        if not self.torso_known():
            self.get_logger().warn("Agarre: elevador sin medicion; se ASUME arriba (torso_lift_joint=0)")
        obj = np.array(tg["obj"], dtype=float)   # el MISMO punto que muestra RViz
        if grasp_z is not None:
            obj[2] = float(grasp_z)
        q_now = [math.radians(v) for v in self.joints_deg]
        tcp_mm, R = self.kin.forward_kinematics(np.array(q_now))
        approach = R[:, 2] / np.linalg.norm(R[:, 2])
        gc_now = self.shoulder_in_base() + tcp_mm / 1000.0 - self.calib.grasp_center_back_m * approach
        need = float(np.linalg.norm(obj - gc_now))  # distancia del centro de la pinza al objeto fijado
        if need * 100.0 > self.max_advance_cm:
            return False, (f"Agarre: el objeto esta a {need * 100:.0f} cm de avance y el tope es "
                           f"{self.max_advance_cm:.0f} cm: acercar el robot/objeto o subir Avance_max_cm. No me muevo.")
        grasp_tcp = obj + self.calib.grasp_center_back_m * approach
        held = "" if self.detection is not None else f" (posicion RETENIDA hace {time.time() - tg['t']:.1f} s)"
        self.get_logger().info(f"Agarre: hacia {np.round(obj, 3).tolist()} en base_link, avance {need * 100:.1f} cm "
                               f"(tope {self.max_advance_cm:.0f} cm){held}")
        app_tcp = grasp_tcp - 0.10 * approach
        quat = quat_from_matrix(R.tolist())
        self.status = "Agarre: planificando..."
        # Candidatos de orientacion: primero la de la pose actual; si la linea recta no alcanza (borde del
        # espacio de trabajo), la pinza se inclina 8..30 grados (arriba/abajo/costados) hacia el objeto.
        # Medido offline (2026-11-03) desde PRE-GRASP: recta hasta ~20 cm de avance, inclinada 18 grados ~25 cm.
        q_app = plan = line = None
        chosen = None
        t_plan = time.time()
        for tilt_deg, axis_rot in self._approach_candidates(approach):
            R_c = axis_rot @ R
            a_c = R_c[:, 2] / np.linalg.norm(R_c[:, 2])
            g_tcp = obj + self.calib.grasp_center_back_m * a_c
            p_tcp = g_tcp - 0.10 * a_c
            quat_c = quat_from_matrix(R_c.tolist())
            # filtro rapido: si el punto de AGARRE no tiene IK, esta orientacion no sirve (evita caminos
            # cartesianos de ~1-3 s que van a fallar; antes la planificacion tardaba hasta 38 s)
            if self.moveit.compute_ik(pose_from(g_tcp, quat_c), q_now, avoid_collisions=False) is None:
                continue
            cand = self.moveit.compute_ik(pose_from(p_tcp, quat_c), q_now)
            if not cand:
                continue
            # el avance final y la subida van a la mitad de la velocidad elegida (mas control al agarrar)
            cand_line = self.moveit.cartesian_path(cand, [pose_from(g_tcp, quat_c)], vel_scale=self.vel_scale() / 2)
            if not cand_line:
                continue
            cand_plan = self.moveit.plan_to_joints(q_now, cand, vel_scale=self.vel_scale())
            if cand_plan:
                q_app, plan, line, chosen = cand, cand_plan, cand_line, (tilt_deg, g_tcp, quat_c)
                break
            if time.time() - t_plan > 45.0:
                break
        if line:
            tilt_deg, grasp_tcp, quat = chosen
            self.get_logger().info(f"Agarre: aproximacion con la pinza inclinada {tilt_deg:.0f} grados "
                                   f"({time.time() - t_plan:.1f} s de planificacion)")
        if not line:
            why = self.moveit.last_failure or "sin IK de aproximacion en ninguna orientacion"
            return False, (f"Agarre: objeto a {need * 100:.0f} cm de la pinza, fuera del alcance desde esta pose "
                           f"(probe hasta 30 grados de inclinacion; {why}). Acercar el robot u observar desde mas "
                           f"atras. No me muevo.")
        if not self.enabled and not self.set_torque(True):
            return False, f"Agarre: no se pudo habilitar el brazo ({self.status})"
        self.gripper(GRIPPER_OPEN_MM)
        self.status = "Agarre: aproximando..."
        if not self._execute(plan) or not self._execute(line):
            return False, f"Agarre: movimiento fallido ({self.last_exec_msg})"
        self.gripper(close_mm)
        time.sleep(1.5)
        lift = self.moveit.cartesian_path(line[-1][0], [pose_from(grasp_tcp + np.array([0, 0, 0.03]), quat)],
                                          vel_scale=self.vel_scale() / 2)
        if lift and not self._execute(lift):
            return False, f"Agarre: no pude levantar ({self.last_exec_msg})"
        return True, "Agarre: listo."

    # -------------------------------------------------------------------- UI
    def draw(self) -> np.ndarray:
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            q, g, fb, en = list(self.joints_deg), self.gripper_mm, self.feedback_ok, self.enabled
        if frame is None:
            frame = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(frame, "Sin imagen de la camara de muneca", (90, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        elif self.detection:
            (x1, y1, x2, y2), z = self.detection
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(frame, f"avance {self.used_advance(z) * 100:.1f} cm (tope {self.max_advance_cm:.0f})",
                        (int(x1), max(20, int(y1) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(frame, self.det_label, (int(x1), min(frame.shape[0] - 8, int(y2) + 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        h = frame.shape[0]
        panel = np.full((max(h, 480), 520, 3), 28, np.uint8)
        y = 30

        def put(text, color=(230, 230, 230), scale=0.55, bold=1, dy=26):
            nonlocal y
            cv2.putText(panel, text, (14, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, bold, cv2.LINE_AA)
            y += dy

        put("MANIPULACION", (0, 210, 255), 0.75, 2, 34)
        put(f"Brazo: {'DATOS OK' if fb else 'SIN DATOS'} | {'BLOQUEADO' if en else 'LIBRE'}",
            (90, 230, 90) if fb else (60, 60, 255))
        put("Actual: " + " ".join(f"{v:.0f}" for v in q) + f" | pinza {g:.0f} mm", scale=0.5)
        y += 6
        if self.wizard.active:
            cv2.rectangle(panel, (8, y - 22), (512, y + 62), (0, 110, 220), -1)
            put(f"ENSENANDO {self.wizard.labels[self.wizard.current]}  ({self.wizard.index + 1}/{len(self.wizard.sequence)})",
                (255, 255, 255), 0.7, 2, 30)
            put("Mueve el brazo a mano y presiona ENTER", (255, 255, 255), 0.6, 1, 40)
        for i, name in enumerate(self.TEACH_POSES, start=1):
            p = self.poses.get(name)
            mark = ">" if self.wizard.current == name else " "
            if p:
                diff = max(abs(a - b) for a, b in zip(p["joints_deg"], q))
                put(f"{mark}[{i}] {self.wizard.labels[name]:<10} dif {diff:5.1f} grados", (120, 230, 120) if diff < 2 else (230, 230, 230))
                put("     " + " ".join(f"{v:.0f}" for v in p["joints_deg"]), (170, 170, 170), 0.45, 1, 22)
            else:
                put(f"{mark}[{i}] {self.wizard.labels[name]:<10} (sin guardar)", (60, 160, 255))
        y += 8
        put("[T] ensenar las 3   [4][5][6] ensenar solo " + "/".join(self.wizard.labels[p] for p in self.TEACH_POSES),
            (200, 200, 200), 0.5)
        put("[ENTER] guardar   [ESC] cancelar", (200, 200, 200), 0.5)
        put("[1][2][3] ir a pose  [O]/[C] pinza  [A] alinear  [G] agarrar", (200, 200, 200), 0.5)
        put(f"[M] calibrar distancia (Medido_cm = {self.measured_cm:.0f})   [Q] salir", (200, 200, 200), 0.5)
        put(f"Avance: tope {self.max_advance_cm:.0f} cm, ajuste {self.advance_adjust_cm:+.0f} cm | "
            f"Velocidad {self.speed_pct:.0f} %   [K] guardar", (200, 200, 200), 0.5)
        y += 6
        for line in _wrap(self.wizard.message if self.wizard.active or not self.status else self.status, 52):
            put(line, (0, 230, 255), 0.5, 1, 22)
        out = np.zeros((max(h, panel.shape[0]), frame.shape[1] + panel.shape[1], 3), np.uint8)
        out[:h, :frame.shape[1]] = frame
        out[:panel.shape[0], frame.shape[1]:] = panel
        return out

    def handle_key(self, key: int) -> bool:
        if key in (ord("q"), ord("Q")):
            return False
        if key == 27:
            if self.wizard.active:
                self.wizard.cancel()
            elif self.busy:
                self.abort = True
            else:
                return False
        elif key in (ord("t"), ord("T")):
            if not self.busy:
                self.wizard.start()
        elif key in (ord("4"), ord("5"), ord("6")):  # ensenar SOLO una pose: 4 HOME, 5 PRE, 6 POST
            if not self.busy and not self.wizard.active:
                self.wizard.start(only=self.TEACH_POSES[key - ord("4")])
        elif key in (13, 10):
            with self.lock:
                q, g, fb = list(self.joints_deg), self.gripper_mm, self.feedback_ok
            self.wizard.confirm(q, g, fb)
        elif key in (ord("1"), ord("2"), ord("3")):
            self.go_to(self.TEACH_POSES[key - ord("1")])
        elif key in (ord("o"), ord("O")):
            self.gripper(GRIPPER_OPEN_MM)
        elif key in (ord("c"), ord("C")):
            self.gripper(GRIPPER_CLOSE_MM)
        elif key in (ord("g"), ord("G")):
            self.grasp()
        elif key in (ord("m"), ord("M")):
            self.calibrate()
        elif key in (ord("a"), ord("A")):
            self.align_only()
        elif key in (ord("k"), ord("K")):
            self.save_advance_limits()
        return True


def _wrap(text: str, width: int) -> List[str]:
    words, lines, cur = (text or "").split(), [], ""
    for w in words:
        if len(cur) + len(w) + 1 > width:
            lines.append(cur)
            cur = w
        else:
            cur = f"{cur} {w}".strip()
    if cur:
        lines.append(cur)
    return lines[:5]
