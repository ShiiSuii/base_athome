#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Manipulacion Base@Home (version simple, 2026-10-30).

Una sola ventana: video de la camara de MUNECA (no la de la cabeza) + panel de texto.

  [T]        ensenar poses: el brazo queda LIBRE; llevalo a HOME y [ENTER], luego a
             PRE-GRASP y [ENTER], luego a POST-GRASP y [ENTER]. Al final se bloquea.
  [ESC]      cancelar la ensenanza (bloquea el brazo) / salir si no hay ensenanza
  [1][2][3]  ir a HOME / PRE-GRASP / POST-GRASP guardadas
  [O] [C]    abrir / cerrar pinza
  [G]        agarrar el objeto visto con cinematica inversa (MoveIt arranca solo con este script)
  [M]        calibrar distancia con la cinta (Medido_cm = centro de la pinza -> cara del objeto)
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
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool

from base_athome_perception import pose_store
from base_athome_perception import wrist_geometry as wg
from base_athome_perception.moveit_bridge import MoveItBridge, decimate, pose_from, quat_from_matrix
from base_athome_perception.piper_kinematics import PiperKinematicsSolver
from base_athome_perception.teach_wizard import LABELS, POSES, TeachWizard

CONFIG = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config", "arm_poses_and_limits.yaml")
SHOULDER_IN_BASE = np.array([0.005, 0.175, 1.065])  # arm_base_link en base_link (URDF), mismos ejes
GRIPPER_OPEN_MM, GRIPPER_CLOSE_MM = 105.0, 50.0
WINDOW = "Manipulacion Base@Home"


class Manipulacion(Node):
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
        self.create_subscription(Image, "/arm_camera/image_raw", self._image_cb, qos_profile_sensor_data)
        self.pub_joint = self.create_publisher(JointState, "/arm/joint_cmd", 5)
        self.pub_traj = self.create_publisher(JointTrajectory, "/arm/joint_trajectory", 2)
        self.pub_gripper = self.create_publisher(Float32, "/arm/gripper_cmd", 5)
        self.cli_enable = self.create_client(SetBool, "/arm/set_enable")

        self.poses = {}
        self.reload_poses()
        self.calib = wg.WristCalib.from_dict(pose_store.load_pose_config(CONFIG).get("wrist_camera_calibration"))
        self.measured_cm = 24.0
        self.detection = None  # (bbox, z_est)
        self.kin = PiperKinematicsSolver()
        self.moveit = MoveItBridge(self)
        self.move_group_proc = None
        threading.Thread(target=self._ensure_move_group, daemon=True).start()
        self.yolo = None
        self.wizard = TeachWizard(self.set_torque, self.save_pose)
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
        if name == "pre_grasp":
            gripper_mm = GRIPPER_OPEN_MM  # el pre-agarre siempre con la pinza abierta
        res = pose_store.save_named_pose(name, joints_deg, gripper_mm, path=CONFIG, source="teach_wizard")
        if not res:
            return res.get("error") or "pose invalida"
        self.reload_poses()
        self.get_logger().info(f"Pose {name} guardada: {[round(v, 1) for v in joints_deg]}")
        return None

    def reload_poses(self):
        self.poses = {n: p for n, p in pose_store.load_named_poses(CONFIG).items() if n in POSES}

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
            self.status = f"{LABELS[name]} no esta guardada. Usa [T] para ensenarla."
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
                                                  [math.radians(v) for v in target], vel_scale=0.25)
            if plan:
                self.status = f"Yendo a {LABELS[name]} (MoveIt)..."
                self._execute(plan)
            else:
                dmax = max(abs(a - b) for a, b in zip(target, self.joints_deg))
                dur = float(np.clip(dmax / 20.0, 2.0, 10.0))  # ~20 grados/s
                self.status = f"Yendo a {LABELS[name]} ({dur:.1f} s)..."
                self.send_joints(target, dur)
                time.sleep(dur + 0.3)
            self.gripper(pose.get("gripper_mm", GRIPPER_OPEN_MM))
            if not self.abort:
                self.status = f"En {LABELS[name]}."
        finally:
            self.busy = False

    def _ensure_move_group(self):
        """MoveIt dentro del mismo comando: si no hay move_group corriendo, lo lanza y lo cierra al salir."""
        if self.moveit.cli_ik.wait_for_service(timeout_sec=1.5):
            return
        self.status = "Iniciando MoveIt..."
        env = dict(os.environ)
        log = open("/tmp/move_group_manipulacion.log", "w")
        self.move_group_proc = subprocess.Popen(
            ["ros2", "launch", "base_athome_moveit_config", "move_group.launch.py"],
            stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
        ok = self.moveit.cli_ik.wait_for_service(timeout_sec=40.0)
        self.status = "MoveIt listo." if ok else "MoveIt no arranco (ver /tmp/move_group_manipulacion.log)."

    def stop_move_group(self):
        if self.move_group_proc is not None and self.move_group_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.move_group_proc.pid), signal.SIGINT)
                self.move_group_proc.wait(timeout=5)
            except Exception:
                pass

    def _execute(self, points) -> bool:
        """Manda la trayectoria COMPLETA al controlador (la sigue continua a 50 Hz: sin temblar)."""
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
        self.pub_traj.publish(msg)
        t_end = time.time() + pts[-1][1] + 0.5
        while time.time() < t_end:
            if self.abort:
                # detener: mantener la posicion actual medida
                self.send_joints(list(self.joints_deg), 0.6)
                self.status = "Movimiento cancelado."
                return False
            time.sleep(0.05)
        return True

    # ---------------------------------------------------------------- vision
    def _vision_loop(self):
        while rclpy.ok():
            with self.lock:
                frame = None if self.frame is None else self.frame.copy()
            if frame is None:
                time.sleep(0.05)
                continue
            if self.yolo is None:
                try:
                    from ultralytics import YOLO
                    models = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "models")
                    self.yolo = YOLO(os.path.join(models, "yolov8n.pt"))
                except Exception as e:
                    self.get_logger().warn(f"YOLO no disponible: {e}")
                    self.yolo = False
            if not self.yolo:
                time.sleep(0.5)
                continue
            res = self.yolo.predict(frame, imgsz=320, conf=0.28, classes=[39, 41, 75], verbose=False, device="cpu")[0]
            best = None
            for b in res.boxes:
                x1, y1, x2, y2 = (float(v) for v in b.xyxy[0].tolist())
                if x2 - x1 >= 25 and y2 - y1 >= 40 and (best is None or float(b.conf) > best[1]):
                    best = ((x1, y1, x2, y2), float(b.conf))
            if best:
                (x1, y1, x2, y2), _ = best
                self.detection = ((x1, y1, x2, y2), wg.size_distance(x2 - x1, self.calib))
            else:
                self.detection = None
            time.sleep(0.1)

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

    def _grasp_worker(self):
        self.busy, self.abort = True, False
        try:
            (x1, y1, x2, y2), z = self.detection
            q_now = [math.radians(v) for v in self.joints_deg]
            tcp_mm, R = self.kin.forward_kinematics(np.array(q_now))
            adv = wg.advance_needed(z, self.calib)
            left, up = wg.lateral_offsets((x1 + x2) / 2, (y1 + y2) / 2, z, self.calib)
            obj = SHOULDER_IN_BASE + wg.object_center(tcp_mm / 1000.0, R, adv, left, up, self.calib)
            approach = R[:, 2] / np.linalg.norm(R[:, 2])
            grasp_tcp = obj + self.calib.grasp_center_back_m * approach
            app_tcp = grasp_tcp - 0.10 * approach
            quat = quat_from_matrix(R.tolist())
            self.status = "Agarre: planificando..."
            q_app = self.moveit.compute_ik(pose_from(app_tcp, quat), q_now)
            plan = self.moveit.plan_to_joints(q_now, q_app, vel_scale=0.2) if q_app else None
            line = self.moveit.cartesian_path(q_app, [pose_from(grasp_tcp, quat)]) if plan else None
            if not line:
                self.status = "Agarre: sin solucion (objeto fuera de alcance o choque). No me muevo."
                return
            if not self.enabled and not self.set_torque(True):
                return
            self.gripper(GRIPPER_OPEN_MM)
            self.status = "Agarre: aproximando..."
            if not self._execute(plan) or not self._execute(line):
                return
            self.gripper(GRIPPER_CLOSE_MM)
            time.sleep(1.5)
            lift = self.moveit.cartesian_path(line[-1][0], [pose_from(grasp_tcp + np.array([0, 0, 0.03]), quat)])
            if lift:
                self._execute(lift)
            self.status = "Agarre: listo."
            if "post_grasp" in self.poses:
                self.busy = False
                self.go_to("post_grasp")
        finally:
            self.busy = False

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
            cv2.putText(frame, f"avance {wg.advance_needed(z, self.calib) * 100:.1f} cm", (int(x1), max(20, int(y1) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
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
            put(f"ENSENANDO {LABELS[self.wizard.current]}  ({self.wizard.index + 1}/3)", (255, 255, 255), 0.7, 2, 30)
            put("Mueve el brazo a mano y presiona ENTER", (255, 255, 255), 0.6, 1, 40)
        for i, name in enumerate(POSES, start=1):
            p = self.poses.get(name)
            mark = ">" if self.wizard.current == name else " "
            if p:
                diff = max(abs(a - b) for a, b in zip(p["joints_deg"], q))
                put(f"{mark}[{i}] {LABELS[name]:<10} dif {diff:5.1f} grados", (120, 230, 120) if diff < 2 else (230, 230, 230))
                put("     " + " ".join(f"{v:.0f}" for v in p["joints_deg"]), (170, 170, 170), 0.45, 1, 22)
            else:
                put(f"{mark}[{i}] {LABELS[name]:<10} (sin guardar)", (60, 160, 255))
        y += 8
        put("[T] ensenar 3 poses   [ENTER] guardar   [ESC] cancelar", (200, 200, 200), 0.5)
        put("[1][2][3] ir a pose   [O]/[C] pinza   [G] agarrar (MoveIt)", (200, 200, 200), 0.5)
        put(f"[M] calibrar distancia (Medido_cm = {self.measured_cm:.0f})   [Q] salir", (200, 200, 200), 0.5)
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
        elif key in (13, 10):
            with self.lock:
                q, g, fb = list(self.joints_deg), self.gripper_mm, self.feedback_ok
            self.wizard.confirm(q, g, fb)
        elif key in (ord("1"), ord("2"), ord("3")):
            self.go_to(POSES[key - ord("1")])
        elif key in (ord("o"), ord("O")):
            self.gripper(GRIPPER_OPEN_MM)
        elif key in (ord("c"), ord("C")):
            self.gripper(GRIPPER_CLOSE_MM)
        elif key in (ord("g"), ord("G")):
            self.grasp()
        elif key in (ord("m"), ord("M")):
            self.calibrate()
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


def main():
    rclpy.init()
    node = Manipulacion()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1160, 520)
    cv2.createTrackbar("Medido_cm", WINDOW, 24, 80, lambda v: setattr(node, "measured_cm", float(max(3, v))))
    try:
        while rclpy.ok():
            cv2.imshow(WINDOW, node.draw())
            key = cv2.waitKey(30) & 0xFF
            if key != 255 and not node.handle_key(key):
                break
    finally:
        if node.wizard.active:
            node.wizard.cancel()
        node.stop_move_group()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
