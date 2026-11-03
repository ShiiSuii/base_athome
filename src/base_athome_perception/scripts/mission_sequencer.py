#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Secuenciador de misiones de Base@Home (maquina de estados).

  ESPERANDO (en el lugar por defecto)
      - sigue con la cabeza (pan-tilt) la cara mas cercana que ve la RealSense
      - cuando ve a alguien, saluda y le pregunta que quiere; escucha 6 s (Whisper)
      - tambien acepta pedidos escritos en el topico /mission/command (std_msgs/String)
  PEDIDO -> lista de pasos (mission_intents.parse_commands), los confirma en voz alta
  EJECUTANDO paso a paso:
      go_to(waypoint)      Nav2 navigate_to_pose
      receive_guest        saluda, nombre y bebida, memoriza la cara (memoria temporal)
      fetch(objeto)        va al punto de aproximacion del objeto del mapa, PRE-GRASP, agarre MoveIt
      guide(waypoint)      "follow me" y navega
      place                PRE-GRASP (brazo adelante), abre la pinza, HOME
      go_home              vuelve al lugar por defecto
  Al terminar (o si falla un paso) avisa y vuelve a ESPERANDO.

Objetos y waypoints: src/base_athome_navigation/config/semantic_map.yaml (editor_mapa).
  objects: {obj_1: {number: 1, name: pringles, class: can, x, y, z, approach: {x, y, yaw_deg}}}
Lugar por defecto: waypoint 'home' o 'default' del mapa; si no existe, donde arranco el robot.

Uso:  mision            (alias)      |  mision --sim  (sin navegar ni mover el brazo: solo dice/loguea)
      ros2 topic pub --once /mission/command std_msgs/String "{data: 'do the mission'}"
"""
import argparse
import math
import os
import queue
import threading
import time
from typing import Dict, List, Optional, Tuple

os.environ.setdefault("YOLO_OFFLINE", "1")

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Vector3
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from base_athome_perception import scene_analysis as scene
from base_athome_perception.face_recognizer import FaceRecognizer
from base_athome_perception.mission_intents import describe, parse_commands
from base_athome_perception.speaker import RoboCupSpeaker
from base_athome_perception.guest_vocabulary import GuestVocabulary
from base_athome_perception.guest_dialog import parse_yes_no
from base_athome_perception.voice_client import VoiceClient

try:
    from nav2_msgs.action import NavigateToPose
except ImportError:  # pragma: no cover
    NavigateToPose = None

SEMANTIC_MAP = os.path.expanduser("~/base_athome_ws/src/base_athome_navigation/config/semantic_map.yaml")
WAYPOINTS_YAML = os.path.expanduser("~/base_athome_ws/src/base_athome_navigation/config/waypoints.yaml")
PKG = os.path.expanduser("~/base_athome_ws/src/base_athome_perception")


def yaw_to_quat(yaw: float) -> Tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class MissionSequencer(Node):
    IDLE, RUNNING = "ESPERANDO", "EJECUTANDO"

    def __init__(self, sim: bool = False):
        super().__init__("mission_sequencer")
        self.sim = sim
        self.state = self.IDLE
        self.current_step = ""
        self.steps: List = []
        self.commands: "queue.Queue[str]" = queue.Queue()
        self.voice_text: Optional[str] = None
        self.robot_pose: Optional[Tuple[float, float, float]] = None
        self.home_pose: Optional[Tuple[float, float, float]] = None
        self.frame = None
        self.frame_lock = threading.Lock()
        self.last_greet = 0.0
        self.faces: List = []
        self.pan, self.tilt = 0.0, -5.0
        self.guest: Dict = {}

        self.load_map()
        # --- interfaces ---
        self.create_subscription(String, "/mission/command", lambda m: self.commands.put(m.data), 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._pose_cb, 10)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self._image_cb, qos_profile_sensor_data)
        self.pub_status = self.create_publisher(String, "/mission/status", 10)
        self.pub_say = self.create_publisher(String, "/hri/say_something", 10)
        self.pub_mute = self.create_publisher(Bool, "/voice/mute", 10)
        self.pub_pantilt = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 10)
        self.voice = VoiceClient(self)   # escucha con id, fin de habla automatico y resultado explicito
        self.cli_torque = self.create_client(SetBool, "/pan_tilt/set_torque")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose") if NavigateToPose else None
        self.vocab = GuestVocabulary()
        self.create_timer(0.5, lambda: self.pub_status.publish(String(data=f"{self.state}: {self.current_step}")))

        self.speaker = RoboCupSpeaker(piper_bin="/home/athome/piper_bin/piper/piper",
                                      model_path="/home/athome/piper_bin/voices/en/en_US-lessac-medium.onnx",
                                      beep_path=os.path.join(PKG, "resources", "beep.wav"))
        models = os.path.join(PKG, "models")
        self.faces_rec = FaceRecognizer(os.path.join(models, "deploy.prototxt"),
                                        os.path.join(models, "res10_300x300_ssd_iter_140000.caffemodel"),
                                        os.path.join(models, "face_recognition_sface_2021dec.onnx"),
                                        session_id=f"mission_{time.strftime('%Y%m%d_%H%M%S')}")
        self.arm = None
        if not sim:
            from base_athome_perception.arm_manipulation import Manipulacion
            self.arm = Manipulacion()  # nodo de manipulacion headless (lanza MoveIt si hace falta)
        threading.Thread(target=self._face_loop, daemon=True).start()
        threading.Thread(target=self._main_loop, daemon=True).start()
        self.get_logger().info(f"Secuenciador listo ({'SIMULACION' if sim else 'REAL'}). Objetos: "
                               f"{[(o.get('number'), o.get('name')) for o in self.objects.values()]}")

    # ------------------------------------------------------------------ mapa
    def load_map(self):
        data = {}
        try:
            with open(SEMANTIC_MAP, encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except OSError:
            pass
        self.waypoints: Dict[str, Dict] = dict(data.get("waypoints") or {})
        try:
            with open(WAYPOINTS_YAML, encoding="utf-8") as f:
                for k, v in ((yaml.safe_load(f) or {}).get("waypoints") or {}).items():
                    self.waypoints.setdefault(k, v)
        except OSError:
            pass
        self.objects: Dict[str, Dict] = dict(data.get("objects") or {})

    # ----------------------------------------------------------------- ROS cb
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.robot_pose = (p.x, p.y, yaw)
        if self.home_pose is None:
            self.home_pose = self.robot_pose

    def _image_cb(self, msg: Image):
        if msg.encoding not in ("rgb8", "bgr8"):
            return
        img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]
        with self.frame_lock:
            self.frame = img.copy()

    # ------------------------------------------------------------- habla/escucha
    def say(self, text: str):
        self.get_logger().info(f"🗣 {text}")
        self.pub_say.publish(String(data=text))
        self.pub_mute.publish(Bool(data=True))
        try:
            self.speaker.speak(text, blocking=True)
        finally:
            self.pub_mute.publish(Bool(data=False))

    def command_prompt(self) -> str:
        """Vocabulario para Whisper al escuchar ORDENES (no nombres/bebidas)."""
        places = ", ".join(w.replace("_", " ") for w in self.waypoints)
        objs = ", ".join(str(o.get("name", k)).replace("_", " ") for k, o in self.objects.items())
        return (f"Commands for a home robot. Go to the {places or 'kitchen'}. Bring me the {objs or 'pringles'}. "
                "Bring me object number one. Pick up the object. Receive the guest. Go home. Do the full mission.")

    def listen(self, prompt: Optional[str] = None, asr_prompt: str = "", tries: int = 2) -> Optional[str]:
        """Dice `prompt`, beep y escucha (corta solo cuando la persona termina). Devuelve el texto o None."""
        for attempt in range(tries):
            if prompt:
                self.say(prompt if attempt == 0 else "I could not hear you. Please come closer and say it again "
                                                     "after the beep.")
            self.speaker.play_beep(blocking=True)
            self.pub_mute.publish(Bool(data=False))
            res = self.voice.listen(prompt=asr_prompt or self.vocab.whisper_prompt())
            if res.ok:
                self.get_logger().info(f"👂 {res.text}")
                return res.text
            self.get_logger().warn(f"👂 sin texto: {res.status} {res.reason}")
            if res.status in ("offline", "error"):
                self.say("My microphone is not working right now.")
                return None
        return None

    def confirm(self, question: str) -> bool:
        """Pregunta si/no. Si no se entiende la respuesta, se asume que si (no bloquear la mision)."""
        text = self.listen(f"{question} Please say yes or no after the beep.",
                           asr_prompt="Answer yes or no. Yes. No. Yes, that is correct. No, that is wrong.")
        ans = parse_yes_no(text)
        return ans is not False

    # ---------------------------------------------------------- cabeza / caras
    def _head(self, pan: float, tilt: float):
        if self.sim:
            return
        if self.cli_torque.service_is_ready() and not getattr(self, "_torque_on", False):
            self.cli_torque.call_async(SetBool.Request(data=True))
            self._torque_on = True
        self.pan, self.tilt = float(np.clip(pan, -60, 60)), float(np.clip(tilt, -25, 25))
        self.pub_pantilt.publish(Vector3(x=self.pan, y=self.tilt, z=0.0))

    def _face_loop(self):
        while rclpy.ok():
            with self.frame_lock:
                frame = None if self.frame is None else self.frame.copy()
            if frame is None:
                time.sleep(0.1)
                continue
            try:
                faces = self.faces_rec.detect_faces(frame)
            except Exception:
                faces = []
            self.faces = faces
            if faces and self.state == self.IDLE:
                x, y, w, h, _ = max(faces, key=lambda f: f[2] * f[3])
                ih, iw = frame.shape[:2]
                fx = iw / (2.0 * math.tan(math.radians(34.5)))
                dpan, dtilt = scene.face_tracking_step(x + w / 2, y + h / 2, iw, ih, fx, fx)
                if dpan or dtilt:
                    self._head(self.pan + dpan, self.tilt + dtilt)
            time.sleep(0.1)

    def person_in_front(self) -> bool:
        return any(w * h > 60 * 60 for (_, _, w, h, _) in self.faces)

    # ---------------------------------------------------------------- bucle
    def _main_loop(self):
        time.sleep(3.0)
        self.say("Mission sequencer ready.")
        while rclpy.ok():
            text = None
            try:
                text = self.commands.get(timeout=0.2)
            except queue.Empty:
                if self.state == self.IDLE and self.person_in_front() and time.time() - self.last_greet > 25.0:
                    self.last_greet = time.time()
                    text = self.listen("Hello! What can I do for you? After the beep, tell me your request.",
                                       asr_prompt=self.command_prompt())
                    from_voice = True
            else:
                from_voice = False
            if not text:
                continue
            steps = parse_commands(text, self.objects, list(self.waypoints))
            if not steps:
                self.say("Sorry, I did not understand the request.")
                continue
            if from_voice and not self.confirm(f"I understood: {describe(steps, self.objects)}. Is that right?"):
                self.say("Okay, I will not do it. Tell me again when you are ready.")
                self.last_greet = time.time() - 15.0
                continue
            self.say(f"Okay. I will {describe(steps, self.objects)}.")
            self.run_steps(steps)

    def run_steps(self, steps):
        self.state, self.steps = self.RUNNING, list(steps)
        try:
            for kind, arg in steps:
                self.current_step = f"{kind} {arg or ''}".strip()
                ok = getattr(self, f"step_{kind}")(arg)
                if not ok:
                    self.say(f"I could not complete the step: {describe([(kind, arg)], self.objects)}.")
                    break
            else:
                self.say("Mission completed.")
        finally:
            self.state, self.current_step, self.steps = self.IDLE, "", []
            self.last_greet = time.time()

    # ---------------------------------------------------------------- pasos
    def _navigate(self, x: float, y: float, yaw: float) -> bool:
        self.get_logger().info(f"Navegando a ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f} grados)")
        if self.sim:
            time.sleep(1.0)
            return True
        if self.nav is None or not self.nav.wait_for_server(timeout_sec=5.0):
            self.say("Navigation is not available.")
            return False
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_to_quat(yaw)
        fut = self.nav.send_goal_async(goal)
        while not fut.done():
            time.sleep(0.05)
        handle = fut.result()
        if not handle or not handle.accepted:
            return False
        res = handle.get_result_async()
        t0 = time.time()
        while not res.done():
            if time.time() - t0 > 180.0:
                handle.cancel_goal_async()
                return False
            time.sleep(0.1)
        return res.result().status == 4  # STATUS_SUCCEEDED

    def _waypoint(self, name: str) -> Optional[Tuple[float, float, float]]:
        wp = self.waypoints.get(name)
        if not wp:
            return None
        return float(wp["x"]), float(wp["y"]), math.radians(float(wp.get("yaw_deg", 0.0)))

    def step_go_to(self, name) -> bool:
        p = self._waypoint(name)
        if p is None:
            self.say(f"I do not know where the {name.replace('_', ' ')} is.")
            return False
        return self._navigate(*p)

    def step_go_home(self, _=None) -> bool:
        p = self._waypoint("home") or self._waypoint("default") or self.home_pose
        if p is None:
            return True
        return self._navigate(*p)

    def step_receive_guest(self, _=None) -> bool:
        from base_athome_perception.guest_dialog import extract_name_and_drink
        self._head(0.0, 0.0)
        self.say("Hello! Welcome. I am Robot.")
        name = drink = None
        for _ in range(3):
            text = self.listen("After the beep, please say: My name is, your name, and my favorite drink is, your drink.")
            if text:
                n, d = extract_name_and_drink(text, self.vocab)
                name, drink = name or n, drink or d
            if name and drink:
                break
        self.guest = {"name": name or "guest", "drink": drink or "unknown"}
        self.say(f"Nice to meet you, {self.guest['name']}. Please look at me for a moment.")
        feats, t_end = [], time.time() + 4.0
        while time.time() < t_end and len(feats) < 8:
            with self.frame_lock:
                frame = None if self.frame is None else self.frame.copy()
            if frame is not None:
                for d in self.faces_rec.detect_and_recognize(frame):
                    if d["feature"] is not None and d["quality_valid"]:
                        feats.append(d["feature"])
            time.sleep(0.2)
        if feats and name:
            self.faces_rec.enroll_person(name, feats, attributes={"favorite_drink": drink, "role": "guest"})
        self.say("Thank you. Please wait here for a moment.")
        return True

    def _object_approach(self, key) -> Optional[Tuple[float, float, float]]:
        o = self.objects.get(key) or {}
        a = o.get("approach")
        if a:
            return float(a["x"]), float(a["y"]), math.radians(float(a.get("yaw_deg", 0.0)))
        if "x" in o and self.robot_pose:  # 0.6 m del objeto, del lado del robot
            ox, oy = float(o["x"]), float(o["y"])
            rx, ry, _ = self.robot_pose
            d = math.hypot(rx - ox, ry - oy) or 1.0
            ax, ay = ox + 0.6 * (rx - ox) / d, oy + 0.6 * (ry - oy) / d
            return ax, ay, math.atan2(oy - ay, ox - ax)
        return None

    def step_fetch(self, key) -> bool:
        name = self.objects.get(key, {}).get("name", key)
        app = self._object_approach(key)
        if app is None:
            self.say(f"I do not know where the {name} is. Please add it in the map editor.")
            return False
        if not self._navigate(*app):
            return False
        if self.sim:
            self.say(f"Simulation: I would grasp the {name} now.")
            return True
        arm = self.arm
        arm.go_to("pre_grasp")
        if not self._wait_arm(25.0):
            return False
        t_end = time.time() + 6.0
        while arm.detection is None and time.time() < t_end:
            time.sleep(0.1)
        if arm.detection is None:
            self.say(f"I cannot see the {name}.")
            return False
        arm.grasp()
        if not self._wait_arm(60.0, start_timeout=3.0):
            return False
        held = 8.0 < arm.gripper_mm < 95.0  # pinza no cerro del todo -> hay algo entre los dedos
        self.say(f"I have the {name}." if held else f"I am not sure I grabbed the {name}.")
        return True

    def _wait_arm(self, timeout_s: float, start_timeout: float = 1.0) -> bool:
        t0 = time.time()
        while not self.arm.busy and time.time() - t0 < start_timeout:
            time.sleep(0.05)
        while self.arm.busy and time.time() - t0 < timeout_s:
            time.sleep(0.1)
        return not self.arm.busy

    def step_guide(self, name) -> bool:
        guest = self.guest.get("name", "")
        self.say(f"{guest + ', ' if guest else ''}please follow me to the {name.replace('_', ' ')}.")
        ok = self.step_go_to(name)
        if ok:
            self.say(f"We have arrived at the {name.replace('_', ' ')}.")
        return ok

    def step_place(self, _=None) -> bool:
        if self.sim:
            self.say("Simulation: I would release the object now.")
            return True
        self.arm.go_to("pre_grasp")
        self._wait_arm(25.0)
        self.arm.gripper(105.0)
        time.sleep(1.5)
        self.say("Here you are.")
        self.arm.go_to("home")
        self._wait_arm(25.0)
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true", help="no navega ni mueve el brazo (solo habla y loguea)")
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = MissionSequencer(sim=args.sim)
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    if node.arm is not None:
        ex.add_node(node.arm)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.arm is not None:
            node.arm.stop_move_group()
        node.speaker.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
