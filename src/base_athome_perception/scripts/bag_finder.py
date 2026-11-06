#!/usr/bin/env python3
"""Ve a una persona senalando y ubica la bolsa con la RealSense de la cabeza. Mueve solo la cabeza.

  sub /bag_finder/request  String JSON {id, timeout_s, mode: locate_person | refine_bag (con near:[x,y]) | la bolsa senalada}
  pub /bag_finder/result   String JSON {id, ok, bag:[x,y,z], pointed:[x,y]} o {id, ok:false, error, message}
z es la parte superior de la bolsa (donde esta la manija). Marco map. Las mediciones estan en base_athome_perception.bag_vision.
Detector de bolsas: YOLO-World (wrist_detector.BagDetector), el mismo de la muneca. Con ATHOME_SIM=1 corre todo este codigo
contra la imagen dibujada por la simulacion, sin modelos: la persona llega con sus keypoints por /sim/people.
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from base_athome_perception import bag_vision as bv
from base_athome_perception import scene_analysis as scene
from base_athome_perception.wrist_detector import make_bag_detector
from base_athome_skills.mission import bag_task

MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")
SIM = os.environ.get("ATHOME_SIM") == "1"
FRAME = "camera_color_optical_frame"
SEARCH_PANS = (0.0, 30.0, -30.0, 60.0, -60.0)
SEARCH_TILT = 0.0
SEARCH_VIEW_S = 1.5
LOST_PERSON_S = 1.5
GESTURE_MAX_JUMP_M = 0.8      # el gesto humano tiene 8-15 grados de error: entre vistas salta decimetros
BAG_VIEWS = 3                 # mediciones coherentes de la bolsa antes de creerla
BAG_MAX_SPREAD_M = 0.10


class BagFinder(Node):
    def __init__(self):
        super().__init__("bag_finder")
        self.lock = threading.Lock()
        self.serve_lock = threading.Lock()
        self.rgb, self.depth, self.K = None, None, None
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self._rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self._depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self._info, qos_profile_sensor_data)
        self.create_subscription(String, "/bag_finder/request", self._request, 5)
        self.pub = self.create_publisher(String, "/bag_finder/result", 5)
        self.pub_head = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)
        self.head = (0.0, 0.0)                 # posicion MEDIDA, en grados
        self.cli_torque = self.create_client(SetBool, "/pan_tilt/set_torque")
        self.create_subscription(JointState, "/pan_tilt/joint_states", self._head_state, 10)
        self.head_speed, self._head_prev = 0.0, None     # grados por segundo, medidos
        self.sim_people = []
        self.pose_model = None
        self.bags = make_bag_detector(MODELS, wrist=False)
        if SIM:
            self.create_subscription(String, "/sim/people", self._sim_people, 5)
        else:
            from ultralytics import YOLO
            self.pose_model = YOLO(os.path.join(MODELS, "yolov8n-pose.pt"))
            if not self.bags.available():
                raise SystemExit(f"Faltan los pesos del detector de bolsas: {self.bags.weights}")
            self.bags.load()
            blank = np.zeros((480, 640, 3), np.uint8)
            self.pose_model.predict(blank, verbose=False, device="cpu")
            self.bags.detect(blank)
        self.get_logger().info("bag_finder listo" + (" (simulacion)" if SIM else "; detector de bolsas: YOLO-World"))

    def _sim_people(self, m):
        self.sim_people = json.loads(m.data)["people"]

    def _rgb(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        with self.lock:
            self.rgb = img[:, :, ::-1].copy() if m.encoding == "rgb8" else img.copy()

    def _depth(self, m):
        with self.lock:
            self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()

    def _head_state(self, m):
        d = dict(zip(m.name, m.position))
        if "pan_joint" in d and "tilt_joint" in d:
            self.head = (math.degrees(d["pan_joint"]), math.degrees(d["tilt_joint"]))
            now = time.time()
            if self._head_prev is None:
                self._head_prev = (now, self.head)
            elif now - self._head_prev[0] >= 0.2:
                t0, h0 = self._head_prev
                self.head_speed = math.hypot(self.head[0] - h0[0], self.head[1] - h0[1]) / (now - t0)
                self._head_prev = (now, self.head)

    def _head_still(self):
        """Con la cabeza en movimiento la imagen y el TF no son del mismo instante: esa medicion se descarta."""
        return self.head_speed <= bv.HEAD_STILL_DPS

    def _ensure_torque(self):
        """Sin torque el nodo ignora los comandos de la cabeza."""
        if self.cli_torque.wait_for_service(timeout_sec=2.0):
            self.cli_torque.call_async(SetBool.Request(data=True))
            time.sleep(0.5)
        else:
            self.get_logger().warn("Sin /pan_tilt/set_torque: la cabeza no va a poder mirar la bolsa")

    def _info(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _cam(self):
        """Posicion y ejes de la camara en map, o None sin TF."""
        try:
            t = self.tf_buf.lookup_transform("map", FRAME, rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001 - sin TF no hay medicion
            return None
        return np.array([t.translation.x, t.translation.y, t.translation.z]), bv.quat_to_R(t.rotation)

    def _frames(self):
        with self.lock:
            return (None, None) if self.rgb is None or self.depth is None else (self.rgb.copy(), self.depth.copy())

    def _poses(self, rgb, cam=None):
        if SIM:
            cam = cam or self._cam()
            if cam is None or self.K is None:
                return []
            return bv.project_people(self.sim_people, cam[0], cam[1], self.K, rgb.shape[1], rgb.shape[0])
        return bv.yolo_people(self.pose_model.predict(rgb, conf=0.45, verbose=False, device="cpu")[0])

    def _track(self, person, img_w, img_h):
        """Un paso de seguimiento: lleva el torso de la persona al centro de la imagen."""
        if self.K is None:
            return
        u, v = bv.torso_pixel(person)
        dpan, dtilt = scene.face_tracking_step(u, v, img_w, img_h, self.K[0], self.K[1], gain=0.6, deadband_deg=3.0, max_step_deg=8.0)
        if dpan or dtilt:
            self._send_head(self.head[0] + dpan, self.head[1] + dtilt)

    def _send_head(self, pan, tilt):
        self.pub_head.publish(Vector3(x=max(-65.0, min(65.0, float(pan))), y=max(-35.0, min(25.0, float(tilt))), z=0.0))

    def _search_person(self, end):
        """Busca una persona barriendo con la cabeza. Devuelve (persona, rgb, depth) o None."""
        self._ensure_torque()
        for pan in SEARCH_PANS:
            self._send_head(pan, SEARCH_TILT)
            view_end = min(end, time.time() + SEARCH_VIEW_S)
            while time.time() < view_end:
                rgb, depth = self._frames()
                people = self._poses(rgb) if rgb is not None else []
                if people:
                    return people[0], rgb, depth
                time.sleep(0.1)
            if time.time() >= end:
                break
        return None

    def _locate_person(self, req):
        """Encuentra a la persona, la centra y devuelve donde esta en el mapa."""
        end = time.time() + float(req.get("timeout_s", 30.0))
        fail = dict(id=req.get("id"), ok=False, error="no_person", message="no veo a nadie")
        found = self._search_person(end)
        if found is None:
            return fail
        spots = []
        while time.time() < end and len(spots) < 5:
            rgb, depth = self._frames()
            cam = self._cam()
            people = self._poses(rgb, cam) if rgb is not None else []
            if people:
                self._track(people[0], rgb.shape[1], rgb.shape[0])
                if cam is not None and self.K is not None and self._head_still():
                    spot = bv.person_map_position(people[0], depth, cam[0], cam[1], self.K)
                    if spot:
                        spots.append(spot)
            time.sleep(0.15)
        if not spots:
            return dict(fail, message="veo a alguien, sin distancia valida")
        person = [float(np.median([s[0] for s in spots])), float(np.median([s[1] for s in spots]))]
        self.get_logger().info(f"Persona en ({person[0]:.2f}, {person[1]:.2f})")
        return dict(id=req.get("id"), ok=True, person=person)

    def _look_at(self, xy):
        """Lleva la cabeza a mirar el punto del piso, usando la pose actual de la cabeza."""
        cam = self._cam()
        if cam is None:
            return
        here, ahead = cam[0], cam[0] + cam[1][:, 2]
        now_bearing = math.atan2(ahead[1] - here[1], ahead[0] - here[0])
        want = math.atan2(xy[1] - here[1], xy[0] - here[0])
        dpan = math.degrees(math.atan2(math.sin(want - now_bearing), math.cos(want - now_bearing)))
        pitch_now = math.degrees(math.atan2(ahead[2] - here[2], math.hypot(ahead[0] - here[0], ahead[1] - here[1])))
        pitch_want = math.degrees(math.atan2(-here[2], math.hypot(xy[0] - here[0], xy[1] - here[1])))
        self._send_head(self.head[0] + dpan, self.head[1] + pitch_want - pitch_now)

    def _request(self, msg):
        threading.Thread(target=self._serve, args=(json.loads(msg.data),), daemon=True).start()

    def _serve(self, req):
        with self.serve_lock:      # un pedido por vez: dos hilos mandando la cabeza la harian oscilar
            self._serve_one(req)

    def _serve_one(self, req):
        mode = req.get("mode")
        out = self._locate_person(req) if mode == "locate_person" else self._refine(req) if mode == "refine_bag" else self._find(req)
        self.pub.publish(String(data=json.dumps(out)))

    def _wait_gesture(self, end):
        """Sigue a la persona hasta ver tres veces el gesto. (punto del piso, motivo si no)."""
        hits, why = [], "no vi a nadie"
        last_seen = time.time()
        while time.time() < end and len(hits) < 3:        # tres vistas coherentes: un gesto, no un brazo de paso
            rgb, depth = self._frames()
            cam = self._cam()
            people = self._poses(rgb, cam) if rgb is not None else []
            if people:
                last_seen = time.time()
                self._track(people[0], rgb.shape[1], rgb.shape[0])
            elif time.time() - last_seen > LOST_PERSON_S:    # p. ej. un reintento: la cabeza sigue mirando al piso
                self._search_person(end)
                last_seen = time.time()
                continue
            if cam is not None and self.K is not None and rgb is not None and self._head_still():
                target, reason = bv.pointed_target(people, depth, cam[0], cam[1], self.K)
                if target:
                    # El gesto vale si las vistas coinciden: uno que salta de lugar es otra cosa
                    if hits and math.hypot(target[0] - hits[-1][0], target[1] - hits[-1][1]) > GESTURE_MAX_JUMP_M:
                        hits = []
                    hits.append(target)
                else:
                    why = reason
            time.sleep(0.15)
        if len(hits) < 3:
            return None, why
        return (float(np.median([h[0] for h in hits])), float(np.median([h[1] for h in hits]))), ""

    def _find_bag(self, target, end, gate_m=bag_task.MAX_BAG_FROM_RAY_M, allow_clipped=True):
        """Junta mediciones de la bolsa cerca del punto dado. (creencia o None, motivo si no)."""
        belief, why = bv.BagBelief(), "no ve una bolsa"
        while time.time() < end:
            rgb, depth = self._frames()
            cam = self._cam()
            if rgb is not None and cam is not None and self.K is not None and self._head_still():
                measures = [bv.head_measure(d[0], depth, cam[0], cam[1], self.K, allow_clipped) for d in self.bags.detect(rgb)]
                best, why = bv.choose_nearest(measures, target, gate_m=gate_m)
                if best is not None:
                    belief.update(measures[best])
                if belief.count >= BAG_VIEWS and belief.spread() <= BAG_MAX_SPREAD_M:
                    return belief, ""
            time.sleep(0.2)
        return None, why

    def _refine(self, req):
        """Ya cerca: vuelve a medir la bolsa que se cree en `near`. De cerca la profundidad tiene menos error."""
        end = time.time() + float(req.get("timeout_s", 8.0))
        near = [float(v) for v in req["near"]]
        self._ensure_torque()
        self._look_at(near)
        time.sleep(1.2)
        belief, why = self._find_bag(near, end, gate_m=bv.BELIEF_GATE_M, allow_clipped=False)
        if belief is None:
            return dict(id=req.get("id"), ok=False, error="bag_not_found", message=why)
        return dict(id=req.get("id"), ok=True, bag=[belief.xy[0], belief.xy[1], belief.top_z])

    def _find(self, req):
        """Sigue a la persona mientras espera el gesto, y despues mira donde senala y busca la bolsa."""
        end = time.time() + float(req.get("timeout_s", 30.0))
        self._ensure_torque()
        target, why = self._wait_gesture(end)
        if target is None:
            return dict(id=req.get("id"), ok=False, error="not_pointing", message=f"no vi a nadie senalando: {why}")
        self.get_logger().info(f"Senala al piso en ({target[0]:.2f}, {target[1]:.2f})")
        self._look_at(target)
        time.sleep(1.5)
        belief, why = self._find_bag(target, end)
        if belief is None:
            return dict(id=req.get("id"), ok=False, error="bag_not_found", message=f"no encuentro la bolsa senalada: {why}")
        # Se miro hacia donde senalaba: la bolsa puede estar en el borde de la imagen. Centrarla y medirla entera.
        first = belief.xy
        self._look_at(first)
        time.sleep(1.2)
        centered, _ = self._find_bag(first, min(end, time.time() + 4.0), gate_m=bv.BELIEF_GATE_M, allow_clipped=False)
        if centered is not None:
            belief = centered
        else:
            self.get_logger().warn("No pude ver la bolsa entera: queda la medicion aproximada")
        bag = [belief.xy[0], belief.xy[1], belief.top_z]
        self.get_logger().info(f"Bolsa en {np.round(bag, 2).tolist()} (senalaba {np.round(target, 2).tolist()})")
        return dict(id=req.get("id"), ok=True, bag=bag, pointed=list(target))


def main():
    if not SIM:
        import torch
        torch.set_num_threads(2)      # no quitarle CPU a Nav2 ni a los controladores
    rclpy.init()
    node = BagFinder()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
