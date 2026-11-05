#!/usr/bin/env python3
"""Ve a una persona senalando y ubica la bolsa con la RealSense de la cabeza. Mueve solo la cabeza.

  sub /bag_finder/request  String JSON {id, timeout_s}
  pub /bag_finder/result   String JSON {id, ok, bag:[x,y,z], pointed:[x,y]} o {id, ok:false, error, message}
z es la parte superior de la bolsa (donde esta la manija). Marco map. Los calculos estan en base_athome_skills.mission.bag_task.
Detector de bolsas: YOLO-World (wrist_detector.BagDetector). La camara de muneca usa el mismo para el agarre.
"""
import json
import math
import os
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Vector3
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from base_athome_perception import scene_analysis as scene
from base_athome_perception.wrist_detector import BagDetector
from base_athome_skills.mission import bag_task

MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")
L_SHOULDER, R_SHOULDER, L_WRIST, R_WRIST, L_HIP, R_HIP = 5, 6, 9, 10, 11, 12
MIN_KP_CONF = 0.4
FRAME = "camera_color_optical_frame"
SEARCH_PANS = (0.0, 30.0, -30.0, 60.0, -60.0)
SEARCH_TILT = 0.0
SEARCH_VIEW_S = 1.5


def deproject(u, v, depth_m, fx, fy, cx, cy):
    return np.array([(u - cx) * depth_m / fx, (v - cy) * depth_m / fy, depth_m])


def depth_at(depth, u, v, radius=4):
    """Mediana de la profundidad valida alrededor del pixel, en metros (None si no hay)."""
    h, w = depth.shape[:2]
    u, v = int(round(u)), int(round(v))
    if not (0 <= u < w and 0 <= v < h):
        return None
    patch = depth[max(0, v - radius):v + radius + 1, max(0, u - radius):u + radius + 1]
    valid = patch[patch > 0]
    return float(np.median(valid)) / 1000.0 if valid.size else None


class BagFinder(Node):
    def __init__(self):
        super().__init__("bag_finder")
        self.lock = threading.Lock()
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
        from ultralytics import YOLO
        self.pose_model = YOLO(os.path.join(MODELS, "yolov8n-pose.pt"))
        self.bags = BagDetector(MODELS)
        if not self.bags.available():
            raise SystemExit(f"Faltan los pesos del detector de bolsas: {self.bags.weights}")
        self.bags.load()
        blank = np.zeros((480, 640, 3), np.uint8)
        self.pose_model.predict(blank, verbose=False, device="cpu")
        self.bags.detect(blank)
        self.get_logger().info("bag_finder listo; detector de bolsas: YOLO-World")

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

    def _ensure_torque(self):
        """Sin torque el nodo ignora los comandos de la cabeza."""
        if self.cli_torque.wait_for_service(timeout_sec=2.0):
            self.cli_torque.call_async(SetBool.Request(data=True))
            time.sleep(0.5)
        else:
            self.get_logger().warn("Sin /pan_tilt/set_torque: la cabeza no va a poder mirar la bolsa")

    def _info(self, m):
        self.K = (m.k[0], m.k[4], m.k[2], m.k[5])

    def _to_map(self, xyz):
        p = PointStamped()
        p.header.frame_id = FRAME
        p.point.x, p.point.y, p.point.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        try:
            out = self.tf_buf.transform(p, "map", timeout=rclpy.duration.Duration(seconds=0.3))
        except Exception:  # noqa: BLE001 - sin TF no hay medicion
            return None
        return [out.point.x, out.point.y, out.point.z]

    def _frames(self):
        with self.lock:
            return (None, None) if self.rgb is None or self.depth is None else (self.rgb.copy(), self.depth.copy())

    def _kp_map(self, kp, conf, i, depth):
        if conf[i] < MIN_KP_CONF or self.K is None:
            return None
        z = depth_at(depth, kp[i][0], kp[i][1])
        return None if z is None else self._to_map(deproject(kp[i][0], kp[i][1], z, *self.K))

    def _poses(self, rgb):
        res = self.pose_model.predict(rgb, conf=0.45, verbose=False, device="cpu")[0]
        if res.keypoints is None or res.keypoints.conf is None or res.boxes is None:
            return []
        out = [(box, kp, conf) for box, kp, conf in zip(res.boxes.xyxy.cpu().numpy(), res.keypoints.xy.cpu().numpy(),
                                                        res.keypoints.conf.cpu().numpy())]
        return sorted(out, key=lambda p: -(p[0][2] - p[0][0]) * (p[0][3] - p[0][1]))     # la mas cercana primero

    @staticmethod
    def _torso_pixel(person):
        """Centro del torso: al centrarlo entran los brazos y las caderas que hacen falta para leer el gesto."""
        box, kp, conf = person
        pts = [kp[i] for i in (L_SHOULDER, R_SHOULDER, L_HIP, R_HIP) if conf[i] >= MIN_KP_CONF]
        if pts:
            return float(np.mean([p[0] for p in pts])), float(np.mean([p[1] for p in pts]))
        return float((box[0] + box[2]) / 2), float((box[1] + box[3]) / 2)

    def _track(self, person, img_w, img_h):
        """Un paso de seguimiento: lleva el torso de la persona al centro de la imagen."""
        if self.K is None:
            return
        u, v = self._torso_pixel(person)
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

    def _pointed_by(self, people, depth):
        """Punto del piso que senala alguna de las personas, o None."""
        for _, kp, conf in people:
            for sh, wr, hip in ((L_SHOULDER, L_WRIST, L_HIP), (R_SHOULDER, R_WRIST, R_HIP)):
                s, w, h = self._kp_map(kp, conf, sh, depth), self._kp_map(kp, conf, wr, depth), self._kp_map(kp, conf, hip, depth)
                if s and w and h and bag_task.is_pointing(s, w, h):
                    target = bag_task.pointed_floor_point(s, w)
                    if target:
                        return target
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
            people = self._poses(rgb) if rgb is not None else []
            if people:
                self._track(people[0], rgb.shape[1], rgb.shape[0])
                u, v = self._torso_pixel(people[0])
                z = depth_at(depth, u, v) if self.K is not None else None
                spot = self._to_map(deproject(u, v, z, *self.K)) if z else None
                if spot:
                    spots.append(spot)
            time.sleep(0.15)
        if not spots:
            return dict(fail, message="veo a alguien, sin distancia valida")
        person = [float(np.median([s[0] for s in spots])), float(np.median([s[1] for s in spots]))]
        self.get_logger().info(f"Persona en ({person[0]:.2f}, {person[1]:.2f})")
        return dict(id=req.get("id"), ok=True, person=person)

    def _bags(self, rgb, depth):
        """Bolsas de la imagen: [x, y, z superior] en map."""
        out = []
        for b, _, _ in self.bags.detect(rgb):
            cu, cv = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
            z = depth_at(depth, cu, cv)
            if z is None or self.K is None:
                continue
            center = self._to_map(deproject(cu, cv, z, *self.K))
            top = self._to_map(deproject(cu, b[1], z, *self.K))
            if center and top:
                out.append([center[0], center[1], max(0.05, top[2])])
        return out

    def _look_at(self, xy):
        """Lleva la cabeza a mirar el punto del piso, usando la pose actual de la cabeza."""
        here = self._to_map([0.0, 0.0, 0.0])
        ahead = self._to_map([0.0, 0.0, 1.0])
        if not here or not ahead:
            return
        now_bearing = math.atan2(ahead[1] - here[1], ahead[0] - here[0])
        want = math.atan2(xy[1] - here[1], xy[0] - here[0])
        dpan = math.degrees(math.atan2(math.sin(want - now_bearing), math.cos(want - now_bearing)))
        pitch_now = math.degrees(math.atan2(ahead[2] - here[2], math.hypot(ahead[0] - here[0], ahead[1] - here[1])))
        pitch_want = math.degrees(math.atan2(-here[2], math.hypot(xy[0] - here[0], xy[1] - here[1])))
        self._send_head(self.head[0] + dpan, self.head[1] + pitch_want - pitch_now)

    def _request(self, msg):
        threading.Thread(target=self._serve, args=(json.loads(msg.data),), daemon=True).start()

    def _serve(self, req):
        out = self._locate_person(req) if req.get("mode") == "locate_person" else self._find(req)
        self.pub.publish(String(data=json.dumps(out)))

    def _find(self, req):
        """Sigue a la persona mientras espera el gesto, y despues mira donde senala y busca la bolsa."""
        end = time.time() + float(req.get("timeout_s", 30.0))
        out = dict(id=req.get("id"), ok=False, error="not_pointing", message="no vi a nadie senalando")
        self._ensure_torque()
        hits = []
        while time.time() < end and len(hits) < 3:        # tres vistas coherentes: un gesto, no un brazo de paso
            rgb, depth = self._frames()
            people = self._poses(rgb) if rgb is not None else []
            if people:
                self._track(people[0], rgb.shape[1], rgb.shape[0])
                target = self._pointed_by(people, depth)
                if target:
                    hits.append(target)
            time.sleep(0.15)
        if len(hits) == 3:
            target = (float(np.median([h[0] for h in hits])), float(np.median([h[1] for h in hits])))
            self.get_logger().info(f"Senala al piso en ({target[0]:.2f}, {target[1]:.2f})")
            self._look_at(target)
            time.sleep(1.5)
            out = dict(id=req.get("id"), ok=False, error="bag_not_found", message="no veo una bolsa donde senala")
            while time.time() < end:
                rgb, depth = self._frames()
                bag = bag_task.choose_bag(target, self._bags(rgb, depth)) if rgb is not None else None
                if bag:
                    self.get_logger().info(f"Bolsa en {np.round(bag, 2).tolist()}")
                    out = dict(id=req.get("id"), ok=True, bag=[float(v) for v in bag], pointed=list(target))
                    break
                time.sleep(0.2)
        return out


def main():
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
