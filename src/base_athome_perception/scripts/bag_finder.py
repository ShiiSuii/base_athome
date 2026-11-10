#!/usr/bin/env python3
"""Ve a una persona senalando y ubica la bolsa con la RealSense de la cabeza. Mueve solo la cabeza.

  sub /bag_finder/request  String JSON {id, timeout_s, mode: locate_person | refine_bag (con near:[x,y]) | la bolsa senalada}
  pub /bag_finder/result   String JSON {id, ok, bag:[x,y,z], pointed:[x,y]} o {id, ok:false, error, message}
  mode find_person: pedido {id, mode:"find_person", descriptor:{gesture|posture|clothing: str}, timeout_s}
                    respuesta {id, ok, person:{xy:[x,y]}|null, reason, error}; coordenadas map.
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
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import Buffer, TransformListener

from base_athome_perception import bag_vision as bv
from base_athome_perception import scene_analysis as scene
from base_athome_perception.raised_hand import RaisedHandsAggregator, classify_raised_hand
from base_athome_perception.pose_debug import draw_pose_debug
from base_athome_perception.person_search import PersonSearch, valid_descriptor
from base_athome_perception.person_attributes import extract_person_attributes
from base_athome_perception.wrist_detector import make_bag_detector
from base_athome_skills.mission import bag_task

MODELS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models")
SIM = os.environ.get("ATHOME_SIM") == "1"
FRAME = "camera_color_optical_frame"
SEARCH_PANS = (0.0, 30.0, -30.0, 60.0, -60.0)
RAISED_HANDS_PANS = (0.0, -90.0, 0.0, 90.0, 0.0)
RAISED_HANDS_VIEW_S = 2.0
RAISED_HANDS_SETTLE_S = 1.5
SEARCH_TILT = 0.0
SEARCH_VIEW_S = 1.5
HEAD_SWEEP_DPS = 15.0      # velocidad del barrido de la cabeza: lento y parejo, sin saltos de 30-120 grados
HEAD_STEP_S = 0.1
LOST_PERSON_S = 1.5
GESTURE_MAX_JUMP_M = 0.8      # el gesto humano tiene 8-15 grados de error: entre vistas salta decimetros
BAG_VIEWS = 3                 # mediciones coherentes de la bolsa antes de creerla
BAG_MAX_SPREAD_M = 0.10



def head_ramp(start, goal, dps, step_s):
    """Puntos intermedios de (pan, tilt) en grados, a dps grados por segundo, terminando exactamente en goal."""
    dist = max(abs(goal[0] - start[0]), abs(goal[1] - start[1]))
    steps = max(1, int(math.ceil(dist / max(1e-3, dps * step_s))))
    return [(start[0] + (goal[0] - start[0]) * k / steps, start[1] + (goal[1] - start[1]) * k / steps)
            for k in range(1, steps + 1)]

class BagFinder(Node):
    def __init__(self):
        super().__init__("bag_finder")
        self.lock = threading.Lock()
        self.serve_lock = threading.Lock()
        self.pose_lock = threading.Lock()
        self.rgb, self.depth, self.K = None, None, None
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.create_subscription(Image, "/camera/camera/color/image_raw", self._rgb, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self._depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self._info, qos_profile_sensor_data)
        self.create_subscription(String, "/bag_finder/request", self._request, 5)
        self.pub = self.create_publisher(String, "/bag_finder/result", 5)
        self.pub_people = self.create_publisher(String, "/bag_finder/people", 5)
        self.pub_debug = self.create_publisher(Image, "/bag_finder/debug_image", 2)   # lo que detecta, para RViz
        self.rgb_stamp = None
        self.rgb_received = 0.0
        self.customer_tracking = None
        self.follow_stop = False
        self.pub_head = self.create_publisher(Vector3, "/pan_tilt/target_degrees", 5)
        self.head_stamp = 0.0
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
        # Grupo separado: inferir no bloquea los callbacks de RGB/depth/feedback.
        self.debug_timer = self.create_timer(1.0 / 3.0, self._debug_tick,
                                            callback_group=MutuallyExclusiveCallbackGroup())

    def _sim_people(self, m):
        self.sim_people = json.loads(m.data)["people"]

    def _rgb(self, m):
        img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        with self.lock:
            self.rgb_received = time.monotonic()
            self.rgb_stamp = m.header.stamp
            self.rgb = img[:, :, ::-1].copy() if m.encoding == "rgb8" else img.copy()

    def _depth(self, m):
        with self.lock:
            self.depth = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()

    def _head_state(self, m):
        d = dict(zip(m.name, m.position))
        if "pan_joint" in d and "tilt_joint" in d:
            self.head = (math.degrees(d["pan_joint"]), math.degrees(d["tilt_joint"]))
            now = time.time()
            self.head_stamp = now
            if self._head_prev is None:
                self._head_prev = (now, self.head)
            elif now - self._head_prev[0] >= 0.2:
                t0, h0 = self._head_prev
                self.head_speed = math.hypot(self.head[0] - h0[0], self.head[1] - h0[1]) / (now - t0)
                self._head_prev = (now, self.head)

    def _publish_debug(self, bgr, people, stamp):
        """Imagen con cada persona: esqueleto, caja y si levanta la mano. Solo si alguien la mira (RViz)."""
        if self.pub_debug.get_subscription_count() == 0:
            return
        img = draw_pose_debug(bgr, people, head=self.head)
        msg = Image()
        msg.header.frame_id = "camera_color_optical_frame"
        if stamp is not None:
            msg.header.stamp = stamp
        msg.height, msg.width = img.shape[:2]
        msg.encoding, msg.step = "bgr8", img.shape[1] * 3
        msg.data = img.tobytes()
        self.pub_debug.publish(msg)

    def _debug_tick(self):
        """Pose a ~3 Hz incluso durante barridos, sin depender de profundidad o cabeza quieta."""
        tracking = getattr(self, "customer_tracking", None)
        if tracking is not None and time.monotonic() >= tracking[1]:
            with self.lock:
                if self.customer_tracking == tracking:
                    self.customer_tracking = None
            tracking = None
        if self.pub_debug.get_subscription_count() == 0 and tracking is None:
            return
        try:
            # Solo requiere RGB: no depende de profundidad, TF ni feedback de cabeza.
            with self.lock:
                rgb = None if self.rgb is None else self.rgb.copy()
                stamp = self.rgb_stamp
            if rgb is not None:
                people = self._poses(rgb)
                self._publish_debug(rgb, people, stamp)
                if tracking is not None:
                    self._track_customer(rgb, people, stamp, tracking)
        except Exception as exc:
            self.get_logger().warn(f"No se pudo actualizar debug de pose: {exc}")

    def _track_customer(self, rgb, people, stamp, tracking):
        """Solo cabeza: asociar al cliente por mapa; nunca cambiar al mayor distractor."""
        if (time.monotonic() >= tracking[1] or time.monotonic() - self.rgb_received > 0.5
                or time.time() - self.head_stamp > 0.5):
            return
        if not self.serve_lock.acquire(blocking=False):
            return
        try:
            cam = self._cam(stamp)
            with self.lock:
                depth = self.depth
            if cam is None or depth is None or self.K is None:
                return
            best, distance = None, 0.8
            for person in people:
                xy = bv.person_map_position(person, depth, cam[0], cam[1], self.K)
                if xy is not None:
                    d = math.hypot(xy[0] - tracking[0][0], xy[1] - tracking[0][1])
                    if d < distance:
                        best, distance = person, d
            if best is None:
                return  # Sin cliente identificado, mantener posicion; no barrer.
            u, v = bv.torso_pixel(best)
            if best[2][0] >= 0.4:  # nariz: orientar camara y microfono a la cara
                u, v = best[1][0]
            if not (math.isfinite(u) and math.isfinite(v) and 0 <= u < rgb.shape[1] and 0 <= v < rgb.shape[0]):
                return
            dpan, dtilt = scene.face_tracking_step(
                u, v, rgb.shape[1], rgb.shape[0], self.K[0], self.K[1],
                gain=0.6, deadband_deg=3.0, max_step_deg=2.0)
            with self.lock:
                if self.customer_tracking == tracking and time.monotonic() < tracking[1]:
                    if dpan or dtilt:
                        self._send_head(self.head[0] + dpan, self.head[1] + dtilt)
        finally:
            self.serve_lock.release()

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

    def _cam(self, stamp=None):
        """Posicion y ejes de la camara en map, o None sin TF. Con `stamp`, donde estaba al sacar esa imagen: la cabeza se
        mueve mientras sigue a alguien, y con el TF de ahora una persona a 3 m aparecia a 5. Si no hay TF de ese instante
        (relojes sin sincronizar), el ultimo."""
        try:
            if stamp is not None:
                t = self.tf_buf.lookup_transform("map", FRAME, rclpy.time.Time.from_msg(stamp)).transform
                return np.array([t.translation.x, t.translation.y, t.translation.z]), bv.quat_to_R(t.rotation)
        except Exception:  # noqa: BLE001 - se sigue con el ultimo TF
            pass
        try:
            t = self.tf_buf.lookup_transform("map", FRAME, rclpy.time.Time()).transform
        except Exception:  # noqa: BLE001 - sin TF no hay medicion
            return None
        return np.array([t.translation.x, t.translation.y, t.translation.z]), bv.quat_to_R(t.rotation)

    def _frames(self, with_stamp=False):
        with self.lock:
            frames = (None, None) if self.rgb is None or self.depth is None else (self.rgb.copy(), self.depth.copy())
            return (*frames, self.rgb_stamp) if with_stamp else frames

    def _poses(self, rgb, cam=None):
        if SIM:
            cam = cam or self._cam()
            if cam is None or self.K is None:
                return []
            return bv.project_people(self.sim_people, cam[0], cam[1], self.K, rgb.shape[1], rgb.shape[0])
        # Ultralytics mantiene estado mutable: compartir el modelo exige serializar inferencia.
        with self.pose_lock:
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
        pan, tilt = max(-90.0, min(90.0, float(pan))), max(-35.0, min(25.0, float(tilt)))
        self.head_cmd = (pan, tilt)
        self.pub_head.publish(Vector3(x=pan, y=tilt, z=0.0))

    def _move_head(self, pan, tilt, dps=HEAD_SWEEP_DPS):
        """Lleva la cabeza a (pan, tilt) de a pasos chicos a velocidad constante: barrido suave, sin saltos."""
        start = getattr(self, "head_cmd", None) or (float(self.head[0]), float(self.head[1]))
        for p, t in head_ramp(start, (pan, tilt), dps, HEAD_STEP_S):
            self._send_head(p, t)
            time.sleep(HEAD_STEP_S)

    def _search_person(self, end):
        """Busca una persona barriendo con la cabeza. Devuelve (persona, rgb, depth) o None."""
        self._ensure_torque()
        for pan in SEARCH_PANS:
            self._move_head(pan, SEARCH_TILT)
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
        req = json.loads(msg.data)
        if req.get("mode") == "track_customer":
            xy = req.get("xy")
            valid = (isinstance(xy, (list, tuple)) and len(xy) == 2
                     and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                             and math.isfinite(v) for v in xy))
            with self.lock:
                self.customer_tracking = (tuple(xy), time.monotonic() + 1.5) if valid else None
            return
        if req.get("mode") == "stop_follow":
            self.follow_stop = True
            return
        threading.Thread(target=self._serve, args=(req,), daemon=True).start()

    def _serve(self, req):
        with self.serve_lock:      # un pedido por vez: dos hilos mandando la cabeza la harian oscilar
            self._serve_one(req)

    def _serve_one(self, req):
        mode = req.get("mode")
        if mode == "follow_person":
            self._follow(req)
            return
        if mode == "find_person":
            out = self._find_person(req)
        elif mode == "raised_hands":
            out = self._find_raised_hands(req)
        elif mode == "locate_person":
            out = self._locate_person(req)
        elif mode == "refine_bag":
            out = self._refine(req)
        else:
            out = self._find(req)
        self.pub.publish(String(data=json.dumps(out)))

    def _robot_pos(self):
        """Ubicacion del robot en el frame map (base_footprint o camara como fallback)."""
        try:
            t = self.tf_buf.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
            return float(t.translation.x), float(t.translation.y)
        except Exception:
            pass
        cam = self._cam()
        if cam is not None:
            return float(cam[0][0]), float(cam[0][1])
        return 0.0, 0.0

    def _sim_person_spot(self, person, cam):
        """En simulacion halla la coordenada mapa de un cliente por proyeccion geometrica."""
        if not self.sim_people or cam is None or self.K is None:
            return None
        u, v = bv.torso_pixel(person)
        best_p, best_d = None, 60.0  # tolerancia en pixeles
        for sp in self.sim_people:
            pos = np.array([sp["xy"][0], sp["xy"][1], 1.2])
            c = cam[1].T @ (pos - cam[0])
            if c[2] > 0.1:
                pu = self.K[2] + self.K[0] * c[0] / c[2]
                pv = self.K[3] + self.K[1] * c[1] / c[2]
                d = math.hypot(pu - u, pv - v)
                if d < best_d:
                    best_d = d
                    best_p = sp["xy"]
        if best_p is not None:
            return float(best_p[0]), float(best_p[1])
        return None

    def _find_person(self, req):
        search = PersonSearch(req.get("descriptor"), req.get("min_frames", 3))
        if not valid_descriptor(search.descriptor):
            return search.build_response(req.get("id"))
        return self._sweep_people(req, search=search)

    def _find_raised_hands(self, req):
        return self._sweep_people(req)

    def _sweep_people(self, req, search=None):
        """Barrido comun de Restaurant/GPSR en -60..+60 grados, con posicion mapa."""
        req_id = req.get("id", "raised_hands")
        timeout_s = float(req.get("timeout_s", 20.0))
        min_frames = int(req.get("min_frames", 3))
        head_options = {}
        for name, low, high in (("head_dps", 2.0, 30.0), ("view_s", 0.5, 10.0)):
            if name in req:
                value = req[name]
                if (isinstance(value, bool) or not isinstance(value, (int, float))
                        or not math.isfinite(value) or not low <= value <= high):
                    return {"id": req_id, "ok": False, "customers": [], "error": "invalid_" + name}
                head_options[name] = float(value)
        end = time.time() + timeout_s

        # pans opcional en grados: Restaurant usa [0.0, -90.0, 0.0, 90.0, 0.0] (centro, 90 der, centro, 90 izq, centro).
        # Validar antes de tocar la cabeza; sin el campo se conserva el barrido por defecto.
        sweep_pans = req.get("pans", RAISED_HANDS_PANS) if search is None else RAISED_HANDS_PANS
        if search is None and "pans" in req and (not isinstance(sweep_pans, (list, tuple)) or not sweep_pans
                or any(isinstance(p, bool) or not isinstance(p, (int, float))
                       or not math.isfinite(p) or not -90.0 <= p <= 90.0 for p in sweep_pans)):
            return {"id": req_id, "ok": False, "customers": [], "error": "invalid_pans"}
        self._ensure_torque()
        aggregator = RaisedHandsAggregator(min_frames=min_frames, match_dist_m=0.8) if search is None else None

        fixed_head = search is None and "pans" in req and len(sweep_pans) == 1
        seen = dict(frames=0, people=0, raised=0, mapped_raised=0, estimated_raised=0,
                    no_tf=0, no_depth=0, no_intrinsics=0, invalid_position=0)
        pan_idx = 0
        last_stamp = None
        while time.time() < end:
            pan = sweep_pans[pan_idx % len(sweep_pans)]
            pan_idx += 1
            if "head_dps" in head_options:
                self._move_head(pan, SEARCH_TILT, dps=head_options["head_dps"])
            else:
                self._move_head(pan, SEARCH_TILT)

            settle_end = min(end, time.time() + RAISED_HANDS_SETTLE_S)
            while time.time() < settle_end:
                # Con la cabeza fija (pedido con "pans", la base es la que gira) no se espera llegar: alcanza con que este quieta.
                if (time.time() - self.head_stamp < 0.5 and self._head_still()
                        and (fixed_head or (abs(self.head[0] - pan) < 3.0 and abs(self.head[1] - SEARCH_TILT) < 3.0))):
                    break
                time.sleep(0.05)
            else:
                # La cabeza no llego (pan-tilt trabado o sin torque). Si hay feedback fresco y esta quieta, se mira igual
                # desde donde esta: el TF usa la posicion medida, asi que la proyeccion sigue siendo correcta.
                if time.time() - self.head_stamp < 0.5 and self._head_still():
                    self.get_logger().warn(f"Barrido: la cabeza no llego a pan={pan:.0f} (esta en pan={self.head[0]:.0f}, "
                                           f"tilt={self.head[1]:.0f}); miro desde ahi")
                else:
                    self.get_logger().warn(f"Barrido: sin feedback fresco en pan={pan}")
                    continue
            view_end = min(end, time.time() + head_options.get("view_s", RAISED_HANDS_VIEW_S))
            # Solo imagenes nuevas despues de llegar; una imagen congelada no cuenta
            # de nuevo al cambiar de pan ni se proyecta con el TF de otra vista.
            last_stamp = self.rgb_stamp

            while time.time() < view_end:
                if self._head_still() and time.time() - self.head_stamp < 0.5:
                    rgb, depth, stamp = self._frames(with_stamp=True)
                    if stamp is None or stamp == last_stamp:
                        time.sleep(0.12)
                        continue
                    last_stamp = stamp
                    cam = self._cam(stamp)
                    if rgb is not None:
                        people = self._poses(rgb, cam)
                        seen["frames"] += 1
                        seen["people"] = max(seen["people"], len(people))
                        seen["raised"] += sum(1 for p in people if classify_raised_hand(p[1], p[2]).is_raised)
                        candidates = []
                        for person in people:
                            is_raised = classify_raised_hand(person[1], person[2]).is_raised
                            # En simulacion los puntos del cuerpo no coinciden con el modelo dibujado: la profundidad del
                            # torso caia en la pared de atras (~2x de distancia). Ahi se usa la posicion de la persona simulada.
                            spot = self._sim_person_spot(person, cam) if SIM else None
                            estimated = False
                            if spot is None and depth is not None and cam is not None and self.K is not None:
                                spot = bv.person_map_position(person, depth, cam[0], cam[1], self.K)
                            if spot is None and search is None and is_raised and cam is not None and self.K is not None:
                                if hasattr(bv, "person_estimated_map_position"):
                                    spot = bv.person_estimated_map_position(person, depth, cam[0], cam[1], self.K)
                                    estimated = spot is not None
                            if is_raised:
                                if spot is not None:
                                    seen["mapped_raised"] += 1
                                    if estimated:
                                        seen["estimated_raised"] += 1
                                elif cam is None:
                                    seen["no_tf"] += 1
                                elif depth is None:
                                    seen["no_depth"] += 1
                                elif self.K is None:
                                    seen["no_intrinsics"] += 1
                                else:
                                    seen["invalid_position"] += 1
                            if spot is not None:
                                if search is None:
                                    aggregator.update_observation(
                                        spot, keypoints=person[1], conf=person[2], estimated=estimated)
                                else:
                                    attrs = (extract_person_attributes(rgb, person[1], person[2])
                                             if search.descriptor.get("clothing") else None)
                                    candidates.append(dict(xy=spot, bbox=person[0], keypoints=person[1],
                                                           keypoints_conf=person[2], attributes=attrs))
                        if search is not None:
                            # La misma imagen no cuenta varias veces como evidencia temporal.
                            search.update_frame(candidates)
                        self._publish_debug(rgb, people, stamp)
                        if (search is None and req.get("stop_on_first") is True
                                and aggregator.get_confirmed_customers(self._robot_pos())):
                            return aggregator.build_response(req_id, robot_pos=self._robot_pos())
                time.sleep(0.12)

            # Si ya se cubrio el barrido completo al menos una vez y hay clientes confirmados, se devuelve
            if pan_idx >= len(sweep_pans):
                confirmed = (search.build_response(req_id, self._robot_pos())["ok"] if search is not None
                             else aggregator.get_confirmed_customers(self._robot_pos()))
                if confirmed:
                    break

        if search is not None:
            return search.build_response(req_id, self._robot_pos())
        robot_xy = self._robot_pos()
        reason = ("no_raised_hand" if not seen["raised"] else
                  "unlocalized_raised_hand" if not seen["mapped_raised"] else "unconfirmed_raised_hand")
        out = aggregator.build_response(req_id, robot_pos=robot_xy, error_msg=reason)
        out["diagnostics"] = seen
        if out["ok"]:
            self.get_logger().info(f"Modo raised_hands: {len(out['customers'])} clientes encontrados")
        else:
            self.get_logger().info(f"Modo raised_hands: sin clientes ({out['error']}); cuadros={seen['frames']}, "
                                   f"maximo de personas en un cuadro={seen['people']}, cuadros con mano levantada={seen['raised']}; diagnostico={seen}")
        return out

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

    def _follow(self, req):
        """Mantiene la cabeza sobre la persona y publica donde estan las personas, hasta que lo paren."""
        end = time.time() + float(req.get("timeout_s", 300.0))
        self.follow_stop = False
        self._ensure_torque()
        last_seen, sweep = time.time(), 0
        while time.time() < end and not self.follow_stop:
            stamp = self.rgb_stamp
            rgb, depth = self._frames()
            cam = self._cam(stamp)
            found = []
            people = self._poses(rgb, cam) if rgb is not None else []
            if not people and time.time() - last_seen > LOST_PERSON_S:      # barrer: se pudo ir de cuadro al doblar
                sweep = (sweep + 1) % len(SEARCH_PANS)
                self._move_head(SEARCH_PANS[sweep], SEARCH_TILT)
                last_seen = time.time()
            if people and cam is not None and self.K is not None:
                last_seen = time.time()
                self._track(people[0], rgb.shape[1], rgb.shape[0])
                for person in people:
                    spot = bv.person_map_position(person, depth, cam[0], cam[1], self.K)
                    if spot:
                        found.append(dict(x=spot[0], y=spot[1]))
            self.pub_people.publish(String(data=json.dumps(dict(id=req.get("id"), people=found, t=time.time()))))
            time.sleep(0.25)

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
    import cv2
    cv2.setNumThreads(2)
    if not SIM:
        import torch
        torch.set_num_threads(2)      # no quitarle CPU a Nav2 ni a los controladores
        torch.set_num_interop_threads(1)
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
