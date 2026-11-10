#!/usr/bin/env python3
"""Deteccion del objeto en la camara de muneca, estable cuadro a cuadro.

- Detector principal: YOLOv8n (COCO: bottle, cup, vase), rapido en CPU.
- Respaldo de vocabulario abierto: YOLO-World (yolov8s-worldv2, clases por texto como
  "pringles can", "can", "box") cuando el principal no ve nada varios cuadros seguidos.
  Antes la lata de Pringles podia salir como "parking meter" y no se detectaba.
- TargetTracker: elige siempre el MISMO objeto (continuidad por IoU) y suaviza el ancho con
  la mediana de los ultimos cuadros, asi la distancia estimada no salta.
El tracker es puro (testeable); los modelos se cargan perezosamente.
"""
import os
import time
from collections import deque
from typing import List, Optional, Sequence, Tuple

Box = Tuple[float, float, float, float]

COCO_CLASSES = [39, 41, 75]  # bottle, cup, vase
WORLD_CLASSES = ["pringles can", "can", "bottle", "cup", "box", "jar"]


def iou(a: Sequence[float], b: Sequence[float]) -> float:
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


class TargetTracker:
    def __init__(self, window: int = 5, hold_s: float = 0.6, min_conf: float = 0.25):
        self.widths = deque(maxlen=window)
        self.hold_s = hold_s
        self.min_conf = min_conf
        self.box: Optional[Box] = None
        self.label = ""
        self.conf = 0.0
        self.last_seen = 0.0

    def update(self, dets: List[Tuple[Box, float, str]], now: Optional[float] = None) -> Optional[Tuple[Box, float]]:
        """dets: [(caja, confianza, etiqueta)]. Devuelve (caja, ancho_suavizado) o None."""
        now = time.time() if now is None else now
        dets = [d for d in dets if d[1] >= self.min_conf and d[0][2] - d[0][0] >= 20 and d[0][3] - d[0][1] >= 30]
        if dets:
            if self.box is not None:
                best = max(dets, key=lambda d: 0.6 * iou(d[0], self.box) + 0.4 * d[1])
                if iou(best[0], self.box) < 0.1 and now - self.last_seen < self.hold_s:
                    # objeto distinto que aparece de golpe: exigir mas confianza para cambiar
                    best = best if best[1] > self.conf + 0.2 else None
            else:
                best = max(dets, key=lambda d: d[1])
            if best is not None:
                if self.box is not None and iou(best[0], self.box) < 0.1:
                    self.widths.clear()
                self.box, self.conf, self.label = best
                self.widths.append(best[0][2] - best[0][0])
                self.last_seen = now
        elif self.box is not None and now - self.last_seen > self.hold_s:
            self.box, self.conf, self.label = None, 0.0, ""
            self.widths.clear()
        if self.box is None or not self.widths:
            return None
        w = sorted(self.widths)[len(self.widths) // 2]
        return self.box, float(w)


class WristDetector:
    def __init__(self, models_dir: str, world_weights: Optional[str] = None, imgsz: int = 416):
        self.models_dir = models_dir
        self.world_weights = world_weights or os.path.expanduser("~/base_athome_ws/yolov8s-worldv2.pt")
        self.imgsz = imgsz
        self.coco = None
        self.world = None
        self.misses = 0
        self.source = ""
        self.coco_classes = list(COCO_CLASSES)  # configurable por instancia, sin cambiar otros usuarios

    def _load_coco(self):
        if self.coco is None:
            from ultralytics import YOLO
            self.coco = YOLO(os.path.join(self.models_dir, "yolov8n.pt"))

    def _load_world(self):
        if self.world is None and os.path.exists(self.world_weights):
            from ultralytics import YOLOWorld
            cwd = os.getcwd()
            try:
                os.chdir(os.path.expanduser("~"))  # pesos de CLIP en ~/weights/clip (sin internet)
                self.world = YOLOWorld(self.world_weights)
                self.world.set_classes(WORLD_CLASSES)
            except Exception:
                self.world = False
            finally:
                os.chdir(cwd)
        return self.world

    def detect(self, frame) -> List[Tuple[Box, float, str]]:
        self._load_coco()
        r = self.coco.predict(frame, imgsz=self.imgsz, conf=0.2, classes=self.coco_classes, verbose=False, device="cpu")[0]
        dets = [(tuple(float(v) for v in b.xyxy[0].tolist()), float(b.conf), r.names[int(b.cls)]) for b in r.boxes]
        self.source = "YOLOv8n"
        if dets:
            self.misses = 0
            return dets
        self.misses += 1
        if self.misses >= 3 and self._load_world():
            r = self.world.predict(frame, imgsz=320, conf=0.15, verbose=False, device="cpu")[0]
            self.source = "YOLO-World"
            return [(tuple(float(v) for v in b.xyxy[0].tolist()), float(b.conf), r.names[int(b.cls)]) for b in r.boxes]
        return []


# Medido 2026-11-05 con 24 fotos de bolsas: con estos distractores World vio 17; solo las clases de bolsa, 6; COCO, 4.
# Los distractores le dan otra respuesta posible y la confianza de las bolsas sube de ~0,2 a ~0,6.
BAG_WORLD_CLASSES = ["black garbage bag", "trash bag", "plastic bag", "bag", "person", "chair", "table", "bottle", "box",
                     "wall", "floor"]
BAG_NAMES = {"black garbage bag", "trash bag", "plastic bag", "bag"}


# La camara de muneca ve abajo su propia garra y, segun la pose, la rueda y el chasis (imagen real, 2026-11-05).
# Una caja centrada en esa franja no se toma como bolsa, y la confianza minima deja afuera esos falsos (0,15; la bolsa dio 0,57).
OWN_ROBOT_BAND_FRAC = 0.70
WRIST_BAG_MIN_CONF = 0.30


def is_own_robot_box(box: Box, img_h: float, band_frac: float = OWN_ROBOT_BAND_FRAC) -> bool:
    """Caja centrada en la franja inferior de la camara de muneca, donde aparece el propio robot."""
    return (float(box[1]) + float(box[3])) / 2.0 >= band_frac * float(img_h)


class BagDetector:
    """Bolsas con YOLO-World en la CPU de la NUC (~125 ms). Pesos: models/yolov8s-worldv2.pt, no versionados."""

    source = "YOLO-World bolsas"
    def __init__(self, models_dir: str, conf: float = 0.15, ignore_own_robot: bool = False):
        self.weights = os.path.join(models_dir, "yolov8s-worldv2.pt")
        self.conf = conf
        self.ignore_own_robot = ignore_own_robot
        self.model = None

    def available(self) -> bool:
        return os.path.exists(self.weights)

    def load(self) -> None:
        if self.model is None:
            from ultralytics import YOLOWorld
            cwd = os.getcwd()
            try:
                os.chdir(os.path.expanduser("~"))  # pesos de CLIP en ~/weights/clip
                self.model = YOLOWorld(self.weights)
                self.model.set_classes(BAG_WORLD_CLASSES)
            finally:
                os.chdir(cwd)

    def detect_all(self, frame) -> List[Tuple[Box, float, str]]:
        """Todo lo que ve, distractores incluidos: para el debug."""
        self.load()
        r = self.model.predict(frame, imgsz=640, conf=0.15, verbose=False, device="cpu")[0]
        return [(tuple(float(v) for v in b.xyxy[0].tolist()), float(b.conf), r.names[int(b.cls)]) for b in r.boxes]

    def detect(self, frame) -> List[Tuple[Box, float, str]]:
        self.load()
        r = self.model.predict(frame, imgsz=640, conf=self.conf, verbose=False, device="cpu")[0]
        out = [(tuple(float(v) for v in b.xyxy[0].tolist()), float(b.conf), r.names[int(b.cls)]) for b in r.boxes
               if r.names[int(b.cls)] in BAG_NAMES]
        if self.ignore_own_robot:
            out = [d for d in out if not is_own_robot_box(d[0], frame.shape[0])]
        return out


class SimColorDetector:
    """Detector para la simulacion (ATHOME_SIM=1): segmenta el objeto magenta que dibuja sim_robot.
    Misma interfaz que WristDetector.detect -> [(caja, confianza, etiqueta)]."""

    def __init__(self, *_, ignore_own_robot: bool = False, **__):
        self.ignore_own_robot = ignore_own_robot
        self.source = "sim-color"
        self.misses = 0

    def detect(self, frame) -> List[Tuple[Box, float, str]]:
        import cv2
        import numpy as np
        # Prueba de robustez: ATHOME_SIM_DETECTION_DROPOUT=0.8 pierde el 80 % de los cuadros (deteccion que parpadea)
        drop = float(os.environ.get("ATHOME_SIM_DETECTION_DROPOUT", "0") or 0)
        if drop > 0 and np.random.random() < drop:
            return []
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([140, 120, 80]), np.array([165, 255, 255]))
        n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        dets = []
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if area >= 40:
                dets.append(((float(x), float(y), float(x + w), float(y + h)), 0.9,
                             os.environ.get("ATHOME_SIM_OBJECT_LABEL", "pringles can")))
        if self.ignore_own_robot:
            dets = [d for d in dets if not is_own_robot_box(d[0], frame.shape[0])]
        return dets

    def detect_all(self, frame):
        return self.detect(frame)


def make_bag_detector(models_dir: str, wrist: bool = True):
    """Detector de bolsas, real o de la simulacion. En la muneca descarta al propio robot y exige mas confianza."""
    if os.environ.get("ATHOME_SIM") == "1":
        return SimColorDetector(ignore_own_robot=wrist)
    return BagDetector(models_dir, conf=WRIST_BAG_MIN_CONF, ignore_own_robot=True) if wrist else BagDetector(models_dir)


def make_detector(models_dir: str):
    """WristDetector real, o SimColorDetector si ATHOME_SIM=1."""
    if os.environ.get("ATHOME_SIM") == "1":
        return SimColorDetector()
    return WristDetector(models_dir)
