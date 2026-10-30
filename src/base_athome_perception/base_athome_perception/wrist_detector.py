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
        r = self.coco.predict(frame, imgsz=self.imgsz, conf=0.2, classes=COCO_CLASSES, verbose=False, device="cpu")[0]
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
