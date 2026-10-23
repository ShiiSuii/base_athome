#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Detector de Basura, Residuos y Contenedores (YOLOv8)
# RoboCup @Home: Tarea "Take Out the Garbage" / "Clean Up"
# ==============================================================================

import os
from typing import Dict, List

import cv2
import numpy as np
from ultralytics import YOLO


class GarbageDetector:
    """Detector de objetos de basura, residuos y contenedores para RoboCup @Home."""

    # Clases COCO tipicas de basura y recipientes
    GARBAGE_CLASSES = {
        39: "botella",
        40: "copa_vidrio",
        41: "taza_vaso",
        42: "tenedor",
        43: "cuchillo",
        44: "cuchara",
        45: "tazon",
        46: "banana_cascara",
        47: "manzana_resto",
        48: "sandwich",
        24: "mochila_bolsa",
        26: "bolso_bolsa",
        28: "valija_contenedor",
        # Cualquier objeto comunmente considerado residuo
    }

    def __init__(self, model_path: str, conf_thresh: float = 0.40):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"No se encontro modelo YOLOv8 en: {model_path}")
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh

    def detect_garbage(self, image: np.ndarray, depth_image: np.ndarray = None) -> List[Dict]:
        """Detecta residuos u objetos en la imagen y opcionalmente calcula distancia 3D."""
        results = self.model(image, conf=self.conf_thresh, verbose=False)[0]
        detections = []

        if results.boxes is None:
            return detections

        boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()
        classes = results.boxes.cls.cpu().numpy().astype(int)

        for i in range(len(boxes)):
            cls_id = classes[i]
            # Filtrar si pertenece a clases de residuos o recipientes
            label = self.GARBAGE_CLASSES.get(cls_id, results.names.get(cls_id, f"obj_{cls_id}"))

            box = boxes[i].astype(int)
            score = float(scores[i])
            cx = (box[0] + box[2]) // 2
            cy = (box[1] + box[3]) // 2

            distance_m = None
            if depth_image is not None:
                # Muestreo seguro alrededor del centro del objeto
                h_d, w_d = depth_image.shape[:2]
                if 0 <= cy < h_d and 0 <= cx < w_d:
                    depth_val = float(depth_image[cy, cx])
                    if depth_val > 100:  # En mm
                        distance_m = depth_val / 1000.0

            detections.append({
                "bbox": box,
                "score": score,
                "class_id": cls_id,
                "label": label,
                "center": (cx, cy),
                "distance_m": distance_m,
            })

        return detections

    def draw_garbage(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """Dibuja bounding boxes y estimacion de distancia sobre la imagen."""
        vis = image.copy()

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            label = det["label"]
            score = det["score"]
            dist = det["distance_m"]
            cx, cy = det["center"]

            color = (0, 140, 255)  # Naranja residuos
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)

            dist_str = f" | {dist:.2f}m" if dist else ""
            tag = f"{label} {int(score * 100)}%{dist_str}"

            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(vis, (x1, y1 - th - 6), (x1 + tw + 6, y1), color, -1)
            cv2.putText(vis, tag, (x1 + 3, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return vis
