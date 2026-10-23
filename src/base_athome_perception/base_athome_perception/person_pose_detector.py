#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Detector de Poses Humanas y Posturas (YOLOv8-Pose)
# RoboCup @Home: Tareas Receptionist, Carry My Luggage, Find Person
# ==============================================================================

import os
from typing import Dict, List, Tuple

import cv2
import numpy as np
from ultralytics import YOLO


class PersonPoseDetector:
    """Detector de poses humanas usando YOLOv8-pose con clasificacion de postura."""

    # Conexiones anatomicas del esqueleto COCO (17 keypoints)
    SKELETON_CONNECTIONS = [
        (0, 1), (0, 2), (1, 3), (2, 4),  # Cabeza / Rostro
        (5, 6),                          # Hombros
        (5, 7), (7, 9),                  # Brazo izquierdo
        (6, 8), (8, 10),                 # Brazo derecho
        (5, 11), (6, 12),                # Tronco
        (11, 12),                        # Caderas
        (11, 13), (13, 15),              # Pierna izquierda
        (12, 14), (14, 16),              # Pierna derecha
    ]

    def __init__(self, model_path: str, conf_thresh: float = 0.50):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"No se encontro modelo YOLOv8-pose en: {model_path}")
        self.model = YOLO(model_path)
        self.conf_thresh = conf_thresh

    def detect_poses(self, image: np.ndarray) -> List[Dict]:
        """Detecta personas, keypoints y postura en la imagen."""
        results = self.model(image, conf=self.conf_thresh, verbose=False)[0]
        detections = []

        if results.boxes is None or results.keypoints is None:
            return detections

        boxes = results.boxes.xyxy.cpu().numpy()
        scores = results.boxes.conf.cpu().numpy()
        kpts = results.keypoints.xy.cpu().numpy()
        kpts_conf = results.keypoints.conf.cpu().numpy() if results.keypoints.conf is not None else None

        for i in range(len(boxes)):
            box = boxes[i].astype(int)
            score = float(scores[i])
            points = kpts[i]
            points_conf = kpts_conf[i] if kpts_conf is not None else np.ones(len(points))

            posture, posture_desc = self.classify_posture(points, points_conf)

            detections.append({
                "bbox": box,
                "score": score,
                "keypoints": points,
                "keypoints_conf": points_conf,
                "posture": posture,
                "description": posture_desc,
            })

        return detections

    def classify_posture(
        self, points: np.ndarray, conf: np.ndarray
    ) -> Tuple[str, str]:
        """Clasifica la postura humana segun la cinematica de las articulaciones."""
        # Indices COCO:
        # 5: L_Shoulder, 6: R_Shoulder
        # 7: L_Elbow, 8: R_Elbow
        # 9: L_Wrist, 10: R_Wrist
        # 11: L_Hip, 12: R_Hip
        # 13: L_Knee, 14: R_Knee
        # 15: L_Ankle, 16: R_Ankle

        # 1. Comprobar brazo levantado (saludando o senalando)
        l_wrist_up = conf[9] > 0.4 and conf[5] > 0.4 and points[9][1] < points[5][1]
        r_wrist_up = conf[10] > 0.4 and conf[6] > 0.4 and points[10][1] < points[6][1]

        if l_wrist_up and r_wrist_up:
            return "BRAZOS_ARRIBA", "Ambos brazos levantados"
        elif l_wrist_up or r_wrist_up:
            side = "izquierdo" if l_wrist_up else "derecho"
            return "SALUDANDO", f"Brazo {side} levantado / Senalando"

        # 2. Comprobar postura de pie vs sentado
        # Si caderas y rodillas estan visibles
        if conf[11] > 0.3 and conf[12] > 0.3 and conf[13] > 0.3 and conf[14] > 0.3:
            hip_y = (points[11][1] + points[12][1]) / 2.0
            knee_y = (points[13][1] + points[14][1]) / 2.0
            # Si los tobillos estan visibles
            if conf[15] > 0.3 and conf[16] > 0.3:
                ankle_y = (points[15][1] + points[16][1]) / 2.0
                thigh_len = abs(knee_y - hip_y)
                calf_len = abs(ankle_y - knee_y)
                if thigh_len < calf_len * 0.55:
                    return "SENTADO", "Persona sentada"

        return "DE_PIE", "Persona de pie"

    def draw_poses(self, image: np.ndarray, detections: List[Dict]) -> np.ndarray:
        """Dibuja esqueletos, articulaciones y etiquetas sobre la imagen."""
        vis = image.copy()

        for det in detections:
            x1, y1, x2, y2 = det["bbox"]
            posture = det["posture"]
            desc = det["description"]
            score = det["score"]
            points = det["keypoints"]
            conf = det["keypoints_conf"]

            # Color segun postura
            if "SALUDANDO" in posture or "BRAZOS" in posture:
                color = (0, 255, 255)  # Amarillo
            elif "SENTADO" in posture:
                color = (255, 100, 0)  # Azul/Naranja
            else:
                color = (0, 255, 120)  # Verde

            # Bounding box
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

            # Banner de texto
            lbl = f"{desc} ({int(score * 100)}%)"
            (tw, th), _ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(vis, (x1, y1 - th - 6), (x1 + tw + 8, y1), color, -1)
            cv2.putText(vis, lbl, (x1 + 4, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (15, 15, 20), 2)

            # Dibujar lineas del esqueleto
            for p1_idx, p2_idx in self.SKELETON_CONNECTIONS:
                if conf[p1_idx] > 0.4 and conf[p2_idx] > 0.4:
                    pt1 = (int(points[p1_idx][0]), int(points[p1_idx][1]))
                    pt2 = (int(points[p2_idx][0]), int(points[p2_idx][1]))
                    cv2.line(vis, pt1, pt2, (255, 255, 255), 2)

            # Dibujar articulaciones (keypoints)
            for idx, pt in enumerate(points):
                if conf[idx] > 0.4:
                    kpt_x, kpt_y = int(pt[0]), int(pt[1])
                    cv2.circle(vis, (kpt_x, kpt_y), 4, (0, 0, 255), -1)

        return vis
