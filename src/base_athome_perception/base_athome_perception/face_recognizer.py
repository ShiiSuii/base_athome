#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Sistema de Deteccion y Reconocimiento Facial en Tiempo Real
# Basado en Deep Learning: OpenCV DNN SSD ResNet-10 + SFace (Embeddings 128-D)
# Compatible con ROS 2 Jazzy y Python 3.12 (RoboCup @Home)
# ==============================================================================

import os
import pickle
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


class FaceRecognizer:
    """Motor de deteccion y reconocimiento facial en tiempo real usando OpenCV DNN."""

    def __init__(
        self,
        caffe_proto_path: str,
        caffe_model_path: str,
        sface_model_path: str,
        db_path: str,
        conf_thresh: float = 0.55,
        cosine_thresh: float = 0.363,
    ):
        self.caffe_proto_path = caffe_proto_path
        self.caffe_model_path = caffe_model_path
        self.sface_model_path = sface_model_path
        self.db_path = db_path
        self.conf_thresh = conf_thresh
        self.cosine_thresh = cosine_thresh

        # 1. Cargar Detector Facial Deep Learning SSD ResNet-10
        if not os.path.exists(self.caffe_proto_path) or not os.path.exists(self.caffe_model_path):
            raise FileNotFoundError(f"Modelos Caffe no encontrados: {self.caffe_proto_path}")
        self.detector_net = cv2.dnn.readNetFromCaffe(self.caffe_proto_path, self.caffe_model_path)

        # 2. Cargar Extractor Facial Deep Learning SFace (128-D)
        if not os.path.exists(self.sface_model_path):
            raise FileNotFoundError(f"Modelo SFace no encontrado: {self.sface_model_path}")
        self.sface = cv2.FaceRecognizerSF.create(self.sface_model_path, "")

        # 3. Base de datos de rostros conocidos: { "nombre": [vector_feat_1, vector_feat_2, ...] }
        self.known_faces: Dict[str, List[np.ndarray]] = {}
        self.load_database()

    def load_database(self) -> None:
        """Carga la base de datos de embeddings desde disco."""
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, "rb") as f:
                    self.known_faces = pickle.load(f)
                total = sum(len(v) for v in self.known_faces.values())
                print(f"[FaceRecognizer] Base de datos cargada: {len(self.known_faces)} personas, {total} muestras.")
            except Exception as e:
                print(f"[FaceRecognizer] Error cargando DB ({e}), iniciando vacia.")
                self.known_faces = {}
        else:
            self.known_faces = {}

    def save_database(self) -> None:
        """Persiste la base de datos de embeddings en disco."""
        try:
            os.makedirs(os.path.dirname(os.path.abspath(self.db_path)), exist_ok=True)
            with open(self.db_path, "wb") as f:
                pickle.dump(self.known_faces, f)
            print(f"[FaceRecognizer] Base de datos guardada en: {self.db_path}")
        except Exception as e:
            print(f"[FaceRecognizer] Error guardando DB: {e}")

    def detect_faces(self, image: np.ndarray) -> List[Tuple[int, int, int, int, float]]:
        """Detecta rostros usando DNN SSD ResNet-10. Retorna lista de (x, y, w, h, score)."""
        h, w = image.shape[:2]
        # Preprocesamiento DNN (Blob 300x300 con substraccion de medias BGR)
        blob = cv2.dnn.blobFromImage(
            cv2.resize(image, (300, 300)),
            scalefactor=1.0,
            size=(300, 300),
            mean=(104.0, 177.0, 123.0),
        )
        self.detector_net.setInput(blob)
        detections = self.detector_net.forward()

        faces = []
        for i in range(detections.shape[2]):
            confidence = float(detections[0, 0, i, 2])
            if confidence > self.conf_thresh:
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                x1, y1, x2, y2 = box.astype("int")

                # Asegurar limites
                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(w, x2)
                y2 = min(h, y2)
                bw = x2 - x1
                bh = y2 - y1

                if bw > 20 and bh > 20:
                    faces.append((x1, y1, bw, bh, confidence))

        return faces

    def extract_feature(self, image: np.ndarray, bbox: Tuple[int, int, int, int]) -> Optional[np.ndarray]:
        """Recorta y extrae el vector biometrico de 128 caracteristicas con SFace."""
        x, y, w, h = bbox
        img_h, img_w = image.shape[:2]

        # Margen del 10% para contexto facial
        margin_x = int(w * 0.10)
        margin_y = int(h * 0.10)
        x1 = max(0, x - margin_x)
        y1 = max(0, y - margin_y)
        x2 = min(img_w, x + w + margin_x)
        y2 = min(img_h, y + h + margin_y)

        face_crop = image[y1:y2, x1:x2]
        if face_crop.size == 0:
            return None

        # Redimensionar al tamano de entrada de SFace (112x112)
        face_aligned = cv2.resize(face_crop, (112, 112))
        feature = self.sface.feature(face_aligned)
        return feature

    def recognize_face(self, feature: np.ndarray) -> Tuple[str, float]:
        """Compara un embedding contra los rostros guardados usando Similitud Coseno."""
        if not self.known_faces or feature is None:
            return "Desconocido", 0.0

        best_name = "Desconocido"
        best_score = -1.0

        for name, feats in self.known_faces.items():
            for known_feat in feats:
                score = self.sface.match(feature, known_feat, cv2.FaceRecognizerSF_FR_COSINE)
                if score > best_score:
                    best_score = score
                    if score >= self.cosine_thresh:
                        best_name = name

        return best_name, max(0.0, float(best_score))

    def detect_and_recognize(self, image: np.ndarray) -> List[Dict]:
        """Detecta todos los rostros en la imagen y retorna identificaciones y cajas."""
        detected = self.detect_faces(image)
        results = []

        for (x, y, w, h, score) in detected:
            feature = self.extract_feature(image, (x, y, w, h))
            if feature is not None:
                name, similarity = self.recognize_face(feature)
            else:
                name, similarity = "Desconocido", 0.0

            results.append({
                "bbox": [x, y, w, h],
                "det_score": score,
                "name": name,
                "similarity": similarity,
                "feature": feature,
            })

        return results

    def enroll_face(self, image: np.ndarray, name: str) -> bool:
        """Registra un nuevo rostro en la base de datos."""
        faces = self.detect_faces(image)
        if not faces:
            print("[FaceRecognizer] No se detecto ningun rostro para registrar.")
            return False

        # Tomar el rostro de mayor area
        largest_face = max(faces, key=lambda f: f[2] * f[3])
        x, y, w, h, _ = largest_face
        feature = self.extract_feature(image, (x, y, w, h))

        if feature is None:
            print("[FaceRecognizer] Error extrayendo embedding facial.")
            return False

        if name not in self.known_faces:
            self.known_faces[name] = []

        self.known_faces[name].append(feature)
        self.save_database()
        print(f"[FaceRecognizer] ¡Rostro registrado exitosamente para: '{name}'!")
        return True

    def enroll_feature(self, feature: np.ndarray, name: str) -> bool:
        """Registra directamente un vector de caracteristicas para un nombre."""
        if feature is None:
            return False
        if name not in self.known_faces:
            self.known_faces[name] = []
        self.known_faces[name].append(feature)
        self.save_database()
        return True

    def clear_database(self) -> None:
        """Borra todos los rostros guardados."""
        self.known_faces = {}
        if os.path.exists(self.db_path):
            os.remove(self.db_path)
        print("[FaceRecognizer] Base de datos borrada.")
