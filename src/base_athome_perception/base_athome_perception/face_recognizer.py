#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Sistema de Memoria Biometrica y Reconocimiento Facial Multi-Persona
# Inspirado en estandares RoboCup @Home (UNAM Justina, Homer Koblenz, TU/e)
# Motor: OpenCV DNN SSD ResNet-10 + SFace (Embeddings Biometricos 128-D)
# Compatible con ROS 2 Jazzy y Python 3.12
# ==============================================================================

from dataclasses import dataclass, field
import datetime
import json
import os
import pickle
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class PersonProfile:
    """Perfil biometrico y de memoria de una persona registrada en RoboCup @Home."""

    person_id: str
    name: str
    enrolled_at: float
    enrolled_date: str
    last_seen: float
    last_seen_date: str
    sighting_count: int = 1
    thumbnail_file: str = ""
    samples_count: int = 0
    attributes: Dict[str, Any] = field(default_factory=dict)
    features: List[np.ndarray] = field(default_factory=list, repr=False)
    thumbnail_img: Optional[np.ndarray] = field(default=None, repr=False)
    _last_sighting_recorded: float = field(default=0.0, repr=False)

    def to_metadata_dict(self) -> Dict[str, Any]:
        return {
            "person_id": self.person_id,
            "name": self.name,
            "enrolled_at": self.enrolled_at,
            "enrolled_date": self.enrolled_date,
            "last_seen": self.last_seen,
            "last_seen_date": self.last_seen_date,
            "sighting_count": self.sighting_count,
            "thumbnail_file": self.thumbnail_file,
            "samples_count": len(self.features),
            "attributes": self.attributes,
        }


class FaceRecognizer:
    """Motor de deteccion, reconocimiento y memoria facial multi-persona para RoboCup @Home."""

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
        self.conf_thresh = conf_thresh
        self.cosine_thresh = cosine_thresh

        # Directorio de persistencia de la base de datos de memoria
        if db_path.endswith(".pkl") or db_path.endswith(".json"):
            base_dir = os.path.dirname(os.path.abspath(db_path))
            self.db_dir = os.path.join(base_dir, "faces_database")
            self.legacy_pkl_path = db_path
        else:
            self.db_dir = os.path.abspath(db_path)
            self.legacy_pkl_path = os.path.join(os.path.dirname(self.db_dir), "faces_db.pkl")

        self.metadata_file = os.path.join(self.db_dir, "profiles.json")
        self.features_file = os.path.join(self.db_dir, "features.pkl")
        self.thumbnails_dir = os.path.join(self.db_dir, "thumbnails")

        os.makedirs(self.thumbnails_dir, exist_ok=True)

        # 1. Cargar Detector Facial Deep Learning SSD ResNet-10
        if not os.path.exists(self.caffe_proto_path) or not os.path.exists(self.caffe_model_path):
            raise FileNotFoundError(f"Modelos Caffe no encontrados: {self.caffe_proto_path}")
        self.detector_net = cv2.dnn.readNetFromCaffe(self.caffe_proto_path, self.caffe_model_path)

        # 2. Cargar Extractor Facial Deep Learning SFace (128-D)
        if not os.path.exists(self.sface_model_path):
            raise FileNotFoundError(f"Modelo SFace no encontrado: {self.sface_model_path}")
        self.sface = cv2.FaceRecognizerSF.create(self.sface_model_path, "")

        # 3. Diccionario de perfiles en memoria: { person_id: PersonProfile }
        self.profiles: Dict[str, PersonProfile] = {}
        self.load_database()

    @property
    def known_faces(self) -> Dict[str, List[np.ndarray]]:
        """Compatibilidad retroactiva con implementaciones previas."""
        return {p.name: p.features for p in self.profiles.values()}

    def load_database(self) -> None:
        """Carga la base de datos de perfiles y caracteristicas biometricas desde disco."""
        self.profiles = {}

        # Comprobar si existe la nueva estructura (profiles.json + features.pkl)
        if os.path.exists(self.metadata_file) and os.path.exists(self.features_file):
            try:
                with open(self.metadata_file, "r", encoding="utf-8") as f:
                    meta_dict = json.load(f)

                with open(self.features_file, "rb") as f:
                    feats_dict = pickle.load(f)

                for pid, meta in meta_dict.items():
                    feats = feats_dict.get(pid, [])
                    thumb_img = None
                    thumb_path = os.path.join(self.db_dir, meta.get("thumbnail_file", ""))
                    if os.path.exists(thumb_path):
                        thumb_img = cv2.imread(thumb_path)

                    profile = PersonProfile(
                        person_id=pid,
                        name=meta["name"],
                        enrolled_at=meta.get("enrolled_at", time.time()),
                        enrolled_date=meta.get("enrolled_date", ""),
                        last_seen=meta.get("last_seen", time.time()),
                        last_seen_date=meta.get("last_seen_date", ""),
                        sighting_count=meta.get("sighting_count", 1),
                        thumbnail_file=meta.get("thumbnail_file", ""),
                        samples_count=len(feats),
                        attributes=meta.get("attributes", {}),
                        features=feats,
                        thumbnail_img=thumb_img,
                        _last_sighting_recorded=meta.get("last_seen", time.time()),
                    )
                    self.profiles[pid] = profile

                total_samples = sum(len(p.features) for p in self.profiles.values())
                print(f"[FaceRecognizer] Memoria cargada: {len(self.profiles)} perfiles, {total_samples} muestras biometricas.")
                return
            except Exception as e:
                print(f"[FaceRecognizer] Error cargando memoria ({e}), verificando respaldo...")

        # Migracion automatica si solo existe faces_db.pkl legado
        if os.path.exists(self.legacy_pkl_path):
            try:
                with open(self.legacy_pkl_path, "rb") as f:
                    legacy_data = pickle.load(f)
                if isinstance(legacy_data, dict):
                    print(f"[FaceRecognizer] Migrando base de datos legada ({len(legacy_data)} nombres)...")
                    idx = 1
                    now = time.time()
                    now_str = datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S")
                    for name, feats in legacy_data.items():
                        # Omitir nombres accidentales o invalidos
                        if name.lower() in ["desconocido", "unknown", "me llamo"]:
                            continue
                        pid = f"person_{idx:03d}"
                        self.profiles[pid] = PersonProfile(
                            person_id=pid,
                            name=name,
                            enrolled_at=now,
                            enrolled_date=now_str,
                            last_seen=now,
                            last_seen_date=now_str,
                            sighting_count=1,
                            thumbnail_file="",
                            samples_count=len(feats) if isinstance(feats, list) else 1,
                            features=feats if isinstance(feats, list) else [feats],
                            _last_sighting_recorded=now,
                        )
                        idx += 1
                    self.save_database()
                    print(f"[FaceRecognizer] Migracion exitosa a formato RoboCup @Home: {len(self.profiles)} perfiles.")
            except Exception as e:
                print(f"[FaceRecognizer] Error migrando archivo legado: {e}")

    def save_database(self) -> None:
        """Persiste perfiles (JSON) y vectores biometricos (PKL) de forma atomica."""
        try:
            os.makedirs(self.thumbnails_dir, exist_ok=True)

            meta_dict = {}
            feats_dict = {}

            for pid, p in self.profiles.items():
                meta_dict[pid] = p.to_metadata_dict()
                feats_dict[pid] = p.features

            # 1. Guardar metadata en JSON
            temp_meta = self.metadata_file + ".tmp"
            with open(temp_meta, "w", encoding="utf-8") as f:
                json.dump(meta_dict, f, indent=2, ensure_ascii=False)
            os.replace(temp_meta, self.metadata_file)

            # 2. Guardar vectores en PKL
            temp_feats = self.features_file + ".tmp"
            with open(temp_feats, "wb") as f:
                pickle.dump(feats_dict, f)
            os.replace(temp_feats, self.features_file)

            # 3. Compatibilidad con legacy_pkl_path
            if self.legacy_pkl_path:
                try:
                    with open(self.legacy_pkl_path, "wb") as f:
                        pickle.dump(self.known_faces, f)
                except Exception:
                    pass

        except Exception as e:
            print(f"[FaceRecognizer] Error persistiendo base de datos: {e}")

    def save_metadata_only(self) -> None:
        """Persiste rapidamente solo la metadata (para contadores de avistamiento y timestamps)."""
        try:
            meta_dict = {pid: p.to_metadata_dict() for pid, p in self.profiles.items()}
            temp_meta = self.metadata_file + ".tmp"
            with open(temp_meta, "w", encoding="utf-8") as f:
                json.dump(meta_dict, f, indent=2, ensure_ascii=False)
            os.replace(temp_meta, self.metadata_file)
        except Exception:
            pass

    @staticmethod
    def assess_face_quality(image: np.ndarray, bbox: Tuple[int, int, int, int]) -> Tuple[bool, float, str]:
        """Evalua la calidad de una deteccion facial (nitidez, tamano, bordes)."""
        x, y, w, h = bbox
        if w < 50 or h < 50:
            return False, 0.0, "Rostro demasiado pequeno"

        img_h, img_w = image.shape[:2]
        if x < 0 or y < 0 or (x + w) > img_w or (y + h) > img_h:
            return False, 0.0, "Rostro fuera de los margenes"

        crop = image[y : y + h, x : x + w]
        if crop.size == 0:
            return False, 0.0, "Recorte vacio"

        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())

        # Umbral minimo de nitidez para evitar muestras borrosas durante movimiento
        if lap_var < 30.0:
            return False, lap_var, "Muestra borrosa (desenfoque)"

        return True, lap_var, "Excelente calidad"

    def detect_faces(self, image: np.ndarray) -> List[Tuple[int, int, int, int, float]]:
        """Detecta rostros usando DNN SSD ResNet-10. Retorna lista de (x, y, w, h, score)."""
        h, w = image.shape[:2]
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

                x1 = max(0, x1)
                y1 = max(0, y1)
                x2 = min(w, x2)
                y2 = min(h, y2)
                bw = x2 - x1
                bh = y2 - y1

                if bw > 25 and bh > 25:
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

    def recognize_face(self, feature: np.ndarray) -> Tuple[str, float, Optional[str]]:
        """Compara un embedding contra los perfiles guardados usando Similitud Coseno."""
        if not self.profiles or feature is None:
            return "Desconocido", 0.0, None

        best_profile: Optional[PersonProfile] = None
        best_score = -1.0

        for profile in self.profiles.values():
            if not profile.features:
                continue
            # Obtener la maxima similitud entre todas las muestras biometricas de la persona
            scores = [
                float(self.sface.match(feature, known_feat, cv2.FaceRecognizerSF_FR_COSINE))
                for known_feat in profile.features
            ]
            person_max_score = max(scores) if scores else -1.0
            if person_max_score > best_score:
                best_score = person_max_score
                best_profile = profile

        if best_profile is not None and best_score >= self.cosine_thresh:
            return best_profile.name, max(0.0, float(best_score)), best_profile.person_id

        return "Desconocido", max(0.0, float(best_score)), None

    def detect_and_recognize(self, image: np.ndarray) -> List[Dict]:
        """Detecta todos los rostros en la imagen y retorna identificaciones, cajas y calidad."""
        detected = self.detect_faces(image)
        results = []

        for (x, y, w, h, score) in detected:
            feature = self.extract_feature(image, (x, y, w, h))
            is_valid, q_score, q_reason = self.assess_face_quality(image, (x, y, w, h))

            if feature is not None:
                name, similarity, person_id = self.recognize_face(feature)
            else:
                name, similarity, person_id = "Desconocido", 0.0, None

            results.append({
                "bbox": [x, y, w, h],
                "det_score": score,
                "name": name,
                "similarity": similarity,
                "person_id": person_id,
                "feature": feature,
                "quality_valid": is_valid,
                "quality_score": q_score,
                "quality_reason": q_reason,
            })

        return results

    def enroll_person(
        self,
        name: str,
        features: List[np.ndarray],
        thumbnail_img: Optional[np.ndarray] = None,
        attributes: Optional[Dict] = None,
    ) -> Tuple[str, bool]:
        """Registra o actualiza una persona con multiples muestras biometricas y foto de perfil."""
        name = name.strip().title()
        if not name or not features:
            return "", False

        now = time.time()
        now_str = datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S")

        # Verificar si la persona ya existe en la base de datos (por nombre insensible a mayusculas)
        existing_profile: Optional[PersonProfile] = None
        for p in self.profiles.values():
            if p.name.lower() == name.lower():
                existing_profile = p
                break

        if existing_profile is not None:
            # Actualizar perfil existente con nuevas muestras biometricas diversas
            pid = existing_profile.person_id
            existing_profile.last_seen = now
            existing_profile.last_seen_date = now_str
            existing_profile.sighting_count += 1
            if attributes:
                existing_profile.attributes.update(attributes)

            # Agregar hasta un maximo de 15 muestras biometricas
            for feat in features:
                if len(existing_profile.features) < 15:
                    existing_profile.features.append(feat)

            if thumbnail_img is not None and thumbnail_img.size > 0:
                thumb_resized = cv2.resize(thumbnail_img, (140, 140))
                rel_path = f"thumbnails/{pid}.jpg"
                abs_path = os.path.join(self.db_dir, rel_path)
                cv2.imwrite(abs_path, thumb_resized)
                existing_profile.thumbnail_file = rel_path
                existing_profile.thumbnail_img = thumb_resized

            self.save_database()
            print(f"[FaceRecognizer] Perfil actualizado para '{name}' ({len(existing_profile.features)} muestras).")
            return pid, False

        else:
            # Crear nuevo perfil de persona
            pid = f"person_{len(self.profiles) + 1:03d}"
            # Asegurar que el ID sea unico
            while pid in self.profiles:
                next_idx = int(pid.split("_")[1]) + 1
                pid = f"person_{next_idx:03d}"

            rel_thumb = ""
            thumb_cache = None
            if thumbnail_img is not None and thumbnail_img.size > 0:
                thumb_resized = cv2.resize(thumbnail_img, (140, 140))
                rel_thumb = f"thumbnails/{pid}.jpg"
                abs_path = os.path.join(self.db_dir, rel_thumb)
                cv2.imwrite(abs_path, thumb_resized)
                thumb_cache = thumb_resized

            profile = PersonProfile(
                person_id=pid,
                name=name,
                enrolled_at=now,
                enrolled_date=now_str,
                last_seen=now,
                last_seen_date=now_str,
                sighting_count=1,
                thumbnail_file=rel_thumb,
                samples_count=len(features),
                attributes=attributes or {},
                features=list(features),
                thumbnail_img=thumb_cache,
                _last_sighting_recorded=now,
            )
            self.profiles[pid] = profile
            self.save_database()
            print(f"[FaceRecognizer] ¡Nuevo perfil registrado con exito: '{name}' [ID: {pid}]!")
            return pid, True

    def enroll_face(self, image: np.ndarray, name: str) -> bool:
        """Compatibilidad: Registra una persona a partir de un fotograma."""
        faces = self.detect_faces(image)
        if not faces:
            return False

        largest = max(faces, key=lambda f: f[2] * f[3])
        x, y, w, h, _ = largest
        feature = self.extract_feature(image, (x, y, w, h))
        if feature is None:
            return False

        crop = image[max(0, y) : min(image.shape[0], y + h), max(0, x) : min(image.shape[1], x + w)]
        pid, _ = self.enroll_person(name, [feature], thumbnail_img=crop)
        return bool(pid)

    def enroll_feature(self, feature: np.ndarray, name: str) -> bool:
        """Compatibilidad: Registra directamente un feature."""
        if feature is None:
            return False
        pid, _ = self.enroll_person(name, [feature])
        return bool(pid)

    def update_sighting(self, identifier: str, min_interval_sec: float = 20.0) -> bool:
        """Actualiza el contador de encuentros de una persona si ha transcurrido el intervalo minimo."""
        target: Optional[PersonProfile] = None
        if identifier in self.profiles:
            target = self.profiles[identifier]
        else:
            for p in self.profiles.values():
                if p.name.lower() == identifier.lower():
                    target = p
                    break

        if target is None:
            return False

        now = time.time()
        target.last_seen = now
        target.last_seen_date = datetime.datetime.fromtimestamp(now).strftime("%Y-%m-%d %H:%M:%S")

        # Verificar si constituye un nuevo encuentro para no saturar el contador
        if now - target._last_sighting_recorded >= min_interval_sec:
            target.sighting_count += 1
            target._last_sighting_recorded = now
            self.save_metadata_only()
            return True

        return False

    def get_known_profiles(self) -> List[Dict[str, Any]]:
        """Retorna resumen de todos los perfiles cargados para renderizado visual."""
        result = []
        for p in self.profiles.values():
            result.append({
                "id": p.person_id,
                "name": p.name,
                "enrolled_date": p.enrolled_date,
                "last_seen": p.last_seen,
                "last_seen_date": p.last_seen_date,
                "sighting_count": p.sighting_count,
                "samples_count": len(p.features),
                "thumbnail_img": p.thumbnail_img,
                "attributes": p.attributes,
            })
        return result

    def delete_person(self, identifier: str) -> bool:
        """Elimina un perfil especifico de la memoria."""
        pid_to_remove = None
        if identifier in self.profiles:
            pid_to_remove = identifier
        else:
            for p in self.profiles.values():
                if p.name.lower() == identifier.lower():
                    pid_to_remove = p.person_id
                    break

        if pid_to_remove and pid_to_remove in self.profiles:
            profile = self.profiles.pop(pid_to_remove)
            if profile.thumbnail_file:
                thumb_path = os.path.join(self.db_dir, profile.thumbnail_file)
                if os.path.exists(thumb_path):
                    try:
                        os.remove(thumb_path)
                    except Exception:
                        pass
            self.save_database()
            print(f"[FaceRecognizer] Perfil '{profile.name}' eliminado de la memoria.")
            return True
        return False

    def clear_database(self) -> None:
        """Borra todos los perfiles guardados y limpia el directorio de miniaturas."""
        self.profiles.clear()
        if os.path.exists(self.metadata_file):
            try:
                os.remove(self.metadata_file)
            except Exception:
                pass
        if os.path.exists(self.features_file):
            try:
                os.remove(self.features_file)
            except Exception:
                pass
        if os.path.exists(self.legacy_pkl_path):
            try:
                os.remove(self.legacy_pkl_path)
            except Exception:
                pass
        if os.path.exists(self.thumbnails_dir):
            for f in os.listdir(self.thumbnails_dir):
                try:
                    os.remove(os.path.join(self.thumbnails_dir, f))
                except Exception:
                    pass
        print("[FaceRecognizer] Memoria biometrica completamente vaciada.")
