#!/usr/bin/env python3
# ==============================================================================
# Base@Home - Memoria biometrica y reconocimiento facial multi-persona
#
# Pipeline (como ros4hri hri_face_detect + identificacion, Apache-2.0, solo diseno):
#   YuNet (caja + 5 puntos) -> alineacion SFace.alignCrop -> embedding SFace 128-D
#   -> similitud coseno contra perfiles. Si YuNet no esta disponible se usa el
#   detector SSD ResNet-10 sin alineacion (menos preciso).
#
# Memoria en dos niveles:
#   - persistente: personas que deben recordarse entre ejecuciones (p.ej. el host)
#   - temporal (de sesion): invitados de la prueba en curso. Solo se comparan los
#     perfiles persistentes y los de la sesion actual, asi un invitado de una
#     corrida anterior no se confunde con uno nuevo. clear_session() los borra.
# ==============================================================================

from dataclasses import dataclass, field
import datetime
import json
import os
import pickle
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np

MAX_SAMPLES = 15
PLACEHOLDER_NAMES = {"", "desconocido", "unknown", "guest", "me llamo", "followed by"}
DEFAULT_DB_DIR = os.path.expanduser("~/.local/share/base_athome/faces_database")


def _now_str(ts: float) -> str:
    return datetime.datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")


def _valid_feat(f) -> bool:
    return f is not None and getattr(f, "size", 0) == 128


def _unit(f: np.ndarray) -> np.ndarray:
    v = f.reshape(1, 128).astype(np.float32)
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-6 else v


@dataclass
class PersonProfile:
    """Perfil biometrico y de memoria de una persona."""

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
    centroid: Optional[np.ndarray] = field(default=None, repr=False)
    thumbnail_img: Optional[np.ndarray] = field(default=None, repr=False)
    _last_sighting_recorded: float = field(default=0.0, repr=False)

    @property
    def persistent(self) -> bool:
        return bool(self.attributes.get("persistent", False))

    @property
    def session_id(self) -> str:
        return str(self.attributes.get("session_id", ""))

    def update_centroid(self) -> None:
        """Vector medio normalizado de las muestras validas."""
        self.features = [f.reshape(1, 128).astype(np.float32) for f in self.features if _valid_feat(f)]
        if self.features:
            mean_vec = np.mean(np.vstack([_unit(f) for f in self.features]), axis=0, keepdims=True)
            norm = float(np.linalg.norm(mean_vec))
            self.centroid = (mean_vec / norm).astype(np.float32) if norm > 1e-6 else self.features[0].copy()
        else:
            self.centroid = None
        self.samples_count = len(self.features)

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
    """Deteccion, reconocimiento y memoria facial multi-persona (thread-safe)."""

    def __init__(
        self,
        caffe_proto_path: str,
        caffe_model_path: str,
        sface_model_path: str,
        db_path: str = DEFAULT_DB_DIR,
        conf_thresh: float = 0.55,
        cosine_thresh: Optional[float] = None,
        yunet_model_path: Optional[str] = None,
        session_id: Optional[str] = None,
    ):
        self.caffe_proto_path = caffe_proto_path
        self.caffe_model_path = caffe_model_path
        self.sface_model_path = sface_model_path
        self.conf_thresh = conf_thresh
        self.lock = threading.RLock()

        # Directorio de persistencia (fuera del repo por defecto: contiene biometria)
        if db_path.endswith(".pkl") or db_path.endswith(".json"):
            self.db_dir = os.path.join(os.path.dirname(os.path.abspath(db_path)), "faces_database")
            self.legacy_pkl_path = db_path
        else:
            self.db_dir = os.path.abspath(db_path)
            self.legacy_pkl_path = os.path.join(os.path.dirname(self.db_dir), "faces_db.pkl")
        self.metadata_file = os.path.join(self.db_dir, "profiles.json")
        self.features_file = os.path.join(self.db_dir, "features.pkl")
        self.thumbnails_dir = os.path.join(self.db_dir, "thumbnails")
        os.makedirs(self.thumbnails_dir, exist_ok=True)

        # 1. Detector: YuNet (con 5 landmarks para alinear) o SSD ResNet-10 de respaldo
        self.yunet = None
        self.yunet_size = (0, 0)
        for path in self._yunet_candidates(yunet_model_path, caffe_proto_path):
            try:
                self.yunet = cv2.FaceDetectorYN.create(path, "", (320, 320), conf_thresh, 0.3, 50)
                self.yunet.detect(np.zeros((320, 320, 3), np.uint8))  # falla aqui si el modelo no es compatible
                self.yunet_model_path = path
                break
            except Exception:
                self.yunet = None
        self.detector_net = None
        if self.yunet is None:
            if not os.path.exists(caffe_proto_path) or not os.path.exists(caffe_model_path):
                raise FileNotFoundError(f"Ni YuNet ni el detector Caffe disponibles: {caffe_proto_path}")
            self.detector_net = cv2.dnn.readNetFromCaffe(caffe_proto_path, caffe_model_path)

        # 2. Extractor SFace 128-D
        if not os.path.exists(sface_model_path):
            raise FileNotFoundError(f"Modelo SFace no encontrado: {sface_model_path}")
        self.sface = cv2.FaceRecognizerSF.create(sface_model_path, "")

        # Umbral coseno: OpenCV Zoo recomienda 0.363 con caras alineadas; con recorte
        # sin alinear las similitudes son mas bajas y ruidosas -> umbral mas alto.
        if cosine_thresh is None:
            cosine_thresh = 0.40 if self.aligned else 0.52
        self.cosine_thresh = cosine_thresh

        self.session_id = session_id or f"session_{datetime.datetime.now():%Y%m%d_%H%M%S}"
        self.profiles: Dict[str, PersonProfile] = {}
        self.load_database()

    # ------------------------------------------------------------------ util
    @staticmethod
    def _yunet_candidates(explicit: Optional[str], caffe_proto_path: str) -> List[str]:
        models_dir = os.path.dirname(os.path.abspath(caffe_proto_path))
        names = ["face_detection_yunet_2023mar.onnx", "face_detection_yunet_2022mar.onnx"]
        cands = [explicit] if explicit else []
        cands += [os.path.join(models_dir, n) for n in names]
        return [c for c in cands if c and os.path.exists(c)]

    @property
    def aligned(self) -> bool:
        return self.yunet is not None

    @property
    def known_faces(self) -> Dict[str, List[np.ndarray]]:
        """Compatibilidad retroactiva."""
        with self.lock:
            return {p.name: list(p.features) for p in self.profiles.values()}

    def _in_scope(self, p: PersonProfile) -> bool:
        """Perfiles que participan del reconocimiento: persistentes o de la sesion actual."""
        return p.persistent or p.session_id == self.session_id

    def session_profiles(self) -> List[PersonProfile]:
        with self.lock:
            return [p for p in self.profiles.values() if self._in_scope(p)]

    # ----------------------------------------------------------- persistencia
    def load_database(self) -> None:
        with self.lock:
            self.profiles = {}
            if os.path.exists(self.metadata_file) and os.path.exists(self.features_file):
                try:
                    with open(self.metadata_file, "r", encoding="utf-8") as f:
                        meta_dict = json.load(f)
                    with open(self.features_file, "rb") as f:
                        feats_dict = pickle.load(f)
                    for pid, meta in meta_dict.items():
                        thumb_img = None
                        thumb_path = os.path.join(self.db_dir, meta.get("thumbnail_file", "") or "_")
                        if os.path.isfile(thumb_path):
                            thumb_img = cv2.imread(thumb_path)
                        attrs = meta.get("attributes", {}) or {}
                        # Perfiles anteriores a la memoria de sesion: host persistente, resto temporal viejo
                        attrs.setdefault("persistent", attrs.get("role") == "host")
                        attrs.setdefault("session_id", "legacy")
                        profile = PersonProfile(
                            person_id=pid,
                            name=meta["name"],
                            enrolled_at=meta.get("enrolled_at", time.time()),
                            enrolled_date=meta.get("enrolled_date", ""),
                            last_seen=meta.get("last_seen", time.time()),
                            last_seen_date=meta.get("last_seen_date", ""),
                            sighting_count=meta.get("sighting_count", 1),
                            thumbnail_file=meta.get("thumbnail_file", ""),
                            attributes=attrs,
                            features=list(feats_dict.get(pid, [])),
                            thumbnail_img=thumb_img,
                            _last_sighting_recorded=meta.get("last_seen", time.time()),
                        )
                        profile.update_centroid()
                        self.profiles[pid] = profile
                    total = sum(len(p.features) for p in self.profiles.values())
                    print(f"[FaceRecognizer] Memoria cargada: {len(self.profiles)} perfiles, {total} muestras "
                          f"({len(self.session_profiles())} activos en {self.session_id}).")
                    return
                except Exception as e:
                    print(f"[FaceRecognizer] Error cargando memoria ({e}); se intenta el formato legado.")

            if self.legacy_pkl_path.endswith(".pkl") and os.path.exists(self.legacy_pkl_path):
                try:
                    with open(self.legacy_pkl_path, "rb") as f:
                        legacy = pickle.load(f)
                    if isinstance(legacy, dict):
                        now = time.time()
                        for i, (name, feats) in enumerate(legacy.items(), start=1):
                            if name.strip().lower() in PLACEHOLDER_NAMES:
                                continue
                            pid = f"person_{i:03d}"
                            prof = PersonProfile(pid, name, now, _now_str(now), now, _now_str(now),
                                                 attributes={"persistent": False, "session_id": "legacy"},
                                                 features=feats if isinstance(feats, list) else [feats],
                                                 _last_sighting_recorded=now)
                            prof.update_centroid()
                            self.profiles[pid] = prof
                        self.save_database()
                        print(f"[FaceRecognizer] Migrados {len(self.profiles)} perfiles del formato legado.")
                except Exception as e:
                    print(f"[FaceRecognizer] Error migrando archivo legado: {e}")

    @staticmethod
    def _atomic_write(path: str, data: bytes) -> None:
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)

    def save_database(self) -> None:
        """Persiste perfiles (JSON) y vectores (PKL) de forma atomica."""
        with self.lock:
            try:
                os.makedirs(self.thumbnails_dir, exist_ok=True)
                meta = {pid: p.to_metadata_dict() for pid, p in self.profiles.items()}
                feats = {pid: p.features for pid, p in self.profiles.items()}
                self._atomic_write(self.metadata_file, json.dumps(meta, indent=2, ensure_ascii=False).encode("utf-8"))
                self._atomic_write(self.features_file, pickle.dumps(feats))
            except Exception as e:
                print(f"[FaceRecognizer] Error persistiendo base de datos: {e}")

    def save_metadata_only(self) -> None:
        with self.lock:
            try:
                meta = {pid: p.to_metadata_dict() for pid, p in self.profiles.items()}
                self._atomic_write(self.metadata_file, json.dumps(meta, indent=2, ensure_ascii=False).encode("utf-8"))
            except Exception:
                pass

    # ------------------------------------------------------------- vision
    @staticmethod
    def assess_face_quality(image: np.ndarray, bbox: Tuple[int, int, int, int]) -> Tuple[bool, float, str]:
        """Calidad de una deteccion (tamano, bordes, nitidez)."""
        x, y, w, h = bbox
        if w < 50 or h < 50:
            return False, 0.0, "Rostro demasiado pequeno"
        img_h, img_w = image.shape[:2]
        if x < 0 or y < 0 or (x + w) > img_w or (y + h) > img_h:
            return False, 0.0, "Rostro fuera de los margenes"
        crop = image[y:y + h, x:x + w]
        if crop.size == 0:
            return False, 0.0, "Recorte vacio"
        lap_var = float(cv2.Laplacian(cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
        if lap_var < 30.0:
            return False, lap_var, "Muestra borrosa (desenfoque)"
        return True, lap_var, "Excelente calidad"

    def _detect(self, image: np.ndarray) -> List[Tuple[Tuple[int, int, int, int, float], Optional[np.ndarray]]]:
        """[(x, y, w, h, score), fila YuNet de 15 valores o None]."""
        h, w = image.shape[:2]
        out = []
        if self.yunet is not None:
            if self.yunet_size != (w, h):
                self.yunet.setInputSize((w, h))
                self.yunet_size = (w, h)
            _, faces = self.yunet.detect(image)
            if faces is not None:
                for row in faces:
                    x, y, bw, bh = (int(v) for v in row[:4])
                    x1, y1 = max(0, x), max(0, y)
                    x2, y2 = min(w, x + bw), min(h, y + bh)
                    if x2 - x1 > 25 and y2 - y1 > 25:
                        out.append(((x1, y1, x2 - x1, y2 - y1, float(row[14])), row))
            return out
        blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.detector_net.setInput(blob)
        det = self.detector_net.forward()
        for i in range(det.shape[2]):
            conf = float(det[0, 0, i, 2])
            if conf > self.conf_thresh:
                x1, y1, x2, y2 = (det[0, 0, i, 3:7] * np.array([w, h, w, h])).astype(int)
                x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
                if x2 - x1 > 25 and y2 - y1 > 25:
                    out.append(((x1, y1, x2 - x1, y2 - y1, conf), None))
        return out

    def detect_faces(self, image: np.ndarray) -> List[Tuple[int, int, int, int, float]]:
        """Compatibilidad: lista de (x, y, w, h, score)."""
        return [box for box, _ in self._detect(image)]

    def extract_feature(self, image: np.ndarray, bbox: Tuple[int, int, int, int],
                        landmarks: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """Embedding SFace. Con landmarks YuNet usa alignCrop (recomendado por OpenCV)."""
        if landmarks is not None:
            try:
                return self.sface.feature(self.sface.alignCrop(image, landmarks))
            except Exception:
                pass
        x, y, w, h = bbox
        img_h, img_w = image.shape[:2]
        side = int(max(w, h) * 1.12)
        half = side // 2
        x1, y1 = int(x + w / 2.0 - half), int(y + h / 2.0 - half)
        x2, y2 = x1 + side, y1 + side
        crop = image[max(0, y1):min(img_h, y2), max(0, x1):min(img_w, x2)]
        pads = (max(0, -y1), max(0, y2 - img_h), max(0, -x1), max(0, x2 - img_w))
        if any(pads):
            crop = cv2.copyMakeBorder(crop, *pads, cv2.BORDER_CONSTANT, value=[0, 0, 0])
        if crop.size == 0 or crop.shape[0] < 20 or crop.shape[1] < 20:
            return None
        return self.sface.feature(cv2.resize(crop, (112, 112)))

    def similarity(self, a: np.ndarray, b: np.ndarray) -> float:
        return float(self.sface.match(a.reshape(1, 128).astype(np.float32), b.reshape(1, 128).astype(np.float32),
                                      cv2.FaceRecognizerSF_FR_COSINE))

    def _profile_score(self, feature: np.ndarray, profile: PersonProfile) -> float:
        scores = sorted((self.similarity(feature, kf) for kf in profile.features if _valid_feat(kf)), reverse=True)
        if not scores:
            return -1.0
        top_k = float(np.mean(scores[:min(2, len(scores))]))  # robusto a un outlier
        if profile.centroid is not None:
            return 0.60 * top_k + 0.40 * self.similarity(feature, profile.centroid)
        return top_k

    def recognize_face(self, feature: np.ndarray) -> Tuple[str, float, Optional[str]]:
        """(nombre, similitud, person_id) del mejor perfil en alcance, o 'Desconocido'."""
        if not _valid_feat(feature):
            return "Desconocido", 0.0, None
        best, best_score = None, -1.0
        with self.lock:
            candidates = [p for p in self.profiles.values()
                          if self._in_scope(p) and p.features and not p.attributes.get("synthetic")]
            for p in candidates:
                s = self._profile_score(feature, p)
                if s > best_score:
                    best, best_score = p, s
        if best is not None and best_score >= self.cosine_thresh:
            return best.name, max(0.0, best_score), best.person_id
        return "Desconocido", max(0.0, best_score), None

    def detect_and_recognize(self, image: np.ndarray) -> List[Dict]:
        results = []
        for (x, y, w, h, score), lm in self._detect(image):
            feature = self.extract_feature(image, (x, y, w, h), lm)
            ok, q_score, q_reason = self.assess_face_quality(image, (x, y, w, h))
            name, sim, pid = self.recognize_face(feature) if feature is not None else ("Desconocido", 0.0, None)
            results.append({
                "bbox": [x, y, w, h], "det_score": score, "name": name, "similarity": sim,
                "person_id": pid, "feature": feature, "landmarks": lm,
                "quality_valid": ok, "quality_score": q_score, "quality_reason": q_reason,
            })
        return results

    # ------------------------------------------------------------- memoria
    def _new_pid(self) -> str:
        i = len(self.profiles) + 1
        while f"person_{i:03d}" in self.profiles:
            i += 1
        return f"person_{i:03d}"

    def _write_thumbnail(self, pid: str, img: Optional[np.ndarray]) -> Tuple[str, Optional[np.ndarray]]:
        if img is None or getattr(img, "size", 0) == 0:
            return "", None
        thumb = cv2.resize(img, (140, 140))
        rel = f"thumbnails/{pid}.jpg"
        cv2.imwrite(os.path.join(self.db_dir, rel), thumb)
        return rel, thumb

    def enroll_person(
        self,
        name: str,
        features: List[np.ndarray],
        thumbnail_img: Optional[np.ndarray] = None,
        attributes: Optional[Dict] = None,
        persistent: bool = False,
    ) -> Tuple[str, bool]:
        """Registra o actualiza una persona. Devuelve (person_id, es_nuevo).

        - Solo se fusiona con un perfil del mismo nombre que este en alcance
          (persistente o de esta sesion); un homonimo de una corrida anterior no se toca.
        - Si el homonimo en alcance tiene una cara distinta (similitud < umbral) se
          crea otro perfil: el nombre pudo entenderse mal y no hay que mezclar caras.
        - Muestras sinteticas nunca se mezclan con reales.
        """
        name = (name or "").strip().title()
        feats = [f for f in features if _valid_feat(f)]
        if not name or name.lower() in PLACEHOLDER_NAMES or not feats:
            return "", False
        attrs = dict(attributes or {})
        attrs["persistent"] = bool(persistent or attrs.get("persistent", False))
        attrs["session_id"] = self.session_id
        incoming_synthetic = bool(attrs.get("synthetic"))
        now = time.time()

        with self.lock:
            existing = next((p for p in self.profiles.values()
                             if p.name.lower() == name.lower() and self._in_scope(p)), None)
            if existing is not None and existing.features and not incoming_synthetic \
                    and not existing.attributes.get("synthetic"):
                probe = PersonProfile("probe", name, now, "", now, "", features=list(feats))
                probe.update_centroid()
                if self._profile_score(probe.centroid, existing) < self.cosine_thresh:
                    print(f"[FaceRecognizer] '{name}' ya existe con otra cara: se crea un perfil separado.")
                    existing = None

            if existing is not None:
                p = existing
                was_synthetic = bool(p.attributes.get("synthetic"))
                if was_synthetic and not incoming_synthetic:
                    p.features = []  # llegan muestras reales: descartar las sinteticas
                elif incoming_synthetic and not was_synthetic:
                    feats = []  # nunca mezclar sinteticas en un perfil real
                p.attributes.update(attrs)
                p.attributes["synthetic"] = was_synthetic and incoming_synthetic
                p.attributes["persistent"] = p.persistent or attrs["persistent"]
                p.features = (p.features + feats)[-MAX_SAMPLES:]  # FIFO: conserva las mas recientes
                p.update_centroid()
                p.last_seen, p.last_seen_date = now, _now_str(now)
                p.sighting_count += 1
                rel, thumb = self._write_thumbnail(p.person_id, thumbnail_img)
                if rel:
                    p.thumbnail_file, p.thumbnail_img = rel, thumb
                self.save_database()
                print(f"[FaceRecognizer] Perfil actualizado '{name}' ({len(p.features)} muestras).")
                return p.person_id, False

            pid = self._new_pid()
            rel, thumb = self._write_thumbnail(pid, thumbnail_img)
            p = PersonProfile(pid, name, now, _now_str(now), now, _now_str(now), thumbnail_file=rel,
                              attributes=attrs, features=feats[-MAX_SAMPLES:], thumbnail_img=thumb,
                              _last_sighting_recorded=now)
            p.update_centroid()
            self.profiles[pid] = p
            self.save_database()
            scope = "persistente" if p.persistent else f"temporal ({self.session_id})"
            print(f"[FaceRecognizer] Nuevo perfil '{name}' [{pid}] {scope}.")
            return pid, True

    def enroll_face(self, image: np.ndarray, name: str, persistent: bool = False) -> bool:
        dets = self._detect(image)
        if not dets:
            return False
        (x, y, w, h, _), lm = max(dets, key=lambda d: d[0][2] * d[0][3])
        feature = self.extract_feature(image, (x, y, w, h), lm)
        if feature is None:
            return False
        crop = image[max(0, y):y + h, max(0, x):x + w]
        pid, _ = self.enroll_person(name, [feature], thumbnail_img=crop, persistent=persistent)
        return bool(pid)

    def enroll_feature(self, feature: np.ndarray, name: str) -> bool:
        if feature is None:
            return False
        pid, _ = self.enroll_person(name, [feature])
        return bool(pid)

    def update_sighting(self, identifier: str, min_interval_sec: float = 20.0) -> bool:
        with self.lock:
            target = self.profiles.get(identifier) or next(
                (p for p in self.profiles.values() if p.name.lower() == identifier.lower() and self._in_scope(p)), None)
            if target is None:
                return False
            now = time.time()
            target.last_seen, target.last_seen_date = now, _now_str(now)
            if now - target._last_sighting_recorded >= min_interval_sec:
                target.sighting_count += 1
                target._last_sighting_recorded = now
                self.save_metadata_only()
                return True
            return False

    def get_known_profiles(self, scope_only: bool = True) -> List[Dict[str, Any]]:
        """Resumen de perfiles (por defecto solo los activos: persistentes + sesion actual)."""
        with self.lock:
            profs = [p for p in self.profiles.values() if self._in_scope(p) or not scope_only]
            return [{
                "id": p.person_id, "name": p.name, "enrolled_date": p.enrolled_date,
                "last_seen": p.last_seen, "last_seen_date": p.last_seen_date,
                "sighting_count": p.sighting_count, "samples_count": len(p.features),
                "thumbnail_img": p.thumbnail_img, "attributes": p.attributes,
                "persistent": p.persistent,
            } for p in profs]

    def _remove(self, pid: str) -> Optional[PersonProfile]:
        p = self.profiles.pop(pid, None)
        if p and p.thumbnail_file:
            try:
                os.remove(os.path.join(self.db_dir, p.thumbnail_file))
            except OSError:
                pass
        return p

    def delete_person(self, identifier: str) -> bool:
        with self.lock:
            pid = identifier if identifier in self.profiles else next(
                (p.person_id for p in self.profiles.values() if p.name.lower() == identifier.lower()), None)
            if pid is None:
                return False
            p = self._remove(pid)
            self.save_database()
            print(f"[FaceRecognizer] Perfil '{p.name}' eliminado.")
            return True

    def clear_session(self) -> int:
        """Borra la memoria temporal (todos los perfiles no persistentes). Devuelve cuantos borro."""
        with self.lock:
            temp = [pid for pid, p in self.profiles.items() if not p.persistent]
            for pid in temp:
                self._remove(pid)
            self.save_database()
        print(f"[FaceRecognizer] Memoria temporal vaciada: {len(temp)} perfiles.")
        return len(temp)

    def clear_database(self) -> None:
        """Borra todos los perfiles (persistentes incluidos) y sus miniaturas."""
        with self.lock:
            self.profiles.clear()
            for path in (self.metadata_file, self.features_file, self.legacy_pkl_path):
                try:
                    os.remove(path)
                except OSError:
                    pass
            if os.path.isdir(self.thumbnails_dir):
                for f in os.listdir(self.thumbnails_dir):
                    try:
                        os.remove(os.path.join(self.thumbnails_dir, f))
                    except OSError:
                        pass
        print("[FaceRecognizer] Memoria biometrica completamente vaciada.")
