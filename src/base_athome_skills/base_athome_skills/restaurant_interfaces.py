"""Adaptadores puros del contrato Restaurant (compatibilidad y memoria facial).

No importa ROS. La biometria solo vive en ~/.local/share/base_athome/restaurant_faces,
se borra al comenzar cada prueba y nunca usa la base del receptionist.
"""

import inspect
import math
from pathlib import Path
import re
import shutil
import time


def supported_kwargs(factory, kwargs):
    """Permite integrar el nodo antes que la nueva version de RestaurantSession."""
    params = inspect.signature(factory).parameters
    if any(p.kind == inspect.Parameter.VAR_KEYWORD for p in params.values()):
        return dict(kwargs)
    return {k: v for k, v in kwargs.items() if k in params and params[k].kind in (
        inspect.Parameter.POSITIONAL_OR_KEYWORD, inspect.Parameter.KEYWORD_ONLY)}


def reset_face_database(*, home=None):
    """Vacia SOLO el directorio dedicado, rechazando repositorios y enlaces simbolicos."""
    path = Path(home or Path.home()) / ".local/share/base_athome/restaurant_faces"
    for parent in (path, *path.parents):
        if parent.is_symlink():
            raise ValueError("La base facial Restaurant no admite enlaces simbolicos")
        marker = parent / ".git"
        if marker.is_file() or (marker / "HEAD").is_file():
            raise ValueError("La base facial Restaurant debe estar fuera del repositorio")
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    path.chmod(0o700)
    for child in path.iterdir():
        if child.is_symlink() or not child.is_dir():
            child.unlink()
        else:
            shutil.rmtree(child)
    return path


def person_map_point(person, default_height=1.2):
    """Admite tanto x/y del contrato como xy del detector actual; altura en map."""
    if not isinstance(person, dict):
        return None
    try:
        xy = person.get("xy")
        x, y = xy[:2] if xy is not None else (person["x"], person["y"])
        point = (float(x), float(y), float(person.get("face_z", default_height)))
        return point if all(math.isfinite(v) for v in point) else None
    except (KeyError, TypeError, ValueError):
        return None


def transform_point(point, transform):
    """Transformacion TF target<-source con cuaternion completo, sin dependencia ROS."""
    t, q = transform.translation, transform.rotation
    x, y, z, w = (float(q.x), float(q.y), float(q.z), float(q.w))
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("TF con cuaternion invalido")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    px, py, pz = point
    result = (
        (1-2*(y*y+z*z))*px + 2*(x*y-z*w)*py + 2*(x*z+y*w)*pz + t.x,
        2*(x*y+z*w)*px + (1-2*(x*x+z*z))*py + 2*(y*z-x*w)*pz + t.y,
        2*(x*z-y*w)*px + 2*(y*z+x*w)*py + (1-2*(x*x+y*y))*pz + t.z,
    )
    if not all(math.isfinite(v) for v in result):
        raise ValueError("TF con traslacion invalida")
    return result


def head_angles(point_in_head_base):
    x, y, z = point_in_head_base
    distance = math.hypot(x, y)
    if distance < 0.1:
        return None
    pan = math.degrees(math.atan2(y, x))
    # Eje tilt 8 cm sobre pan_tilt_base_link (head.xacro); positivo hacia arriba.
    tilt = math.degrees(math.atan2(z - 0.08, distance))
    if not (-90 <= pan <= 90 and -35 <= tilt <= 25):
        return None
    return pan, tilt


def make_face_recognizer(models_dir, db_path):
    from base_athome_perception.face_recognizer import FaceRecognizer

    class RestaurantRecognizer(FaceRecognizer):
        def load_database(self):
            # Base nueva por prueba: tampoco importar el faces_db.pkl legado del padre.
            self.profiles = {}

    return RestaurantRecognizer(
        caffe_proto_path=str(models_dir / "deploy.prototxt"),
        caffe_model_path=str(models_dir / "res10_300x300_ssd_iter_140000.caffemodel"),
        sface_model_path=str(models_dir / "face_recognition_sface_2021dec.onnx"),
        yunet_model_path=str(models_dir / "face_detection_yunet_2023mar.onnx"),
        db_path=str(db_path),
    )


class RestaurantFaces:
    """SFace del receptionist, muestras frescas y sin elegir entre caras ambiguas."""

    def __init__(self, recognizer_fn, aim_fn, frame_fn, *, active_fn=lambda: True,
                 clock_fn=time.monotonic, sleep_fn=time.sleep, warn_fn=lambda _: None):
        self.recognizer_fn = recognizer_fn
        self.aim = aim_fn
        self.frame = frame_fn  # -> (frame BGR, instante monotonic) o None
        self.active = active_fn
        self.clock = clock_fn
        self.sleep = sleep_fn
        self.warn = warn_fn
        self.recognizer = None
        self.labels = {}

    def reset(self, *, home=None):
        self.recognizer = None
        self.labels.clear()
        self.db_path = reset_face_database(home=home)

    def _samples(self, person, *, count):
        if not self.active() or (person is not None and not self.aim(person)):
            return []
        after = self.clock()  # No reutilizar cuadros anteriores al apuntado.
        if self.recognizer is None:
            self.recognizer = self.recognizer_fn(self.db_path)
        end = self.clock() + 6.0
        samples = []
        while self.active() and self.clock() < end:
            snapshot = self.frame()
            if snapshot is not None:
                image, stamp = snapshot
                if stamp > after and 0 <= self.clock() - stamp < 1.0:
                    after = stamp
                    faces = self.recognizer.detect_and_recognize(image)
                    # La cabeza apunta a la persona: vale la UNICA cara centrada aunque haya otras de
                    # fondo (prueba real 10/11: con alguien atras no se guardaba la cara del barman).
                    # Dos caras centradas es ambiguo: no se memoriza ninguna, ni la mas grande.
                    centered_faces = [f for f in faces
                                      if abs(f["bbox"][0] + f["bbox"][2] / 2 - image.shape[1] / 2) <= image.shape[1] * 0.20]
                    if len(centered_faces) == 1:
                        face = centered_faces[0]
                        if face.get("quality_valid") and face.get("feature") is not None:
                            if (not samples or self.recognizer.similarity(
                                    face["feature"], samples[0]["feature"]) >= self.recognizer.cosine_thresh):
                                samples.append(face)
                                if len(samples) >= count:
                                    return samples
            self.sleep(0.1)
        return []

    def enroll(self, label, person=None):
        if not re.fullmatch(r"barman|customer_[1-9][0-9]*", label):
            return False
        try:
            samples = self._samples(person, count=4)
            if not samples:
                return False
            pid, _ = self.recognizer.enroll_person(
                label, [f["feature"] for f in samples],
                attributes={"restaurant_label": label}, persistent=False,
            )
            if pid:
                self.labels[pid] = label  # FaceRecognizer title-casea nombres: conservar el label exacto.
            return bool(pid)
        except Exception as exc:
            self.warn(f"Restaurant: registro facial no disponible ({type(exc).__name__})")
            return False

    def identify(self, person):
        if not self.labels:
            return None
        try:
            samples = self._samples(person, count=3)
            if len(samples) != 3:
                return None
            ids = [f.get("person_id") for f in samples]
            if (ids[0] and len(set(ids)) == 1 and ids[0] in self.labels
                    and all(f.get("similarity", 0) >= self.recognizer.cosine_thresh for f in samples)):
                return self.labels[ids[0]]
        except Exception as exc:
            self.warn(f"Restaurant: identificacion facial no disponible ({type(exc).__name__})")
        return None
