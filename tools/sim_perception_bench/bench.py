#!/usr/bin/env python3
"""Benchmark de realismo de percepcion para la simulacion (Gazebo Harmonic / ROS 2 Jazzy).

README corto
------------
Toma N imagenes de la camara de la cabeza (`/camera/camera/color/image_raw`) en un
dominio ROS aislado y corre el MISMO detector de poses del robot
(`yolov8n-pose.pt` + `base_athome_perception.bag_vision.yolo_people`). Tambien
puede correr offline sobre una carpeta de imagenes para comparar simulacion con
fotos reales. Reporta por cuadro y agregado en JSON y TSV: personas, confianza
media, keypoints visibles, manos levantadas (`raised_hand.classify_raised_hand`)
y caras detectadas si hay modelo disponible.

Uso:

    # Simulacion / robot (dominio aislado; nunca ROS_DOMAIN_ID=10)
    ~/ros2_vision_venv/bin/python3 tools/sim_perception_bench/bench.py \
        --domain 42 --topic /camera/camera/color/image_raw --frames 30 \
        --out /tmp/bench_sim

    # Offline sobre una carpeta de imagenes
    ~/ros2_vision_venv/bin/python3 tools/sim_perception_bench/bench.py \
        --images ~/fotos_reales --out /tmp/bench_real

    # Tests unitarios (no requieren ROS ni los pesos)
    ~/ros2_vision_venv/bin/python3 -m pytest -q tools/sim_perception_bench

Salidas: `<out>.json` y `<out>.tsv` (o `--out-json`/`--out-tsv`). Con
`--save-images DIR` guarda los cuadros analizados como PNG para inspeccion.

Entorno: usar `~/ros2_vision_venv/bin/python3` (ultralytics + cv2 + numpy 1.26).
No instalar con pip ahí: numpy 2 rompe cv2. El sistema Python no trae ultralytics.

Licencias de los modelos (no se versionan ni se redistribuyen; se descargan con
`src/base_athome_perception/scripts/download_models.sh`):
  - `yolov8n-pose.pt`: Ultralytics YOLOv8n-pose, AGPL-3.0 (mismo peso que usa el robot).
  - `face_detection_yunet_2023mar.onnx`: YuNet de OpenCV Zoo, MIT.
  - `deploy.prototxt` + `res10_300x300_ssd_iter_140000.caffemodel`: detector SSD
    ResNet-10 de OpenCV, Apache-2.0.
"""
import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

DEFAULT_TOPIC = "/camera/camera/color/image_raw"
DEFAULT_POSE_MODEL = "yolov8n-pose.pt"
DEFAULT_CONF = 0.45          # el mismo umbral que bag_finder en el robot
DEFAULT_KP_CONF = 0.40       # MIN_KP_CONF de bag_vision / raised_hand
FACE_YUNET_NAMES = ("face_detection_yunet_2023mar.onnx", "face_detection_yunet_2022mar.onnx")
FACE_CAFFE_PROTO = "deploy.prototxt"
FACE_CAFFE_WEIGHTS = "res10_300x300_ssd_iter_140000.caffemodel"
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".webp", ".tif", ".tiff"}
TSV_COLUMNS = (
    "frame", "source", "persons", "mean_conf", "visible_keypoints",
    "raised_hands", "raised_left", "raised_right", "raised_both", "faces",
)


def _ensure_src_paths() -> None:
    """Permite importar el paquete sin depender de un build/instalacion previos."""
    repo = Path(__file__).resolve().parents[2]
    for name in ("base_athome_perception", "base_athome_skills"):
        path = repo / "src" / name
        if path.is_dir() and str(path) not in sys.path:
            sys.path.insert(0, str(path))


_ensure_src_paths()

import cv2  # noqa: E402  (despues del bootstrap de sys.path)

from base_athome_perception import bag_vision  # noqa: E402
from base_athome_perception.raised_hand import (  # noqa: E402
    RaisedHandState,
    classify_raised_hand,
)


# --------------------------------------------------------------------------- modelos
def _model_candidates() -> List[Path]:
    repo = Path(__file__).resolve().parents[2]
    dirs = [
        repo / "src" / "base_athome_perception" / "models",
        Path.home() / "base_athome_ws" / "src" / "base_athome_perception" / "models",
    ]
    install = Path.home() / "base_athome_ws" / "install" / "base_athome_perception"
    dirs.extend(sorted(install.glob("share/base_athome_perception/models")))
    dirs.extend([repo, repo / "tools" / "sim_perception_bench", Path.home() / "base_athome_ws"])
    env_dir = os.environ.get("BASE_ATHOME_PERCEPTION_MODELS")
    if env_dir:
        dirs.insert(0, Path(env_dir).expanduser())
    return [d for d in dirs if d]


def find_model(name: str, models_dir: Optional[str] = None) -> Optional[Path]:
    if models_dir:
        path = Path(models_dir).expanduser() / name
        if path.is_file():
            return path
    for directory in _model_candidates():
        path = directory / name
        if path.is_file():
            return path
    return None


def resolve_pose_model(explicit: Optional[str], models_dir: Optional[str]) -> Path:
    if explicit:
        path = Path(explicit).expanduser()
        if not path.is_file():
            raise SystemExit(f"No existe el modelo de pose: {path}")
        return path
    path = find_model(DEFAULT_POSE_MODEL, models_dir)
    if path is None:
        raise SystemExit(
            "No encontre yolov8n-pose.pt. Pasa --pose-model o --models-dir; "
            "se descarga con src/base_athome_perception/scripts/download_models.sh"
        )
    return path


@dataclass
class FaceDetector:
    """Detector de caras opcional: YuNet (preferido) o SSD ResNet-10 de OpenCV."""

    models_dir: Optional[str] = None
    conf_thresh: float = 0.50
    enabled: bool = True
    name: str = ""

    def __post_init__(self) -> None:
        self._yunet = None
        self._caffe = None
        self._yunet_size: Tuple[int, int] = (0, 0)
        if not self.enabled:
            return
        for model_name in FACE_YUNET_NAMES:
            path = find_model(model_name, self.models_dir)
            if path is None:
                continue
            try:
                detector = cv2.FaceDetectorYN.create(str(path), "", (320, 320), self.conf_thresh, 0.3, 50)
                detector.detect(np.zeros((320, 320, 3), np.uint8))
            except Exception:
                continue
            self._yunet = detector
            self.name = model_name
            break
        if self._yunet is None:
            proto = find_model(FACE_CAFFE_PROTO, self.models_dir)
            weights = find_model(FACE_CAFFE_WEIGHTS, self.models_dir)
            if proto is not None and weights is not None:
                try:
                    self._caffe = cv2.dnn.readNetFromCaffe(str(proto), str(weights))
                    self.name = FACE_CAFFE_WEIGHTS
                except Exception:
                    self._caffe = None

    @property
    def available(self) -> bool:
        return self._yunet is not None or self._caffe is not None

    def detect(self, image: np.ndarray) -> List[Tuple[int, int, int, int, float]]:
        """[(x, y, w, h, score)] en pixeles de `image` (BGR)."""
        if not self.available or image is None or image.size == 0:
            return []
        h, w = image.shape[:2]
        if self._yunet is not None:
            if self._yunet_size != (w, h):
                self._yunet.setInputSize((w, h))
                self._yunet_size = (w, h)
            _, faces = self._yunet.detect(image)
            out = []
            if faces is None:
                return out
            for row in faces:
                x, y, bw, bh = (int(v) for v in row[:4])
                x1, y1 = max(0, x), max(0, y)
                x2, y2 = min(w, x + bw), min(h, y + bh)
                if x2 - x1 > 0 and y2 - y1 > 0:
                    out.append((x1, y1, x2 - x1, y2 - y1, float(row[14])))
            return out
        blob = cv2.dnn.blobFromImage(cv2.resize(image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self._caffe.setInput(blob)
        det = self._caffe.forward()
        out = []
        for i in range(det.shape[2]):
            score = float(det[0, 0, i, 2])
            if score < self.conf_thresh:
                continue
            x1, y1, x2, y2 = (det[0, 0, i, 3:7] * np.array([w, h, w, h])).astype(int)
            x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
            if x2 - x1 > 0 and y2 - y1 > 0:
                out.append((x1, y1, x2 - x1, y2 - y1, score))
        return out


# --------------------------------------------------------------------------- deteccion
@dataclass
class PersonDetection:
    box: np.ndarray
    score: float
    keypoints: np.ndarray
    keypoints_conf: np.ndarray

    def visible_keypoints(self, threshold: float = DEFAULT_KP_CONF) -> int:
        return int(np.sum(np.asarray(self.keypoints_conf, dtype=float) >= threshold))


def people_from_result(result: Any) -> List[PersonDetection]:
    """Salida de YOLO pose -> personas ordenadas por area, con su confianza de caja."""
    if result is None or result.boxes is None or result.keypoints is None:
        return []
    people = bag_vision.yolo_people(result)
    if not people:
        return []
    boxes = result.boxes.xyxy.cpu().numpy()
    scores = result.boxes.conf.cpu().numpy()
    order = np.argsort(-((boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])), kind="stable")
    scores = scores[order]
    out = []
    for (box, keypoints, keypoints_conf), score in zip(people, scores):
        out.append(PersonDetection(
            box=np.asarray(box, dtype=float),
            score=float(score),
            keypoints=np.asarray(keypoints, dtype=float),
            keypoints_conf=np.asarray(keypoints_conf, dtype=float),
        ))
    return out


def analyze_frame(index: int, source: str, image: np.ndarray, model: Any,
                  face_detector: FaceDetector, conf: float = DEFAULT_CONF,
                  kp_conf: float = DEFAULT_KP_CONF) -> Dict[str, Any]:
    result = model.predict(image, conf=conf, verbose=False, device="cpu")[0]
    return frame_record(index, source, people_from_result(result),
                        face_detector.detect(image), kp_conf=kp_conf)


def frame_record(index: int, source: str, people: Sequence[PersonDetection],
                 faces: Sequence[Sequence[float]] = (), kp_conf: float = DEFAULT_KP_CONF) -> Dict[str, Any]:
    raised = {"hands": 0, RaisedHandState.LEFT: 0, RaisedHandState.RIGHT: 0, RaisedHandState.BOTH: 0}
    detail = []
    visible_total = 0
    for person in people:
        visible = person.visible_keypoints(kp_conf)
        visible_total += visible
        cls = classify_raised_hand(person.keypoints, person.keypoints_conf, conf_thresh=kp_conf)
        if cls.is_raised:
            raised["hands"] += 1
            raised[cls.state] += 1
        detail.append({
            "box": [round(float(v), 1) for v in person.box],
            "score": round(person.score, 4),
            "visible_keypoints": visible,
            "raised": bool(cls.is_raised),
            "hand": cls.state,
            "left_wrist_conf": round(float(cls.left_wrist_conf), 4),
            "right_wrist_conf": round(float(cls.right_wrist_conf), 4),
        })
    mean_conf = float(np.mean([p.score for p in people])) if people else 0.0
    return {
        "frame": int(index),
        "source": source,
        "persons": len(people),
        "mean_conf": round(mean_conf, 4),
        "visible_keypoints": int(visible_total),
        "raised_hands": raised["hands"],
        "raised_left": raised[RaisedHandState.LEFT],
        "raised_right": raised[RaisedHandState.RIGHT],
        "raised_both": raised[RaisedHandState.BOTH],
        "faces": len(faces),
        "people": detail,
    }


def summarize(frames: Sequence[Dict[str, Any]]) -> Dict[str, Any]:
    n = len(frames)
    persons_total = sum(f["persons"] for f in frames)
    raised_total = sum(f["raised_hands"] for f in frames)
    faces_total = sum(f["faces"] for f in frames)
    conf_weighted = (sum(f["mean_conf"] * f["persons"] for f in frames) / persons_total) if persons_total else 0.0
    return {
        "frames": n,
        "persons_total": int(persons_total),
        "persons_per_frame_mean": round(persons_total / n, 4) if n else 0.0,
        "mean_conf": round(conf_weighted, 4),
        "visible_keypoints_mean": round(sum(f["visible_keypoints"] for f in frames) / n, 4) if n else 0.0,
        "frames_with_person": sum(1 for f in frames if f["persons"] > 0),
        "frames_with_raised_hand": sum(1 for f in frames if f["raised_hands"] > 0),
        "frames_with_face": sum(1 for f in frames if f["faces"] > 0),
        "raised_hand_frames": int(raised_total),
        "faces_total": int(faces_total),
    }


# --------------------------------------------------------------------------- E/S
def image_to_bgr(message: Any) -> Optional[np.ndarray]:
    """sensor_msgs/Image -> BGR (o None si el formato no se entiende)."""
    encoding = str(getattr(message, "encoding", "")).lower()
    height, width = int(message.height), int(message.width)
    if height <= 0 or width <= 0:
        return None
    if encoding in ("rgb8", "bgr8"):
        channels = 3
    elif encoding in ("rgba8", "bgra8"):
        channels = 4
    elif encoding in ("mono8", "8uc1"):
        channels = 1
    elif encoding in ("8uc3",):
        channels = 3
    else:
        return None
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8)
    step = int(getattr(message, "step", 0)) or width * channels
    try:
        rows = raw.reshape(height, step)[:, :width * channels]
        image = rows.reshape(height, width, channels)
    except ValueError:
        return None
    if channels == 1:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if channels == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image.copy()


def load_images(directory: str) -> List[Tuple[str, np.ndarray]]:
    folder = Path(directory).expanduser()
    if not folder.is_dir():
        raise SystemExit(f"No es una carpeta: {folder}")
    out = []
    for path in sorted(folder.iterdir()):
        if path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        image = cv2.imread(str(path))
        if image is not None:
            out.append((str(path), image))
    return out


def capture_ros(topic: str, domain: int, frames: int, timeout: float) -> List[Tuple[str, np.ndarray]]:
    os.environ["ROS_DOMAIN_ID"] = str(domain)
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        raise SystemExit(f"rclpy no esta disponible; source /opt/ros/jazzy/setup.bash ({exc})")
    rclpy.init(args=None)
    node = Node("sim_perception_bench")
    collected: List[Tuple[str, np.ndarray]] = []

    def callback(message: Any) -> None:
        if len(collected) >= frames:
            return
        image = image_to_bgr(message)
        if image is not None:
            collected.append((f"{topic}#{len(collected)}", image))

    node.create_subscription(Image, topic, callback, qos_profile_sensor_data)
    deadline = time.monotonic() + max(1.0, timeout)
    try:
        while len(collected) < frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return collected


def write_json(meta: Dict[str, Any], frames: Sequence[Dict[str, Any]], summary: Dict[str, Any], path: str) -> None:
    payload = {"meta": meta, "frames": list(frames), "summary": summary}
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def write_tsv(frames: Sequence[Dict[str, Any]], path: str) -> None:
    lines = ["\t".join(TSV_COLUMNS)]
    for frame in frames:
        lines.append("\t".join(str(frame[column]) for column in TSV_COLUMNS))
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


# --------------------------------------------------------------------------- CLI
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Benchmark de percepcion sobre camara simulada o imagenes.")
    parser.add_argument("--domain", type=int, default=42, help="ROS_DOMAIN_ID (nunca 10); default 42")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="topico sensor_msgs/Image de camara")
    parser.add_argument("--frames", type=int, default=30, help="imagenes a capturar en modo ROS")
    parser.add_argument("--timeout", type=float, default=60.0, help="segundos maximos de captura ROS")
    parser.add_argument("--images", help="carpeta de imagenes para el modo offline")
    parser.add_argument("--out", default="bench_out", help="prefijo de salida (.json y .tsv)")
    parser.add_argument("--out-json", help="ruta exacta del JSON")
    parser.add_argument("--out-tsv", help="ruta exacta del TSV")
    parser.add_argument("--models-dir", help="carpeta de modelos (yolov8n-pose.pt, YuNet, Caffe)")
    parser.add_argument("--pose-model", help="ruta exacta de yolov8n-pose.pt")
    parser.add_argument("--conf", type=float, default=DEFAULT_CONF, help="umbral de deteccion de personas")
    parser.add_argument("--kp-conf", type=float, default=DEFAULT_KP_CONF, help="umbral de keypoint visible")
    parser.add_argument("--save-images", help="carpeta donde guardar los cuadros analizados (PNG)")
    parser.add_argument("--no-face", action="store_true", help="no intentar deteccion de caras")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.domain == 10:
        raise SystemExit("ROS_DOMAIN_ID=10 es el robot real: usa un dominio aislado (42/77).")

    if args.images:
        mode = "offline"
        images = load_images(args.images)
        if not images:
            raise SystemExit(f"No hay imagenes en {args.images}")
    else:
        mode = "ros"
        images = capture_ros(args.topic, args.domain, args.frames, args.timeout)
        if not images:
            raise SystemExit(
                f"No llegaron imagenes de {args.topic} en el dominio {args.domain}. "
                "Revisa que la sim/robot publique y que ROS_DOMAIN_ID coincida."
            )

    pose_path = resolve_pose_model(args.pose_model, args.models_dir)
    try:
        from ultralytics import YOLO
    except ImportError as exc:
        raise SystemExit(
            "ultralytics no esta disponible en este Python. Usa "
            f"~/ros2_vision_venv/bin/python3 ({exc})"
        )

    model = YOLO(str(pose_path))
    face_detector = FaceDetector(args.models_dir, enabled=not args.no_face)

    if args.save_images:
        Path(args.save_images).mkdir(parents=True, exist_ok=True)

    frames = []
    for index, (source, image) in enumerate(images):
        record = analyze_frame(index, source, image, model, face_detector, args.conf, args.kp_conf)
        frames.append(record)
        if args.save_images:
            cv2.imwrite(str(Path(args.save_images) / f"frame_{index:04d}.png"), image)
        print(f"[{index + 1}/{len(images)}] {source}: personas={record['persons']} "
              f"manos={record['raised_hands']} caras={record['faces']}")

    summary = summarize(frames)
    out_json = args.out_json or f"{args.out}.json"
    out_tsv = args.out_tsv or f"{args.out}.tsv"
    meta = {
        "mode": mode,
        "topic": args.topic if mode == "ros" else None,
        "domain": args.domain if mode == "ros" else None,
        "images_dir": str(Path(args.images).expanduser()) if args.images else None,
        "pose_model": str(pose_path),
        "face_model": face_detector.name or None,
        "conf_thresh": args.conf,
        "kp_conf_thresh": args.kp_conf,
        "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }
    write_json(meta, frames, summary, out_json)
    write_tsv(frames, out_tsv)
    print(json.dumps(summary, indent=2, ensure_ascii=False))
    print(f"JSON: {out_json}\nTSV:  {out_tsv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
