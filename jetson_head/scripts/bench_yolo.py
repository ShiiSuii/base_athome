#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Benchmark de YOLOv8 en la Jetson: onnxruntime (CPU) vs TensorRT (GPU).

Mide la latencia de punta a punta por frame con el MISMO pre/post-proceso que
usa yolo_trt_node.py (letterbox + inferencia + decodificación + NMS), separando
el tiempo de inferencia pura. No usa ROS ni abre cámaras ni actuadores.

    python3 bench_yolo.py --backend ort --model ~/models/yolo/yolov8n.onnx
    python3 bench_yolo.py --backend trt --model ~/models/yolo/yolov8n_fp16.engine
    python3 bench_yolo.py ... --image foto.jpg --compare-with ~/models/yolo/yolov8n.onnx

Salida: una línea JSON (para pegar en STATUS.md / ROBOT.md).
"""
import argparse
import json
import os
import statistics
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from yolo_trt import COCO_CLASSES, TRTEngine, letterbox, postprocess  # noqa: E402


class ORTEngine:
    """Misma interfaz que TRTEngine sobre onnxruntime (solo CPU)."""

    def __init__(self, model_path: str, threads: int = 0):
        import onnxruntime as ort
        opts = ort.SessionOptions()
        if threads:
            opts.intra_op_num_threads = threads
        self.sess = ort.InferenceSession(model_path, opts, providers=['CPUExecutionProvider'])
        self.input_name = self.sess.get_inputs()[0].name

    def infer(self, tensor: np.ndarray) -> np.ndarray:
        return self.sess.run(None, {self.input_name: tensor})[0]

    def close(self):
        pass


def make_engine(backend: str, path: str, threads: int):
    if backend == 'trt':
        return TRTEngine(path)
    return ORTEngine(path, threads)


def load_image(path: str) -> np.ndarray:
    if path:
        import cv2
        img = cv2.imread(os.path.expanduser(path))
        if img is None:
            raise SystemExit(f'no se pudo leer {path}')
        return img
    rng = np.random.default_rng(0)
    return rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)  # tamaño de la cámara de muñeca


def run(engine, img, n: int, warmup: int, conf: float, iou: float):
    infer_ms, total_ms, dets = [], [], None
    for i in range(warmup + n):
        t0 = time.perf_counter()
        tensor, scale, pad = letterbox(img)
        t1 = time.perf_counter()
        out = engine.infer(tensor)
        t2 = time.perf_counter()
        dets = postprocess(out, scale, pad, img.shape[:2], conf, iou)
        t3 = time.perf_counter()
        if i >= warmup:
            infer_ms.append((t2 - t1) * 1e3)
            total_ms.append((t3 - t0) * 1e3)
    return infer_ms, total_ms, dets


def fmt(dets):
    return [(COCO_CLASSES[d[5]], round(d[4], 3), [round(v) for v in d[:4]]) for d in dets[:10]]


def summary(xs):
    xs = sorted(xs)
    return {'mean': round(statistics.fmean(xs), 2), 'p50': round(xs[len(xs) // 2], 2),
            'p95': round(xs[int(len(xs) * 0.95) - 1], 2)}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--backend', choices=['ort', 'trt'], required=True)
    ap.add_argument('--model', required=True)
    ap.add_argument('--image', default='')
    ap.add_argument('--n', type=int, default=100)
    ap.add_argument('--warmup', type=int, default=10)
    ap.add_argument('--threads', type=int, default=0, help='hilos de onnxruntime (0 = auto)')
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--iou', type=float, default=0.45)
    ap.add_argument('--compare-with', default='', help='ONNX para comparar detecciones (CPU)')
    a = ap.parse_args()

    img = load_image(a.image)
    eng = make_engine(a.backend, os.path.expanduser(a.model), a.threads)
    try:
        infer_ms, total_ms, dets = run(eng, img, a.n, a.warmup, a.conf, a.iou)
    finally:
        eng.close()
    res = {'backend': a.backend, 'model': os.path.basename(a.model), 'n': a.n,
           'infer_ms': summary(infer_ms), 'total_ms': summary(total_ms),
           'fps_total': round(1000.0 / statistics.fmean(total_ms), 1),
           'detections': fmt(dets or [])}
    if a.compare_with:
        ref = ORTEngine(os.path.expanduser(a.compare_with))
        _, _, ref_dets = run(ref, img, 1, 0, a.conf, a.iou)
        res['reference_detections'] = fmt(ref_dets or [])
    print(json.dumps(res))


if __name__ == '__main__':
    main()
