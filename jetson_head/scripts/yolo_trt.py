#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Inferencia YOLOv8 (deteccion) con TensorRT en la Jetson, sin PyTorch.

Pre/post-proceso en numpy (letterbox, decodificacion, NMS) separado del motor
para poder probarlo en la NUC sin GPU. El motor usa `tensorrt` + `cuda-python`.
Engine: jetson_head/scripts/build_yolo_trt.sh (FP16, entrada 1x3x640x640).
"""
from typing import List, Tuple

import numpy as np

COCO_CLASSES = [
    'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
    'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
    'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
    'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
    'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
    'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
    'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
    'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
    'hair drier', 'toothbrush',
]


def letterbox(img: np.ndarray, size: int = 640) -> Tuple[np.ndarray, float, Tuple[float, float]]:
    """BGR HxWx3 uint8 -> tensor 1x3xSxS float32 RGB [0,1], escala y padding (px, py)."""
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    if (nh, nw) != (h, w):
        import cv2
        img = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    px, py = (size - nw) / 2.0, (size - nh) / 2.0
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    top, left = int(round(py - 0.1)), int(round(px - 0.1))
    canvas[top:top + nh, left:left + nw] = img
    tensor = canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return np.ascontiguousarray(tensor), r, (left, top)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thr: float) -> List[int]:
    """NMS clasico sobre cajas xyxy. Devuelve indices ordenados por score."""
    order = scores.argsort()[::-1]
    areas = (boxes[:, 2] - boxes[:, 0]).clip(0) * (boxes[:, 3] - boxes[:, 1]).clip(0)
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = (xx2 - xx1).clip(0) * (yy2 - yy1).clip(0)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= iou_thr]
    return keep


def postprocess(output: np.ndarray, scale: float, pad: Tuple[float, float], img_shape: Tuple[int, int],
                conf_thr: float = 0.4, iou_thr: float = 0.5, max_det: int = 100):
    """Salida YOLOv8 (1, 4+C, N) -> lista de (x1, y1, x2, y2, score, class_id) en pixeles de la imagen original."""
    pred = output.reshape(output.shape[-2], output.shape[-1]).T  # (N, 4+C)
    cls_scores = pred[:, 4:]
    cls_ids = cls_scores.argmax(axis=1)
    scores = cls_scores[np.arange(len(pred)), cls_ids]
    mask = scores >= conf_thr
    if not mask.any():
        return []
    pred, cls_ids, scores = pred[mask], cls_ids[mask], scores[mask]
    cx, cy, w, h = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
    boxes = np.stack([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2], axis=1)
    # NMS por clase desplazando cajas de clases distintas
    offset = cls_ids[:, None].astype(np.float32) * 4096.0
    keep = nms(boxes + offset, scores, iou_thr)[:max_det]
    boxes = boxes[keep]
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad[0]) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad[1]) / scale
    ih, iw = img_shape
    boxes[:, [0, 2]] = boxes[:, [0, 2]].clip(0, iw)
    boxes[:, [1, 3]] = boxes[:, [1, 3]].clip(0, ih)
    return [(*map(float, b), float(scores[k]), int(cls_ids[k])) for b, k in zip(boxes, keep)]


class TRTEngine:
    """Motor TensorRT 10 con un tensor de entrada y uno de salida (buffers fijos)."""

    def __init__(self, engine_path: str):
        import tensorrt as trt
        try:  # cuda-python >= 12.8 / 13 (JetPack 7)
            from cuda.bindings import runtime as cudart
        except ImportError:  # cuda-python 12.6 (JetPack 6)
            from cuda import cudart
        self.cudart = cudart
        self.logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            self.engine = trt.Runtime(self.logger).deserialize_cuda_engine(f.read())
        self.context = self.engine.create_execution_context()
        names = [self.engine.get_tensor_name(i) for i in range(self.engine.num_io_tensors)]
        self.in_name = next(n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT)
        self.out_name = next(n for n in names if self.engine.get_tensor_mode(n) == trt.TensorIOMode.OUTPUT)
        self.in_shape = tuple(self.engine.get_tensor_shape(self.in_name))
        self.out_shape = tuple(self.engine.get_tensor_shape(self.out_name))
        self.out_host = np.empty(self.out_shape, dtype=np.float32)
        self.in_bytes = int(np.prod(self.in_shape)) * 4
        self.d_in = self._check(cudart.cudaMalloc(self.in_bytes))
        self.d_out = self._check(cudart.cudaMalloc(self.out_host.nbytes))
        self.stream = self._check(cudart.cudaStreamCreate())
        self.context.set_tensor_address(self.in_name, int(self.d_in))
        self.context.set_tensor_address(self.out_name, int(self.d_out))

    def _check(self, res):
        err, *val = res
        if int(err) != 0:
            raise RuntimeError(f'CUDA error {err}')
        return val[0] if val else None

    def infer(self, tensor: np.ndarray) -> np.ndarray:
        c = self.cudart
        tensor = np.ascontiguousarray(tensor, dtype=np.float32)
        self._check(c.cudaMemcpyAsync(self.d_in, tensor.ctypes.data, self.in_bytes,
                                      c.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream))
        self.context.execute_async_v3(int(self.stream))
        self._check(c.cudaMemcpyAsync(self.out_host.ctypes.data, self.d_out, self.out_host.nbytes,
                                      c.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream))
        self._check(c.cudaStreamSynchronize(self.stream))
        return self.out_host

    def close(self):
        for ptr in (self.d_in, self.d_out):
            self.cudart.cudaFree(ptr)
        self.cudart.cudaStreamDestroy(self.stream)
