#!/bin/bash
# Construye engines TensorRT FP16 de YOLO en la Jetson a partir de ONNX.
# Uso (en la Jetson):  ~/build_yolo_trt.sh ~/models/yolo/yolov8n.onnx [...]
# El ONNX se exporta en la NUC con ultralytics en un venv descartable (no en
# ~/ros2_vision_venv: el export auto-instala paquetes y rompe cv2):
#   YOLO('yolov8n.pt').export(format='onnx', imgsz=640, opset=17)
# Un engine solo sirve para la misma version de TensorRT: rehacer tras migrar JetPack.
set -euo pipefail
TRTEXEC=/usr/src/tensorrt/bin/trtexec
[ $# -ge 1 ] || { echo "uso: $0 modelo.onnx [...]"; exit 2; }
for onnx in "$@"; do
    out="${onnx%.onnx}_fp16.engine"
    echo "== $onnx -> $out"
    "$TRTEXEC" --onnx="$onnx" --saveEngine="$out" --fp16 --memPoolSize=workspace:1024 > "${onnx%.onnx}_build.log" 2>&1
    grep -E "Throughput|GPU Compute Time: min" "${onnx%.onnx}_build.log"
done
