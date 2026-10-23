#!/usr/bin/env bash
# ==============================================================================
# Base@Home - Script de Descarga de Modelos de IA / Vision para RoboCup @Home
# ==============================================================================

set -e

MODELS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../models" && pwd)"
mkdir -p "$MODELS_DIR"

echo "=================================================="
echo " Verificando y descargando modelos de IA en:"
echo "   $MODELS_DIR"
echo "=================================================="

# 1. Detector Facial Deep Learning (OpenCV Caffe SSD ResNet-10)
if [ ! -f "${MODELS_DIR}/deploy.prototxt" ]; then
    echo "[+] Descargando deploy.prototxt..."
    curl -sL -o "${MODELS_DIR}/deploy.prototxt" \
        https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt
fi

if [ ! -f "${MODELS_DIR}/res10_300x300_ssd_iter_140000.caffemodel" ]; then
    echo "[+] Descargando res10_300x300_ssd_iter_140000.caffemodel (~10 MB)..."
    curl -L -o "${MODELS_DIR}/res10_300x300_ssd_iter_140000.caffemodel" \
        https://raw.githubusercontent.com/opencv/opencv_3rdparty/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel
fi

# 2. Extractor Biometrico SFace (128-D Embeddings)
if [ ! -f "${MODELS_DIR}/face_recognition_sface_2021dec.onnx" ]; then
    echo "[+] Descargando face_recognition_sface_2021dec.onnx (~37 MB)..."
    curl -L -o "${MODELS_DIR}/face_recognition_sface_2021dec.onnx" \
        https://github.com/opencv/opencv_zoo/raw/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx
fi

# 3. Detector de Poses Humanas YOLOv8-pose
if [ ! -f "${MODELS_DIR}/yolov8n-pose.pt" ]; then
    echo "[+] Descargando yolov8n-pose.pt (~6.5 MB)..."
    curl -L -o "${MODELS_DIR}/yolov8n-pose.pt" \
        https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n-pose.pt
fi

# 4. Detector de Objetos y Basura YOLOv8
if [ ! -f "${MODELS_DIR}/yolov8n.pt" ]; then
    echo "[+] Descargando yolov8n.pt (~6.2 MB)..."
    curl -L -o "${MODELS_DIR}/yolov8n.pt" \
        https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt
fi

echo "--------------------------------------------------"
echo "Todos los modelos de percepcion estan listos en:"
ls -lh "$MODELS_DIR"
echo "--------------------------------------------------"
