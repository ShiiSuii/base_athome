#!/usr/bin/env bash
# ==============================================================================
# Script de Guardado de Mapa 2D (Base@Home / RoboCup @Home)
# Guarda mapa 2D (.yaml y .pgm) desde el topico /map de SLAM Toolbox
# ==============================================================================

set -e

SOURCE_ROS="/opt/ros/jazzy/setup.bash"
SOURCE_WS="/home/athome/base_athome_ws/install/setup.bash"

[ -f "$SOURCE_ROS" ] && source "$SOURCE_ROS"
[ -f "$SOURCE_WS" ] && source "$SOURCE_WS"

TARGET_NAME="${1:-mapa_base_athome}"
BASE_DIR="/home/athome"
TARGET_PATH="${BASE_DIR}/${TARGET_NAME}"

BACKUP_DIR="${BASE_DIR}/mapas"
mkdir -p "$BACKUP_DIR"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BACKUP_PATH="${BACKUP_DIR}/${TARGET_NAME}_${TIMESTAMP}"

echo "=================================================="
echo " Guardando Mapa 2D desde /map..."
echo " Destino principal: ${TARGET_PATH}.yaml (.pgm)"
echo " Copia historica:   ${BACKUP_PATH}.yaml (.pgm)"
echo "=================================================="

# Guardar mapa principal
ros2 run nav2_map_server map_saver_cli -f "${TARGET_PATH}" --ros-args -p save_map_timeout:=10000.0 -p overwrite:=true

# Generar copia de seguridad con fecha
cp "${TARGET_PATH}.yaml" "${BACKUP_PATH}.yaml"
cp "${TARGET_PATH}.pgm" "${BACKUP_PATH}.pgm"

echo "--------------------------------------------------"
echo "Mapa 2D guardado con exito."
echo "Archivos generados:"
ls -lh "${TARGET_PATH}.yaml" "${TARGET_PATH}.pgm"
echo "--------------------------------------------------"
