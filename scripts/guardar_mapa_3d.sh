#!/usr/bin/env bash
# ==============================================================================
# Script de Guardado y Reconstruccion 3D de Mapa (RTAB-Map)
# Exporta base de datos SQLite, Nube de Puntos (PLY) y Malla Texturada (OBJ)
# ==============================================================================

set -e

SOURCE_ROS="/opt/ros/jazzy/setup.bash"
SOURCE_WS="/home/athome/base_athome_ws/install/setup.bash"

[ -f "$SOURCE_ROS" ] && source "$SOURCE_ROS"
[ -f "$SOURCE_WS" ] && source "$SOURCE_WS"

DB_SOURCE="${HOME}/.ros/rtabmap.db"

if [ ! -f "$DB_SOURCE" ]; then
    echo "ERROR: No se encontro el archivo de base de datos RTAB-Map en: $DB_SOURCE"
    echo "Asegurate de haber ejecutado 'mapeo3d' antes de guardar."
    exit 1
fi

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
CUSTOM_NAME="${1:-mapa_3d_${TIMESTAMP}}"
OUTPUT_DIR="${HOME}/mapas_3d/${CUSTOM_NAME}"
mkdir -p "$OUTPUT_DIR"

echo "=================================================="
echo " Guardando y reconstruyendo Mapa 3D..."
echo " Directorio destino: $OUTPUT_DIR"
echo " Base de datos origen: $DB_SOURCE ($(du -h "$DB_SOURCE" | cut -f1))"
echo "=================================================="

# 1. Llamar al servicio de backup si RTAB-Map esta activo
if ros2 service list 2>/dev/null | grep -q "/rtabmap/rtabmap/backup"; then
    echo "Sincronizando estado en vivo de RTAB-Map con backup en disco..."
    ros2 service call /rtabmap/rtabmap/backup std_srvs/srv/Empty "{}" >/dev/null 2>&1 || true
fi

# 2. Copiar la base de datos completa de keyframes y poses
TARGET_DB="${OUTPUT_DIR}/rtabmap.db"
echo "Copiando base de datos a $TARGET_DB..."
cp "$DB_SOURCE" "$TARGET_DB"

# 3. Exportar nube de puntos 3D ensamblada (.ply) y Malla Texturada (.obj)
echo "Ejecutando reconstruccion 3D (rtabmap-export)..."
rtabmap-export \
    --cloud \
    --mesh \
    --texture \
    --poisson_size 0.04 \
    --max_polygons 300000 \
    --output_dir "$OUTPUT_DIR" \
    "$TARGET_DB"

echo "--------------------------------------------------"
echo "Reconstruccion 3D guardada con exito en:"
echo "  $OUTPUT_DIR"
echo ""
echo "Archivos generados:"
ls -lh "$OUTPUT_DIR"
echo "--------------------------------------------------"
