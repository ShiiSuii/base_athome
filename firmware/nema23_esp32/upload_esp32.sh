#!/usr/bin/env bash
# ==============================================================================
# Script para compilar y flashear el firmware NEMA 23 en el ESP32-S3
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH="${SCRIPT_DIR}/nema23_esp32.ino"
FQBN="esp32:esp32:esp32s3"
ARDUINO_CLI="/home/athome/bin/arduino-cli"

if [ ! -f "$ARDUINO_CLI" ]; then
    ARDUINO_CLI="$(which arduino-cli 2>/dev/null)"
fi

if [ -z "$ARDUINO_CLI" ] || [ ! -x "$ARDUINO_CLI" ]; then
    echo "[X] Error: arduino-cli no encontrado en /home/athome/bin/arduino-cli"
    exit 1
fi

PORT="$1"

if [ -z "$PORT" ]; then
    # Chequeo por udev persistente de la ESP32-S3 (QinHeng CH343)
    ESP32_BY_ID=$(ls /dev/serial/by-id/usb-1a86_USB_Single_Serial* 2>/dev/null | head -n 1)
    if [ -n "$ESP32_BY_ID" ]; then
        PORT="$ESP32_BY_ID"
    elif [ -e "/dev/ttyACM0" ]; then
        PORT="/dev/ttyACM0"
    else
        PORT=$(ls /dev/ttyACM* 2>/dev/null | head -n 1)
    fi
fi

echo "=================================================="
echo "    COMPILADOR Y FLASHER PARA ESP32-S3 (NEMA 23)  "
echo "=================================================="
echo " Sketch: $SKETCH"
echo " FQBN:   $FQBN"
echo " Puerto: ${PORT:-'No detectado (solo compilacion)'}"
echo "=================================================="

echo "[1/2] Compilando firmware para ESP32-S3..."
"$ARDUINO_CLI" compile --fqbn "$FQBN" "$SKETCH"

if [ $? -ne 0 ]; then
    echo "[X] Error en la compilacion."
    exit 1
fi
echo "[✓] Compilacion exitosa!"

if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
    echo "[!] No se especifico puerto o no se detecto /dev/ttyACM* conectado."
    echo "    Para flashear manualmente ejecuta: ./upload_esp32.sh /dev/ttyACM0"
    exit 0
fi

echo ""
echo "[2/2] Flasheando a ESP32-S3 en $PORT..."
"$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH"

if [ $? -eq 0 ]; then
    echo ""
    echo "[✓] ¡Firmware flasheado correctamente en ESP32-S3!"
    echo "    Puedes controlar el motor ejecutando: stepper o nema23"
    exit 0
else
    echo ""
    echo "[!] Si el flasheo fallo por timeout de conexion, mantén presionado el boton 'BOOT'"
    echo "    en la placa ESP32 mientras ejecutas este script para forzar el modo descarga (Download Mode)."
    exit 1
fi
