#!/bin/bash

echo "======================================"
echo "   COMPILANDO Y FLASHEANDO FIRMWARE   "
echo "======================================"

# Ruta del repositorio del firmware hack
FIRMWARE_DIR="../../hoverboard-firmware-hack-FOC"

# Verificar que exista la carpeta
if [ ! -d "$FIRMWARE_DIR" ]; then
    echo "[!] No se encontró la carpeta del firmware en $FIRMWARE_DIR"
    exit 1
fi

echo "[+] Copiando el nuevo config.h con odometría habilitada..."
cp ../firmware_config/config.h "$FIRMWARE_DIR/Inc/config.h"

echo "[+] Limpiando compilación anterior..."
cd "$FIRMWARE_DIR"
make clean

echo "[+] Compilando..."
make -j4

if [ $? -eq 0 ]; then
    echo "[+] Compilación exitosa."
    echo "[+] Iniciando flasheo vía ST-Link..."
    
    # Dependiendo de tu herramienta, usa unlock y flash
    # make unlock (opcional si la placa está bloqueada)
    make flash
    
    if [ $? -eq 0 ]; then
        echo "======================================"
        echo " ✅ FLASHEO COMPLETADO CON ÉXITO ✅ "
        echo "======================================"
    else
        echo "======================================"
        echo " ❌ ERROR en el flasheo. Verifica el ST-Link. "
        echo "======================================"
    fi
else
    echo "======================================"
    echo " ❌ ERROR de compilación. "
    echo "======================================"
fi
