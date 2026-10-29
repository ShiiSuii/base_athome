#!/usr/bin/env bash
# ==============================================================================
# Script de compilacion y verificacion del modelo URDF modular de Base@Home
# ==============================================================================
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "==> Compilando URDF modular desde: $DIR/robot.urdf.xacro"

xacro "$DIR/robot.urdf.xacro" -o "$DIR/robot.urdf"
echo "✔ Generado: $DIR/robot.urdf"

echo "==> Verificando estructura cinematica con check_urdf..."
check_urdf "$DIR/robot.urdf"

echo "🎉 Modelo URDF compilado y verificado con exito."
