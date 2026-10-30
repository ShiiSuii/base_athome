#!/bin/bash
# Despliega jetson_head/ (fuente unica) a la Jetson y compila el workspace.
# Por defecto es DRY-RUN: muestra diferencias sin escribir nada.
#
#   jetson_head/scripts/deploy_to_jetson.sh                 # dry-run
#   jetson_head/scripts/deploy_to_jetson.sh --apply         # copia + colcon build
#   JETSON=ubuntu@192.168.0.203 jetson_head/scripts/deploy_to_jetson.sh --apply
#
# No reinicia jetson-head.service ni toca systemd/red: hacerlo a mano tras revisar.
# Antes de copiar guarda un respaldo de lo reemplazado en ~/deploy_backups/<fecha>.
set -euo pipefail

JETSON="${JETSON:-ubuntu@10.42.0.12}"
HERE="$(cd "$(dirname "$0")/.." && pwd)"
# DEPLOY_SCRIPTS permite desplegar un subconjunto (separado por espacios).
read -r -a SCRIPTS <<< "${DEPLOY_SCRIPTS:-piper_arm_controller.py start_head.sh start_mapping_head.sh stream_orbbec.py}"
APPLY=false
[ "${1:-}" = "--apply" ] && APPLY=true

# La calibracion vive en la Jetson (la escribe calibrate_pan_tilt.py): no pisarla.
RSYNC_OPTS=(-rlptci --exclude=__pycache__ --exclude=build --exclude=install --exclude=log
            --exclude=pan_tilt_controller/config/pan_tilt_params.yaml)
$APPLY || RSYNC_OPTS+=(--dry-run)

echo "Destino: $JETSON  Modo: $($APPLY && echo APPLY || echo DRY-RUN)"
ssh -o BatchMode=yes -o ConnectTimeout=8 "$JETSON" true

if $APPLY; then
    STAMP="$(date +%Y%m%d_%H%M%S)"
    ssh "$JETSON" "mkdir -p ~/deploy_backups/$STAMP && tar czf ~/deploy_backups/$STAMP/before.tgz \
        -C ~ athome_ws/src ${SCRIPTS[*]} 2>/dev/null; echo respaldo: ~/deploy_backups/$STAMP/before.tgz"
fi

echo "--- paquetes ROS 2 (athome_ws/src)"
rsync "${RSYNC_OPTS[@]}" "$HERE/src/" "$JETSON:athome_ws/src/"
echo "--- scripts de ~"
for s in "${SCRIPTS[@]}"; do
    rsync "${RSYNC_OPTS[@]}" "$HERE/scripts/$s" "$JETSON:$s"
done

if $APPLY; then
    ssh "$JETSON" 'source /opt/ros/humble/setup.bash && cd ~/athome_ws && colcon build --symlink-install 2>&1 | tail -5'
    echo "Listo. Revisar y reiniciar a mano si corresponde: sudo systemctl restart jetson-head"
fi
