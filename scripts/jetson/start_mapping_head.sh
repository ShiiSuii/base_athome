#!/bin/bash
# Distro autodetectada: Jazzy tras HOME-11 (JetPack 7.2.1), Humble en JetPack 6.
ROS_DIR=$(ls -d /opt/ros/jazzy /opt/ros/humble 2>/dev/null | head -1)
export PATH=$ROS_DIR/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source $ROS_DIR/setup.bash
if [ -f /home/ubuntu/athome_ws/install/setup.bash ]; then
    source /home/ubuntu/athome_ws/install/setup.bash
fi

export LD_LIBRARY_PATH=$ROS_DIR/lib/aarch64-linux-gnu:$ROS_DIR/lib:/usr/local/cuda/lib64
# Ubuntu 24.04 (PEP 668): dependencias pip en ~/robot_venv (bootstrap_jazzy.sh)
VENV_SITE=$(ls -d /home/ubuntu/robot_venv/lib/python3*/site-packages 2>/dev/null | head -1)
[ -n "$VENV_SITE" ] && export PYTHONPATH=$VENV_SITE:$PYTHONPATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
export ROS_DOMAIN_ID=10

# Esperar a que alguna interfaz Ethernet tenga IP 10.42.x (max 30 s). Desde
# 2026-10-30 el enlace usa el adaptador USB RTL8153 (enx00e04c68090c) porque el
# puerto integrado enP8p1s0 no levanta carrier; cyclonedds.xml liga por address.
echo "[Jetson Bringup] Esperando que la interfaz de red Ethernet tenga IP asignada..."
LINK_OK=0
for i in {1..30}; do
    if ip addr 2>/dev/null | grep -q "inet 10.42."; then
        echo "[Jetson Bringup] Interfaz de red Ethernet lista: $(ip -br a | grep 10.42.)"
        LINK_OK=1
        break
    fi
    sleep 1
done

# Sin IP 10.42.x el cyclonedds.xml (ligado a 10.42.0.12) impide crear cualquier
# nodo. Caer a DDS solo-localhost: los nodos arrancan pero no son alcanzables
# (ni comandables) desde la WiFi u otros hosts.
if [ "$LINK_OK" != "1" ]; then
    echo "[Jetson Bringup] AVISO: sin IP 10.42.x. DDS en modo solo-localhost."
    export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface address="127.0.0.1"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><ParticipantIndex>auto</ParticipantIndex><Peers><Peer address="127.0.0.1"/></Peers></Discovery></Domain></CycloneDDS>'
fi

# Microfono RØDE VideoMic GO II siempre a maxima ganancia. El indice de tarjeta cambia
# entre arranques (con JetPack 7 es la 2), asi que se busca por nombre. whisper_node
# vuelve a fijarla cada vez que (re)abre el stream.
RODE_CARD=$(awk '/RØDE|RODE|VideoMic/ {print $1; exit}' /proc/asound/cards)
[ -n "$RODE_CARD" ] && amixer -c "$RODE_CARD" sset 'Mic' 100% cap >/dev/null 2>&1 || true

# Asignacion de computo (docs/ROBOT.md §1): la Jetson corre lo que esta
# conectado a ella (Piper CAN, pan-tilt U2D2, DaBai, RØDE) y la inferencia con GPU.
# La RealSense D435 esta en la NUC (enable_camera:=false aca).
# Overrides por entorno (p.ej. en un drop-in de systemd):
WHISPER_MODEL="${WHISPER_MODEL:-base.en}"   # base.en: 6% WER vs tiny 20% (medido)
ASR_LANGUAGE="${ASR_LANGUAGE:-en}"          # base.en es solo ingles; para demos en espanol: tiny + auto
ENABLE_YOLO="${ENABLE_YOLO:-1}"
YOLO_ENGINE="${YOLO_ENGINE:-/home/ubuntu/models/yolo/yolov8n_fp16.engine}"

echo "[Jetson Bringup] Iniciando: Pan-Tilt + Whisper ($WHISPER_MODEL/$ASR_LANGUAGE) + Piper CAN + DaBai + YOLO TensorRT"

# Iniciar controlador CAN del brazo Piper y camara de muneca Orbbec DaBai
# Lazo de relanzado: el controlador sale si pierde la realimentacion CAN (adaptador re-enumerado)
if ! pgrep -f "python3 /home/ubuntu/piper_arm_controller.py" >/dev/null; then
    : > /tmp/piper_arm_controller.log
    nohup bash -c 'while true; do python3 /home/ubuntu/piper_arm_controller.py >> /tmp/piper_arm_controller.log 2>&1; sleep 2; done' < /dev/null >/dev/null 2>&1 &
fi
pgrep -f stream_orbbec.py >/dev/null || nohup python3 /home/ubuntu/stream_orbbec.py > /tmp/stream_orbbec.log 2>&1 < /dev/null &
if [ "$ENABLE_YOLO" = "1" ] && [ -f "$YOLO_ENGINE" ]; then
    pgrep -f yolo_trt_node.py >/dev/null || nohup python3 /home/ubuntu/yolo_trt_node.py --ros-args -p engine_path:="$YOLO_ENGINE" > /tmp/yolo_trt_node.log 2>&1 < /dev/null &
else
    echo "[Jetson Bringup] YOLO deshabilitado o falta $YOLO_ENGINE (build_yolo_trt.sh)"
fi
pkill -f stream_zed.py 2>/dev/null || true

exec ros2 launch robot_bringup_jetson jetson_head.launch.py enable_camera:=false enable_audio:=true \
    language:="$ASR_LANGUAGE" whisper_model:="$WHISPER_MODEL"
