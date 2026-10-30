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

# Calibracion optima de ganancia para microfono RØDE VideoMic GO II (+18dB)
amixer -c 0 sset 'Mic' 18 2>/dev/null || true

echo '[Jetson Bringup] Iniciando: RealSense D435 + Pan-Tilt + Whisper Audio + Piper CAN + DaBai Camera'

# Iniciar controlador CAN del brazo Piper y camara de muneca Orbbec DaBai
pgrep -f piper_arm_controller.py >/dev/null || nohup python3 /home/ubuntu/piper_arm_controller.py > /tmp/piper_arm_controller.log 2>&1 < /dev/null &
pgrep -f stream_orbbec.py >/dev/null || nohup python3 /home/ubuntu/stream_orbbec.py > /tmp/stream_orbbec.log 2>&1 < /dev/null &
pkill -f stream_zed.py 2>/dev/null || true

exec ros2 launch robot_bringup_jetson jetson_head.launch.py enable_camera:=false enable_audio:=true language:=auto
