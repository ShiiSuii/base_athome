#!/bin/bash
# HOME-11 Fase D: deja una Jetson recien instalada con JetPack 7.2.1 (Ubuntu 24.04)
# lista para el software de la cabeza con ROS 2 Jazzy. Ejecutar EN LA JETSON como
# el usuario del robot (con sudo), despues del primer arranque.
#
#   ./bootstrap_jazzy.sh            # DRY-RUN: solo imprime lo que haria
#   ./bootstrap_jazzy.sh --apply    # ejecuta
#
# Requisitos previos (desde la NUC): copiar el proyecto con
#   JETSON=ubuntu@10.42.0.12 jetson_head/scripts/deploy_to_jetson.sh --apply
# o, si la Jetson aun no tiene IP fija, restaurar
#   ~/jetson_backups/2026-10-30_pre_migration_extra/jetson_project_current.tgz
#
# Seguridad: NO habilita jetson-head.service ni levanta CAN con actuadores; eso
# queda para el humano (ver docs/collab/RUNBOOK_MIGRACION_JETSON.md). No ROS 1.
set -euo pipefail

APPLY=0; [ "${1:-}" = "--apply" ] && APPLY=1
ROS_DISTRO_TARGET=jazzy
VENV="$HOME/robot_venv"
WS="$HOME/athome_ws"
ETH_IF="${ETH_IF:-}"          # vacio = autodetectar el adaptador USB RTL8153 (enx...)
JETSON_IP=10.42.0.12/24
NUC_IP=10.42.0.1

run() { echo "+ $*"; [ $APPLY -eq 1 ] && eval "$@"; return 0; }
step() { echo; echo "=== $* ==="; }

step "0. Verificaciones"
. /etc/os-release
echo "SO: $PRETTY_NAME"; uname -r
[ "$VERSION_ID" = "24.04" ] || { echo "Se esperaba Ubuntu 24.04"; exit 1; }
cat /etc/nv_tegra_release 2>/dev/null | head -1 || true
nvcc --version 2>/dev/null | tail -1 || echo "nvcc no en PATH (CUDA en /usr/local/cuda)"

step "1. ROS 2 Jazzy (apt oficial packages.ros.org, gratuito)"
run "sudo apt-get update"
run "sudo apt-get install -y software-properties-common curl gnupg lsb-release"
run "sudo add-apt-repository -y universe"
run "sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg"
run "echo \"deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main\" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null"
run "sudo apt-get update"
run "sudo apt-get install -y ros-$ROS_DISTRO_TARGET-ros-base ros-$ROS_DISTRO_TARGET-rmw-cyclonedds-cpp \
  ros-$ROS_DISTRO_TARGET-vision-msgs ros-$ROS_DISTRO_TARGET-dynamixel-sdk ros-$ROS_DISTRO_TARGET-realsense2-camera \
  python3-colcon-common-extensions python3-rosdep python3-venv python3-pip can-utils v4l-utils alsa-utils \
  portaudio19-dev libportaudio2 iperf3"

step "2. Entorno Python del robot (PEP 668: venv con acceso a rclpy del sistema)"
run "python3 -m venv --system-site-packages $VENV"
# Versiones del respaldo 2026-10-29 (pip3_freeze.txt) salvo donde Jazzy/py3.12 obliga a cambiar
run "$VENV/bin/pip install --upgrade pip"
run "$VENV/bin/pip install piper_sdk==0.6.2 python-can==4.6.1 dynamixel-sdk==4.1.0 sounddevice==0.5.6 faster-whisper==1.2.1 PyYAML"
echo "AVISO: ctranslate2 del pip no trae CUDA en aarch64 -> Whisper en CPU (igual que hoy). GPU: compilar ctranslate2 para CUDA 13 (tarea aparte)."
echo "AVISO: cuda-python debe coincidir con CUDA 13 del JetPack 7.2.1: instalar 'cuda-python~=13.0' en $VENV tras verificar 'nvcc --version'."

step "3. CAN (gs_usb) y udev"
if modinfo gs_usb >/dev/null 2>&1; then echo "gs_usb disponible en el kernel $(uname -r)"; else
  echo "FALTA gs_usb: compilar desde ~/gs_usb (respaldo) contra los headers de $(uname -r) o habilitar CONFIG_CAN_GS_USB"; fi
run "echo gs_usb | sudo tee /etc/modules-load.d/gs_usb.conf >/dev/null"
# Igual que la Jetson actual. Levanta can* a 1 Mbit/s al enchufar el adaptador: el
# bus queda listo pero el Piper no se mueve sin /arm/set_enable con realimentacion.
run "echo 'ACTION==\"add\", SUBSYSTEM==\"net\", KERNEL==\"can*\", RUN+=\"/sbin/ip link set %k type can bitrate 1000000\", RUN+=\"/sbin/ip link set %k txqueuelen 1000\", RUN+=\"/sbin/ip link set %k up\"' | sudo tee /etc/udev/rules.d/98-can-piper.rules >/dev/null"
run "echo 'SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"0403\", ATTRS{idProduct}==\"6014\", SYMLINK+=\"ttyU2D2\", MODE=\"0666\", ATTR{device/latency_timer}=\"1\"' | sudo tee /etc/udev/rules.d/99-dynamixel-u2d2.rules >/dev/null"
run "sudo usermod -aG dialout,audio,video,plugdev $USER"
run "sudo udevadm control --reload-rules"

step "4. Red NUC<->Jetson (IP fija en el adaptador USB) y DDS"
[ -n "$ETH_IF" ] || ETH_IF=$(ls /sys/class/net | grep -m1 '^enx' || true)
echo "Interfaz Ethernet: ${ETH_IF:-NO ENCONTRADA}"
if [ -n "$ETH_IF" ]; then
  run "sudo nmcli con add type ethernet ifname $ETH_IF con-name robot-nuc ipv4.method manual ipv4.addresses $JETSON_IP ipv4.gateway $NUC_IP ipv4.never-default yes ipv6.method ignore || true"
  run "sudo nmcli con up robot-nuc || true"
fi
[ -f "$HOME/cyclonedds.xml" ] && echo "cyclonedds.xml presente" || echo "Copiar jetson_head/config/cyclonedds_jetson.xml a ~/cyclonedds.xml"
run "grep -q ROS_DOMAIN_ID ~/.bashrc || cat >> ~/.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
[ -f ~/athome_ws/install/setup.bash ] && source ~/athome_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://\$HOME/cyclonedds.xml
export ROS_DOMAIN_ID=10
export PYTHONPATH=\$HOME/robot_venv/lib/python3.12/site-packages:\$PYTHONPATH
EOF"

step "5. Workspace de la cabeza (colcon, Jazzy)"
if [ -d "$WS/src" ]; then
  run "cd $WS && source /opt/ros/$ROS_DISTRO_TARGET/setup.bash && colcon build --symlink-install"
else
  echo "Falta $WS/src: desplegar desde la NUC primero (deploy_to_jetson.sh)"
fi

step "6. Servicios (instalados, NO habilitados)"
if [ -f "$HOME/jetson-head.service" ] || [ -f "$(dirname "$0")/jetson-head.service" ]; then
  SRC=$( [ -f "$HOME/jetson-head.service" ] && echo "$HOME/jetson-head.service" || echo "$(dirname "$0")/jetson-head.service")
  run "sudo cp $SRC /etc/systemd/system/jetson-head.service && sudo systemctl daemon-reload"
fi
echo "Para arrancar la cabeza (decision humana, tras checklist de energizacion):"
echo "  sudo systemctl enable --now jetson-head"

step "7. Modelos"
echo "Restaurar caches: tar xzf jetson_models_cache.tgz -C ~   (whisper en ~/.cache/huggingface, YOLO en ~/models)"
echo "Los engines TensorRT de JetPack 6 (TRT 10.3) NO sirven en TRT 10.16: ~/build_yolo_trt.sh ~/models/yolo/*.onnx"

echo; [ $APPLY -eq 1 ] && echo "Listo." || echo "DRY-RUN: nada ejecutado. Repetir con --apply."
