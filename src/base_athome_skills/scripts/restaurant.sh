#!/bin/bash
# ==============================================================================
# Script y alias "restaurant" para la prueba Restaurant (RoboCup@Home 2026.2 §5.5)
# Levanta el stack completo:
#   - Modo REAL (por defecto): dominio 10, base, LiDAR, RealSense, joints,
#     bag_finder, LLM, VLM, SLAM + Nav2 (restaurant_nav) y restaurant_node.
#   - Modo SIM=1: dominio 42, Gazebo (worlds/restaurant.sdf), ATHOME_SIM_CUSTOMERS,
#     bag_finder, LLM, VLM, SLAM + Nav2 y restaurant_node con use_sim_time:=true.
# ==============================================================================
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../.." && pwd)

if pgrep -f 'hri_receptionist' >/dev/null; then
  echo "El receptionist esta corriendo: usa el mismo microfono. Cerralo antes."
  exit 1
fi
if pgrep -f 'commander' >/dev/null; then
  echo "Hay una sesion de commander abierta. Cerrala antes."
  exit 1
fi
if pgrep -f 'lib/base_athome_skills/restaurant_node' >/dev/null; then   # el ejecutable, no cualquier linea que lo nombre
  echo "Ya hay un restaurant_node abierto. Cerralo antes."
  exit 1
fi

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi
set -u

. "$HERE/session_lib.sh"
session_init "${RESTAURANT_LOG_DIR:-/tmp}/restaurant_$$"

# ------------------------------------------------------------------------------
# MODO SIMULACION (SIM=1)
# ------------------------------------------------------------------------------
if [ "${SIM:-0}" = 1 ]; then
  export ROS_DOMAIN_ID=${SIM_DOMAIN:-42}
  export ATHOME_SIM=1
  export ATHOME_SIM_WORLD=restaurant.sdf   # el simulador crea los clientes en este mundo: sin esto los pedia en otro y no aparecian
  export ATHOME_SIM_NAV2=1      # la navegacion la hace Nav2 + SLAM, no el simulador
  export ATHOME_SIM_CUSTOMERS="${ATHOME_SIM_CUSTOMERS:-1.5,2.3,right,seated;2.5,-2.4,left,seated}"   # sentados en las mesas 1 y 3 de restaurant.sdf
  [ -f "$HOME/cyclonedds_local.xml" ] && export CYCLONEDDS_URI=file://$HOME/cyclonedds_local.xml

  echo "============================================================"
  echo "MODO SIMULACION: ROS_DOMAIN_ID=$ROS_DOMAIN_ID (Gazebo)"
  echo "Clientes en: $ATHOME_SIM_CUSTOMERS"
  echo "============================================================"

  REST_WORLD="$WS/simulation/base_athome_sim/worlds/restaurant.sdf"
  W_ARG=()
  if [ -f "$REST_WORLD" ]; then
    W_ARG=(world:="$REST_WORLD")
  else
    echo "AVISO: $REST_WORLD aun no existe, usando mundo por defecto de sim.launch.py"
  fi

  GUI=false
  [ "${GZ_GUI:-0}" = 1 ] && GUI=true
  RV=false
  [ "${RVIZ:-0}" = 1 ] && RV=true

  session_start sim ros2 launch base_athome_sim sim.launch.py gazebo:=true gz_gui:=$GUI rviz:=$RV "${W_ARG[@]}"
  echo "Esperando arranque de Gazebo..."
  sleep 15

  session_start bag_finder python3 "$WS/src/base_athome_perception/scripts/bag_finder.py"

  LLM_PY=$HOME/.local/share/llm-venv/bin/python
  [ -x "$LLM_PY" ] && session_start llm env -u PYTHONPATH nice -n 10 "$LLM_PY" "$HERE/llm_server.py"

  VLM_MODEL=$HOME/.local/share/base_athome/models/SmolVLM-500M-Instruct-Q8_0.gguf
  [ -x "$LLM_PY" ] && [ -f "$VLM_MODEL" ] && session_start vlm env -u PYTHONPATH nice -n 10 "$LLM_PY" "$HERE/vlm_server.py"

  echo "Levantando navegacion con SLAM (restaurant_nav)..."
  session_start nav ros2 launch base_athome_skills restaurant_nav.launch.py use_sim_time:=false   # nuestra simulacion publica con la hora real, como nav2_sim.launch.py
  sleep 6

  echo "Iniciando restaurant_node..."
  AUTO_ARG=(); [ "${AUTO:-0}" = 1 ] && AUTO_ARG=(-p auto_start:=true)   # AUTO=1: sin esperar ENTER (pruebas automaticas)
  ros2 run base_athome_skills restaurant_node --ros-args "${AUTO_ARG[@]}"
  wait
  exit 0
fi

# ------------------------------------------------------------------------------
# MODO ROBOT REAL (POR DEFECTO)
# ------------------------------------------------------------------------------
export ROS_DOMAIN_ID=10

echo "============================================================"
echo "ROBOT REAL: se mueve solo; E-stop a mano"
echo "============================================================"

nav_up() { timeout 6 ros2 action list 2>/dev/null | grep -q '^/navigate_to_pose$'; }

# 1. Base y actuadores
if ! timeout 6 ros2 topic list 2>/dev/null | grep -q '^/odom$'; then
  session_start base ros2 launch base_athome_bringup base.launch.py
fi

# 2. LiDAR Hokuyo
if ! timeout 6 ros2 topic list 2>/dev/null | grep -q '^/scan$'; then
  session_start lidar ros2 launch urg_node urg_node_launch.py
fi

# 3. Camara RealSense
if ! timeout 6 ros2 topic list 2>/dev/null | grep -q '^/camera/camera/color/image_raw$'; then
  session_start realsense ros2 launch base_athome_bringup realsense_nuc.launch.py
fi

# 4. Joint state publisher (brazo, cabeza, torso)
if ! timeout 6 ros2 node list 2>/dev/null | grep -q '/joint_state_publisher'; then
  session_start joints ros2 run joint_state_publisher joint_state_publisher "$WS/src/hoverboard_base/urdf/robot.urdf" --ros-args -p rate:=10 \
    -p "source_list:=['/arm/joint_states','/pan_tilt/joint_states','/torso/joint_states']"
fi

# 5. Elevador
if [ -e /dev/serial/by-id/usb-1a86_USB_Single_Serial_5CBD013609-if00 ] && ! pgrep -f 'stepper_controller' >/dev/null; then
  session_start torso ros2 run base_athome_bringup torso_state --ros-args -p allow_motion:=true
fi

# 6. Deteccion de comensales (bag_finder con modo raised_hands)
export OMP_NUM_THREADS=2
session_start bag_finder /home/athome/ros2_vision_venv/bin/python3 "$WS/src/base_athome_perception/scripts/bag_finder.py"

# 7. Modelos locales LLM y VLM
LLM_PY=$HOME/.local/share/llm-venv/bin/python
[ -x "$LLM_PY" ] && session_start llm env -u PYTHONPATH nice -n 10 "$LLM_PY" "$HERE/llm_server.py"

VLM_MODEL=$HOME/.local/share/base_athome/models/SmolVLM-500M-Instruct-Q8_0.gguf
[ -x "$LLM_PY" ] && [ -f "$VLM_MODEL" ] && session_start vlm env -u PYTHONPATH nice -n 10 "$LLM_PY" "$HERE/vlm_server.py"

# 8. SLAM Toolbox y Nav2 sin mapa previo
if nav_up; then
  echo "Nav2 ya esta corriendo."
else
  echo "Levantando SLAM y Nav2 (restaurant_nav)..."
  session_start nav ros2 launch base_athome_skills restaurant_nav.launch.py
  for _ in $(seq 15); do nav_up && break; sleep 2; done
fi

# 9. Nodo Restaurant en primer plano
echo "Iniciando nodo Restaurant..."
ros2 run base_athome_skills restaurant_node
wait
