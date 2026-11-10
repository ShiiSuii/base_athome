#!/bin/bash
# ==============================================================================
# Script y alias "restaurant" para la prueba Restaurant (RoboCup@Home 2026.2 §5.5)
# Levanta el stack completo:
#   - Modo REAL (por defecto): dominio 10, base, LiDAR, RealSense, joints,
#     bag_finder, LLM, VLM, SLAM + Nav2 (restaurant_nav) y restaurant_node.
#   - Modo SIM=1: dominio 42, Gazebo (worlds/restaurant.sdf), ATHOME_SIM_CUSTOMERS,
#     bag_finder, LLM, VLM, SLAM + Nav2 y restaurant_node con use_sim_time:=true.
#     SIM=1 ATHOME_SIM_ARENA=1: lo mismo en el comedor de la casa completa (worlds/arena.sdf), SLAM sin mapa previo.
# ==============================================================================
set -u
ARM=${ARM:-0}  # ARM=1: brazo opcional; requiere poses calibradas y habilitacion humana previa
ARM_MOTION=false
[ "$ARM" = 1 ] && ARM_MOTION=true
# Parametro acotado al nodo Restaurant: nunca propagar este permiso al elevador.
ARM_ARG=(-p "arm:=$ARM_MOTION" -p "restaurant_node:allow_motion:=$ARM_MOTION")
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../.." && pwd)

set +u
source /opt/ros/jazzy/setup.bash
if [ -f "$WS/install/setup.bash" ]; then
  source "$WS/install/setup.bash"
fi
set -u

. "$HERE/session_lib.sh"

nav_up() { timeout 6 ros2 action list 2>/dev/null | grep -q '^/navigate_to_pose$'; }
# Alguien PUBLICA el topico. "ros2 topic list" tambien lista topicos que solo tienen suscriptores: asi el lidar no se lanzaba.
has_publisher() { timeout 6 ros2 topic info "$1" 2>/dev/null | grep -Eq '^Publisher count: [1-9]'; }
# Debug visual (RVIZ=0 lo apaga): mapa SLAM, costmaps, lidar completo y el que usa Nav2, plan y camara de la cabeza.
start_rviz() {
  [ "${RVIZ:-1}" = 1 ] || return 0
  session_start rviz env DISPLAY="${DISPLAY:-:1}" nice -n 5 rviz2 -d "$WS/src/base_athome_skills/config/restaurant.rviz"
}

restaurant_processes_idle() {
  # Se busca el programa que corre, no cualquier linea que lo nombre (un agente o un editor con ese texto no cuenta)
  if pgrep -f 'python3? [^ ]*hri_receptionist[^ ]*\.py|lib/base_athome_perception/hri_receptionist' >/dev/null; then
    echo "El receptionist esta corriendo: usa el mismo microfono. Cerralo antes."
    exit 1
  fi
  if pgrep -f 'lib/base_athome_skills/commander( |$)|^/bin/bash [^ ]*scripts/commander\.sh' >/dev/null; then
    echo "Hay una sesion de commander abierta. Cerrala antes."
    exit 1
  fi
  if pgrep -f 'lib/base_athome_skills/restaurant_node' >/dev/null; then   # el ejecutable, no cualquier linea que lo nombre
    echo "Ya hay un restaurant_node abierto. Cerralo antes."
    exit 1
  fi
}

restaurant_graph_snapshot() {
  # Equivalente a node list --no-daemon, incluyendo acciones en la MISMA vista DDS.
  # Jazzy no ofrece --no-daemon en `ros2 action list`.
  timeout "$1" python3 - <<'PYTHON'
from argparse import Namespace
from ros2cli.node.direct import DirectNode
with DirectNode(Namespace(spin_time=1.0)) as node:
    for name, namespace in node.get_node_names_and_namespaces():
        print("node " + namespace.rstrip("/") + "/" + name)
    for name, _ in node.get_action_names_and_types():
        print("action " + name)
PYTHON
}

restaurant_graph_idle() {
  local graph conflicts='' remaining deadline=$SECONDS
  # Solo se concede la espera de descubrimiento cuando acabamos de cerrar huerfanos.
  [ "${SESSION_ORPHANS_STOPPED:-0}" -gt 0 ] && deadline=$((SECONDS + 20))
  while :; do
    remaining=$((deadline - SECONDS))
    # No iniciar otra consulta de 6 s si ya se agoto la espera de 20 s.
    [ "$remaining" -gt 0 ] || [ -z "$conflicts" ] || break
    [ "$remaining" -gt 0 ] || remaining=6
    [ "$remaining" -le 6 ] || remaining=6
    graph=$(restaurant_graph_snapshot "$remaining" 2>/dev/null) || {
      echo "No se pudo comprobar la configuracion ROS activa; inicio rechazado."
      [ -z "$conflicts" ] || echo "Ultimos conflictos encontrados: $conflicts"
      return 1
    }
    conflicts=$(echo "$graph" | grep -E '(^node /(.*/)?(amcl|map_server|slam_toolbox|planner_server|controller_server|behavior_server|bt_navigator|lifecycle_manager_navigation|lifecycle_manager_localization|lifecycle_manager_slam)$|^action /(.*/)?navigate_to_pose$)')
    [ -n "$conflicts" ] || return 0
    [ "$SECONDS" -lt "$deadline" ] || break
    sleep 1
  done
  echo "Hay navegacion, localizacion o un mapa activos. Restaurant requiere su propio SLAM nuevo."
  echo "$conflicts"
  echo "Inicio rechazado; no se reutiliza ni se detiene el stack existente."
  return 1
}

restaurant_nav_ready() {
  local nodes
  nodes=$(timeout 6 ros2 node list 2>/dev/null) || return 1
  echo "$nodes" | grep -Eq '(^|/)(amcl|map_server)$' && return 1
  timeout 6 ros2 param get /slam_toolbox mode 2>/dev/null | grep -qx 'String value is: mapping' || return 1
  timeout 6 ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q '^active ' || return 1
  timeout 6 ros2 param get /planner_server GridBased.allow_unknown 2>/dev/null | grep -qx 'Boolean value is: True' || return 1
  nav_up
}

wait_restaurant_nav() {
  local attempt
  for attempt in $(seq 15); do
    restaurant_nav_ready && return 0
    sleep 2
  done
  echo "Restaurant no tiene SLAM mapping activo y Nav2 compatible; inicio rechazado."
  return 1
}

# ------------------------------------------------------------------------------
# MODO SIMULACION (SIM=1)
# ------------------------------------------------------------------------------
if [ "${SIM:-0}" = 1 ]; then
  export ROS_DOMAIN_ID=${SIM_DOMAIN:-42}
  export ATHOME_SIM=1
  if [ "${ATHOME_SIM_ARENA:-0}" = 1 ]; then
    # Arena (worlds/arena.sdf): el comedor de la casa. El robot arranca junto a la barra mirando a las mesas; clientes en mesas 1 y 3.
    export ATHOME_SIM_WORLD=arena.sdf
    export ATHOME_SIM_START="${ATHOME_SIM_START:-7.75,4.4,-90}"
    export ATHOME_SIM_CUSTOMERS="${ATHOME_SIM_CUSTOMERS:-6.5,3.5,right,seated;8.55,0.1,left,seated}"
  else
    export ATHOME_SIM_WORLD=restaurant.sdf   # el simulador crea los clientes en este mundo: sin esto los pedia en otro y no aparecian
    export ATHOME_SIM_CUSTOMERS="${ATHOME_SIM_CUSTOMERS:-2.0,2.8,right,seated;3.0,-2.9,left,seated;-2.6,0.0,right,standing}"   # clientes sentados en las mesas 1 y 3 de restaurant.sdf (12 x 9 m); el ultimo es el barman, de pie detras de la barra (x=-2), a espaldas del arranque
  fi
  export ATHOME_SIM_NAV2=1      # la navegacion la hace Nav2 + SLAM, no el simulador
  [ -f "$HOME/cyclonedds_local.xml" ] && export CYCLONEDDS_URI=file://$HOME/cyclonedds_local.xml
  session_init "${RESTAURANT_LOG_DIR:-/tmp}/restaurant_$$" || exit 1
  restaurant_graph_idle || exit 1
  restaurant_processes_idle

  echo "============================================================"
  echo "MODO SIMULACION: ROS_DOMAIN_ID=$ROS_DOMAIN_ID (Gazebo)"
  echo "Clientes en: $ATHOME_SIM_CUSTOMERS"
  echo "============================================================"

  REST_WORLD="$WS/simulation/base_athome_sim/worlds/$ATHOME_SIM_WORLD"
  W_ARG=()
  if [ -f "$REST_WORLD" ]; then
    W_ARG=(world:="$REST_WORLD")
  else
    echo "AVISO: $REST_WORLD aun no existe, usando mundo por defecto de sim.launch.py"
  fi

  GUI=false
  [ "${GZ_GUI:-0}" = 1 ] && GUI=true
  RV=false     # el RViz de Restaurant se abre aparte (start_rviz), con costmaps y plan

  session_start sim ros2 launch base_athome_sim sim.launch.py gazebo:=true gz_gui:=$GUI rviz:=$RV "${W_ARG[@]}"
  echo "Esperando arranque de Gazebo..."
  sleep 15

  export OMP_NUM_THREADS=2
  session_start bag_finder python3 "$WS/src/base_athome_perception/scripts/bag_finder.py"

  LLM_PY=$HOME/.local/share/llm-venv/bin/python
  [ -x "$LLM_PY" ] && session_start llm env -u PYTHONPATH nice -n 10 "$LLM_PY" "$HERE/llm_server.py"

  VLM_MODEL=$HOME/.local/share/base_athome/models/SmolVLM-500M-Instruct-Q8_0.gguf
  [ -x "$LLM_PY" ] && [ -f "$VLM_MODEL" ] && session_start vlm env -u PYTHONPATH nice -n 10 "$LLM_PY" "$HERE/vlm_server.py"

  echo "Levantando navegacion con SLAM (restaurant_nav)..."
  session_start nav ros2 launch base_athome_skills restaurant_nav.launch.py use_sim_time:=false   # nuestra simulacion publica con la hora real, como nav2_sim.launch.py
  wait_restaurant_nav || exit 1
  start_rviz

  echo "Iniciando restaurant_node..."
  AUTO_ARG=(); [ "${AUTO:-0}" = 1 ] && AUTO_ARG=(-p auto_start:=true)   # AUTO=1: sin esperar ENTER (pruebas automaticas)
  session_run_terminal ros2 run base_athome_skills restaurant_node --ros-args "${AUTO_ARG[@]}" "${ARM_ARG[@]}"
  exit $?     # al terminar la mision se cierra toda la sesion: quedarse esperando dejaba SLAM y Gazebo abiertos
fi

# ------------------------------------------------------------------------------
# MODO ROBOT REAL (POR DEFECTO)
# ------------------------------------------------------------------------------
export ROS_DOMAIN_ID=10
# Procesos de una corrida anterior que murio sin limpiar: se cierran antes de decidir si hay otro stack activo.
session_init "${RESTAURANT_LOG_DIR:-/tmp}/restaurant_$$" || exit 1
restaurant_graph_idle || exit 1
session_start audio_keepalive python3 "$HERE/audio_keepalive.py"   # sonido breve cada 5 s: el parlante Bluetooth no se apaga
restaurant_processes_idle

echo "============================================================"
echo "ROBOT REAL: se mueve solo; E-stop a mano"
echo "============================================================"

# 1. Base y actuadores
if ! has_publisher /odom; then
  session_start base ros2 launch base_athome_bringup base.launch.py
fi

# 2. LiDAR Hokuyo
if ! has_publisher /scan; then
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
  session_start torso ros2 run base_athome_bringup torso_state --ros-args -p allow_motion:=false
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
echo "Levantando SLAM y Nav2 (restaurant_nav)..."
session_start nav ros2 launch base_athome_skills restaurant_nav.launch.py
wait_restaurant_nav || exit 1
start_rviz

# 9. Nodo visible con stdin preservado; wait permite ejecutar los traps inmediatamente
echo "Iniciando nodo Restaurant..."
AUTO_ARG=(); [ "${AUTO:-0}" = 1 ] && AUTO_ARG=(-p auto_start:=true)
session_run_terminal ros2 run base_athome_skills restaurant_node --ros-args "${AUTO_ARG[@]}" "${ARM_ARG[@]}"
