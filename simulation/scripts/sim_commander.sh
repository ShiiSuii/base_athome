#!/bin/bash
# Simulacion + mission_executor + commander (ROS_DOMAIN_ID=42). Cierra TODO lo propio por PID al terminar.
#   Interactivo: [RVIZ=1] simulation/scripts/sim_commander.sh        -> abrir http://127.0.0.1:8770 (Ctrl-C cierra)
#   Automatico:  simulation/scripts/sim_commander.sh "anda a la mesa" "volve a casa"
#     corre cada pedido por la API web, espera el fin y sale con rc!=0 si alguno no termina SUCCEEDED.
set -u
WS=$(cd "$(dirname "$0")/../.." && pwd)
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export SIM_MISSION_TAG=$$ ROS_DOMAIN_ID=${SIM_DOMAIN:-42} ATHOME_SIM=1 ATHOME_SIM_OBJECT_LABEL=${ATHOME_SIM_OBJECT_LABEL:-bottle}
[ -f "$HOME/cyclonedds_local.xml" ] && export CYCLONEDDS_URI=file://$HOME/cyclonedds_local.xml
LOG=${SIM_LOG_DIR:-/tmp}/sim_commander_$$; mkdir -p "$LOG"
MAP=$WS/simulation/base_athome_sim/config/sim_semantic_map.yaml
export ATHOME_SIM_POSES=${ATHOME_SIM_POSES:-$WS/simulation/config/arm_poses_sim.yaml}
RV=false; [ "${RVIZ:-0}" = "1" ] && RV=true
PIDS=()
start() { setsid "$@" & PIDS+=($!); }
cleanup() {
  trap - EXIT HUP INT TERM
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do kill -TERM -- -"${PIDS[i]}" 2>/dev/null; sleep 2; done
  for p in $(pgrep -u "$USER" -x move_group); do
    tr "\0" "\n" < /proc/$p/environ 2>/dev/null | grep -qx "SIM_MISSION_TAG=$$" && kill -TERM "$p"
  done
  for _ in $(seq 30); do
    alive=0; for g in "${PIDS[@]}"; do pgrep -g "$g" >/dev/null 2>&1 && alive=1; done
    [ $alive = 0 ] && break; sleep 0.5
  done
  [ $alive = 1 ] && echo "AVISO: quedan procesos propios de la simulacion"
  echo "logs: $LOG"
}
trap cleanup EXIT; trap 'exit 130' HUP INT TERM
[ "${NAV2:-0}" = 1 ] && export ATHOME_SIM_NAV2=1
# GAZEBO=1: mundo con fisica y sensores reales de gz sim. GZ_GUI=1 abre su ventana.
if [ "${GAZEBO:-0}" = 1 ]; then
  export GZ_PARTITION=sim_$$
  GUI=false; [ "${GZ_GUI:-0}" = 1 ] && GUI=true
  start ros2 launch base_athome_sim sim.launch.py rviz:=$RV gazebo:=true gz_gui:=$GUI > "$LOG/sim.log" 2>&1
  sleep 25
  # NAV2=1: navega con Nav2 de verdad, con los parametros del robot, sobre el mapa de este mundo
  if [ "${NAV2:-0}" = 1 ]; then
    export ATHOME_SIM_MAP=$WS/simulation/config/casa_basura.yaml
    start ros2 launch base_athome_sim nav2_sim.launch.py > "$LOG/nav2.log" 2>&1
    sleep 15
  fi
else
  start ros2 launch base_athome_sim sim.launch.py rviz:=$RV > "$LOG/sim.log" 2>&1
  sleep 8
fi
ARGS=(--ros-args -p semantic_map:="$MAP" -p waypoints:=/nonexistent)
# La percepcion real de la bolsa, contra la imagen dibujada por la simulacion
[ -n "${ATHOME_SIM_BAG:-}" ] && start python3 "$WS/src/base_athome_perception/scripts/bag_finder.py" > "$LOG/bag_finder.log" 2>&1
start ros2 run base_athome_skills mission_executor "${ARGS[@]}" > "$LOG/executor.log" 2>&1
# COMMANDER_ARGS: parametros extra, p. ej. "-p voice:=true -p tts:=false" para probar el dialogo con un ASR falso
start ros2 run base_athome_skills commander "${ARGS[@]}" ${COMMANDER_ARGS:-} > "$LOG/commander.log" 2>&1
sleep 4
if [ $# -eq 0 ]; then echo "Commander listo: http://127.0.0.1:8770  (Ctrl-C cierra todo)"; wait; exit 0; fi
python3 "$(dirname "$0")/commander_client.py" "$@"
