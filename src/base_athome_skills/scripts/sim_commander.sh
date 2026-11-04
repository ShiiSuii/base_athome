#!/bin/bash
# Simulacion + mission_executor + commander (ROS_DOMAIN_ID=42). Cierra TODO lo propio por PID al terminar.
#   Interactivo: [RVIZ=1] src/base_athome_skills/scripts/sim_commander.sh        -> abrir http://127.0.0.1:8770 (Ctrl-C cierra)
#   Automatico:  src/base_athome_skills/scripts/sim_commander.sh "anda a la mesa" "volve a casa"
#     corre cada pedido por la API web, espera el fin y sale con rc!=0 si alguno no termina SUCCEEDED.
set -u
WS=$(cd "$(dirname "$0")/../../.." && pwd)
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export SIM_MISSION_TAG=$$ ROS_DOMAIN_ID=42 ATHOME_SIM=1 ATHOME_SIM_OBJECT_LABEL=${ATHOME_SIM_OBJECT_LABEL:-bottle}
[ -f "$HOME/cyclonedds_local.xml" ] && export CYCLONEDDS_URI=file://$HOME/cyclonedds_local.xml
LOG=${SIM_LOG_DIR:-/tmp}/sim_commander_$$; mkdir -p "$LOG"
MAP=$WS/src/base_athome_sim/config/sim_semantic_map.yaml
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
start ros2 launch base_athome_sim sim.launch.py rviz:=$RV > "$LOG/sim.log" 2>&1
sleep 8
ARGS=(--ros-args -p semantic_map:="$MAP" -p waypoints:=/nonexistent)
start ros2 run base_athome_skills mission_executor "${ARGS[@]}" > "$LOG/executor.log" 2>&1
# COMMANDER_ARGS: parametros extra, p. ej. "-p voice:=true -p tts:=false" para probar el dialogo con un ASR falso
start ros2 run base_athome_skills commander "${ARGS[@]}" ${COMMANDER_ARGS:-} > "$LOG/commander.log" 2>&1
sleep 4
if [ $# -eq 0 ]; then echo "Commander listo: http://127.0.0.1:8770  (Ctrl-C cierra todo)"; wait; exit 0; fi
python3 "$(dirname "$0")/commander_client.py" "$@"
