#!/bin/bash
# Alias `commander`: robot real, con microfono. Un solo comando: levanta Nav2 si no esta, el ejecutor y el commander.
# El robot dice el plan y solo se mueve si le contestan "yes"; "stop" cancela. Ctrl-C cierra lo que abrio.
# Solo para simulacion, sin microfono: scripts/sim_commander.sh
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../.." && pwd)
[ $# -eq 0 ] || { echo "commander no lleva argumentos"; exit 2; }
if pgrep -f 'hri_receptionist' >/dev/null; then
  echo "El receptionist esta corriendo: usa el mismo microfono. Cerralo antes."; exit 1
fi
if ss -ltn | grep -q ':8770 '; then echo "Ya hay un commander abierto (puerto 8770)."; exit 1; fi
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=10
LOG=${COMMANDER_LOG_DIR:-/tmp}/commander_$$; mkdir -p "$LOG"
PIDS=()
start() { name=$1; shift; setsid "$@" > "$LOG/$name.log" 2>&1 & PIDS+=($!); }
cleanup() {
  trap - EXIT INT TERM
  for ((i=${#PIDS[@]}-1; i>=0; i--)); do kill -TERM -- -"${PIDS[i]}" 2>/dev/null; sleep 1; done
  for g in "${PIDS[@]}"; do for _ in $(seq 20); do pgrep -g "$g" >/dev/null 2>&1 || break; sleep 0.5; done; done
  echo "logs: $LOG"
}
trap cleanup EXIT; trap 'exit 130' INT TERM
nav_up() { timeout 6 ros2 action list 2>/dev/null | grep -q '^/navigate_to_pose$'; }
if nav_up; then
  echo "Nav2 ya esta corriendo."
elif [ "${COMMANDER_NO_NAV:-0}" = 1 ]; then
  echo "Nav2 no esta corriendo (COMMANDER_NO_NAV=1: no se levanta)."
else
  echo "Levantando Nav2..."
  start nav ros2 launch base_athome_navigation nav2_bringup.launch.py
  for _ in $(seq 20); do nav_up && break; sleep 2; done
  nav_up && echo "Nav2 listo. Verifica en RViz que el robot este bien ubicado." || echo "AVISO: Nav2 no respondio (ver $LOG/nav.log)."
fi
start executor ros2 run base_athome_skills mission_executor --ros-args -p mode:=real -p supervised:=true \
  -p allow_manipulation_real:=true
start commander ros2 run base_athome_skills commander --ros-args -p real:=true -p voice:=true
echo "ROBOT REAL: decile que hacer en ingles, responde 'yes' para que se mueva, 'stop' para cancelar. E-stop a mano."
echo "Plan y avance: http://127.0.0.1:8770  (Ctrl-C cierra)"
wait
