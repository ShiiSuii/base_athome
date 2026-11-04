#!/bin/bash
# Alias `commander`. Web en http://127.0.0.1:8770; Ctrl-C cierra lo que abrio.
#   commander              simulacion (dominio 42, no mueve el robot)
#   commander real         robot real, solo navegacion (requiere Nav2 corriendo)
#   commander real manip   robot real, con manipulacion
# Escribir `real` es la declaracion de supervision humana: el robot se mueve al tocar Ejecutar.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../.." && pwd)
MODE=${1:-sim}; EXTRA=${2:-}
case "$MODE $EXTRA" in
  "sim "|"real "|"real manip") ;;
  *) echo "uso: commander [sim | real | real manip]"; exit 2 ;;
esac
if [ "$MODE" = sim ]; then
  [ -n "${DISPLAY:-}" ] && export RVIZ=${RVIZ:-1}
  exec "$HERE/sim_commander.sh"
fi
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=10
MANIP=false; [ "$EXTRA" = manip ] && MANIP=true
LOG=${COMMANDER_LOG_DIR:-/tmp}/commander_$$; mkdir -p "$LOG"
setsid ros2 run base_athome_skills mission_executor --ros-args -p mode:=real -p supervised:=true \
  -p allow_manipulation_real:=$MANIP > "$LOG/executor.log" 2>&1 & EXE=$!
setsid ros2 run base_athome_skills commander --ros-args -p real:=true > "$LOG/commander.log" 2>&1 & CMD=$!
cleanup() {
  trap - EXIT INT TERM
  kill -TERM -- -$CMD -$EXE 2>/dev/null
  for _ in $(seq 20); do pgrep -g $CMD,$EXE >/dev/null 2>&1 || break; sleep 0.5; done
  echo "logs: $LOG"
}
trap cleanup EXIT; trap 'exit 130' INT TERM
echo "ROBOT REAL (manipulacion: $MANIP). El robot se mueve al tocar Ejecutar: supervisalo, E-stop a mano."
echo "Commander: http://127.0.0.1:8770  (Ctrl-C cierra)"
wait
