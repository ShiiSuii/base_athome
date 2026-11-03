#!/bin/bash
# Levanta base_athome_sim + mission_executor en ROS_DOMAIN_ID=42, corre un plan y cierra TODO por PID.
#   src/base_athome_skills/scripts/sim_mission.sh <plan.yaml> [mapa_semantico.yaml] [args de mission_run]
set -u
PLAN=${1:?plan.yaml}; MAP=${2:-}; shift; [ $# -gt 0 ] && shift
WS=$(cd "$(dirname "$0")/../../.." && pwd)
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=42 ATHOME_SIM=1 ATHOME_SIM_OBJECT_LABEL=${ATHOME_SIM_OBJECT_LABEL:-bottle}
[ -f "$HOME/cyclonedds_local.xml" ] && export CYCLONEDDS_URI=file://$HOME/cyclonedds_local.xml
LOG=${SIM_LOG_DIR:-/tmp}/sim_mission_$$; mkdir -p "$LOG"
setsid ros2 launch base_athome_sim sim.launch.py rviz:=false > "$LOG/sim.log" 2>&1 & SIM=$!
sleep 8
EXTRA=(); [ -n "$MAP" ] && EXTRA=(-p semantic_map:="$MAP" -p waypoints:=/nonexistent)
setsid ros2 run base_athome_skills mission_executor --ros-args "${EXTRA[@]}" > "$LOG/executor.log" 2>&1 & EXE=$!
sleep 4
ros2 run base_athome_skills mission_run "$PLAN" "$@"; RC=$?
# cierre ordenado de los grupos de procesos propios (incluye move_group lanzado por la manipulacion)
kill -TERM -- -$EXE 2>/dev/null; sleep 4; kill -TERM -- -$SIM 2>/dev/null
for _ in $(seq 20); do pgrep -g $EXE,$SIM >/dev/null 2>&1 || break; sleep 0.5; done
LEFT=$(pgrep -g $EXE,$SIM 2>/dev/null | tr "\n" " "); [ -n "$LEFT" ] && echo "AVISO: procesos sin cerrar: $LEFT (SIGTERM manual)"
echo "logs: $LOG (rc=$RC)"; exit $RC
