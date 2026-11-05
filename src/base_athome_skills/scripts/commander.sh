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
. "$HERE/session_lib.sh"
session_init "${COMMANDER_LOG_DIR:-/tmp}/commander_$$"
nav_up() { timeout 6 ros2 action list 2>/dev/null | grep -q '^/navigate_to_pose$'; }
if nav_up; then
  echo "Nav2 ya esta corriendo."
  "$HERE/nav_map_sync.sh"
elif [ "${COMMANDER_NO_NAV:-0}" = 1 ]; then
  echo "Nav2 no esta corriendo (COMMANDER_NO_NAV=1: no se levanta)."
else
  echo "Levantando Nav2..."
  session_start nav ros2 launch base_athome_navigation nav2_bringup.launch.py
  for _ in $(seq 20); do nav_up && break; sleep 2; done
  nav_up && echo "Nav2 listo; se cierra con commander, no abras navegacion aparte. Verifica en RViz la ubicacion." || echo "AVISO: Nav2 no respondio (ver $SESSION_LOG/nav.log)."
fi
# Elevador, cabeza y deteccion de la bolsa
if [ -e /dev/serial/by-id/usb-1a86_USB_Single_Serial_5CBD013609-if00 ] && ! pgrep -f 'stepper_controller' >/dev/null; then
  session_start torso ros2 run base_athome_bringup torso_state --ros-args -p allow_motion:=true
else
  echo "AVISO: elevador sin conectar o con el CLI abierto: 'take out the trash' no va a poder bajar."
fi
if ! timeout 6 ros2 topic list 2>/dev/null | grep -q '^/camera/camera/color/image_raw$'; then
  session_start realsense ros2 launch base_athome_bringup realsense_nuc.launch.py
fi
session_start bag_finder /home/athome/ros2_vision_venv/bin/python3 "$WS/src/base_athome_perception/scripts/bag_finder.py"
session_start executor ros2 run base_athome_skills mission_executor --ros-args -p mode:=real -p supervised:=true \
  -p allow_manipulation_real:=true
session_start commander ros2 run base_athome_skills commander --ros-args -p real:=true -p voice:=true
echo "ROBOT REAL: decile que hacer en ingles, responde 'yes' para que se mueva, 'stop' para cancelar. E-stop a mano."
echo "Plan y avance: http://127.0.0.1:8770  (Ctrl-C cierra)"
sleep 3
if timeout 6 ros2 topic echo --once /torso/status 2>/dev/null | grep -q '"calibrated": false\|calibrated.: false'; then
  echo "Elevador sin calibrar. Para moverlo: despeja el recorrido y corre calibrar_elevador"
fi
wait
