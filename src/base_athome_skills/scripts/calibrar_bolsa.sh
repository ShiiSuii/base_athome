#!/bin/bash
# Alias `calibrar_bolsa`. Solo lectura: abre lo que haga falta para ver con las dos camaras, sin mover nada. Ctrl-C cierra.
HERE=$(cd "$(dirname "$0")" && pwd)
WS=$(cd "$HERE/../../.." && pwd)
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=10 DISPLAY=${DISPLAY:-:1}
[ -z "${XAUTHORITY:-}" ] && [ -f "/run/user/$(id -u)/gdm/Xauthority" ] && export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
. "$HERE/session_lib.sh"
session_init "/tmp/calibrar_bolsa_$$"
NODES=$(timeout 6 ros2 node list 2>/dev/null)
if ! echo "$NODES" | grep -q '/robot_state_publisher'; then
  echo "Falta robot_state_publisher: abri 'odometria' en otra terminal y volve a correr esto."; exit 1
fi
echo "$NODES" | grep -q '/joint_state_publisher' || session_start joints ros2 run joint_state_publisher joint_state_publisher \
  "$WS/src/hoverboard_base/urdf/robot.urdf" --ros-args -p rate:=10 -p "source_list:=['/arm/joint_states','/pan_tilt/joint_states','/torso/joint_states']"
timeout 6 ros2 topic list 2>/dev/null | grep -q '^/camera/camera/color/image_raw$' || session_start realsense ros2 launch base_athome_bringup realsense_nuc.launch.py
# sin allow_motion: este torso_state solo lee la altura
if ! echo "$NODES" | grep -q '/torso_state' && ! pgrep -f 'stepper_controller' >/dev/null && [ -e /dev/serial/by-id/usb-1a86_USB_Single_Serial_5CBD013609-if00 ]; then
  session_start torso ros2 run base_athome_bringup torso_state
fi
sleep 4
# En segundo plano: asi Ctrl-C o TERM lo cierran enseguida. Al salir con Q, se cierra todo lo demas.
session_start debug /home/athome/ros2_vision_venv/bin/python3 "$WS/src/base_athome_perception/scripts/calibrar_bolsa.py" "$@"
echo "Ventana 'Debug bolsa' abierta. Q sobre la ventana o Ctrl-C cierran. Salida: $SESSION_LOG/debug.log"
wait "${SESSION_PIDS[-1]}"
