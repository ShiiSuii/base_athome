#!/bin/bash
# Demo en pantalla: "take out the trash" en simulacion, con RViz siguiendo al robot. Ctrl-C cierra todo. No mueve el robot.
HERE=$(cd "$(dirname "$0")" && pwd)
export DISPLAY=${DISPLAY:-:1} RVIZ=1 ATHOME_SIM_BAG=${ATHOME_SIM_BAG:-1.5,1.2,0.30} ATHOME_SIM_OBJECT_LABEL=handbag
export COMMANDER_ARGS="-p voice:=true -p tts:=false"
[ -z "${XAUTHORITY:-}" ] && [ -f "/run/user/$(id -u)/gdm/Xauthority" ] && export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
(
  sleep 24
  export ROS_DOMAIN_ID=42
  [ -f "$HOME/cyclonedds_local.xml" ] && export CYCLONEDDS_URI=file://$HOME/cyclonedds_local.xml
  source /opt/ros/jazzy/setup.bash
  echo "Le digo: take out the trash"
  ros2 topic pub --once /sim/speech std_msgs/String "{data: 'take out the trash'}" >/dev/null
  sleep 6
  echo "Le digo: yes"
  ros2 topic pub --once /sim/speech std_msgs/String "{data: 'yes'}" >/dev/null
) &
exec "$HERE/sim_commander.sh"
