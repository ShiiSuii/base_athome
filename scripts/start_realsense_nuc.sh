#!/bin/bash
# Driver unico de la RealSense D435 de la NUC (pensado para correr supervisado por
# systemd: scripts/systemd/realsense-nuc.service). Un solo dueno de la camara evita
# que dos drivers se peleen por el USB (el receptionist y su launch lanzaban otro).
source /opt/ros/jazzy/setup.bash
source /home/athome/base_athome_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
if [ "$(cat /sys/class/net/enp1s0/carrier 2>/dev/null)" = "1" ]; then
    export CYCLONEDDS_URI=file:///home/athome/cyclonedds.xml
else
    export CYCLONEDDS_URI=file:///home/athome/cyclonedds_local.xml
fi
exec ros2 launch base_athome_bringup realsense_nuc.launch.py
