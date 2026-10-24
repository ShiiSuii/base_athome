#!/bin/bash
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/athome_ws/install/setup.bash ]; then
    source /home/ubuntu/athome_ws/install/setup.bash
fi
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
export ROS_DOMAIN_ID=10

echo "=================================================="
echo "  Iniciando Cabeza: RealSense + Pan-Tilt + ASR    "
echo "=================================================="
exec ros2 launch robot_bringup_jetson jetson_head.launch.py "$@"
