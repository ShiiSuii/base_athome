#!/bin/bash
export PATH=/opt/ros/humble/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/athome/.gemini/antigravity/bin:/home/athome/.config/Antigravity/bin:/home/athome/bin:/home/athome/solverRos2_ws/install/solver_navigation/bin:/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/athome/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin
source /opt/ros/humble/setup.bash
if [ -f /home/ubuntu/athome_ws/install/setup.bash ]; then
    source /home/ubuntu/athome_ws/install/setup.bash
fi

export PATH=/opt/ros/humble/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/athome/.gemini/antigravity/bin:/home/athome/.config/Antigravity/bin:/home/athome/bin:/home/athome/solverRos2_ws/install/solver_navigation/bin:/opt/ros/jazzy/opt/gz_msgs_vendor/bin:/opt/ros/jazzy/opt/gz_tools_vendor/bin:/opt/ros/jazzy/opt/gz_ogre_next_vendor/bin:/opt/ros/jazzy/bin:/home/athome/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin:/snap/bin
export LD_LIBRARY_PATH=/opt/ros/humble/lib/aarch64-linux-gnu:/opt/ros/humble/lib:/usr/local/cuda/lib64:/home/athome/base_athome_ws/install/urg_node/lib:/home/athome/solverRos2_ws/install/urg_node/lib:/opt/ros/jazzy/opt/gz_sim_vendor/lib:/opt/ros/jazzy/opt/gz_sensors_vendor/lib:/opt/ros/jazzy/opt/gz_physics_vendor/lib:/opt/ros/jazzy/opt/sdformat_vendor/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/opt/gz_gui_vendor/lib:/opt/ros/jazzy/opt/gz_transport_vendor/lib:/opt/ros/jazzy/opt/gz_rendering_vendor/lib:/opt/ros/jazzy/opt/gz_plugin_vendor/lib:/opt/ros/jazzy/opt/gz_fuel_tools_vendor/lib:/opt/ros/jazzy/opt/gz_msgs_vendor/lib:/opt/ros/jazzy/opt/gz_common_vendor/lib:/opt/ros/jazzy/opt/gz_math_vendor/lib:/opt/ros/jazzy/opt/gz_utils_vendor/lib:/opt/ros/jazzy/opt/gz_tools_vendor/lib:/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib:/opt/ros/jazzy/opt/gz_cmake_vendor/lib:/opt/ros/jazzy/lib
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
export ROS_DOMAIN_ID=10

# Calibracion optima de ganancia para microfono RØDE VideoMic GO II (+18dB)
amixer -c 0 sset 'Mic' 18 2>/dev/null

echo 'Iniciando Cabeza: RealSense + Pan-Tilt + Whisper Audio (Deteccion Adaptativa Bilingue ES/EN)'
# Iniciar controlador CAN del brazo Piper y camara de muneca Orbbec
pgrep -f piper_arm_controller.py >/dev/null || nohup python3 /home/ubuntu/piper_arm_controller.py > /tmp/piper_arm_controller.log 2>&1 < /dev/null &
pgrep -f stream_orbbec.py >/dev/null || nohup python3 /home/ubuntu/stream_orbbec.py > /tmp/stream_orbbec.log 2>&1 < /dev/null &

exec ros2 launch robot_bringup_jetson jetson_head.launch.py enable_audio:=true language:=auto 
