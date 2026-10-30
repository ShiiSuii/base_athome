# Alias de Base@Home para la NUC. Se carga desde ~/.bashrc:
#   [ -f ~/base_athome_ws/scripts/aliases/nuc_aliases.sh ] && source ~/base_athome_ws/scripts/aliases/nuc_aliases.sh
# `ayuda_robot` lista todo. Redefine head_start/head_stop/head_status para usar systemd.

WS_ATHOME=/home/athome/base_athome_ws
JETSON_HOST=ubuntu@10.42.0.12
_ros() { source /opt/ros/jazzy/setup.bash && [ -f "$WS_ATHOME/install/setup.bash" ] && source "$WS_ATHOME/install/setup.bash"; }

# --- Herramientas ---
urdf_editor() {  # editor URDF/Xacro en http://127.0.0.1:8765 (solo preview)
    (_ros; cd "$WS_ATHOME/tools/urdf_editor" && \
     { [ -d .venv ] || { python3 -m venv .venv && .venv/bin/pip install -q -r requirements.txt; }; } && \
     (sleep 2; xdg-open http://127.0.0.1:8765/ >/dev/null 2>&1 &) && .venv/bin/python -m backend "$@")
}
alias editor_urdf='urdf_editor'
gpsr() {  # gpsr "bring me a coke from the kitchen"  -> plan JSON + frase
    (cd "$WS_ATHOME/tools/gpsr_parser" && env -u PYTHONPATH python3 -m gpsr_parser "$@")
}
alias gpsr_parser='gpsr'
alias puerta='_ros && ros2 launch base_athome_skills door_open_detector.launch.py'
alias detector_puerta='puerta'
alias compilar='(cd $WS_ATHOME && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install) && _ros'

# --- Red NUC <-> Jetson ---
alias jetson='ssh $JETSON_HOST'
alias red_jetson='$WS_ATHOME/scripts/net/check_nuc_jetson_link.sh'
alias check_red='red_jetson'
alias deploy_jetson='$WS_ATHOME/jetson_head/scripts/deploy_to_jetson.sh --apply'
alias deploy_jetson_dry='$WS_ATHOME/jetson_head/scripts/deploy_to_jetson.sh'

# --- Cabeza en la Jetson (servicio systemd jetson-head) ---
alias head_start='ssh $JETSON_HOST sudo systemctl start jetson-head && echo "jetson-head iniciado"'
alias head_stop='ssh $JETSON_HOST sudo systemctl stop jetson-head && echo "jetson-head detenido"'
alias head_restart='ssh $JETSON_HOST sudo systemctl restart jetson-head && echo "jetson-head reiniciado"'
alias head_status='ssh $JETSON_HOST "systemctl is-active jetson-head; pgrep -af \"piper_arm|stream_orbbec|yolo_trt|pan_tilt|whisper\" | cut -c1-120"'
alias head_logs='ssh -t $JETSON_HOST journalctl -u jetson-head -f -o cat'
alias cabeza_start='head_start'; alias cabeza_stop='head_stop'; alias cabeza_status='head_status'; alias cabeza_logs='head_logs'
alias jetson_stats='ssh -t $JETSON_HOST tegrastats --interval 1000'

# --- Monitoreo ROS (solo lectura) ---
alias topicos='_ros && ros2 topic list -t'
alias nodos='_ros && ros2 node list'
alias brazo_estado='_ros && ros2 topic echo /arm/status'
alias pantilt_conectado='_ros && ros2 topic echo --once /pan_tilt/connected'
alias voz='_ros && ros2 topic echo /voice/transcription'
alias detecciones='_ros && ros2 topic echo /arm_camera/detections --field detections'
alias camara_muneca_hz='_ros && ros2 topic hz /arm_camera/image_raw'
alias camara_muneca='_ros && ros2 run rqt_image_view rqt_image_view /arm_camera/image_raw'

ayuda_robot() {
    cat <<'EOF'
Workspace:     ws | devel | sws  (= source install/setup.bash)
Herramientas:  urdf_editor | gpsr "<comando>" | puerta | compilar | editor_mapa
Red/Jetson:    jetson (ssh) | red_jetson | deploy_jetson[_dry] | jetson_stats
Cabeza:        head_start | head_stop | head_restart | head_status | head_logs
Manipulacion:  manipulacion (RViz+brazo) | rviz_manipulacion | agarrar_botella | teach_record
Receptionist:  receptionist | hri_sim | limpiar_memoria | memoria_personas
ROS (lectura): topicos | nodos | brazo_estado | pantilt_conectado | voz | detecciones | camara_muneca[_hz]
Existentes:    mapeo2d | navegacion | rviznav | lidar | receptionist | rostros | cara | piper | stepper ...
EOF
}
alias ayuda='ayuda_robot'

# --- Receptionist / memoria de personas ---
alias limpiar_memoria='_ros && ros2 service call /hri/clear_memory std_srvs/srv/Trigger'
alias memoria_personas='python3 -c "import json,os;d=json.load(open(os.path.expanduser(\"~/.local/share/base_athome/faces_database/profiles.json\")));[print(k,v[\"name\"],\"persistente\" if v[\"attributes\"].get(\"persistent\") else v[\"attributes\"].get(\"session_id\"),v[\"samples_count\"]) for k,v in d.items()]"'

# --- Cargar el workspace (equivalente ROS 2 de "source devel/setup.bash" de ROS 1) ---
alias ws='source /opt/ros/jazzy/setup.bash && source $WS_ATHOME/install/setup.bash && echo "workspace cargado: $WS_ATHOME/install"'
alias devel='ws'
alias sws='ws'

# --- Manipulacion (brazo Piper + camara de muneca) ---
unalias manipulacion 2>/dev/null
manipulacion() {  # abre RViz con el URDF (si no esta abierto) y lanza la manipulacion autonoma
    _ros
    if ! pgrep -f "^/opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher" >/dev/null; then
        (DISPLAY="${DISPLAY:-:1}" ros2 launch hoverboard_base view_robot.launch.py > /tmp/view_robot.log 2>&1 &)
        sleep 3
    fi
    DISPLAY="${DISPLAY:-:1}" /home/athome/ros2_vision_venv/bin/python3 "$WS_ATHOME/src/base_athome_perception/scripts/manipulacion_autonoma.py" "$@"
}
alias manipular='manipulacion'
alias rviz_manipulacion='_ros && DISPLAY="${DISPLAY:-:1}" ros2 launch hoverboard_base view_robot.launch.py'
