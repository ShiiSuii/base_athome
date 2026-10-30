# Interfaces ROS 2 (HOME-02, extracción estática parcial)

2026-10-29. Obtenido con `git grep` de `create_publisher/subscription/service/client` en `main@1da2388` (excluye `urg_node`). **No medido en ejecución**: tasas, QoS efectivas y tópicos con nombre parametrizado/multilínea pueden faltar. Navegación: solo se listan interfaces, sin cambios.

| Tópico / servicio | Tipo | Productor | Consumidor | Host |
|---|---|---|---|---|
| `/cmd_vel` | Twist | Nav2, `teleop_joy` | `hoverboard_driver` | NUC |
| `/odom`, `/odom_raw` | Odometry | `hoverboard_driver` | Nav2 | NUC |
| `/hoverboard/battery` | BatteryState | `hoverboard_driver` | — | NUC |
| `/hoverboard/reset_odom` | srv Trigger | `hoverboard_driver` | — | NUC |
| `/joy` | Joy | joy_node | `teleop_joy` | NUC |
| `navigate_to_pose` | action NavigateToPose | Nav2 | HRI receptionist (`_check_nav2_ready`) | NUC |
| `/waypoint_markers` | MarkerArray | `waypoint_navigator` | RViz | NUC |
| `/joint_states` | JointState | **dos publicadores**: `pan_tilt_node` y `piper_arm_controller.py` (subconjuntos de joints) | `robot_state_publisher` | Jetson |
| `/pan_tilt/joint_states` | JointState | `pan_tilt_node` (legado) | — | Jetson |
| `/pan_tilt/command`, `/pan_tilt/target_degrees` | JointState, Vector3 | HRI / tracking | `pan_tilt_node` | Jetson |
| `/arm/joint_cmd`, `/arm/gripper_cmd`, `/arm/enable` | JointState, Float32, Bool | `manipulacion_autonoma.py`, teach recorder | `piper_arm_controller.py` | Jetson |
| `/arm/current_pose`, `/arm/status` | PoseStamped, String | `piper_arm_controller.py` | manipulación | Jetson |
| `/arm/home`, `/arm/pre_grasp`, `/arm/grasp_pringles`, `/arm/release` | srv Trigger | `piper_arm_controller.py` | manipulación | Jetson |
| `/arm/set_enable` | srv SetBool | `piper_arm_controller.py` | `piper_teach_recorder.py` | Jetson |
| `/arm/trigger_grasp` | srv Trigger | `pringles_grasping_node` | — | NUC |
| `/arm_camera/image_raw` | Image | `stream_orbbec.py` | manipulación, calibración ArUco | Jetson→NUC |
| `/arm_camera/aruco_markers`, `/arm_camera/object_markers` | MarkerArray | calibración / manipulación | RViz | NUC |
| `/voice/transcription`, `/voice/status` | String | `whisper_node` | HRI | Jetson→NUC |
| `/voice/mute`, `/voice/initial_prompt` | Bool, String | HRI | `whisper_node` | NUC→Jetson |
| `/voice/listen_trigger` | srv Trigger | `whisper_node` | HRI | Jetson |
| `/hri/say_something`, `/hri/detected_persons`, `/hri/ui_display`, `/robot/emotion` | String/Image | HRI | cara animada / TTS | NUC |
| `/hri/introduce_person`, `/hri/play_beep`, `/hri/clear_memory` | srv Trigger | HRI receptionist | — | NUC |

Observaciones: todas las interfaces de actuadores de la Jetson (`/arm/*`, `/pan_tilt/*`) cruzan el enlace NUC↔Jetson, hoy caído. El sistema mezcla Humble (Jetson) y Jazzy (NUC). Pendiente: medir con `ros2 topic list -t`/`hz` en un dominio de prueba cuando el sistema esté en marcha y autorizado.
