# Repositorios abiertos reutilizables (equipos @Home y drivers)

Relevado 2026-10-30 por Claude Code (API pública de GitHub + `git ls-remote` + README). **Licencia "ninguna" = se puede leer y estudiar, pero NO copiar código** sin permiso del autor (copyright por defecto). Verificar licencia y versión antes de reutilizar (ver `AGENTS.md` §8).

## A. Drivers / SDK de nuestro hardware (prioridad máxima)

| Repo | Licencia | ROS 2 | Uso para Base@Home |
|---|---|---|---|
| [agilexrobotics/agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) rama `ros2` | ver repo | **Humble y Jazzy "Pass"**, incluye `agx_arm_moveit` | Driver oficial ROS 2 + MoveIt para Piper. Candidato a reemplazar `piper_arm_controller.py` tras migrar la Jetson a Jazzy |
| [agilexrobotics/piper_sdk](https://github.com/agilexrobotics/piper_sdk) | MIT | (Python) | SDK que usamos hoy (`C_PiperInterface_V2`) |
| [agilexrobotics/piper_ros](https://github.com/agilexrobotics/piper_ros) | MIT | ramas `humble`, `humble_beta1` | Alternativa vieja; preferir agx_arm_ros |
| [agilexrobotics/pyAgxArm](https://github.com/agilexrobotics/pyAgxArm) | LGPL-3.0 | (Python) | Nuevo driver Python para Piper/Nero |
| [orbbec/OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2) ramas `main`, `v2-main`, `main-legacy` | ver repo | sí | Driver para DaBai DC1; hoy usamos `stream_orbbec.py` propio. Verificar soporte DC1 y Jazzy en `v2-main` |
| [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) `ros2-master` | Apache-2.0 | sí | D435 (ya en uso) |
| [ROBOTIS-GIT/DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) ramas `humble`, `jazzy` | Apache-2.0 | sí | Ya usado por pan-tilt |
| [ROBOTIS-GIT/dynamixel_hardware_interface](https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface) `jazzy` | Apache-2.0 | sí | Opción ros2_control para el pan-tilt |

## B. Stacks completos de equipos (estudio de arquitectura)

| Repo | Licencia | Distro | Qué aprovechar |
|---|---|---|---|
| [RoBorregos/home2](https://github.com/RoBorregos/home2) (FRIDA) | **ninguna** | Docker (CPU/CUDA/**L4T**) | Arquitectura por áreas en contenedores (`run.sh vision|hri|navigation|manipulation`), soporte Jetson L4T, tareas `--gpsr`, `--restaurant`; configuración CycloneDDS. Solo estudio |
| [RoBorregos/frida-cortex](https://github.com/RoBorregos/frida-cortex) | **MIT** | Python | Intérprete de comandos GPSR → JSON de acciones (paper Springer LNAI 2025). **Reutilizable para GPSR** |
| [RoBorregos/home-hri](https://github.com/RoBorregos/home-hri) | GPL-3.0 | — | HRI R&D (copyleft: cuidado al mezclar) |
| [SparkRibeiro21/charmie_ws](https://github.com/SparkRibeiro21/charmie_ws) (LAR@Home) | **MIT** | Humble | Workspace completo de un finalista: xArm6, RealSense, **Orbbec**, Livox; tareas de competencia. **Reutilizable** |
| [fbotathome/fbot_hri](https://github.com/fbotathome/fbot_hri) | ninguna | Humble | TTS/STT, cuello + emociones, pantalla (nuestro `whisper_node` dice estar inspirado en fbot_hri) |
| [fbotathome/fbot_vision](https://github.com/fbotathome/fbot_vision) | ninguna | Humble | Reconocimiento facial, tracking de personas |
| [fbotathome/fbot_manipulator](https://github.com/fbotathome/fbot_manipulator) | ninguna | Humble | Primitivas MoveIt + **MoveIt Task Constructor** (pick, place, pour) como action server: excelente modelo de interfaz |
| [tue-robotics/tue_robocup](https://github.com/tue-robotics/tue_robocup) | ninguna | ROS 1 histórico | Implementaciones de pruebas (máquinas de estados). Solo lectura |
| [tue-robotics/image_recognition](https://github.com/tue-robotics/image_recognition) | MIT | — | Paquetes de reconocimiento de imágenes |
| [USTC-WrightEagle-AI](https://github.com/USTC-WrightEagle-AI) (Manipulation, person_tracker, Speech, MHRC-planning, YOLO-Perception) | ninguna | Foxy/Humble | Grounded-SAM-2 + GraspNet; ASR/TTS sherpa-onnx offline; planificación con LLM. Solo estudio |
| [b-it-bots/mas_domestic_robotics](https://github.com/b-it-bots/mas_domestic_robotics) | GPL-3.0 | ROS 1 | Arquitectura de habilidades independiente del robot (referencia conceptual) |
| [uchile-robotics](https://github.com/uchile-robotics) | ninguna | ROS 1 | Referencia regional |
| [demulab/happymini_ros2](https://github.com/demulab/happymini_ros2) | ninguna | Humble | Robot @Home de bajo costo para principiantes |

## C. Componentes puntuales directamente útiles para 2026.2

| Necesidad (reglamento) | Repo | Licencia | Nota |
|---|---|---|---|
| Timbre HRI (2×30) | [socrob/doorbell_detector](https://github.com/socrob/doorbell_detector) | ninguna | ROS 2, YAMNet, `/doorbell/listen` → `/doorbell/event`. Pedir permiso o reimplementar con YAMNet (Apache-2.0) |
| GPSR: generador oficial de comandos | [RoboCupAtHome/CommandGenerator](https://github.com/RoboCupAtHome/CommandGenerator) | ver repo | `athome-generator -d DATA_DIR`; generar dataset de prueba offline para nuestro parser |
| GPSR: parser | frida-cortex (MIT), [socrob/rule_based_gpsr](https://github.com/socrob/rule_based_gpsr) (MIT, ROS 1) | MIT | Parser basado en reglas como fallback sin LLM |
| Agarre | [socrob/grasp_detection_ros2](https://github.com/socrob/grasp_detection_ros2) | ninguna | Wrapper ROS 2 de GPD |
| HRI estándar | [ros4hri](https://github.com/ros4hri) (`hri_face_detect`, `hri_body_detect`, `hri_msgs`, `hri_person_manager`) | Apache-2.0 | **Reutilizable**: estándar ROS4HRI para personas/caras/cuerpos |
| Simulación | [RoboCupatHomeSim](https://github.com/RoboCupatHomeSim) | MIT (msgs) | Mensajes de competencia |
| Educación | [robocupathomeedu/rc-home-edu-learn-ros2](https://github.com/robocupathomeedu/rc-home-edu-learn-ros2) | MIT | Ejemplos ROS 2 básicos (2021) |
| Wiki comunitaria | [AtHomeCommunityWiki](https://github.com/RoboCupAtHome/AtHomeCommunityWiki/wiki) | — | Hardware/software usado por equipos |

## D. Recomendación

1. **Piper**: migrar a `agx_arm_ros` (ros2, Jazzy) + MoveIt después de HOME-11; mantener nuestro wrapper `/arm/*` como capa de compatibilidad para no romper `manipulacion_autonoma.py`.
2. **GPSR**: empezar con `frida-cortex` (MIT) o un parser propio + dataset de `CommandGenerator`.
3. **HRI**: adoptar mensajes ROS4HRI (Apache-2.0) para personas; reimplementar el timbre con YAMNet.
4. **Manipulación de alto nivel**: copiar el *diseño* de interfaz de `fbot_manipulator` (action `ManipulationTask` con PICK/PLACE/POUR vía MTC), no su código (sin licencia).
5. **Docker/L4T** como en `home2` para aislar dependencias en la Jetson tras la migración.
