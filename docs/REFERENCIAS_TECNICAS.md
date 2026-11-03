# Contexto vigente v3 — lectura antes del documento siguiente

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Nota de uso:** Se agrega catálogo especializado de hoverboard ROS 2: `docs/BASE_HOVERBOARD_REFERENCIAS.md`. Evitar ejecutar ejemplos ROS 1 aunque se citen por contexto.

---

# Referencias técnicas verificables para agentes — Base@Home

> **Regla:** primero identificar edición, distro y API del repositorio local. Estas referencias sirven para estudiar patrones, no para copiar código ciegamente. No ejecutar ROS 1, no instalar software desde repos externos sin revisión, no utilizar datos de Google, no contratar servicios. Consultar licencias y compatibilidad; registrar fecha de revisión.

## A. Proyecto propio / reglas / orquestación

| Fuente | URL | Qué investigar |
|---|---|---|
| Repositorio base | https://github.com/ShiiSuii/base_athome | Fuente primaria de árbol, README, firmware, launch, URDF y scripts. Contrastar con checkout LOCAL |
| Rulebook 2026.2 | https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2 | Versión de competición de referencia, scoring y tareas |
| Changelog RuleBook | https://github.com/RoboCupAtHome/RuleBook/blob/master/CHANGELOG.md | Para contexto histórico 2026; **master puede contener 2027** |
| RoboCup@Home | https://athome.robocup.org/ | Liga, organización y novedades |
| RoboCup@Home comunidad | https://github.com/RoboCupAtHome/AtHomeCommunityWiki | Hardware, competencias, buenas prácticas; validar antigüedad |
| Orca worktrees | https://www.onorca.dev/docs/model/worktrees | Aislamiento de ramas, worktrees, conflictos |
| Orca orquestación | https://www.onorca.dev/docs/cli/orchestration | Run/tasks/dispatch/worker_done, gates, guía versión actual |
| Orca agentes y permisos | https://www.onorca.dev/docs/model/agents-sessions | Revisar por qué es riesgosa la autonomía predeterminada |
| Orca supported agents | https://www.onorca.dev/docs/agents/supported | Claude y Antigravity disponibles, revisar versión local |
| Orca skills | https://www.onorca.dev/docs/cli/skills | Cargar skill versionado y evitar comandos legacy |
| Claude en Orca | https://www.onorca.dev/docs/agents/claude-code | Integración de sesión/CLI |

## B. Repositorios de equipos RoboCup@Home avanzados (ROS 2)

**Analizar especialmente:** estructura por habilidades, máquina de estados, task planner, interfaces, tests, datasets, despliegue, soporte de percepción/manipulación, reinicios y documentación. No inferir que están actualizados o listos para reproducir en vuestro hardware.

| Equipo | Repositorio | Utilidad | Precaución |
|---|---|---|---|
| RoBorregos / FRIDA | https://github.com/RoBorregos/home2 | ROS 2 para RoboCup@Home OPL; visión, navegación, manipulación, HRI, Docker | Arquitectura distinta; comparar dependencias/hardware |
| RoBorregos documentación | https://roborregos.github.io/home2/ | API/estructura ROS 2 y ejemplos de integración | Verificar versión de código y bibliotecas |
| LAR@Home / CHARMIE | https://github.com/SparkRibeiro21/charmie_ws | Workspace ROS 2 de robot CHARMIE; HRI/percepción y tareas | No copiar definiciones de robot sin adaptación |
| WrightEagle.AI Manipulation | https://github.com/USTC-WrightEagle-AI/26-WrightEagle.AI-Manipulation | Pipeline ROS 2 detección, estimación de agarre y manipulación | Licencia, drivers y compatibilidad con Piper |
| Organización WrightEagle | https://github.com/USTC-WrightEagle-AI | Otros módulos y documentación del equipo | Comprobar calidad y vigencia |

**No proponer repos ROS 1 como base ejecutable** (por ejemplo ediciones antiguas de RoBorregos). Es posible analizarlos solo como historia conceptual, sin incorporar comandos ROS 1.

## C. ROS 2/Jazzy, robótica distribuida, calidad

| Tema | URL | Uso |
|---|---|---|
| ROS 2 Jazzy docs | https://docs.ros.org/en/jazzy/ | Arquitectura, nodos, launch, QoS, lifecycle, actions |
| Instalación Jazzy Ubuntu | https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html | Ubuntu Noble arm64/amd64; utilizar en fase aprobada |
| ROS 2 source workspace | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Colcon-Tutorial.html | `colcon`, ament, aislación de builds |
| Robot state publisher | https://github.com/ros/robot_state_publisher | Fuente y publicación TF a partir URDF/joint states |
| ros2_control | https://control.ros.org/jazzy/index.html | Separar estado medido, comandos y hardware interfaces (estudiar, no desplegar) |
| TF2 | https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html | Arboles TF, timestamps, optical frames |
| REP 103 | https://www.ros.org/reps/rep-0103.html | Convenciones ejes/unidades, frames de cámaras |
| REP 105 | https://www.ros.org/reps/rep-0105.html | Convenciones map/odom/base_link |
| Nav2 documentación | https://docs.nav2.org/ | **Solo lectura**: contratos/actions/BT, NO tuning |
| ROS2 Jazzy REP 2000 | https://www.ros.org/reps/rep-2000.html | Plataformas/distro y arquitectura |
| GoogleTest ROS2 | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Cpp/Basic.html | Pruebas unitarias C++ si corresponde |
| pytest ROS2 | https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Python.html | Unit tests Python |

## D. AgileX Piper, CAN, MoveIt2 y hand-eye

| Tema | URL | Uso |
|---|---|---|
| AgileX Piper SDK | https://github.com/agilexrobotics/piper_sdk | Protocolos, CAN, feedback y restricciones. **NO ejecutar comandos mientras apagado** |
| AgileX driver ROS 2 | https://github.com/agilexrobotics/agx_arm_ros | Driver ROS2 actual, revisar rama/versión antes de adaptar |
| AgileX modelos URDF | https://github.com/agilexrobotics/agx_arm_urdf | Geometrías, joints y límites Piper |
| AgileX simulación | https://github.com/agilexrobotics/agx_arm_sim | Ejemplos y simulación ROS 2/MoveIt; evitar launch real |
| MoveIt 2 | https://moveit.picknik.ai/main/index.html | Planning scene, colisiones, grasping, simulación |
| MoveIt Calibration | https://github.com/moveit/moveit_calibration | Principios hand-eye/transformaciones; revisar vigencia ROS2 |
| OpenCV ArUco | https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html | Detección marcadores y poses con intrínsecos válidos |
| OpenCV calibrateHandEye | https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html | Método eye-in-hand, convenciones de entrada/salida |
| GitHub easy_handeye2 | https://github.com/IFL-CAMP/easy_handeye2 | Ejemplo ROS 2 calibración mano-ojo (examinar compatibilidad) |

## E. Sensores, audio, cámaras y percepción

| Fuente | URL | Uso |
|---|---|---|
| Intel RealSense ROS | https://github.com/IntelRealSense/realsense-ros | Tópicos RGB-D, align, device startup/ownership, versiones ROS2 |
| Orbbec SDK | https://github.com/orbbec/OrbbecSDK | API, calibración, drivers cámara muñeca; confirmar modelo |
| Orbbec ROS2 | https://github.com/orbbec/OrbbecSDK_ROS2 | Integración cámaras; comprobar si DaBai DC1 está soportada |
| ROBOTIS Dynamixel SDK | https://github.com/ROBOTIS-GIT/DynamixelSDK | Feedback posicional y torque, seguridad y protocolo 2.0 |
| ROBOTIS Dynamixel hardware | https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/ | Unidades, límites eléctricos y modelo XL430 |
| ROS2 audio | https://github.com/ros-drivers/audio_common | Inspira separación captura/publicación; confirmar mantenimiento |
| OpenCV calib3d | https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html | Intrínsecos, ArUco, PnP, hand-eye |
| ultralytics | https://docs.ultralytics.com/ | Detectar personas/objetos; revisar licencia antes de integrar |
| Intel RealSense samples | https://github.com/IntelRealSense/librealsense | RGB-D alignment, hardware dependencies, ejemplos |

## F. URDF/Xacro, preview web, geometría y tooling

| Fuente | URL | Uso |
|---|---|---|
| URDF loader Three.js | https://github.com/gkjohnson/urdf-loaders | Renderizar árbol URDF y meshes en navegador |
| URDF editor | https://github.com/gkjohnson/urdf-editor | Referencia de UX y edición; comparar licencia y limitaciones |
| xacro ROS | https://github.com/ros/xacro | Expansión Xacro a URDF, expresiones/macros |
| Three.js | https://threejs.org/docs/ | OrbitControls, loaders, geometría y render |
| URDF docs | https://wiki.ros.org/urdf/XML | Formato XML (solo lectura, no ejecutar código ROS1 de la wiki) |
| Nav2 URDF integration | https://docs.nav2.org/setup_guides/urdf/setup_urdf.html | Solo para comprender frames: no cambiar navegación |
| FastAPI | https://fastapi.tiangolo.com/ | API local opcional, validación de esquemas; sin instalación global |
| Pydantic | https://docs.pydantic.dev/latest/ | Validación tipada para parámetros del editor |

## G. NVIDIA Jetson / Ubuntu 24.04 / ROS 2 Jazzy

- **JetPack 7.2.1 oficial (Jetson Linux 39.2.1)**: https://developer.nvidia.com/embedded/jetpack/downloads — NVIDIA lista Jetson Orin Family, Ubuntu 24.04, CUDA 13.2.2 y kernel 6.8. Las versiones pueden cambiar; fijar la seleccionada en el plan.
- **JetPack 7.2 archive (familia Orin)**: https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2
- **Orin Nano Devkit User Guide**: https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/quick_start.html
- **Jetson Linux Developer Guide**: https://docs.nvidia.com/jetson/archives/r39.2/DeveloperGuide/index.html (comprobar archivo/versionado si cambia)
- **SDK Manager**: https://developer.nvidia.com/sdk-manager (fase humana, no ejecutar agente)
- **ROS 2 Jazzy Ubuntu**: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- **NVIDIA Developer Forums**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/70 (consulta pública, verificar respuestas oficiales)

Advertencias: no `do-release-upgrade` directo; respaldo y estrategia de rollback; no flashear sin revisar carrier, UEFI/QSPI, storage, soportes y alimentación. JetPack/OS/ROS, drivers de Orbbec, CAN, Dynamixel, TTS/ASR y Python requieren matriz de compatibilidad.

## H. Documentación de calidad / trazabilidad

- GitHub PR reviews: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/reviewing-changes-in-pull-requests
- Git worktrees: https://git-scm.com/docs/git-worktree
- Conventional Commits (referencia opcional): https://www.conventionalcommits.org/en/v1.0.0/
- Mermaid diagrams: https://mermaid.js.org/intro/
- pytest: https://docs.pytest.org/en/stable/
- Ruff: https://docs.astral.sh/ruff/
- mypy: https://mypy.readthedocs.io/en/stable/

## Método obligatorio de evaluación de repos externos

Para cada candidato a reutilización documentar: `URL`, `commit/tag`, `licencia`, `fecha de revisión`, `distro ROS`, `hardware soportado`, `dependencias`, `qué parte copiar/adaptar`, `conflictos`, `costos/licencias`, `test offline`, `riesgo de mantenimiento` y `decisión ADOPTAR/ADAPTAR/NO USAR`. La existencia de un repo no prueba que sea apropiado para NUC/Jetson/Orca.

## Red Ethernet directa y coordinación entre computadoras

**Estado confirmado por usuario:** NUC y Jetson encendidas, conectadas físicamente por cable Ethernet; direcciones/IP/documentación histórica deben verificarse en vivo. Para los comandos seguros y pruebas ROS2 sin actuadores, leer `docs/RED_ETHERNET.md`. Documentación pública: https://cyclonedds.io/docs/cyclonedds/latest/config/index.html ; https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Domain-ID.html ; https://docs.ros.org/en/jazzy/How-To-Guides/Working-with-multiple-RMW-implementations.html .

**Nota de Orca:** https://www.onorca.dev/docs/agents/supported enumera explícitamente Antigravity como agente soportado y advierte que los agentes pueden iniciar con permisos amplios; revisar `Settings → Agents → Agent Permissions` antes de trabajos sobre dos hosts.
