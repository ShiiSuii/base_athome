# Base@Home (base_athome)

Robot de servicio doméstico de **UruBots** para **RoboCup@Home – Open Platform League**.
Manipulación móvil e interacción humano-robot sobre **ROS 2 Jazzy**.

- Base diferencial con dos motores hub de hoverboard (odometría Hall) y LiDAR Hokuyo UST-05LX.
- Columna con elevador (NEMA 23 + ESP32-S3), brazo **AgileX Piper 6-DOF** con pinza y cámara de muñeca Orbbec DaBai DC1.
- Cabeza pan-tilt (2× Dynamixel XL430) con **Intel RealSense D435**, micrófono **RØDE VideoMic GO II**, pantalla facial y voz.

## Arquitectura

```
 NUC (Ubuntu 24.04 · ROS 2 Jazzy · 10.42.0.1)          Jetson Orin Nano (Ubuntu 24.04 · JetPack 7.2.1 · Jazzy · 10.42.0.12)
 ├─ hoverboard_driver  ── UART ── base hoverboard        ├─ piper_arm_controller ── CAN (can1) ── Piper
 ├─ urg_node           ── Ethernet ── Hokuyo             ├─ pan_tilt_node ── U2D2 ── Dynamixel
 ├─ Nav2 + AMCL (navegación, protegida)                  ├─ whisper_node (ASR) ── USB ── RØDE
 ├─ realsense2_camera  ── USB 3 ── D435                  ├─ stream_orbbec ── USB ── DaBai (muñeca)
 ├─ HRI / receptionist / misiones / manipulación         └─ yolo_trt_node (GPU TensorRT)
 └─ cara animada, TTS, pantalla del invitado
            └──────── Ethernet 1 Gb/s · CycloneDDS unicast · ROS_DOMAIN_ID=10 ────────┘
```

Detalle completo (hardware, interfaces, TF, navegación, calibraciones): **[`docs/ROBOT.md`](docs/ROBOT.md)**.

## Preparar el entorno (NUC)

```bash
sudo apt install -y ros-jazzy-desktop ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-realsense2-camera ros-jazzy-urg-node \
  ros-jazzy-rmw-cyclonedds-cpp ros-jazzy-xacro liburdfdom-tools python3-colcon-common-extensions
# Percepción / HRI (venv con paquetes del sistema). No exportar modelos con este venv.
python3 -m venv ~/ros2_vision_venv --system-site-packages
~/ros2_vision_venv/bin/pip install ultralytics opencv-python pyyaml
# Workspace
cd ~/base_athome_ws && colcon build --symlink-install && source install/setup.bash
# Alias del robot (NUC). Algunos alias históricos (`receptionist`, `navegacion`, `mapeo2d`, `stepper`) están en `~/.bashrc`, fuera del repo
echo 'source ~/base_athome_ws/scripts/aliases/nuc_aliases.sh' >> ~/.bashrc   # luego: ayuda
```

Variables: `ROS_DOMAIN_ID=10`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=file://$HOME/cyclonedds.xml`
(peers `10.42.0.1`/`10.42.0.12`, multicast desactivado).

**Jetson:** el código fuente vive en `jetson_head/` (no se compila en la NUC). Se despliega con
`jetson_head/scripts/deploy_to_jetson.sh` (dry-run por defecto; alias `deploy_jetson_dry` / `deploy_jetson`)
y corre como `jetson-head.service` (alias `head_status`, `head_logs`, `head_restart`). Instalación
desde cero: `jetson_head/scripts/bootstrap_jazzy.sh` (ver `docs/ROBOT.md` §11).

## Usar el robot

> Las aplicaciones que mueven actuadores requieren supervisión presencial y E-stop (`AGENTS.md` §3).

| Qué | Comando / alias |
|---|---|
| Base + LiDAR + Nav2 (mapa por defecto `~/mapa_base_athome.yaml`) | `ros2 launch base_athome_navigation nav2_bringup.launch.py` |
| Mapeo 2D / 3D | `ros2 launch base_athome_mapping slam_toolbox.launch.py` · `rtabmap_realsense.launch.py`; guardar con `scripts/guardar_mapa_2d.sh` |
| Teleoperación | `ros2 launch base_athome_navigation teleop_ps4.launch.py` |
| RealSense (NUC) | `ros2 launch base_athome_bringup realsense_nuc.launch.py` |
| Receptionist (HRI Challenge) | `ros2 launch base_athome_perception hri_receptionist.launch.py` (en la NUC también el alias `receptionist` de `~/.bashrc`) |
| Commander por voz | alias `commander` en la NUC |
| Demo en pantalla de sacar la basura (simulación) | alias `sim_basura` en la NUC |
| Enseñar la pose de agarre de piso | alias `ensenar_pregrasp` en la NUC |
| Calibrar y probar el pan-tilt | alias `calibrar_pantilt` en la NUC (`python3 src/base_athome_perception/scripts/pan_tilt_calibrate.py`) |
| Secuenciador de misiones | `mision`; pedir: `mision_pedir <orden>` |
| Manipulación (enseñar poses, agarrar con MoveIt) | `manipulacion` (simple) · `manipulacion_avanzada` |
| Señal de inicio por puerta abierta | `puerta` (`base_athome_skills`) |
| Elevador | `python3 scripts/stepper_controller.py` (`home`, `top`, `bottom`, `rpm <v>`); alias `stepper` en `~/.bashrc` |
| Estado Jetson / brazo / pan-tilt / voz | `head_status` · `brazo_estado` · `pantilt_conectado` · `voz` |
| Micrófono (probar, diagnosticar) | `escuchar --n 5` · `prueba_microfono` · `microfono_logs` — manual en [`docs/MICROFONO_ASR.md`](docs/MICROFONO_ASR.md) |
| Pantalla del invitado | `pantalla_invitado` → `http://127.0.0.1:8090` |
| Red NUC↔Jetson | `check_red` (`scripts/net/check_nuc_jetson_link.sh`) |

## Herramientas

| Herramienta | Uso |
|---|---|
| Editor URDF/Xacro (solo preview, `127.0.0.1:8765`) | `urdf_editor` — ver [`tools/urdf_editor/README.md`](tools/urdf_editor/README.md) |
| Parser GPSR (texto → plan JSON) | `gpsr "bring me a coke from the kitchen"` — ver [`tools/gpsr_parser/README.md`](tools/gpsr_parser/README.md) |
| Editor de mapa semántico (zonas, waypoints, objetos) | `src/base_athome_navigation/base_athome_navigation/map_zone_editor.py` |
| Firmware base y elevador | [`firmware/README.md`](firmware/README.md) |

## Simulación (sin robot)

Mismo código que en el robot; el simulador reemplaza Jetson, cámaras y Nav2 con las mismas interfaces,
en `ROS_DOMAIN_ID=42`.

```bash
# Terminal 1
simulacion                    # liviana, sin física   |   sim_gazebo   # Gazebo Harmonic
simulacion auto_guest:=true   # el invitado responde solo
# Terminal 2: la app real contra la simulación
sim_env                       # ROS_DOMAIN_ID=42 + ATHOME_SIM=1 (solo esta terminal)
receptionist --sim-nav --max-guests 1      # o: manipulacion | mision
sim_salir                     # volver al dominio del robot (10)
# Terminal 3: hacer de invitado
sim_invitado door             # none = se va, next = otro invitado
sim_decir "My name is Robin and my favorite drink is iced tea"
```

Qué simula y qué no: `docs/ROBOT.md` §9.

## Pruebas

```bash
export ROS_DOMAIN_ID=42
python3 -m pytest -q src/base_athome_perception/test src/base_athome_sim/test \
  src/base_athome_skills/test src/base_athome_navigation/test jetson_head/scripts/tests
```

Resto de suites (ASR, pan-tilt, GPSR, editor URDF) y resultados: `docs/ROBOT.md` §10.

## Documentación

| Documento | Para qué |
|---|---|
| [`AGENTS.md`](AGENTS.md) | reglas de trabajo, seguridad y permisos (humanos y agentes) |
| [`docs/ROBOT.md`](docs/ROBOT.md) | conocimiento técnico del robot |
| [`docs/ROADMAP.md`](docs/ROADMAP.md) | reglamento 2026.2, prioridades y tareas |
| [`docs/collab/STATUS.md`](docs/collab/STATUS.md) | estado actual del proyecto |
| [`docs/MICROFONO_ASR.md`](docs/MICROFONO_ASR.md) | manual de la cadena de voz |

## Colaborar

- Trabajar en una rama (`claude/…`, `antigravity/…`, `feature/…`), nunca directo en `main`; PR revisado.
- **La navegación (Nav2, AMCL, SLAM, driver y firmware de la base, mapas, odometría) no se modifica**
  sin autorización explícita; las mejoras se proponen en `docs/ROADMAP.md` §6.
- Solo ROS 2. Cada cambio con tests, rollback y la documentación actualizada en el mismo commit
  (sin crear Markdown nuevos por tarea: ver `AGENTS.md` §9).
- No subir credenciales, audios, rostros ni videos de personas.

Licencia: Apache 2.0.
