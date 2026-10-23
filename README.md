# Base@Home (base_athome)

Plataforma robotica de servicio para la competencia RoboCup @Home (Open Platform League). El sistema integra una base diferencial de alta traccion basada en motores hub de hoverboard con control de odometria cerrada, LiDAR 2D, sistema de elevacion lineal de torso (NEMA 23), cabezal con camara RGB-D sobre pan-tilt de dos ejes, y navegacion autonoma Nav2 optimizada para superar la friccion estatica en maniobras de giro sobre el eje.

---

## Tabla de Contenidos

1. [Arquitectura del Sistema y Computacion Distribuida](#1-arquitectura-del-sistema-y-computacion-distribuida)
2. [Arbol de Transformaciones (TF Tree) y Cinemática](#2-arbol-de-transformaciones-tf-tree-y-cinematica)
3. [Mapeo 2D y 3D](#3-mapeo-2d-y-3d)
4. [Navegacion Autonoma (Nav2)](#4-navegacion-autonoma-nav2)
5. [Sistema de Elevacion de Torso](#5-sistema-de-elevacion-de-torso)
6. [Referencia de Comandos y Alias de Terminal](#6-referencia-de-comandos-y-alias-de-terminal)
7. [Instalacion y Compilacion](#7-instalacion-y-compilacion)

---

## 1. Arquitectura del Sistema y Computacion Distribuida

El robot opera bajo una arquitectura heterogenea distribuida conectada mediante red Gigabit Ethernet dedicada con middleware ROS 2 CycloneDDS:

```
+-----------------------------------------------------------------------------------+
|                        INTEL NUC (10.42.0.1 - ROS 2 Jazzy)                        |
|                                                                                   |
|  +--------------------+   UART (/dev/ttyUSB0)   +------------------------------+  |
|  |  hoverboard_base   |<=======================>| Dual Hub Motors + Hall Odom  |  |
|  | (hoverboard_driver)|                         +------------------------------+  |
|  +---------+----------+                                                           |
|            | /odom, /tf                                                           |
|            v                                                                      |
|  +--------------------+   Ethernet (192.168.0.10)+-----------------------------+  |
|  |      urg_node      |<========================>|   LiDAR Hokuyo UST-05LX     |  |
|  +---------+----------+                          +-----------------------------+  |
|            | /scan                                                                |
|            +-----------------------+-----------------------+                      |
|            |                       |                       |                      |
|            v                       v                       v                      |
|  +--------------------+  +--------------------+  +--------------------+           |
|  |    slam_toolbox    |  |     Nav2 Stack     |  |   teleop_ps4_joy   |           |
|  |  (Mapeo 2D Online) |  | (AMCL, Smac, RPP)  |  | (Manual Deadman)   |           |
|  +---------+----------+  +--------------------+  +--------------------+           |
|            | /map, /tf                                                            |
|            v                                                                      |
|  +--------------------+                                                           |
|  |   map_assembler    |                                                           |
|  |  (Nube 3D Global)  |                                                           |
|  +---------^----------+                                                           |
|            | /rtabmap/mapData                                                     |
|  +---------+----------+                                                           |
|  |    rtabmap_slam    |                                                           |
|  |  (Grafo SLAM 3D)   |                                                           |
|  +---------^----------+                                                           |
|            |                                                                      |
|            | CycloneDDS (ROS_DOMAIN_ID=10, Unicast Peered)                        |
+------------|----------------------------------------------------------------------+
             |
             | Gigabit Ethernet (10.42.0.0/24)
             |
+------------|----------------------------------------------------------------------+
|            v                                                                      |
|  +--------------------+   USB 3.0 (/dev/video*)  +-----------------------------+  |
|  | realsense2_camera  |<========================>| Intel RealSense D435 (RGB-D)|  |
|  +---------+----------+                          +-----------------------------+  |
|            | /camera/color/image_raw, /camera/aligned_depth_to_color/image_raw    |
|            |                                                                      |
|  +---------+----------+   UART (/dev/ttyU2D2)    +-----------------------------+  |
|  |   pan_tilt_node    |<========================>| ROBOTIS Dynamixel (Pan/Tilt)|  |
|  +--------------------+                          +-----------------------------+  |
|            | /joint_states                                                        |
|            |                                                                      |
|  +---------+----------+   USB Audio              +-----------------------------+  |
|  | whisper_asr_node   |<========================>| RODE VideoMic GO II         |  |
|  +--------------------+                          +-----------------------------+  |
|                                                                                   |
|                 NVIDIA JETSON ORIN NANO (10.42.0.12 - ROS 2 Humble)               |
+-----------------------------------------------------------------------------------+
```

### Configuracion DDS (CycloneDDS)
Ambos equipos ejecutan CycloneDDS con descubrimiento explicito unicast en `~/cyclonedds.xml`:
- Direccion NUC: `10.42.0.1`
- Direccion Jetson: `10.42.0.12`
- Identificador de Dominio: `ROS_DOMAIN_ID=10`

---

## 2. Arbol de Transformaciones (TF Tree) y Cinemática

El modelo cinematico y geometrico del robot esta definido en `src/hoverboard_base/urdf/robot.urdf.xacro`:

```
map
 └── odom (Publicado por slam_toolbox en mapeo, o nav2_amcl en navegacion)
      └── base_footprint (Publicado por hoverboard_driver a 50 Hz)
           └── base_link
                ├── left_wheel (Rotacion eje Y)
                ├── right_wheel (Rotacion eje Y)
                ├── rear_caster (Rueda loca de apoyo)
                ├── laser (Hokuyo UST-05LX, x=0.20m, z=0.18m)
                └── torso_column (Columna fija central de 1.0 m, x=0.0m, y=0.0m)
                     └── torso_lift_link (Plataforma superior del elevador, z=1.0m)
                          └── pan_tilt_base_link
                               └── pan_link (Motor Dynamixel 1 - Guiñada / Yaw, eje Z)
                                    └── tilt_link (Motor Dynamixel 2 - Cabeceo / Pitch, eje Y)
                                         └── camera_link (Soporte RealSense D435)
                                              └── camera_color_optical_frame
```

### Parametros Fisicos Calibrados
- **Ancho de via (Track Width):** `0.5620 m`
- **Radio efectivo de rueda (Wheel Radius):** `0.0853 m`
- **Ubicacion del torso vertical:** `x=0.0 m, y=0.0 m` (centro exacto del chasis).
- **Altura de columna:** `1.0 m`.
- **Servomotores Dynamixel XL430:**
  - ID 1 (Pan): Centro calibrado = `2059` ticks.
  - ID 2 (Tilt): Centro calibrado = `3932` ticks.
  - Tasa de actualizacion de joints: `20 Hz`.

---

## 3. Mapeo 2D y 3D

El sistema ofrece dos modalidades de mapeo independientes segun las necesidades de la prueba:

### Modo 1: Mapeo Convencional 2D
Lanza el stack minimo para generar planos de navegacion rapida usando unicamente odometria y LiDAR:
- Comando de inicio: `mapeo2d` (o `slam`)
- Componentes: Driver Hoverboard + URDF + Hokuyo LiDAR + Teleop PS4 + SLAM Toolbox + RViz2 2D.
- Guardado de mapa:
  ```bash
  guardarmapa [nombre_opcional]
  ```
  Genera los archivos `.yaml` y `.pgm` correspondientes en `~/` con respaldo historico fechado en `~/mapas/`.

### Modo 2: Mapeo 3D Multimodal (SLAM Toolbox + RTAB-Map)
Combina la estabilidad del mapa de ocupacion 2D por laser con la reconstruccion tridimensional densa de la camara RealSense D435 montada en el cuello:
- Preparacion previa del cabezal (en Jetson):
  ```bash
  head_start
  ```
- Comando de inicio (en NUC): `mapeo3d`
- Componentes: Driver Base + LiDAR + Teleop + SLAM Toolbox + `point_cloud_xyzrgb` + `rtabmap` + `map_assembler` + RViz2 3D.
- Caracteristicas tecnicas:
  - Nube en tiempo real (`/camera/cloud_points`): cono instantaneo de la camara para monitoreo visual a 15 FPS.
  - Mapa 3D persistente (`/rtabmap/cloud_map`): ensamblado global de keyframes por `map_assembler`, voxelizado a 0.03 m.
  - Persistencia continua en disco: base de datos SQLite en `~/.ros/rtabmap.db`.
- Guardado y Reconstruccion 3D:
  ```bash
  guardarmapa3d [nombre_opcional]
  ```
  Ejecuta automaticamente `rtabmap-export` para generar:
  1. Copia de seguridad del archivo `.db`.
  2. Nube de puntos 3D completa voxelizada en formato `.ply`.
  3. Malla tridimensional texturada reconstruida mediante algoritmo Poisson en formato `.obj` con material `.mtl` y texturas UV.

---

## 4. Navegacion Autonoma (Nav2)

La navegacion autonoma esta configurada en `base_athome_navigation/config/nav2_params.yaml` con arquitectura disenada para bases diferenciales pesadas con neumaticos de goma solida:

### Control de Movimiento y Mitigacion de Friccion Estatica
- **RotationShimController:** Envoltorio jerarquico sobre el controlador de seguimiento. Si el robot se encuentra desviado respecto a la ruta mas de `0.785 rad` (45 grados), el controlador desacopla el avance y ejecuta una rotacion pura sobre el eje a velocidad angular controlada (`1.0 rad/s`).
- **Alineacion Final de Orientacion (`rotate_to_goal_heading: true`):** Al alcanzar la tolerancia cartesiana del objetivo (`xy_goal_tolerance: 0.18 m`), el RotationShimController toma el control para alinear la orientacion final requerida con precision de `yaw_goal_tolerance: 0.10 rad` (~5.7 grados).
- **RegulatedPurePursuitController (RPP):** Seguimiento de trayectoria principal:
  - `min_lookahead_dist: 0.58 m` (superior al ancho de via para evitar sobre-correcciones y oscilaciones laterales).
  - `desired_linear_vel: 0.38 m/s`.
  - `min_approach_linear_velocity: 0.16 m/s`.
- **Filtro de Despegue en Driver (`hoverboard_driver`):** Piso minimo de torque (`min_speed_raw: 38`, `min_steer_raw: 45`) para romper la inercia estatica (stiction) sin producir sacudidas ni disparos bruscos de corriente.

### Ejecucion de Navegacion Autonoma
Un unico comando levanta todos los componentes requeridos:
```bash
navegacion
```
Este comando compila e inicializa:
1. Driver de hoverboard con filtro de par y odometria a 50 Hz.
2. Robot State Publisher con URDF completo.
3. LiDAR Hokuyo UST-05LX publicando `/scan` a 40 Hz.
4. Teleoperacion manual por joystick PS4 (habilitada como deadman switch en R1/L1 para intervencion de emergencia).
5. Servidores Nav2: Map Server, AMCL, SmacPlanner2D, Controller Server, Behavior Server, BT Navigator y Lifecycle Manager.
6. Interfaz grafica RViz2 con panel de navegacion y herramientas de objetivo.

---

## 5. Sistema de Elevacion de Torso

El mecanismo de subida y bajada del torso esta accionado por un motor paso a paso NEMA 23 de alto torque conectado a un tornillo sin fin / guia lineal, controlado mediante microcontrolador (ESP32-S3 / Arduino Nano) a traves de puerto serie a 115200 baudios:

- Driver de control: `scripts/stepper_controller.py`
- Acceso rapido:
  ```bash
  stepper
  ```
- Comandos soportados en consola interactiva:
  - `calibrate` / `home`: Rutina automatica de busqueda de limites fisicos y establecimiento de referencia.
  - `top`: Desplazamiento a tope superior.
  - `bottom`: Desplazamiento a fondo inferior.
  - `speed <us>`: Ajuste del periodo de pulsos por micropaso.
  - `rpm <val>`: Ajuste directo de velocidad en RPM.
  - `turbo` / `fast` / `normal` / `slow`: Perfiles de velocidad preconfigurados.
  - `keys`: Modo interactivo por flechas de teclado.

---

## 6. Referencia de Comandos y Alias de Terminal

Todos los comandos se encuentran configurados en el archivo `~/.bashrc` de la maquina NUC:

| Alias / Comando | Descripcion Tecnica |
| :--- | :--- |
| `mapeo2d` (o `slam`) | Inicia mapeo 2D completo (Base + LiDAR + SLAM Toolbox + Teleop + RViz) |
| `mapeo3d` | Inicia mapeo 3D completo (Base + LiDAR + RealSense + RTAB-Map 3D + RViz) |
| `guardarmapa [nombre]` | Guarda mapa 2D en `.yaml` y `.pgm` con respaldo fechado |
| `guardarmapa3d [nombre]` | Guarda y exporta reconstruccion 3D (`.db`, nube `.ply`, malla texturada `.obj`) |
| `navegacion` | Compila y lanza el stack completo de navegacion autonoma Nav2 |
| `rviznav` | Abre RViz2 configurado para monitoreo de rutas y costo de navegacion |
| `rviz3d` | Abre RViz2 configurado para visualizacion de mapa 3D y nubes densas |
| `odometria` | Lanza unicamente el driver de la base hoverboard y el arbol URDF |
| `lidar` | Lanza unicamente el driver Ethernet del LiDAR Hokuyo |
| `control` | Lanza la teleoperacion por joystick PS4 con botones de seguridad |
| `stepper` | Abre la consola interactiva de control para el elevador NEMA 23 |
| `head_start` | Inicia en segundo plano el cabezal Jetson (RealSense D435 + Pan-Tilt) por SSH |
| `head_stop` | Detiene los procesos del cabezal Jetson por SSH |
| `head_status` | Consulta el estado de ejecucion de los procesos del cabezal en el Jetson |

---

## 7. Instalacion y Compilacion

### Requisitos del Sistema
- **Sistema Operativo NUC:** Ubuntu 24.04 LTS con ROS 2 Jazzy Jalisco.
- **Sistema Operativo Jetson:** Ubuntu 22.04 LTS con ROS 2 Humble Hawksbill.
- **Compilador:** GCC 13+ / Clang 18+.

### Proceso de Compilacion en NUC
```bash
cd ~/base_athome_ws
colcon build --symlink-install
source install/setup.bash
```
