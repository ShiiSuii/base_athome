# ROBOT.md — Conocimiento técnico de Base@Home

Documento maestro de **arquitectura, hardware, software, navegación, calibraciones y simulación**.
Cambios de arquitectura → se actualizan aquí (ver política en `AGENTS.md` §9).

**Etiquetas de evidencia** (definidas en `AGENTS.md` §8):
`VERIFICADO 2026-11-03` = observado por comando en vivo en esa fecha ·
`CODIGO` = leído en el código del repo · `HISTORICO <fecha>` = medido en esa fecha, no revalidado ·
`FABRICANTE` = ficha pública · `PENDIENTE` = sin evidencia, no inventar.

---

## 1. Visión general

Robot de servicio doméstico para **RoboCup@Home OPL** (UruBots). Base diferencial con motores de
hoverboard, columna con elevador, brazo AgileX Piper 6-DOF con cámara de muñeca, cabeza pan-tilt
con RealSense D435, micrófono direccional RØDE, pantalla facial y parlante.

Dos computadoras unidas por **Ethernet directo** con ROS 2 Jazzy + CycloneDDS:

```mermaid
flowchart LR
  subgraph NUC["NUC · Ubuntu 24.04 · Jazzy · 10.42.0.1"]
    HB[hoverboard_driver] --- NAV[Nav2 · AMCL · map_server<br/>PROTEGIDO]
    URG[urg_node /scan] --> NAV
    RS[RealSense D435] --> HRI[Percepción + HRI<br/>receptionist · misiones · manipulación]
    HRI -- NavigateToPose --> NAV
    STP[elevador ESP32-S3]
    UI[cara animada · TTS · pantalla invitado]
  end
  subgraph JET["Jetson Orin Nano · Ubuntu 24.04 · Jazzy · 10.42.0.12"]
    PIP[piper_arm_controller · CAN]
    PT[pan_tilt_node · U2D2]
    ASR[whisper_node · RØDE]
    ORB[stream_orbbec · DaBai]
    YOLO[yolo_trt_node · GPU TensorRT]
  end
  HRI <-- "/arm/* /pan_tilt/* /voice/* /arm_camera/*" --> JET
  NUC <-. "Ethernet 1 Gb/s · ROS_DOMAIN_ID=10 · CycloneDDS unicast" .-> JET
```

**Criterios de asignación de cómputo** (decisión 2026-10-30):
1. Cada nodo corre en el host donde está conectado su periférico (no viajan datos crudos ni comandos de bajo nivel).
2. La inferencia pesada va a la GPU de la Jetson si la entrada ya está allí.
3. La NUC queda para navegación, lógica de pruebas, UI y percepción de la cabeza.

| Función | Host | Motivo |
|---|---|---|
| Nav2, AMCL, `/scan`, odometría | NUC | sensores y base en la NUC; protegido |
| RealSense + HRI (rostros, personas, receptionist, máquinas de estado) | NUC | cámara en la NUC; necesita Nav2 y UI |
| TTS, pantalla, GUI, elevador, parser GPSR, editor URDF | NUC | hardware/herramientas locales |
| Piper (control 50 Hz), pan-tilt | Jetson | CAN y U2D2 en la Jetson |
| Cámara de muñeca + YOLO TensorRT → `/arm_camera/detections` | Jetson | solo viajan cajas, no imágenes |
| ASR Whisper | Jetson | micrófono en la Jetson; libera CPU de la NUC |
| Futuro: MoveIt con `agx_arm_ros` | NUC o Jetson | ambos en Jazzy; decidir midiendo |

## 2. Cómputo y sistema operativo

| | NUC (CHUWI LarkBox S) | Jetson Orin Nano Dev Kit **Super** (8 GB) |
|---|---|---|
| SO | Ubuntu 24.04.3, kernel 7.0 — `VERIFICADO 2026-11-03` | Ubuntu 24.04.4, L4T **R39.2.1** (JetPack 7.2.1), kernel 6.8.12-tegra; TensorRT 10.16.2 + CUDA 13.2 runtime — `VERIFICADO 2026-11-03` |
| ROS 2 | Jazzy, `rmw_cyclonedds_cpp`, `CYCLONEDDS_URI=~/cyclonedds.xml`, dominio 10 — `VERIFICADO` | Jazzy (`/opt/ros/jazzy`) — `VERIFICADO` |
| CPU / RAM | i3-1220P 10C/12T, 15 GiB, NVMe 468 GB | 6× A78AE; modo **25W** (CPU 1,344 GHz de 1,728) — `VERIFICADO`; 7,4 GiB compartida |
| GPU | Intel UHD (sin CUDA) | Ampere; CUDA 13.2, TensorRT 10.16 (`FABRICANTE` JP 7.2.1) |
| Almacenamiento | NVMe | **microSD** (flasheada 2026-10-30). NVMe recomendado para robustez |
| Entornos Python | `~/ros2_vision_venv` (percepción/HRI) | `~/robot_venv`; workspace `~/athome_ws` (**no es repo Git**, se despliega desde `jetson_head/`) |
| Servicios | `realsense-nuc.service` **no instalado** (`VERIFICADO`) | `jetson-head.service` (sistema) **activo** y `jetson-performance` (`jetson_clocks`) — `VERIFICADO` |

**Jetson migrada** el 2026-10-30 desde JetPack 6.2.3/Humble: flasheo por USB-C en recovery desde la
NUC (`jetson_head/scripts/flash_jetson_r39.sh`, microSD + QSPI), luego `bootstrap_jazzy.sh`
(Jazzy, venv, `gs_usb` compilado para 6.8, udev, red). **No se vuelve a migrar.** Ver §11 para recuperación.

**Riesgo `numpy`/`cv2`:** un export de Ultralytics instaló numpy 2 en `~/ros2_vision_venv` y rompió
`cv2` (revertido). No exportar modelos con ese venv; usar `YOLO_OFFLINE=1`.

## 3. Red y ROS 2 distribuido

| Host | Interfaz | Dirección | Estado |
|---|---|---|---|
| NUC | `enp1s0` (PCI) | `10.42.0.1/24`, NetworkManager "shared" (DHCP/NAT para la Jetson) | UP — `VERIFICADO 2026-11-03` |
| Jetson | `enx00e04c68090c` (adaptador **USB RTL8153**, `r8152`) | `10.42.0.12/24` fija (conexión `robot-nuc`) | UP — `VERIFICADO 2026-11-03` |
| Jetson | `enP8p1s0` (integrado) | — | DOWN, sin carrier desde 2026-10-29 (puerto o cable sospechoso) |
| NUC | `enx00e04c68060b` (USB) | `192.168.0.14/29` → Hokuyo `192.168.0.10` | red del LiDAR, independiente |

- Mediciones `HISTORICO 2026-10-30`: 1 Gb/s full, RTT 0,2 ms, ≥660 Mb/s por SSH, desfase de reloj −11,9 ms (NTP).
  La cámara de muñeca cruda ocupa ~150 Mb/s; no enviar RGB-D crudo de la D435 por la red (comprimir o procesar en origen).
- **CycloneDDS:** unicast, multicast desactivado, peers `10.42.0.1` y `10.42.0.12`. La Jetson liga por
  `address=10.42.0.12` (copia de referencia `jetson_head/config/cyclonedds_jetson.xml`). En la NUC
  `~/.bashrc` usa `~/cyclonedds.xml` solo si `enp1s0` tiene carrier; si no, `~/cyclonedds_local.xml`.
- **Dominios:** `10` = robot real; `42` = simulación/pruebas (`ATHOME_SIM=1`); `77` se usó para pruebas DDS sintéticas.
- Con ambos hosts en Jazzy, `ros2 node list` entre hosts funciona (la incompatibilidad Humble↔Jazzy de `ros_discovery_info` quedó resuelta).
- Diagnóstico de un comando: `scripts/net/check_nuc_jetson_link.sh` (carrier, IP, ping, SSH, `jetson-head`, reloj, DDS); `scripts/net/clock_offset.py`.
- **Reglamento:** prohíbe hotspots propios (descalificación) → el enlace debe ser cableado y ninguna máquina debe emitir WiFi.
- En competencia sin Internet: considerar la NUC como servidor de hora (chrony) — decisión humana.

## 4. Hardware, sensores y actuadores

| Componente | Host / interfaz | Datos | Estado |
|---|---|---|---|
| Base: 2 motores hub BLDC 6,5" de hoverboard con Hall | NUC, USB-TTL FTDI FT232R `/dev/ttyUSB0`, 115200 | placa STM32F103 (Lenzod o genérica) con firmware FOC propio (`firmware/`) | `CODIGO`; batería 38,5 V / 70 % vista en `/hoverboard/battery` (`HISTORICO 2026-10-30`) |
| Hokuyo UST-05LX | NUC, Ethernet `192.168.0.10` | 270°, 0,06–5 m, 40 Hz | `/scan` 40 Hz (`HISTORICO 2026-10-30`) |
| RealSense D435 | NUC USB 3 | perfil 640×480×15 (`realsense_nuc.launch.py`); fw 5.17.0.10 | 15 Hz estable 60 s (`HISTORICO 2026-10-30`) |
| Elevador NEMA23 + ESP32-S3 | NUC, CH343 `1a86:55d3` → `/dev/ttyACM0` | `scripts/stepper_controller.py`; firmware `firmware/nema23_esp32` | sin prueba física registrada; auditar homing/límites antes de usar |
| Piper 6-DOF + pinza | Jetson, CAN `gs_usb` (hoy **`can0`**; el número cambia entre arranques, `can1` = mttcan integrado sin transceptor) 1 Mbit/s; `can_port=auto` lo resuelve | 1,5 kg, ~626 mm (`FABRICANTE`); pinza: ficha 70 mm, feedback medido 75 mm, URDF y controlador 105 mm | `piper_sdk` 0.6.2 en `~/robot_venv`. Sin alimentación el 2026-11-03: 0 tramas RX en `can0`, `feedback_ok=false` (`JETSON_VERIFIED`) |
| Orbbec DaBai DC1 (muñeca, eye-in-hand) | Jetson USB | profundidad útil 0,3–3 m (`FABRICANTE`): **por debajo de ~30 cm no hay profundidad** | `stream_orbbec.py` corriendo — `VERIFICADO`; 20–30 Hz `HISTORICO` |
| Pan-tilt 2× Dynamixel XL430-W250 | Jetson, U2D2 (FTDI `0403:6014`) `/dev/ttyU2D2`, 1 Mbaud, prot. 2.0, IDs 1/2 | 12 V; `pan_center_tick=2059`, tilt homing offset −2212 y modo extendido (multivuelta); convención pan + izquierda, tilt + arriba | **2026-11-03:** con torque el U2D2 se desconecta por USB en segundos (~6/min), sin torque estable → **falla eléctrica** (12 V bajo carga / power hub / masa). El tilt quedaba corrido 1 vuelta y se trababa contra el tope (HW error 32 sobrecarga): corregido en software (desenvoltura de vueltas), más `pwm_limit` 600 y `position_p_gain` 450 |
| Micrófono RØDE VideoMic GO II | Jetson USB | shotgun supercardioide: **apuntarlo al invitado** | `whisper_node` corriendo — `VERIFICADO` |
| Pantalla facial, parlante, TTS | NUC (HDMI) | `animated_face_node.py`, `/robot/emotion` | `CODIGO`. El parlante HDMI no sirve para probar el camino acústico |

**Alimentación — `PENDIENTE` (sin evidencia eléctrica):** plan histórico del equipo: 7s6p NUC/LiDAR,
10s3p motores hoverboard, 7s5p elevador + Jetson, ~12 V para Dynamixel. Falta: BMS, fusibles por rama,
qué corta el E-stop, reguladores. **No usar como plano eléctrico; ningún agente ordena conectar baterías.**
El reglamento exige que el E-stop detenga *todo* (base, brazo, elevador, pan-tilt) — verificación eléctrica pendiente.

Firmware de la base: `firmware/README.md` y `firmware/docs/PINOUTS_AND_WIRING.md` (conexión por el
cable Hall derecho/USART3; **15 V nunca al conversor USB**). Flashear la placa es intervención de alto
riesgo (Nivel C).

## 5. URDF / Xacro, TF y calibraciones

- Entrada: `src/hoverboard_base/urdf/robot.urdf.xacro`; parámetros `xacro/properties.xacro`; módulos
  `base`, `torso`, `head`, `arm_piper`, `materials`. `robot.urdf` versionado = expansión del Xacro.
  `xacro` + `check_urdf`: OK, raíz `base_footprint`, 30 joints (11 móviles: `torso_lift_joint`, `pan/tilt_joint`, `joint1..8`).
- Árbol: `map → odom → base_footprint → base_link → {ruedas, caster, laser, torso_column → torso_lift_link → {mounting_cube → arm_base_link → link1..6 → pinza → cámara de muñeca, face_display_link, pan_tilt_base_link → pan_link → tilt_link → camera_link → frames ópticos}}`.
- `base_footprint` está **6 cm por debajo** de `base_link` (`base_joint z=0.06`, commit `b2b093d`, 2026-10-30); el puente MoveIt lee ese offset del URDF.
- **Protegidos (navegación/cinemática):** `track_width=0.5620`, `wheel_radius=0.0853`, `laser_offset_{x,y,z}=0.22/0/0.18`, límites del elevador.
- **`/joint_states` con un dueño por joint** (HOME-20): Piper publica `/arm/joint_states`, pan-tilt `/pan_tilt/joint_states`, torso `/torso/joint_states`; `joint_state_publisher` agrega. Las poses **comandadas** van a `/arm/joint_states_cmd` (solo visualización). Publicar ceros cuando no hay feedback es un defecto (ver §7, S-11).
- **Calibraciones y configuración del brazo:** `src/base_athome_perception/config/arm_poses_and_limits.yaml` (poses enseñadas HOME/PRE-GRASP/POST-GRASP, límites, `focal_px`; backup `<pose>_previous`), `safe_arm_trajectories.yaml`, `tracking_params.yaml`. Calibración ArUco: `src/base_athome_perception/scripts/calibrar_aruco_manipulacion.py`, marcador `aruco_marker_id0.png`, guía `aruco_calibracion_piper.pdf` (raíz del repo). Hand-eye completa con residuales: `PENDIENTE`.
- Medida física de la columna/torso para colisiones MoveIt: `PENDIENTE` (tarea humana).

### 5.1 Brazo Piper: diagnóstico (2026-11-03, `CODIGO` + cálculo offline; sin hardware)

Calculado con FK propia sobre el URDF expandido y comparado con el modelo DH del SDK
(`piper_kinematics.py`). Ninguna pose ni calibración fue modificada.

- **Montaje en el modelo:** Piper sobre la cara izquierda (+Y) del cubo del torso, **acostado**: `cube_to_arm_joint`
  `xyz (-0.15, 0.175, -0.11)`, `rpy (-π/2, 0, 0)` → el eje z de `arm_base_link` apunta a +Y (afuera) y su y a −Z.
  `arm_base_link` en `base_link` = **(0.005, 0.175, 1.065) m** con el elevador arriba (1,125 m sobre el piso);
  baja con `torso_lift_joint`: z = 0,465 (−0,60) y 0,315 (−0,75). Cadena: `base_link → torso_column →
  torso_lift_link (prismático) → mounting_cube_link → arm_base_link → link1..6 → gripper_base → {link7, link8,
  arm_camera_link → arm_camera_optical_frame}`, `link6 → tcp_link`.
- **¿Coincide con el robot real?** Evidencia indirecta a favor: el reposo medido por CAN con motores libres
  (`[86.97, 172.87, -162.63, -99.89, -0.68, 44.98]`) deja en el modelo la pinza **colgando vertical**
  (eje de avance (0.04, −0.09, −0.99)) a 0,30 m sobre `base_link`, junto al torso: es lo que haría la gravedad con
  el montaje lateral. **No hay medición física** del offset ni fotos: `PENDIENTE`. Medir (cinta, elevador en el tope
  superior): centro de la brida de la base del Piper respecto del eje de la columna (x, y), altura sobre el piso, y
  que la cara de montaje sea vertical (nivel); posición real del cubo (25×35×35 cm en el modelo).
- **Cinemática:** DH del SDK (`dh_is_offset=1`) ≡ URDF: error máx. **0,15 mm / 0,004°** en 2000 configuraciones
  aleatorias. `R_mount` de `piper_kinematics` = rotación del URDF. `tcp_link` = **punta de los dedos** (link6 +0,1358 m;
  los dedos van de 0,059 a 0,136 m); `wrist_geometry` usa centro de agarre = TCP − 2 cm.
- **Límites:** el URDF (= `SAFE_JOINT_LIMITS_DEG`) es **más amplio que la ficha** (j1 ±154 vs ±150, j2 0..195 vs
  0..180, j3 −175 vs −170, j4 ±110 vs ±100, j5 ±80 vs ±70, j6 ±180 vs ±120). Fuera de la ficha (dentro del URDF):
  `pre_grasp` (j2 184,7, j5 −74,8), `post_grasp` (j2 194,7), `DEFAULT_READY` del controlador (j2 182,7, j5 −75,1) y
  la secuencia `grasp_pringles` (j6 −154). Las de j2 se midieron por CAN moviendo el brazo a mano, así que el
  mecanismo llega; **los límites del firmware no se conocen**: leerlos en LAB (consulta de límites del SDK) antes de
  comandar esas poses. Si el firmware recorta, el brazo ejecuta otra trayectoria que la planificada.
- **Pinza:** URDF 0..52,5 mm por dedo (105 mm); ficha 70 mm; feedback histórico 75 mm. Masa `gripper_base`
  0,15 kg (fabricante 0,45): solo afecta dinámica (MoveIt no la usa).
- **Cámara de muñeca (URDF):** 25 mm en +x de `gripper_base`, 56 mm sobre el eje de avance, eje óptico = eje de avance,
  8 cm detrás de las yemas. Sin calibración hand-eye. `wrist_geometry` supone la lente sobre el eje
  (`camera_above_axis_m=0`) y estima la distancia por el **ancho de la caja YOLO con diámetro fijo 75 mm** (vale para
  un solo objeto; la DaBai no da profundidad < 30 cm y la profundidad no se usa).
- **Elevador sin realimentación:** `torso_lift_joint=0` lo publican `piper_arm_controller` (cada 5 ciclos), el
  receptionist y la manipulación autónoma **sin medirlo**, y `arm_manipulation.SHOULDER_IN_BASE` está fijo en z 1,065.
  Si el elevador baja, MoveIt y el cálculo del agarre quedan desplazados hasta 0,75 m. Hasta tener un estado medido
  del elevador, no manipular con el elevador fuera del tope superior.
- **Constantes obsoletas:** `piper_kinematics.HEAD_TO_ARM_OFFSET_M` y `CAMERA_LEVER_ARM_M` ("ZED Mini") no coinciden
  con el URDF (su comentario ubica el brazo en `torso_lift_link` (0.155, 0.175, −0.075); el URDF da (0.005, 0.175, −0.285)).
  Los usan `pringles_grasping_node.py`, `hri_receptionist_challenge.py` y `calibrar_aruco_manipulacion.py` (legado).
  `manipulacion_simple`/`arm_manipulation` usan el URDF (MoveIt) y no dependen de ellas.
- **MoveIt 2** (`base_athome_moveit_config`): grupo `arm` = cadena `arm_base_link → tcp_link`, grupo `gripper`
  (joint7/8), `home` = reposo medido, TRAC-IK (`Distance`, 0,1 s), OMPL, 1,5 rad/s y 1,0 rad/s², escala 0,3.
  **Solo planifica** (`allow_trajectory_execution: false`): `moveit_bridge` pide IK, plan articular y camino cartesiano
  con el estado inicial **medido**; la trayectoria se publica en `/arm/joint_trajectory` y el controlador la interpola
  a 50 Hz. ACM: se chequean solo pares brazo↔cuerpo; **todos los pares internos del brazo están deshabilitados**
  (pinza/cámara contra link2/link3 no se detecta); sin objetos de escena (la mesa no existe para MoveIt);
  `ensure_start_valid` libera durante la sesión los pares que ya chocan en el estado medido. Torso y cabeza son cajas
  sin medir. No hay acción `FollowJointTrajectory`: sin resultado, cancelación ni error de seguimiento estándar;
  `arm_manipulation` espera por tiempo.
- **Oficial `agx_arm_ros` (rama `ros2`, MIT):** MoveIt ejecuta con `JointTrajectoryController` de ros2_control
  (`/arm_controller/follow_joint_trajectory`) → `/control/joint_states` → driver `agx_arm_ctrl` (SDK **pyAgxArm**,
  no `piper_sdk`); feedback `/feedback/joint_states`, servicios `/enable_agx_arm`, `/control_enable`, `/emergency_stop`;
  `tcp_offset` configurable. **Recomendación:** no migrar el driver todavía (cambia el SDK y todas las interfaces
  `/arm/*`); sí adoptar su patrón: servidor `FollowJointTrajectory` en nuestro controlador con verificación de
  llegada por feedback, y gate de ejecución explícito.
- **Pipeline de agarre actual** (`arm_manipulation._grasp_worker`): detección YOLO en la muñeca → distancia por tamaño
  → centro del objeto por FK + `wrist_geometry` → IK de aproximación (10 cm) → plan → línea cartesiana → abrir →
  ejecutar → cerrar a 50 mm fijos → subir 3 cm → `post_grasp`. **Falta:** confirmar el agarre (ancho medido de la
  pinza), verificar la llegada por feedback, la mesa en la escena, la profundidad, hand-eye y el estado del elevador.
- **Controlador (`piper_arm_controller.py`), corregido en HOME-41** (tests; **no desplegado**): ver S-14 en §7.
  Siguen abiertos: heurística rad/grados de `/arm/joint_cmd` (todos |v| ≤ 3,5 → radianes), `/arm/status` publica
  `joints_deg` en cero sin feedback (los consumidores deben mirar `feedback_ok`), errores de TX CAN no reportados.

### 5.1b Manipulación: estado al 2026-11-03 (tarde)

- **Ejecución:** `piper_arm_controller` ofrece `/arm/follow_joint_trajectory` (`control_msgs`): rechaza goals sin habilitar
  o sin feedback, aborta por error de seguimiento > 15° y confirma la llegada con las articulaciones **medidas**
  (tolerancia 2°). Desplegado y `JETSON_VERIFIED` (rechazo con el brazo deshabilitado). La pinza tiene rampa propia:
  antes un `gripper_cmd` borraba la trayectoria (error "trayectoria perdida" visto por el usuario).
- **Objetivo fijado:** `arm_manipulation` usa un solo punto en `base_link` para RViz y para el agarre, calculado con las
  articulaciones del momento de la detección y retenido `object_hold_s` (8 s) si la detección parpadea
  (RViz: verde en vivo, amarillo retenido). Simulación con 80 % de cuadros perdidos: 4/4.
- **Alcance medido** (offline con límites del URDF y en simulación con MoveIt), desde la `pre_grasp` enseñada:
  línea recta con orientación fija ~15–20 cm de avance; con la pinza inclinada hasta 24–30° **~22–25 cm**; **30 cm
  inalcanzable en cualquier orientación** (el TCP ya está a 44 cm del hombro). Para objetos más lejos: acercar la
  base/elevador u observar desde una pose más retraída (sobre la misma línea hay 60 cm de recorrido).
  El planificador prueba 17 orientaciones (0–30°) con filtro rápido de IK en el punto de agarre: 22 cm → éxito,
  ~10 s de planificación; 26 cm → rechazo explícito en ~7 s. Timeout de IK 0,5 s (antes 0,2 s fallaba soluciones existentes).
- **Colisiones:** ACM regenerado por muestreo (2000 configuraciones): 117 pares brazo↔cuerpo y 18 internos del brazo
  chequeados (antes 0 internos); cámara↔dedos deshabilitados (solapan en reposo). La mesa todavía no está en la escena.
- **Elevador:** nodo `torso_state` (`base_athome_bringup`, solo lectura: envía `?`) publica `/torso/joint_states` únicamente
  si el ESP32 está calibrado; la manipulación y MoveIt usan esa altura. Sin probar en el robot (ESP32 no conectado).

### 5.1c Teleoperación con Meta Quest 2 (2026-11-03, `SIM_PASS`; real parcial)

- **Cadena:** Quest 2 por USB a la **Jetson** → navegador del Quest abre `http://localhost:8765` (túnel `adb reverse`,
  sin WiFi; el servidor escucha en `127.0.0.1` y `::1` porque `adb` resuelve `localhost` a IPv6) → página WebXR
  (`immersive-ar` passthrough si está disponible) envía botones + pose del control a 30 Hz → `jetson_head/scripts/quest_teleop_node.py`
  → `/arm/joint_trajectory` (horizonte 0,15 s) y `/arm/gripper_cmd` → `piper_arm_controller` (límites, feedback, rechazo si deshabilitado).
- **Modos:** `cartesian` (por defecto): embrague + pose 6D relativa → TCP, IK de mínimos cuadrados amortiguados sobre el DH del SDK
  (`quest_cartesian_core.py`, idéntico a `piper_kinematics`), filtro de temblor, zona muerta, `rot_scale`; `joint`: joystick por pares.
- **Robot real (`real:=true`):** X = hombre muerto y embrague; primero HOME (`home_deg`, 15°/s) solo con X; al soltar X se manda
  sostener la posición medida; objetivo ≤ `max_lead_deg` (5°) de lo medido; 12°/s brazo, 25°/s muñeca; `ik_damping` 80 (cerca
  de la singularidad de muñeca en HOME reduce el giro opuesto j4/j6 de 40° a 7°). El nodo nunca habilita el brazo.
- **Diagnóstico:** `/quest_teleop/status` (página viva, XR, mano, pose recibida, embrague, `track_mm`/`track_deg` = objetivo vs FK
  medido, errores de la página). Con `keep_awake` el Quest no se duerme sin el visor (proximidad simulada; se revierte al salir).
- **Trampa conocida:** el navegador restaura pestañas viejas que siguen enviando datos (sin pose): cerrar todas antes de usar.
- **Fidelidad (2026-11-04, medida con modelo de la cadena, brazo ideal):** a 12 cm/s el error pasó de 47 mm / 460 ms a **10 mm / 80 ms**; a 3 cm/s, de 8 mm / 300 ms a 3 mm / 120 ms. Causas corregidas: horizonte de 3 ciclos (se recorría 1/3 de cada tramo), filtro fijo → **One Euro**, límites por articulación que deformaban la trayectoria → escalado uniforme, amortiguación fija → adaptativa según el menor valor singular (5 lejos, 80 cerca), 25 Hz nodo / 60 Hz página, escalas 1:1. Velocidades 20/40°/s: con 12/25 el brazo no alcanza una mano a velocidad normal. Validación: `quest_teleop_simcheck.py` (solo dominio 42): 40 mm pedidos → 39,7 mm. `HARDWARE_VERIFIED`: pendiente; la lag de firmware se mide con `track_mm`.
- **GRIP (modo real) = calibrar frente:** apuntar el control hacia donde mira el robot. Sin calibrar, el frente es hacia donde miraba el visor al entrar en VR.
- **Abierto, SDK `piper_sdk` 0.6.2:** `MotionCtrl_2(installation_pos)` admite 0x01 horizontal, 0x02 lateral izquierdo, 0x03 lateral derecho ("cable hacia atrás"); nunca lo configuramos y el brazo está montado de costado. Probable efecto en la compensación de gravedad: `PENDIENTE` de verificar en LAB, con el lado confirmado físicamente. Además usamos MOVE J con `stream_speed_pct` 60 (planificador interno del firmware); MIT (0xAD) no se usa.
- **Referencias:** [XRoboToolkit](https://arxiv.org/html/2508.00097v2) (relativo con embrague; IK QP con manipulabilidad), [LeFranX](https://www.emergentmind.com/topics/lefranx), [ExtremControl](https://arxiv.org/pdf/2602.11321) (feedforward de velocidad), [filtro 1€](https://cristal.univ-lille.fr/~casiez/1euro).

### 5.2 Procedimiento de calibración eye-in-hand (DaBai en la pinza) — `PENDIENTE` de ejecutar

Núcleo implementado y probado con datos sintéticos: `base_athome_perception/handeye.py` (5 tests; solvePnP
`IPPE_SQUARE`, `calibrateHandEye` con los 5 métodos de OpenCV, residual de consistencia, validación independiente,
criterio de aceptación). El script `calibrar_aruco_manipulacion.py` **no** hace hand-eye (centrado 2D).
Requiere `LAB_SUPERVISADO` (mueve el brazo) y el controlador con S-14 desplegado.

1. **Intrínsecos:** tablero de ajedrez impreso y medido; ≥20 vistas variadas a 30–60 cm con `camera_calibration`
   (u OpenCV) sobre `/arm_camera/image_raw` en la resolución de trabajo. Aceptar RMS < 0,5 px. Guardar K y dist
   (reemplazan `focal_px=485`, que hoy es un supuesto).
2. **Marcador:** ArUco `DICT_4X4_50` id 0 impreso plano, **lado medido con calibre** (no el nominal), fijo a la mesa.
3. **Muestras (≥15):** el humano lleva el brazo (motores libres o poses enseñadas) a vistas del marcador a 35–60 cm,
   con **≥20° de variación de rotación en dos ejes**. En cada una: quieto 1 s, guardar articulaciones **medidas**
   (`/arm/joint_states` con `feedback_ok`) y la pose del marcador con su error de reproyección (descartar > 1 px).
4. **Resolver:** `handeye.solve()` ordena los métodos por residual. Aceptar si residual ≤ 5 mm / 1°.
5. **Validar independientemente:** 3–5 puntos con posición medida en `base_link` (cinta desde el eje de la columna y
   el piso, o la punta de la pinza tocando el punto) → `validate_points`; aceptar si el peor ≤ 10 mm. Sin este paso
   la calibración **no** es válida (`accept()` la rechaza).
6. **Publicar y persistir:** reemplazar `arm_camera_mount_joint` del Xacro por el resultado (respecto de
   `gripper_base`), regenerar `robot.urdf`, registrar fecha, método, residual y validación en esta sección; el
   resultado crudo va en `arm_poses_and_limits.yaml` (`wrist_camera_handeye`). Repetir si se desmonta la cámara.

## 6. Paquetes y software

| Ruta | Contenido |
|---|---|
| `src/hoverboard_base` | driver de la base, URDF/Xacro, `bringup/teleop_joy/view_robot.launch.py` — **driver y config protegidos** |
| `src/base_athome_navigation` | `nav2_bringup.launch.py`, `nav2_params.yaml`, BT, `teleop_ps4` — **protegido**; capa semántica editable: `semantic_map.yaml`, `waypoints.yaml`, `map_zone_editor.py`, `semantic_objects.py` (9 tests) |
| `src/base_athome_mapping` | `slam_toolbox`, `slam_2d/3d_mapping`, `rtabmap_realsense` — **protegido** |
| `src/base_athome_bringup` | `base.launch.py`, `realsense_nuc.launch.py` |
| `src/base_athome_perception` | HRI receptionist, rostros (YuNet/SFace), pose (YOLOv8-pose), `voice_client`, vocabulario, secuenciador de misiones, manipulación (`manipulacion_simple`, `manipulacion_autonoma`, teach panel, `pose_store`, `moveit_bridge`), `guest_input_server` (111 tests) |
| `src/base_athome_moveit_config` | SRDF, límites, kinematics, `move_group.launch.py` |
| `src/base_athome_skills` | detector de puerta abierta desde `/scan` (señal de inicio, regla 3.6.8) (8 tests) |
| `src/base_athome_sim` | simulador liviano + opción Gazebo Harmonic (6 tests) |
| `src/piper_description`, `src/piper_msgs`, `src/urg_node` | descripción/mensajes del Piper; driver Hokuyo (externo) |
| `jetson_head/` | fuente de lo que corre en la Jetson: `src/{audio_asr, pan_tilt_controller, robot_bringup_jetson}`, `scripts/` (Piper, Orbbec, YOLO TRT, `start_mapping_head.sh`, `jetson-head.service`, `deploy_to_jetson.sh`, `bootstrap_jazzy.sh`, `flash_jetson_r39.sh`, `build_yolo_trt.sh`) — `COLCON_IGNORE` en la NUC |
| `tools/urdf_editor` | editor Xacro/URDF en `127.0.0.1:8765`, solo preview (45 tests; ver su README) |
| `tools/gpsr_parser` | parser GPSR determinista texto→plan JSON (34 tests; ver su README) |
| `scripts/` | alias (`aliases/nuc_aliases.sh`, ayuda: `ayuda`), red, `systemd/realsense-nuc.service`, `stepper_controller.py`, mapas |
| `firmware/` | firmware hoverboard (STM32) y elevador (ESP32-S3) |

**Despliegue a la Jetson:** `jetson_head/scripts/deploy_to_jetson.sh` (dry-run por defecto, respaldo en
`~/deploy_backups/<fecha>/`, no pisa calibraciones, no reinicia servicios). `scripts/jetson/` es una copia
duplicada: eliminada (S-05).

### Interfaces ROS principales (`CODIGO`; tasas `HISTORICO 2026-10-30`)

| Interfaz | Tipo | Productor → consumidor |
|---|---|---|
| `/cmd_vel` | Twist | Nav2 / teleop → `hoverboard_driver` |
| `/odom`, `/odom_raw` (50 Hz), `/hoverboard/battery`, srv `/hoverboard/reset_odom` | Odometry / BatteryState / Trigger | `hoverboard_driver` |
| `/scan` (40 Hz) | LaserScan | `urg_node` |
| action `navigate_to_pose` | NavigateToPose | Nav2 ← receptionist, misiones, `waypoint_navigator` |
| `/arm/joint_states` (50 Hz), `/arm/status` (JSON: `enabled`, `feedback_ok`), `/arm/current_pose` (brida link6 según el firmware, frame `arm_base_link`, solo con feedback) | JointState / String / PoseStamped | `piper_arm_controller` |
| `/arm/joint_cmd`, `/arm/joint_trajectory`, `/arm/gripper_cmd`, `/arm/enable`; srv `/arm/set_enable`, `/arm/home`… | — | apps → Piper (**actuador**) |
| `/pan_tilt/joint_states` (20 Hz), `/pan_tilt/connected` (latched) | JointState / Bool | `pan_tilt_node` |
| `/pan_tilt/command`, `/pan_tilt/target_degrees`; srv `/pan_tilt/set_torque` | — | apps → pan-tilt (**actuador**) |
| `/voice/listen_request` (JSON con id), srv `/voice/listen_trigger`, `/voice/result`, `/voice/transcription`, `/voice/status`, `/voice/mute`, `/voice/debug_inject` | — | ver `docs/MICROFONO_ASR.md` |
| `/arm_camera/image_raw` (~30 Hz), `/arm_camera/detections` (Detection2DArray, 29,9 Hz medido en la NUC 2026-11-03) | — | Jetson → NUC |
| `/camera/camera/color/image_raw`, `aligned_depth_to_color` | — | RealSense (NUC) |
| `/hri/guest_input` (JSON `{type, value}`), `/hri/say_something`, `/robot/emotion`, `/mission/command` | — | HRI / pantalla / misiones |
| `/mission/run` (plan JSON), srv `/mission/cancel`, `/mission/status` (JSON, latched), `/mission/skills` | String / Trigger | `mission_executor` (`src/base_athome_skills/README.md`) |
| `/commander/command` (texto), `/commander/result` (JSON), web `127.0.0.1:8770` | String | usuario → `commander` → `/mission/run` |

### Seguridad del software de actuadores (vigente, no revertir)

- Piper: `auto_enable=False` por defecto; **rechaza habilitarse sin tramas CAN recientes** (`feedback_timeout_s=0.5`); `/arm/status` informa `enabled`/`feedback_ok`.
- Pan-tilt: `auto_torque=false`; torque solo con `/pan_tilt/set_torque` o `pan_tilt_auto_torque:=true` (el HRI llama al servicio al empezar). Reboot/EEPROM de servos solo tras habilitar.
- HRI: perfiles sintéticos (`synthetic`) excluidos del matching; nunca guardar un vector aleatorio como rostro real.
- `jetson-head` arranca Piper (deshabilitado), pan-tilt (sin torque), DaBai, Whisper `base.en`/`en` y YOLO si existe el engine. Overrides: `WHISPER_MODEL`, `ASR_LANGUAGE`, `ENABLE_YOLO`, `YOLO_ENGINE`.

## 7. Hallazgos de auditoría de software (S-xx)

| ID | Severidad | Hallazgo | Estado |
|---|---|---|---|
| S-01 | Crítica | Piper se habilitaba solo al arrancar | **Corregido y desplegado** |
| S-02 | Alta | DDS de la Jetson sin fallback si falta el cable | fallback solo-localhost en repo, **no desplegado** |
| S-03 | Alta | Pan-tilt habilitaba torque solo | **Corregido y desplegado** |
| S-04 | Alta | Perfiles HRI con embeddings aleatorios | **Corregido** |
| S-05 | Media | Scripts duplicados `scripts/jetson/` vs `jetson_head/scripts/` | **Corregido** (`scripts/jetson/` eliminado 2026-11-03) |
| S-06 | Media | 29 archivos con rutas absolutas `/home/athome` (incluye `nav2_bringup.launch.py`, BT y mapa por defecto `~/mapa_base_athome.yaml`) | Propuesto fuera de navegación; en navegación solo documentar |
| S-07 | Media | `except Exception: pass` en el bucle del Piper | Parcial (feedback guard) |
| S-08 | Media | Linters ament fallan en todos los paquetes; no hay CI | Propuesto (HOME-16) |
| S-10 | Baja | entry points de `base_athome_perception` irregulares; alias ejecutan scripts directos | Propuesto |
| S-11 | Media | Pan-tilt/Piper publican 0.0 como medido sin hardware | Parcial: flags `connected`/`feedback_ok`; faltan consumidores |
| S-12 | Media | Cámara de muñeca no publicaba / 5 Hz | **Corregido** (20 Hz, CPU 19 %) |
| S-09/S-13 | — | Humble↔Jazzy | **Resuelto** por la migración |
| S-14 | Alta | Piper: trayectorias recibidas con el brazo deshabilitado quedaban guardadas y **se ejecutaban (o saltaban al final) al habilitar**; deshabilitar no las borraba; sin límites articulares; con feedback perdido seguía la trayectoria a ciegas 5 s; `/arm/home`, `/pre_grasp`, `/grasp_pringles`, `/release` respondían `success` sin estar habilitado; `/arm/current_pose` en ceros y con frame `base_link` (es `arm_base_link`); pinza recortada a 70 mm en `/joint_states` | **Corregido y desplegado** 2026-11-03 (md5 `2e347c49`), + FollowJointTrajectory y rampa de pinza |
| S-15 | Alta | Elevador sin feedback: `torso_lift_joint=0` sintético y `SHOULDER_IN_BASE` fijo | Parcial: nodo `torso_state` + geometría con altura medida; falta correrlo en el robot y apagar `publish_assumed_torso` |
| S-16 | Media | Poses enseñadas y del controlador fuera de los límites de la ficha; límites del firmware desconocidos | Abierto: leer límites en LAB |
| S-17 | Media | ACM sin autocolisión interna del brazo; mesa fuera de la escena | Parcial: 18 pares internos chequeados; falta la mesa |
| S-18 | Media | Constantes de cámara/brazo obsoletas en `piper_kinematics` (código legado) | Abierto |

## 8. Navegación actual (SOLO LECTURA — no modificar)

Funciona según el usuario y es **protegida** (`AGENTS.md` §4). Lo siguiente es `CODIGO` leído, sin cambios.

**Cadena:** `Nav2 → /cmd_vel → hoverboard_driver → UART 50 Hz → placa STM32 FOC → motores` y de
vuelta `ticks Hall → /odom + TF odom→base_footprint (50 Hz) → AMCL (map→odom) → Nav2`.

- **Placa:** config Lenzod (`firmware/firmware_config/lenzod_red_stm32_usart3_odometry/config.h`): FOC en `VLT_MODE`, `I_MOT_MAX 15 A`, `N_MOT_MAX 1000 rpm`, `TIMEOUT 20` comandos perdidos → apagado, `ENABLE_ODOMETRY`. El binario realmente flasheado: `PENDIENTE`.
- **Driver** (`src/hoverboard_base/hoverboard_base/hoverboard_driver.py`, parámetros efectivos en
  `config/hoverboard_params.yaml`): trama `0xABCD` + steer + speed + checksum XOR (protocolo serie del
  firmware FOC tipo EFeru); telemetría con contadores Hall por rueda, RPM, batería y temperatura.
  `wheel_radius_m=0.0853`, `track_width_m=0.5620`, `ticks_per_rev=90` (15 pares de polos × 6),
  `linear_scale=280`, `angular_scale=260`, `max_speed_raw=450`, `max_steer_raw=400`,
  **pisos anti-stiction** `min_speed_raw=75`, `min_steer_raw=165`, **watchdog `cmd_vel_timeout=0.5 s`**
  (envía stop), `publish_tf=true`. Los defaults del `.py` difieren del YAML: manda el YAML.
- **Cinemática diferencial:** `v=(v_r+v_l)/2`, `ω=(v_r−v_l)/b`, `v=r·ω_rueda`.
- **Nav2** (`src/base_athome_navigation/config/nav2_params.yaml`, `launch/nav2_bringup.launch.py`):
  `map_server` + **AMCL** (`DifferentialMotionModel`, `likelihood_field`, `base_footprint`), planner
  **SmacPlanner2D**, controlador **RotationShimController** (gira en el lugar si el error > 0,785 rad;
  `rotate_to_goal_heading`) envolviendo **RegulatedPurePursuit** (`desired_linear_vel 0.38`,
  `min_lookahead 0.58` > trocha), `controller_frequency 15`, tolerancias xy 0,18 m / yaw 0,10 rad,
  `robot_radius 0.44`, `inflation_radius 0.59`, behaviors Spin/BackUp/DriveOnHeading/Wait, BT propio
  `config/bt_trees/base_athome_bt.xml` (ruta absoluta). Mapa por defecto `~/mapa_base_athome.yaml`.
  El launch incluye base, LiDAR y teleop.
- **Interfaz que usan las apps:** acción `navigate_to_pose` y waypoints/zonas de la capa semántica.
  Integrar siempre como cliente de la acción (mocks en simulación), nunca cambiando Nav2.
- **Mapeo:** `base_athome_mapping` (slam_toolbox 2D, RTAB-Map con D435). El Restaurant (2026.2) exige
  navegar sin mapa previo → SLAM online; evaluar sin tocar la configuración operativa.

**Preguntas abiertas** (propuestas NAV-PROP en `docs/ROADMAP.md` §6): watchdog de la placa si se
pierde UART, covarianzas de odometría, dueño único de `odom→base_footprint`, prioridad Nav2 vs teleop,
desacople footprint visual/operativo, efecto del `base_joint z=0.06` sobre la altura del láser.

### Bases hoverboard de referencia (estudio, no adoptar sin decisión)

| Fuente | Qué estudiar | Advertencia |
|---|---|---|
| [DataBot-Labs/hoverboard_ros2_control](https://github.com/DataBot-Labs/hoverboard_ros2_control) | `ros2_control` hardware_interface | verificar licencia/hardware |
| [TheHassanShahzad/OpenBase2](https://github.com/TheHassanShahzad/OpenBase2) | driver/odom/diff_drive/URDF | proyecto chico |
| [Cornell IRL mobile_hoverboard_robot_base](https://github.com/IRL-CT/mobile_hoverboard_robot_base) | plataforma HRI, wiring, calibración | usa ODrive, no nuestra placa |
| [hoverboard-robotics/hoverboard-driver (rama humble)](https://github.com/hoverboard-robotics/hoverboard-driver/tree/humble) | driver UART ROS 2 | `master` es ROS 1: no usar |
| [EFeru/hoverboard-firmware-hack-FOC](https://github.com/EFeru/hoverboard-firmware-hack-FOC) | firmware FOC, protocolo serie, protecciones | **solo lectura**; flashear = Nivel C |
| [ros2_controllers diff_drive](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html), [Nav2 first-time setup](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/) | referencia oficial | no re-tunear por moda |

## 9. Simulación (`base_athome_sim`)

Reemplaza Jetson (Piper, pan-tilt, Whisper), RealSense, cámara de muñeca y Nav2 con **las mismas
interfaces**; las apps solo notan `ATHOME_SIM=1`. Siempre en `ROS_DOMAIN_ID=42`. Uso: `README.md`.

| Real | Simulado | Fidelidad |
|---|---|---|
| Piper | `/arm/*` iguales, perfil coseno, trayectorias | cinemática exacta; sin dinámica/colisiones |
| Pan-tilt | `/pan_tilt/*` iguales, 90 °/s | buena |
| Whisper | protocolo con id, `/voice/result`, espera pausada por mute | misma temporización; no simula errores de ASR |
| RealSense | color + profundidad alineada + `camera_info`; invitado = foto | rostros/pose funcionan; "sentado" no se simula |
| Cámara de muñeca | lata magenta proyectada con la pose real del brazo | geometría correcta |
| Nav2 | acción `navigate_to_pose`, `/amcl_pose`, `/odom`, TF | línea recta 0,5 m/s, sin planificador ni obstáculos; un goal nuevo aborta al anterior (como Nav2) |
| Hokuyo | raycasting sobre el mapa real | visualización |
| Gazebo (`gazebo:=true`) | URDF real + `gz_model.py`, mundo `worlds/casa_simple.sdf`, `ros_gz_bridge` con hora de pared | física de base y sensores; el agarre no es físico |

Con `ATHOME_SIM=1`: base de rostros y config del brazo separadas en `~/.local/share/base_athome/sim/`,
no arranca la RealSense, detector por color (`SimColorDetector`).
**No prueba:** audio real, postura sentada, Nav2 real, agarre físico.

## 10. Pruebas disponibles

| Suite | Comando (desde la raíz, `ROS_DOMAIN_ID=42`) | Resultado 2026-11-03 |
|---|---|---|
| Percepción/HRI | `python3 -m pytest -q src/base_athome_perception/test` | 111 passed |
| Simulación | `python3 -m pytest -q src/base_athome_sim/test` | 6 passed |
| Skills (puerta + misiones + commander) | `env -u PYTHONPATH python3 -m pytest -q src/base_athome_skills/test` | 63 passed, 1 skipped (2026-11-04) |
| Elevador | `ROS_DOMAIN_ID=77 python3 -m pytest -q src/base_athome_bringup/test/test_torso_state.py` | 6 passed |
| Aprobación acotada Antigravity | `cd tools/agents && python3 -m pytest -q` | 40 passed |
| Misiones en simulación | `[RVIZ=1] src/base_athome_skills/scripts/sim_mission.sh <plan.yaml> <mapa_sim>` | demo_fetch_bottle SUCCEEDED (72–112 s) |
| Commander en simulación | `[RVIZ=1] src/base_athome_skills/scripts/sim_commander.sh ["pedido" ...]` (sin pedidos: web en `:8770`) | pedido compuesto SUCCEEDED en 66 s; cancelación, objeto no visto y lugar desconocido correctos (2026-11-04) |
| Capa semántica | `python3 -m pytest -q src/base_athome_navigation/test` | 9 passed |
| Scripts Jetson (Piper, FJT, Orbbec, YOLO, bench) | `ROS_DOMAIN_ID=77 python3 -m pytest -q jetson_head/scripts/tests` | 32 passed |
| ASR | `cd jetson_head/src/audio_asr && python3 -m pytest -q test/test_asr_core.py` | 12 passed |
| Pan-tilt | `python3 -m pytest -q jetson_head/src/pan_tilt_controller/test` | 5 passed |
| GPSR | `cd tools/gpsr_parser && env -u PYTHONPATH python3 -m pytest -q tests` (sin entorno ROS: el plugin `launch_testing` rompe la colección) | 34 passed |
| Editor URDF | `cd tools/urdf_editor && .venv/bin/python -m pytest -q` | 45 passed |
| URDF | `xacro src/hoverboard_base/urdf/robot.urdf.xacro > /tmp/r.urdf && check_urdf /tmp/r.urdf` | OK (2026-10-29) |
| Linters ament | — | fallan (preexistente, S-08) |

Todo esto es `UNIT_PASS`/`SIM_PASS`; ninguna prueba con actuadores reales está registrada.

## 11. Recuperación y mantenimiento de la Jetson

- **No reflashear** salvo falla grave y autorización Nivel C. Si hiciera falta: recovery por USB-C
  (`lsusb` → `0955:7523`), `flash_jetson_r39.sh prepare|flash` (borra microSD y actualiza QSPI;
  pide `CONFIRM=BORRAR_MICROSD`), luego en la Jetson IP fija en el adaptador USB
  (`nmcli … ifname enx00e04c68090c … 10.42.0.12/24 gw 10.42.0.1`), `ssh-copy-id`, y desde la NUC
  `deploy_to_jetson.sh --apply` + `bootstrap_jazzy.sh --apply`. Con el QSPI en r39 una microSD con
  JetPack 6 probablemente no arranca. Runbook completo: `git show ac9b069:docs/RUNBOOK_MIGRACION_JETSON.md`.
- Respaldos en la NUC: `~/jetson_backups/2026-10-29_pre_migration/`, `~/jetson_backups/2026-10-30_pre_migration_extra/`
  (modelos Whisper/YOLO, `gs_usb`, udev/systemd/CAN), ISO r39.2.1 en `~/jetson_backups/jetpack_7.2.1/`.
  Respaldos de cada despliegue en la Jetson: `~/deploy_backups/<fecha>/`.
- **TensorRT (2026-11-03):** el flasheo instaló solo el BSP. Se agregaron del repo apt oficial de NVIDIA
  (r39.2) `libnvinfer-bin`, `python3-libnvinfer` y `cuda-cudart-13-2` (TRT 10.16.2, CUDA 13.2; 12 paquetes,
  lista y rollback `apt-get remove` en `~/deploy_backups/trt_20261103/installed.txt`). Engines FP16 en
  `~/models/yolo/*_fp16.engine`. `cuda-python 13.2` vive en `~/robot_venv`; el nodo YOLO corre con ese
  Python (`YOLO_PYTHON` en `start_mapping_head.sh`). Benchmark: `jetson_head/scripts/bench_yolo.py`.
- **Pendiente:** ctranslate2 con CUDA (Whisper sigue en CPU int8), `MAXN_SUPER` solo si la fuente lo soporta (decisión humana).
- Comandos de inspección seguros: `cat /etc/os-release; head -1 /etc/nv_tegra_release; systemctl is-active jetson-head; journalctl -u jetson-head -n 30 --no-pager`.

## 12. Limitaciones y riesgos conocidos

- Seguridad de host (2026-11-03): URL de `origin` sin credenciales; NUC sin sudo sin contraseña; Jetson aún con `90-ubuntu-nopasswd` (`NOPASSWD: ALL`) más `91-ubuntu-jetson-head` (solo `systemctl/journalctl jetson-head`) a la espera de confirmar la contraseña de `ubuntu`; auto-aprobador de Orca eliminado.
- Orbbec sin profundidad < 30 cm → estrategia de pose de observación a 35–45 cm, aproximación final abierta y confirmación por feedback de la pinza. Para profundidad registrada migrar a `OrbbecSDK_ROS2` (`v2-main`, verificar DC1).
- Pringles (~75 mm) en el límite de apertura de la pinza (~70 mm).
- Hokuyo de 5 m: limitado para lugares grandes (Restaurant).
- Whisper en CPU (~300 % CPU al transcribir) comparte la Jetson con YOLO.
- Arranque desde microSD: lento y sensible a cortes de energía.
- D435: 848×480 es la resolución de profundidad óptima según Intel (`FABRICANTE`); hoy se usa 640×480×15. Un solo dueño de la cámara (ya hubo congelamientos por doble apertura).

**Preguntas abiertas para el equipo (no adivinar):** BMS, fusibles, reguladores y qué corta el E-stop;
versiones de firmware de la placa hoverboard y del ESP32; qué dispositivos reciben energía por USB con
solo la NUC encendida; límites mecánicos medidos del elevador y montaje del Piper; si existen fotos,
CAD o esquemas fuera del repo; uso de `can0` vs `can1` si se agrega otro dispositivo CAN.

## 13. Referencias oficiales y de equipos

- Reglamento fijo: [RuleBook 2026.2](https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2) (no `master`).
- ROS 2 Jazzy: <https://docs.ros.org/en/jazzy/> · Nav2: <https://docs.nav2.org/> · MoveIt 2: <https://moveit.picknik.ai/main/index.html> · REP 103/105.
- CycloneDDS: <https://cyclonedds.io/docs/cyclonedds/latest/config/index.html>
- Jetson: [JetPack](https://developer.nvidia.com/embedded/jetpack/downloads), [Orin Nano user guide](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/).
- Hardware: [piper_sdk](https://github.com/agilexrobotics/piper_sdk) (MIT, en uso), [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros) (rama `ros2`, Jazzy + MoveIt, candidato), [agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf), [OrbbecSDK_ROS2](https://github.com/orbbec/OrbbecSDK_ROS2), [realsense-ros](https://github.com/IntelRealSense/realsense-ros), [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK), [XL430 e-manual](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/).
- Calibración: [OpenCV ArUco](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html), [calibrateHandEye](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html), [easy_handeye2](https://github.com/IFL-CAMP/easy_handeye2).
- URDF web: [urdf-loaders](https://github.com/gkjohnson/urdf-loaders), [xacro](https://github.com/ros/xacro).
- Equipos @Home (licencia "ninguna" = solo estudiar, no copiar):

| Repo | Licencia | Para qué |
|---|---|---|
| [RoBorregos/home2](https://github.com/RoBorregos/home2) (FRIDA) | ninguna | arquitectura por áreas, Docker L4T, HRI speech pipeline |
| [RoBorregos/frida-cortex](https://github.com/RoBorregos/frida-cortex) | MIT | intérprete GPSR → acciones |
| [SparkRibeiro21/charmie_ws](https://github.com/SparkRibeiro21/charmie_ws) | MIT | workspace completo con Orbbec |
| [fbotathome/fbot_hri](https://github.com/fbotathome/fbot_hri), [fbot_manipulator](https://github.com/fbotathome/fbot_manipulator) | ninguna | STT por pedido; diseño de action PICK/PLACE con MTC |
| [ros4hri](https://github.com/ros4hri) | Apache-2.0 | estándar de personas/caras/cuerpos |
| [RoboCupAtHome/CommandGenerator](https://github.com/RoboCupAtHome/CommandGenerator) | ver repo | dataset GPSR oficial |
| [socrob/doorbell_detector](https://github.com/socrob/doorbell_detector) | ninguna | timbre con YAMNet (reimplementar) |
| [USTC-WrightEagle-AI](https://github.com/USTC-WrightEagle-AI) | ninguna | grasping, ASR offline sherpa-onnx |
