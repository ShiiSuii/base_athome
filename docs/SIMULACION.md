# Simulación de Base@Home (`base_athome_sim`)

Objetivo: seguir desarrollando **receptionist, manipulación y misiones** sin el robot, con el
**mismo código** que corre en el robot real. El simulador reemplaza a la Jetson (Piper, pan-tilt,
whisper), a la RealSense, a la cámara de muñeca y a Nav2, con **los mismos tópicos, servicios y
acciones**. Las apps no saben que están en simulación, salvo por `ATHOME_SIM=1`, que solo
cambia rutas de datos (ver más abajo).

## Uso rápido (alias en `scripts/aliases/nuc_aliases.sh`)

```bash
# Terminal 1: robot simulado + RViz
simulacion                      # liviana, sin física (recomendada en casa, casi sin CPU)
sim_gazebo                      # Gazebo Harmonic: física de la base, lidar y cámaras renderizadas
simulacion auto_guest:=true     # el invitado responde solo cada vez que el robot escucha

# Terminal 2: la app real, en el dominio de la simulación
sim_env                         # ROS_DOMAIN_ID=42 + ATHOME_SIM=1 (solo esta terminal)
receptionist --sim-nav --max-guests 1      # o: manipulacion | mision
sim_salir                       # vuelve la terminal al robot real (dominio 10)

# Terminal 3: hacer de invitado
sim_invitado door               # aparece un invitado en la puerta (none = se va, next = otro invitado)
sim_decir "My name is Robin and my favorite drink is iced tea"
pantalla_invitado               # responder tocando la pantalla (http://127.0.0.1:8090)
```

> **Seguridad:** la simulación usa `ROS_DOMAIN_ID=42`. El robot real usa 10. Nunca se mezclan
> mientras no ejecutes `sim_env` en una terminal que controla el robot real.

## Qué se simula y con qué fidelidad

| Real | Simulado (`sim_robot`) | Fidelidad |
|---|---|---|
| Piper (`piper_arm_controller.py`, Jetson) | `/arm/status` (mismo JSON), `/joint_states`, `/arm/joint_states`, `/arm/set_enable`, `/arm/enable`, `/arm/joint_cmd` (perfil coseno, `velocity[0]` = duración, radianes o grados, pinza en `position[6]`), `/arm/joint_trajectory` (interpolación lineal temporizada), `/arm/gripper_cmd`, `/arm/home` | Cinemática exacta (mismo URDF). Sin dinámica, colisiones ni límites de corriente |
| Pan-tilt (`pan_tilt_node.py`) | `/pan_tilt/connected` (latched), `/pan_tilt/joint_states`, `/pan_tilt/set_torque`, `/pan_tilt/target_degrees`, `/pan_tilt/command`; 90 °/s; misma convención (pan + izquierda, tilt + arriba) | Buena |
| Whisper (`whisper_node.py`, modo por pedido) | `/voice/listen_trigger` abre una ventana de 6 s; `/voice/mute`; `/voice/status` (IDLE/LISTENING/PROCESSING/MUTED); `/voice/transcription` sale **al final de la ventana + 1,5 s**, igual que el real. Lo que el invitado dice fuera de la ventana **se pierde** | Reproduce la temporización real (la causa de "I could not hear you"). No simula errores de reconocimiento |
| RealSense D435 (NUC) | `/camera/camera/color/image_raw` (640×480, fx = 615), `aligned_depth_to_color` (16UC1, mm), `camera_info`. Invitado = foto de rostro + cuerpo, posicionada según el pan-tilt | YuNet/SFace/YOLO-pose funcionan sobre la imagen. La postura "sentado" **no** se simula |
| Cámara de muñeca (Orbbec, Jetson) | `/arm_camera/image_raw`: lata magenta (7,5 × 23 cm) proyectada con la pose real del brazo (TF), con la `focal_px` de la calibración | La distancia aparente es geométricamente correcta |
| Nav2 | Acción `navigate_to_pose` (feedback `distance_remaining`; cancelable), `/goal_pose` de RViz, `/amcl_pose`, `/odom`, TF `map→odom→base_footprint`, `/cmd_vel` (teleop) | Va en línea recta a 0,5 m/s, sin planificador ni obstáculos. **Nav2 real no se toca** |
| Hokuyo | `/scan` por *raycasting* sobre el mapa real (`~/mapa_base_athome.yaml`), 270°, 0,5°, 5 m | Bueno para visualizar; el láser sale del centro de la base |
| Agarre | Si la pinza se cierra a < 6 cm del objeto, el objeto sigue al TCP (marcador `/sim/world`) | Solo cinemático |

### Modo Gazebo (`sim_gazebo`)

- **Modelo:** el URDF real (`robot.urdf`, sin modificarlo) más los bloques `<gazebo>` que genera
  `base_athome_sim/gz_model.py`:
  - la base usa `VelocityControl` (sigue `/cmd_vel`) y `OdometryPublisher`;
  - el brazo, la pinza, el pan-tilt y el elevador usan `JointPositionController` por joint;
  - lidar `gpu_lidar` de 270° y 5 m, RealSense `rgbd_camera` y cámara de muñeca.
- **Mundo:** `worlds/casa_simple.sdf`, con habitación, puerta, mesa con la lata y dos sillas.
- **Reparto:** `sim_robot` sigue siendo el "firmware" (Piper, voz, Nav2 falso) y reenvía los
  sensores de gz con los nombres reales.
- **Hora:** `ros_gz_bridge` usa `override_timestamps_with_wall_time`, porque las apps usan reloj
  de pared, no `use_sim_time`.
- **Paquetes:** `ros-jazzy-ros-gz-sim`, `ros-gz-bridge` y `ros-gz-image` ya estaban instalados.
  No hizo falta instalar nada.

### `ATHOME_SIM=1` (lo único que cambia en las apps)

- `face_recognizer`: base de rostros en `~/.local/share/base_athome/sim/faces_database`. No mezcla
  personas reales y no migra la base vieja.
- `hri_receptionist_challenge`: no arranca el driver de la RealSense.
- `arm_manipulation`: usa una copia de la configuración en
  `~/.local/share/base_athome/sim/arm_poses_and_limits.yaml`. Calibrar con [K] en la simulación no
  pisa la del robot.
- `arm_manipulation`: el detector pasa a `SimColorDetector`, que segmenta la lata magenta.
  YOLO no reconoce una lata dibujada.

## Pruebas que se ejecutaron (30-10-2026, salida real)

- `pytest src/base_athome_sim/test`: 6 pasaron. `pytest src/base_athome_perception/test`: 78 pasaron.
- **Sim simple:**
  - cámara de cabeza a 14,8 Hz;
  - `joint_cmd` movió el brazo;
  - lo dicho antes de abrir la ventana quedó "PERDIDO"; lo dicho dentro, "ESCUCHADO";
  - `/voice/transcription` publicó el texto al cerrar la ventana;
  - `navigate_to_pose` terminó en SUCCEEDED y `/amcl_pose` quedó en (0,5; 0,3);
  - `/scan`: 460 de 541 rayos con impacto en el mapa real.
- **Cámaras:**
  - YuNet detectó 1 rostro en la imagen simulada;
  - `SimColorDetector` detectó la lata en PRE-GRASP (pose enseñada real).
- **Gazebo (sin GUI):**
  - `/scan` a 9,6 Hz, RGB y profundidad a 14,8 Hz, `/odom` a 28,7 Hz;
  - `navigate_to_pose` a (−0,5; 0) terminó en SUCCEEDED, con la base física en (−0,43; −0,02);
  - la cámara de muñeca renderiza la lata entre los dedos de la pinza y el detector la encuentra.
- **Receptionist real contra la sim (1 invitado):**
  - puerta, ASR, parseo (Robin / Iced Tea), memorización del rostro (10 muestras), sala y escena;
  - Robin fue reconocido en la sala y el cierre se dijo una sola vez.
- **Modalidad pantalla:**
  - el invitado tocó "Jordan" y "Milk" en `http://127.0.0.1:8090`;
  - el robot tomó y guardó los datos sin usar el micrófono.

## Limitaciones conocidas (qué NO prueba la simulación)

- **Audio real:** ruido, eco del TTS, ganancia del RØDE y errores de Whisper. Para eso hay que
  usar el robot o grabaciones.
- **"Invitado sentado":** no se simula. `WAIT_SEATED` agota su espera de 45 s y continúa. Es
  correcto, pero lento.
- **Navegación:** sin obstáculos ni planificador real. Para probar Nav2 se necesita el robot.
- **Agarre:** sin física. En Gazebo, el objeto no queda tomado físicamente por la pinza.
- **Objeto en la sim simple:** se define en `base_footprint` y por lo tanto se mueve con el robot.
  En Gazebo está fijo en el mundo.
