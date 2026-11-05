# ROADMAP.md — Desarrollo futuro de Base@Home

Documento maestro de **objetivos, prioridades, tareas pendientes y propuestas**. Nueva tarea → se
agrega aquí. Lo terminado se resume en §3 y su detalle vive en Git. El estado en curso
(quién hace qué hoy) está en `docs/collab/STATUS.md`.

## 1. Objetivo

Competir en **RoboCup@Home OPL** con un robot confiable, medible y mantenible. Reglamento de referencia:
**release 2026.2** (<https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2>, PDF sha256
`d35e93c7…0ce128`). `master` (2027) no es normativo. Antes de una competencia concreta, confirmar qué
reglamento aplica.

## 2. Requisitos de competencia (2026.2)

Una sola liga (OPL). Pruebas: **HRI Challenge, Pick and Place, GPSR, Doing Laundry, Restaurant + Final**.
"Give Me a Hand" y "Help Me Carry" **no existen** en 2026.2. Pruebas regulares de 7:00 min
(Restaurant 15:00 una sola vez, sin mapa previo; Final 10:00). Cada día se corren las mismas 4 pruebas
y cuenta la mejor puntuación.

| Prueba | Máx. | Clave para nosotros |
|---|---|---|
| HRI Challenge (§5.1) | 1450 | receptionist + mirada + bolsa + seguir anfitrión; sin preguntas de confirmación no esenciales |
| Pick and Place (§5.2) | 3515 | la de más puntos; objetos comunes (Red Bull, Pringles, cubo Rubik, Colgate) entrenables ya |
| GPSR (§5.3) | 1490 | entender 3 comandos (3×80) + mostrar plan (3×100) = 540 sin manipular |
| Doing Laundry (§5.4) | 4415 | mayormente bimanual; con un brazo: sacar del lavarropas, levantar prenda, doblez simple |
| Restaurant (§5.5) | 2360 | SLAM online, detectar cliente que saluda, tomar pedido |
| Final (§6) | 8450 | integración (abrir puerta 600, basura, objetos fuera de lugar…) |

**Reglas generales con impacto técnico:**
- §3.3.2: E-stop rojo único que detiene **todo**; sin marcas artificiales visibles (sacar el ArUco); evitar a una persona delante (Nav2, verificar en ensayo).
- §3.2.8/B.3: solo red de la arena; **hotspot propio = descalificación**; enlace NUC↔Jetson cableado.
- §3.4: cómputo externo ≤5 equipos, avisado; el robot debe operar seguro sin ellos. Política propia: nada de APIs pagas.
- §3.6.8: señal de inicio por puerta abierta o señal manual aprobada (sin mouse/teclado); la alternativa cuesta −100.
- §3.6.9/3.6.14: un restart (≤60 s, antes de 40 pts); 30 s quieto = retirado → el secuenciador debe hablar/pedir tiempo.
- §3.7: asistencia humana como último recurso, pedida explícitamente y verificada. §3.7.4: ASR custom −10 %, interfaz alternativa hasta −20 %.
- Skill certificates (§1.7.5): ≥75 % de los puntos de una habilidad en todas las pruebas.
- Calificación 2027 (entrega 6/12/2026): TDP de 8 páginas, video ≤10 min con al menos 5 habilidades integradas, web en inglés con código abierto (`docs/VIDEO_CUALIFICACION.txt`). Premio open-source (§1.7.6): candidato el editor URDF.

**Checklist de inspección (§4.4) — pendiente:** E-stop corta base+brazo+elevador+pan-tilt · robot ≤200×70 cm sin cables sueltos · freno ante persona · voz clara y salida de audio para RF · señal de inicio aprobada · ninguna WiFi propia (NUC y Jetson) · sin ArUco visible · dispositivos externos informados.

## 3. Funcionalidades terminadas (detalle en Git y `STATUS.md`)

| Área | Logro | Evidencia |
|---|---|---|
| Plataforma | Jetson migrada a JetPack 7.2.1 / Ubuntu 24.04 / Jazzy; Ethernet por adaptador USB; `ros2 node list` entre hosts | `VERIFICADO 2026-11-03`; commits `5055d4d`, `e6ed289` |
| Seguridad | Piper `auto_enable=False` + guard de feedback CAN; pan-tilt sin torque automático; perfiles sintéticos excluidos | `cdfe8fa`, `7b5c05e`, `ad6a00b` (desplegado) |
| Cámaras | Cámara de muñeca 5→20 Hz (S-12); RealSense con un solo driver | `b5019d1`, `99d3ad5` |
| Percepción | YOLO TensorRT recuperado en JP7 (TRT 10.16): 55 FPS de punta a punta vs 5,2 en CPU; `/arm_camera/detections` 29,9 Hz en la NUC | 2026-11-03 (ver `STATUS.md`) |
| ASR/HRI | Escucha por pedido con VAD Silero, fin a 1,1 s, ids y `/voice/result`; `VoiceClient`; vocabulario fonético; pantalla del invitado (`:8090`) | `398f876`, `a8550a3`, `01ecbf7`; manual `docs/MICROFONO_ASR.md` |
| Receptionist | presentación mutua con mirada; centrado de rostro, escaneo, oferta de asiento libre y espera a que se siente; anfitrión elegido entre sentados; cierre único; respaldo de rostro | `f56238d`, `137007f`, `99d3ad5`, `01ecbf7` |
| Misiones | secuenciador (puerta/invitado/objeto/living) con prompt de órdenes y confirmación | `a1a4442`, `a8550a3` |
| Señal de inicio | detector de puerta abierta desde `/scan` | `e37d199` (SIM_PASS) |
| GPSR | parser determinista offline texto→plan JSON, CLI, dataset | `5e656ab` (34 tests) |
| Manipulación | app simple con asistente de enseñanza HOME/PRE/POST, teach panel y `pose_store`, IK con MoveIt 2 (`base_athome_moveit_config`), lectura de pose real, velocidad regulable | `b2b093d`, `0174088`, `49b2a4f`, `88f288a` |
| URDF | `/joint_states` con un dueño por joint; editor Xacro/URDF localhost fase 1 | `c2f2843`, `3a957f1` (45 tests) |
| Mapa semántico | editor de zonas/waypoints con capa de objetos numerados y puntos de aproximación | `7a74239` |
| Simulación | `base_athome_sim` liviana + Gazebo Harmonic | `74d8973` |
| Misiones | `mission_executor`: registro de habilidades con estado real, plan validado, reintentos, timeouts, cancelación, recuperación; demo A→B→voz→C→casa | HOME-41, `SIM_PASS` 2026-11-03 |
| Manipulación | diagnóstico Piper (`ROBOT.md` §5.1), controlador endurecido (S-14, sin desplegar), núcleo hand-eye (§5.2) | HOME-41 (`UNIT_PASS`) |

## 4. Prioridades activas

Escala: **P0** desbloquea seguridad/progreso · **P1** valor demostrable · **P2** posterior.

### P0 — Seguridad y estabilidad

| ID | Tarea | Quién | Criterio de terminado |
|---|---|---|---|
| HOME-SEC | Hecho: URL de `origin` sin credenciales, sudo temporal de la NUC borrado, regla acotada en la Jetson, auto-aprobador eliminado. **Falta (humano):** revocar el token en GitHub; confirmar la contraseña de `ubuntu` en la Jetson para borrar `90-ubuntu-nopasswd`; Orca sin `--dangerously-skip-permissions`/`--yolo` | Humano + Claude | push habilitado; sudo mínimo |
| HOME-S02 | Desplegar el fallback DDS solo-localhost de la Jetson (S-02) | Claude (nivel A con checklist) | `jetson-head` sobrevive sin cable |
| HOME-S11 | Consumidores HRI/manipulación verifican `/pan_tilt/connected` y `feedback_ok` | Claude | tests que fallan sin el chequeo |
| HOME-41d | ~~Desplegar el controlador corregido~~ **hecho 2026-11-03** (md5 `2e347c49`, FJT, rampa de pinza) | Claude | `JETSON_VERIFIED` |
| HOME-43 | Estado medido del elevador (homing + pasos de `stepper_controller.py` → `/torso/joint_states`); quitar `torso_lift_joint=0` sintético y `SHOULDER_IN_BASE` fijo (S-15) | Claude | TF del brazo sigue al elevador en sim; nada publicado sin medición |

### P1 — Cerrar funcionalidades existentes

| ID | Tarea | Quién | Criterio |
|---|---|---|---|
| HOME-VOZ | Validar el ASR nuevo con personas reales (`escuchar --n 10`, 2–3 personas a 1 y 2 m); grabaciones con ruido para `asr_benchmark --wav-dir`; reducción de ruido solo si la CPU alcanza | Claude + humano | tasa de comprensión y latencia medidas en el robot |
| HOME-06c | Brechas HRI 2026.2 (§5) | Claude + Antigravity | estados nuevos con tests y SIM_PASS |
| HOME-RS | Instalar `realsense-nuc.service` de usuario (nivel A con checklist: revisar unit, sin conflicto de dueño): `cp scripts/systemd/realsense-nuc.service ~/.config/systemd/user/ && systemctl --user daemon-reload && systemctl --user enable --now realsense-nuc` (+ `loginctl enable-linger athome`, con rollback `disable-linger`) | Claude | un solo dueño de la cámara, 15 Hz |
| HOME-15b | GPSR: pasos perdidos en comandos compuestos; dataset con planes esperados (CommandGenerator); métrica de plan exacto | Antigravity | exact-plan accuracy reportada |
| HOME-05 | Editor URDF: prueba visual en navegador, export candidato, colisiones; nunca toca footprint/Nav2 | Antigravity | capturas + tests UI |
| HOME-22b | Mapa semántico: waypoint `home` y objetos numerados medidos por el humano; validación de esquema | humano + Antigravity | YAML validado, sin coordenadas inventadas |
| HOME-16 | CI offline: pytest por paquete, xacro/check_urdf, lint progresivo; arreglar colección GPSR bajo entorno ROS | Antigravity | un comando corre todas las suites |
| HOME-17 | Rutas absolutas fuera de navegación (S-06), borrar `scripts/jetson/` (S-05), entry points (S-10), `except: pass` (S-07) | Claude | tests verdes, sin cambio de interfaces |
| HOME-30 | Auditoría de navegación **solo lectura** con traza motores→driver→odom→TF→AMCL→Nav2→cliente | Claude | `ROBOT.md` §8 completado + NAV-PROP con evidencia; cero archivos de navegación modificados |
| HOME-31 | Investigación de bases hoverboard ROS 2 (5–8 proyectos: licencia, controlador, feedback, watchdog, tests) | Antigravity | tabla en `ROBOT.md` §8 con decisión ADOPTAR/ADAPTAR/NO |

### Propuestas de mejora (investigación 2026-11-03; no implementadas)

| Área | Propuesta | Beneficio | Costo/riesgo |
|---|---|---|---|
| Percepción | YOLO: pre/post-proceso en GPU o `cv2.dnn.blobFromImage` + NMS de OpenCV; hoy 12 de 18 ms son numpy en CPU | ~2× FPS o la mitad de CPU | bajo |
| Percepción | Fine-tuning de YOLOv8n con los objetos del reglamento (Pringles, Red Bull, Rubik, Colgate…) y export a TRT | habilita Pick and Place | medio (dataset propio) |
| Percepción | YOLO de la cabeza (D435) en la Jetson enviando JPEG comprimido | libera CPU de la NUC (38 ms/frame) | medio (ancho de banda) |
| HRI | Whisper en GPU: ctranslate2 con CUDA 13 (o whisper TRT) para `small.en` a la latencia actual de `base.en` | menos errores de ASR | medio (compilar) |
| HRI | Mirada continua: pan-tilt a 50 Hz con `GroupSyncRead` y filtro de seguimiento | puntos de "mirar al que habla" | bajo |
| Manipulación | `OrbbecSDK_ROS2` para profundidad registrada + estrategia de pose de observación 35–45 cm | agarres 3D reales | medio |
| Manipulación | `agx_arm_ros` + MoveIt con colisión del torso medido; fachada `/arm/*` | planificación segura | alto |
| Base | NAV-PROP-001 (feedback stale) y NAV-PROP-004 (mux) son las de mejor relación seguridad/costo | seguridad | requieren autorización (navegación) |
| Hardware | NVMe para la Jetson; `MAXN_SUPER` solo con fuente verificada; E-stop que corte todos los dominios | robustez, reglamento | decisiones humanas |

### Manipulación y misiones (orden de ejecución)

| ID | Tarea | Quién | Criterio |
|---|---|---|---|
| HOME-42 | LAB: leer límites y versión del firmware del Piper; comparar con URDF/ficha (S-16); medir el montaje (`ROBOT.md` §5.1) | humano + Claude | tabla en §5.1; poses fuera de límite señaladas |
| HOME-44 | Calibración intrínseca + hand-eye según `ROBOT.md` §5.2 (herramienta de captura sobre `handeye.py`) | Claude; LAB con humano | residual ≤ 5 mm/1°, validación ≤ 10 mm |
| HOME-45 | ~~Servidor `FollowJointTrajectory`~~ **hecho** (8 tests, SIM_PASS, desplegado) | Claude | — |
| HOME-50 | Probar con el brazo real (LAB): agarre a 10, 15, 20 cm con objetivo retenido; registrar % de éxito | humano + Claude | serie de 10 por distancia |
| HOME-51 | Pose de observación más retraída o reposicionar la base antes del agarre para objetos > 25 cm | Claude | SIM_PASS a 30 cm |
| HOME-46 | Escena MoveIt: ~~pares internos por muestreo~~ (hecho), **falta mesa** como objeto de colisión | Claude | IK rechaza poses que atraviesan la mesa en sim |
| HOME-47 | Skills `detect_object` (3D), `pick_object`/`place_object` con confirmación por ancho medido de la pinza; recién entonces pasan a `available` | Claude | SIM_PASS → serie LAB con % de éxito |
| HOME-48 | Evaluar MoveIt Task Constructor (rama `ros2`, compilar en Jazzy; no instalado) cuando exista HOME-45 | Claude | decisión ADOPTAR/NO con prueba en sim |
| HOME-57 | Commander: pedido en lenguaje natural ES/EN → plan validado → ejecutor, con web local (reemplaza HOME-40) y objetos/mesas del mapa. Hecho en sim; falta probarlo con Nav2 real y sumar skills de personas (HOME-49) | Claude | `SIM_PASS` 2026-11-04; real `PENDIENTE` |
| HOME-58 | Docking a la mesa: al llegar, acercar la base hasta dejar el objeto al alcance del brazo. Nav2 deja el centro del robot a ≥0,44 m del mueble (tolerancia 0,18 m), así que objetos lejos del borde quedan fuera de alcance. Sin modificar Nav2 | Claude | pendiente; pedido por el usuario para después |
| HOME-59 | Detector de objetos. `yolov8n` COCO no ve `can` ni `box`, y `pick` no distingue dos objetos de la misma clase en la misma mesa. Prueba 2026-11-04 con las fotos del RuleBook: `yolov8s-worldv2` con clases por texto da los mejores resultados (lata y botellas), pero no separa Pringles de Red Bull ni ve Colgate o el cubo; `yolov8n-oiv7` ve `Tin can` y pierde las botellas. Siguiente: exportar World a TensorRT y medir con objetos reales. Ojo: `want in label` también matchea `Candle` con `can`. Los objetos definitivos se publican en competencia: hace falta un flujo de entrenamiento rápido | Claude | pendiente; falta probar en Jetson |
| HOME-60 | `guide_person` (HRI, GPSR): el RuleBook 2026.2 no pide detectar que el juez se detiene ni lo puntúa. Hacer versión simple: pausas para mirar atrás y avisar, sin quedar 30 s quieto y mirando hacia adelante al navegar. Prioridad mayor: no perder al anfitrión al seguirlo (`-50`/`-150`) | Claude | pendiente |
| HOME-49 | Migrar pasos de `mission_sequencer` (receive_guest, fetch, guide, place) a skills del ejecutor; plan GPSR → ejecutor (depende de HOME-15b: el parser pierde pasos) | Claude | un solo sistema de misiones |

### Teleoperación y HRI (2026-11-03)

| ID | Tarea | Quién | Criterio |
|---|---|---|---|
| HOME-52 | Teleop Quest en el brazo real: prueba completa (HOME → seguimiento → pinza), medir `track_mm`/`track_deg` y ajustar `scale`, `smooth`, velocidades | humano + Claude | error medio < 10 mm a 12°/s |
| HOME-56 | LAB: verificar `installation_pos` del Piper (montaje lateral; confirmar lado con el cable hacia atrás) y medir la lag real de firmware con `track_mm` (`stream_speed_pct`) | humano + Claude | seguimiento y sostén medidos antes/después |
| HOME-53 | Grabación de demostraciones (`ros2 bag` de estado, comandos, pose objetivo, cámara de muñeca) para entrenar | Claude | episodios reproducibles, formato documentado |
| HOME-54 | Pan-tilt: reparar la alimentación (12 V bajo carga, power hub U2D2, masa) y volver a lanzar el nodo desde `jetson-head` | humano + Claude | 0 desconexiones en 10 min con torque y movimiento |
| HOME-61 | Calibrador `calibrar_pantilt` usado en el robot: la prueba completa (`M`) mide enlace, sentido, error y retardo. Requiere HOME-54: el 04/11 hubo 118 desconexiones USB del U2D2 (`Cannot enable. Maybe the USB cable is bad?`, error -71, solo el puerto del U2D2) | Humano + Claude | script listo; el usuario lo corrió con torque. Falta `V` (sentido con la cámara) y `M`. `pan_direction` pasó a +1 el 04/11 por observación del usuario; `V` lo confirma |
| HOME-62 | **Cualificación 2027: TDP, video y web antes del 6/12/2026** (Nuremberg, 15–21/6/2027). Plan en `docs/VIDEO_CUALIFICACION.txt`. Falta: corrida real completa del receptionist, decidir bolsa y manipulación, consultar a la organización, datos del equipo | Humano + Claude | pendiente, prioridad máxima |
| HOME-63 | Sacar la basura: gesto + cabeza para ubicar + pose de piso (`ensenar_pregrasp`) + cámara de muñeca para agarrar + elevador + punto `trash_drop`. Detector YOLO-World: 17/24 fotos, sin probar con la cámara real. Falta en real: elevador (sin prueba física), montaje real de la cámara de muñeca (hand-eye `PENDIENTE`), tramo final con `/cmd_vel`, ancho al llevar la bolsa (Nav2 no lo sabe: riesgo en puertas). Bajar los pesos `models/yolov8s-worldv2.pt` (no versionados) | Humano + Claude | `SIM_PASS` 2026-11-05 |
| HOME-64 | Pasar el robot a aprendizaje por refuerzo: requiere el modelo con medidas reales (base 22 cm hecha; falta `laser_offset_z`: el modelo da 24 cm del piso, dentro de la base, y el lidar real está sobre 32 cm; parámetro protegido, requiere autorización y la altura del haz) | Claude, cuando estén las medidas | pendiente |
| HOME-55 | Receptionist en el robot: barrido de 2 filas (~16 s), seguimiento leve del rostro, revisión del pan-tilt al arrancar | humano + Claude | persona sentada detectada; revisión ✔ |

### P2 — Manipulación y competencia

| ID | Tarea | Notas |
|---|---|---|
| HOME-08/09 | Calibración hand-eye ArUco con residuales; transformación base→brazo→cámara verificada; colisiones MoveIt con medidas reales del torso | requiere medida humana de la columna y sesión LAB |
| HOME-PICK | Pick and Place: pose de observación 35–45 cm, aproximación final, confirmación por pinza; tasas de éxito por serie; migrar a `agx_arm_ros` + MoveIt manteniendo `/arm/*` como fachada; `OrbbecSDK_ROS2` para profundidad | priorizar por puntos |
| HOME-HEAD | Pan-tilt a 50 Hz con `GroupSyncRead` para mirada continua | Jetson |
| HOME-13 | Doing Laundry con un brazo: estudio de factibilidad | reemplaza "Give Me a Hand" |
| HOME-REST | Restaurant: SLAM online sin tocar la config operativa, detección de saludo | evaluar en simulación |
| HOME-COMP | Matriz regla→habilidad→test→resultado; ensayos completos con fallbacks y métricas | ambos |
| HOME-INFRA | NVMe para la Jetson; `MAXN_SUPER` si la fuente lo soporta; chrony NUC→Jetson | decisiones humanas |

## 5. HRI Challenge: brechas vigentes

Estado del receptionist frente a la hoja de puntaje §5.1 (análisis HOME-06a, actualizado con lo hecho):

| Ítem | Puntos | Estado |
|---|---|---|
| Mirar a quien habla | 2×50 | hecho (centrado suave; tracking fuerte desactivado por defecto) |
| Ofrecer asiento libre verificado | 2×100 | hecho (ocupación por YOLO-pose) — validar en robot |
| Presentación con nombre y bebida, mirando al invitado correcto | 4×30 + 2×50 | hecho (`f56238d`) — validar |
| Mirar hacia donde navega | 2×15 | parcial (cabeza fija en giros) |
| **No hacer preguntas de confirmación no esenciales** | 4×15 (y −40 por dato mal memorizado) | **CONFLICTO:** desde `a8550a3` el receptionist pregunta "Is that correct?". Decidir: quitar la pregunta en modo competencia y confiar en vocabulario+fonética, o aceptar perder hasta 60 pts para evitar −160. **Decisión humana pendiente.** |
| Timbre | 2×30 | no hecho (YAMNet, reimplementar) |
| Atributo visual del 1.er invitado | 4×20 (−20 si yerra) | no hecho |
| Recibir bolsa (handover) · seguir al anfitrión · soltar bolsa | 50 + 200 + 50 | no hecho (follow como cliente de Nav2, modo seguro de brazo) |
| Abrir puerta | 2×200 | no; pedir puerta abierta al árbitro |
| Watchdog global de 7 min e inactividad 30 s | — | parcial |
| Interfaz táctil | hasta −20 % | usar solo como respaldo tras 2 fallos de ASR |

## 6. Propuestas de navegación (registro sin implementación)

La navegación funciona y **no se modifica** sin autorización específica del usuario (`AGENTS.md` §4).
Cada propuesta: estado (IDEA/ANALIZADA/SIMULADA/DESCARTADA/APROBADA), evidencia con archivo:línea o
comando pasivo, riesgo, alternativa, plan de simulación, rollback, decisión del usuario (PENDIENTE por defecto).

| ID | Pregunta | Estado |
|---|---|---|
| NAV-PROP-001 | Pérdida de comunicación. Lado ROS: watchdog `cmd_vel_timeout` 0,5 s → manda 0/0 (`hoverboard_driver.py:312`). Lado placa: config Lenzod `TIMEOUT 20` comandos perdidos → apagado de emergencia (`firmware_config/lenzod_red_stm32_usart3_odometry/config.h:35`; binario en la placa sin verificar). **Hueco:** si se corta la telemetría pero no el TX, `/odom` sigue publicándose con sello actual, la última velocidad y la pose congelada; `rx_age` solo se usa para un log. Propuesta: dejar de publicar (o marcar) `/odom` y TF si `rx_age` > 0,2 s y exponer `/hoverboard/feedback_ok`. Riesgo bajo, test con puerto serie simulado | ANALIZADA (2026-11-03) |
| NAV-PROP-002 | Covarianzas constantes (pose xy 0,001, yaw 0,03; twist 0,002/0,05), no crecen con la distancia. AMCL no las usa; importarían si se agrega EKF/IMU. Sello de tiempo = momento de publicación (hasta 20 ms después del paquete). Integración RK2, 90 ticks/vuelta → 5,95 mm por tick | ANALIZADA — sin acción mientras no haya EKF |
| NAV-PROP-003 | `odom→base_footprint`: único emisor `hoverboard_driver` (`publish_tf: true`); `slam_3d_mapping` usa `publish_tf_map: false`. Lock `/tmp/hoverboard_driver.lock` evita dos drivers. Verificar en vivo con `tf2_echo` cuando la base esté encendida | ANALIZADA (código) |
| NAV-PROP-004 | **Sin multiplexor:** `controller_server`, `behavior_server` y `teleop_twist_joy` publican todos en `/cmd_vel` (`nav2_bringup.launch.py:139,149`, teleop con hombre muerto R1). Si alguien usa el joystick durante un goal, gana el último mensaje. Propuesta: `twist_mux` con prioridad teleop > Nav2 y tópico de bloqueo (cambio de navegación: requiere autorización) | ANALIZADA — decisión del usuario |
| NAV-PROP-005 | Footprint operativo (`robot_radius 0.44`) desacoplado del URDF visual | POR INVESTIGAR |
| NAV-PROP-006 | Efecto de `base_joint z=0.06` (`b2b093d`) sobre la altura de `laser` y el mapa | POR INVESTIGAR |
| NAV-PROP-007 | Rutas absolutas en `nav2_bringup.launch.py` y `nav2_params.yaml` (BT, mapa) | DOCUMENTADO, sin cambiar |
| NAV-PROP-008 | La placa corre FOC en `VLT_MODE` (tensión, sin lazo de velocidad); por eso el driver necesita pisos anti-stiction (`min_speed_raw 75`, `min_steer_raw 165`) y la respuesta depende de batería y carga. `SPD_MODE` daría velocidad en lazo cerrado en la placa. Requiere reflashear la base (Nivel C) y re-tunear Nav2: solo evaluar en banco | IDEA |
| NAV-PROP-009 | `firmware/firmware/HoverboardController/HoverboardController.ino` habla un protocolo de texto (`TEL …`, timeout 500 ms) distinto del binario `0xABCD` que usa el driver: no es el firmware de la base actual. Documentar qué binario está en la placa (probable `firmware_bin/hover_lenzod_usart3_odometry.bin`) | ANALIZADA |

## 7. Plan de validación

- Etiquetas: `STATIC_PASS` → `UNIT_PASS` → `SIM_PASS` → `NUC/JETSON/ETHERNET_VERIFIED` → `HARDWARE_VERIFIED` → `COMPETITION_READY` (solo con ensayos repetidos en escenario equivalente).
- Cada cambio: tests antes/después, baseline si es optimización, rollback.
- KPIs: tasa de tests, FPS de cámaras, latencia ASR y comprensión, CPU/RAM Jetson, exact-plan GPSR, residual hand-eye (mm/°), % éxito pick/place, errores recuperados, incidentes de seguridad.
- Pruebas físicas solo en `LAB_SUPERVISADO` con autorización de la sesión (`AGENTS.md` §3).
- Privacidad: no subir audios, rostros, videos ni datos biométricos al repo.
