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
- Calificación: TDP de 8 páginas, video ≤10 min, web en inglés con código abierto. Premio open-source (§1.7.6): candidato el editor URDF.

**Checklist de inspección (§4.4) — pendiente:** E-stop corta base+brazo+elevador+pan-tilt · robot ≤200×70 cm sin cables sueltos · freno ante persona · voz clara y salida de audio para RF · señal de inicio aprobada · ninguna WiFi propia (NUC y Jetson) · sin ArUco visible · dispositivos externos informados.

## 3. Funcionalidades terminadas (detalle en Git y `STATUS.md`)

| Área | Logro | Evidencia |
|---|---|---|
| Plataforma | Jetson migrada a JetPack 7.2.1 / Ubuntu 24.04 / Jazzy; Ethernet por adaptador USB; `ros2 node list` entre hosts | `VERIFICADO 2026-11-03`; commits `5055d4d`, `e6ed289` |
| Seguridad | Piper `auto_enable=False` + guard de feedback CAN; pan-tilt sin torque automático; perfiles sintéticos excluidos | `cdfe8fa`, `7b5c05e`, `ad6a00b` (desplegado) |
| Cámaras | Cámara de muñeca 5→20 Hz (S-12); RealSense con un solo driver | `b5019d1`, `99d3ad5` |
| Percepción | YOLOv8n TensorRT FP16 (4,3 ms) y nodo `yolo_trt_node` (24,5 Hz en JP6) | `0e6899e`, `e3a3b76` — **engine a reconstruir en JP7** |
| ASR/HRI | Escucha por pedido con VAD Silero, fin a 1,1 s, ids y `/voice/result`; `VoiceClient`; vocabulario fonético; pantalla del invitado (`:8090`) | `398f876`, `a8550a3`, `01ecbf7`; manual `docs/MICROFONO_ASR.md` |
| Receptionist | presentación mutua con mirada; centrado de rostro, escaneo, oferta de asiento libre y espera a que se siente; anfitrión elegido entre sentados; cierre único; respaldo de rostro | `f56238d`, `137007f`, `99d3ad5`, `01ecbf7` |
| Misiones | secuenciador (puerta/invitado/objeto/living) con prompt de órdenes y confirmación | `a1a4442`, `a8550a3` |
| Señal de inicio | detector de puerta abierta desde `/scan` | `e37d199` (SIM_PASS) |
| GPSR | parser determinista offline texto→plan JSON, CLI, dataset | `5e656ab` (34 tests) |
| Manipulación | app simple con asistente de enseñanza HOME/PRE/POST, teach panel y `pose_store`, IK con MoveIt 2 (`base_athome_moveit_config`), lectura de pose real, velocidad regulable | `b2b093d`, `0174088`, `49b2a4f`, `88f288a` |
| URDF | `/joint_states` con un dueño por joint; editor Xacro/URDF localhost fase 1 | `c2f2843`, `3a957f1` (45 tests) |
| Mapa semántico | editor de zonas/waypoints con capa de objetos numerados y puntos de aproximación | `7a74239` |
| Simulación | `base_athome_sim` liviana + Gazebo Harmonic | `74d8973` |

## 4. Prioridades activas

Escala: **P0** desbloquea seguridad/progreso · **P1** valor demostrable · **P2** posterior.

### P0 — Seguridad y estabilidad

| ID | Tarea | Quién | Criterio de terminado |
|---|---|---|---|
| HOME-SEC | Revocar/rotar el token de GitHub y sacarlo de la URL de `origin`; borrar `99-jetson-flash-temp` (NUC); acotar `90-ubuntu-nopasswd` (Jetson); decidir sobre `scripts/orca/agy_autoapprove.py`; Orca sin `--dangerously-skip-permissions`/`--yolo` | **Humano** (credenciales/sudo) + Claude verifica | remoto sin credenciales; sudo mínimo; push habilitado |
| HOME-S02 | Desplegar el fallback DDS solo-localhost de la Jetson (S-02) | Claude (Nivel B con checklist) | `jetson-head` sobrevive sin cable |
| HOME-S11 | Consumidores HRI/manipulación verifican `/pan_tilt/connected` y `feedback_ok` | Claude | tests que fallan sin el chequeo |
| HOME-TRT | Reconstruir engines YOLO para TensorRT 10.16 y `cuda-python` CUDA 13; medir FPS | Claude | `/arm_camera/detections` ≥20 Hz medido |

### P1 — Cerrar funcionalidades existentes

| ID | Tarea | Quién | Criterio |
|---|---|---|---|
| HOME-VOZ | Validar el ASR nuevo con personas reales (`escuchar --n 10`, 2–3 personas a 1 y 2 m); grabaciones con ruido para `asr_benchmark --wav-dir`; reducción de ruido solo si la CPU alcanza | Claude + humano | tasa de comprensión y latencia medidas en el robot |
| HOME-06c | Brechas HRI 2026.2 (§5) | Claude + Antigravity | estados nuevos con tests y SIM_PASS |
| HOME-RS | Instalar `realsense-nuc.service` de usuario (Nivel B: revisar unit, sin conflicto de dueño): `cp scripts/systemd/realsense-nuc.service ~/.config/systemd/user/ && systemctl --user daemon-reload && systemctl --user enable --now realsense-nuc` (+ `loginctl enable-linger athome` = Nivel C) | Claude | un solo dueño de la cámara, 15 Hz |
| HOME-15b | GPSR: pasos perdidos en comandos compuestos; dataset con planes esperados (CommandGenerator); métrica de plan exacto | Antigravity | exact-plan accuracy reportada |
| HOME-05 | Editor URDF: prueba visual en navegador, export candidato, colisiones; nunca toca footprint/Nav2 | Antigravity | capturas + tests UI |
| HOME-22b | Mapa semántico: waypoint `home` y objetos numerados medidos por el humano; validación de esquema | humano + Antigravity | YAML validado, sin coordenadas inventadas |
| HOME-16 | CI offline: pytest por paquete, xacro/check_urdf, lint progresivo; arreglar colección GPSR bajo entorno ROS | Antigravity | un comando corre todas las suites |
| HOME-17 | Rutas absolutas fuera de navegación (S-06), borrar `scripts/jetson/` (S-05), entry points (S-10), `except: pass` (S-07) | Claude | tests verdes, sin cambio de interfaces |
| HOME-30 | Auditoría de navegación **solo lectura** con traza motores→driver→odom→TF→AMCL→Nav2→cliente | Claude | `ROBOT.md` §8 completado + NAV-PROP con evidencia; cero archivos de navegación modificados |
| HOME-31 | Investigación de bases hoverboard ROS 2 (5–8 proyectos: licencia, controlador, feedback, watchdog, tests) | Antigravity | tabla en `ROBOT.md` §8 con decisión ADOPTAR/ADAPTAR/NO |

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
| NAV-PROP-001 | ¿Qué hace la placa si se pierde la UART? (el driver tiene watchdog 0,5 s del lado ROS) | POR INVESTIGAR |
| NAV-PROP-002 | Covarianzas de `/odom` y calibración Hall | POR INVESTIGAR |
| NAV-PROP-003 | ¿Un solo dueño de `odom→base_footprint`? | POR INVESTIGAR |
| NAV-PROP-004 | Prioridad entre Nav2 y teleop sobre `/cmd_vel` (¿hay mux?) | POR INVESTIGAR |
| NAV-PROP-005 | Footprint operativo (`robot_radius 0.44`) desacoplado del URDF visual | POR INVESTIGAR |
| NAV-PROP-006 | Efecto de `base_joint z=0.06` (`b2b093d`) sobre la altura de `laser` y el mapa | POR INVESTIGAR |
| NAV-PROP-007 | Rutas absolutas en `nav2_bringup.launch.py` y `nav2_params.yaml` (BT, mapa) | DOCUMENTADO, sin cambiar |

## 7. Plan de validación

- Etiquetas: `STATIC_PASS` → `UNIT_PASS` → `SIM_PASS` → `NUC/JETSON/ETHERNET_VERIFIED` → `HARDWARE_VERIFIED` → `COMPETITION_READY` (solo con ensayos repetidos en escenario equivalente).
- Cada cambio: tests antes/después, baseline si es optimización, rollback.
- KPIs: tasa de tests, FPS de cámaras, latencia ASR y comprensión, CPU/RAM Jetson, exact-plan GPSR, residual hand-eye (mm/°), % éxito pick/place, errores recuperados, incidentes de seguridad.
- Pruebas físicas solo en `LAB_SUPERVISADO` con autorización de la sesión (`AGENTS.md` §3).
- Privacidad: no subir audios, rostros, videos ni datos biométricos al repo.
