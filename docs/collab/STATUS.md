# STATUS.md — Estado vivo de Base@Home

Corto y actualizado: se **reemplaza** lo viejo, no se acumula (`AGENTS.md` §9). Lo histórico se
recupera por commit (§8). Editado por el integrador (Claude).

**Última actualización:** 2026-11-03 (tarde) · Claude Code · modo `CASA` · bloque HOME-41.

## 1. Estado comprobado (2026-11-03)

| Ítem | Estado | Evidencia |
|---|---|---|
| NUC | Ubuntu 24.04.3, Jazzy, CycloneDDS, dominio 10 | `NUC_VERIFIED` |
| Jetson | Ubuntu 24.04.4, L4T R39.2.1 (JetPack 7.2.1), Jazzy, modo 25W, uptime 3 días | `JETSON_VERIFIED` por SSH |
| Enlace | NUC `enp1s0` 10.42.0.1 ↔ Jetson `enx00e04c68090c` 10.42.0.12 UP; `enP8p1s0` sigue DOWN | `ETHERNET_VERIFIED` |
| `jetson-head` | activo: Piper controller, pan-tilt, Whisper, `stream_orbbec`, YOLO corriendo; controlador desplegado = `226e233` (md5 igual al repo **antes** de HOME-41) | `JETSON_VERIFIED` |
| Piper | sin alimentación: adaptador gs_usb hoy en **`can0`**, 0 tramas RX, `feedback_ok=false`, `/arm/joint_states` no se publica (correcto) | `JETSON_VERIFIED` (pasivo) |
| YOLO TensorRT | **recuperado**: TRT 10.16.2 + CUDA 13.2 runtime instalados; `yolo_trt_node` corriendo; `/arm_camera/detections` a 29,9 Hz en la NUC | `JETSON_VERIFIED`, `ETHERNET_VERIFIED` |
| `realsense-nuc.service` | no instalado | `NUC_VERIFIED` |
| Actuadores | sin confirmar (modo CASA) | — |
| Seguridad de host | URL de `origin` **sin credenciales**; NUC: `99-jetson-flash-temp` **borrado** (sudo con contraseña vía `%sudo`); Jetson: `90-ubuntu-nopasswd` aún presente + regla acotada `91-ubuntu-jetson-head`; `agy_autoapprove.py` eliminado del repo; token sin restos en historial/config de git, **pero sí en el registro del repo de Orca** (§3) | `NUC_VERIFIED`/`JETSON_VERIFIED` (sin imprimir secretos) |
| Tests offline | 279 passed: percepción 116, URDF 45, GPSR 34, skills 29, Jetson scripts 24, ASR 12, semántica 9, sim 6, pan-tilt 5 (comandos en `ROBOT.md` §10; linters ament siguen fallando, S-08) | `UNIT_PASS` 2026-11-03 |
| Misiones | demo A→B→voz→C→casa en `base_athome_sim` 41,4 s; timeout+reintento+recuperación; cancelación con goal cancelado y base quieta; guardas de dominio/modo | `SIM_PASS` 2026-11-03 |

## 2. Rama de referencia

- Trabajo en curso: rama `claude/home-41-piper-mission` (base `226e233` = `claude/docs-v3-instructions`).
  `main` = `1da2388`. **Nada pusheado**: push bloqueado hasta revocar el token (HOME-SEC).
- Cambios sin commitear del usuario (no tocar): `semantic_map.yaml`, `waypoints.yaml`, `arm_poses_and_limits.yaml`.

## 3. Orca y worktrees

- Orca 1.4.246. **Run `run_daadf85f0e7a`**: HOME-40 (monitor web de misiones) despachada a Antigravity — task
  `task_3b07fc6ec821`, dispatch `ctx_83329ca3042c`, worktree `antigravity-home-40-mission-monitor`, base rama
  `claude/docs-v3-instructions`, escritura solo en `tools/mission_monitor/`. Estado: **`BLOCKED_WORKER`**, frenado en un
  prompt de permisos de su propia TUI (`git status`); lo tiene que aprobar el humano en esa pestaña o configurar
  permisos acotados (Orca → Settings → Agents). Claude no aprueba prompts ajenos (`AGENTS.md` §6.8).
- **Seguridad:** el registro del repo en Orca (`orca repo list --json`, campo `gitRemoteIdentity.remoteUrl`) **todavía
  contiene la URL vieja con el token**. Git ya está limpio; Orca no. Revocar el token resuelve el riesgo; después
  actualizar o re-importar el repo en Orca.
- Run anterior `run_8db8fe9ab48b` (HOME-04…22) cerrado de hecho; HOME-22 figura `ready` pero está integrada (`17461ab`).
- Worktree `antigravity-home-15b-gpsr` (base `7e19cf5`) sigue sin agente: HOME-15b pendiente.
- Worktrees de Antigravity (todas sus ramas **ya integradas** en la rama de referencia; limpios):
  `antigravity-home-04-urdf`, `-06a-hri-gap`, `-15a-gpsr`, `-20-joint-states`, `-21-teach-panel` (solo un `teach_panel_preview.png` sin trackear), `-22-map-objects`.
  Usan el `AGENTS.md` viejo de su rama: no despachar más trabajo en ellos (se pueden borrar cuando el usuario quiera).

## 4. Tareas activas

| Quién | Tarea | Estado |
|---|---|---|
| Humano | HOME-SEC: revocar el token en GitHub; confirmar contraseña de `ubuntu` en la Jetson; elegir método de push (`gh auth setup-git` o SSH); Orca sin permisos irrestrictos | pendiente |
| Humano | Decidir la pregunta de confirmación del receptionist en modo competencia (`ROADMAP.md` §5) | pendiente |
| Humano | Medir la columna del torso; ubicar waypoint `home` y objetos en el editor de mapa | pendiente |
| Humano | Gate para desplegar el controlador corregido (HOME-41d); medir montaje del Piper y elevador; agregar `kitchen`/`home` al mapa semántico | pendiente |
| Claude | HOME-43 (estado del elevador), HOME-45 (FollowJointTrajectory), HOME-46 (escena MoveIt), revisar HOME-40 | próximas |
| Antigravity | HOME-40 en curso (bloqueado en permisos); luego HOME-15b, HOME-16 | `BLOCKED_WORKER` |

## 5. Bloqueos vigentes

- Push/PR: hasta que el token comprometido esté revocado y haya autenticación segura configurada.
- Pruebas con actuadores y voz con personas reales: requieren `LAB_SUPERVISADO`.
- Piper: 0 tramas CAN (30/10 y hoy, sin alimentación). Límites del firmware desconocidos (S-16): no comandar poses con j2 > 180° hasta leerlos.
- Manipulación con el elevador fuera del tope superior: geometría inválida hasta HOME-43 (S-15).
- Pan-tilt: el U2D2 se desconectó por USB el 30/10 → revisar cable y 12 V.

## 6. Últimos cambios importantes

- 2026-11-03 — **HOME-41** (rama `claude/home-41-piper-mission`, **no desplegado**):
  - Diagnóstico del Piper en `ROBOT.md` §5.1 (montaje lateral, FK DH≡URDF 0,15 mm, límites, TCP, cámara, MoveIt, oficial `agx_arm_ros`) y hallazgos S-14…S-18.
  - `piper_arm_controller.py`: rechaza movimientos deshabilitado o sin feedback y fuera de límites; borra trayectorias al habilitar/deshabilitar; aborta y sostiene lo medido si se pierde el feedback; servicios responden `success=false` si no pueden; `/arm/current_pose` en `arm_base_link` y solo con feedback; pinza hasta 105 mm (10 tests nuevos). Rollback: `git revert` del commit; la Jetson sigue con la versión anterior.
  - `base_athome_skills/mission`: registro de habilidades, planes con `recovery`, ejecutor y nodo `mission_executor` (21 tests, SIM_PASS); `missions/demo_a_b_voice_c_home.yaml`. Con el mapa semántico real la demo se **rechaza** (faltan `kitchen` y `home`).
  - `base_athome_sim`: un goal nuevo ya no hereda la cancelación del anterior (carrera encontrada en la prueba).
  - `base_athome_perception/handeye.py` + procedimiento `ROBOT.md` §5.2 (5 tests sintéticos).

- 2026-11-03 — YOLO TensorRT recuperado en JetPack 7 (baseline vs después, misma imagen `bus.jpg`, `bench_yolo.py`, con `jetson-head` corriendo):
  | | Inferencia (media) | Punta a punta | FPS |
  |---|---|---|---|
  | Antes: onnxruntime CPU | 177 ms | 194 ms | 5,2 |
  | Después: TensorRT FP16 GPU | 6,1 ms (`trtexec` 4,25 ms) | 18,0 ms | 55,4 |
  Detecciones iguales a la referencia ONNX (5/5, score ±0,001, cajas ±9 px). En el robot: 28,6 Hz (límite de la cámara), 54 % de un núcleo, 0,6 GB RAM, ~56 °C. Rollback: `apt-get remove` de `~/deploy_backups/trt_20261103/installed.txt`; script anterior en `~/deploy_backups/yolo_venv_20261103/`.
- 2026-11-03 — HOME-SEC parcial (ver §1); `scripts/jetson/` duplicado y `scripts/orca/agy_autoapprove.py` eliminados; auditoría de navegación solo lectura → NAV-PROP-001..009 en `ROADMAP.md` §6.
- 2026-11-03 — Documentación consolidada en 6 documentos maestros; política "no más Markdown por tarea".
- 2026-11-03 — `a8550a3` ASR rediseñado (VAD streaming, ids, `/voice/result`, `VoiceClient`, confirmación sí/no). Desplegado en la Jetson; respaldo `~/deploy_backups/audio_asr_20261103_094455`. Medido con voz inyectada: 16/16 hasta 8 % de volumen, mediana 2,5 s. **No probado con personas reales.**
- 2026-10-30 — Simulación (`74d8973`), pantalla del invitado (`01ecbf7`), manipulación con MoveIt y teach (`49b2a4f`, `b2b093d`), migración de la Jetson a Jazzy (`5055d4d`).

## 7. Decisiones técnicas vigentes

| Fecha | Decisión |
|---|---|
| previa, ratificada 03/11 | Navegación: estudiar y proponer sí, modificar no. Capa semántica editable como candidato. |
| previa | Solo ROS 2; sin Google ni pagos; modo `CASA` por defecto; `LAB_SUPERVISADO` solo con confirmación en la sesión. |
| 30/10 | Jetson en JetPack 7.2.1/Jazzy; **no repetir la migración**. Enlace por adaptador USB. |
| 30/10 | Asignación de cómputo: nodo junto a su periférico; inferencia pesada en la GPU de la Jetson (`ROBOT.md` §1). |
| 30/10 | Whisper `base.en` + `en` por defecto (6 % vs 20 % WER de `tiny`); `tiny`+`auto` solo para demos en español. |
| 30/10 | Un dueño por joint en `/joint_states`; poses comandadas en `/arm/joint_states_cmd`. |
| 30/10 | La pantalla táctil es respaldo del ASR, no modo principal (penaliza hasta −20 %). |
| 03/11 | Claude coordina e integra; Antigravity en tareas acotadas; si un worker cae, Claude retoma. |
| 03/11 | Documentación: 6 maestros; sin HANDOFF/TASKS/archive; historia en Git. |

## 8. Recuperar historial

| Qué | Dónde |
|---|---|
| Handoffs detallados 29/10–03/11, tabla de tareas HOME-00..34 | `git show ac9b069:docs/collab/HANDOFF.md`, `git show ac9b069:docs/collab/TASKS.md` |
| Auditoría inicial, inventario Jetson JP6, topología de red, barrido del 30/10, optimización de hardware, análisis del reglamento y de brechas HRI, catálogo de repos | `git show ac9b069:docs/<archivo>.md` (`software-audit`, `jetson-inventory`, `network-topology`, `BARRIDO_SISTEMA_2026-10-30`, `HARDWARE_OPTIMIZACION`, `RULEBOOK_2026_2_ANALISIS`, `hri_challenge_2026_gap_analysis`, `REPOS_REUTILIZABLES`) |
| Runbooks de migración Jetson y cable Ethernet | `git show ac9b069:docs/RUNBOOK_MIGRACION_JETSON.md`, `…/RUNBOOK_CABLE_ETHERNET.md` |
| Instrucciones v1/v2/v3 completas | `git show 5f49ec2:AGENTS.md`, `git show ac9b069:AGENTS.md` |
| Respaldo en disco de todos los Markdown previos | `~/baseathome_docs_backup_20261103/all_md_pre_consolidation_ac9b069.tgz` |

## 9. Próximas acciones

1. Humano: revocar el token (también está en el registro de Orca) y confirmar la contraseña de la Jetson.
2. Humano: aprobar permisos del worker de Antigravity (HOME-40) en su pestaña de Orca.
3. Humano + Claude: gate de despliegue del controlador (HOME-41d); luego, en LAB, CAN + límites del firmware (HOME-42).
4. Claude: HOME-43 (elevador medido) → HOME-45 (FollowJointTrajectory) → HOME-46 (escena MoveIt) → HOME-44 (hand-eye en LAB).
5. Humano: medir y cargar `kitchen` y `home` en el editor de mapa para correr la demo de misiones en el robot (con supervisión).
