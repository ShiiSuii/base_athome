# STATUS.md — Estado vivo de Base@Home

Corto y actualizado: se **reemplaza** lo viejo, no se acumula (`AGENTS.md` §9). Lo histórico se
recupera por commit (§8). Editado por el integrador (Claude).

**Última actualización:** 2026-11-03 · Claude Code · modo `CASA`.

## 1. Estado comprobado (2026-11-03)

| Ítem | Estado | Evidencia |
|---|---|---|
| NUC | Ubuntu 24.04.3, Jazzy, CycloneDDS, dominio 10 | `NUC_VERIFIED` |
| Jetson | Ubuntu 24.04.4, L4T R39.2.1 (JetPack 7.2.1), Jazzy, modo 25W, uptime 3 días | `JETSON_VERIFIED` por SSH |
| Enlace | NUC `enp1s0` 10.42.0.1 ↔ Jetson `enx00e04c68090c` 10.42.0.12 UP; `enP8p1s0` sigue DOWN | `ETHERNET_VERIFIED` |
| `jetson-head` | activo: Piper controller, pan-tilt, Whisper, `stream_orbbec` corriendo | `JETSON_VERIFIED` |
| YOLO TensorRT | **recuperado**: TRT 10.16.2 + CUDA 13.2 runtime instalados; `yolo_trt_node` corriendo; `/arm_camera/detections` a 29,9 Hz en la NUC | `JETSON_VERIFIED`, `ETHERNET_VERIFIED` |
| `realsense-nuc.service` | no instalado | `NUC_VERIFIED` |
| Actuadores | sin confirmar (modo CASA) | — |
| Seguridad de host | URL de `origin` **sin credenciales**; NUC: `99-jetson-flash-temp` **borrado** (sudo con contraseña vía `%sudo`); Jetson: `90-ubuntu-nopasswd` aún presente + regla acotada `91-ubuntu-jetson-head`; `agy_autoapprove.py` eliminado del repo; token sin restos en historial/config locales | `NUC_VERIFIED`/`JETSON_VERIFIED` (sin imprimir secretos) |
| Tests offline | 245 passed: percepción 111, GPSR 34, URDF 45, Jetson scripts 15, ASR 12, semántica 9, skills 8, sim 6, pan-tilt 5 (comandos en `ROBOT.md` §10) | `UNIT_PASS` 2026-11-03 |

## 2. Rama de referencia

- Trabajo integrado: rama `claude/docs-v3-instructions` (contiene todo `claude/home-00-audit` + docs).
  `main` = `1da2388` (sin los ~60 commits del 29/10–03/11). **Nada pusheado**: push bloqueado hasta revocar el token (HOME-SEC).
- Cambios sin commitear del usuario (no tocar): `semantic_map.yaml`, `waypoints.yaml`, `arm_poses_and_limits.yaml`.

## 3. Orca y worktrees

- Orca 1.4.246 con orquestación disponible (`orca orchestration …`). Run `run_8db8fe9ab48b`: HOME-04, 06a, 15a, 20, 21 `completed`; HOME-22 figura `ready` aunque está integrada (`17461ab`) — no se pudo actualizar porque el Run está ligado al terminal de la sesión de coordinación anterior (`consumer_fenced`). Sin workers activos.
- **Nuevo worktree para Antigravity:** `/home/athome/orca/workspaces/base_athome_ws/antigravity-home-15b-gpsr`, rama `antigravity/home-15b-gpsr-compound`, base `7e19cf5` (reglas consolidadas). Tarea prevista: HOME-15b, escribir solo en `tools/gpsr_parser/`. Sin agente lanzado.
- Worktrees de Antigravity (todas sus ramas **ya integradas** en la rama de referencia; limpios):
  `antigravity-home-04-urdf`, `-06a-hri-gap`, `-15a-gpsr`, `-20-joint-states`, `-21-teach-panel` (solo un `teach_panel_preview.png` sin trackear), `-22-map-objects`.
  Usan el `AGENTS.md` viejo de su rama: no despachar más trabajo en ellos (se pueden borrar cuando el usuario quiera).

## 4. Tareas activas

| Quién | Tarea | Estado |
|---|---|---|
| Humano | HOME-SEC: revocar el token en GitHub; confirmar contraseña de `ubuntu` en la Jetson; elegir método de push (`gh auth setup-git` o SSH); Orca sin permisos irrestrictos | pendiente |
| Humano | Decidir la pregunta de confirmación del receptionist en modo competencia (`ROADMAP.md` §5) | pendiente |
| Humano | Medir la columna del torso; ubicar waypoint `home` y objetos en el editor de mapa | pendiente |
| Claude | push + PR de `claude/docs-v3-instructions` (tras revocación); HOME-S02, HOME-S11; optimizar pre/post-proceso YOLO | próximas |
| Antigravity | HOME-15b en su worktree nuevo; luego HOME-05, HOME-16, HOME-31 | sin despachar |

## 5. Bloqueos vigentes

- Push/PR: hasta que el token comprometido esté revocado y haya autenticación segura configurada.
- Pruebas con actuadores y voz con personas reales: requieren `LAB_SUPERVISADO`.
- Piper: 0 tramas CAN la última vez que se energizó (30/10) → revisar alimentación/E-stop/cable CAN.
- Pan-tilt: el U2D2 se desconectó por USB el 30/10 → revisar cable y 12 V.

## 6. Últimos cambios importantes

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

1. Humano: revocar el token y confirmar la contraseña de la Jetson → Claude borra `90-ubuntu-nopasswd`, configura el push y abre el PR.
2. Claude: desplegar fallback DDS (S-02); consumidores de `feedback_ok`/`connected` (S-11).
3. Con el robot energizado y supervisión: `escuchar --n 10` con personas reales; prueba de CAN del Piper.
4. Despachar HOME-15b a Antigravity en `antigravity-home-15b-gpsr` cuando haya un worker disponible.
