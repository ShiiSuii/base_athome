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
| YOLO TensorRT | **no corre**: solo hay `.onnx` en `~/models/yolo/` (engine sin reconstruir tras JP7) | `JETSON_VERIFIED` |
| `realsense-nuc.service` | no instalado | `NUC_VERIFIED` |
| Actuadores | sin confirmar (modo CASA) | — |
| Seguridad de host | token en URL de `origin`: **sí**; `99-jetson-flash-temp` (NUC) y `90-ubuntu-nopasswd` (Jetson): **presentes**; `agy_autoapprove.py` existe, no corre | `NUC_VERIFIED`/`JETSON_VERIFIED` (sin imprimir secretos) |
| Tests offline | 243 passed: percepción 111, GPSR 34, URDF 45, Jetson scripts 13, ASR 12, semántica 9, skills 8, sim 6, pan-tilt 5 (comandos en `ROBOT.md` §10) | `UNIT_PASS` 2026-11-03 |

## 2. Rama de referencia

- Trabajo integrado: rama `claude/docs-v3-instructions` (contiene todo `claude/home-00-audit` + docs).
  `main` = `1da2388` (sin los ~60 commits del 29/10–03/11). **Nada pusheado**: push bloqueado hasta revocar el token (HOME-SEC).
- Cambios sin commitear del usuario (no tocar): `semantic_map.yaml`, `waypoints.yaml`, `arm_poses_and_limits.yaml`.

## 3. Orca y worktrees

- Orca CLI/app 1.4.246 en ejecución; **sin pestañas de agentes activas**; los comandos `run`/`task` no existen en esta CLI (orquestación no habilitada) → no hay tareas despachadas hoy.
- Worktrees de Antigravity (todas sus ramas **ya integradas** en la rama de referencia; limpios):
  `antigravity-home-04-urdf`, `-06a-hri-gap`, `-15a-gpsr`, `-20-joint-states`, `-21-teach-panel` (solo un `teach_panel_preview.png` sin trackear), `-22-map-objects`.
  Usan el `AGENTS.md` viejo de su rama: **antes de despachar, crear un worktree nuevo desde la rama de referencia** para que Antigravity lea estas reglas.

## 4. Tareas activas

| Quién | Tarea | Estado |
|---|---|---|
| Humano | HOME-SEC: revocar token y limpiar URL de `origin`; borrar sudo temporal; desactivar permisos irrestrictos en Orca | pendiente |
| Humano | Decidir la pregunta de confirmación del receptionist en modo competencia (`ROADMAP.md` §5) | pendiente |
| Humano | Medir la columna del torso; ubicar waypoint `home` y objetos en el editor de mapa | pendiente |
| Claude | HOME-TRT (engines YOLO), HOME-S02, HOME-S11, HOME-30 (auditoría nav solo lectura) | próximas |
| Antigravity | HOME-15b, HOME-05, HOME-16, HOME-31 | sin despachar (worker no activo) |

## 5. Bloqueos vigentes

- Push/PR: token expuesto en el remoto.
- Pruebas con actuadores y voz con personas reales: requieren `LAB_SUPERVISADO`.
- Piper: 0 tramas CAN la última vez que se energizó (30/10) → revisar alimentación/E-stop/cable CAN.
- Pan-tilt: el U2D2 se desconectó por USB el 30/10 → revisar cable y 12 V.

## 6. Últimos cambios importantes

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

1. Humano: HOME-SEC (token, sudo, Orca) → habilita push y PR de la rama de referencia.
2. Claude: reconstruir engines TensorRT y medir; desplegar fallback DDS (S-02).
3. Con el robot energizado y supervisión: `escuchar --n 10` con personas reales; prueba de CAN del Piper.
4. Crear worktree nuevo para Antigravity desde la rama de referencia y despachar HOME-15b.
