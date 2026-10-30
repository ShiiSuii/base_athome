# HANDOFF — registro de traspaso Claude ↔ Antigravity ↔ humano

> **No es mensajería sincrónica**. Orca Run/Task/Dispatch (si habilitado) es el estado activo. Este documento es un **snapshot versionado** que un único integrador actualiza tras incorporar cambios; otro worktree no verá cambios no integrados.

## Estado inicial · 2026-10-29

- **Entorno indicado:** `/home/athome/base_athome_ws` en NUC; verificar con `git rev-parse --show-toplevel`.
- **Robot:** RoboCup@Home OPL; NUC Ubuntu 24.04 + ROS 2 Jazzy, Jetson Orin Nano documentada 22.04/Humble; objetivo 24.04/Jazzy.
- **Alimentación:** usuario confirmó NUC y Jetson ENCENDIDAS, unidas mediante cable Ethernet directo. Otros sistemas desenergizados; sensores concretos no verificados.
- **Navegación:** usuario confirma que funciona y prohíbe modificaciones. Permitido estudiarla e integrarla a nivel de contratos.
- **Colaboración:** Claude Code y Google Antigravity operarán desde Orca, idealmente en worktrees distintos, bajo reglas comunes.
- **Seguridad:** ROS 2 exclusivo, sin Google, sin compras/pagos, sin lectura de credenciales, sin comandos de actuadores, sin publicar a main. Acceso SSH y software Jetson autorizados; flasheo requiere gate de respaldo/destino.
- **Reglamento base:** RoboCup@Home 2026.2. **No** usar master 2027 como normativa 2026.
- **No hay prueba de que Orca se haya configurado todavía, ni despachos, PRs, builds o hardware tests realizados a partir de esta entrega.**

## Tabla de sesión activa (completar únicamente tras observar hechos)

| Fecha | Agente | Worktree real | Rama real | Tarea (ID Orca) | Estado / evidencia |
|---|---|---|---|---|---|
| 2026-10-29 | Claude Code | `/home/athome/base_athome_ws` (worktree principal Orca) | `claude/home-00-audit` | HOME-00/01/01E/02/03/10 (sin Run Orca) | ver handoff abajo |

## Formato de traspaso por tarea

```markdown
### HANDOFF [fecha/hora] — [agente] — [HOME-XX]

- Objetivo y alcance aprobado:
- Worktree y rama REAL:
- Basado en commit SHA:
- Archivos creados/modificados:
- Cambios exactos:
- Pruebas ejecutadas (comando, ambiente, salida/resumen fiel):
- Evidencia (logs/ruta/captura no privada):
- Estado: STATIC_PASS / UNIT_PASS / SIM_PASS / NUC_VERIFIED / JETSON_VERIFIED / ETHERNET_VERIFIED / BLOCKED_HARDWARE / REAL_VERIFIED
- Contratos ROS/TF/API afectados (si ninguno, escribir ninguno):
- Riesgos y riesgos mitigados:
- Hallazgos no resueltos:
- Revisión de seguridad y privacidad:
- Pendiente de otra persona/agente:
- Rollback:
- Commit(s) o PR reales:
- Aprobación humana requerida:
```

## Mensajes modelo entre agentes (no equivalen a mensajes enviados)

**Claude → Antigravity:** "HOME-04 listo para tomar. Te asigno exclusivamente `tools/urdf_editor/` y sus pruebas. No edites `src/base_athome_navigation`, no edites Xacro de producción, no arranques drivers. Devuelve API, diffs, pruebas reales y bloqueos. No uses Google ni pagos. ¿Recibido por Orca?".

**Antigravity → Claude:** "HOME-04: estado OFFLINE_DONE o BLOCKED; branch/SHA, archivos, tests y limitaciones. Confirmo no haber tocado hardware/Nav2/ROS1/cuentas de Google ni realizado pagos. La integración queda a revisión".

**Claude → usuario:** "Revisé el diff, tests y riesgos; los cambios están en la rama X. No hice push ni fusioné a main. Pido aprobación para [acción exacta]".

## Bloqueos que no deben borrarse sin evidencia

- Jetson: conexión SSH, inventario, backups y mantenimiento de software `READY`; migración OS + ROS2 Jazzy tarea `P0`; instalación destructiva/QSPI/particionado `BLOCKED_APPROVAL` hasta backup comprobado y elección explícita del destino.
- Piper, Orbbec, elevador, Dynamixel: `BLOCKED_HARDWARE` hasta energización y protocolo de seguridad.
- Navegación: `READ_ONLY_PROTECTED` indefinidamente salvo nuevo mandato explícito del usuario.
- Google/pagos/ROS1: `PROHIBITED`, no son bloqueos temporales delegables.

### HANDOFF 2026-10-29 23:55 — Claude Code — HOME-00/01/01E/02/03/10

- Objetivo: reconocimiento de ambos hosts, auditoría inicial, respaldo Jetson. Sin actuadores, sin Nav2.
- Worktree/rama: `/home/athome/base_athome_ws`, `claude/home-00-audit`, basada en `main@1da2388`.
- Archivos: `AGENTS.md`, `CLAUDE.md`, `docs/*` (instrucciones copiadas), nuevos `docs/{architecture,hardware,power-domains,ros-interfaces,software-audit,network-topology,jetson-inventory}.md`; fix `jetson_head/scripts/piper_arm_controller.py` y su copia `scripts/jetson/piper_arm_controller.py` (`auto_enable=False` por defecto); test `jetson_head/scripts/tests/test_piper_arm_controller_safety.py`.
- Pruebas: `ROS_DOMAIN_ID=77 pytest jetson_head/scripts/tests` → 2 passed (con fix); contra código original el test de default seguro falla. `xacro` + `check_urdf` OK. Linters ament fallan en todos los paquetes (preexistente).
- Estado: STATIC_PASS / UNIT_PASS / JETSON_VERIFIED (inventario). ETHERNET: **BLOCKED** (sin carrier en ambos extremos; usuario arreglará el cable el 2026-10-30).
- Jetson accedida por WiFi `192.168.0.203` (misma huella SSH que `10.42.0.12`). Respaldo técnico en NUC `/home/athome/jetson_backups/2026-10-29_pre_migration/` (no es imagen completa).
- Contratos afectados: parámetro `auto_enable` de `piper_arm_controller` cambia default true→false; tópicos/servicios sin cambios.
- Riesgos: el fix NO está desplegado en la Jetson; al reconectar Ethernet `jetson-head.service` arrancará el controlador Piper con `auto_enable=True` (versión vieja).
- Seguridad: token de GitHub embebido en la URL del remoto `origin` en la NUC → revocar.
- Orca: CLI 1.4.244 activo; no se creó Run ni se lanzó Antigravity (requiere autorización humana y revisión de permisos).
- Rollback: `git switch main && git branch -D claude/home-00-audit`; respaldos Jetson no alteran nada.
- Commits: locales en `claude/home-00-audit`, sin push.
- Aprobación humana requerida: (1) desplegar fix Piper en Jetson, (2) fallback DDS en Jetson, (3) lanzar worker Antigravity HOME-04, (4) gate de flasheo HOME-11, (5) push/PR.

### HANDOFF 2026-10-30 ~01:00 — Claude Code (coordinador) — sesión nocturna

- Orca: Run `run_8db8fe9ab48b`. Antigravity con Gemini 3.x default sin cuota ("upgrade" rechazado: pago prohibido); con `gemini-3.8-flash-medium` funciona. Prompts de permiso aprobados por el coordinador (prefijos seguros únicamente).
- Integrado en `claude/home-00-audit`: HOME-04 (editor URDF, worker Claude, 45 tests), HOME-06a (gap analysis, Antigravity), HOME-06b presentación mutua (Claude, 5 tests), HOME-19 detector de puerta (8 tests + SIM), análisis del reglamento 2026.2, catálogo de repos, estudio de hardware, herramientas de red.
- Jetson: desplegados S-01/S-03 (respaldo `~/deploy_backups/20261030_002013`). No se reinició ni detuvo ningún servicio manualmente.
- Hallazgos nuevos: Jetson en modo 25W (CPU 1,344/1,728 GHz); `agx_arm_ros` ros2 soporta Jazzy+MoveIt; reglamento 2026.2 sin "Give Me a Hand".
- Mañana: `docs/collab/RUNBOOK_CABLE_ETHERNET.md`.
- Aprobación humana: ver runbook §5.

### HANDOFF 2026-10-30 ~10:00 — Claude Code — enlace Ethernet restablecido

- Enlace NUC `enp1s0` ↔ Jetson **`enx00e04c68090c` (adaptador USB RTL8153)**, 1 Gb/s; el puerto integrado `enP8p1s0` sigue sin carrier. `check_nuc_jetson_link.sh`: 8 OK / 0 FAIL. Reloj −11.9 ms. Detalle: `docs/network-topology.md`.
- Seguridad verificada en vivo: log del Piper `Brazo DESHABILITADO (usar /arm/set_enable)`; pan-tilt sin torque (servos sin potencia, no responden). `piper_arm_controller.py` desplegado = repo (sha256 `831d800e…`).
- Cambio hecho por un humano en la Jetson (09:39–09:40): `cyclonedds.xml` liga por `address=10.42.0.12` y `start_mapping_head.sh` espera cualquier IP `10.42.x`. Repo sincronizado + fallback solo-localhost (no desplegado: requiere OK).
- S-12 corregido y **desplegado**: cámara de muñeca no publicaba (`v4l2-ctl` ausente → `/dev/video6`) y, al publicar, estaba limitada a 5 Hz por `msg.data = bytes` en rclpy. Ahora 20 Hz en la NUC, CPU 100 %→19 %. Se reinició solo `stream_orbbec.py` (sensor). Respaldo `~/deploy_backups/20261030_095446/`.
- Hallazgos: S-11 (joint states 0.0 falsos cuando el hardware no responde), S-13 (Humble↔Jazzy rompe `ros2 node list`; refuerza HOME-11).
- No probado: interfaces de actuadores (sin potencia), RealSense (no está en la Jetson; verificar en la NUC).
