# Nota de vigencia v3

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Se permite exploración profunda de navegación y hoverboard, pero no modificar ni desplegar navegación.** La autoridad detallada está en `AGENTS.md`; los datos históricos pueden estar obsoletos.

---

# Orca · Protocolo de coordinación Claude Code ↔ Antigravity (v2)

**Revisado:** 2026-11-03. Ambos agentes operan como desarrolladores de un mismo robot, no como instancias de chat aisladas. La coordinación solo existe cuando Orca muestra Run, Task, worker y mensajes reales.

## Arranque y verificación

Comprobar el árbol de trabajo y las sesiones, con comandos inocuos:

```bash
pwd
git rev-parse --show-toplevel
git status --short --branch
git worktree list
command -v orca
orca status --json
orca skills get orchestration --full
```

**Interpretar la documentación CLI vigente de esa instalación antes de copiar comandos orientativos de internet.** No inventar IDs ni invocar comandos retirados.

## Roles y ownership

- Claude: coordinador, auditor de seguridad, propietario de integración y de recursos host NUC/Jetson/SSH, final reviewer. Implementa tareas también.
- Antigravity: worker de subtareas definidas, frontend/editor URDF, simulación, pruebas, HRI y revisión cruzada. Puede liderar otras tareas si el contrato lo permite.
- Humano: decide límites de navegación/energía, hardware LAB, merge main y acciones irreversibles.
- Un único editor para `AGENTS.md`, `CLAUDE.md`, `TASKS.md`, `HANDOFF.md`, README y archivos ROS compartidos por iteración.

## Tipos de tarea y permisos

| Clase | Ejemplo | Crear/ejecutar automáticamente desde Orca | Aprobación humana adicional |
|---|---|---|---|
| SOFTWARE | tests, UI local, docs, HRI sim, CI | Sí con worktree + checklist | No |
| HOST REVERSIBLE | unit RealSense sin actuadores, venv, cámara pasiva, Jetson script no actuador | Sí con ownership, backup, monitor y rollback | Si falta un gate o el servicio abre CAN/torque |
| MAPA SEMÁNTICO | zonas y waypoints candidatos | Sí, solo archivos permitidos | Para publicar/recarregar operativo o mover base |
| HARDWARE | Piper, Dynamixel, elevador, base | No automáticamente | Sí, por sesión/ensayo y runbook |
| INFRA DESTRUCTIVA | flash, QSPI, particiones, red sin alternativa | No | Sí, operación concreta |
| SERVICIOS EXTERNOS | pago/Google personal | No | Prohibido hasta nueva instrucción explícita del usuario para cambiar esas políticas |

## Recursos globales: no confiar en worktree como sandbox

Cada tarea declara locks para `JETSON_SSH_WRITE`, `NUC_SYSTEMD`, `REALSENSE_USB`, `ORBBEC_USB`, `ROS_DOMAIN_10`, `ROS_DOMAIN_42`, `GPU_JETSON`, `PIPER_CAN`, `MAP_SEMANTIC`, `GIT_INTEGRATION`. Solo un agente puede adquirir recursos mutables compartidos a la vez. Si Orca no proporciona locks técnicos, usar owner escrito y evitar procesos simultáneos; no fabricar un bloqueo efectivo mediante Markdown.

## Contrato mínimo de dispatch

```text
TASK-ID/Run-ID: <IDs reales>
Owner: Claude | Antigravity
Modo: CASA | LAB_SUPERVISADO (solo tras autorización humana)
Objetivo tangible: <un resultado comprobable>
Baseline commit SHA: <real>
Worktree y branch: <reales>
Archivos lectura: <paths>
Archivos escritura permitidos: <paths sin conflictos>
Recurso compartido reservado: <ej. REALSENSE_USB o ninguno>
Pruebas: <comando, entorno aislado y condición de salida>
No tocar: Nav2 core, ROS1, Google, pagos, secretos, hardware salvo gate
Deliverables: diff, tests, logs saneados, documentación, rollback
Done: <criterio inequívoco>
```

## Estados de agente y mensajes

`PLANNED → DISPATCHED → ACKNOWLEDGED → IMPLEMENTING → REVIEW_READY → ACCEPTED` o `BLOCKED_WORKER` / `REJECTED`. Un `worker_done` no prueba que el código sea aceptable: siempre revisar el diff. Solicitar checkpoints al terminar unidades útiles; no saturar Orca con heartbeats triviales.

## Recuperación sin pérdida de trabajo

1. Obtener estado real del worker y archivos de su worktree. No relanzar dos workers sobre misma rama.
2. Si cuota/login/caída, intentar una reconexión razonable con permisos seguros; sin pagos ni acceso a cuentas Google.
3. Si continúa bloqueado, registrar `BLOCKED_WORKER`, snapshot branch/SHA, test/diff pendientes; Claude puede ejecutar esa tarea en worktree propio tras reservar archivos y dejar handoff de responsabilidad.
4. Para integraciones: revisar `git diff`, `git status`, pruebas, dependencias, línea de seguridad y contratos ROS. No borrar worktree ajeno con cambios sin integrar.

## Git y GitHub

- Trabajo local y commits permitidos. Push de **ramas de tarea** + PR tras remediación de token, secret scan, pruebas y confirmación de que CI no despliega al robot. Las reglas del CLI todavía pueden pedir una aprobación al ejecutar `git push`.
- Directo a `main`, merge main, force-push, rebase de rama ajena, crear releases o borrar branches remotas: no, salvo autorización puntual para la acción que no estuviera prohibida absolutamente.
- NUNCA compartir `.env`, bags con humanos, audio no consentido o tokens en handoffs. Git `remote -v` se inspecciona con salida redactada si hay URL con credenciales.

## Orca Settings (comprobar en host real)

En **Settings → Agents → Agent Permissions**, configurar **Manual** o **Auto** para Claude según soporte, y **Manual** para otros agentes si no existe modo Auto seguro. Eliminar `--dangerously-skip-permissions` o `--yolo` de argumentos personalizados; nuevas sesiones heredan presets, sesiones existentes no necesariamente. Ver https://www.onorca.dev/docs/settings y https://www.onorca.dev/docs/agents/supported. Un modo Yolo no debe correr conectado a actuadores o servicios productivos.

Si hay scripts de aprobación automática en `scripts/orca/`, inspeccionar código e invocaciones; desactivarlos si aprueban comandos sin analizar alcance. Nunca convertir un `question` de worker en aprobación humana ficticia.

## Historial y fuentes

La base del flujo está en `docs/collab/HANDOFF.md` del repo **vivo**. No sobreescribirlo con snapshots antiguos; solo el integrador agrega eventos reales. Leer `docs/collab/TASKS.md` y el Run activo antes de decidir prioridades. Docs de Orca: https://www.onorca.dev/docs/cli/orchestration y https://www.onorca.dev/docs/cli/overview.
