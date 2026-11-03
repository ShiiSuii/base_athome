# Política Git + worktrees Orca + entrega a GitHub

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

## Propiedad de tareas y directorios

Orca gestiona worktrees Git separados, pero ambos agentes **comparten el mismo host, ROS, puertos, servicios, red y hardware físico** si corren en NUC. Worktrees no son un sandbox de seguridad. El coordinador reserva recursos y evita que dos agentes reinicien el mismo servicio, compilen el mismo `build/` o modifiquen `README.md` simultáneamente.

- Claude orquestador/integrador; Antigravity implementador/revisor con worktree dedicado.
- Un agente es owner de cada archivo o subsistema por tarea. Si hay superposición, crear contrato/API antes y dividir por archivos.
- Toda rama nueva parte de un SHA validado, no de una `main` pública atrasada. El integrador comprueba `git status --porcelain` y preserva datos sin commitear.
- Convenciones: `claude/home-30-nav-audit-readonly`, `antigravity/home-31-hoverboard-research`, `antigravity/home-05-urdf-preview`. Son sugerencias, no ramas que existan hoy.

## En Orca

1. Detectar CLI (`orca status --json`), cargar la guía de orquestación *de la versión instalada* (`orca skills get orchestration --full`). Si falta el comando, no inventar IDs ni workers.
2. Crear Run objetivo y Task con los campos: ID, responsable, dependencias, `READ_PATHS`, `WRITE_PATHS`, recursos externos, API/contrato, tests, criterios de terminado y rollback.
3. En cada `worker-start` dejar constancia del dispatch real, worktree, modelo y permisos efectivos. No marcar `IN_PROGRESS` por una sugerencia que no se despachó.
4. Al recibir `worker_done`, revisar `git status`, `git diff --stat`, `git diff --check`, tests y documentación. Un heartbeat no implica trabajo correcto.
5. Fusionar **solo en rama de integración** con conflicto resuelto, preservar la historia, volver a ejecutar pruebas; no tocar main.
6. Sin respuesta del worker: verificar estado; cancelar/reasignar sin borrar worktree no integrado; Claude puede tomar la tarea para evitar que quede bloqueada por cuota.

## Política de push/PR

Permitido al coordinador **después de revocar token expuesto**: publicar ramas de trabajo y crear PRs de revisión, tras escaneo de secretos, licencia, datos privados y tests. No publicar videos/bags/audio/rostros y no añadir enlaces con tokens.

Prohibido sin confirmación humana: push a main, merge, force push, cambiar protecciones, tags/release pública, Actions que consuman recursos facturables, subir dataset privado, activar CI externo con secretos o despliegue desde la rama automáticamente.

Si los ajustes efectivos `.claude/settings.json` u Orca siguen pidiendo confirmación para `git push`, respetarlos: **política en Markdown no elimina el diálogo ni una barrera del sistema**.

## Revisión de código

- Todo patch pasa tests unitarios y de integración aislados donde aplican; `colcon` en `build/` propio por worktree.
- APIs ROS 2: preservar message types, nombres de topic/action, frames y unidades; cambios incompatibles requieren ADR + plan de migración.
- En navegación: `git diff --name-only` debe mostrar **cero** archivos protegidos, salvo documentos `docs/`/candidatos aislados. Si aparecen cambios a nav, bloquear integración.
- Añadir entrada de HANDOFF con SHA y rollback; registrar incidencias sin inventar pruebas.
