# Nota de vigencia v3

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Se permite exploración profunda de navegación y hoverboard, pero no modificar ni desplegar navegación.** La autoridad detallada está en `AGENTS.md`; los datos históricos pueden estar obsoletos.

---

# Permisos y bloqueos — análisis actualizado y autorización propuesta para Base@Home

**Actualizado:** 2026-11-03. **Base:** archivo del agente `PERMISOS_Y_BLOQUEOS.md` (2026-10-30) y HANDOFF/Tasks más recientes proporcionados. La presente actualización no demuestra estado actual de procesos: todo hallazgo operativo debe revalidarse. **No se han cambiado permisos del host ni configuraciones de Orca solo por generar este archivo.**

## Matriz B1–B9: decisión y estrategia

| ID | Problema reportado | Nueva autorización | Plan reproducible y freno |
|---|---|---|---|
| B1 | `realsense-nuc.service` no habilitado por bloqueo de permisos | **Permiso condicionado Nivel B** instalarlo/activarlo como unidad de usuario | Revisar `ExecStart`, usuario, `ROS_DOMAIN_ID`, propietario de `/dev/video*`, conflicto con launch existente; copiar unit tras backup, `daemon-reload`, start temporal, verificar topic FPS/QoS/log; habilitar persistencia solo si es necesaria y con rollback `disable --now`; no relanzar durante ensayo activo |
| B2 | `joint_state_publisher` viejo compite con controlador | **Permiso Nivel B** finalizar procesos **propios huérfanos** | Inspeccionar `ps`, PID/PPID, cmdline, UID y tópico; verificar no es Nav2 ni servicio; usar `SIGTERM PID`; observar desaparición de duplicados. Nunca `kill -9` automático, `pkill -f` global ni señales a procesos de otro propietario |
| B3 | La sesión antigua autorizó usar Piper/pan-tilt, pero reglas prohíben mover | Modos CASA/LAB_SUPERVISADO/COMPETENCIA | CASA por defecto; para LAB autorización de usuario presencial **en esa sesión**, preflight E-stop + feedback + zona, un movimiento concreto, velocidad conservadora y detener ante anomalía. No aprobación heredada |
| B4 | `editor_mapa` escribe YAML en carpeta llamada navegación | **Excepción semántica Nivel B** | Permitir solo `semantic_map.yaml`/`waypoints.yaml` y código editor semántico, validación YAML, backup y diff, no recargar runtime. `Nav2`, AMCL, SLAM, costmaps, controlador, mapa `.pgm`, parámetros de footprint y bringup siguen protegidos |
| B5 | Docs Jetson obsoletas | Actualización factual (reportada) | Actualizar inventario a JetPack 7.2.1, L4T R39.2.1, Ubuntu24, Jazzy según HANDOFF 30/10; corroborar con `/etc/os-release`, `nv_tegra_release`/paquetes Jetson y `ros2 doctor`; archivar runbook de migración como histórico y **no volver a flashear** |
| B6 | Workers Antigravity fallan por cuota, login, contexto y auto-aprobaciones | Claude puede asumir tareas pendientes y orquestar sin pedir permiso por cada worker inocuo | Verificar agente en Orca, un reintento; si falta cuota/no autorizado, registrar `WORKER_UNAVAILABLE` y reasignar. Auditar `scripts/orca/agy_autoapprove.py` y deshabilitar cualquier confirmación indiscriminada. No pagar upgrades ni entrar a cuentas Google |
| B7 | Compactación y pérdida de contexto | Resumen obligatorio por tarea | Registrar SHA, archivos, resultados/limitaciones, modo, host, servicios, métricas, rollback, decisión. Antes de compactar/cerrar y después de cada tarea; snapshot Git, Orca como fuente viva |
| B8 | Exportación Ultralytics cambió NumPy y rompió OpenCV | Entornos aislados y lockfile | No exportar con venv productivo; `YOLO_OFFLINE=1` cuando corresponda; generar nuevo venv sandbox, `pip freeze`, probar import `cv2/numpy/torch`, YOLO engine TensorRT, preservar versión conocida. No ejecutar `pip install -U` amplio en producción |
| B9 | Medición columna, localización objetos, energización | **Solo humano para la medición y energización** | Claude prepara fichas y UI para cargar medidas y tests; no inventar altura/obstáculos. Humano mide columna y valida waypoints `home` y objetos; luego implementar, simular, planificar sesión LAB |

## Riesgos nuevos/relevantes tras el HANDOFF

- **Token GitHub en `origin`:** comprobar si la exposición se solucionó **sin mostrar el token**. Si todavía existe, indicar al humano que lo revoque/rote y sanee URL; bloquear push hasta confirmarlo. No poner token en prompts, logs, handoffs, ZIP, issues, commits ni tests.
- **Privilegios temporales tras flash:** examinar existencia de `/etc/sudoers.d/99-jetson-flash-temp` (NUC) y `90-ubuntu-nopasswd` (Jetson) sin publicar secretos; proponer retirar o acotar mínimo privilegio. No dejar contraseña `sudo` ilimitada para un agente autónomo.
- **El agente autoaprobador:** confirmar si existe/está activo; si acepta prompts por regex amplia o ciega, deshabilitar y reemplazar por autorizaciones por nivel, ver `ESTRATEGIA_EJECUCION.md`.
- **Documentación vieja:** `LEEME_INSTALACION.md`, `MIGRACION_JETSON.md`, `RUNBOOK_MIGRACION_JETSON.md`, `RED_ETHERNET.md` y `PROMPTS_AGENTES.md` contienen la ruta de migración **anterior a la ejecución**. Actualizar el README hacia estado actual y archivarlos como historial, no volver a ejecutar pasos destructivos.
- **Tests/servicios nuevos:** confirmar que `jetson-head` sigue usando `auto_enable=False`, sin estados de articulaciones fingidos y que simulación usa el dominio aislado y no interfiere con componentes USB/CAN.

## Política concreta de permisos

**A — Automático:** lectura técnica, `pytest`, `colcon` aislado, xacro, lint, local Git, documentación, auditoría, simulación con `ATHOME_SIM=1`/dominio 42, commit a rama propia.

**B — Automático después de preflight verificable:** detener procesos huérfanos propios (`SIGTERM`), servicio RealSense de usuario sin actuadores, ediciones semánticas, instalaciones de dependencias en venv específico, despliegues reversibles de nodos no actuadores, tareas/workers Orca con roles de software. Push de rama de trabajo/PR solo tras revocación de token, análisis de diff y pruebas; respetar los prompts de autorización del CLI.

**C — Una aprobación humana específica por operación/sesión:** movimiento Piper/pan-tilt, torque, CAN, base, elevador, `sudoers`, flash/firmware, modificaciones de red crítica y servicios de navegación, merge `main`, datos de personas y cambios que afecten seguridad.

**D — No:** ROS 1, Google personal, pagos, secretos, desactivación safety, Nav2 core, operaciones destructivas ciegas, bypass permisos/Yolo en máquinas con hardware.

## Configuración efectiva

El archivo propuesto `.claude/settings.json` contiene reglas **limitadas**, y `defaultMode=default`/restricción de bypass. Algunas órdenes siguen pidiendo confirmación porque afectan un host o un servicio compartido: eso es deliberado. `ask`/`deny` prevalecen sobre `allow`; reglas Bash basadas en texto no son un control de seguridad completo ante wrappers/comandos equivalentes. Para pruebas fuertes usar usuario Linux con permisos mínimos, y no iniciar Orca en modo Yolo con acceso al hardware.

**Verificación desde Claude:** ejecutar `/permissions` y confirmar qué archivos de settings están activos. **Verificación Orca:** `Settings → Agents → Agent Permissions` y los argumentos concretos de cada agente; una sesión vieja puede conservar el modo al reiniciar. Fuentes oficiales:
- https://code.claude.com/docs/en/permissions
- https://code.claude.com/docs/en/settings
- https://www.onorca.dev/docs/settings
- https://www.onorca.dev/docs/agents/supported

## Requisitos de evidencia por tipo de autorización

| Operación | Qué adjuntar en HANDOFF | Señal de abortar |
|---|---|---|
| venv/deps | freeze antes/después, tests import, rollback | cambios en venv productivo inesperados |
| systemd de usuario | unit file hash, `status`, logs saneados, `disable --now` | doble owner de cámara, reinicios en bucle |
| SIGTERM | PID/PPID/UID/argv, por qué es huérfano, efecto, reversión | propietario incierto o proceso crítico |
| capa semántica | YAML backup/diff, validador, coordenadas de fuente y revisión | difiere mapa SLAM, cambia footprint o Nav2 |
| GitHub branch | sha, test, secret scan, remote saneado, nombre rama/PR | token en URL, push a main, CI con deploy |
| hardware LAB | humano, modo, E-stop, feedback, pose, límites, log, stop | feedback stale, obstáculo, torque inesperado |
