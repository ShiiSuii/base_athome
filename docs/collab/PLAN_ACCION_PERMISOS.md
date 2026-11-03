# Nota de vigencia v3

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Se permite exploración profunda de navegación y hoverboard, pero no modificar ni desplegar navegación.** La autoridad detallada está en `AGENTS.md`; los datos históricos pueden estar obsoletos.

---

# Plan de acción: activar autonomía controlada sin bloquear el desarrollo

## 0. Antes de empezar (usuario / coordinador)

- [ ] Respaldar AGENTS.md, CLAUDE.md y docs/collab/* del Git **vivo**; comparar con esta propuesta antes de copiar. No reemplazar HANDOFF/TASKS vivos por snapshots antiguos.
- [ ] En Orca **Settings → Agents → Agent Permissions** usar Manual o Auto compatible, no Yolo. Revisar argumentos: eliminar `--dangerously-skip-permissions` de Claude y `--yolo` de Antigravity, incluidos argumentos personalizados. Reiniciar sesiones necesarias para aplicar nuevos modos.
- [ ] En Claude `/permissions` verificar que `.claude/settings.json` se carga y que `ask` y `deny` realmente funcionan. No asumir que Markdown prevalece sobre configuración del host.
- [ ] Confirmar en cada sesión `CASA` salvo autorización física actual.

## 1. Primer gate humano (seguridad externa)

- [ ] Verificar si el token GitHub sigue en URL `origin`. **El humano rota/revoca la credencial** si estuvo expuesta. Cambiar a autenticación SSH/credential manager sin incluir token en URL. No compartir secreto en logs.
- [ ] Verificar si sigue `99-jetson-flash-temp` en NUC y NOPASSWD total en Jetson. Humano/administrador elimina las reglas de flasheo sobrantes y configura mínimo privilegio real. No otorgar un sudo root universal a los agentes.
- [ ] Revisar y desactivar el autoaprobador `scripts/orca/agy_autoapprove.py` si aprueba órdenes sin comprobación específica.
- [ ] Confirmar la configuración y el estado de servicios `jetson-head`, Piper disable, RealSense ownership, dominios ROS.

## 2. Operaciones ya autorizadas por política, sujetas al checklist de Nivel B

- [ ] Detener procesos de prueba huérfanos propios identificados (SIGTERM, sin kill global).
- [ ] Corregir `joint_state_publisher` obsoleto en visualización, sin alterar producción ni navegación.
- [ ] RealSense: inspeccionar unit, respaldar, iniciar como usuario sin conflictos, validar y habilitar si efectivamente es necesario.
- [ ] Editor semántico: editar `semantic_map.yaml`/`waypoints.yaml` con backup, schema y pruebas; no recargar runtime.
- [ ] Aislar dependencias Ultralytics/Numpy/OpenCV; bloquear actualizaciones globales, ejecutar pruebas import y comparación FPS.
- [ ] Mejorar UI URDF, VAD/HRI/GPSR y simulación con pruebas unitarias y e2e.
- [ ] Push a ramas feature/PR después de revocar token y validar que no hay secrets y que el CI no despliega al robot.

## 3. Solo al activar LAB_SUPERVISADO por mensaje actual

- [ ] Registrar humano/E-stop/energía/driver/límites/feedback/acción/abort.
- [ ] Autorizar una prueba acotada de Piper o pan-tilt. No base/elevador salvo permiso explícito independiente.
- [ ] Medir torso real, corregir colisiones MoveIt y luego repetir planificación segura.

## 4. Métricas de aceptación

Cero secretos en Git/terminal pública; cero `sudo` general para agentes; 100% cambios con rollback; 100% tests distinguen real/sim; sin cambios Nav2 core; editor URDF renderizado; HRI/GPSR con pruebas e2e; cámara RealSense un solo owner; acceso Jetson estable; PRs por rama; cero movimientos reales fuera del modo LAB.

## 5. Foco: no bloquear por documentación vieja

Cerrar migración Jetson de tareas **pendientes** si estado actual coincide con HANDOFF; no volver a hacer flash. Archivar runbooks de instalación como históricos, documentar errores de versiones. El sistema de reglas debe ir cambiando con evidencia real del robot; cada cambio en política requiere owner y changelog, sin contradicciones.
