# Revisión cruzada y criterios de aceptación

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

## Checklist de PR y agentes

- [ ] Rama y SHA conocidos; `git diff --check` sin errores.
- [ ] No se modificó ningún archivo operativo de navegación, firmware, SLAM, AMCL, costmaps, parámetros base, mapa de ocupación o driver hoverboard; si sí, detener.
- [ ] Cambios limitados al alcance del Task Contract y aprobado por integrador.
- [ ] No se añadieron claves, `ghp_...`, URL con tokens, .env, credenciales, acceso Google o métodos de pago.
- [ ] `colcon build`, `colcon test`, `pytest` y linters según paquete con resultados **reales**; registrar si un test no pudo ejecutarse.
- [ ] Seguridad de HRI/actuadores: sin auto enable, feedback medido vs comandado, sin control físico desde tests.
- [ ] Offline: `ROS_DOMAIN_ID=42`, `ATHOME_SIM=1`/fixtures; no publicar al dominio operativo `10`.
- [ ] Nuevas dependencias: licencia, versión congelada, compatibilidad ARM64/AMD64, sin mutación de venv de percepción/YOLO en producción.
- [ ] Rendimiento: baseline y post-cambio con misma condición de comparación.
- [ ] Documentos README, instrucciones de instalación, roadmap, interfaces y HANDOFF coherentes.
- [ ] Reversión claramente especificada y mínima.

## Clasificación de defectos

| Nivel | Ejemplos | Decisión |
|---|---|---|
| BLOCKER | movimiento involuntario, daño a credenciales, tests que ordenan Nav2, borrado de datos, exposición biométrica | rechazar |
| HIGH | feedback falso, race conditions, crash del nodo central, deadlock de callbacks | corregir antes de integrar |
| MEDIUM | cobertura insuficiente, logs erróneos, tasa baja, rutas absolutas | task prioritaria/condicional |
| LOW | nombres, organización, estilo/documentación menor | corregir o backlog |

## Certificación de pruebas

Separar `STATIC_PASS`, `UNIT_PASS`, `SIM_PASS`, `NUC_VERIFIED`, `JETSON_VERIFIED`, `ETHERNET_VERIFIED`, `HARDWARE_VERIFIED`. No usar `READY` para challenge físico sin ensayo repetible en el escenario relevante. Pruebas HRI con voluntarios deben respetar consentimiento y manejo de grabaciones.
