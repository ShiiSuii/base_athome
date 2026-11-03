# Propuestas técnicas de navegación — registro SIN implementación

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

## Política

**La navegación funciona. No cambiar ningún componente, parámetro, mapa de ocupación, firmware, odometría, TF navegacional, controlador ni goal de base por sugerencia de un agente.** Este archivo es el lugar autorizado para hipótesis, auditorías y mejoras potenciales sobre hoverboard y Nav2. Para cambios reales, el usuario debe indicar explícitamente el alcance y su aceptación después de revisar la propuesta.

## Plantilla de cada propuesta

```markdown
### NAV-PROP-XXX — [Título]
- Estado: IDEA | ANALIZADA | SIMULADA | DESCARTADA | EN ESPERA DE DECISIÓN | APROBADA POR HUMANO
- Autor, fecha, rama/SHA y hardware/ROS observados:
- Hallazgo: qué se observó exactamente (con archivo:línea, comandos pasivos, logs saneados):
- Evidencia real vs hipótesis vs comparación externa:
- Problema/impacto actual: ¿hay fallo reproducible o es mejora potencial?
- Alternativas: A conservar actual / B modificar fuera de Nav2 / C cambiar Nav2 (requiere autorización)
- Propuesta no invasiva: descripción, arquitectura, código candidato **solo offline**:
- Fuente pública y licencia (si procede):
- Riesgos (movimiento, seguridad, fiabilidad, costo de integración, regresión):
- Plan de simulación/mock y métricas antes/después:
- Plan de regresión y prueba supervisada futura:
- Rollback si se aprueba en el futuro:
- Decisión del usuario: PENDIENTE por defecto
```

## Registro inicial — NO son defectos confirmados

| ID | Pregunta investigable | Referencia inicial | Estado | Acción autorizada |
|---|---|---|---|---|
| NAV-PROP-001 | ¿Dónde y cómo se hace el watchdog/paro al perder UART de hoverboard? | fuente local + EFeru FOC | POR INVESTIGAR | lectura código/logs y test mock |
| NAV-PROP-002 | ¿Cómo mide el sistema su odometría Hall y las covarianzas ROS 2? | hoverboard_base local + OpenBase2 | POR INVESTIGAR | lectura, informe y comparación |
| NAV-PROP-003 | ¿Hay un solo propietario de `odom -> base_link`? | REP 105 + TF2 | POR INVESTIGAR | inspección TF pasiva |
| NAV-PROP-004 | ¿Cómo protege la base la prioridad entre Nav2 y órdenes externas? | software de base local | POR INVESTIGAR | lectura del grafo, sin `cmd_vel` |
| NAV-PROP-005 | ¿Está justificadamente desacoplado el footprint operativo del editor URDF visual? | URDF/Xacro vs Nav2 | POR INVESTIGAR | comparar configuraciones, no aplicar cambios |

**No marcar estos ítems como bugs.** No existe todavía evidencia suficiente en esta entrega para afirmar defecto real en cualquiera de ellos.
