# DECISIONES — registro de arquitectura y autorización

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

| Fecha | ID | Decisión | Consecuencia | Quién/estado |
|---|---|---|---|---|
| Previo / ratificada 03-11-2026 | D-001 | Navegación actual funciona; **inspección profunda y recomendaciones permitidas, modificación de navegación prohibida por defecto** | estudios en docs y simulador; ninguna modificación de Nav2/hoverboard operativo | Usuario |
| 03-11-2026 | D-002 | Claude+Antigravity, coordinados mediante Orca, con investigación técnica amplia | trabajos paralelos con tareas/worktrees y revisión cruzada | Usuario |
| Previo | D-003 | Exclusivamente ROS 2; sin Google accounts ni pagos | sin ROS 1, Google personal, nube facturable | Usuario |
| 03-11-2026 | D-004 | Jetson migrada según HANDOFF 30/10; no repetir migración | verificar estado, runbook queda histórico | Handoff, no revalidado hoy |
| Previo | D-005 | NUC-Jetson Ethernet; acceso SSH y mantenimiento reversible permitidos | control de cambios de servicios/rollback; no movimientos implícitos | Usuario |
| Previo | D-006 | Modo CASA por defecto; LAB_SUPERVISADO solo con consentimiento actual | seguridad de actuadores, E-stop y supervisor | Usuario |
| 03-11-2026 | D-007 | Libertad para investigar nuevas bases con motores hoverboard, compararlas y proponer mejoras | HOME-30/31/32 | Usuario |
| 03-11-2026 | D-008 | Reemplazar documentación generada por ChatGPT sin borrar registros vivos nuevos | fusionar TASKS/HANDOFF si existen estados posteriores | Política de integración |

### Pendientes de decisión humana (no autorizaciones implícitas)

- Aplicación futura de cualquier cambio en navegación/base o firmware.
- Pruebas con movimientos físicos: modo, supervisor y alcance exacto.
- Servicios que arranquen hardware automáticamente, red/SSH crítica y cambio de alimentación.
- Operaciones de borrado/particionado/reflasheo y ajustes de seguridad irreversible.
- `main` merge, force push, rotación de credenciales de otras cuentas sin intervención del dueño.
