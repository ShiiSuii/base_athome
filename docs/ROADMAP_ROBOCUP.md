# ROADMAP ACTIVO — Base@Home / RoboCup@Home OPL (v3)

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

## Política de trabajo

El objetivo es perfeccionar el robot completo: NUC/Jetson, HRI, percepción, Piper, brazo/cámara, URDF, manipulación y pruebas RoboCup. **La navegación que ya funciona no se modifica**. Los agentes sí deben investigarla profundamente y sugerir mejoras razonadas. Esas propuestas NO se aplican automáticamente. Las tareas tienen estado real en Orca y en commits/handoffs; este archivo define prioridades técnicas, no despachos.

## Estado derivado de los documentos entregados (revisar hoy)

- **NUC:** Ubuntu 24.04/Jazzy, navegación funcional según usuario.
- **Jetson:** handoff del 30/10/2026 informa migración completada a **JetPack 7.2.1 / Ubuntu 24.04 / ROS2 Jazzy**, servicio `jetson-head` y comunicación ROS2 NUC-Jetson. **No reflashear por seguir un P0 antiguo.**
- **Red:** Ethernet 1 Gb/s NUC `enp1s0` ↔ Jetson USB RTL8153 `enx00e04c68090c`; 10.42.0.1 ↔ 10.42.0.12 documentados, verificar vigencia.
- **Percepción/seguridad:** fixes de Piper feedback, auto enable desactivado, streaming Orbbec mejorado, RealSense y TensorRT según handoff.
- **Herramientas:** editor URDF inicial (45 tests), `base_athome_sim` y Gazebo, Receptionist y GPSR; verificar versiones reales, no duplicar código.
- **Hardware:** alimentación de motores, Piper, elevador y Dynamixel se confirma **en cada sesión**, nunca inferir del día 30/10.
- **Advertencias:** token GitHub antes expuesto en remoto, sudo temporal, autoaprobador Orca y conflictos de dependencias `numpy/cv2` requieren revisión.

## Prioridades activas propuestas

| ID | Prioridad | Entregable | Lead | Ejecución autorizada | Evidencia/DONE |
|---|---|---|---|---|---|
| HOME-33 | P0 | Conciliar checkout, Git, workers/Orca, NUC, Jetson y estado desde el último handoff | Claude | lectura pasiva, diagnósticos, documentos | matriz `OBSERVADO HOY/HISTÓRICO/NO VERIFICADO` y SHA |
| HOME-SEC | P0 | Revisar revocación token GitHub, privilegios sudo temporales, bypass de Orca, backups | Claude + humano para credenciales | auditoría sin imprimir secretos, corrección limitada con rollback | cero credenciales en remotos/logs; sudo restringido |
| HOME-30 | P1 | Auditoría profunda de navegación hoverboard→odom→TF→Nav2 **sin editarla** | Claude | lectura, comparación, tests offline | matriz de contratos, métricas pasivas, sin diff de navegación |
| HOME-31 | P1 | Investigación de repos de bases hoverboard + mejores prácticas | Antigravity | web pública/código en lectura | fichas de 5–8 proyectos, licencias y matriz |
| HOME-32 | P1 | Propuestas de mejoras de navegación sin implementación | Claude + review | documento / simulación aislada | NAV-PROP con evidencia, riesgos, rollback, aprobación pendiente |
| HOME-04/05 | P1 | Validar y perfeccionar editor URDF 3D localhost (no reimplementarlo de cero) | Antigravity | cambios en `tools/urdf_editor` y tests | preview real, undo, candidate export, Xacro válido |
| HOME-06/12 | P1 | HRI Receptionist y Challenge (VAD/EOT/Whisper, diálogo, anfitrión, puerta) | Claude + Antigravity | desarrollo + pruebas aisladas, cámara bajo condiciones | escenarios e2e y test audio real supervisado |
| HOME-08/09 | P1 | Piper + Orbbec + MoveIt2: calibración ArUco eye-in-hand, TF, grasp/recovery | Claude + Antigravity | offline por defecto; laboratorio con autorización actual | residuales, precisiones, tasa éxito en pruebas físicas autorizadas |
| HOME-15 | P1 | GPSR robusto para instrucciones compuestas y planes completos | Antigravity | parser offline, fixtures | exact-plan accuracy, recuperación en falsas detecciones |
| HOME-16/17 | P1 | CI offline, modularización, rendimiento, dependencias y documentación | Ambos | software no protegido | tests y benchmark antes/después; PRs pequeños |
| HOME-18 | P1 | Mantener README, API, hardware, power domains, procedimientos y github | Claude integrador | docs + PR bajo política Git | información de fecha/SHA, sin inventar experiencias |
| HOME-COMP | P2 | Validación simulada de desafíos RoboCup 2026.2 y priorización por puntaje/esfuerzo | ambos | mock/Gazebo | matriz rulebook→skill→test→resultado |

**Escala:** P0 desbloquea seguridad/progreso; P1 es valor demostrable; P2 es trabajo posterior. `HOME-11` queda **históricamente completado según handoff**; la tarea nueva es mantenimiento/verificación, no una segunda migración.

## Trabajo en paralelo (Orca)

- Claude: `HOME-33/HOME-SEC` primero, luego `HOME-30` (nav read-only) y mejoras de HRI/manipulación que no colisionen con worktree de Antigravity.
- Antigravity: `HOME-31` investigación independiente, luego editor URDF y GPSR con contratos bien definidos. Si la cuota falla o worker muere, Claude recupera tarea y registra causa.
- Integrador único mantiene `docs/collab/TASKS.md`, `HANDOFF.md`, README y APIs comunes tras revisar trabajo de ambos. No modificar archivos de navegación ni cambiar main sin aprobación.

## Qué no se debe hacer para acelerar

- No modificar Nav2, archivos de mapa `.pgm`, AMCL/SLAM, odometría o firmware hoverboard para experimentar. **Investigar y sugerir sí.**
- No ensayar movement commands en `ROS_DOMAIN_ID=10` ni activar base/elevador por pruebas automáticas.
- No ROS 1, Google accounts, pagos, servicios cloud cobrables ni lectura de secretos.
- No reinstalar Jetson sin razón y aprobación específica; histórico en `docs/archive/`.
- No confundir pruebas unitarias con resultado de Robocup real, ni arrancar cámaras/drivers duplicados.

## Qué se entrega cada sesión

| Obligatorio | Ejemplo |
|---|---|
| Log de trabajo | tarea+agente+worktree+SHA+fecha |
| Evidencia | comandos reales, tests ejecutados, métricas y logs saneados |
| Seguridad | constancia de no tocar navegación ni mover actuadores fuera de modo |
| Documentación | README/diagramas/API/handoff/roadmap actualizados |
| Recomendaciones | NAV-PROP y mejoras fuera navegación con severidad |
| Próxima decisión | qué puede continuar automáticamente vs qué requiere humano |

Referencia: `docs/HRI_COMPETENCIA.md`, `docs/collab/TASKS.md`, `docs/collab/ORQUESTACION_ORCA.md`.
