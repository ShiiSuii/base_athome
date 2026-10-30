# TASKS — Base@Home / RoboCup@Home — backlog compartido

> **IMPORTANTE:** archivo = *snapshot del plan*, no tablero en tiempo real entre worktrees de Orca. Cuando esté disponible, la **Run de Orca** es fuente de verdad de asignaciones activas. Solo el coordinador/integrador actualizará este documento tras recibir handoffs.
>
> Fecha de creación de plan: 2026-10-29. No implica que se ejecutaran pruebas ni que haya tareas completadas.

## Estados válidos

- `TODO`: planificado, no iniciado.
- `READY_OFFLINE`: se puede comenzar sin dispositivos.
- `IN_PROGRESS`: iniciado en branch real (informar agente, SHA, worktree).
- `IN_REVIEW`: requiere revisión de otro agente/humano.
- `BLOCKED_HARDWARE`: necesita hardware que está apagado.
- `BLOCKED_APPROVAL`: requiere autorización humana o instalación.
- `OFFLINE_DONE`: funcionalidad validada offline; pendiente hardware si aplica.
- `HARDWARE_VERIFIED`: resultados de prueba real autorizada documentados.
- `DONE`: integrada y cerrada con criterios acordados (si el cierre requería hardware, también debe estar verificado).

## Restricciones globales

ROS 2 exclusivo. Navegación/Nav2/SLAM/AMCL/control de base **solo lectura**. NUC Y Jetson encendidas, unidas por cable Ethernet. No Google, no pagos, no secretos, no torque/actuadores, no reflasheo, no push/merge sin autorización.

## Orden sugerido y asignación inicial (provisional, no constituye dispatch)

| ID | Prioridad | Tarea / objetivo | Lead sugerido | Dependencia | Estado inicial |
|---|---|---|---|---|---|
| HOME-00 | P0 | Reconocer Git/worktree/Orca, leer reglas, mapa preliminar y estado NUC+Jetson por Ethernet | Claude | ninguna | READY_OFFLINE |
| HOME-01 | P0 | Inventario de hardware/electricidad/hosts con evidencia; crear `architecture`, `hardware`, `power-domains` | Claude | HOME-00 | READY_OFFLINE |
| HOME-01E | P0 | Confirmar enlace Ethernet directo NUC↔Jetson: IP, SSH, RMW, CycloneDDS, tiempos; prueba ROS2 aislada, sin mover actuadores | Claude | HOME-00 | READY_OFFLINE |
| HOME-02 | P0 | Mapa ROS2 completo: launches→nodos→topics/actions/services/TF/QoS; navegación solo lectura | Claude | HOME-00 | READY_OFFLINE |
| HOME-03 | P0 | Auditoría técnica del código, dependencias, estilo, bugs, contratos y seguridad; backlog priorizado | Claude | HOME-02 | READY_OFFLINE |
| HOME-04 | P1 | Editor URDF localhost: propiedades Xacro y validación backend offline | Antigravity | HOME-00 | READY_OFFLINE |
| HOME-05 | P1 | Editor URDF localhost: visor 3D, controles, preset, diff, no deploy | Antigravity | HOME-04 | TODO |
| HOME-06 | P1 | HRI Receptionist → Challenge: auditoría + simulación de diálogo y estados; test de casos fallidos | Claude | HOME-02 | READY_OFFLINE |
| HOME-07 | P1 | Percepción RGB-D: contratos y fixtures offline, ownership cámaras, launch duplicado, estabilidad | Antigravity | HOME-02 | READY_OFFLINE |
| HOME-08 | P1 | Manipulación Piper: estudio driver/URDF/MoveIt2, poses, TF, seguridad y plan ArUco hand-eye | Claude | HOME-01,02 | READY_OFFLINE |
| HOME-09 | P1 | Pipeline pick/place en simulación aislada y plan de calibración Orbbec eye-in-hand | Antigravity | HOME-08 | TODO |
| HOME-10 | P0 | Jetson ENCENDIDA: acceso SSH sobre Ethernet, inventario real, compatibilidad JetPack 7.2.1, backup verificable, instalación de dependencias reversibles y plan de rollback | Claude | HOME-01E | READY_OFFLINE |
| HOME-11 | P0 | Jetson: migración Ubuntu 24.04/JetPack oficial, ROS2 Jazzy y paquetes; software autorizado, paso de flash/borrado sujeto a aprobación específica + respaldo | Claude (humano para gate de borrado) | HOME-10 | BLOCKED_APPROVAL |
| HOME-12 | P2 | HRI 2026.2: timbre/golpes, nombre libre, registro consentido, seguir a persona | Antigravity | HOME-06 | TODO |
| HOME-13 | P2 | Give Me a Hand: estudio scoring, integración habilidades, mocks | Claude | HOME-06,08 | TODO |
| HOME-14 | P2 | Pick and Place Challenge: escenario reproducible, puntuación, objeto, detección y agarre | Claude + Antigravity (por subtask) | HOME-09 | TODO |
| HOME-15 | P2 | GPSR y orquestación habilidades: parser, scheduler, fallbacks, simulación | Claude | HOME-02,06 | TODO |
| HOME-16 | P2 | CI offline: lint, typechecks, tests, Xacro, docs; evitar acciones en host robot | Antigravity | HOME-03 | READY_OFFLINE |
| HOME-17 | P2 | Limpieza incremental de rutas absolutas, logging, imports, configuración y arquitectura | Claude | HOME-03 | TODO |
| HOME-18 | P2 | Documentación reproducible: manual NUC/Jetson, diagramas, decisiones, status de pruebas | Integrador único | HOME-01,02 | READY_OFFLINE |

## Primera tanda concreta recomendada

**Claude:** HOME-00 y HOME-01E por SSH, luego HOME-01/HOME-02 y HOME-10 (Jetson). Producción esperada: tabla comprobable de arquitectura, diagrama de alto nivel, matriz de riesgos; sin cambios funcionales. **Antigravity:** HOME-04 después del acuerdo de archivos y API con Claude. El coordinador debe crear y registrar tareas reales si Orca permite la orquestación. No iniciar ambos tocando `README.md` o Xacro a la vez.

### Contrato de HOME-04 (Antigravity)

- Lectura permitida: `src/hoverboard_base/urdf/` y documentos de modelos, `docs/URDF_EDITOR_SPEC.md`.
- Escritura sugerida: nueva carpeta de herramienta web (elegir nombre tras auditoría, por ejemplo `tools/urdf_editor/`), tests internos y su README. NO modificar URDF fuente en primera iteración.
- Resultado 1: esquema tipado de parámetros permitidos y endpoint/CLI que genera preview temporal con `xacro` real y warnings. Sin ROS runtime ni dispositivos.
- Resultado 2: frontend liviano Three.js para visualizar URDF, sliders de medidas (cm/m), undo, presets, export de candidato; no publicar `/robot_description`.
- Pruebas: Xacro base coincide con regeneración, rangos invalidos rechazados, no path traversal, no escritura fuera de whitelist, no conexiones fuera localhost, mensajes claros con mesh faltante.
- Bloqueos: si el URDF real no coincide con documentación, detener edición fuente y comunicar incertidumbre.

### Contrato de HOME-01/02 (Claude)

- Inspeccionar TODO el checkout, READMEs y diagramas antes de concluir. Inventariar paquetes incluyendo jetson_head/firmware y hardware y cruzar fuentes.
- Mapear el camino Sensor → Nodo → Mensaje → TF → Planificador tarea → Actuador (con actuador apagado). Para navegación: *solo documentar* callbacks, interfaz Nav2 y dependencias.
- Comprobar en NUC solo lecturas pasivas autorizadas y evitar degradar procesos. Crear documentos de arquitectura con evidencia ruta/función/branch/hash, incertidumbres explícitas y preguntas humanas mínimas.
- Entregar 10 problemas de software priorizados con impacto/seguridad, pruebas faltantes, plan de refactor incremental, ninguna modificación de navegación.

## Registro de asignación (rellenar con datos reales)

| ID | Run Task ID real | Agente | Worktree / branch | Inicio | Estado | PR/commit | Nota |
|---|---|---|---|---|---|---|---|
| HOME-00 | sin Run | Claude | `claude/home-00-audit` | 2026-10-29 | OFFLINE_DONE | local | reconocimiento Git/Orca/hosts |
| HOME-01 | sin Run | Claude | `claude/home-00-audit` | 2026-10-29 | IN_PROGRESS | local | hardware.md/power-domains.md parciales |
| HOME-01E | sin Run | Claude | — | 2026-10-29 | BLOCKED_HARDWARE | — | cable Ethernet sin carrier en ambos lados; retomar 2026-10-30 |
| HOME-02 | sin Run | Claude | `claude/home-00-audit` | 2026-10-29 | IN_PROGRESS | local | ros-interfaces.md estático |
| HOME-03 | sin Run | Claude | `claude/home-00-audit` | 2026-10-29 | IN_REVIEW | local | software-audit.md, fix S-01 UNIT_PASS |
| HOME-04 | — | Antigravity | — | — | READY_OFFLINE | — | no despachado: falta autorización humana |
| HOME-10 | sin Run | Claude | — | 2026-10-29 | IN_PROGRESS | — | inventario + respaldo técnico; falta imagen completa |
| HOME-11 | — | Claude + humano | — | — | BLOCKED_APPROVAL | — | requiere reflasheo; ver jetson-inventory.md |

## Bloqueos de hardware declarados

Jetson ENCENDIDA y conectada por Ethernet; AgileX Piper apagado; cámara de muñeca Orbbec no disponible; Dynamixel/elevador no alimentados o no confirmados; sensores en NUC deben verificarse individualmente, no asumirse. Calibración hand-eye, pick/place real y ensayos físicos de actuadores no pueden declararse completos hasta ensayo autorizado. Reinstalación destructiva de Jetson pendiente de aprobación de destino y backup.

## Condiciones de DONE por cada tarea

- Cada PR/cambio declara alcance de archivos, tests ejecutados de verdad, evidencias y límites.
- Tests sin hardware deben indicar `STATIC/UNIT/SIM/NUC_VERIFIED/JETSON_VERIFIED/ETHERNET_VERIFIED`, no `HARDWARE_VERIFIED`.
- Refactor no altera interfaces públicas (topics/services/actions/frames y unidades) sin plan/aprobación.
- Documentación de diseño, ejecución y rollback actualizada por integrador.
- No cambios en navegación. No acceso Google. No ROS1. No pagos. No pushes no autorizados.
