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
| HOME-13 | P2 | ~~Give Me a Hand~~ (no existe en 2026.2) → Doing Laundry: estudio de factibilidad con un brazo | Claude | HOME-08 | TODO |
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
| HOME-00 | run_8db8fe9ab48b | Claude | `claude/home-00-audit` | 2026-10-29 | OFFLINE_DONE | local | reconocimiento Git/Orca/hosts |
| HOME-01 | run_8db8fe9ab48b | Claude | `claude/home-00-audit` | 2026-10-30 | IN_PROGRESS | local | hardware.md, power-domains.md, HARDWARE_OPTIMIZACION.md |
| HOME-01E | run_8db8fe9ab48b | Claude | — | 2026-10-30 | ETHERNET_CONFIRMED | local | enlace por USB RTL8153 de la Jetson (enP8p1s0 sin carrier); check 8/8 OK; DDS dominio 77 OK ambos sentidos |
| HOME-02 | run_8db8fe9ab48b | Claude | `claude/home-00-audit` | 2026-10-29 | IN_PROGRESS | local | ros-interfaces.md estático; tasas reales al tener link |
| HOME-03 | run_8db8fe9ab48b | Claude | `claude/home-00-audit` | 2026-10-30 | IN_REVIEW | local | S-01..S-05 implementados con tests; S-01/S-03 desplegados en Jetson |
| HOME-04 | task_dd025d65b38a | Antigravity→Claude worker (cuota) | `ShiiSuii/home-04-urdf-editor` | 2026-10-30 | OFFLINE_DONE | 3a957f1 (merge 3640193) | 45 tests; falta prueba visual en navegador |
| HOME-06a | task_d7971ec8bb2a | Antigravity (Gemini 3.8 Flash) | `ShiiSuii/home-06a-hri-gap` | 2026-10-30 | OFFLINE_DONE | 39aced5 | gap analysis HRI 2026.2 |
| HOME-06b | — | Claude | `claude/home-00-audit` | 2026-10-30 | OFFLINE_DONE | f56238d | presentación mutua con mirada + torque pan-tilt; 5 tests |
| HOME-10 | run_8db8fe9ab48b | Claude | — | 2026-10-30 | IN_PROGRESS | — | inventario, respaldo técnico, fixes de seguridad desplegados y verificados en vivo; S-12 cámara muñeca 5→20 Hz desplegado |
| HOME-11 | — | Claude + humano | — | — | BLOCKED_APPROVAL | — | reflasheo; recomendado NVMe nuevo |
| HOME-13 | — | — | — | — | CANCELADA | — | "Give Me a Hand" no existe en 2026.2 → reemplazar por Doing Laundry |
| HOME-15a | task_02674e4e224b | Antigravity (Gemini 3.8 Flash) | `antigravity-home-15a-gpsr` | 2026-10-30 | IN_PROGRESS | — | parser GPSR offline + dataset CommandGenerator |
| HOME-19 | — | Claude | `claude/home-00-audit` | 2026-10-30 | SIM_PASS | e37d199 | señal de inicio por puerta abierta (`base_athome_skills`) |

## Bloqueos de hardware declarados

Jetson ENCENDIDA y conectada por Ethernet; AgileX Piper apagado; cámara de muñeca Orbbec DaBai publicando 20 Hz (2026-10-30); Dynamixel/elevador no alimentados o no confirmados; sensores en NUC deben verificarse individualmente, no asumirse. Calibración hand-eye, pick/place real y ensayos físicos de actuadores no pueden declararse completos hasta ensayo autorizado. Reinstalación destructiva de Jetson pendiente de aprobación de destino y backup.

## Condiciones de DONE por cada tarea

- Cada PR/cambio declara alcance de archivos, tests ejecutados de verdad, evidencias y límites.
- Tests sin hardware deben indicar `STATIC/UNIT/SIM/NUC_VERIFIED/JETSON_VERIFIED/ETHERNET_VERIFIED`, no `HARDWARE_VERIFIED`.
- Refactor no altera interfaces públicas (topics/services/actions/frames y unidades) sin plan/aprobación.
- Documentación de diseño, ejecución y rollback actualizada por integrador.
- No cambios en navegación. No acceso Google. No ROS1. No pagos. No pushes no autorizados.
| HOME-15a | run_8db8fe9ab48b | Antigravity | ctx_a61d1dad86ff | 2026-10-30 | INTEGRATED | merge 84b3c7c | parser GPSR offline, 34 tests; métrica de primera acción débil |
| HOME-15b | — | por asignar | — | 2026-10-30 | READY_OFFLINE | — | corregir pasos perdidos en comandos compuestos; dataset con planes esperados y métrica de plan exacto |
| HOME-11 | run_8db8fe9ab48b | Claude (+humano recovery) | — | 2026-10-30 | DONE (validación sin actuadores) | — | preparado: ISO r39.2.1 en la NUC, respaldos extra, bootstrap_jazzy.sh, runbook; requiere humano con monitor/teclado y destino (NVMe recomendado) |


---

## Actualización del plan v3 — nuevas tareas (03/11/2026, propuestas, no dispatch)

| ID | Prioridad | Tarea | Responsable sugerido | Permiso | Criterio observable |
|---|---|---|---|---|---|
| HOME-30 | P0 | Auditoría profunda **solo lectura** de hoverboard→odometría→TF→Nav2 | Claude | lectura/diagnóstico pasivo/simulación | `docs/AUDITORIA_NAVEGACION.md` con evidencias reales y dudas |
| HOME-31 | P1 | Investigación de bases hoverboard ROS 2; comparar 5–8 diseños y motores/controladores | Antigravity | web pública/docs/código, sin hardware | matriz con links/licencias y fichas ADOPTAR/ADAPTAR/NO |
| HOME-32 | P1 | Análisis independiente de propuestas de navegación y riesgos (no implementación) | Claude + review Antigravity | propuestas únicamente | entradas en `docs/collab/PROPUESTAS_NAVEGACION.md` |
| HOME-33 | P1 | Reconciliar datos 30/10 con hosts, ramas y estado Git actuales | Claude | comandos pasivos, sin cambios Nav2 | inventario real con SHA, estado Jetson y backlog |
| HOME-34 | P1 | Consolidar QA / tests y limpiar deuda técnica no navegacional | ambos, scopes separados | software aislado | tests reproducibles, regresiones y benchmark |

**Importante:** HOME-11, HOME-04, HOME-15a y otros avances anteriores tienen estados contradictorios entre filas históricas: no ejecutarlos de nuevo antes de reconciliar con el último HANDOFF y el estado de Git. Sólo Orca marca asignaciones activas.
