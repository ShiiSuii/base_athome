# RoboCup@Home OPL — priorización por habilidades, no por demos

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

## Normativa

Fijar referencia para 2026 en https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2 (release estático, no `master` 2027). No garantizar que el campeonato futuro use exactamente 2026.2: antes de competir confirmar convocatoria/reglamento aplicables. No inventar nombre, score ni existencia de challenge sin citar sección precisa de la versión fijada.

## Capas reutilizables

1. **Percepción:** detector persona/objeto, segmentación RGB-D, proyección profundidad, transformaciones calibradas, estimador pose, reconocimiento opcional con consentimiento.
2. **Voz/HRI:** VAD y EOT, ASR Whisper, confirmaciones, manejo de repeticiones/pausas, TTS, interfaz táctil fallback, detección timbre/knock, atención a invitados.
3. **Orquestación de habilidades:** FSM/behavior tree, scheduler, timeout, precondiciones, recoveries, estado explícito, seguridad y logging.
4. **Navegación:** cliente de Nav2 vía action existente; **no tocar la pila Nav2**, solo utilizar interfaz publicada y adaptadores de tareas (mock en pruebas).
5. **Manipulación:** Piper, MoveIt2, Orbbec, transformaciones eye-in-hand, ArUco, escena de colisión, agarre por feedback y retirada segura.
6. **Plataforma:** editor URDF/Xacro, simulación Gazebo Harmonic, tests e integración de NUC-Jetson.

## Pruebas técnicas prioritarias

| Prueba/función | Estado reportado 30/10 | Siguiente verificación | No afirmar listo hasta |
|---|---|---|---|
| Receptionist/HRI Challenge | FSM y detección de anfitrión refinadas; simulador con audio/pantalla | probar VAD/EOT en RØDE real, voluntarios con consentimiento, stress tests de cambios de estado | ensayos de punta a punta supervisados |
| Entrada puerta | detector puerta abierta, tests simulados | comprobar sensor/señal con datos reales sin mover base | detección reproducible en arena |
| GPSR | parser integrado, necesidad de recuperar pasos en órdenes compuestas | dataset sintético con plan exacto, negative cases | tasa de planes completos + ejecución segura |
| Manipulación | Piper driver, guardas feedback, ARUCO a completar | hand-eye con protocolo, validación TF, colisión del torso | pick/place de objetos objetivo por series repetidas |
| Mapa semántico | editor de mapas/waypoints en desarrollo | verificar YAML/constraints y goals solo mock | humano revise ubicaciones reales |
| URDF localhost | backend editor reportado con 45 tests | preview navegador, export no destructivo, colisiones | tests UI y geometría reales |
| Simulación | `base_athome_sim` + Gazebo reportados | CI determinista, contratos iguales al real | tests sin falsa detección de hardware |

**Priorizar la facilidad de demostración** según esfuerzo/riesgo/equipo, no únicamente puntuaciones publicadas. Reutilizar infraestructura existente. Mantener una tabla requisito (regla sección y página) → habilidad → test → artefacto → estado.

## Protocolo de validación

- `STATIC_PASS`: inspección/validación de tipo/esquema.
- `UNIT_PASS`: pytest/ament test sin hardware.
- `SIM_PASS`: integración en `ROS_DOMAIN_ID=42` con mocks/Gazebo.
- `NUC_VERIFIED`, `JETSON_VERIFIED`, `ETHERNET_VERIFIED`: lectura/config y tests inocuos con host real.
- `HARDWARE_VERIFIED`: prueba real, supervisada, con logs y métricas.
- `COMPETITION_READY`: requiere escenarios equivalentes a reglas y evaluación repetida, no se deduce por tests unitarios.

**Privacidad:** no subir audios, rostros, identidades, videos de personas ni datos biométricos a repositorios públicos; utilizar datos de consentimiento y retención según protocolo de investigación.
