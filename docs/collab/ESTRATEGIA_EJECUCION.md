# Nota de vigencia v3

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Se permite exploración profunda de navegación y hoverboard, pero no modificar ni desplegar navegación.** La autoridad detallada está en `AGENTS.md`; los datos históricos pueden estar obsoletos.

---

# Estrategia de ejecución profesional para Claude + Antigravity · Orca

## Objetivo

Que ambos agentes avancen hacia un robot competitivo y mantenible de RoboCup@Home. **Autonomía no es anarquía:** más capacidad para desarrollar y probar software, control estricto de acceso al robot físico y pruebas científicamente trazables.

## 1. Flujo por ciclos verticales

Cada ciclo debe producir una mejora observable (bug cerrado, test añadido, interfaz probada, métrica) y tener un dueño único. No plantear tareas como «optimizar todo el repo» ni reescribir el robot.

**Ciclo:** `OBSERVAR → REPRODUCIR → DEFINIR BASELINE → DISEÑAR CAMBIO MÍNIMO → IMPLEMENTAR → VALIDAR OFFLINE → REVISAR → VALIDAR EN HOST → DOCS → INTEGRACIÓN → HANDOFF`.

- Una tarea = máximo un subsistema principal; APIs y versiones explícitas; idealmente pruebas independientes y branch corta.
- Primero capturar comportamiento actual y dependencias. Si no hay métrica reproducible, optimización queda en hipótesis.
- El integrador rechaza cambios con nombres TF/topics distintos sin migración; ningún cambio en navegación core.

## 2. Distribución recomendada de capacidad

**Claude coordinador:** arquitectura ROS 2, safety, interfaces, Jetson (cuando corresponda), auditoría de servicios, ownership RealSense, HRI backends y revisión de cambios. Responsable único de red/SSH y servicios operativos hasta delegación formal.

**Antigravity:** URDF editor/UI 3D, simulación/fixtures, tests, editor semántico, métricas de GPSR/HRI, experimentos aislados de percepción; segundo revisor en PR. No hacer cambios al mismo tiempo sobre un archivo editado por Claude.

**Reasignación:** si Antigravity informa cuota/caída o no hay worker activo, Claude toma la tarea en **nuevo worktree**, nunca sobre archivos a medio escribir en worktree ajeno. Registrar bloqueo y razón, no pagar ni entrar en Google.

## 3. Orden recomendado (sin volver a hacer lo completado)

### Sprint S0 — Seguridad y deuda operativa (prioridad máxima)
1. Verificar versiones reales NUC/Jetson, servicios ROS y estado de energía; reconciliar docs.
2. Tratar token remoto GitHub y sudo temporal; auditar `agy_autoapprove.py`; comprobar que no se reinicie CAN habilitando Piper.
3. Asegurar que `joint_states` distingue medido, estimado y sin feedback; desacoplar `view_robot` de runtime real.
4. Separar entornos python, congelar versiones GPU, import tests; proteger nodo cámara Orbbec y FPS.

**Done:** checklist de seguridad actual, cero credenciales en remotos, pruebas sin hardware, logs del servicio y rollback.

### Sprint S1 — Cierre de HRI, simulación y editores
1. `realsense-nuc` bajo ownership único (solo si no hay driver existente), test de cámara real sin motores.
2. HRI Receptionist: VAD y Whisper con micrófono RØDE real cuando disponible; alternativa pantalla; recuperación de errores, confirmaciones, tiempo de respuesta.
3. GPSR: corregir pasos omitidos en instrucciones compuestas, test de plan exacto y clasificación de errores; no mandar goals a base.
4. URDF editor: prueba visual real en navegador, límites de parámetros, export candidato, validación Xacro y colisiones; protección parámetros de Nav2.
5. Editor mapa: `semantic_map.yaml` y `waypoints.yaml` con esquema, diff/undo y validaciones; no mover robot.

**Done:** tests unitarios + e2e simulación + UI real con capturas, documentación, descriptores de versión.

### Sprint S2 — Manipulación medible
1. Recibir mediciones de columna/torso y actualizar **modelo candidato**; probar colisiones con geometría real.
2. Preparar calibration intrinsics + ArUco eye-in-hand para cámara de muñeca Orbbec y brazo Piper, residual y incertidumbre.
3. Pipeline 3D (segmentación → pose → TF → pregrasp → MoveIt2 → gripper → verificación), primero sim.
4. Sesión `LAB_SUPERVISADO` por ensayos discretos con E-stop y logs; nunca por autonomía permanente.
5. Hacer pick/place reproducible con tasas de éxito y causas de fallo, no solo vídeo de una toma.

### Sprint S3 — RoboCup readiness
1. Regla **2026.2** fijada en release, pruebas OPL asociadas a requisitos explícitos.
2. HRI Challenge, GPSR, Pick and Place y otros retos pertinentes; priorizar puntuación probable × éxito estimado / esfuerzo y riesgos.
3. Ensayo de escenario completo con comprobaciones preliminares, fallbacks, registro de errores y métricas.
4. README / documentación de despliegue, scripts reproducibles, CI offline, GitHub PRs revisados.

## 4. Quality gates

**G0 (inicio):** worktree limpio o cambio ajeno identificado; modo CASA/LAB registrado; conflictos y ownership revisados.

**G1 (software):** lint/test/CI offline, no warnings nuevos críticos, dependencias aisladas, `git diff --check`, archivos permitidos, no secrets, no cambios Nav2 core.

**G2 (host):** antes de tocar systemd/Jetson: backup, comando y rollback, riesgo de reinicio, propietario proceso, monitoreo posterior. No arranque de actuadores en CASA.

**G3 (hardware):** solo LAB presente, E-stop y preflight registrados, un ensayo y feedback real, autorización específica.

**G4 (GitHub):** remote sano, ramas/PR tras secret scan y pruebas, no publicar datos privados, nunca main directo/merge sin usuario.

## 5. Estrategias para problemas concretos

- **Orca pierde worker**: detectar worker realmente caído/limitado, registrar snapshot, cerrar dispatch huérfano sin fingir éxito, reenviar task una sola vez con contexto abreviado; si sigue caído Claude implementa y Antigravity revisa en otro momento.
- **Compactación de contexto**: un resumen técnico por tarea con SHA, recursos, secretos redactados, decision log, tests, problemas, próximos 3 pasos. Revisar HANDOFF al inicio.
- **Sistema vivo rompe una prueba**: no matar a ciegas. Mapear PID/topics/qos, identificar servicio con ownership y detener solo un proceso huérfano propio; opcional `ROS_DOMAIN_ID=42` para pruebas.
- **CPU/GPU Jetson**: medir antes y después de los cambios, comparar FPS/latencia/uso RAM, evitar reinstalar drivers de BSP y TensorRT compatibles sin plan.
- **URDF/MoveIt**: toda medida no confirmada se almacena como candidata, nunca sobrescribe parámetros seguros de navegación. Calibraciones versionadas y reproducibles.
- **Navegación**: consumir APIs de alto nivel por wrappers/test doubles; no refactor Nav2. Solamente editables mapas semánticos y waypoints definidos en `AGENTS.md`.
- **Limpieza**: remover archivos duplicados solo con búsqueda de referencias, tests y plan de deprecación; no usar `git clean -fdx` o `rm -rf` sobre directorios ajenos.

## 6. KPIs de ingeniería

Tasa de tests, duración y tasa de éxito de build, crashes/servicio, FPS cámara, latencia ASR y VAD, WER en conjunto permitido, memoria/CPU Jetson, plan accuracy GPSR, precisión de TF/hand-eye (mm, deg, reproyección), % éxito pick/place, errores recuperados, incidentes de seguridad, tiempos de instalación/rollback y porcentaje de documentación con fuente. Medir sin inventar resultados.

## 7. Enlaces útiles

- Repo: https://github.com/ShiiSuii/base_athome
- Rulebook 2026.2: https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2
- GitHub RoboCup@Home: https://github.com/RoboCupAtHome
- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/
- ROS 2 launch testing: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Testing/Integration.html
- MoveIt 2: https://moveit.picknik.ai/main/index.html
- NVIDIA Jetson: https://docs.nvidia.com/jetson/
- Orca: https://www.onorca.dev/docs/cli/orchestration
- Claude permissions: https://code.claude.com/docs/en/permissions
