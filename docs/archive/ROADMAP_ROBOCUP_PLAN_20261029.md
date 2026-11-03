# DOCUMENTO HISTÓRICO NO EJECUTAR

> Esta versión inicial conserva tareas que después fueron completadas. Consulte `../ROADMAP_ROBOCUP.md` y `../collab/HANDOFF.md` para la política vigente.

# Roadmap técnico Base@Home — RoboCup@Home

Fecha base: 2026-10-29. Este documento define pendientes, NO asegura que se hayan implementado. Las prioridades podrán cambiar tras auditoría del checkout actual.

## Estado operativo TEMPORAL (confirmado por el usuario)

**NUC Y JETSON ORIN NANO ESTÁN ENCENDIDAS Y CONECTADAS MEDIANTE CABLE ETHERNET DIRECTO.** Los demás subsistemas (Piper, pan-tilt, elevador, motores y fuentes de actuadores) están desenergizados. Está permitido el diagnóstico por SSH, mantenimiento de software y migración preparatoria de Jetson; no activar controladores ni ejecutar movimientos. Marcar como `HARDWARE_BLOCKED` toda prueba física que requiera alimentar otros subsistemas. Navegación declarada operativa y protegida para cambios.

**NUEVO ORDEN:** primero conocimiento exhaustivo del robot y auditoría/refactor seguro; después tareas de HRI, editor URDF, manipulación y migración Jetson según dependencias. Ver `docs/AUDITORIA_ARQUITECTURA.md`.

> **Actualización 2026-10-30:** reglamento 2026.2 leído completo (`docs/RULEBOOK_2026_2_ANALISIS.md`): pruebas = HRI, Pick and Place, GPSR, Doing Laundry, Restaurant + Final. *Give Me a Hand* no existe en 2026.2. El cable NUC↔Jetson no tiene link (la Jetson se accedió por WiFi).

## Estado inicial y restricciones

- NUC con Ubuntu 24.04 + ROS 2 Jazzy; navegación existente declarada operativa por el usuario. **Congelada para modificaciones.**
- Jetson Orin Nano 8 GB documentada en Ubuntu 22.04 + ROS 2 Humble: validar versión real antes de migrar.
- Brazo AgileX Piper + Orbbec eye-in-hand: pick-and-place aún no es completamente confiable; calibración ArUco/hand-eye pendiente de verificación.
- URDF Xacro modular; modelo mecánico aún en construcción/ajuste. Editor web paramétrico solicitado.
- Orin/NUC y endpoints ROS: comprobar datos reales de README mediante comandos no destructivos.

## P0-A. Comprensión profunda de la arquitectura de hardware y software

- [ ] `TODO` Leer árbol de código de punta a punta: `src/`, `firmware/`, `jetson_head/`, `scripts/`, `launch/`, YAML, dependencias, URDF/Xacro, README y logs guardados.
- [x] `IN_PROGRESS` Crear `docs/hardware.md`: inventario de NUC, Jetson, motores de base, LiDAR, RealSense, Piper, Orbbec, Dynamixel, RØDE, pantalla, ESP32-S3 y elevador con modelos, buses, conectores, host propietario y estado `VERIFICADO_CODIGO / DOCUMENTADO / NUC_CONFIRMADO / PENDIENTE`.
- [x] `IN_PROGRESS` (sin evidencia eléctrica) Crear `docs/power-domains.md`: reconstruir baterías, BMS, fusibles, step-down y distribución por subsistema. Registrar como hipótesis toda configuración histórica no corroborada; no energizar ni medir peligrosamente.
- [x] `DONE` (1ª pasada) Crear `docs/architecture.md`: arquitectura por host, buses, flujo extremo a extremo de tareas, diagrama de bloques y dependencias entre percepción, HRI, navegación (solo lectura), manipulación y elevador.
- [x] `IN_PROGRESS` (estático) Crear `docs/ros-interfaces.md`: cada nodo, paquete, launch, topic, tipo, QoS, servicio, action, TF, frecuencia/timeout esperado, publisher/consumer y dependencia física.
- [ ] `TODO` Localizar firmware, comandos seriales/CAN, límites, feedback y watchdogs para documentar sus contratos **sin controlarlos**.
- [x] `DONE` (Jetson por WiFi) Guardar evidencia de diagnósticos no invasivos de NUC y Jetson por SSH y separar resultados de software y pruebas de actuadores pendientes.
- [ ] `BLOCKED` (sin carrier; herramienta lista) Documentar y comprobar cable Ethernet NUC↔Jetson, IP/hosts, SSH, rutas, MTU, CycloneDDS, `ROS_DOMAIN_ID`, sincronización temporal y prueba ROS 2 aislada, siguiendo `docs/RED_ETHERNET.md`.

**Aceptación:** un desarrollador nuevo puede explicar qué hace cada subsistema, cómo se comunica, cómo se alimenta y qué ejecuta cada host; todo dato no verificado está identificado como tal. Ninguna activación de hardware ni cambio a navegación.

## P0-B. Auditoría de calidad y reorganización del código

- [x] `DONE` (1ª pasada) Crear `docs/software-audit.md` con mapa de dependencias, duplicaciones, rutas absolutas, valores mágicos, config dispersa, procesos que compiten por sensores, acoplamiento y riesgos.
- [ ] `TODO` Revisar estructura ROS: responsabilidades de paquetes, launch, configuración, separación drivers/lógica/HRI/UI y dependencias Python/C++ (sin romper APIs).
- [ ] `TODO` Revisar callbacks bloqueantes, concurrency, QoS, reconexión, excepciones, gestión de recursos, logging, timeouts, watchdogs, estados de fallo y seguridad.
- [ ] `TODO` Medir baseline solo donde sea seguro y posible en NUC y Jetson: CPU/RAM, latencia y tiempos de proceso; lo que requiera sensores y actuadores apagados queda `HARDWARE_BLOCKED`.
- [ ] `TODO` Priorizar oportunidades de optimización con impacto/beneficio/riesgo; crear tests de regresión antes de refactorizar y cambios incrementales por PR.
- [x] `IN_PROGRESS` (Piper, pan-tilt, HRI, puerta) Añadir mocks y tests offline para evitar que importar módulos intente abrir CAN, serial, cámaras o controlar actuadores.
- [ ] `TODO` Documentar cambios de estructura e instalación en el mismo PR; evitar mover archivos sin pruebas de imports, entrypoints y launch.

**Aceptación:** backlog de mejoras justificado, primeras correcciones seguras con pruebas offline y comparación antes/después. No reorganizaciones masivas sin revisión, sin tocar navegación.

## P0. Reconocimiento y estabilidad del software

- [ ] `TODO` Contrastar arquitectura e inventario inicial de P0-A con el checkout y la NUC; mapear nodos, topics, services, actions y TF; conservar arquitectura actual.
- [ ] `TODO` Auditar la ejecución de Receptionist, ownership de RealSense/Orbbec, argumentos de launch, modos de simulación y rutas absolutas.
- [ ] `IN_PROGRESS` (S-07 documentado) Revisar estados articulares del pan-tilt y brazo: diferenciar target vs encoder/feedback.
- [x] `DONE` (S-04 corregido) Comprobar si las funciones de reconocimiento facial hacen enrolamiento real o usan placeholders; separar datos ficticios de personas reales.
- [ ] `TODO` Añadir smoke tests reproducibles sin encender motores ni modificar Nav2.

**Aceptación:** mapa verificable de arquitectura y comandos de comprobación con evidencia, riesgos conocidos y documentación actualizada.

## P1-A. Manipulación Piper + Orbbec

- [ ] `TODO` Inspeccionar URDF/SRDF, descripción oficial AgileX, offsets del brazo, gripper y cámara de muñeca; no alterar límites del fabricante.
- [ ] `TODO` Hacer protocolo ArUco hand-eye (`eye-in-hand`) con tablero fijo y poses diversas, timestamp/extrínsecos, calidad de detección y rechazo de outliers.
- [ ] `TODO` Verificar manualmente transformación `base -> arm -> tool -> wrist_camera`, frames ópticos y escala/profundidad.
- [ ] `TODO` Guardar calibración con versión, fecha, método, cantidad de tomas y residual; reproducir sobre puntos de prueba.
- [ ] `TODO` Plan de colisiones MoveIt 2 con torso, columna, mobiliario y límites reales del elevador; sin autoejecutar sobre hardware.
- [ ] `TODO` Detección y estimación 3D de objetos (RGB-D); agarre por geometría simple y mejora progresiva con segmentación si aporta robustez.
- [ ] `TODO` Implementar pick-and-place básico con un objeto fácil y una mesa conocida; añadir estados de recuperación por agarre fallido.
- [ ] `TODO` Medir éxitos por intento, repetibilidad, tiempo, errores de pose y causas de fallo; documentar vídeos/fotos sin datos personales.

**Aceptación:** calibración reproducible, pruebas offline y, tras autorización, ensayo físico repetido con resultados medidos y mecanismo seguro de parada.

## P1-B. Editor visual URDF (localhost)

- [x] `DONE` Auditar Xacro y su pipeline real de generación URDF antes de crear el editor.
- [ ] `TODO` Panel web editable con preview 3D, selección de partes, medidas en m/cm, controles `deshacer`, `restaurar`, `guardar preset`.
- [ ] `TODO` Separar parámetros de geometría visual editables y parámetros calibrados protegidos.
- [x] `OFFLINE_DONE` (HOME-04) API local que genera URDF con Xacro real, valida sintaxis/topología/medidas y preserva las expresiones derivadas.
- [ ] `TODO` Generación de candidatura exportable sin aplicar cambios a modelo de producción ni publicarlos a `/robot_description`.
- [ ] `TODO` Tests backend/frontend con valores inválidos, path traversal, ausencia de meshes y diferencias respecto al modelo original.

**Aceptación:** seguir íntegramente `docs/URDF_EDITOR_SPEC.md`.

## P1-C. Actualización Jetson a Ubuntu 24.04 y ROS 2 Jazzy

**Fase 0 (ejecutar ahora por SSH entre hosts):**
- [ ] `TODO` Verificar placa/portadora, versión JetPack, Linux/L4T, UEFI/QSPI, almacenamiento, paquetes CUDA/TensorRT, Python y dependencias de cámara, audio, CAN, Dynamixel y ROS.
- [ ] `TODO` Respaldar usuarios, código, parámetros, servicios systemd, entornos, firmware, datos de calibración y paquetes instalados en soporte externo verificable.
- [ ] `TODO` Elaborar matriz de compatibilidad con JetPack 7.2.1 / Jetson Linux 39.2.1 y Ubuntu 24.04. Verificar dependencias compatibles con ARM64; **no asumir que todos los paquetes de IA existentes siguen funcionando**.
- [ ] `TODO` Plan de reinstalación/rollback, soporte físico, identificación de almacenamiento y gate de autorización específico para cualquier paso de borrado/flasheo.

**Fase 1 (software permitido; reflasheo/firmware solo con respaldo y aprobación específica):**
- [ ] `BLOCKED_APPROVAL` Seguir documentación oficial Jetson ISO (USB de instalación, NO imagen SD directa) y verificar requisito de UEFI/QSPI actualizado desde JetPack 6.x; paso de borrado condicionado a backup probado y confirmación de almacenamiento.
- [ ] `TODO` Actualizar paquetes compatibles y preparar restauración de componentes NVIDIA/controladores; ejecución posterior al cambio de BSP si corresponde, sin alterar NUC.

**Fase 2 (ROS 2 Jazzy, sobre Ubuntu 24.04 compatible):**
- [ ] `TODO` Instalar Jazzy cuando OS compatible, reconstruir workspace Jetson y migrar launch/deps/venvs, comprobar parámetros de DDS y clock.
- [ ] `TODO` Verificar ROS 2 Jazzy NUC↔Jetson con nodos de prueba inocuos. `BLOCKED_HARDWARE` para audio/cámara/CAN/Piper/pan-tilt/USB y sensores o actuadores físicamente desenergizados.
- [ ] `TODO` Documentar comandos y comparación antes/después con plan de reversión.

**Aceptación:** operación de Jetson sobre Ubuntu 24.04 y Jazzy verificada en hardware y comunicación NUC sin regresiones de navegación. La migración es una tarea solicitada y autorizada de software; reinstalar el sistema/borrar almacenamiento exige gate de respaldo y aprobación específica.

Fuentes: https://developer.nvidia.com/embedded/jetpack/downloads y https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/quick_start.html

## P2. HRI Challenge y habilidades de competición

**Reglas vigentes de referencia:** release 2026.2, no `master` (actualmente puede estar adelantado hacia 2027). https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2

- [ ] `TODO` Actualizar Receptionist a Human-Robot Interaction Challenge: timbre/golpes; nombres libres, preferencias, preguntas, confirmación; reconocimiento/seguimiento de personas; introducciones y asientos; fase de seguimiento/handover conforme reglas verificadas.
- [ ] `TODO` Separar habilidades reutilizables (ASR, TTS, VAD, persona/objeto, señalar, seguir, handover, pick, place, navegar a waypoint) de la máquina de estados de cada tarea.
- [ ] `TODO` Priorizar pruebas: percepción y saludo de persona → manipulación de objeto en mesa → handover → Pick and Place Challenge → Give Me a Hand → GPSR; reordenar según puntos/esfuerzo reales del rulebook y habilidades disponibles.
- [ ] `TODO` Preparar estado degradado y recuperación de fallos sin bloquear permanentemente una prueba.
- [ ] `TODO` Definir checklist de precompetición, modo demo, logs, métricas por repetición y documentación/video de resultados.

**Aceptación:** cada habilidad tiene tests y una demostración verificable; una prueba completa se marca lograda solo tras ejecución integral.

## Política de validación con NUC + Jetson encendidas y actuadores apagados

- **Ejecutable ahora:** Git/lectura de código de ambos hosts; SSH Jetson por Ethernet; inventario de JetPack/UEFI/OS/ROS/servicios, respaldos y actualizaciones reversibles de software; pruebas de red/CycloneDDS y ROS2 en dominio aislado; diagramas; Xacro/URDF offline; pruebas unitarias; mocks y benchmarks de bajo impacto.
- **Bloqueado:** motores, CAN/Piper, elevador, Dynamixel y periféricos que requieran alimentación independiente, calibración física ArUco/hand-eye, manipulación real y ensayos actuados; reflasheo pendiente de backup y confirmación explícita.
- **Regla de estado:** `STATIC_PASS` / `UNIT_PASS` / `SIM_PASS` / `NUC_CHECKED` / `JETSON_CHECKED` / `ETHERNET_CHECKED` no equivale a `HARDWARE_VERIFIED`.
- **Cuando se alimenten los demás subsistemas:** planificar encendido escalonado y checklist con autorización; verificar componentes, luego integración y por último tareas, preservando navegación.

## P3. Mantenimiento continuo GitHub y documentación

- [ ] `TODO` Mantener los mapas y tablas de P0-A actualizados a cada cambio de implementación, sin reemplazar datos físicos por suposiciones.
- [ ] `TODO` Crear `docs/hardware.md`, `docs/calibration.md`, `docs/installation-nuc.md`, `docs/installation-jetson.md`, `docs/test-results.md` cuando existan datos respaldatorios.
- [ ] `TODO` Establecer CI: formato, lint, tests unitarios, validación Xacro/URDF sin hardware y checks de documentación.
- [ ] `TODO` README con instalación reproducible, arranque seguro y tabla de funcionalidades **verificadas**.
- [ ] `TODO` PR y registro de cambios por funcionalidad, revisión humana antes de merge; al integrar, sincronizar documentación y roadmap.

## Plantilla breve de informe por tarea

| Campo | A completar |
| --- | --- |
| ID / responsable / agente | |
| Estado (`TODO/IN_PROGRESS/BLOCKED/TESTED/DONE`) | |
| Branch / PR / commit | |
| Objetivo y archivos | |
| Riesgos sobre hardware/nav | |
| Evidencia de pruebas con comandos | |
| Documentación actualizada | |
| Limitaciones, rollback y próximos pasos | |
