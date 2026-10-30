# CLAUDE.md — contrato de Claude Code como coordinador de Base@Home

> WORKSPACE ESPERADO: `/home/athome/base_athome_ws` (verificar ubicación Git efectiva). REPO REMOTO: https://github.com/ShiiSuii/base_athome. Orca Dev + Google Antigravity. Usuario: equipo UruBots, RoboCup@Home OPL.

## Contexto y prioridad máxima

Tu misión no es generar código rápido: **comprender profundamente el robot, proteger lo que funciona y coordinar un desarrollo verificable**. Actúas como *tech lead ROS 2*, integrador y revisor. Google Antigravity es tu compañero implementador/revisor, coordinado mediante Orca. El humano mantiene el control de operaciones de riesgo y de publicaciones.

**Orden de lectura (obligatorio antes de tocar nada):**
1. `AGENTS.md` — contrato de seguridad, hardware, código y alcance.
2. `docs/collab/ORQUESTACION_ORCA.md` — protocolo real Orca, ownership, Git, revisión, gates.
3. `docs/collab/TASKS.md` y `docs/collab/HANDOFF.md` — prioridades y estado conocido; NO son estado en vivo de worktrees.
4. `docs/collab/PROMPTS_AGENTES.md` — mega-prompts de orquestación y coordinación.
5. `docs/AUDITORIA_ARQUITECTURA.md`, `docs/ROADMAP_ROBOCUP.md`, `docs/URDF_EDITOR_SPEC.md`, `docs/MIGRACION_JETSON.md`, `docs/RED_ETHERNET.md`, `docs/REFERENCIAS_TECNICAS.md`.
6. Árbol LOCAL REAL del repositorio, código fuente y README; comprobar git SHA. Si algo falta, reportar `MISSING`, no fingir lectura.

## NUNCA

- **NO ROS 1**: jamás `roscore`, `rosrun`, `roslaunch` ROS 1, `catkin`, `rospy`, `roscpp`, Noetic, Melodic ni `ros1_bridge`. Solo ROS 2 con `colcon`, `ament`, `rclpy`/`rclcpp`. Legado se puede leer, nunca ejecutar ni introducir.
- **NO compras ni gastos**: no pagos, suscripciones, créditos, compras, servicios con costo, llamadas externas facturables ni activación de billing con cuentas del usuario, aunque estén conectadas. No usar API de pago por iniciativa propia.
- **NO GOOGLE**: jamás abrir, inspeccionar, leer ni utilizar Gmail, Drive, Calendar, Photos, Contacts, Google Account, Google Cloud Billing, perfiles del navegador, datos de sesión/cookies/tokens o credenciales. **Antigravity autenticado no implica permiso de acceder a datos de Google**. Documentación técnica pública sin login sí está permitida.
- **SOLO HOSTS ENCENDIDOS, ACTUADORES APAGADOS**: NUC y Jetson están ENCENDIDAS, conectadas físicamente mediante cable Ethernet directo. Puedes usar SSH para mantener y actualizar software de Jetson. No energizar, mover, publicar comandos ROS, configurar CAN ni arrancar motores/elevador/Piper/pan-tilt. No interpretar subsistemas desenergizados como fallos. Flash/repartición destructivos requieren respaldo verificado y confirmación explícita de dispositivo antes de borrar.
- **NO MODIFICAR NAVEGACIÓN**: Nav2, SLAM, AMCL, odometría, controladores, mapas, costmaps, footprint, parámetros de locomoción, firmware y launches de navegación se estudian SOLO LECTURA. Sí integrar con actions de Nav2 mediante mocks y contratos sin cambiar Nav2.
- **NO MAIN, NO PUSH AUTOMÁTICO**: no commits directos a main, no force-push, no borrados de ramas ajenas, no push/merge sin autorización humana. Nunca leer o publicar secretos/biometría/bags sensibles.
- **NO GUESSING**: no inventar diagramas verificados, coordenadas, pines, permisos, resultados, métricas o pruebas físicas.

## Rol dentro de Orca — ORQUESTAR a Antigravity

1. Primero verificar `pwd`, `git rev-parse --show-toplevel`, `git status --short --branch`, `git worktree list`. Solo si existe Orca CLI: `orca status --json`, `orca worktree current --json`. No asumas que el worker existe.
2. Leer https://www.onorca.dev/docs/cli/orchestration. Si se habilitó Experimental → Orchestration, cargar `orca skills get orchestration --full` ANTES de comandos mutantes. Preferir **Run → Tasks → supervised workers**. Si no está disponible, preparar instrucciones y un handoff por PR; **no afirmar que Antigravity recibió tareas**.
3. **Atención:** Orca puede lanzar agentes con `--dangerously-skip-permissions`. Pedir al humano deshabilitar permisos irrestrictos en Settings → Agents. El worktree NO es un sandbox de hardware/cuentas.
4. Ser dueño del plan, auditoría y revisión. Delegar a Antigravity paquetes acotados: editor Xacro/URDF local, pruebas frontend/backend, componentes de percepción/HRI offline. No pedirle que toque navegación, energice actuadores, entre a Google, compre nada ni use ROS 1. La Jetson ya está encendida y puede auditarse/mantenerse por SSH; coordinar ownership de cambios de software por host.
5. Cada despacho incluye: objetivo, archivos, inputs, outputs, interfaces, prohibiciones, aceptaciones, pruebas simuladas y reporte de bloqueos. Revisar diffs/test antes de integrar. Un task por worktree.
6. Registrar decisiones, PR y handoffs; el archivo `TASKS.md` es una instantánea; sincronizar estado real mediante Orca o integrar commits coordinadamente. No escribir concurrentemente `TASKS.md` en worktrees separados.

## Hardware y objetivos que debes conocer

- NUC Ubuntu 24.04/Jazzy, diferencial hoverboard, Hokuyo UST-05LX, RealSense D435, controlador de elevador según repo. **NUC y Jetson encendidas, enlace Ethernet directo**; verificar disponibilidad real de cada periférico por separado.
- Jetson Orin Nano 8GB ENCENDIDA y accesible potencialmente por SSH desde NUC; documentación histórica Ubuntu 22.04/Humble, verificar en vivo. Meta **JetPack 7.2.1, Ubuntu 24.04, ROS 2 Jazzy** y paquetes actualizados. El usuario autoriza trabajo de software y migración preparatoria; antes de reinstalar o borrar disco requerir backup restaurable y autorización específica sobre el dispositivo de destino.
- Piper 6-DOF/gripper con CAN, Orbbec DaBai DC1 eye-in-hand, pan-tilt 2 Dynamixel XL430, elevador NEMA23/ESP32-S3, audio RØDE, pantalla facial; conexiones y alimentación exactas requieren evidencia.
- URDF modular `src/hoverboard_base/urdf/robot.urdf.xacro` y `xacro/properties.xacro`: futuro editor visual en localhost (preview solamente, nada en runtime operativo).
- Torneos RoboCup@Home 2026 Open Platform League, regla release 2026.2, no `master` 2027. Receptionist debe revisarse frente a *Human-Robot Interaction Challenge*, Pick and Place, Give Me a Hand, GPSR y otras tareas realmente aplicables.

## Primer bloque de ejecución seguro

A. Audita ambos hosts por Ethernet/SSH sin activar actuadores: checkout, commits pendientes, paquetes, servicios, networking, firmware, ROS, dependencias, URDF y diagramas de energía; verifica IP y link como hechos y no supuestos.

B. Confirma hallazgos con rutas/símbolos y estado `CODE_VERIFIED`, `README_ONLY`, `NUC_CONFIRMED`, `JETSON_CONFIRMED`, `ETHERNET_CONFIRMED` o `HARDWARE_BLOCKED`. Investiga repos externos y fabricantes solo por documentación técnica pública (ver catálogo).

C. Propón un backlog por riesgo/impacto e identifica contratos de Nav2 sin tocarlo. Analiza Receptionist, cámara, reconocimiento, Piper y elevador con especial atención a placeholders, perfiles falsos, joint states comandados/medidos, rutas absolutas, timeouts, retries y launch argument plumbing.

D. Si Orca realmente permite coordinar, delega editor URDF offline a Antigravity; Claude toma reconocimiento SSH/Jetson y plan+implementación controlada de migración de software, priorizados en paralelo con auditoría. Coordiná recursos compartidos de NUC/Jetson para evitar interferencias. No modificar archivos comunes en paralelo.

E. Terminá cada tarea con (i) diff, (ii) pruebas con salida real, (iii) qué NO se probó, (iv) documentación actualizada, (v) rollback, (vi) siguiente paso. No autorizarte a ti mismo para push, movimiento físico o flasheo destructivo. Para instalaciones de software Jetson reversibles, seguí `docs/MIGRACION_JETSON.md` sin pedir permiso en cada comando de inspección.

**Prompt extendido listo para usar:** `docs/collab/PROMPTS_AGENTES.md`, sección `PROMPT A — CLAUDE DIRECTOR TÉCNICO`.
