# AGENTS.md — Reglas compartidas de Base@Home (Claude Code + Google Antigravity)

Reglas únicas y vigentes para todos los agentes que trabajan en este repositorio dentro de Orca.
Reemplaza a todas las versiones anteriores (historial en Git). Ningún Markdown configura por sí solo
los permisos efectivos de Orca, Linux o Claude Code: esos los fija el host (`.claude/settings.json`,
Orca → Settings → Agents) y hay que respetarlos.

## 0. Proyecto y objetivo

Base@Home (UruBots) es un robot de servicio para **RoboCup@Home Open Platform League**: NUC
(Ubuntu 24.04, ROS 2 Jazzy) + Jetson Orin Nano (Ubuntu 24.04, Jazzy) unidas por Ethernet, base de
hoverboard, Hokuyo, RealSense, Piper 6-DOF con cámara de muñeca, pan-tilt Dynamixel, elevador y HRI
por voz. Repositorio: <https://github.com/ShiiSuii/base_athome>; workspace en la NUC
`/home/athome/base_athome_ws` (confirmar con `git rev-parse --show-toplevel`).

Trabajamos como un equipo senior de ROS 2: **primero medir, después corregir, finalmente demostrar**.
No reescribir lo que funciona. La seguridad física y la protección de credenciales están por encima
de la velocidad.

## 1. Documentos maestros

| Documento | Contenido | Se actualiza cuando… |
|---|---|---|
| `AGENTS.md` | reglas, permisos, coordinación | cambia una regla general |
| `CLAUDE.md` | arranque de Claude | cambia el procedimiento de inicio |
| `README.md` | presentación, instalación, uso | cambia la instalación o el uso |
| `docs/ROBOT.md` | arquitectura, hardware, software, navegación, calibraciones, simulación, referencias | cambia la arquitectura o se verifica un dato |
| `docs/ROADMAP.md` | objetivos, reglamento, prioridades, tareas, propuestas de navegación | aparece o se cierra una tarea |
| `docs/collab/STATUS.md` | estado vivo: rama/SHA, tareas activas, bloqueos, pruebas, decisiones, próximos pasos | se integra un avance |

Manuales especializados que se conservan porque se usan para operar: `docs/MICROFONO_ASR.md`
(cadena de voz: prueba, diagnóstico, rollback), `firmware/README.md` y `firmware/docs/PINOUTS_AND_WIRING.md`,
y los README de cada herramienta o paquete (`tools/*/README.md`, `src/*/README.md`).

**Prioridad ante contradicciones:** pedido actual del usuario > prohibiciones de §2 > este archivo >
resto de documentos maestros > historial de Git. Lo que diga una documentación vieja (p. ej. "Jetson
Humble") se verifica antes de usarlo.

## 2. Prohibiciones absolutas (no se levantan de forma implícita)

- **Solo ROS 2.** Nunca `roscore`, `rosrun`, `roslaunch` de ROS 1, catkin, `rospy`/`roscpp`, Noetic/Melodic ni `ros1_bridge`. Proyectos ROS 1 se pueden leer como referencia; se implementa siempre en ROS 2.
- **Sin Google.** Nunca abrir ni leer Gmail, Drive, Calendar, Photos, Contacts, Workspace, Cloud, cuentas, cookies, tokens o perfiles del navegador. Que Antigravity esté autenticado **no** da permiso. La documentación técnica pública sin login sí se puede consultar.
- **Sin pagos.** Nunca compras, suscripciones, upgrades, créditos, trials con cargo ni servicios facturables. Si se agota una cuota, usar otra alternativa o reasignar; nunca pulsar "upgrade".
- **Sin secretos.** No leer, imprimir, commitear ni publicar credenciales, `.env`, tokens (la URL de `origin` puede contener uno: no mostrar `git remote -v` sin redactar), llaves, audios, rostros, videos o bags con personas.
- **Sin desactivar seguridad.** No quitar `auto_enable=False` del Piper, el guard de feedback, `auto_torque=false` del pan-tilt ni interlocks; no subir límites de fuerza/velocidad/corriente para pasar una prueba; no saltar políticas con scripts, alias, `sudo` o auto-aprobadores.
- **Sin movimiento físico no autorizado** (ver §3) y **sin cambios en la navegación funcional** (ver §4).

## 3. Modos operativos (no se heredan entre sesiones)

| Modo | Condición | Permitido | Nunca |
|---|---|---|---|
| `DESARROLLO` (**por defecto**, autorizado por el usuario 2026-11-03) | NUC y Jetson encendidas; potencia de actuadores (tracción, Piper, elevador, Dynamixel) **desenergizada** | todo el desarrollo de software (§5 A): código, drivers, CAN/serial/DDS sin hardware alimentado, despliegues a la Jetson, reinicio de `jetson-head` y de servicios, sudo administrativo con respaldo, simulación, MoveIt/RViz/Gazebo, mensajes y servicios de prueba en dominios aislados (42/77) o hacia interfaces verificadas como seguras | energizar o mover actuadores; dejar comandos de movimiento pendientes; desactivar protecciones; cambiar la navegación funcional |
| `LAB_SUPERVISADO` | el humano confirma **en la sesión actual**: presencia, E-stop, energía, área libre, dispositivo y prueba concreta | lo anterior + **un** ensayo acotado de Piper, pinza o pan-tilt a velocidad conservadora, con preflight y registro | base, elevador, series autónomas, permisos permanentes |
| `COMPETENCIA` | el humano inicia un procedimiento de ensayo documentado | exactamente lo que diga ese procedimiento | cambiar código, calibración o límites durante la prueba |

**Hardware desenergizado no prohíbe desarrollar ni probar sus interfaces.** Condición permanente para los
controladores de actuadores (Piper, pan-tilt, elevador, base): arrancan **deshabilitados**, descartan
trayectorias/comandos viejos al habilitar o deshabilitar, exigen feedback válido y fresco antes de mover y
nunca publican estados medidos falsos. Los comandos de movimiento se prueban en simulación, con mocks o
contra un controlador que garantiza que no transmite (deshabilitado / sin feedback).

Preflight de cada movimiento físico (`LAB_SUPERVISADO`): quién supervisa, qué está energizado, E-stop, qué
nodo gobierna el actuador, feedback fresco (no ceros sintéticos), trayectoria libre, límites de fabricante,
cómo abortar. Una acción, leer feedback, registrar resultado. Base y elevador requieren pedido separado.
Si el humano trabaja físicamente en el robot, no reiniciar servicios de actuadores sin avisar en `STATUS.md`.

Hacia el dominio 10 (robot real) solo se publica o llama lo que se verificó seguro (p. ej. servicios de
consulta, o comandos a un controlador que los rechaza por estar deshabilitado); nunca goals de Nav2,
`cmd_vel`, ni habilitación de actuadores. `ROS_DOMAIN_ID=42` aísla el descubrimiento, **no** impide que un
driver abra USB/CAN: revisar el launch antes.

## 4. Navegación protegida

**Estudiar sí, modificar no.** La navegación funciona y es del usuario.

- **Permitido sin pedir permiso:** leer todo (Nav2, AMCL, SLAM, mapas, costmaps, BT, driver y firmware de la base, odometría, TF, launch, parámetros, logs), observar tópicos de forma pasiva y acotada (`ros2 topic hz/info/echo --once`), reproducir en simulación aislada, comparar con proyectos externos y escribir propuestas en `docs/ROADMAP.md` §6.
- **Prohibido sin autorización específica:** modificar Nav2, SLAM, AMCL, costmaps, planner, controller, BT, footprint, `map.pgm`/mapas, odometría, TF de navegación, driver o firmware de la base, parámetros de locomoción, launches de navegación; reiniciar sus servicios; enviar goals o `cmd_vel`; `ros2 param set`/`lifecycle set` sobre ellos.
- **Excepción — capa semántica:** `semantic_map.yaml`, `waypoints.yaml` y el editor de mapas (`map_zone_editor.py`, `semantic_objects.py`) son de aplicación: se editan como candidatos offline con validación, backup y diff. Coordenadas no medidas se marcan provisionales; no se recarga el mapa operativo ni se mueve la base.
- Integración desde las apps: siempre como cliente de la acción `navigate_to_pose` (mocks en simulación).
- Antes de integrar cualquier diff: `git diff --name-only` debe mostrar **cero** archivos de navegación protegidos.

## 5. Permisos por nivel de riesgo (autorización de desarrollo autónomo del usuario, 2026-11-03)

**A — Autonomía completa (no preguntar; avanzar, probar, corregir, documentar):** todo lo de software en NUC y
Jetson: leer e investigar (repos, docs públicas, terceros); crear, refactorizar y probar código no protegido;
instalar/actualizar/quitar dependencias (venvs propios con versiones fijadas; nunca `pip install -U` amplio en
producción ni exportar modelos con `~/ros2_vision_venv`); SSH y `sudo` administrativo con **respaldo previo y
rollback**; crear/modificar/reiniciar servicios systemd (incluido `jetson-head`) y reiniciar NUC/Jetson
conservando la reconexión; desplegar nodos (también los del Piper/pan-tilt) con `deploy_to_jetson.sh`;
drivers, CAN, serial y DDS con el hardware desenergizado; compilar, MoveIt 2, RViz, Gazebo, simulación;
diagnóstico y optimización de CPU/RAM/GPU; terminar con SIGTERM procesos propios huérfanos identificados por
PID (nunca `kill -9`/`pkill -f` global/`killall`, nunca procesos de navegación); capa semántica (§4); Git local,
ramas, worktrees y PR de revisión (push solo con el token revocado y remoto sin credenciales, tras tests,
`git diff --check` y búsqueda de secretos); Orca: crear tareas, lanzar, reasignar y retomar workers.

Checklist para cambios de host/servicio (no es un pedido de permiso): (a) recurso identificado (b) respaldo
(c) rollback ejecutable (d) efecto sobre NUC, Jetson, Nav2 y dominio 10 (e) prueba y monitoreo (f) registro
en `STATUS.md` sin secretos.

**C — Aprobación humana específica:** energizar o mover físicamente cualquier actuador (sesión de pruebas
acordada); desactivar protecciones contra movimientos inesperados; reflashear firmware, QSPI/UEFI, particionar o
borrar almacenamiento (la Jetson **ya está migrada**); borrar datos de forma irreversible sin respaldo; cambiar
la configuración funcional de navegación (§4); cambios de red/SSH que puedan cortar NUC↔Jetson sin vía de
recuperación; modificar la autenticación o `sudoers` sin vía de recuperación; merge a `main`, borrar ramas
compartidas, releases; sacar del laboratorio datos de personas. Siempre prohibido: §2 (ROS 1, Google, pagos,
secretos).

## 6. Coordinación en Orca

- **Claude Code:** coordinador técnico, dueño del plan, integración, revisión final, recursos de host (SSH, Jetson, servicios) y de `STATUS.md`. También implementa.
- **Antigravity:** implementa o revisa tareas acotadas en su worktree (editor URDF, simulación, tests, GPSR, HRI, investigación). Segundo revisor de los cambios de Claude cuando esté disponible.
- **Humano:** decide energía, movimiento, navegación, credenciales, `main` y acciones irreversibles.

Reglas:
1. **Verificar antes de coordinar:** `git worktree list`, rama y SHA de cada worktree, `orca status --json`. Si la orquestación (`orca skills get orchestration --full`) no está habilitada, no inventar Run/Task/IDs ni afirmar que Antigravity recibió algo.
2. **Una tarea por worktree, un dueño por archivo.** Archivos comunes (`AGENTS.md`, `CLAUDE.md`, `README.md`, `docs/*.md`, `STATUS.md`, archivos ROS compartidos) los edita solo el integrador.
3. **Los worktrees no son sandbox:** comparten ROS, USB, GPU, cámaras, red, `systemd` y cuentas. Reservar por escrito el recurso mutable (`JETSON_SSH_WRITE`, `NUC_SYSTEMD`, `REALSENSE_USB`, `ORBBEC_USB`, `PIPER_CAN`, `GPU_JETSON`, `ROS_DOMAIN_10/42`, `MAP_SEMANTIC`, `GIT_INTEGRATION`); uno a la vez.
4. **Contrato de despacho:** ID real · dueño · modo · objetivo verificable · SHA base · worktree/rama · archivos de lectura y de escritura · recurso reservado · pruebas (comando y condición) · prohibiciones (navegación, ROS 1, Google, pagos, secretos, hardware) · entregables (diff, tests, rollback) · criterio de terminado.
5. Estados: `PLANNED → DISPATCHED → ACKNOWLEDGED → IMPLEMENTING → REVIEW_READY → ACCEPTED` o `BLOCKED_WORKER`/`REJECTED`. Un `worker_done` no prueba nada: revisar `git status`, `git diff --stat`, `git diff --check`, tests y docs.
6. **Recuperación:** si un worker cae por cuota/login, registrar `BLOCKED_WORKER` con rama/SHA y Claude retoma la tarea en un worktree nuevo; nunca sobre archivos a medio escribir ajenos ni borrando worktrees con trabajo sin integrar.
7. Documentación de Orca (leer la de la versión instalada antes de usar comandos): <https://www.onorca.dev/docs/cli/orchestration>, <https://www.onorca.dev/docs/agents/supported>, <https://www.onorca.dev/docs/settings>. Permisos de Claude Code: <https://code.claude.com/docs/en/permissions>.
8. Orca debe correr sin `--dangerously-skip-permissions`/`--yolo` en hosts con hardware; nada de auto-aprobadores ciegos ni convertir una pregunta de un worker en aprobación humana. **Sí** se usa la aprobación acotada `tools/agents/agy_bounded_allow.py` (hook `PreToolUse` de Antigravity instalado en `~/.gemini/config/`): aprueba solo inspección Git, `git add/commit`, lectura, pytest, colcon y xacro dentro de los worktrees; lo demás pregunta. Ampliarla solo con tests.

## 7. Git y GitHub

- Nunca commit directo a `main`, force-push, rebase de ramas ajenas, `git reset --hard` o `git clean -fdx` sobre trabajo ajeno.
- Ramas: `claude/<id>-<tema>`, `antigravity/<id>-<tema>`. Commits pequeños con mensaje claro.
- **`development`** es la base de trabajo: se integra todo ahí, documentos de coordinación incluidos. **`main`** es la versión pública limpia: solo se actualiza con `tools/agents/publicar_main.sh` (excluye lo de `tools/agents/main_excluir.txt`) y con autorización humana. Las dos son públicas: `development` no es lugar para datos sensibles.
- Cambios sin commitear que no son tuyos (p. ej. `semantic_map.yaml`, `waypoints.yaml`, `arm_poses_and_limits.yaml`) no se stagean, sobrescriben ni descartan.
- Integración en rama de trabajo, re-ejecutando tests; merge a `main` solo con autorización humana.
- Si la configuración del host pide confirmación para `git push`, respetarla.

## 8. Ingeniería, pruebas y evidencia

1. Reconciliar antes de trabajar: estado Git, último `STATUS.md`, worktrees.
2. Cambios mínimos con tests (reproducir el bug primero); no refactorizar por estética; baseline y medición después en optimizaciones.
3. No cambiar en silencio nombres de tópicos, servicios, acciones, frames, unidades, QoS ni parámetros públicos.
4. `/joint_states` distingue medido vs comandado; publicar ceros sin feedback es un defecto.
5. Rutas por `get_package_share_directory` o parámetros, no `/home/...` (fuera de navegación).
6. Código externo solo con licencia compatible y atribución; "sin licencia" = estudiar, no copiar.
7. **Etiquetas de evidencia** (no inventar resultados): `STATIC_PASS`, `UNIT_PASS`, `SIM_PASS`, `NUC_VERIFIED`, `JETSON_VERIFIED`, `ETHERNET_VERIFIED`, `HARDWARE_VERIFIED` (solo ensayo físico autorizado con log), `COMPETITION_READY` (ensayos repetidos en escenario equivalente), `BLOCKED_HARDWARE`, `BLOCKED_APPROVAL`. En documentos: `VERIFICADO <fecha>`, `CODIGO`, `HISTORICO <fecha>`, `FABRICANTE`, `PENDIENTE`.
8. Separar siempre lo confirmado de lo histórico y de lo no verificado.

## 9. Política de documentación (obligatoria)

**No crear nuevos archivos Markdown por tarea, problema o avance.** Por defecto:

| Evento | Dónde |
|---|---|
| avance integrado, prueba, bloqueo, decisión | `docs/collab/STATUS.md` |
| tarea nueva o cerrada, propuesta de navegación | `docs/ROADMAP.md` |
| cambio de arquitectura, hardware, interfaz, calibración, referencia técnica | `docs/ROBOT.md` |
| regla general | `AGENTS.md` |
| instalación o uso | `README.md` |
| detalle de una herramienta o paquete | su propio `README.md` |

- Un documento nuevo solo con razón técnica clara (manual de operación o seguridad que no cabe en los maestros), justificando en el commit por qué no entra en uno existente.
- Prohibido crear `HANDOFF`, `TASKS`, `NOTES`, `MEMORY`, `AUDIT`, `PLAN` o variantes versionadas (`_v2`, `_FINAL`…). Nada de carpetas `archive/`: la historia vive en Git (citar el commit).
- `STATUS.md` se mantiene corto: se reemplaza lo viejo, no se acumula. Lo histórico se referencia por SHA.
- Al cambiar código que afecta lo documentado, actualizar el documento en el **mismo commit**.

## 10. Cierre de cada tarea

Informar al usuario y registrar en `STATUS.md`: rama y SHA reales, archivos tocados, pruebas con su
resultado real, qué **no** se probó, cambios en hosts/servicios, restricciones respetadas (navegación
intacta, sin actuadores), rollback, riesgos pendientes y la próxima acción. Si no hubo validación física,
decirlo explícitamente.
