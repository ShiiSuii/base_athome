# Protocolo de colaboración Orca — Claude Code + Google Antigravity

**Proyecto:** Base@Home / UruBots · RoboCup@Home OPL · `https://github.com/ShiiSuii/base_athome`  
**Ruta indicada del workspace NUC:** `/home/athome/base_athome_ws` (validar raíz de Git con `git rev-parse --show-toplevel`).  
**Estado operativo:** NUC Y JETSON encendidas; enlace directo por cable Ethernet. Solo sistemas periféricos/actuadores desenergizados. Se permite SSH, diagnóstico y mantenimiento de software en Jetson; no control de hardware físico.  
**Autoridad:** instrucciones del usuario y `AGENTS.md` > este documento > documentación técnica externa.

## 1. Contrato de los dos agentes

| Rol | Claude Code | Google Antigravity |
|---|---|---|
| Coordinación | Director técnico y coordinador **si Orca permite orquestación verificable** | Worker supervisado o desarrollador en worktree propio |
| Ámbito principal | Auditoría de arquitectura/ROS 2, drivers/interfaces solo lectura, revisión de código, contratos, dependencias, pruebas y documentación | Editor paramétrico Xacro/URDF localhost, UI, componentes testables de percepción/HRI y tests offline |
| Revisión cruzada | Revisa PR/diff de Antigravity, riesgos de ROS/safety | Revisa claridad de API, frontend/UX, tests y cambios no peligrosos de Claude |
| Invariantes | Navegación protegida, ROS 2, NUC+Jetson activos por Ethernet, actuadores apagados, no Google, no pagos | Las mismas restricciones |
| Entrega | Informe técnico con evidencias + backlog + revisión | Cambio acotado con tests y documentación |

Roles son recomendaciones, no privilegios de acceso. **Ningún agente tiene autoridad para aprobarse sus propios cambios de riesgo, gastos o hardware.** El usuario decide.

## 2. Principio de infraestructura

- Orca usa worktrees Git separados. Un worktree aísla **archivos versionados**, no los procesos, puertos, devices, servicios, usuario Linux, ROS_DOMAIN_ID ni credenciales del host.
- Un `AGENTS.md` del worktree de Claude NO modifica automáticamente el del worktree de Antigravity hasta que ambos incluyan los commits correspondientes. Al iniciar una rama desde `origin/main` verificar que incluye esta documentación. No presuponer sincronización.
- Los archivos `docs/collab/TASKS.md` y `HANDOFF.md` son **instantáneas versionadas**; los cambios allí no se reflejan inmediatamente en worktrees hermanos. Preferir Tasks/Run de Orca como autoridad de estado activo. Integrar cambios al snapshot en una rama de coordinación.
- No usar `.env` compartido ni compartir paths que contengan tokens, sesiones o llaveros entre worktrees. Evitar `orca.yaml` que comparta carpetas de build ROS/`install` con procesos activos; cada worktree se compila separadamente o utiliza salida temporal no operativa.
- Si se usa la NUC a través de SSH, el mismo host puede ejecutar comandos de dos agentes: los comandos que alteran estado externo requieren **reserva de recurso** y aprobación, aunque cada agente tenga un worktree distinto.

## 3. Verificación inicial: NO cambiar nada todavía

Desde el worktree exacto que se abrió en Orca:

```bash
pwd
git rev-parse --show-toplevel
git remote -v
git status --short --branch
git log -1 --format='%h %cs %s'
git worktree list
command -v orca || true
# Solo si orca CLI está disponible:
orca status --json
orca worktree current --json
```

Verificar si está activo el módulo Experimental → Orchestration. Leer https://www.onorca.dev/docs/cli/orchestration y, **antes de mutar estado de Orca**, cargar la guía de la versión instalada:

```bash
orca skills get orchestration --full
```

Si Orca no existe en ese host o no tiene orquestación: usar el procedimiento manual de la sección 9. No ejecutar instaladores, ni iniciar sesión en cuentas, ni habilitar integraciones de forma autónoma.

## 4. Ajustes de seguridad recomendados al humano

Según documentación de Orca, algunos agentes pueden iniciarse con banderas que omiten controles de permisos, por ejemplo Claude `--dangerously-skip-permissions` (véase https://www.onorca.dev/docs/model/agents-sessions). **No operar ROS ni hardware con esas banderas.** Pedir al usuario que revise `Settings → Agents` y restablezca un modo interactivo/restrictivo según soporte de cada agente. No intentar cambiar ajustes del usuario unilateralmente.

Adicionalmente:
- Deshabilitar conectores Google (Gmail/Drive/Calendar/Cloud) y perfiles con acceso a cuentas, salvo la autenticación estrictamente necesaria para usar Antigravity como IDE. **Nunca usarlos**.
- No otorgar credenciales de pago ni tokens de nube. No crear servicios ni proyectos cloud.
- Limitar permisos SSH a lectura de workspace cuando sea posible, y no iniciar agente con privilegios root.
- Permitir SSH Jetson y administración de software reversible con verificación de impacto; **reservar control humano para el flasheo destructivo/particionado/UEFI-QSPI, alteraciones de red que corten SSH, CAN, motores, elevador, cámaras activas, Git push/merge y cualquier modificación de navegación.**
- Para tareas con navegador, usar solamente páginas públicas de documentación técnica, sin perfiles personales.

## 5. Modelo de orquestación (solo si realmente está disponible)

El mecanismo recomendado por Orca es **Run → Task → supervised worker → mensajes/heartbeat → `worker_done` → revisión/gate**. No invocar comandos legados `orca orchestration run` ni `run-stop` (retirados). El agente debe revisar `orca skills get orchestration --full` porque opciones y sintaxis cambian entre versiones.

### Ejemplo orientativo (NO ejecutar hasta verificar y autorizar)

```bash
# Estado primero
orca status --json
orca skills get orchestration --full

# Crear un Run de desarrollo exclusivamente OFFLINE
orca orchestration run-create \
  --objective "Base@Home: auditoria segura, URDF editor offline, ROS2 only, no nav changes" --json

# Crear tarea pequeña, sin hablar con dispositivos
orca orchestration task-create \
  --task-title "URDF offline: schema + preview" \
  --spec "Leer AGENTS.md, revisar Xacro, implementar preview localhost en worktree propio. Sin cambios Nav2, sin dispositivos, sin cuenta Google, sin gastos. Documentar pruebas y diff." --json

# Usar el ID real del resultado anterior. Esto LANZA un agente: pedir autorización humana
orca orchestration worker-start \
  --task '<TASK_ID_REAL>' --worktree new-child \
  --name urdf-preview --agent antigravity --setup run --json

# Revisar resultados y preguntas. Evitar bucles infinitos de polling.
orca orchestration check --wait --types worker_done,escalation,question --timeout-ms 60000 --json
```

La secuencia no es una autorización para ejecutar: es documentación de una funcionalidad optativa. **Nunca inventar TASK_ID, DISPATCH_ID ni mensajes recibidos**. El worker reporta sus IDs REALES y el coordinador comprueba el diff de esa rama. No realizar dispatch a un agente mientras el usuario no haya revisado permisos, objetivos y exclusiones.

### Comunicación formal

- `question`: duda técnica que exige respuesta del coordinador.
- `escalation`: situación de seguridad, intervención humana, conflicto de archivos o necesidad de probar hardware.
- `heartbeat`: avance real con datos, no mensajes vacíos.
- `worker_done`: exactamente un reporte final por dispatch con `outcome`, `taskId`, `dispatchId` y archivos modificados; luego la revisión del coordinador determina si se acepta.
- Aceptar/ack del mensaje no significa que el código esté aprobado.
- Mantener un registro legible para el usuario en `HANDOFF.md` **tras integrar el trabajo del agente**, nunca editando el mismo archivo simultáneamente.

## 6. Plantilla de asignación obligatoria

```text
TASK-ID: HOME-XX
Agente: claude | antigravity
Objetivo: [resultado observable, pequeño y comprobable]
Worktree / branch: [nombre real, jamás inventado]
Permitido: [rutas concretas del repo + operaciones offline]
Prohibido: [nav, dispositivos, ROS1, Google, pagos, credenciales, main]
Entradas: [rutas + commit SHA + documentación]
API/contrato de salida: [archivos, funciones, eventos, estructura JSON, tests]
Dependencias: [bloqueadas o ya disponibles]
Validación: [comando offline; expected output]
Definición de terminado: [criterios observables]
Rollback: [cómo revertir sin afectar otros cambios]
Handoff: [resumen, diff, comandos ejecutados, resultados reales, limitaciones]
```

## 7. Flujo diario seguro

**Fase PLAN** — Claude identifica rama/worktree, inspecciona estado, revisa conflictos, prioriza una tarea del tablero. Reparte con exclusividad de archivos, recursos y contratos. La tarea `HOME-00` (documentación/revisión de contexto) precede a cambios funcionales.

**Fase IMPLEMENT** — cada agente trabaja en su rama. Prohibido editar áreas ajenas, copiar archivos privados, modificar navegación o iniciar roslaunch/bringup de hardware. Agregar tests desde el comienzo. Compilar solo en ambiente aislado y con recursos limitados.

**Fase REVIEW** — validar diff de archivos y status, `git diff --check`, pruebas unitarias y tests Xacro; anotaciones de defectos concretos. No afirmar éxito de hardware. No absorber automáticamente PR grandes; solicitar división.

**Fase APPROVAL** — pedir al humano aprobación específica para `git push`, PR remoto, merge, despliegue operativo, flasheo/particionado/UEFI o instalaciones capaces de romper el canal SSH (las instalaciones reversibles de Jetson ya están autorizadas). Una aprobación de PR no autoriza mover brazo ni reiniciar el robot.

**Fase HANDOFF** — registrar cambios, evidencia, bloqueos, próximos pasos, riesgos y enlaces al commit. Sin autorización de publicar, dejar los commits locales listos.

## 8. Política de Git y propiedad de archivos

- `main` protegida: nunca commits directos ni force-push. Tampoco `git reset --hard` sobre trabajos ajenos, limpieza agresiva de branches ni rebase que afecte ramas compartidas.
- Convención de ramas: `claude/home-01-hardware-audit`, `antigravity/home-04-urdf-ui`, `claude/home-06-hri-tests` (ejemplos; Orca puede asignar otros nombres).
- Un branch/worktree por tarea. Crear tareas que afecten distintos paquetes, no dos tareas sobre el mismo launch a la vez.
- Si Claude necesita cambiar contratos usados por Antigravity, crear primero un documento de interfaz compartida y revisar su compatibilidad; tras integrar, Antigravity hace rebase/cherry-pick de forma consciente en su rama, con estado limpio.
- Los cambios comunes a `AGENTS.md`, `CLAUDE.md`, `ROADMAP_ROBOCUP.md`, `TASKS.md`, `HANDOFF.md`, `README.md` se hacen por un agente **integrador**, después de recibir handoffs separados, para reducir conflictos.
- Antes de un commit: comprobar `.gitignore`, secretos, datos biométricos/logs/imágenes privadas, modelos con licencia y archivos grandes. No subir bags, videos de personas, claves, tokens ni datos del entorno.
- Mensaje de commit: `docs(home-01): map nuc and jetson ownership`, `feat(home-04): preview xacro parameters offline`, `test(home-06): add receptionist mocks`.

## 9. Fallback si no hay orquestación automática

1. Claude escribe en su rama un plan de reparto y solicita al humano que inicie sesión independiente de Antigravity en otro worktree.
2. El usuario entrega a Antigravity el prompt B de `PROMPTS_AGENTES.md` más el Task Contract de su tarea.
3. Antigravity ejecuta su tarea, produce diff, pruebas y nota de entrega **en su rama**, sin editar `TASKS.md` compartido simultáneamente.
4. Claude revisa el diff y comenta qué acepta/qué requiere corrección; se integran los commits a una rama técnica aprobada, sin tocar main.
5. Solo el integrador actualiza `TASKS.md` y `HANDOFF.md` después de conciliar los resultados.

No confundir la coexistencia de sesiones en Orca con una conversación real entre agentes.

## 10. Matriz de conflictos y bloqueos

| Evento | Acción obligatoria |
|---|---|
| Antigravity quiere editar `src/base_athome_navigation` | Bloquear y pedir alternativa sobre wrapper externo |
| Se encuentra una necesidad de Nav2/footprint | Documentar dependencia; no tocar hasta nuevo mandato humano explícito |
| Jetson, Piper o cámaras fuera de línea | Marcar `HARDWARE_BLOCKED`, desarrollar mock/offline |
| Otro worktree editó mismo archivo | Detener, comunicar al coordinador, elegir propietario único |
| Un test puede publicar en `ROS_DOMAIN_ID=10` | Usar dominio aislado y mock; no ejecutar contra red de robot |
| El CLI solicita login Google | Detener, no inspeccionar cuentas; humano configura autenticación si desea |
| Un link externo exige pago o token | Detener, elegir alternativa pública/gratuita |
| Un tutorial contiene comandos ROS 1 | Leer solo como antecedentes; no instalar, no ejecutar ni adaptar sin convertir a ROS 2 |
| `git` no está limpio | No integrar; averiguar propietario de cambios y preservar archivos |
| Se solicita `sudo` o dependencia no presente | Jetson: software reversible autorizado con revisión de impacto y rollback; NUC, firmware, red crítica o acciones destructivas: gate humano específico |
| Un agente dice que una prueba física pasó sin logs | Rechazar afirmación; solicitar evidencia de ensayo real autorizado |

## 11. Definición de listo por tarea

`IMPLEMENTED` ≠ `OFFLINE_TESTED` ≠ `REVIEWED` ≠ `HARDWARE_VERIFIED` ≠ `DEPLOYED`. Cada etiqueta requiere evidencia distinta. Trabajo offline sin hardware puede estar terminado para desarrollo pero permanecer `HARDWARE_BLOCKED` para validación física.

### Referencias oficiales Orca

- https://www.onorca.dev/docs/model/worktrees
- https://www.onorca.dev/docs/model/agents-sessions
- https://www.onorca.dev/docs/agents/supported
- https://www.onorca.dev/docs/cli/orchestration
- https://www.onorca.dev/docs/cli/skills
- https://www.onorca.dev/docs/cli/overview
- https://www.onorca.dev/docs/agents/claude-code
