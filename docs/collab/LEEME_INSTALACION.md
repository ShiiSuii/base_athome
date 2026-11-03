# LEEME — Paquete integral Base@Home / Orca + Claude + Antigravity v3

**Fecha:** 03/11/2026. **Propósito:** reemplazar los documentos generales que preparó ChatGPT anteriormente con una versión completa, manteniendo los progresos reales comunicados por Claude. **Esto NO modifica el repositorio remoto ni los equipos físicos.**

## 1. Instrucción de instalación

Este ZIP está estructurado **desde la raíz Git real** de `/home/athome/base_athome_ws`, que debe confirmarse con `git rev-parse --show-toplevel` antes de copiarlo. El ZIP contiene `AGENTS.md`, `CLAUDE.md`, `.claude/settings.json` y `docs/...`.

**No extraer encima de una sesión Orca activa sin revisar primero.** Claude/Antigravity pueden haber agregado nuevas líneas a `HANDOFF.md`, `TASKS.md`, configs o más documentación desde el 30/10. Deben preservarse todas las novedades.

### Instalación recomendada en NUC (desde terminal propia o con Claude en revisión)

```bash
cd /home/athome/base_athome_ws
git rev-parse --show-toplevel
git status --short --branch
# Guardar copia local antes de actualizar documentación. No usar comandos que sobrescriban archivos no rastreados.
mkdir -p ~/baseathome_docs_backup_20261103
cp -a AGENTS.md CLAUDE.md docs ~/baseathome_docs_backup_20261103/ 2>/dev/null || true
# Extraer ZIP aparte (ajustar ruta donde se descargó):
mkdir -p /tmp/baseathome-v3-import
unzip -q /ruta/al/BaseAtHome_Orca_Claude_Antigravity_COMPLETO_v3.zip -d /tmp/baseathome-v3-import
# Examinar diferencias ANTES de copiar:
diff -u AGENTS.md /tmp/baseathome-v3-import/AGENTS.md || true
# Integrar archivos revisados de manera selectiva, sin sobrescribir handoff/task viva.
```

**Si el workspace no coincide con la raíz Git**, colocar los archivos donde esté el `AGENTS.md` raíz y `CLAUDE.md` correspondiente. No reemplazar `.git` ni `.venv` ni `build/`, `install/`, `log/`.

### Casos especiales

- `docs/collab/HANDOFF.md` del ZIP incluye la **última versión que el usuario compartió** con registros del 29–30/10/2026. Si el archivo de la NUC tiene registros posteriores, hacer merge, **no reemplazarlo**.
- `docs/collab/TASKS.md` conserva el backlog suministrado por el usuario y añade HOME-30 a HOME-34. Integrar con estados reales de Orca y Git. No reabrir HOME-11 ya completado por haber sido P0 en archivos antiguos.
- `docs/RUNBOOK_MIGRACION_JETSON.md` y `docs/MIGRACION_JETSON.md` quedan como historial, no como orden de reinstalar Jetson.
- `.claude/settings.json` es una configuración conservadora de allow/ask/deny para revisión humana. **No sustituye** Settings → Agents → Agent Permissions de Orca. Las reglas de permisos podrían variar según versión CLI; revisar `/permissions` en Claude antes de confiar en ellas.
- No reemplazar scripts o código ROS con este ZIP: esta entrega contiene solamente reglas y documentos.

## 2. Qué cambió de manera esencial

1. Claude y Antigravity pueden investigar libremente documentación pública, código ROS2, proyectos robóticos y **bases hoverboard** y comparar tecnologías.
2. Navegación puede auditarse **sin límite de profundidad** (firmware documentado, motor, UART, odometría, TF, Nav2, métricas pasivas), pero **no se edita ni se prueba con movimiento**. Propuestas en `docs/collab/PROPUESTAS_NAVEGACION.md`.
3. Los modos CASA/LAB_SUPERVISADO/COMPETENCIA y permisos de operaciones remotas se mantienen; una autorización antigua no se traslada a una sesión nueva.
4. Jetson figura reportada migrada a Ubuntu 24.04 / ROS 2 Jazzy; se debe verificar, **no volver a flashear**.
5. Las tareas RoboCup, simulación, HRI, percepción, Piper, URDF localhost, documentación y mantenimiento siguen como objetivos activos.
6. La orquestación Orca se basa en Run/Task/Dispatch real, worktrees separados, revisión mutua y handoff versionado; sin crear mensajes ficticios.
7. Prohibiciones vigentes: ROS1, cuentas de Google/privadas, compras, pagos o facturación, gestión de secretos, movimientos no autorizados, navegación productiva y publicación/merge de `main` sin confirmación.

## 3. Archivos incluidos

| Archivo | Uso principal |
|---|---|
| `AGENTS.md` | Política compartida, autonomía, límites y conocimiento robótico |
| `CLAUDE.md` | Onboarding del director técnico Claude en Orca |
| `.claude/settings.json` | Permisos efectivos propuestos a inspeccionar |
| `docs/BASE_HOVERBOARD_REFERENCIAS.md` | Investigación verificable ROS2 de bases hoverboard |
| `docs/AUDITORIA_NAVEGACION.md` | Auditoría profunda pasiva del sistema funcional |
| `docs/collab/PROPUESTAS_NAVEGACION.md` | Propuestas con evidencia, costo y rollback |
| `docs/HARDWARE_ELECTRICA.md` | Inventario plataforma/distribución de potencia no verificada |
| `docs/ROADMAP_ROBOCUP.md` | Roadmap y prioridades (histórico + nota vigente) |
| `docs/HRI_COMPETENCIA.md` | Habilidades y tests para RoboCup 2026.2 |
| `docs/AUDITORIA_ARQUITECTURA.md` | Arquitectura y lista de auditorías |
| `docs/URDF_EDITOR_SPEC.md` | Editor localhost y validación Xacro/3D |
| `docs/RED_ETHERNET.md` | NUC↔Jetson, CycloneDDS, pruebas de red |
| `docs/MIGRACION_JETSON.md` | Referencia histórica de migración ya reportada |
| `docs/ESTADO_A_VERIFICAR.md` | Evidencia/estado conocido vs desconocido |
| `docs/MODOS_OPERACION.md` | Condiciones de CASA y ensayos supervisados |
| `docs/REFERENCIAS_TECNICAS.md` | Fuentes ROS2, fabricantes, equipos, repositorios |
| `docs/RUNBOOK_CABLE_ETHERNET.md` | Runbook histórico de recuperación del enlace |
| `docs/RUNBOOK_MIGRACION_JETSON.md` | Runbook histórico de flasheo, NO reejecutar |
| `docs/collab/ORQUESTACION_ORCA.md` | Contratos de comunicación Claude+Antigravity |
| `docs/collab/ESTRATEGIA_EJECUCION.md` | Ciclo calidad/automatización/herramientas |
| `docs/collab/WORKTREE_GIT.md` | Branches, recursos compartidos, PRs seguros |
| `docs/collab/PROTOCOLO_REVISION.md` | Seguridad, tests, revisión técnica |
| `docs/collab/PERMISOS_Y_BLOQUEOS.md` | Restricciones detalladas/qué desbloquear |
| `docs/collab/PLAN_ACCION_PERMISOS.md` | Acciones para configurar Orca y servidores |
| `docs/collab/PROMPTS_AGENTES.md` | Megaprompts de delegación, HRI, hoverboard, navegación |
| `docs/collab/TASKS.md` | Backlog/historial conservado + HOME-30..34 |
| `docs/collab/HANDOFF.md` | Historial conservado de trabajos/mediciones |
| `docs/collab/DECISIONES.md` | Registro de decisiones del usuario |

## 4. Comandos recomendados para probar coherencia (no robot)

```bash
python3 -m json.tool .claude/settings.json >/dev/null
rg -n 'BASE_HOVERBOARD_REFERENCIAS|AUDITORIA_NAVEGACION|PROPUESTAS_NAVEGACION' AGENTS.md CLAUDE.md docs/collab/PROMPTS_AGENTES.md
rg -n 'ROS 1|Google|pagos|Nav2|Humble|Jazzy' AGENTS.md CLAUDE.md docs/MIGRACION_JETSON.md
# No ejecutar comandos de arranque real desde scripts de documentos sin revisar su estado histórico.
```

## 5. Primer mensaje recomendado a Claude Code

> Analizá AGENTS.md y CLAUDE.md v3, y conservá todo el trabajo existente. Estamos en Orca con Antigravity; el robot ya tiene navegación que funciona. Autorizo una investigación profunda y libre de bases móviles hoverboard y de nuestra propia navegación, pero **sin cambiarla**: quiero propuestas de mejora documentadas. Jetson fue migrada a Jazzy según el último handoff, verificá antes de planificar. Reconciliá TASKS/HANDOFF con el estado real de Git/Orca y distribuí tareas entre vos y Antigravity. Trabajá sin trabarte por permisos menores en áreas de código no protegidas, con tests, docs y PRs revisables; respetá el modo operativo y los bloqueos de seguridad, privacidad, pagos y ROS1. Mostrame las primeras conclusiones con evidencias, no cambios hipotéticos.

## 6. Verificación de integridad

El ZIP y los archivos de este paquete son **documentación creada**, no resultados de pruebas ejecutadas en la NUC ni credenciales. El instalador debe comparar SHA y diff de cada archivo si hay contenido más reciente. El paquete se construyó con referencias web públicas consultadas el 03/11/2026; revisar commits/licencias antes de incorporar código externo.
