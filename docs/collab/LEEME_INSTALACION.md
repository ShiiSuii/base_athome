# INICIO — Paquete de instrucciones Base@Home para Claude + Antigravity en Orca

**Última actualización:** 29/10/2026. Estos archivos son instrucciones para que los agentes auditen e implementen; **NO** certifican que se haya modificado el robot, actualizado Ubuntu/ROS ni desplegado software.

## 1. Dónde copiar cada archivo

Verificar con `git rev-parse --show-toplevel` desde `/home/athome/base_athome_ws`. Las rutas son relativas a la RAÍZ GIT, que normalmente será el workspace indicado por Claude; no asumir la ruta si el comando indica otra.

```text
<RAIZ_GIT>/
├── AGENTS.md
├── CLAUDE.md
├── docs/
│   ├── AUDITORIA_ARQUITECTURA.md
│   ├── ROADMAP_ROBOCUP.md
│   ├── URDF_EDITOR_SPEC.md
│   ├── MIGRACION_JETSON.md
│   ├── RED_ETHERNET.md
│   ├── REFERENCIAS_TECNICAS.md
│   └── collab/
│       ├── ORQUESTACION_ORCA.md
│       ├── TASKS.md
│       ├── HANDOFF.md
│       └── PROMPTS_AGENTES.md
└── src/ (existente)
```

`LEEME_INSTALACION.md` es solo una ayuda para copiar, opcional en `docs/collab/`.

## 2. Qué comunicar en la primera sesión

- **NUC y Jetson Orin Nano están ENCENDIDAS**, físicamente unidas por cable Ethernet directo. Las direcciones del README (NUC `10.42.0.1`, Jetson `10.42.0.12`) son pistas a VERIFICAR.
- **Los demás subsistemas están DESENERGIZADOS**: no intentar arrancar el Piper, actuadores, elevador, cabezal ni hardware con auto-enable.
- **La navegación funciona y NO debe modificarse.** Solo estudiar sus interfaces y proteger su comportamiento.
- Se permite acceder a la Jetson por **SSH**, inventariar y mantener software, preparar backups, actualizar paquetes compatibles y avanzar hacia **JetPack 7.2.1/Ubuntu 24.04/ROS 2 Jazzy**. Para cualquier flash, borrado, particionado o escritura UEFI/QSPI: backup verificado, disco de destino identificado y confirmación humana explícita antes del paso irreversible.
- Solo ROS 2. No instalar ni ejecutar ROS 1 ni usar `ros1_bridge`.
- No usar cuentas Google (Gmail/Drive/Calendar/Cloud), aunque Antigravity esté conectado. No pagos, compras, suscripciones o recursos facturables.
- Claude es coordinador/integrador; Antigravity implementa tareas separadas y revisa. Orca debe registrar Run/Task/worker real. Usar worktrees independientes y revisar cada diff.

## 3. Inicio recomendado para el usuario

1. Copiar los dos MD a la raíz de Git y los demás según el árbol.
2. Revisar en Orca **Settings → Agents → Agent Permissions**. Deshabilitar permisos peligrosos automáticos antes de dar acceso SSH/hardware; consultar la guía de la versión instalada.
3. Abrir Claude Code en ese repo; pedirle que lea todo `AGENTS.md` y `CLAUDE.md`, que siga el **PROMPT A** de `docs/collab/PROMPTS_AGENTES.md` y abra un Run de coordinación real si el sistema lo permite.
4. Primeras tareas: `HOME-00` (reconocer repo), `HOME-01E` (Ethernet NUC↔Jetson), `HOME-10` (inventario y respaldos Jetson); Antigravity `HOME-04` (editor URDF offline). Sin iniciar actuadores.
5. Claude muestra estado real, diffs, tests y gates pendientes; no declarar upgrades hechos hasta comprobarlos ni hacer push/merge sin tu aprobación.

## 4. Enlaces de referencia

- Repo: https://github.com/ShiiSuii/base_athome
- Orca / agents: https://www.onorca.dev/docs/agents/supported
- Orca / orchestration: https://www.onorca.dev/docs/cli/orchestration
- JetPack 7.2.1: https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/quick_start.html
- ROS 2 Jazzy ARM64: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- RoboCup@Home rulebook release 2026.2: https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2
