# Permisos y bloqueos de los agentes (Claude Code y Antigravity), 30-10-2026

Pedido del usuario: "detectá qué cosas te están trancando de las instrucciones y dame un archivo
con todo lo que necesitás (permisos, etc.) para trabajar mejor".

Este archivo es para que el humano decida y actualice `AGENTS.md`, `CLAUDE.md` y `HANDOFF.md`.
**No se cambió ningún permiso ni configuración de seguridad.** Todo lo de abajo son propuestas.

---

## 1. Bloqueos reales de estas sesiones (con evidencia)

| # | Bloqueo | Qué pasó | Qué lo destraba |
|---|---|---|---|
| B1 | **Servicios systemd de usuario y linger** | El clasificador de permisos rechazó instalar `scripts/systemd/realsense-nuc.service` (`systemctl --user enable` + `loginctl enable-linger`). El archivo existe, pero no está instalado | Instalarlo ustedes (ver la sección 4) o autorizarlo por escrito ("instalá el servicio realsense-nuc") |
| B2 | **Matar procesos del usuario** | Se rechazó `kill -9` de un `joint_state_publisher` viejo de `view_robot`, que pisaba el brazo con ceros en RViz | Cerrar esas ventanas a mano, o autorizar "podés cerrar procesos ROS viejos míos" |
| B3 | **Las instrucciones dicen "actuadores apagados / no publicar comandos ROS"**, pero el usuario autorizó usar el brazo y el pan-tilt estando al lado ("no tengas miedo de utilizarlo yo estoy al lado") | Cada sesión nueva arranca leyendo la prohibición y pierde tiempo dudando o pidiendo confirmación | Definir **modos explícitos** en `AGENTS.md` (sección 2) |
| B4 | **"Navegación solo lectura"** frente al editor de mapas | El editor (HOME-22) escribe `semantic_map.yaml` y `waypoints.yaml` en `base_athome_navigation/config`. Por la regla literal, eso es "tocar navegación" | Aclarar que la **capa semántica** (zonas, waypoints, objetos) sí se puede editar. Nav2, AMCL, costmaps, mapas `.pgm`, controladores y launches siguen protegidos |
| B5 | **Jetson documentada con Humble / 22.04** | Ya está en JetPack 7.2.1, Ubuntu 24.04 y Jazzy (flasheo verificado). `AGENTS.md` (línea 31) y partes de `CLAUDE.md` siguen diciendo Humble | Actualizar el texto (sección 2) |
| B6 | **Workers de Antigravity en Orca** | Mueren cuando se reinicia la sesión, son lentos, piden login y tienen cuota. Los mensajes de orquestación (*heartbeats*) interrumpen el trabajo. El auto-aprobador quedó en `scripts/orca/agy_autoapprove.py` porque el *scratchpad* se borró | Abrir Antigravity antes de pedir delegación. Aceptar que, si el worker no responde, Claude implemente directamente. Desactivar `--dangerously-skip-permissions` en Orca → Settings → Agents |
| B7 | **Límite de contexto** | Las sesiones largas se compactan y se pierden detalles (rutas, valores probados) | Mantener `HANDOFF.md` al día al cerrar cada bloque (ya se hace) y tareas cortas por sesión |
| B8 | **Ultralytics auto-instala paquetes** | Una exportación instaló numpy 2 en `~/ros2_vision_venv` y rompió `cv2` | Mantener `YOLO_OFFLINE=1` y no exportar modelos con ese venv (queda en la memoria del agente) |
| B9 | **Tareas físicas** | Medir la columna del torso (colisión en MoveIt), definir el waypoint `home`, ubicar objetos en `editor_mapa`, energizar subsistemas | Solo el humano (sección 4) |

## 2. Cambios propuestos a `AGENTS.md` / `CLAUDE.md`

1. **Agregar una tabla de modos y declarar cuál rige al comenzar cada sesión:**

   | Modo | Hardware | Agentes pueden |
   |---|---|---|
   | `CASA` | Solo NUC + Jetson | Simulación (`ROS_DOMAIN_ID=42`, `ATHOME_SIM=1`), SSH y software de la Jetson, tests, docs. **Nada de comandos a actuadores** |
   | `LAB_SUPERVISADO` | Actuadores energizados **con un humano al lado** | Lo anterior + mover el brazo, el pan-tilt y la cámara a velocidad ≤ 30 %, probar apps reales. **No mover la base ni el elevador** sin un pedido explícito |
   | `COMPETENCIA` | Todo | Solo lo que el humano lance |

   Hoy (30-10) rige **`CASA`**.

2. Reemplazar "Jetson documentada con Ubuntu 22.04 + ROS 2 Humble" por "**Jetson: JetPack 7.2.1
   (L4T R39.2.1), Ubuntu 24.04, ROS 2 Jazzy**. Servicio `jetson-head`".
3. **Navegación:** "La capa semántica (`semantic_map.yaml`: zonas, waypoints, objetos) se edita con
   `editor_mapa` y está permitida. Nav2, AMCL, costmaps, mapas, odometría, controladores y launches
   de navegación siguen siendo solo lectura".
4. **Simulación:** "Toda prueba sin robot usa `base_athome_sim` en `ROS_DOMAIN_ID=42` (alias
   `sim_env`). Está prohibido publicar en el dominio 10 desde una terminal de simulación". Ver
   `docs/SIMULACION.md`.
5. **Antigravity:** "Si el worker no confirma recepción en 10 min, el coordinador puede implementar
   la tarea él mismo y lo deja registrado". Esto evita esperas indefinidas.
6. **Receptionist:** agregar que en el rulebook 2026.2 se revisa contra el *HRI Challenge*, y que la
   entrada táctil se usa como respaldo (`docs/HRI_MICROFONO_REVISION.md`).

## 3. Permisos de Claude Code que acelerarían el trabajo (propuesta, no aplicada)

`~/.claude/settings.json` hoy está vacío (`{}`), así que cada comando nuevo puede pedir
confirmación o ser evaluado por el clasificador. Propuesta para
`.claude/settings.json` del proyecto. Revísenla antes de copiarla:

```json
{
  "permissions": {
    "allow": [
      "Bash(colcon build:*)", "Bash(colcon test:*)", "Bash(python3 -m pytest:*)",
      "Bash(ros2 topic list:*)", "Bash(ros2 topic echo:*)", "Bash(ros2 topic hz:*)",
      "Bash(ros2 node list:*)", "Bash(ros2 launch base_athome_sim:*)",
      "Bash(ssh ubuntu@10.42.0.12:*)", "Bash(scp:*)",
      "Bash(systemctl --user status:*)", "Bash(journalctl:*)",
      "Bash(git status:*)", "Bash(git diff:*)", "Bash(git log:*)", "Bash(git add:*)", "Bash(git commit:*)"
    ],
    "deny": [
      "Bash(git push:*)", "Bash(git push --force:*)", "Bash(git checkout main:*)",
      "Bash(roscore:*)", "Bash(rosrun:*)", "Bash(catkin_make:*)",
      "Bash(sudo dd:*)", "Bash(sudo mkfs:*)"
    ]
  }
}
```

**Reglas de esta propuesta:**
- `git push` sigue **denegado**: la autorización de push es siempre humana.
- Para actuadores no se propone ningún permiso automático. En modo `LAB_SUPERVISADO` se autoriza
  en el chat, por sesión.

## 4. Acciones del humano (checklist)

- [ ] **Revocar el token de GitHub** que está dentro de la URL del remoto `origin` en la NUC
  (`ghp_…`). Después usar `gh auth login` o un *credential helper*. Ningún agente copió ni publicó
  el token.
- [ ] **Borrar el sudo temporal** con `sudo rm /etc/sudoers.d/99-jetson-flash-temp`. Era para el
  flasheo, que ya terminó. Si quieren que el agente instale paquetes, mejor una regla acotada, por
  ejemplo solo `apt-get install` y `systemctl restart jetson-head`.
- [ ] En la Jetson, decidir si se mantiene `/etc/sudoers.d/90-ubuntu-nopasswd` (NOPASSWD total) o
  se acota.
- [ ] Instalar el servicio de la RealSense, si lo quieren (B1):
  `mkdir -p ~/.config/systemd/user && cp ~/base_athome_ws/scripts/systemd/realsense-nuc.service ~/.config/systemd/user/ && systemctl --user daemon-reload && systemctl --user enable --now realsense-nuc && sudo loginctl enable-linger athome`
- [ ] Medir la columna del torso (alto, ancho y profundidad, y su posición respecto de
  `base_link`) para corregir la colisión en MoveIt.
- [ ] En `editor_mapa`: crear el waypoint `home` y ubicar los objetos numerados (por ejemplo, la
  Pringles = 1).
- [ ] Orca → Settings → Agents: desactivar los permisos irrestrictos.
- [ ] Copiar a `AGENTS.md` / `CLAUDE.md` lo que aprueben de la sección 2.
