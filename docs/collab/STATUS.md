# STATUS.md — Estado vivo de Base@Home

Corto y actualizado: se **reemplaza** lo viejo, no se acumula (`AGENTS.md` §9). Lo histórico se
recupera por commit (§8). Editado por el integrador (Claude).

**Última actualización:** 2026-11-04 · Claude Code · modo `DESARROLLO` · fidelidad de la teleop.

## 1. Estado comprobado al pausar

| Ítem | Estado | Evidencia |
|---|---|---|
| NUC | Ubuntu 24.04, kernel 7.0.0-42 (reiniciada hoy), Jazzy, CycloneDDS, dominio 10 | `NUC_VERIFIED` |
| Jetson | L4T R39.2.1, Jazzy; `jetson-head` activo | `JETSON_VERIFIED` |
| Piper | energizado, **deshabilitado** tras reiniciarse la Jetson (colgando en reposo, feedback válido); controlador md5 `2e347c49` | `JETSON_VERIFIED` 2026-11-04 |
| Pan-tilt | **falla eléctrica abierta**: con torque el U2D2 se cae por USB en segundos; sin torque estable. Software corregido (vueltas multivuelta, `pwm_limit`, `position_p_gain`). Corre desde `jetson-head` (la Jetson se reinició; ya no hay proceso manual) | `JETSON_VERIFIED` |
| Teleop Meta Quest 2 | **nodo corriendo** en la Jetson (dominio 10, `real:=true`, 20/40°/s), sin mover nada hasta X. Quest desconectado al último chequeo (rearma `adb` solo). Fidelidad mejorada y validada en sim; seguimiento real pendiente | `SIM_PASS`; real parcial |
| Simulación | cerrada | — |
| Tests offline | Jetson scripts 32 + Quest 18, percepción 122, skills 33, bringup (elevador) 6, sim 7, pan-tilt 8, agentes 40 | `UNIT_PASS` 2026-11-03 |

## 2. Rama y Git

- Rama `claude/home-41-piper-mission` **subida a GitHub** el 2026-11-03 (82 commits; escaneo de secretos limpio; los rostros de `faces_database` quedan borrados del árbol). `main` = `1da2388` sin cambios: merge a `main` = decisión humana (PR pendiente). HOME-SEC: el token viejo sigue sin revocar.
- Cambios del usuario sin commitear (no tocar): `semantic_map.yaml`, `waypoints.yaml`, `arm_poses_and_limits.yaml`
  (incluye la `home` re-enseñada `[73.87, 169.72, -138.02, 26.69, -15.7, -117.17]`).
- Reparación tras el reinicio de la NUC: commit vacío `c98344d` rehecho como `f16f457`; respaldo `~/git_backup_20261103_165442`; `core.fsync=committed`.

## 3. Cómo retomar la teleoperación con el Quest (pendiente: prueba completa en el brazo real)

1. Quest por USB a la **Jetson**, depuración USB autorizada (`adb devices` → `device`).
2. **Cerrar todas las pestañas del navegador del Quest** (las viejas se restauran y mandan datos sin pose: fue la causa
   de "no se mueve"). Abrir una sola: `http://localhost:8765` (debe decir **v 7**).
3. Lanzar en la Jetson (robot real, dominio 10):
   ```bash
   source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=10 CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml
   cd ~ && python3 quest_teleop_node.py --ros-args -p mode:=cartesian -p real:=true -p max_joint_dps:=20.0 \
     -p max_wrist_dps:=40.0 -p max_lead_deg:=5.0
   ```
   Simulación: `ROS_DOMAIN_ID=42`, sin `real:=true`, con `base_athome_sim` en la NUC usando `~/cyclonedds.xml` (red).
4. Habilitar el brazo a mano (`/arm/set_enable`; el nodo nunca lo hace). En el visor: gatillo sobre la página → **GRIP apuntando hacia donde mira el robot** (calibra el frente) → **mantener X**: primero va a HOME (15°/s), después la pinza copia la mano;
   soltar X = frena en el lugar; Y = giro on/off; gatillo = pinza abierta ↔ 50 %.
5. Medir precisión real: `/quest_teleop/status` → `track_mm`, `track_deg` (objetivo vs FK de lo medido).

## 4. Procesos fuera de lo normal (limpiar al retomar)

| Dónde | Proceso | Qué hacer |
|---|---|---|
| Jetson | `quest_teleop_node.py` lanzado a mano (dominio 10) | `kill -TERM` al terminar (revierte el Quest) |
| Jetson | `adb` server + regla udev `/etc/udev/rules.d/51-meta-quest.rules` | dejar (necesarios para el Quest) |
| Jetson | `ros-jazzy-control-msgs` y `adb` instalados por apt | rollback: `~/deploy_backups/{control_msgs,adb}_20261103/installed.txt` |

## 5. Tareas activas y bloqueos

| Quién | Tarea | Estado |
|---|---|---|
| Humano | Pan-tilt: revisar fuente de 12 V bajo carga, cableado del power hub del U2D2 y masa común; cambiar cable USB del U2D2 | **bloquea** el seguimiento de rostro |
| Humano | HOME-SEC: revocar el token (también está en el registro de Orca); contraseña de la Jetson | pendiente |
| Humano | Aprobar `python3 -c` en la pestaña de Antigravity (HOME-40, dispatch `ctx_40a871d802de`) o cerrar ese worker | `BLOCKED_WORKER` |
| Claude | Teleop: prueba completa en el robot real, ajuste fino con `track_mm`; luego grabación de demostraciones | siguiente |
| Claude | Receptionist: probar en el robot el barrido nuevo (2 filas, ~16 s), seguimiento leve y la revisión del pan-tilt | código listo (`SIM_PASS` parcial) |
| Claude | HOME-46 mesa en la escena MoveIt; HOME-43 correr `torso_state` con el ESP32 conectado | pendiente |

## 6. Cambios de esta sesión (detalle en Git)

- `0c282ec`…`1a901f3` — Teleop Meta Quest 2 (WebXR por `adb reverse`, sin WiFi): pose 6D con embrague, IK DLS (modelo DH
  del SDK, validado contra `piper_kinematics`), filtro de temblor, sensibilidad de giro, modo real con X de hombre
  muerto + HOME primero + freno al soltar, Quest despierto sin el visor, gatillo pinza abierta/50 %.
- `a4e67b5` — Pan-tilt: desenvoltura de vueltas (el tilt quedaba corrido 1 vuelta → sobrecarga HW 32), `pwm_limit`, `position_p_gain`.
- `c9e4121`, `e10d7f3`, `0e71811`, `f97e6f0`, `b236945` — Receptionist: revisión del pan-tilt al arrancar, centrado en la
  puerta, seguimiento leve del rostro, barrido en dos filas más bajo y lento (~16 s), sin frases de pantalla táctil.
- `2ed3295`, `42134a2`, `f16f457`, `ea3016a` — Manipulación: FollowJointTrajectory desplegado, pinza paralela a la
  trayectoria (arregla "trayectoria perdida"), objetivo retenido, agarre multiorientación (~22–25 cm), ACM por muestreo,
  estado del elevador, misiones compuestas (`demo_fetch_bottle` SUCCEEDED en sim).
- Incidentes: `move_group` del usuario terminado por error (corregido, la sim solo cierra lo propio); commit vacío tras reinicio (reparado).

## 7. Decisiones vigentes

| Fecha | Decisión |
|---|---|
| 03/11 | Modo `DESARROLLO`: autonomía de software con actuadores; movimiento físico solo con supervisión y sesión acordada. |
| 03/11 | Teleop del brazo real: siempre HOME primero y X como hombre muerto; el nodo nunca habilita el brazo. |
| 03/11 | Navegación: estudiar sí, modificar no. Solo ROS 2; sin Google ni pagos. |
