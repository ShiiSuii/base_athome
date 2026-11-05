# STATUS.md — Estado vivo de Base@Home

Corto y actualizado: se **reemplaza** lo viejo, no se acumula (`AGENTS.md` §9). Lo histórico se
recupera por commit (§8). Editado por el integrador (Claude).

**Última actualización:** 2026-11-05 · Claude Code · modo `DESARROLLO` · receptionist revisado, guía del video de cualificación.

## 1. Estado comprobado al pausar

| Ítem | Estado | Evidencia |
|---|---|---|
| NUC | Ubuntu 24.04, kernel 7.0.0-42 (reiniciada hoy), Jazzy, CycloneDDS, dominio 10 | `NUC_VERIFIED` |
| Jetson | L4T R39.2.1, Jazzy; `jetson-head` activo | `JETSON_VERIFIED` |
| Piper | energizado, **habilitado** con feedback válido (`/arm/status` leído 2026-11-04; deshabilitarlo lo deja caer: hacerlo con el usuario presente); controlador md5 `2e347c49` | `JETSON_VERIFIED` |
| Pan-tilt | **falla eléctrica abierta**: 118 desconexiones USB del U2D2 el 04/11, solo ese puerto (`Cannot enable. Maybe the USB cable is bad?`, error -71). Si la revisión inicial falla, el receptionist desactiva el seguimiento. `pan_tilt_params.yaml` no existe en la Jetson: usa valores por defecto del nodo. **Sentido del pan corregido** (`pan_direction` +1, lo observó el usuario desde atrás): falta confirmarlo con `V` del calibrador. Cable cambiado: 3 caídas a las 19:32, falta probarlo con torque y movimiento (`M`) | `JETSON_VERIFIED` |
| Teleop Meta Quest 2 | **en pausa** por pedido del usuario; nodo detenido en la Jetson. Fidelidad validada en sim; seguimiento real pendiente | `SIM_PASS`; real parcial |
| Simulación | cerrada | — |
| Commander | alias `commander`: pedido por micrófono (inglés), dice el plan y actúa con "yes"; "stop" cancela; web en `:8770`. Diálogo probado con ASR y ejecutor falsos (6 tests). Whisper real: 3 frases bien, ~1,4 s; el resto sin resultado en 14 s porque el ASR detectó voz continua (posible ruido del ambiente). **Nunca ejecutó una misión en el robot.** Agarre real: exige `torso_state` corriendo | `UNIT_PASS`/`SIM_PASS`; real `PENDIENTE` |
| Tests offline | Jetson scripts 32 + Quest 18, percepción 122, skills 76 (1 skipped), bringup (elevador) 6, sim 7, pan-tilt 8, agentes 40 | `UNIT_PASS` 2026-11-04 (skills) |

## 2. Rama y Git

- Rama `claude/home-41-piper-mission` **subida a GitHub** el 2026-11-04 (repo público). `main` = `1da2388` sin cambios: merge a `main` = decisión humana (PR pendiente). HOME-SEC: el token viejo sigue sin revocar.
- Cambios del usuario sin commitear (no tocar): `semantic_map.yaml`, `waypoints.yaml`, `arm_poses_and_limits.yaml`
  `waypoints.yaml` y `~/waypoints_base_athome.yaml` quedaron vacíos por un test (ver §6); regenerados desde `semantic_map.yaml` el 2026-11-04.
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
| Jetson | `pan_tilt_node` lanzado a mano (`/tmp/pan_tilt_manual.log`), reemplaza al de `jetson-head` | dejar: al reiniciar `jetson-head` vuelve solo, con el mismo código |
| Jetson | `adb` server + regla udev `/etc/udev/rules.d/51-meta-quest.rules` | dejar (necesarios para el Quest) |
| Jetson | `ros-jazzy-control-msgs` y `adb` instalados por apt | rollback: `~/deploy_backups/{control_msgs,adb}_20261103/installed.txt` |

## 5. Tareas activas y bloqueos

| Quién | Tarea | Estado |
|---|---|---|
| Humano | Pan-tilt: revisar fuente de 12 V bajo carga, cableado del power hub del U2D2 y masa común; cambiar cable USB del U2D2 | **bloquea** el seguimiento de rostro |
| Humano | HOME-SEC: revocar el token (también está en el registro de Orca); contraseña de la Jetson | pendiente |
| Humano + Claude | **Prioridad: cualificación 2027, entrega 6/12/2026** (HOME-62, `docs/VIDEO_CUALIFICACION.txt`). Receptionist en el robot: seguimiento y carga corregidos, falta probarlos | en curso |
| Claude | En pausa, retomar después (detalle en `ROADMAP.md`): commander con Nav2 real y personas (HOME-57, HOME-49), docking (HOME-58), detector World en Jetson (HOME-59), `guide_person` (HOME-60), teleop (HOME-52) | guardado |
| Humano | TDP: seguir `docs/TDP_INSTRUCTIVO.txt` desde la otra computadora | pendiente |
| Claude | Receptionist: probar en el robot el barrido nuevo (2 filas, ~16 s), seguimiento leve y la revisión del pan-tilt | código listo (`SIM_PASS` parcial) |
| Claude | HOME-46 mesa en la escena MoveIt; HOME-43 correr `torso_state` con el ESP32 conectado | pendiente |

## 6. Cambios de esta sesión (detalle en Git)

- Receptionist: el detector de muebles se carga en el pre-vuelo (bloqueaba la ventana ~4 s en medio del barrido) y el seguimiento de rostro responde más rápido (error máximo simulado de 16,6° a menos de 12° con un invitado que se mueve). Parámetros sin probar en el robot.

- Receptionist: espera ESPACIO al terminar de cargar (`--no-wait` lo omite; en competencia no se permite teclado, §3.6.8) y `P` repite la revisión del pan-tilt. Calibrador: `calibrar_pantilt`. `SIM_PASS`/tests con nodo falso; la cabeza real solo se leyó.
- **Incidente:** `commander` dejaba procesos vivos al cerrar la terminal (HUP sin trampa): el 04/11 quedaron 4 ejecutores reales y un Nav2 completo duplicando al de `navegacion`. Cerrados por grupo con el robot quieto. Arreglo: `session_lib.sh` cierra también con HUP y limpia restos de sesiones anteriores.

- Mapa: `map_server` solo lee el archivo al arrancar, así que un Nav2 abierto seguía con el mapa viejo. Ahora el editor lo recarga al guardar y `commander` al arrancar (`nav_map_sync.sh`, servicio `load_map`), sin cambios en archivos protegidos. Verificado el 2026-11-04: `/map` = PGM, los 200 obstáculos agregados son letales en el costmap global y AMCL no saltó.

- Commander + objetos: `places.py` (objetos y muebles), `command_parser.py` (número, nombre, clase; pide el número si hay ambigüedad), web con la lista numerada. Editor: aproximación frente al borde de la mesa, fuera de ella. Rollback: revertir el commit.
- **Incidente:** `test_semantic_map_manager_objects_integration` escribía los `waypoints` reales al llamar `save_data` (los vació en corridas del 03/11 y 04/11). Test aislado; archivos regenerados con door y living_room. Si existían otros waypoints solo en esos archivos, no se pueden recuperar.

- Commander (HOME-57, reemplaza HOME-40, cuyo worker terminó sin commits): `command_parser.py`, `commander_node.py`, `sim_commander.sh`. Rollback: revertir el commit; no toca al ejecutor.
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
