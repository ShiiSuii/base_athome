# base_athome_skills

Habilidades para RoboCup@Home 2026.2 y la **capa de misiones** (planes deterministas supervisados).
No modifica Nav2 ni el driver de la base. La navegación usa `navigate_to_pose`;
el docking final publica `/cmd_vel` y mueve la base.

## door_open_detector (señal de inicio, reglamento §3.6.8)

El robot espera fuera de la arena frente a la puerta cerrada. El nodo mide la mediana de la
distancia en una ventana frontal del `/scan`, aprende la distancia a la puerta cerrada y publica
la señal cuando la distancia crece `open_margin_m` durante `open_scans_required` scans seguidos.

| Interfaz | Tipo | QoS |
|---|---|---|
| sub `/scan` | sensor_msgs/LaserScan | sensor data |
| pub `/start_signal/door_open` | std_msgs/Bool (`true` una vez) | transient_local (latched) |
| pub `/start_signal/status` | std_msgs/String: `CALIBRATING`, `WAITING_CLOSED`, `OPEN`, `NO_DOOR` | transient_local |

`NO_DOOR`: no hay nada a menos de `max_closed_distance_m` al frente → usar la señal manual de inicio.

```bash
ros2 launch base_athome_skills door_open_detector.launch.py
```

Pruebas: `python3 -m pytest -q src/base_athome_skills/test` (lógica, 8 casos) — `UNIT_PASS`;
nodo probado con LaserScan sintético en dominio 77 — `SIM_PASS`. Con el Hokuyo real: `PENDIENTE`
(verificar `center_deg` según el montaje del láser).

## mission_executor (capa de misiones, HOME-41)

Convierte un **plan estructurado** (JSON/YAML, nunca órdenes libres de un LLM a los motores) en una
secuencia de habilidades con reintentos, timeouts, cancelación y recuperación. Núcleo puro y testeable
en `base_athome_skills/mission/` (`skills.py` registro, `plan.py` formato/validación/GPSR,
`executor.py` ejecución, `places.py` lugares); `node.py` es el adaptador ROS 2.

**Decisión:** ejecutor secuencial propio en Python en vez de BehaviorTree.CPP o `py_trees`. El resto del
HRI ya es Python, `py_trees` no está instalado y BT.CPP obligaría a escribir nodos en C++. El contrato
(plan + `/mission/status`) permite migrar a BT más adelante sin cambiar a los clientes. Reutiliza los
lugares de `semantic_map.yaml`/`waypoints.yaml` (solo lectura) y la acción `navigate_to_pose`
existente. `mission_sequencer.py` sigue funcionando: sus pasos se migran a skills de a uno, cuando se validan.

### Commander: pedidos en lenguaje natural (HOME-57)
`mission/command_parser.py` compila un pedido en español o inglés a un plan del ejecutor, sin LLM y sin
adivinar: agrega pasos implícitos (mirar antes de buscar, buscar antes de agarrar), recuperación y `success_if`,
entiende "si no la ves ...", y rechaza con un motivo los lugares desconocidos y lo no implementado (personas, responder).
`mission/commander_node.py` valida el plan con las reglas del ejecutor y lo publica en `/mission/run`.
En la NUC, el alias `commander` (`scripts/commander.sh`) trabaja con el robot real y el micrófono: levanta Nav2 si no está,
el ejecutor (con manipulación) y el commander. El pedido se dice en inglés porque el ASR es `base.en`. El robot repite el plan
y solo actúa si le contestan "yes"; silencio o cualquier otra respuesta no ejecuta nada. "Stop" cancela una misión en curso.
No corre junto al receptionist, que usa el mismo micrófono. Prueba del diálogo: `test/test_commander_voice.py` (dominio 77).
```bash
simulation/scripts/sim_commander.sh      # sim + ejecutor + commander; abrir http://127.0.0.1:8770
simulation/scripts/sim_commander.sh "anda a la mesa, busca una botella y llevala a la cocina"
```
La web muestra el plan antes de ejecutar y el avance en vivo. Por tópico (`/commander/command`) solo ejecuta con
`auto_execute:=true`; "stop"/"cancelá" cancela siempre. En dominio 10 exige `real:=true`, y el ejecutor mantiene
sus guardas (`mode:=real` con `supervised:=true`). Estado: `SIM_PASS` 2026-11-04; con Nav2 real `PENDIENTE`.

**Objetos del mapa.** Lee `objects` y `furniture` de `semantic_map.yaml` (se colocan en la pestaña Objetos del editor):
"agarrá el objeto 1 y llevalo a la cocina", "traé las pringles", "poné el 2 en la mesa de centro", "andá al objeto 1".
Va al punto de aproximación del objeto, lo busca con la cámara y lo agarra. Si la clase coincide con varios, pide el
número; un lugar nombrado antes manda ("andá a la cocina y buscá una botella" busca ahí). Un mueble sin waypoint usa
el punto del objeto de menor número que tenga encima. La posición del objeto en el mapa no guía el agarre: lo decide
la cámara, por clase (`other` usa el nombre).

**Sacar la basura.** "take out the trash" / "sacá la basura" exige el waypoint `trash_drop` (botón en el editor de mapa).
Orden, para no golpear nada: ver el gesto y la bolsa → Nav2 hasta 0,40 m antes → brazo a `pre_grasp_floor` → avanzar en línea
recta con el elevador arriba (la pinza pasa por encima) → bajar → agarrar con la cámara de muñeca (detector de bolsas; la altura la da la cabeza) → subir → ir → bajar →
soltar → subir → recién ahí HOME. La bolsa viaja en la pose de piso, lejos del cuerpo. Demo en pantalla: `sim_basura`.
La pose se enseña a mano con `ensenar_pregrasp`, con el elevador arriba: de ella salen dónde se para el robot y cuánto baja.

### Plan
```yaml
name: demo
steps:                       # flujo normal
  - {id: go, skill: navigate_to, args: {place: door}, retries: 1, timeout_s: 180, on_failure: sorry}
  - {id: hi, skill: speak, args: {text: "Hello"}}
recovery:                    # solo se alcanza por on_failure
  - {id: sorry, skill: speak, args: {text: "I could not get there"}}
```
`on_failure`: `abort` (por defecto: la misión se detiene) · `continue` (fallo tolerado y explícito) ·
`<id>` (salto de recuperación; hacia adelante = la misión termina `FAILED` aunque la recuperación
funcione; hacia atrás = se re-ejecuta). Máximo 20 saltos. El plan se **valida antes de mover nada**:
skill desconocida o no implementada, argumentos, lugares inexistentes, `home` faltante, ids, límites.

### Habilidades (estado real)
| Skill | Estado | Notas |
|---|---|---|
| `navigate_to {place}`, `navigate_sequence {places}`, `return_home` | available | cliente de `navigate_to_pose`; cancela el goal y espera su cierre |
| `speak {text}` | available | TTS de la NUC (`RoboCupSpeaker`) en modo real; en sim solo se registra |
| `listen {prompt?, expect?, listen_timeout_s?}` | available | `VoiceClient` (`/voice/*`); no probado contra Whisper real desde el ejecutor |
| `wait {seconds}` | available | cancelable |
| `find_person {gesture?, posture?, clothing?}` | available (`UNIT_PASS`) | requiere al menos un descriptor visual; barre con `bag_finder` y se acerca; mueve cabeza/base, exige confirmación en modo real; nombres pendientes |
| `detect_object`, `pick_object`, `place_object`, `follow_person`, `guide_person`, `deliver_object`, `answer_question` | consultar `/mission/skills` | el registro `default_registry()` contiene el estado vigente de cada habilidad |

### Interfaces
| Interfaz | Tipo | Notas |
|---|---|---|
| sub `/mission/run` | std_msgs/String | plan JSON/YAML; se ignora si hay una misión en curso |
| srv `/mission/cancel` | std_srvs/Trigger | cancela el paso actual (goal de Nav2 incluido) y nunca arranca el siguiente |
| pub `/mission/status` | std_msgs/String JSON, transient_local, en cada cambio y a 1 Hz | `{mission_id, name, mode, state, current_step, started_at, finished_at, message, steps[{index, id, skill, args, state, attempts, max_attempts, started_at, finished_at, duration_s, message, error}], log[]}`; estados `IDLE/RUNNING/SUCCEEDED/FAILED/CANCELED` y por paso `PENDING/RUNNING/SUCCEEDED/FAILED/SKIPPED/CANCELED` |
| pub `/mission/skills` | std_msgs/String JSON, transient_local | registro con contratos |

`find_person` pide `/bag_finder/request` con `mode:"find_person"` y
`descriptor:{gesture|posture|clothing: texto}`. Recibe `/bag_finder/result`
con `{id, ok, person:{xy:[x,y]}|null, reason}` en coordenadas `map`.
Solo se acerca a coincidencias confirmadas por `person_matcher`; ante incertidumbre
falla con `no_matching_person`. Protocolo, cobertura medida y límites en
[`GPSR_PERSONAS.txt`](../../docs/GPSR_PERSONAS.txt).

### Seguridad
- `mode:=sim` (por defecto) **se niega a arrancar en `ROS_DOMAIN_ID=10`** (chequeo antes de `rclpy.init`).
- `mode:=real` exige `supervised:=true`: `navigate_*` mueve la base → solo con supervisión humana (`AGENTS.md` §3).
- `dock_front`, `dock_table` y `undock_table` exigen llegadas de `/scan` y `/odom` de
  no más de 0.3 s, medidas con reloj monotónico. Comprueban la frescura cada ciclo de
  50 ms y, al acercarse, también después de calcular la decisión. Si falta un sensor
  o vence, publican velocidad cero al salir y fallan con `no_scan`, `no_odom`,
  `stale_scan` o `stale_odom`; no esperan al timeout de la habilidad.
- Ante una pared, el docking frontal considera cualquier punto del contorno de un
  cluster que entre en el corredor de avance. Basta el mínimo configurado por
  `min_cluster_points` (3 por defecto) para bloquear un obstáculo interpuesto.

### Uso y pruebas
```bash
# simulación (dominio 42)
ROS_DOMAIN_ID=42 ros2 launch base_athome_sim sim.launch.py rviz:=false &
ROS_DOMAIN_ID=42 ros2 run base_athome_skills mission_executor --ros-args -p semantic_map:=<mapa con los lugares>
ROS_DOMAIN_ID=42 ros2 topic pub --once /mission/run std_msgs/String \
  "{data: '$(python3 -c "import json,yaml;print(json.dumps(yaml.safe_load(open('src/base_athome_skills/missions/demo_a_b_voice_c_home.yaml'))))")'}"
ROS_DOMAIN_ID=42 ros2 service call /mission/cancel std_srvs/srv/Trigger
```
Demo `missions/demo_a_b_voice_c_home.yaml` (A → B → voz → C → casa). Con el mapa semántico actual se
**rechaza** (faltan `kitchen` y `home`): hay que medirlos y agregarlos con el editor de mapa.
Pruebas: `python3 -m pytest -q src/base_athome_skills/test` (8 puerta + 21 misiones) — `UNIT_PASS`.
`SIM_PASS` 2026-11-03 con `base_athome_sim`: demo completa 41,4 s; timeout + reintento + recuperación;
cancelación a mitad de navegación (goal cancelado, base quieta). Con Nav2 real: `PENDIENTE`.

## Restaurant: aproximación y barrido

Si Nav2 corta la vuelta de búsqueda, se aprovechan los clientes con mano levantada
confirmada que ya se vieron. Cuando alcanzan para los clientes pendientes, se omite
el barrido adicional de cabeza (hasta 80 s). Se excluyen el barman, los clientes
ya visitados y las detecciones duplicadas; si faltan clientes, se conserva el barrido.
Restaurant solicita una ventana que cubra ambos giros y la locución intermedia,
limitada por el plazo global; `stop_scan` la termina al acabar la vuelta.
PENDIENTE: `bag_finder` aún limita esa vista a 55 s (timeout 60 s), de modo que
puede cerrarla antes de terminar el segundo giro en una simulación lenta.
El nodo limita la búsqueda a
`max_no_progress_rounds=3` rondas sin servir a otro cliente y termina en `FAILED`
con `reason=no_customer_progress`; cada entrega reinicia ese contador. El contrato
puro acepta `max_no_progress_rounds=None` para conservar el comportamiento anterior
hasta el plazo global en adaptadores existentes.

`restaurant_node` consume `/global_costmap/costmap` (`nav_msgs/OccupancyGrid`,
transient local, marco `map`). La sesión evalúa 12 ángulos por radio (standoff,
0.8 y 1.1 m), ordenados por distancia al robot, con radio circular de 0.3 m:
toda celda intersectada debe tener costo conocido menor que 50. Sin candidato
libre o sin grilla conserva la aproximación por la recta. No garantiza un camino:
Nav2 lo resuelve; ante `ABORTED` intenta una sola alternativa libre por visita
(también al entregar). No reintenta ante rechazo, cancelación o timeout.
`navigation_timeout_s` (positivo, por defecto 90 s) limita la espera del resultado
por goal y solicita cancelación al vencer. El goal o su aceptación pendiente
permanecen registrados hasta el resultado terminal; el ACK de cancelación no
libera la operación. Mientras falta esa confirmación no termina la navegación
ni se admite otro goal o reinicio de sesión. Una aceptación posterior a 10 s,
al deadline global o a stop se cancela al recibirla. Una caída del servidor
puede mantener esa espera hasta recuperar el resultado.

Durante la navegación abre escucha no bloqueante con `VoiceClient` y cancela
ante la palabra `stop`, también con mayúsculas y puntuación (`STOP!`). Cierra
ese pedido de voz antes de pasar al diálogo. La sesión usa reloj monotónico y
pasa `deadline_s` absoluto a `navigate_fn(x, y, yaw, deadline_s=...)`;
el deadline global cancela el goal y termina en `TIMEOUT` sin enviar un retorno
a la barra. La recuperación requiere un nuevo inicio explícito.

Sin TF válida `map → base_footprint`, espera hasta 5 s, acotados por el plazo
global; si no aparece, aborta sin inventar una pose. El arranque `restaurant.sh`
rechaza navegación, AMCL, map_server o SLAM previos antes de iniciar procesos.
Inicia su propio `restaurant_nav` y exige SLAM activo en modo `mapping`, sin
AMCL/map_server, con el overlay de planner esperado y la acción disponible.
Arranca `torso_state` con `allow_motion:=false`: este flujo no usa el elevador.
Estos controles no modifican los parámetros funcionales de la navegación
habitual. Validación con Nav2, ASR y hardware reales: pendiente.

Al iniciar Restaurant anuncia "I will search for the barman. Barman, please raise
your hand above your head as high as you can." y "Searching for people.".
Mira al frente (pan 0, tilt de búsqueda actual: 0°) y busca manos alzadas en
esa vista. Si no encuentra a nadie, gira la base mediante la acción Nav2 `spin`
en pasos de +30°, usando la velocidad lenta existente del perfil Restaurant,
y detecta entre pasos con la cabeza fija. El límite total es 360°. Al ver a una
persona deja de girar, renueva el lease `track_customer` durante "Are you the
barman? Please answer robot yes or robot no." y, si responde sí, registra su
cara como `barman` y su posición en `map` (`barman_position`). Al volver a la
barra conserva el punto inicial y se orienta hacia esa posición. El barman se
excluye como cliente por cara o cercanía menor que 0.8 m. Si responde no, no
confirma o no aparece tras la vuelta, lo anuncia y conserva la pose inicial
como barra, sin más giros de búsqueda del barman.

La búsqueda de clientes anuncia "Searching for people." antes de cada pedido
al detector y mantiene la base quieta. Envía `raised_hands` con
`"pans": [-90.0, 90.0, 0.0]` (derecha, izquierda, centro), `head_dps=12.0`,
`view_s=2.5` y `stop_on_first=false`, para reunir el lote visible. El barman usa
`"pans": [0.0]`, `stop_on_first=true`, hasta 14 s por vista. Sin `pans`,
bag_finder conserva su default `0, -90, 0, 90, 0`; GPSR/bag no se modifican.
Espera feedback fresco de cabeza y cuenta imágenes distintas con su TF.
Atiende primero las posiciones medidas y más cercanas al punto inicial.
Cada giro tiene `spin_timeout_s` (positivo, 40 s por defecto), acotado por
el plazo global y cancelación por voz `stop`, usando la misma espera de
resultado terminal que navegación. Si falla, mira desde donde quedó y sigue
con la barra inicial si nadie se confirma, sin más giros relativos.

Callbacks opcionales del contrato puro: `spin_fn(angle_rad) -> bool` debe
cancelar y esperar resultado terminal ante stop o plazo;
`detect_barman_fn(timeout_s, min_frames)` debe detectar con la cabeza frontal
fija (si falta, se usa `detect_customers_fn`); `track_person_fn(person)` y
`stop_tracking_fn()` inician/cancelan el seguimiento de confirmación. La captura
facial usa el contrato existente `enroll_face_fn("barman", person)`. Sin Spin
se evalúa una sola vista; sin callbacks de seguimiento no se controla la cabeza
por lease. Validación física del flujo nuevo: pendiente.

Solo el overlay `nav2_restaurant.yaml` usa un costmap global móvil de 20 × 20 m
en `map`: el tamaño inicial de SLAM ya no deja a la base fuera del costmap.
Mantiene StaticLayer, obstáculos, inflado y espacio desconocido. El publicador
de `/scan_nav` ofrece RELIABLE con profundidad 2; su entrada sigue con QoS de
sensor. No cambia el YAML de navegación habitual.

`restaurant.sh` (robot real) y `commander.sh` arrancan `audio_keepalive.py`
con la sesión. Cada 45 s genera un tono de 150 ms, 220 Hz y amplitud 0.3 %,
dirigido al sink Bluetooth o al predeterminado si no hay Bluetooth.
Configurar con `AUDIO_KEEPALIVE_INTERVAL_S` o `--interval` al ejecutar el programa.
Saltea el ciclo si el sink está RUNNING, su estado es desconocido o TTS/beeps
tienen el bloqueo compartido de `base_athome_perception.audio_lock`.
Requiere reconstruir/desplegar también `base_athome_perception`; no reproduce
un tono inmediatamente al iniciar. La sesión cierra el proceso al salir.

`UNIT_PASS` 2026-11-09: pruebas con acciones, reloj, cabeza y audio falsos.
Ventana móvil con SLAM/Nav2, giros y persistencia de la conexión Bluetooth en
hardware: `PENDIENTE`; estas pruebas unitarias no iniciaron nodos ni reprodujeron audio.
