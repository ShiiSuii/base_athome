# base_athome_skills

Habilidades para RoboCup@Home 2026.2 y la **capa de misiones** (planes deterministas supervisados).
No modifica Nav2 ni la base: navega solo como cliente de `navigate_to_pose`.

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
```bash
src/base_athome_skills/scripts/sim_commander.sh      # sim + ejecutor + commander; abrir http://127.0.0.1:8770
src/base_athome_skills/scripts/sim_commander.sh "anda a la mesa, busca una botella y llevala a la cocina"
```
La web muestra el plan antes de ejecutar y el avance en vivo. Por tópico (`/commander/command`) solo ejecuta con
`auto_execute:=true`; "stop"/"cancelá" cancela siempre. En dominio 10 exige `real:=true`, y el ejecutor mantiene
sus guardas (`mode:=real` con `supervised:=true`). Estado: `SIM_PASS` 2026-11-04; con Nav2 real `PENDIENTE`.

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
| `detect_object`, `find_person`, `pick_object`, `place_object`, `follow_person`, `guide_person`, `deliver_object`, `answer_question` | **unavailable** | vocabulario GPSR sin implementación validada: el plan se rechaza |

### Interfaces
| Interfaz | Tipo | Notas |
|---|---|---|
| sub `/mission/run` | std_msgs/String | plan JSON/YAML; se ignora si hay una misión en curso |
| srv `/mission/cancel` | std_srvs/Trigger | cancela el paso actual (goal de Nav2 incluido) y nunca arranca el siguiente |
| pub `/mission/status` | std_msgs/String JSON, transient_local, en cada cambio y a 1 Hz | `{mission_id, name, mode, state, current_step, started_at, finished_at, message, steps[{index, id, skill, args, state, attempts, max_attempts, started_at, finished_at, duration_s, message, error}], log[]}`; estados `IDLE/RUNNING/SUCCEEDED/FAILED/CANCELED` y por paso `PENDING/RUNNING/SUCCEEDED/FAILED/SKIPPED/CANCELED` |
| pub `/mission/skills` | std_msgs/String JSON, transient_local | registro con contratos |

### Seguridad
- `mode:=sim` (por defecto) **se niega a arrancar en `ROS_DOMAIN_ID=10`** (chequeo antes de `rclpy.init`).
- `mode:=real` exige `supervised:=true`: `navigate_*` mueve la base → solo con supervisión humana (`AGENTS.md` §3).

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
