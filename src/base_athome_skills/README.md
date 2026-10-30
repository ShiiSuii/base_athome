# base_athome_skills

Habilidades para RoboCup@Home 2026.2 que **no controlan actuadores**. No modifica Nav2 ni la base.

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
