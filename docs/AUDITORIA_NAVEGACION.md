# Auditoría de navegación Base@Home — evaluación profunda de SOLO LECTURA

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

## Política estricta

El usuario declara que su navegación actual funciona. Claude y Antigravity tienen libertad para **entender cada detalle** de hardware, algoritmos, configuración, integración, herramientas y resultados. El análisis no puede modificar Nav2, SLAM, AMCL, costmaps, archivos de mapa, odometría, drivers hoverboard, TF navegacionales ni firmware de la base. Una mejora detectada se **propone** y se valida aparte sin desplegarla.

## A. Inventario y localización del código

Registrar checkout, SHA, host, paquete y función (la estructura exacta debe comprobarse en disco):

```bash
pwd
git rev-parse --show-toplevel
git status --short --branch
git log -1 --oneline
find src -maxdepth 3 -type f \( -name 'package.xml' -o -name '*.launch.py' -o -name '*.yaml' -o -name '*.xacro' \) | sort
rg -n 'cmd_vel|odom|diff_drive|hoverboard|Nav2|nav2|amcl|slam|footprint|EKF|tf2|robot_localization' src docs scripts || true
```

**Nunca usar `find` que recorra archivos de credenciales o datos sensibles ni volcar `.env`.** Registrar solo rutas técnicas. Si la NUC contiene cambios locales sin commit, respetarlos: no realizar reset ni branch switches peligrosos.

## B. Matriz funcional a completar

| Eslabón | Preguntas técnicas | Evidencia mínima | Estado inicial |
|---|---|---|---|
| Motores BLDC | Tipo, wheel radius, Hall, driver | código + inspección autorizada del hardware | NO REVALIDADO |
| Firmware/controlador | versión, protocolo UART, límites, timeout | archivos firmware, documentación, logs pasivos | NO REVALIDADO |
| Comando de velocidad | publicador y suscriptor `cmd_vel`, selector mux, smoothing | código, `ros2 topic info -v` | NO REVALIDADO |
| Odómetro | fórmula, covarianzas, frecuencia, timestamps, publisher | fuentes y `ros2 topic hz /odom` si no interfiere | NO REVALIDADO |
| TF | map/odom/base_link, ownership y latencia | código + `tf2_tools` o lecturas pasivas | NO REVALIDADO |
| LiDAR | driver Hokuyo, rate, timestamp, VLAN/red | código + status únicamente | NO REVALIDADO |
| Localización | SLAM/AMCL/EKF activos, startup, resets | configuración + lifecycle pasivo | NO REVALIDADO |
| Nav2 | planner, controller, recoveries, BT, targets | launch/config + observación de logs | FUNCIONAL SEGÚN USUARIO, DETALLES NO REVALIDADOS |
| Misión | actions/services, límites y feedback, waypoints | código cliente HRI y pruebas de mock | NO REVALIDADO |
| Safety | E-stop, watchdog, deadman, faults | hardware/firmware, runbooks, evidencias | NO REVALIDADO |

## C. Comandos de inspección **pasiva**

- `ros2 node list`, `ros2 topic list -t`, `ros2 service list`, `ros2 action list` para mapa de interfaces, tras comprobar `ROS_DOMAIN_ID` y host correcto.
- `ros2 topic info /odom -v`, `ros2 topic info /cmd_vel -v`, `ros2 node info /<nombre_verificado>` para owner/qos.
- `ros2 topic hz /odom` o `ros2 topic echo --once /odom` sólo en ventana sin ensayos físicos activos y con tiempo de muestreo acotado; jamás `ros2 topic pub` ni enviar goals.
- `systemctl status`, `journalctl` sobre unidades conocidas; logs acotados y saneados. No reiniciar Nav2 para "probar".
- Análisis offline con `ros2 bag info` de bag autorizado y sin datos privados, `pytest`, xacro y simulación aislada.
- Comandos `tf2_echo`/view_frames son suscriptores, pero consumen recursos. Usarlos solo si no afecta la navegación ni se solapa con otra tarea.

**Prohibiciones:** escribir en dominio operativo `10`, `ros2 service call` de control, `ros2 action send_goal`, `ros2 param set`, `ros2 lifecycle set`, `ros2 topic pub`, `teleop`, `rviz InteractiveMarkers` que envíen goals, cambios `udev` o CAN, cambios de red, reiniciar/kill servicios de navegación.

## D. Auditoría sin cambio funcional

### 1. Modelo cinemático

Documentar ecuaciones de diferencial para comparar con la implementación existente:

- `v = (v_r + v_l) / 2`
- `ω = (v_r − v_l) / b`
- `v_r = r·ω_r`, `v_l = r·ω_l`

Aquí `r` es radio efectivo calibrado y `b` separación de ruedas efectiva; **no tomar valores nominales si el código actual tiene calibración propia**. Examinar saturación, signo izquierdo/derecho, giro puro y deadband, sin modificarla.

### 2. Odometría/TF

Comprobar coherencia entre `odom`, `base_link`, velocidad y timestamps. Señalar cómo usa `robot_localization` (si existe), covarianzas, yaw y clocks. No cambiar fuentes, no añadir publicadores TF a la red activa.

### 3. Confiabilidad

Examinar seguridad ante comandos perdidos, watchdog, restart, timeout, permisos de puerto serial, reconexión, checksum/CRC, QoS, fase de boot de motores. Distinguir bugs de código reproducibles vs hipótesis. Si la evidencia requiere pruebas con movimiento, diseñar el protocolo y dejarlo `BLOCKED_APPROVAL`.

### 4. Comportamiento Nav2

Leer costmaps, AMCL, footprint, controller/BT y sus interfaces para comprender **por qué funciona** y qué precondiciones necesita HRI. Revisar la interacción con `NavigateToPose`, navegación por waypoints y cancelación; proponer adaptadores en módulos de habilidad, no parches dentro Nav2.

### 5. Módulo semántico

`semantic_map.yaml` y `waypoints.yaml` pueden ser candidatos editables en laboratorio offline con snapshots/validación de esquema. Aun así, no publicar goals ni modificar la configuración operativa. Si el archivo también gobierna el sistema en vivo, duplicarlo para pruebas y dejar cambios en rama hasta aprobación.

## E. Informe de resultados

Escribir `docs/collab/PROPUESTAS_NAVEGACION.md`, una ficha por hallazgo: ID, ubicación exacta, contexto, pruebas, riesgo, efecto estimado y confirmación del usuario. No inferir que un benchmark de otro proyecto prueba una mejora propia. Tabla de comparación con DataBot-Labs, OpenBase2 y Cornell en `BASE_HOVERBOARD_REFERENCIAS.md`.

### Criterio de finalización HOME-30

- [ ] Traza verificable motores→driver→odom→TF→localización→Nav2→cliente HRI.
- [ ] Lista de nodos, fuentes, ownership TF, topics/actions y riesgos, con SHA.
- [ ] Comprobación no invasiva de 1–3 interfaces si la red está libre.
- [ ] Mapa de módulos protegidos vs capa de habilidades editable.
- [ ] Propuestas no invasivas con aceptación/rollback y sus incertidumbres.
- [ ] **Cero archivos operativos de navegación modificados, cero comandos de movimiento.**
