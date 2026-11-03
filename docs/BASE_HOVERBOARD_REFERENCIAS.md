# Investigación comparativa — bases móviles con motores de hoverboard (ROS 2)

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

## 1. Objetivo y alcance

Tarea permanente para Claude y Antigravity: explorar libremente ingeniería de plataformas móviles construidas con motores brushless de hoverboard (6,5 pulgadas y variantes) y compararlas con **nuestra implementación**. Buscar nuevas referencias públicas y discutirlas; **no instalar firmware ni sustituir un controlador que funciona**. Resultado esperado: catálogo versionado de repositorios, matriz de compatibilidad, una propuesta de experimento en simulación y mejoras documentadas a evaluar con el usuario.

### Arquitectura de referencia Base@Home: lo que está documentado

En el `README.md` público de https://github.com/ShiiSuii/base_athome figura una base diferencial con dos motores hub BLDC 6,5", realimentación Hall, comunicación UART por USB-TTL (se menciona `/dev/ttyUSB0`, 115200 baud, 50 Hz), odometría y TF en NUC, Hokuyo UST-05LX y Nav2 en ROS 2 Jazzy. Estas son afirmaciones del **README**, NO verificación de hardware actual. El README aún llama a Jetson Humble, mientras HANDOFF 30/10 informa Jazzy; por eso no extrapolar versiones ni perfiles de software de publicaciones antiguas.

**Confirmado por el usuario:** la navegación de Base@Home funciona y debe permanecer sin cambios. Su funcionamiento exacto, versión de firmware de hoverboard, controlador, encoder/Hall, tasa odom, wheel radius, wheel separation, conversión rpm→rad/s y mecanismos de watchdog se deben **leer del código y del host**, nunca inventar.

### Fuentes verificadas (consultadas 03/11/2026)

| ID | Fuente pública | Tipo | Para estudiar | Advertencia |
|---|---|---|---|---|
| HB-01 | https://github.com/DataBot-Labs/hoverboard_ros2_control | Implementación ROS 2 `ros2_control` | `hardware_interface`, interfaces wheel states, control y bringup | No implementar en producción; comprobar hardware y licencia |
| HB-02 | https://github.com/TheHassanShahzad/OpenBase2 | Robot ROS 2 diferencial con motores hoverboard | Arquitectura hardware_driver/odom/diff_drive, URDF, bringup | Proyecto pequeño; comprobar madurez y cobertura de tests |
| HB-03 | https://irl.tech.cornell.edu/mobile_hoverboard_robot_base/ | Cornell Tech HRI | Plataforma low cost, motores hoverboard con ODrive, ROS 2 Humble, calibración, wiring | Usa ODrive y dos Raspberry Pi; no copiar configuración al hardware local |
| HB-04 | https://github.com/IRL-CT/mobile_hoverboard_robot_base | Repositorio Cornell | Manual hardware, ensamblaje, electrónica y teleoperación | Contrastar documentación con licencia y commits |
| HB-05 | https://github.com/hoverboard-robotics/hoverboard-driver/tree/humble | Driver ROS 2 de UART hoverboard | Conversión control diferencial, odometría serial, nodo driver | Rama ROS 2; **no usar README/comandos ROS 1 de `master`** |
| HB-06 | https://github.com/EFeru/hoverboard-firmware-hack-FOC | Firmware FOC STM32 para hoverboard | Arquitectura control BLDC, serial, torque/velocidad, protecciones | **Solo estudio.** Flashear motor es intervención de alto riesgo |
| HB-07 | https://hoverboard-robotics.github.io/ | Comunidad/galería | Tipos de proyectos de servicio con hoverboard, wiring e historia | Parte del material trata ROS 1; lectura histórica únicamente |
| HB-08 | https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html | Documento oficial ROS 2 Jazzy | Control diferencial, límites, timeout, odometría realimentada | No reemplazar algoritmo propio por moda |
| HB-09 | https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/ | Nav2 documentación | odom→base_link/map, TF, footprint, lifecycle, sensores | Recomendaciones como lectura, sin ajustar parámetros del robot |
| HB-10 | https://github.com/ros-controls/ros2_control | ROS 2 framework | Hardware interfaces, test harness, controlador mock | Considerar coste de migración y tiempo real |
| HB-11 | https://github.com/RoBorregos/home2 | Referencia RoboCup@Home ROS 2 | Orquestación, percepción, manipulación y arquitectura | Equipo/robot distinto; no asumir compatibilidad |
| HB-12 | https://github.com/RoboCupAtHome/RuleBook/releases/tag/2026.2 | Reglas RoboCup 2026 | Requisitos y métricas competición | No sustituir por rama principal de 2027 |

## 2. Plan estructurado de investigación

### Línea A — Hardware y electrónica del hoverboard

1. Identificar mediante inventario físico/documental de qué hoverboard provienen motores/controlador (modelo hub, sensores Hall, conectores, placas). **No abrir ni energizar cajas, ni medir alto voltaje sin humano.**
2. Desde fuente y repo, localizar qué controla realmente los motores: placa de hoverboard original y firmware FOC modificado, controlador ODrive, ESC, STM32, interfaz UART u otra implementación. No asumir que usa EFeru.
3. Identificar protocolos y protección: velocidad máxima, aceleración, watchdog, parada por pérdida de comunicación, comportamiento tras reset, E-stop, calentamiento, corriente y brownout. Documentar dónde se implementan y qué prueba lo verifica.
4. Examinar el dominio eléctrico separado (motores hoverboard ~batería 10s3p, según planificación previa del equipo; **confirmar en campo**), BMS, fusibles, masa, EMI, conexión USB-TTL, aislamientos y riesgo de retorno de tierra. **Nada de cambiar cableado automáticamente.**
5. Determinar si Hall feedback representa posición absoluta, conteos por revolución o velocidad estimada, cómo se reconcilian dirección/signo, pérdida de paquetes, saturación y saltos.

### Línea B — Software de base diferencial ROS 2

- Driver: `src/hoverboard_base` y paquetes relacionados. Comprobar `rclpy/rclcpp`, UART baud/rate, framing/CRC, timeout y reconexión, configuración puerto por udev, límites de inputs y APIs ROS.
- Twist→ruedas: cinemática, wheel radius/separation reales, saturación, comandos nulos, límites de aceleración; buscar si se usan `cmd_vel`, `cmd_vel_smoothed` y qué nodo es dueño exclusivo de la cadena. **Solo leer, no publicar.**
- Odometry: encoder/Hall, timestamp monotónico, covarianzas, integración xyθ, giro in situ, deslizamiento, odom reset y drift. Revisar EKF con o sin IMU sólo si existe y cómo se conecta.
- TF: `map → odom → base_link` y árbol de sensores con un publicador por transformación; distinguir estado medido de comandado.
- Navigation: describir Nav2 (planner, controller, BT, AMCL/SLAM), comportamiento ante goal, recuperación y obstáculos; verificar parámetros **sin tocar**. La navegación ya funciona: comparar objetivos y telemetría pasivamente.
- Estabilidad: CPU/RAM, latencia de callback, jitter, tramas perdidas, QoS, DDS, tiempo de relojes, saturación `cmd_vel` sin alterar ejecución.

### Línea C — Simulación y experimentos reversibles

- Reproducir contratos ROS/TF en **`ROS_DOMAIN_ID=42` con `ATHOME_SIM=1`**, `base_athome_sim`, bags públicos o fixtures sintéticos sin datos de personas.
- Probar edge cases de transformaciones, timestamp, pérdidas de trama simuladas y llamadas Nav2 mock.
- Dibujar trayectoria y odometría en análisis offline, nunca aplicar al sistema.
- Fijar **baseline** (capturas originales y pruebas actuales) antes de cualquier recomendación que implique cambiar algoritmos.

### Línea D — Diseño mecánico y URDF

- Comparar ubicación real de motores, puntos de apoyo, centro de gravedad, footprint estético vs footprint operativo, elevador, torso y brazo con robot de referencia, sin asumir medidas faltantes.
- El editor URDF **puede ajustar solo visualización/candidatos**; no sobrescribe footprint Nav2, rueda calibrada, TF navegacionales ni parámetros de odometría.
- Analizar riesgo de vuelco con columna elevada/Piper extendido como hipótesis. Las mediciones reales y cargas deben venir del equipo antes de afirmar márgenes.

## 3. Tabla de comparabilidad que debe completar cada agente

```markdown
| Repo / commit | Fecha | Licencia | ROS distro | Tipo motor/controlador | Feedback Hall/encoder | Interface | URDF | TF/Odom | Watchdog/E-stop | Tests | Reutilizar? | Justificación |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
```

No presentar `STAR count` como calidad. Usar compatibilidad de versión/licencia, completitud de tests, seguridad y claridad de API. Código fuente ajeno sólo se incorpora con licencia compatible, archivos de atribución y revisión técnica.

## 4. Ejemplos de consultas para seguir investigando

- `ROS2 hoverboard 6.5 inch differential drive hall UART odometry github`
- `ROS2 Jazzy hoverboard motors ros2_control hardware_interface`
- `hoverboard firmware FOC robot autonomous navigation watchdog serial fail safe`
- `hoverboard BLDC odometry hall encoder wheel separation calibration test`
- `RoboCup@Home open platform hoverboard base lessons learned ROS2`
- `ROS 2 robot_localization odometry yaw drift hall wheel slip differential drive`

Nunca copiar código del primer resultado sin examinar licencias y dependencias. El navegador puede visitar documentación pública de GitHub, ROS, NVIDIA y universidades; **no entrar a cuentas privadas** ni hacer compras.

## 5. Producto final y decisión

Cada investigación de 2–4 horas debe terminar con: (a) 3–5 fuentes verificables; (b) comparación frente al robot real con rutas de código y SHA; (c) lista de diferencias; (d) al menos un test no invasivo o plan de test; (e) una propuesta `NO CAMBIAR / INVESTIGAR / CANDIDATO FUTURO`, sin tocar navegación; (f) entrada en `PROPUESTAS_NAVEGACION.md` cuando corresponda.

**Indicadores útiles para una propuesta**, no requisitos para cambiar la base: tasa y varianza de odometría, deriva en bucles cerrados, paquetes perdidos del protocolo, latencia de control, consumo CPU, tiempo de recuperación, estabilidad TF, colisiones/tasa de éxito, errores en goal. Nunca inventar las métricas; si no existen, declarar `NO MEDIDO`.
