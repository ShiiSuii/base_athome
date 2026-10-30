# Base@Home — Protocolo de auditoría profunda (NUC + Jetson por Ethernet)

**Propósito:** permitir que Claude Code y Antigravity, coordinados con Orca, comprendan *cómo funciona realmente* este robot de RoboCup@Home antes de reorganizar código, optimizarlo o agregar habilidades. Este documento es un **plan y plantilla de investigación**, no un informe de hardware validado.

**Estado actual declarado (última actualización 29/10/2026): NUC y Jetson ENCENDIDAS, interconectadas por un cable Ethernet; demás sistemas DESENERGIZADOS.** Acceso SSH y pruebas de software entre hosts autorizados; no activar actuadores. Marcar como `HARDWARE_BLOCKED` únicamente las validaciones de sensores/actuadores sin alimentación.

## 1. Jerarquía de fuentes y evidencia

1. **NUC verificada ahora**: sistema, interfaces y procesos visibles mediante comandos pasivos, con fecha y salida guardada.
2. **Código del checkout local**: fuente exacta (path, clase, función, versión de Git) de configuraciones/contratos; no equivale a prueba física.
3. **README y documentación técnica del repositorio**: intención arquitectónica; puede estar desactualizada.
4. **Manual del fabricante o documentación oficial**: capacidad nominal, no estado real de esta instalación.
5. **Información histórica del equipo**: hipótesis que requieren verificación con esquema, foto, etiqueta o medida segura por un humano.

Etiquetas obligatorias: `NUC_CONFIRMADO`, `JETSON_CONFIRMADO`, `ETHERNET_CONFIRMADO`, `CODIGO_VERIFICADO`, `DOCUMENTADO`, `HIPOTESIS`, `HARDWARE_BLOCKED`, `HARDWARE_VERIFICADO` (solo tras ensayo real autorizado). Anotar ruta/URL y fecha para afirmaciones importantes. Ante discrepancias: registrar ambas, no corregir configuraciones al azar.

## 2. Modelo inicial del robot (provisional; contrastar con código local)

| Subsistema | Host/propietario según README | Interfaz prevista | Alimentación/estado **ahora** | Qué verificar |
|---|---|---|---|---|
| Intel NUC i3-1220P, 16 GB | Cómputo principal, ROS 2 Jazzy/Ubuntu 24.04 | Ethernet/USB/ROS 2 | **NUC encendida** (usuario) | Sistema, RAM, paquetes, versiones, interfaces, procesos existentes |
| Base diferencial hoverboard | Driver conectado a NUC | USB-TTL/UART, `/odom`, `/tf` | No asumir energizada | Modelo de control, serial, watchdog, límites; navegación **solo lectura** |
| Hokuyo UST-05LX | NUC | Ethernet, `/scan`, posible IP `192.168.0.10` | No confirmado | Ubicación, IP, alimentación, TF, driver; sin configuración ni tráfico activo |
| Intel RealSense D435 | NUC, cámara RGB-D en pan-tilt | USB 3, tópicos `/camera/...` | No confirmado: podría recibir USB de NUC | Quién abre cámara, frames ópticos, ownership, QoS, alimentación real |
| NVIDIA Jetson Orin Nano 8 GB | Cómputo periféricos, ROS 2 Humble/Ubuntu 22.04 documentados | Ethernet, ROS_DOMAIN_ID 10 | **Encendida** (usuario); conectada a NUC por Ethernet | Confirmar SSH, placa/portadora, JetPack/UEFI, servicios, disco, GPU, dependencias ROS2 |
| AgileX Piper + pinza | Jetson | SocketCAN `can0`/`can1`, ROS 2 | Apagado/no verificado | Adaptador, tasas, unidades, joints, feedback, gripper, seguridad |
| Orbbec DaBai DC1 en muñeca | Jetson | USB, RGB-D eye-in-hand | Apagada/no verificada | Driver, calibración intrínseca/hand-eye, tf tool-camera |
| Cabeza pan-tilt 2× XL430 | Jetson | U2D2/serial Dynamixel 1 Mbps | Apagada/no verificada | IDs, modelo, cero, cableado 12 V, límites, feedback vs comando |
| Micrófono RØDE VideoMic GO II | Jetson | USB Audio, ASR/VAD | Apagado/no verificado | ALSA, pipeline y contratos de voz |
| Altavoz/pantalla facial | Por confirmar | Audio/display | No confirmado | Alimentación, hosts, puertos, estado y software |
| Elevador NEMA23, ESP32-S3 | Control documentado desde NUC | USB serial y pulsos al driver | No confirmado | ESP32, driver, homing, finales de carrera, alimentación, límites |

**Origen:** README público `https://github.com/ShiiSuii/base_athome`, secciones `System Architecture`, `Hardware Components & Pinout` y `TF Tree`, a contrastar con el checkout real. No sustituir esta tabla por suposiciones; reemplazar o complementar solo con evidencias.

## 3. Mapa eléctrico a reconstruir (NO ES UN ESQUEMA CERTIFICADO)

El historial de trabajo del equipo menciona **subsistemas de alimentación separados** (7S6P para NUC/LiDAR y posibles reductores, 10S3P para motores hoverboard, 7S5P para Jetson/elevación/ESP32 y un step-down de 12 V para Dynamixel). **No dar por válidas las asignaciones** sin verificar documentación eléctrica actual: el cableado pudo cambiar.

Entregable `docs/power-domains.md`:

| Dominio/batería | BMS y tensión nominal | Fusible y ubicación | Step-down / tensión salida | Cargas conectadas | Evidencia / incertidumbres |
|---|---|---|---|---|---|
| PWR-NUC | pendiente | pendiente | NUC 19 V? **por confirmar** | NUC, ¿LiDAR? | no verificado |
| PWR-DRIVE | pendiente | pendiente | pendiente | Hoverboard motores | no verificado |
| PWR-JETSON | pendiente | pendiente | pendiente | Jetson, ¿elevación, ESP32? | no verificado |
| PWR-AUX12 | pendiente | pendiente | 12 V? **por confirmar** | ¿Dynamixel? | no verificado |

Dibujar topología real batería → BMS → fusibles → convertidores → cargas, identificar GND compartidas y aislamiento donde corresponda, corrientes máximas, interruptores y paro; **ningún cambio de cableado, medición bajo tensión o energización por cuenta del agente**. Si existen documentos/fotos del montaje, cotejarlos sin inferir conexiones invisibles.

## 4. Rastreabilidad de software ROS 2

Por cada paquete/nodo generar una ficha normalizada:

| Campo | Registrar |
|---|---|
| Paquete/nodo/proceso/host | ruta, entrypoint, ROS distro, quién lo lanza |
| Responsabilidad | tarea exacta y dependencia física |
| Inputs | tópico/tipo/QoS o servicio/action, origen |
| Outputs | tópico/tipo/QoS, consumidores |
| TF/joints | frames padres/hijos, fuente, timestamp, calibración |
| Configuración | archivos YAML/env/CLI; valores críticos y defaults |
| Control de hardware | puerto real, comandos, permisos, feedback, timeout, watchdog |
| Seguridad/degradación | qué ocurre si Jetson, cámara o actuador está apagado |
| Pruebas | estática, simulada, NUC/Jetson/Ethernet verificadas, periféricos bloqueados/real, evidencia |

Rastrear casos de uso enteros como: `voz → transcripción → interpretación → máquina de estados → navegación por action` (Nav2 solo lectura); y `RGB-D → detección → TF → plan de agarre → CAN/Piper`, indicando exactamente dónde faltan datos o pruebas. Evitar publicar en ROS_DOMAIN_ID=10 durante tests.

## 5. Inspección no destructiva permitida de NUC

Ejecutar comandos **solo si existe acceso aprobado al host**. Son ejemplos de consulta; confirmar entorno y cargas antes:

```bash
pwd
git status --short --branch
git rev-parse HEAD
cat /etc/os-release
uname -a
lscpu
free -h
df -h
ip -brief addr
lsusb
ls -l /dev/serial/by-id 2>/dev/null || true
printenv ROS_DISTRO ROS_DOMAIN_ID RMW_IMPLEMENTATION
ros2 pkg list
ros2 node list
ros2 topic list -t
ros2 service list
ros2 action list
```

No lanzar Nav2 ni `ros2 launch` de actuadores, `colcon test` que abra dispositivos, `ros2 topic pub`, `ros2 service call`, `ros2 action send_goal`, scripts que escriban a serial/CAN. Se permite SSH/ping e introspección de red y software a Jetson encendida; `sudo` para actualizaciones de software controladas según `docs/MIGRACION_JETSON.md` (NO flasheo ni borrar datos sin gate humano). Leer logs existentes sin reiniciar servicios. Un `ros2 node list` vacío con dispositivos apagados no demuestra error.

## 6. Auditoría de calidad y rendimiento

Evaluar sin modificar navegación:

1. **Estructura:** paquetes y ownership; archivos huérfanos, duplicados, extensos, código importado incorrectamente, tests ausentes; dependencia de `~/` y rutas absolutas.
2. **Lógica:** separación drivers/algoritmos/HRI; estados y transiciones; falta de timeout/fallback; uso de mocks y `dry_run` por defecto.
3. **ROS 2:** QoS/TF/tiempo, lifecycle, namespaces, creación duplicada de drivers, publishers `joint_states` de comandos vs medición, nombres de interfaces, API documentada.
4. **Python/C++:** bloqueo de callbacks, hilos, recursos, cierres, memoria, uso de CPU/GPU, inferencia por frame, cuellos de botella, excepciones silenciadas, logging.
5. **Mantenibilidad:** estructura estándar, parámetros YAML, instalación reproducible, launch arguments, versiones fijadas, style/typing/lint/CI y documentación real.
6. **Seguridad:** permisos de hardware, validación de límites, arming, watchdog, parada segura, qué hace cada nodo cuando falta un sensor.

Para cada hallazgo indicar `ID`, archivo/línea, comportamiento esperado, evidencia, gravedad, propuesta, alcance, tests antes/después, riesgo de regresión, bloqueo por hardware y aceptación. No mezclar reorganización total con nuevas funcionalidades: cambios pequeños, revisables y con rollback.

### Prioridades de refactor

- **Primero** corregir errores demostrables, rutas rígidas, falta de manejo de ausencias, logging engañoso y posibilidad de duplicar drivers.
- **Después** extraer interfaces reutilizables y tests para percepción, ASR/TTS, HRI, brazo/elevador, calibración, seguridad y UI.
- **Solo después de medir** hacer optimizaciones de rendimiento, con comparación antes/después. La NUC no debe sufrir cargas extensas si ejecuta sistemas esenciales.
- **Nunca** modificar software/firmware/parámetros de navegación o base motriz sin nueva autorización explícita.

## 7. Entregables, revisiones y estado

1. `docs/hardware.md` — inventario, conectividad, ownership y estado físico.
2. `docs/power-domains.md` — diagrama eléctrico y tablas con incógnitas visibles.
3. `docs/architecture.md` — diagrama computacional y funcional, dependencias, secuencias de tareas.
4. `docs/ros-interfaces.md` — interfaces completas y mapa TF.
5. `docs/software-audit.md` — riesgos y backlog de refactor/optimización.
6. `docs/test-results.md` — comandos, salidas resumidas, fecha, entorno, etiquetas de prueba y bloqueos.

**La primera entrega de Orca debe ser un PR solo de documentación/auditoría**, más fixes triviales estrictamente seguros si pueden justificarse, antes de refactorizaciones importantes. Claude puede llevar arquitectura/refactor y Antigravity inventario visual/URDF y tests; evitar conflictos mediante worktrees. Marcar la tarea completa solo cuando la evidencia existente permita comprender el sistema, no cuando todos los dispositivos estén encendidos.

## 8. Preguntas pendientes que no deben adivinarse

- ¿Qué dispositivos siguen conectados/reciben energía por USB cuando únicamente está encendida la NUC?
- ¿Qué placas/controladores, BMS, fusibles y step-down están montados y cuál es su cableado real?
- ¿Qué versiones de firmware y adaptadores CAN/UART están efectivamente instalados?
- ¿Qué servicios se ejecutan automáticamente en Jetson y NUC al arrancar, y cuáles intentan controlar periféricos desenergizados?
- ¿Hay fotos, planos, archivos CAD, esquemas o logs externos al repositorio?
- ¿Cuáles son los límites mecánicos medidos del elevador y el mounting del Piper?

Registrar cada incertidumbre como tarea y no bloquear los avances estáticos por falta de respuesta inmediata.
