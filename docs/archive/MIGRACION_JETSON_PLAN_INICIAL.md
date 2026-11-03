# DOCUMENTO HISTÓRICO NO REFLASHEAR

> Plan inicial previo al handoff 30/10; Jetson fue reportada migrada a Jazzy. Consulte `../MIGRACION_JETSON.md`.

# Migración controlada Jetson Orin Nano → Ubuntu 24.04 + ROS 2 Jazzy

**Estado:** TAREA PRIORITARIA ACTIVA; la Jetson y la NUC están ENCENDIDAS y conectadas mediante un cable Ethernet directo. El usuario autoriza SSH, diagnóstico, mantenimiento, paquetes compatibles y preparación/ejecución de la migración de software en la Jetson, si es viable. El flasheo/UEFI/particionado y toda acción que BORRE almacenamiento requieren backup externo comprobado, comprobación del disco y confirmación específica del paso de borrado. Nunca enviar comandos de movimiento. Fuente NVIDIA: https://developer.nvidia.com/embedded/jetpack/downloads y guía Orin Nano https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/quick_start.html.

## 1. Meta, evidencia y no objetivos

- Meta: Jetson Orin Nano (modelo/portadora a verificar) sobre JetPack **7.2.1 / Jetson Linux 39.2.1 / Ubuntu 24.04**, con ROS 2 Jazzy; integración con NUC 24.04/Jazzy por red privada y CycloneDDS.
- Confirmado por **fuente pública NVIDIA**: JetPack 7.2.1 soporta familia Orin y Ubuntu 24.04; **NO está confirmado** aún que todas las bibliotecas actuales del equipo estén listas.
- README del repo documenta Jetson con Ubuntu 22.04 + ROS2 Humble, NO verificado actualmente; confirmar por SSH en la Jetson encendida.
- No modificar OS/ROS de NUC ni Nav2; preservar navegación.
- No utilizar `do-release-upgrade` directo como método. Elegir instalación oficialmente soportada según carrier, almacenamiento y bootloader.
- No inventar versiones de GPU, firmware, bootloader ni estado eléctrico.

## 2. Fase A — inventario y evaluación (AHORA, por SSH Ethernet)

| Pregunta | Evidencia requerida | Estado |
|---|---|---|
| ¿Orin Nano 8GB devkit o módulo con carrier de tercero? | etiqueta/foto/manual confirmados por humano | PENDIENTE |
| JetPack/L4T actual, Ubuntu/kernel | outputs obtenidos por SSH de la Jetson encendida | TODO_SSH |
| UEFI/QSPI y storage boot | particiones/boot logs/versión UEFI por SSH + firmware menú si hace falta | TODO_SSH |
| Capacidad/vida útil almacenamiento, backup externo | inventario físico y `lsblk` por SSH, respaldo externo | TODO_SSH |
| Piper CAN/Sockets, bitrate real y servicio | código/README vs inspección futura | REVISAR CÓDIGO |
| Audio RØDE/ALSA, micrófono y TTS/ASR | services/venvs, requirements | REVISAR CÓDIGO |
| Orbbec DaBai DC1 / USB / calibraciones | driver y cámara exacta, SDK soportado | REVISAR CÓDIGO |
| Dynamixel U2D2/IDs/baud y límites | driver y fabricante, no comandos | REVISAR CÓDIGO |
| Python, rclpy, OpenCV, PyTorch, CUDA/cuDNN/TensorRT | requirements y envs documentados | REVISAR CÓDIGO |
| RMW, ROS_DOMAIN_ID, IP, NTP/clock/TF | configs NUC/Jetson y red | REVISAR CÓDIGO |

Antes de sugerir repos o versiones, revisar compatibilidad **arm64**, paquetes ROS2 Jazzy, drivers y kernel NVIDIA. No recomendar `pip install` masivo sobre OS base. Planificar venvs/contenedores compatibles de ser necesarios.

## 3. Fase B — respaldo verificable (AHORA, con Jetson encendida)

Resguardar, entre otros: workspace y ramas Git; repos submódulos; `colcon` metadata; `package.xml`, rosdep y requirements; YAML; Xacro/URDF; TF extrínsecos; hand-eye e intrínsecos; `/etc/systemd/system/` relacionados; custom udev; CAN y serial settings; SSH config **sin copiar llaves privadas al repo**; usuarios/permisos; servicios Jetson; Python venvs listados; Dockerfile/imágenes por tag; TTS/STT local model assets; logs relevantes anonimizados.

**Los agentes NO extraen tokens, claves ni carpetas privadas.** El humano realiza respaldo sensible local y verifica que se puede restaurar. Mantener backup en dispositivo independiente, checksum/verificación, inventario, versión fuente, fecha e instrucciones para rollback. No sobrescribir el único almacenamiento de arranque sin validación.

## 4. Fase C — instalación compatible, con gate de borrado

1. Confirmar carrier y método oficial de instalación (Jetson ISO/SDK Manager, almacenamiento y versiones de UEFI/QSPI); revisar notas de errores conocidos para Orin Nano. No asumir que ISO se graba en la microSD como imagen tradicional.
2. Comprobar compatibilidad de accesorios y alimentaciones. NUC y base física aisladas del proceso; no energizar actuadores accidentalmente.
3. Registrar matriz de paquetes instalados, kernel/BSP, drivers, apt source y versiones JetPack.
4. Para actualizaciones de paquetes no destructivas, documentar estado anterior, dependencias y rollback, trabajando sin drivers físicos. Para flash/UEFI/particionado, solicitar aprobación humana específica: dispositivo de destino, datos a borrar, respaldo restaurable y plan de reversión.
5. Solo después de ese gate, un humano o agente autorizado explícitamente para ESA operación puede ejecutar el proceso, con acceso físico/recovery disponible. No iniciar borrados ni escrituras de firmware sin permiso específico.

## 5. Fase D — ROS 2 Jazzy y software robot

- Instalar únicamente ROS 2 Jazzy arm64 en Ubuntu 24.04 compatible, siguiendo https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html; sin ROS 1 ni bridges.
- `rosdep` y `colcon` en workspace Jetson separado, versiones Python/ABI de Jazzy y paquetes `rclpy`, `tf2_ros`, cv_bridge; evitar paths `/opt/ros/humble` en new launch/scripts.
- Reconfigurar CycloneDDS, ROS_DOMAIN_ID compartido si autorizado y red entre hosts; comprobar timestamps, TF/QoS solo cuando hardware autorizado.
- Recuperar componentes uno por vez con `SAFE_MODE`, sin torque por defecto, sin auto-home; no `ros2 launch` de driver físico sin autorización.
- Verificar USB/audio/cámara/CAN/U2D2 en modo lectura; después ensayos escalonados, aprobados, con parada disponible.

## 6. Matriz de validación (no llenar con éxito ficticio)

| Subsystem | Criterio de aceptación | Estado actual |
|---|---|---|
| Boot | JetPack 7.2.1 y Ubuntu 24.04 arrancan consistentemente | TODO / BLOCKED_APPROVAL si requiere flash |
| GPU | librerías NVIDIA compatibles con versión documentada | TODO_SSH |
| ROS 2 | Jazzy funciona con workspace reconstruido | TODO / dependiente de OS |
| DDS | tópicos de prueba inocuos por Ethernet, discovery/time/QoS | TODO_SSH |
| Orbbec | imágenes e intrínsecos coherentes | BLOCKED_HARDWARE (periférico no confirmado) |
| Piper CAN | feedback y seguridad sin movimiento inesperado | BLOCKED_HARDWARE |
| Dynamixel | feedback, límites y cero verificados | BLOCKED_HARDWARE |
| Audio | captura, VAD, ASR, TTS bajo latencia objetivo | TODO_SSH si dispositivo alimentado, de lo contrario bloqueado |
| Integración | HRI/percepción/TF/NAV interface sin regresiones en NUC | TODO_SOFTWARE, navegación sin cambios |

## 7. Rollback, go/no-go y documentación

**GO** solo si: backup verificado; carrier y método soportados; dependencias críticas compatibles o ruta de contención clara; respaldo de calibraciones; NUC estable; confirmación humana para operaciones de borrado; ventanas de ensayo; plan de regreso operativo.

**NO-GO** si: solo una copia del sistema; dudas de soporte de carrier/UEFI; riesgo a baterías/motores; falta fuente/alimentación estable; incompatibilidad crítica desconocida; usuario no aprobó el paso destructivo específico.

Documentar: manifest pre/post, cambios en packages, scripts de aprovisionamiento revisados, versiones, errores y recuperación, resultados con evidencias. No ocultar incompatibilidades.
