# Nota de vigencia v3

> **Edición integral v3 — 03/11/2026.** Esta documentación es una guía de trabajo, no prueba de ejecución en las máquinas. Los avances operativos se basan en los registros compartidos por el usuario (principalmente `HANDOFF.md`, 29–30/10/2026), deben cotejarse con el checkout y el estado de cada host. Política vigente: la navegación es **estudiable y auditable sin modificarla**; observaciones y propuestas se registran en `docs/collab/PROPUESTAS_NAVEGACION.md`. El estado de tareas en Orca/Git puede ser más reciente que estos documentos.

**Se permite exploración profunda de navegación y hoverboard, pero no modificar ni desplegar navegación.** La autoridad detallada está en `AGENTS.md`; los datos históricos pueden estar obsoletos.

---

# Registro de hechos reportados versus verificación actual

**Fecha del archivo:** 2026-11-03. Estos hechos vienen de HANDOFF/TASKS aportados por el usuario, fechados hasta 2026-10-30. **No se accedió a la NUC/Jetson ni al checkout privado para verificarlos en este trabajo.**

| Componente | Estado reportado 30/10 | Qué comprobar hoy | Estado verificado hoy |
|---|---|---|---|
| Jetson Orin Nano | JetPack 7.2.1, L4T R39.2.1, Ubuntu 24.04, ROS Jazzy | OS, `nv_tegra_release`/BSP, ROS distro, kernel, uptime | PENDIENTE |
| NUC | Ubuntu24/Jazzy y navegación funcional | OS, ROS, interfaces nav solo lectura; no iniciar/mover | PENDIENTE |
| Ethernet | NUC enp1s0 ↔ Jetson USB RTL8153 enx00e04c68090c, 1Gb | carrier, IP, SSH, ping, RMW, reloj y tópicos lectura | PENDIENTE |
| Piper | `auto_enable=False`, `feedback_ok` guard; hardware apagado en registro | código servicio, logs, parámetros y feedback; **sin torque** | PENDIENTE |
| Pan-tilt | torque por defecto apagado, Dynamixel sin 12V | estado log sin movimiento | PENDIENTE |
| Orbbec | `stream_orbbec.py` 20Hz después de fix | propietario de cámara, FPS y CPU, sin actuadores | PENDIENTE |
| TensorRT | YOLOv8n FP16 engine en Jetson | engine compatible, medición tasas con mismo input | PENDIENTE |
| Reconocimiento de voz | Whisper `base.en`, WER local reportado 6% | dataset/condiciones, WER replicable | PENDIENTE |
| URDF | HOME-04 reportó 45 tests | checkout actual, UI 3D, modelos y Xacro | PENDIENTE |
| GPSR | HOME-15a integrado, 34 tests; faltan planes compuestos | exact plan scoring + dataset | PENDIENTE |
| Simulación | Gazebo Harmonic, `base_athome_sim` en dominio 42 | tests aislados y no hardware | PENDIENTE |
| Receptionist/HRI | mejoras de estado y entrada pantalla | pruebas reales de voz y simulación | PENDIENTE |
| RealSense | servicio usuario aún no instalado según PERMISOS | proceso actual, potencial duplicación, dueño de dispositivo | PENDIENTE |
| Credenciales | token GitHub incrustado en origin reportado | no imprimir, verificar sanitizado y revocación | PENDIENTE |
| Sudo temporal | archivo de flasheo NUC y NOPASSWD Jetson reportados | inspección no reveladora y eliminación aprobada | PENDIENTE |

**Regla:** los informes anteriores **no** son certificados de estado actual. La verificación corresponde a Claude/Antigravity al abrir la próxima sesión, y los resultados se agregan al `HANDOFF.md` activo sin reemplazarlo con el archivo proporcionado aquí.
