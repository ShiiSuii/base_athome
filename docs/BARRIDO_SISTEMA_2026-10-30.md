# Barrido general del sistema — 2026-10-30 13:50 (todo alimentado según el usuario)

Solo lectura (lsusb, journal, `ros2 topic hz/info`, contadores). No se publicó nada a actuadores.

## NUC (Ubuntu 24.04, Jazzy) — `NUC_CONFIRMED`
| Ítem | Estado |
|---|---|
| Navegación | **En marcha** (lanzada por el usuario): `nav2_bringup`, `robot_state_publisher`, `hoverboard_driver` (ttyUSB0, FTDI FT232R), `urg_node`, `joy`, `map_server`, `amcl`, planner… |
| `/scan` (Hokuyo 192.168.0.10, ping 0,16 ms) | 40 Hz |
| `/odom` | 50 Hz |
| `/hoverboard/battery` | 38,47 V, 70,6 % (1 Hz) |
| RealSense D435 | USB 3 (5 Gb/s), firmware 5.17.0.10; stream estable 15 Hz en prueba de 60 s |
| Elevador ESP32-S3 | CH343 `1a86:55d3` → `/dev/ttyACM0` (by-id `5CBD013609`), libre (nadie lo abre) |
| `/joint_states` | **3 publicadores**: `hri_receptionist_challenge_node` (poses comandadas), `pan_tilt_node`, `piper_arm_controller` → saltos en RViz (HOME-20 en curso) |

## Jetson (JetPack 7.2.1, Jazzy) — `JETSON_CONFIRMED`
| Ítem | Estado |
|---|---|
| `jetson-head` | activo; RAM 1,3/7,5 GB; 8,3 W; 52 °C; modo 25W |
| Micrófono RØDE | tarjeta ALSA 2, ganancia 100 %. Con ganancia máxima el ruido de sala da RMS ~0,1 → umbral de voz adaptativo (techo subido a 0,15) |
| Whisper `base.en` | ~300 % CPU mientras transcribe (CPU, sin CUDA) |
| **Piper (CAN `can1`, gs_usb)** | interfaz UP a 1 Mbit/s pero **0 tramas RX/TX**: el brazo no responde → revisar alimentación del Piper, E-stop y cableado CAN H/L. `can0` es el CAN integrado (mttcan, sin transceptor): ahora queda DOWN |
| **Pan-tilt (U2D2 FTDI `0403:6014`)** | **desconectado**: el kernel registró a las 13:14:16 errores USB `-71` y desconexión del FTDI; no volvió a enumerar. Típico de caída de tensión/ruido al energizar los servos o cable USB flojo → revisar cable/puerto del U2D2 y los 12 V de los Dynamixel |
| DaBai | 30 Hz |
| Antigravity | HOME-20 (joint_states/URDF en manipulación) trabajando |

## Acciones pendientes (humano)
1. Reconectar el U2D2 (y revisar 12 V de los servos). El nodo lo detecta solo al volver.
2. Revisar por qué el Piper no habla por CAN (alimentación/E-stop/cable).
3. Reiniciar `jetson-head` (`head_restart`) cuando termine la prueba en curso, para cargar el ajuste del micrófono.
4. Instalar el servicio de la cámara (comando en `scripts/systemd/realsense-nuc.service`).
