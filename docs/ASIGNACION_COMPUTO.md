# Asignación de cómputo NUC ↔ Jetson (2026-10-30)

Criterios, en orden:
1. **El nodo corre en el host donde está conectado el periférico.** Así no viajan por la red ni datos crudos ni comandos de bajo nivel.
2. **La inferencia pesada va a la GPU de la Jetson**, siempre que la entrada ya esté en la Jetson o sea liviana.
3. **La NUC queda libre para navegación y la lógica de las pruebas.** Nav2, SLAM y AMCL no se tocan (solo lectura).

## Capacidades medidas

| | NUC (CHUWI LarkBox S) | Jetson Orin Nano Super |
|---|---|---|
| CPU | i3-1220P, 10 núcleos / 12 hilos | 6× A78AE a 1,34 GHz (modo 25W) |
| GPU | Intel UHD integrada (sin CUDA) | Ampere, CUDA 13, TensorRT 10.16 (JetPack 7.2.1) |
| RAM | 15 GiB | 7,4 GiB compartida CPU/GPU |
| YOLOv8n | 38 ms en CPU | **4,3 ms en GPU** (TRT FP16, JetPack 6; rehacer en JP7) |
| Whisper | — | CPU int8: `base.en` 1,5 s/frase |
| SO / ROS | Ubuntu 24.04 / Jazzy | Ubuntu 24.04 / **Jazzy** (desde 2026-10-30) |

## Periféricos por host (`CODE_VERIFIED` / `JETSON_CONFIRMED`)

| Periférico | Conectado a | Nodo |
|---|---|---|
| Base hoverboard (UART) | NUC | `hoverboard_driver` (**solo lectura**) |
| Hokuyo UST-05LX (Ethernet `enx…060b`) | NUC | `urg_node` |
| RealSense D435 (USB 3) | NUC | `realsense2_camera` |
| Elevador ESP32-S3 (USB serie) | NUC | controlador del elevador |
| Pantalla / altavoz / TTS | NUC | cara animada, TTS |
| Piper (CAN `gs_usb`) | Jetson | `piper_arm_controller.py` |
| Pan-tilt XL430 (U2D2) | Jetson | `pan_tilt_node` |
| Orbbec DaBai DC1 (muñeca) | Jetson | `stream_orbbec.py` |
| RØDE VideoMic GO II | Jetson | `whisper_node` |

## Decisión

| Función | Host | Motivo |
|---|---|---|
| Nav2, SLAM, AMCL, `/scan`, odometría | **NUC** | sensores y base en la NUC; protegido |
| RealSense + HRI (detección y seguimiento de personas, caras, receptionist, máquina de estados de las pruebas) | **NUC** | la cámara está en la NUC; la lógica necesita Nav2 y la UI |
| TTS, pantalla, GUI | **NUC** | hardware en la NUC |
| Elevador | **NUC** | USB en la NUC |
| Parser GPSR (`tools/gpsr_parser`) | **NUC** | liviano, sin GPU; lo usa la máquina de estados |
| Editor URDF | **NUC** | herramienta local (127.0.0.1:8765) |
| Brazo Piper (control 50 Hz) | **Jetson** | CAN en la Jetson; latencia de lazo |
| Pan-tilt | **Jetson** | U2D2 en la Jetson |
| Cámara de muñeca + **YOLO TensorRT** → `/arm_camera/detections` | **Jetson** | cámara y GPU en el mismo host: solo viajan las cajas, no la imagen |
| ASR Whisper (`base.en`, inglés) | **Jetson** | micrófono en la Jetson; libera la CPU de la NUC |
| Futuro: YOLO de la cabeza en GPU | Jetson (opcional) | requiere mandar la imagen de la D435 por la red (~14 MB/s); hoy la NUC lo resuelve en CPU a 38 ms |
| Futuro: MoveIt (`agx_arm_ros`) | Jetson | junto al driver del brazo; ahora que ambos hosts están en Jazzy, la planificación también podría correr en la NUC |

## Implementado

- `jetson_head/scripts/start_mapping_head.sh`:
  - Arranca Piper (deshabilitado por defecto), pan-tilt (sin torque), DaBai, `yolo_trt_node.py` y Whisper `base.en`/`en`.
  - Overrides por entorno: `WHISPER_MODEL`, `ASR_LANGUAGE`, `ENABLE_YOLO`, `YOLO_ENGINE`.
  - Para demos en español: `WHISPER_MODEL=tiny ASR_LANGUAGE=auto`.
- Con ambos hosts en Jazzy desaparece la incompatibilidad de descubrimiento S-13.
