# Hardware (HOME-01, primera pasada)

2026-10-29. `NUC_CONFIRMED`/`JETSON_CONFIRMED` = visto por comando en vivo; `VERIFICADO_CODIGO` = parámetro en código; `SOLO_DOCUMENTADO` = README/instrucciones.

| Componente | Host / interfaz | Dato | Estado |
|---|---|---|---|
| NUC CHUWI LarkBox S | — | Ubuntu 24.04.3, BIOS EM_IDL15_S16_V0.41.6, NVMe 468 GB | NUC_CONFIRMED |
| Jetson Orin Nano Dev Kit Super | — | JetPack 6.2.3, microSD 116 GB, 7.4 GiB RAM | JETSON_CONFIRMED |
| Hokuyo UST-05LX | NUC, USB-Ethernet `enx00e04c68060b` 192.168.0.14/29 → lidar 192.168.0.10 | interfaz sin carrier ahora | NUC_CONFIRMED (interfaz) / SOLO_DOCUMENTADO (IP lidar) |
| RealSense D435 | NUC (`realsense_nuc.launch.py`) | no detectado en esta sesión | PENDIENTE |
| Base hoverboard | NUC serial → `hoverboard_driver` | firmware `firmware/firmware/HoverboardController` | VERIFICADO_CODIGO, no inspeccionado en vivo |
| Elevador NEMA23 + ESP32-S3 | NUC, `scripts/stepper_controller.py`, `firmware/nema23_esp32` | — | VERIFICADO_CODIGO |
| Piper 6-DOF + pinza | Jetson, CAN gs_usb (1d50:606f), `can0` 1 Mbit/s UP | `can_port=can0` | JETSON_CONFIRMED (adaptador/bus) · actuador desenergizado |
| `can1` | Jetson | UP 1 Mbit/s, uso desconocido | JETSON_CONFIRMED / PENDIENTE uso |
| Orbbec Dabai DC1 (muñeca) | Jetson USB 2bc5:0557 + 2bc5:0657 | — | JETSON_CONFIRMED (USB) |
| Pan-tilt 2× Dynamixel XL430 | Jetson, `/dev/ttyU2D2` 1 Mbaud, IDs 1/2, protocolo 2.0 | FTDI FT232H presente (probable U2D2) | VERIFICADO_CODIGO / PENDIENTE confirmar udev |
| RØDE VideoMic GO II | Jetson USB 19f7:001c | ganancia `amixer Mic 18` | JETSON_CONFIRMED (USB) |
| Pantalla facial | NUC (`animated_face_node.py`) | — | VERIFICADO_CODIGO |

Ver `power-domains.md` para alimentación (casi todo pendiente).
