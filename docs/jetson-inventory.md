# Inventario Jetson Orin Nano (HOME-10)

Fecha: 2026-10-29 ~23:40 (-03). Recolectado por Claude Code vía SSH desde la NUC, solo comandos de lectura.
Etiquetas: `JETSON_CONFIRMED` = salida de comando real; `SOLO_DOCUMENTADO`; `PENDIENTE`.

## Acceso

| Dato | Valor | Estado |
|---|---|---|
| Ruta de acceso usada | **WiFi** `192.168.0.203` (red `Limo_5G`), usuario `ubuntu`, clave SSH ya autorizada en la NUC | JETSON_CONFIRMED |
| Identidad del host | Huella ed25519 de `192.168.0.203` idéntica a la registrada en `known_hosts` para `10.42.0.12` | JETSON_CONFIRMED |
| Ruta Ethernet `10.42.0.12` | **No disponible**: `enP8p1s0` sin carrier (DOWN) en la Jetson y `enp1s0` sin carrier en la NUC | JETSON_CONFIRMED / NUC_CONFIRMED |
| `sudo` sin contraseña | Sí (`sudo -n true` OK) | JETSON_CONFIRMED |

## Sistema

| Dato | Valor |
|---|---|
| Modelo | NVIDIA Jetson Orin Nano Engineering Reference Developer Kit **Super** (`/proc/device-tree/model`) |
| JetPack | `nvidia-jetpack 6.2.3+b81` |
| L4T | R36 rev 5.2 (`/etc/nv_tegra_release`), kernel `5.15.199-tegra` |
| SO | Ubuntu 22.04.5 LTS, arm64 |
| Bootloader / QSPI | `nvbootctrl`: versión 36.5.2, slot activo B, ambos slots `normal` |
| Almacenamiento | **microSD** `mmcblk0` 116.6 GB (rootfs `mmcblk0p1` ext4 115 GB, 35 GB usados). **No hay NVMe.** |
| RAM / swap | 7.4 GiB / 11 GiB (zram + `/swapfile`) |
| Reloj | NTP activo y sincronizado, TZ America/Montevideo |
| CUDA / cuDNN / TensorRT | 12.6 / 9.3.0 / 10.3.0 |
| Python | 3.10.12; OpenCV 4.13.0; **PyTorch no instalado** para python3 del sistema |
| ROS 2 | Humble (`/opt/ros/humble`, 200 paquetes en share), `RMW=rmw_cyclonedds_cpp`, `ROS_DOMAIN_ID=10` (`~/.bashrc`) |
| Docker | no instalado |

## Periféricos USB detectados (`lsusb`)

RØDE VideoMic GO II · adaptador CAN gs_usb (OpenMoko 1d50:606f) · Orbbec Dabai DC1 + Orbbec Depth Sensor · FTDI FT232H (probable U2D2 de Dynamixel, `PENDIENTE` confirmar) · Realtek RTL8153 USB-Ethernet (`enx00e04c68090c`, DOWN).
Detectar un periférico por USB **no** implica que su alimentación de potencia esté activa.

`can0` y `can1` están **UP a 1 Mbit/s, ERROR-ACTIVE** (JETSON_CONFIRMED). No se inspeccionó tráfico ni se enviaron tramas.

## Software del robot en la Jetson

- Workspace `~/athome_ws` (**no es repo Git**) con paquetes `audio_asr`, `pan_tilt_controller`, `robot_bringup_jetson`.
- Scripts sueltos en `~`: `piper_arm_controller.py`, `stream_orbbec.py`, `stream_zed.py`, `start_head.sh`, `start_mapping_head.sh`, `test_u2d2.py`, `Hri_Calibration_System/`, `gs_usb/`.
- **Deriva respecto al repo** (`jetson_head/`): idéntico salvo
  - `launch/jetson_head.launch.py`: Jetson usa `model_size: 'tiny'`, repo `base.en` (el commit `b9c2e77` dice "upgrade whisper to tiny" pero el repo no lo refleja).
  - `start_mapping_head.sh`: Jetson pasa `enable_camera:=false`, repo no.
- Servicios propios: `jetson-head.service` (enabled, `Restart=always`, ejecuta `~/start_mapping_head.sh`) y `jetson-performance.service` (`jetson_clocks`). También `anydesk.service`.

### Estado de `jetson-head.service`

**2026-10-30 09:39:** activo, 0 reinicios, con enlace por `enx00e04c68090c`. Nodos arriba: `pan_tilt_node` (servos sin potencia), `whisper_node` (tiny, RØDE OK), `piper_arm_controller.py` (deshabilitado), `stream_orbbec.py` (20 Hz tras S-12). Carga: RAM 1.1/7.6 GB, ~6.7 W VDD_IN, 51 °C.

#### Histórico 2026-10-29: bucle de fallos

Se reinicia cada ~5 s. Causa (journal): `~/cyclonedds.xml` fija `NetworkInterface name="enP8p1s0"`; sin link Ethernet CycloneDDS falla con *"enP8p1s0: does not match an available interface"* → `rmw_create_node: failed to create domain` → mueren `pan_tilt_node`, `whisper_node`, `piper_arm_controller.py` y `stream_orbbec.py`. **Se resuelve restableciendo el enlace Ethernet**; no se modificó nada.

### Riesgo de seguridad asociado

`start_mapping_head.sh` lanza `piper_arm_controller.py` en cada arranque, y ese script tenía `auto_enable=True` (envía `EnableArm`, `MotionCtrl_2` y, en el bucle a 50 Hz, `JointCtrl` con el setpoint leído al arrancar). Al reconectar Ethernet el servicio arrancaría el controlador del brazo sin intervención humana. El fix (`auto_enable=False` por defecto) fue **desplegado** el 2026-10-30 00:20 y verificado en el log del 09:39 (`Brazo DESHABILITADO`). Ver `docs/software-audit.md` S-01.

## Respaldo realizado (Fase B, parcial)

Ubicación en la NUC: `/home/athome/jetson_backups/2026-10-29_pre_migration/` (fuera del repo), con `SHA256SUMS`.

| Archivo | Contenido |
|---|---|
| `jetson_home_project.tgz` | `athome_ws` (sin build/install/log), scripts de `~`, `cyclonedds.xml`, `Hri_Calibration_System`, `gs_usb` (174 entradas; sin `.ssh`, `.env` ni llaves; búsqueda de nombres sensibles vacía) |
| `jetson_etc_selected.tgz` | units `jetson-head`/`jetson-performance`, `udev/rules.d`, `netplan`, `modules-load.d`, apt sources, `nv_tegra_release` |
| `dpkg_selections.txt`, `dpkg_versions.tsv`, `apt_manual.txt`, `pip3_freeze.txt` | Manifiesto de paquetes |
| `system_info.txt` | cmdline, fstab, `lsblk -f`, `nvbootctrl` |

**No es un respaldo restaurable del sistema completo.** Falta: imagen completa de la microSD (requiere apagar y clonar la tarjeta en otra máquina, o `dd` hacia almacenamiento externo), datos de calibración que estén fuera de estas rutas, modelos Whisper descargados (`~/.cache`), y verificación humana de que nada sensible quedó fuera. Es suficiente para reconstruir el software del robot en un sistema nuevo.

## Migración a JetPack 7.2.1 / Ubuntu 24.04 / Jazzy (HOME-11)

Estado: **BLOCKED_APPROVAL**.

- **Verificado en la página oficial de NVIDIA (2026-10-30)** — [JetPack 7.2.1](https://developer.nvidia.com/embedded/jetpack/downloads/archive-7.2.1): Jetson Linux 39.2.1, **L4T Ubuntu 24.04, kernel 6.8, CUDA 13.2.2, cuDNN 9.20, TensorRT 10.16.2**. Soporta "Jetson Orin Family" (el Orin Nano no se nombra explícitamente). Para el Orin Nano Dev Kit **ya no hay imagen de microSD**: se instala con la **ISO unificada desde un pendrive USB**, que usa por defecto la configuración de flasheo *Super*. La página no documenta ruta de actualización desde JetPack 6 ni el procedimiento de QSPI → seguir la Getting Started Guide. Consecuencias: (1) destino NVMe o microSD nueva vía instalador ISO (necesita monitor+teclado o consola en la Jetson), (2) **CUDA 13** → ctranslate2 y cualquier wheel CUDA deben ser builds para CUDA 13, (3) los engines TensorRT 10.3 actuales **no sirven** en 10.16: reconstruir con `build_yolo_trt.sh`.
- No existe ruta de actualización in-place soportada de JetPack 6 (L4T 36, Ubuntu 22.04) a JetPack 7.x (Ubuntu 24.04); `do-release-upgrade` está prohibido. La migración implica **reflashear el medio de arranque (borra la microSD) y probablemente actualizar QSPI** (actual 36.5.2). Que JetPack 7.2.1 soporte Orin Nano Super está `SOLO_DOCUMENTADO` en las instrucciones; verificar en las release notes de NVIDIA antes de proceder.
- Prerrequisitos antes de pedir el gate de borrado:
  1. Restablecer Ethernet (o tener acceso físico: monitor/teclado o consola serie `ttyTCU0`) — hoy el único canal es WiFi.
  2. Imagen completa de la microSD verificada por checksum, o usar una **microSD/NVMe nueva** como destino y conservar la actual intacta (opción recomendada: rollback = volver a insertar la tarjeta vieja, salvo por QSPI).
  3. Comprobar compatibilidad arm64/Jazzy de: `piper_sdk` (python-can, gs_usb), Orbbec SDK / `OrbbecSDK_ROS2`, `dynamixel_sdk`, faster-whisper/CTranslate2 con CUDA del JetPack 7, `realsense2_camera`.
  4. Decisión humana explícita sobre: dispositivo destino, datos que se borran, método (ISO / SDK Manager) y ventana.
- No se aplicaron `apt upgrade` ni otros cambios a la Jetson en esta sesión: con el único canal por WiFi y una migración que reemplazará el sistema, el beneficio no justifica el riesgo de perder acceso.

## Matriz de compatibilidad para Jazzy / Ubuntu 24.04 (preparación HOME-11)

Fuente: `apt-mark showmanual` y `pip3 freeze` del respaldo (2026-10-29). Estado de soporte arm64/Jazzy a verificar en la ventana de migración.

| Componente | Versión actual (Humble/22.04) | Ruta en Jazzy/24.04 | Riesgo |
|---|---|---|---|
| ROS base | `ros-humble-ros-base` | `ros-jazzy-ros-base` (apt oficial arm64) | Bajo |
| DDS | `ros-humble-rmw-cyclonedds-cpp`, `cyclonedds` | `ros-jazzy-rmw-cyclonedds-cpp` | Bajo; reusar `cyclonedds.xml` |
| RealSense | `ros-humble-realsense2-camera` + `pyrealsense2 2.59.1` (pip) | `ros-jazzy-realsense2-camera` (apt) | Medio: librealsense con kernel L4T 39.x |
| Mensajes visión | `ros-humble-vision-msgs` | `ros-jazzy-vision-msgs` | Bajo |
| Piper | `piper_sdk 0.6.2`, `python-can 4.6.1` (pip) | igual vía pip en venv, o `agx_arm_ros` rama `ros2` (Jazzy "Pass") | Bajo-medio: `gs_usb` debe estar en el kernel nuevo (hoy hay `~/gs_usb` compilado a mano) |
| Dynamixel | `dynamixel-sdk 4.1.0` (pip) | `ros-jazzy-dynamixel-sdk` o pip | Bajo |
| ASR | `faster-whisper 1.2.1`, `ctranslate2 4.8.3` (pip) | ctranslate2 con CUDA del JetPack 7 → probablemente compilar desde fuente | **Alto**: sin wheel CUDA arm64 oficial |
| ONNX | `onnxruntime 1.23.2` (CPU) | `onnxruntime-gpu` de Jetson AI Lab | Medio |
| Orbbec | `stream_orbbec.py` propio (SDK Python) | `OrbbecSDK_ROS2` `v2-main` | Medio: confirmar DC1 |
| Audio | `sounddevice 0.5.6`, PulseAudio/PipeWire | igual (24.04 usa PipeWire) | Bajo |
| numpy | 2.2.6 (pip, sobre python 3.10) | python 3.12 del sistema: usar venv (PEP 668) | Medio: no `pip install` global |

Conclusión: el bloqueo real es **ctranslate2/faster-whisper con CUDA** y el **módulo `gs_usb`** en el kernel nuevo. Plan B para ASR si ctranslate2 no compila: whisper vía TensorRT-LLM / `whisper.cpp` con CUDA, o sherpa-onnx (usado por WrightEagle).
