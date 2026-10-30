# Runbook HOME-11: Jetson Orin Nano → JetPack 7.2.1 (Ubuntu 24.04) + ROS 2 Jazzy

Preparado 2026-10-30 por Claude Code. **El paso de instalación borra el disco destino y actualiza el firmware QSPI: requiere a un humano frente a la Jetson y su confirmación explícita del dispositivo destino.** Claude no puede hacerlo por SSH (el instalador corre desde un pendrive, con monitor y teclado).

## 0. Ya preparado (hecho)

| Ítem | Dónde | Estado |
|---|---|---|
| ISO oficial r39.2.1 (5 035 601 920 bytes, igual al `content-length` del servidor de NVIDIA; NVIDIA no publica sha256 del ISO) | NUC `~/jetson_backups/jetpack_7.2.1/` (sha256 local `f31cf350…95d0`) | descargado |
| Respaldo técnico (proyecto, /etc seleccionado, manifiestos apt/pip) | NUC `~/jetson_backups/2026-10-29_pre_migration/` | hecho |
| Respaldo extra: modelos Whisper (680 MB) + YOLO, `gs_usb` compilado, proyecto actual, config udev/systemd/CAN | NUC `~/jetson_backups/2026-10-30_pre_migration_extra/` (`SHA256SUMS`) | hecho |
| Script post-instalación (Jazzy, venv, udev, CAN, red, colcon; no habilita servicios) | `jetson_head/scripts/bootstrap_jazzy.sh` (dry-run por defecto) | listo, dry-run probado en la NUC |
| `start_mapping_head.sh` detecta Jazzy/Humble y usa `~/robot_venv` | repo | listo (no desplegado) |
| Firmware actual | `nvbootctrl`: **36.5.2** → cumple el requisito (JetPack 6.x / 36.x) | verificado |

**No** es una imagen completa de la microSD: lo que no esté en esas rutas se pierde si se instala sobre la microSD actual.

## 1. Decisiones humanas (antes de empezar)

1. **Destino** (se borra entero):
   - **Recomendado: NVMe M.2 nuevo** (ranura Key M del devkit). Es más rápido y aguanta mejor los cortes de energía. La microSD actual queda intacta.
   - Alternativa: microSD nueva de 64 GB o más (UHS-1).
   - No recomendado: instalar sobre la microSD actual.
2. **El rollback no es gratis**:
   - El instalador **actualiza el QSPI a r39**. Con ese firmware, la microSD vieja (JetPack 6) probablemente **no arranque**: en Jetson, el firmware y el sistema deben ser de la misma versión.
   - Volver atrás implica reflashear con SDK Manager desde un PC Ubuntu, en modo recovery (puentear pines 9 y 10 del header J14; aparece como `0955:7523 NVIDIA Corp. APX`).
   - Este punto es `INFERIDO`: hay que verificarlo en la guía de NVIDIA antes de contar con el rollback.
3. **Ventana**:
   - Tiempo: unas 1–2 h para instalar, más el tiempo del bootstrap.
   - Material: monitor DisplayPort, teclado y mouse USB, y un pendrive de 16 GB o más (se borra).
   - Actuadores **apagados** durante todo el proceso.

## 2. Pendrive (en la NUC, humano)

Escribir el ISO con Balena Etcher, o con `dd` **después de identificar el pendrive**. `dd` sobre el disco equivocado borra la NUC:

```bash
lsblk -o NAME,SIZE,MODEL,TRAN     # identificar el pendrive (TRAN=usb), p.ej. /dev/sdX
sudo dd if=~/jetson_backups/jetpack_7.2.1/jetsoninstaller-r39.2.1-2026-08-07-18-30-47-arm64.iso of=/dev/sdX bs=4M status=progress conv=fsync
```

## 3. Instalación (humano, frente a la Jetson)

Según la [Quick Start Guide de NVIDIA](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/latest/quick_start.html):

1. Colocar el disco destino. Si es NVMe y se quiere conservar la microSD, **retirar la microSD** para no elegirla por error.
2. Conectar el pendrive y encender. Al ver el logo de NVIDIA, pulsar `Esc` → **Boot Manager** → elegir el pendrive.
3. Cuando aparezca el aviso de actualización de QSPI, **pulsar `Y` dentro de 30 s**. Si no se pulsa a tiempo, la instalación falla más adelante. La actualización hace 2 pasadas y puede reiniciar.
4. En GRUB elegir *Jetson ISO r39.2.1 install*, luego el destino, y confirmar.
5. Primer arranque: aceptar el EULA, elegir idioma y zona horaria (`America/Montevideo`), y crear el usuario **`ubuntu`**. Las rutas de los scripts lo asumen.
6. Problema conocido (foro de NVIDIA): algunos usuarios ven la pantalla negra al elegir el NVMe. Plan B: SDK Manager desde un PC con Ubuntu.

## 4. Volver a conectar con la NUC

En la Jetson nueva:
1. Conectar el adaptador USB-Ethernet. El puerto integrado no da link.
2. Fijar la IP:
   ```bash
   sudo nmcli con add type ethernet ifname enx00e04c68090c con-name robot-nuc ipv4.method manual ipv4.addresses 10.42.0.12/24 ipv4.gateway 10.42.0.1
   ```
3. Activar SSH: `sudo apt install openssh-server`.
4. Desde la NUC: `ssh-copy-id ubuntu@10.42.0.12`. Después avisarle a Claude: el resto es por SSH.

## 5. Software (Claude, por SSH)

```bash
# en la NUC
ssh-keygen -R 10.42.0.12      # la host key cambia con la reinstalacion
JETSON=ubuntu@10.42.0.12 jetson_head/scripts/deploy_to_jetson.sh --apply     # falla el colcon (sin ROS): normal
scp jetson_head/config/cyclonedds_jetson.xml ubuntu@10.42.0.12:cyclonedds.xml
scp ~/jetson_backups/2026-10-30_pre_migration_extra/jetson_models_cache.tgz ubuntu@10.42.0.12:
# en la Jetson
tar xzf jetson_models_cache.tgz -C ~
~/bootstrap_jazzy.sh            # revisar el dry-run
~/bootstrap_jazzy.sh --apply
```

Riesgos conocidos (ver `docs/jetson-inventory.md`, matriz de compatibilidad):
- **`gs_usb`**: si el kernel 6.8 no lo trae, compilarlo desde `~/gs_usb`.
- **Whisper**: sigue en CPU.
- **TensorRT**: los engines hay que reconstruirlos con `build_yolo_trt.sh`.
- **`cuda-python`**: instalar la versión que corresponde a CUDA 13.
- **Humble ↔ Jazzy**: el problema de descubrimiento entre la NUC y la Jetson desaparece cuando ambas quedan en Jazzy.

## 6. Validación (sin actuadores) y go/no-go

| Prueba | Comando | Criterio |
|---|---|---|
| Enlace | `scripts/net/check_nuc_jetson_link.sh` | 8 OK |
| DDS | `ros2 node list` desde la NUC | ahora debe listar los nodos de la Jetson (S-13 resuelto) |
| Cámara de muñeca | `ros2 topic hz /arm_camera/image_raw` | ≈20 Hz |
| ASR | log de `whisper_node` | modelo cargado, micrófono RØDE encontrado |
| YOLO | `yolo_trt_node.py` con engine reconstruido | ≥20 Hz |
| Piper | log | `Brazo DESHABILITADO`; `/arm/status` con `feedback_ok=false` sin alimentación |
| Pan-tilt | `/pan_tilt/connected` | `false` sin 12 V |

Solo después de esto: `sudo systemctl enable --now jetson-head` (decisión humana) y el checklist de energización por etapas.
