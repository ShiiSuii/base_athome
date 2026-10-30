# Hardware de Base@Home: capacidades, límites y optimizaciones para competir

2026-10-30. Etiquetas: `MEDIDO` (comando en vivo esta sesión), `VERIFICADO_CODIGO`, `FABRICANTE` (ficha pública; verificar con el modelo exacto), `INFERIDO`. Ninguna recomendación aquí modifica Nav2, ni energiza actuadores; los cambios de energía/potencia son decisiones humanas.

## 1. Cómputo

### NUC (CHUWI LarkBox S) — `MEDIDO`
- Intel Core i3-1220P (12 hilos, hasta 4,4 GHz), 15 GiB RAM (5,7 usados en reposo con escritorio), iGPU UHD (sin CUDA), NVMe 468 GB.
- **Implicación:** la NUC no tiene GPU útil para redes grandes. Todo lo que es inferencia pesada (YOLO grande, Whisper, segmentación) conviene en la Jetson; la NUC queda para Nav2/SLAM, lógica de tareas y percepción liviana (OpenVINO si se quiere acelerar en la iGPU — `INFERIDO`, no probado).
- Hoy la percepción/HRI corre en la NUC con `~/ros2_vision_venv` y los pesos `yolov8n/s` en la raíz del repo → candidatos a mover/duplicar en la Jetson con TensorRT.

### Jetson Orin Nano Developer Kit **Super** — `MEDIDO`
- 6 núcleos A78AE, 7,4 GiB RAM compartida CPU/GPU, GPU Ampere 4 TPC, CUDA 12.6, cuDNN 9.3, TensorRT 10.3.
- **Modo de energía actual: `25W` (ID 1, default).** CPU limitada a **1,344 GHz** (hardware hasta 1,728 GHz), GPU a **918 MHz** (hasta 1020 MHz). `jetson_clocks` (servicio `jetson-performance`) fija los máximos *del modo*, no del hardware. Disponible `MAXN_SUPER` (ID 2): ~+28 % CPU, ~+11 % GPU.
  - **Recomendación:** evaluar `MAXN_SUPER` solo si la fuente de 19 V de la Jetson en el robot soporta el consumo pico (MAXN no tiene tope de potencia) y la temperatura se mantiene (hoy ~50 °C en reposo, consumo ~6 W). Cambiarlo es `sudo nvpmodel -m 2` (+ reboot); es decisión humana.
- Reposo: RAM 739 MB, CPU ~2 %. Hay margen amplio.
- **Optimizaciones de software:**
  1. **TensorRT FP16/INT8** para YOLO (Ultralytics exporta `.engine`): típicamente varias veces más rápido que PyTorch en Orin (`INFERIDO`; medir). PyTorch ni siquiera está instalado en el python del sistema.
  2. **Whisper**: hoy `model_size: tiny` multilingüe vía faster-whisper/CTranslate2 (ver `audio_asr`). En Orin, `small` o `base.en` con CUDA FP16 da mucha mejor precisión con latencia aceptable; medir RTF (real-time factor) con grabaciones de prueba antes de cambiar. Para HRI 2026.2 basta inglés → modelos `.en` son más precisos a igual tamaño.
  3. Evitar copias de imagen CPU↔GPU: usar `cv_bridge` una vez y pipelines CUDA/NVMM donde se pueda.
  4. zram ya activo (6×634 MB) + swapfile; para modelos grandes preferir no depender de swap en microSD.
- **Almacenamiento:** arranca desde **microSD 116 GB** → lenta y propensa a corrupción por cortes de energía. **Recomendación fuerte:** en la migración a JetPack 7.2.1 instalar en un **NVMe M.2** (el devkit tiene ranura M.2 Key M) — mejora tiempos de carga de modelos y robustez; además permite conservar la microSD actual como rollback.

## 2. Sensores

### Hokuyo UST-05LX (NUC, Ethernet) — `FABRICANTE`
- 270°, alcance ~5 m (0,06–5 m), 25 ms/scan (40 Hz), resolución angular 0,25°. Interfaz Ethernet (`enx00e04c68060b` → 192.168.0.10, `MEDIDO` interfaz; sin carrier ahora).
- Alcance corto: en el Restaurant (lugar desconocido y grande) el SLAM online puede quedar limitado; complementar con la D435.
- **Señal de inicio por puerta:** el láser frontal es la forma más simple y confiable de detectar "puerta abierta" (§3.6.8): ventana angular frontal, distancia media > umbral durante N scans. Implementar como nodo aparte que solo lee `/scan` (no toca Nav2).

### Intel RealSense D435 (NUC) — `FABRICANTE`
- Estéreo activo, profundidad hasta 1280×720 @ 30–90 fps, FOV profundidad ~87°×58°, **distancia mínima ~0,28 m a resolución máxima** (menor a resoluciones bajas). RGB 1920×1080.
- Hoy: `realsense_nuc.launch.py`, perfil 640×480×15 en la Jetson (deshabilitado con `enable_camera:=false`).
- **Optimización:** 848×480 es la resolución óptima de profundidad del D435 (mejor relación precisión/ruido según Intel, `FABRICANTE`); activar filtros spatial/temporal/hole-filling solo donde se use la nube (cuestan CPU). Un solo dueño de la cámara (evitar dos nodos abriendo el mismo dispositivo — ya hubo "camera freeze", commit `70267ef`).

### Orbbec DaBai DC1 (Jetson, muñeca del Piper) — `FABRICANTE` (familia DaBai; ficha DC1 no encontrada)
- Luz estructurada estéreo, profundidad 640×400 @ 30 fps, **rango de trabajo 0,3–3 m**, precisión ±6 mm @ 1 m, FOV profundidad ~68°×45°; RGB 1920×1080. Fuente: listados de distribuidor, verificar con Orbbec.
- **Implicación crítica para eye-in-hand:** por debajo de ~30 cm no hay profundidad. Estrategia: (1) "pose de observación" a 35–45 cm sobre el objeto, (2) estimar pose 3D y agarre, (3) aproximación final en lazo abierto (o con la D435 de la cabeza como verificación), (4) confirmar agarre con el feedback de la pinza (`gripper_state`) — no con la cámara.
- Hoy se usa `stream_orbbec.py` propio que solo publica `/arm_camera/image_raw` (RGB). Para manipulación hace falta **profundidad registrada + intrínsecos** → migrar a `OrbbecSDK_ROS2` (`v2-main`) y verificar soporte DC1.
- La luz estructurada se degrada con sol directo (Restaurant con ventanas) — `INFERIDO`.

### RØDE VideoMic GO II (Jetson USB) — `MEDIDO` (USB) / `FABRICANTE`
- Micrófono shotgun supercardioide (direccional): excelente si apunta al hablante, malo para voces laterales. **Montarlo en la cabeza pan-tilt** hace que el "mirar a quien habla" del HRI (2×50) mejore también el ASR.
- Ganancia fijada por script (`amixer Mic 18`). Agregar VAD (Silero) antes de Whisper para no transcribir ruido; medir SNR en la arena.
- Hubo problemas de reconexión USB (commit `b9c2e77` "auto-reconnect USB mic").

## 3. Actuadores

### AgileX Piper 6-DOF + pinza (CAN 1 Mbit/s, Jetson `can0`) — `FABRICANTE` / `MEDIDO` (bus)
- Carga útil ~1,5 kg, alcance ~626 mm, repetibilidad ~0,1 mm, pinza paralela hasta ~70 mm de apertura (el código usa `GRIPPER_MAX_MM` y lee 69,4 mm por defecto). **Verificar** con el manual del modelo exacto.
- Objetos del reglamento: Pringles (diámetro ~75 mm — **en el límite de apertura de la pinza**; agarrar por el borde/tapa o de costado), Red Bull 250 ml (~53 mm, ok), cubo Rubik 57 mm (ok), caja Colgate (ok de canto). Bolsa con asas rígidas verticales (HRI) → gancho/pinza en el asa.
- Control actual: `piper_arm_controller.py` a 50 Hz con `JointCtrl` y trayectorias coseno; sin planificación de colisiones. **Recomendación:** migrar a `agx_arm_ros` (ros2, Jazzy "Pass") + **MoveIt 2** tras la migración de la Jetson, con el modelo de colisión del torso/columna del URDF; mantener `/arm/*` como fachada.
- `can0` y `can1` están ambos UP: confirmar qué hay en `can1` (¿segundo brazo, otro dispositivo?).
- Seguridad: con el fix S-01 ya desplegado, el brazo no se habilita solo. Antes de energizar: E-stop debe cortar la alimentación del brazo (verificación eléctrica).

### Pan-tilt 2× Dynamixel XL430-W250 (Jetson, U2D2 `/dev/ttyU2D2`, 1 Mbaud, protocolo 2.0) — `VERIFICADO_CODIGO` / `FABRICANTE`
- XL430: 12 V nominal, torque de bloqueo ~1,4 N·m @ 11,1 V, 4096 ticks/vuelta, modo posición extendida (el código configura modo 4 en el tilt y homing offset −2212).
- El nodo lee posición a 20 Hz (`publish_rate_hz`). Para **seguimiento de mirada continuo** (HRI: los invitados se mueven a propósito) conviene 50 Hz con lectura sincronizada (`GroupSyncRead`) y perfil de velocidad suave (hoy `PROFILE_VEL=80`, `ACCEL=20`).
- Con el fix S-03, el torque solo se habilita vía `/pan_tilt/set_torque` o `pan_tilt_auto_torque:=true`. **El flujo HRI debe llamar al servicio al iniciar la prueba.**

### Base hoverboard (NUC, firmware `HoverboardController`) — **SOLO LECTURA**
- No se modifica. Para el HRI "mirar en la dirección de navegación" (2×15) se puede orientar la cabeza según el plan/velocidad que publica Nav2 (solo suscribirse).

### Elevador NEMA23 + ESP32-S3 — `VERIFICADO_CODIGO`
- Controlado por `scripts/stepper_controller.py` desde la NUC; firmware en `firmware/nema23_esp32`. Útil para Pick and Place (alcanzar alacena con estantes a ≥30 cm y lavavajillas bajo) y Laundry (lavarropas de carga frontal, bajo). Auditar límites/homing antes de cualquier uso.

## 4. Red y DDS
- El enlace NUC↔Jetson debe ser **cableado** (el reglamento prohíbe redes propias y la WiFi de la arena no es confiable). RTT medido por WiFi doméstica 1,7–355 ms (muy variable) vs. esperado <1 ms por cable.
- Relojes: desfase NUC↔Jetson **−0,2 ms** (`MEDIDO` con `scripts/net/clock_offset.py`), ambos NTP sincronizados. En la competencia sin Internet, configurar a la NUC como servidor de hora para la Jetson (chrony) — `INFERIDO`, decisión humana.
- Imágenes por DDS entre hosts: usar `image_transport` comprimido o procesar en el host dueño de la cámara; no enviar RGB-D crudo por la red.

## 5. Prioridades de optimización por puntos (resumen)

| # | Acción | Prueba(s) | Esfuerzo | Riesgo |
|---|---|---|---|---|
| 1 | Detección de puerta abierta con Hokuyo + start button | Todas menos Restaurant/HRI | Bajo | Bajo |
| 2 | HRI sin preguntas de confirmación + mirada continua (pan-tilt 50 Hz) + seguir al anfitrión | HRI | Medio | Bajo |
| 3 | YOLO TensorRT en Jetson para los "common objects" del reglamento | Pick&Place, Final | Medio | Bajo |
| 4 | Orbbec con profundidad (OrbbecSDK_ROS2) + estrategia de pose de observación | Pick&Place | Medio | Medio |
| 5 | MoveIt 2 + agx_arm_ros tras migrar la Jetson | Pick&Place, HRI handover, Laundry | Alto | Medio |
| 6 | Parser GPSR (frida-cortex MIT / reglas) + dataset CommandGenerator | GPSR, Final | Medio | Bajo |
| 7 | NVMe + JetPack 7.2.1 + Jazzy | Todo (confiabilidad) | Alto | Alto (gate humano) |
| 8 | MAXN_SUPER si la fuente lo permite | Inferencia | Bajo | Medio (eléctrico) |
