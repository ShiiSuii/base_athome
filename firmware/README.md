# Hoverboard Driver Setup & Control Hub

Repositorio de configuración de hardware, firmware y herramientas de software para convertir placas base de hoverboard (STM32F103) en una base móvil diferencial controlada por computadora, lista para integración con **ROS 2**.

Soporta tanto placas comerciales genéricas como la placa roja **Lenzod** (STM32F103RCT6).

---

## 📋 Características Principales

- **Soporte Multi-Placa:** Configuraciones probadas y validadas para placas **Lenzod Red Board** y placas **Genéricas STM32**.
- **Control FOC Bidireccional:** Comunicación serial UART a 115200 baudios con protocolo binario robusto de 50 Hz.
- **Odometría Integrada (`ENABLE_ODOMETRY`):** Paquetes de telemetría de 22 bytes con contadores de ticks de sensores Hall para cada rueda (`wheelR_cnt`, `wheelL_cnt`), lectura de RPM, voltaje de batería en tiempo real y temperatura de la placa.
- **Suite de Herramientas Python:** Teleoperación interactiva por teclado con HUD en tiempo real, escáner de baudios, pruebas pasivas de sensores Hall y monitores de telemetría.
- **Binarios Precompilados:** Firmware listo para flashear sin necesidad de instalar la toolchain de compilación ARM.

---

## 🔌 Hardware y Conexiones

> [!IMPORTANT]
> Para el esquema detallado de pines, fotos y advertencias de seguridad de cada placa, consulta **[docs/PINOUTS_AND_WIRING.md](docs/PINOUTS_AND_WIRING.md)**.

### Conexión Serial (FTDI / USB-TTL)
La comunicación se realiza a través del conector del **cable del sensor Hall DERECHO (USART3)**:
- **GND** -> **GND**
- **TX Hoverboard (Verde)** -> **RX FTDI**
- **RX Hoverboard (Azul)** -> **TX FTDI**
- **15V / Rojo** -> ⛔ **DESCONECTADO** (Nunca conectar al conversor USB).

---

## ⚡ Flasheo del Firmware

### Opción 1: Flashear Binario Precompilado (Recomendado para Lenzod)
Si tienes una placa roja Lenzod con STM32F103RCT6:
```bash
st-flash write firmware_bin/hover_lenzod_usart3_odometry.bin 0x08000000
```
> [!CAUTION]
> Recuerda **NO conectar el pin de 3.3V del ST-Link** a la placa Lenzod si la batería de 36V está conectada. Conecta únicamente `GND`, `SWCLK` y `SWDIO`.

### Opción 2: Compilar desde el Código Fuente
En la carpeta `firmware_config/` se proporcionan presets listos para usar:
- `firmware_config/lenzod_red_stm32_usart3_odometry/config.h`: Placa Lenzod con UART Derecho (USART3) y odometría de 22 bytes.
- `firmware_config/generic_stm32_usart2/config.h`: Placa genérica con UART Izquierdo (USART2).
- `firmware_config/generic_stm32_usart3/config.h`: Placa genérica con UART Derecho (USART3).

Para compilar:
1. Copia el preset deseado a tu repositorio de firmware FOC:
   ```bash
   cp firmware_config/lenzod_red_stm32_usart3_odometry/config.h ruta/a/hoverboard-firmware-hack-FOC/Inc/config.h
   ```
2. Compila con `make`:
   ```bash
   make clean && make
   ```
3. Flashea el binario resultante:
   ```bash
   st-flash write build/hover.bin 0x08000000
   ```

---

## 🎮 Herramientas de Control y Teleoperación

Todas las utilidades se encuentran en la carpeta `scripts/` y utilizan por defecto el puerto `/dev/ttyUSB0` a `115200` baudios.

### 1. `teleop_hoverboard.py` (Control Interactivo con HUD)
Permite controlar el movimiento del robot en tiempo real desde el teclado con feedback en vivo de batería, temperatura, RPM y ticks de odometría:
```bash
python3 scripts/teleop_hoverboard.py
```
**Controles:**
- `W` / `↑`: Acelerar adelante (+50)
- `S` / `↓`: Retroceder (-50)
- `A` / `←`: Girar a la izquierda (-40)
- `D` / `→`: Girar a la derecha (+40)
- `ESPACIO`: Freno de emergencia (detener motores)
- `Q`: Salir de forma segura

### 2. `debug_motors.py` (Monitor Pasivo de Sensores Hall)
Herramienta pasiva de monitoreo de telemetría. No envía comandos de movimiento a los motores; permite girar las ruedas manualmente para verificar que los sensores Hall y los contadores de ticks respondan correctamente:
```bash
python3 scripts/debug_motors.py
```

### 3. `test_tx_rx_loop.py` (Diagnóstico de Comunicación)
Valida la bidireccionalidad del lazo serie verificando que la placa escuche los comandos de velocidad y responda con los valores correctos:
```bash
python3 scripts/test_tx_rx_loop.py
```

### 4. `hoverboard_terminal.py` (Consola de Comandos Directos)
Permite ingresar valores numéricos manuales de `Steer` y `Speed`:
```bash
python3 scripts/hoverboard_terminal.py
```

### 5. `baud_scanner.py` (Buscador de Baudrate)
Escanea diferentes velocidades de transmisión (9600, 19200, 38400, 57600, 115200) para detectar automáticamente a qué velocidad responde la placa:
```bash
python3 scripts/baud_scanner.py
```

---

## 🤖 Integración con ROS 2

En `scripts/hoverboard_ws/src/hoverboard_base` se encuentra el paquete ROS 2 con el nodo `hoverboard_driver`, el cual:
- Se suscribe al tópico `cmd_vel` (`geometry_msgs/msg/Twist`).
- Publica la odometría de las ruedas en el tópico `odom` (`nav_msgs/msg/Odometry`).
- Publica la transformada de coordenadas `odom -> base_link` mediante `tf2_ros`.

---

## 📁 Estructura del Repositorio

```text
Hoverboard-Driver-Setup/
├── docs/
│   ├── PINOUTS_AND_WIRING.md       # Esquemas de pines, ST-Link y FTDI
│   └── images/
│       └── lenzod_pinout.jpg       # Foto y diagrama de la placa Lenzod
├── firmware/                       # Sketches de prueba para Arduino
│   ├── HoverboardController/
│   ├── HoverboardTest/
│   ├── SimplifiedRamp/
│   └── UltimateTest/
├── firmware_bin/                   # Binarios precompilados listos para flashear
│   └── hover_lenzod_usart3_odometry.bin
├── firmware_config/                # Presets de configuración config.h
│   ├── config.h                    # Configuración activa actual
│   ├── generic_stm32_usart2/       # Preset placa genérica USART2 (izq)
│   ├── generic_stm32_usart3/       # Preset placa genérica USART3 (der)
│   └── lenzod_red_stm32_usart3_odometry/ # Preset Lenzod USART3 con odometría
├── scripts/                        # Scripts de control y diagnóstico Python
│   ├── baud_scanner.py
│   ├── debug_motors.py
│   ├── flash_firmware.sh
│   ├── hoverboard_teleop.py
│   ├── hoverboard_terminal.py
│   ├── hoverboard_ws/              # Espacio de trabajo ROS 2 (hoverboard_base)
│   ├── read_hoverboard_telemetry.py
│   ├── run_driver.sh
│   ├── teleop_hoverboard.py
│   ├── test_hoverboard_motors.py
│   ├── test_hoverboard_serial.py
│   ├── test_hoverboard_uart.py
│   └── test_tx_rx_loop.py
├── .gitignore                      # Reglas de exclusión de git
├── README.md                       # Documentación principal
└── test_motors.py                  # Prueba rápida de motores
```
