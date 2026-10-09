# 🤖 Base@Home (base_athome)

Repositorio oficial y stack de software completo para la plataforma móvil diferencial **Base@Home**, basada en hoverboard hackeado con firmware STM32, LiDAR Hokuyo Ethernet (UST-05LX) y ROS 2 Jazzy.

---

## 📋 Tabla de Contenidos
- [Arquitectura del Sistema](#-arquitectura-del-sistema)
- [Estructura del Repositorio](#-estructura-del-repositorio)
- [Especificaciones y Calibración](#-especificaciones-y-calibración)
- [Instalación y Compilación](#-instalación-y-compilación)
- [Comandos y Alias de Ejecución](#-comandos-y-alias-de-ejecución)
- [Firmware del Hoverboard](#-firmware-del-hoverboard)
- [Hardware y Conectividad](#-hardware-y-conectividad)

---

## 📐 Arquitectura del Sistema

El robot utiliza una arquitectura diferencial con ROS 2 Jazzy:

```
[ Dual Hub Motors + Encoders ]
              ▲
              │ UART (115200 baud, /dev/ttyUSB0)
              ▼
   [ hoverboard_driver ] (hoverboard_base)
              │
              ├──► /odom (nav_msgs/Odometry)
              ├──► /tf (odom -> base_footprint -> base_link -> laser)
              └──◄ /cmd_vel (geometry_msgs/Twist)
                     ▲
[ Hokuyo UST-05LX ]  │
         │           │
         │ Ethernet  │
         ▼           │
   [ urg_node ]      │
         │           │
         ▼           │
       /scan         │
         │           │
         ├───────────┼──────────────────────────────┐
         ▼           ▼                              ▼
 [ slam_toolbox ] [ Nav2 Stack ]              [ teleop_joy ]
  (Mapeo Online)  (AMCL + SmacPlanner2D +       (Control PS4)
                   RegulatedPurePursuit)
```

---

## 📁 Estructura del Repositorio

```
base_athome_ws/
├── firmware/                       # Firmware STM32 de la controladora hoverboard
│   ├── firmware/                   # Código fuente (STM32F103)
│   ├── firmware_bin/               # Binarios compilados (.bin / .hex)
│   ├── firmware_config/            # Archivos de configuración (config.h)
│   ├── scripts/                    # Scripts de flasheo (ST-Link / OpenOCD)
│   └── docs/                       # Guías de cableado y pines
├── src/
│   ├── hoverboard_base/            # Driver serie C++, odometría, URDF y teleop joy
│   ├── urg_node/                   # Driver Ethernet para LiDAR Hokuyo UST-05LX
│   ├── base_athome_bringup/        # Launchers de inicio de sensores y base
│   ├── base_athome_mapping/        # Configuración y launcher de SLAM Toolbox
│   └── base_athome_navigation/     # Nav2 bringup, costmaps, planner y controller
├── .gitignore
└── README.md
```

---

## ⚙️ Especificaciones y Calibración

### Dimensiones y Odometría Calibrada
- **Ancho de vía (Track Width):** `0.5620 m`
- **Radio de rueda (Wheel Radius):** `0.0853 m`
- **Compensación de fricción estática (Breakaway Torque):**
  - Linear mínima: `38` unidades (~`0.19 m/s`)
  - Angular mínima: `45` unidades (~`0.35 rad/s`)
- **Frecuencia de odometría:** `50 Hz`

### LiDAR Hokuyo UST-05LX
- **Interfaz:** Ethernet (RJ45 / Adaptador USB-Ethernet)
- **IP del sensor:** `192.168.0.10`
- **Puerto:** `10940`
- **Rango de escaneo:** `-135°` a `+135°` (270° de apertura), alcance de 5.5 m.

---

## 🚀 Instalación y Compilación

1. Clonar el repositorio en el espacio de trabajo:
```bash
git clone <URL_DEL_REPOSITORIO> ~/base_athome_ws
cd ~/base_athome_ws
```

2. Instalar dependencias de ROS 2:
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

3. Compilar el espacio de trabajo:
```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🎮 Comandos y Alias de Ejecución

Para facilitar el control diario, se encuentran configurados los siguientes comandos en la terminal:

| Comando / Alias | Descripción |
| :--- | :--- |
| `odometria` | Inicia el driver del hoverboard, odometría, TF y URDF robot state publisher |
| `lidar` | Inicia el driver Ethernet del LiDAR Hokuyo UST-05LX |
| `control` | Inicia el nodo de control manual vía joystick PS4 |
| `slam` / `mapanuevo` | Lanza SLAM Toolbox con RViz2 para generar un mapa del entorno |
| `guardarmapa <nombre>` | Guarda el mapa generado en formato `.yaml` y `.pgm` |
| `navegacion` | Inicia la navegación autónoma Nav2 (`AMCL`, `SmacPlanner`, `RegulatedPurePursuit`) |
| `rviznav` | Abre la interfaz gráfica RViz2 configurada para Nav2 |

### Ejecución manual vía `ros2 launch`:

```bash
# 1. Base y Sensores (Odometría + LiDAR + URDF)
ros2 launch base_athome_bringup base.launch.py

# 2. Mapeo SLAM
ros2 launch base_athome_mapping slam_toolbox.launch.py

# 3. Navegación Autónoma
ros2 launch base_athome_navigation nav2_bringup.launch.py map:=/ruta/al/mapa.yaml
```

---

## ⚡ Firmware del Hoverboard

La carpeta `firmware/` contiene las herramientas para la placa controladora STM32 del hoverboard:
- Protocolo serie bidireccional a 115200 baudios.
- Lectura de sensores Hall en tiempo real para odometría de rueda cerrada.
- Control de torque y velocidad suavizado por firmware.

Para flashear el firmware usando ST-Link:
```bash
cd ~/base_athome_ws/firmware/scripts
./flash_stlink.sh
```

---

## 🔌 Hardware y Conectividad

- **Placa Controladora:** Hoverboard Split Board / Main Board (STM32F103RCT6).
- **Convertidor USB a Serial:** FTDI / CP2102 conectado a `/dev/ttyUSB0` (USART3: TX/RX).
- **LiDAR:** Hokuyo UST-05LX conectado por adaptador Ethernet USB (`enx...`).
- **Energía:** Batería Li-ion 36V 10S conectada a reguladores Step-Down para PC/LiDAR (12V y 5V).
