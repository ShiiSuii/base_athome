# Modelo Cinemático Modular URDF - Base@Home RoboCup

Este directorio contiene el modelo URDF modular y limpio para el robot de servicio **Base@Home** (RoboCup @Home OPL).

## Estructura Modular

```
urdf/
├── robot.urdf.xacro        # Archivo maestro principal (ensambla los módulos)
├── robot.urdf              # URDF compilado estático (usado por launch files)
├── build_urdf.sh           # Script para compilar con xacro y validar con check_urdf
├── xacro/                  # Módulos cinematográficos independientes
│   ├── properties.xacro    # Dimensiones, offsets y parámetros físicos
│   ├── materials.xacro     # Materiales y colores RGBA
│   ├── base.xacro          # Chasis, ruedas hoverboard, rueda loca y LiDAR Hokuyo
│   ├── torso.xacro         # Columna elevadora NEMA23, carro prismático y pantalla HRI
│   ├── head.xacro          # Cabezal Pan-Tilt Robotis Dynamixel y RealSense D435
│   └── arm_piper.xacro     # Brazo AgileX Piper 6-DOF + cámara DaBai en muñeca
└── legacy/
    └── robot_base_only.urdf.xacro # Modelo anterior histórico sin brazo
```

## Compilación y Verificación

Para recompilar `robot.urdf` tras realizar cambios en los archivos `.xacro`:
```bash
./src/hoverboard_base/urdf/build_urdf.sh
```

## Frames Clave de Percepción y Navegación

| Frame | Descripción | Tipo |
| :--- | :--- | :--- |
| `base_footprint` | Proyección en el suelo del centro de rotación | Raíz Cinemática |
| `base_link` | Chasis móvil principal | Enlace físico |
| `laser` | Sensor LiDAR Hokuyo UST-05LX | Sensor Nav2 |
| `camera_link` | Chasis de la cámara Intel RealSense D435 (Cabezal Pan-Tilt) | Sensor RGB-D |
| `camera_color_optical_frame` | Marco óptico de color (Z hacia adelante) | Visión / Detección |
| `arm_base_link` | Base del brazo robótico AgileX Piper 6-DOF | Manipulación |
| `arm_camera_optical_frame` | Marco óptico de la cámara de muñeca Orbbec DaBai | Eye-in-Hand |
