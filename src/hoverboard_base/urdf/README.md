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

## Quién publica cada joint (Política de dueño único — HOME-20)

Para evitar saltos y parpadeos en RViz, cada articulación tiene un **único dueño productor** de su estado medido:

| Joint(s) | Tipo | Tópico de publicación | Nodo responsable / Dueño | Tasa | Nota |
| :--- | :--- | :--- | :--- | :--- | :--- |
| `pan_joint`, `tilt_joint` | Revolute | `/pan_tilt/joint_states` | `pan_tilt_node` (Jetson) | 20 Hz | Medido de servos Dynamixel XL430 |
| `joint1`..`joint6` (brazo) + `joint7`, `joint8` (pinza) | Revolute / Prismatic | `/arm/joint_states` | `piper_arm_controller.py` (Jetson) | 50 Hz | Medido del bus CAN del brazo Piper |
| `torso_lift_joint` | Prismatic | `/torso/joint_states` | Driver elevador NEMA23 / scripts de fallback | 25 Hz | Eje vertical del torso prismático |
| *(Agregación de todos los joints)* | — | `/joint_states` | `joint_state_publisher` (`view_robot.launch.py`) | ~30 Hz | Suscrito a `source_list=['/arm/joint_states', '/pan_tilt/joint_states', '/torso/joint_states']`; completa con 0.0 cualquier joint sin fuente |
| `joint1`..`joint8` (comandados) | — | `/arm/joint_states_cmd` | `manipulacion_autonoma.py`, `hri_receptionist_challenge.py` | 25 Hz | Visualización de pose objetivo (NUNCA publicado en `/joint_states`) |

### Reglas de integración
1. **NUNCA publicar poses de aplicación en `/joint_states`**: los nodos de alto nivel (`manipulacion_autonoma.py`, `hri_receptionist_challenge.py`) no publican directamente en `/joint_states`.
2. **Visualización de poses planificadas**: si un nodo necesita mostrar la pose comandada en RViz, la publica exclusivamente en `/arm/joint_states_cmd`.
3. **Agregación centralizada**: `joint_state_publisher` fusiona las fuentes parciales (`/arm/joint_states`, `/pan_tilt/joint_states`, `/torso/joint_states`) en `/joint_states`, desde donde `robot_state_publisher` calcula y emite el árbol de transformaciones TF.
