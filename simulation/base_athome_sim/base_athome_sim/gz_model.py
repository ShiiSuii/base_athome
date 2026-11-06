#!/usr/bin/env python3
"""Agrega a robot.urdf (sin modificarlo) los bloques <gazebo> para Gazebo Harmonic (gz sim):

- Base: VelocityControl (sigue /cmd_vel cinematicamente; las ruedas son 'fixed' en el URDF) +
  OdometryPublisher (odom -> base_footprint).
- Brazo, pinza, pan-tilt y elevador: JointPositionController por joint (los comanda sim_robot con
  el mismo modelo temporal que el controlador real) y gravedad apagada en todo el robot.
- Sensores: Hokuyo (gpu_lidar 270 grados, 5 m), RealSense (rgbd 640x480) y camara de muneca.
Asi Gazebo usa EXACTAMENTE la cinematica y las medidas del URDF del robot.
"""

import json
import math
import re

MODEL = "base_athome"
POSITION_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8",
                   "pan_joint", "tilt_joint", "torso_lift_joint"]


BAG_MODEL, GRASP_LINK = "bolsa", "link6"
BAG_ATTACH_TOPIC, BAG_DETACH_TOPIC = "/sim/gz/bolsa/attach", "/sim/gz/bolsa/detach"
BAG_ODOM_TOPIC = "/sim/gz/bolsa/odometry"
BAG_SIDE_M = 0.28
HAND_M = 0.15
BAG_SOLID_BELOW_TOP_M = 0.12      # arriba de esto la bolsa es blanda: los dedos entran sin empujarla


def _static_box(name, xyz, size, rgb, rpy=(0.0, 0.0, 0.0)):
    color = f"{rgb[0]} {rgb[1]} {rgb[2]} 1"
    return f"""<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>
  <pose>{xyz[0]} {xyz[1]} {xyz[2]} {rpy[0]} {rpy[1]} {rpy[2]}</pose><link name="l"><visual name="v">
  <geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>
  <material><diffuse>{color}</diffuse><ambient>{color}</ambient></material></visual></link></model></sdf>"""


def bag_sdf(xy, top_z: float) -> str:
    """Bolsa con fisica. Magenta emisiva: es lo que segmenta el detector de simulacion, con cualquier luz."""
    h, side = float(top_z), BAG_SIDE_M
    solid = max(0.05, h - BAG_SOLID_BELOW_TOP_M)
    return f"""<?xml version="1.0"?><sdf version="1.9"><model name="{BAG_MODEL}">
  <pose>{xy[0]} {xy[1]} {h / 2.0 + 0.002} 0 0 0</pose><link name="l">
  <inertial><mass>0.4</mass><inertia><ixx>0.005</ixx><iyy>0.005</iyy><izz>0.005</izz></inertia></inertial>
  <collision name="c"><pose>0 0 {solid / 2.0 - h / 2.0} 0 0 0</pose><geometry><box><size>{side} {side} {solid}</size></box></geometry>
    <surface><friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction></surface></collision>
  <visual name="v"><geometry><box><size>{side} {side} {h}</size></box></geometry>
    <material><diffuse>0.8 0.1 0.8 1</diffuse><ambient>0.8 0.1 0.8 1</ambient><emissive>0.78 0.12 0.78 1</emissive></material></visual>
  </link><plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <odom_topic>{BAG_ODOM_TOPIC}</odom_topic><odom_frame>map</odom_frame><robot_base_frame>{BAG_MODEL}</robot_base_frame>
    <odom_publish_frequency>10</odom_publish_frequency><dimensions>3</dimensions></plugin></model></sdf>"""


def table_object_sdf(xyz, length: float = 0.23, radius: float = 0.0375) -> str:
    """Objeto rigido sobre una mesa, magenta como la bolsa. Usa el modelo y la union de la bolsa."""
    return f"""<?xml version="1.0"?><sdf version="1.9"><model name="{BAG_MODEL}">
  <pose>{xyz[0]} {xyz[1]} {xyz[2]} 0 0 0</pose><link name="l">
  <inertial><mass>0.15</mass><inertia><ixx>0.0007</ixx><iyy>0.0007</iyy><izz>0.0001</izz></inertia></inertial>
  <collision name="c"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry></collision>
  <visual name="v"><geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
    <material><diffuse>0.8 0.1 0.8 1</diffuse><ambient>0.8 0.1 0.8 1</ambient><emissive>0.78 0.12 0.78 1</emissive></material></visual>
  </link><plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <odom_topic>{BAG_ODOM_TOPIC}</odom_topic><odom_frame>map</odom_frame><robot_base_frame>{BAG_MODEL}</robot_base_frame>
    <odom_publish_frequency>10</odom_publish_frequency><dimensions>3</dimensions></plugin></model></sdf>"""


def person_sdf(xy) -> str:
    """Cuerpo de la persona: bloque visual, sin colision, de la altura que usa sim_scene.make_person."""
    return _static_box("persona", (xy[0], xy[1], 0.81), (0.52, 0.52, 1.62), (0.2, 0.35, 0.6))


def arm_pose(shoulder, wrist):
    """(centro, largo, cabeceo, guinada) de una barra entre hombro y muneca, con su eje X sobre el brazo."""
    dx, dy, dz = wrist[0] - shoulder[0], wrist[1] - shoulder[1], wrist[2] - shoulder[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    center = ((shoulder[0] + wrist[0]) / 2.0, (shoulder[1] + wrist[1]) / 2.0, (shoulder[2] + wrist[2]) / 2.0)
    return center, length, math.atan2(-dz, math.hypot(dx, dy)), math.atan2(dy, dx)


def arm_with_hand(shoulder, wrist):
    """Pose del brazo. La mano sigue mas alla de la muneca: sin ella, la profundidad en la muneca cae al fondo."""
    n = math.dist(shoulder, wrist)
    return arm_pose(shoulder, [w + HAND_M * (w - v) / n for v, w in zip(shoulder, wrist)])


def arm_sdf(name, shoulder, wrist) -> str:
    center, length, pitch, yaw = arm_with_hand(shoulder, wrist)
    return _static_box(name, center, (length, 0.10, 0.10), (0.3, 0.45, 0.7), (0.0, pitch, yaw))


def create_command(world: str, sdf: str) -> list:
    return ["gz", "service", "-s", f"/world/{world}/create", "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", "sdf: " + json.dumps(sdf)]


def set_pose_command(world: str, name: str, xyz, pitch: float = 0.0, yaw: float = 0.0) -> list:
    cp, sp, cy, sy = math.cos(pitch / 2.0), math.sin(pitch / 2.0), math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    req = (f'name: "{name}" position: {{x: {xyz[0]} y: {xyz[1]} z: {xyz[2]}}} '
           f"orientation: {{x: {-sp * sy} y: {sp * cy} z: {cp * sy} w: {cp * cy}}}")
    return ["gz", "service", "-s", f"/world/{world}/set_pose", "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
            "--timeout", "2000", "--req", req]


def remove_command(world: str, name: str) -> list:
    return ["gz", "service", "-s", f"/world/{world}/remove", "--reqtype", "gz.msgs.Entity",
            "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", f'name: "{name}" type: MODEL']


def joint_cmd_topic(joint: str) -> str:
    return f"/sim/gz/{joint}/cmd_pos"


def _joint_controller(j: str) -> str:
    return f"""  <gazebo><plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
    <joint_name>{j}</joint_name><topic>{joint_cmd_topic(j)}</topic>
    <use_velocity_commands>true</use_velocity_commands><p_gain>8</p_gain><cmd_max>3</cmd_max><cmd_min>-3</cmd_min>
  </plugin></gazebo>
"""


def gazebo_blocks() -> str:
    out = [f"""  <!-- ===== Generado por base_athome_sim/gz_model.py (solo simulacion) ===== -->
  <gazebo><plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">
    <topic>/model/{MODEL}/cmd_vel</topic></plugin></gazebo>
  <gazebo><plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <odom_frame>odom</odom_frame><robot_base_frame>base_footprint</robot_base_frame>
    <odom_publish_frequency>30</odom_publish_frequency><dimensions>2</dimensions></plugin></gazebo>
  <gazebo><plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
    <topic>/sim/gz/joint_states</topic></plugin></gazebo>
  <gazebo reference="base_link"><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>
  <gazebo reference="laser"><sensor name="hokuyo" type="gpu_lidar">
    <topic>/sim/gz/scan</topic><gz_frame_id>laser</gz_frame_id><update_rate>10</update_rate>
    <lidar><scan><horizontal><samples>541</samples><min_angle>-2.356</min_angle><max_angle>2.356</max_angle></horizontal></scan>
    <range><min>0.06</min><max>5.0</max></range></lidar><always_on>1</always_on></sensor></gazebo>
  <gazebo reference="camera_link"><sensor name="realsense" type="rgbd_camera">
    <topic>/sim/gz/head</topic><gz_frame_id>camera_color_optical_frame</gz_frame_id><update_rate>15</update_rate>
    <camera><horizontal_fov>0.962</horizontal_fov><image><width>640</width><height>480</height></image>
    <clip><near>0.1</near><far>8</far></clip></camera><always_on>1</always_on></sensor></gazebo>
  <gazebo reference="arm_camera_link"><sensor name="wrist_camera" type="camera">
    <topic>/sim/gz/wrist</topic><gz_frame_id>arm_camera_optical_frame</gz_frame_id><update_rate>15</update_rate>
    <camera><horizontal_fov>1.165</horizontal_fov><image><width>640</width><height>480</height></image>
    <clip><near>0.03</near><far>5</far></clip></camera><always_on>1</always_on></sensor></gazebo>
"""]
    out += [_joint_controller(j) for j in POSITION_JOINTS]
    # Agarrar la bolsa: una union que se crea al cerrar la pinza sobre ella y se suelta al abrir
    out.append(f"""  <gazebo><plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
    <parent_link>{GRASP_LINK}</parent_link><child_model>{BAG_MODEL}</child_model><child_link>l</child_link>
    <attach_topic>{BAG_ATTACH_TOPIC}</attach_topic><detach_topic>{BAG_DETACH_TOPIC}</detach_topic>
    <suppress_child_warning>true</suppress_child_warning></plugin></gazebo>
""")
    return "".join(out)


def gazebo_urdf(urdf: str) -> str:
    """Devuelve el URDF con los bloques de Gazebo insertados antes de </robot>."""
    i = urdf.rfind("</robot>")
    if i < 0:
        raise ValueError("URDF sin </robot>")
    # Todo el robot sin gravedad: la base se mueve por velocidad, sin ruedas ni rueda loca que la sostengan. Con gravedad
    # cabeceaba 12 grados al avanzar, y las camaras dejaban de estar donde dice el TF.
    links = re.findall(r'<link name="([^"]+)"', urdf)
    gravity = "".join(f'  <gazebo reference="{ln}"><gravity>0</gravity></gazebo>\n' for ln in links)
    return urdf[:i] + gazebo_blocks() + gravity + urdf[i:]


def bridge_args() -> list:
    """Argumentos para ros_gz_bridge parameter_bridge (gz -> ROS con [, ROS -> gz con ])."""
    args = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        f"/model/{MODEL}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        f"/model/{MODEL}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/sim/gz/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/sim/gz/head/image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/sim/gz/head/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
        "/sim/gz/head/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        "/sim/gz/wrist@sensor_msgs/msg/Image[gz.msgs.Image",  # sensor "camera": la imagen va en el topico base
    ]
    args += [f"{joint_cmd_topic(j)}@std_msgs/msg/Float64]gz.msgs.Double" for j in POSITION_JOINTS]
    args += [f"{BAG_ATTACH_TOPIC}@std_msgs/msg/Empty]gz.msgs.Empty", f"{BAG_DETACH_TOPIC}@std_msgs/msg/Empty]gz.msgs.Empty",
             f"{BAG_ODOM_TOPIC}@nav_msgs/msg/Odometry[gz.msgs.Odometry"]
    return args
