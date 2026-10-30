#!/usr/bin/env python3
"""Agrega a robot.urdf (sin modificarlo) los bloques <gazebo> para Gazebo Harmonic (gz sim):

- Base: VelocityControl (sigue /cmd_vel cinematicamente; las ruedas son 'fixed' en el URDF) +
  OdometryPublisher (odom -> base_footprint).
- Brazo, pinza, pan-tilt y elevador: JointPositionController por joint (los comanda sim_robot con
  el mismo modelo temporal que el controlador real) y gravedad apagada en esos eslabones.
- Sensores: Hokuyo (gpu_lidar 270 grados, 5 m), RealSense (rgbd 640x480) y camara de muneca.
Asi Gazebo usa EXACTAMENTE la cinematica y las medidas del URDF del robot.
"""

MODEL = "base_athome"
POSITION_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7", "joint8",
                   "pan_joint", "tilt_joint", "torso_lift_joint"]
NO_GRAVITY_LINKS = ["torso_lift_link", "link1", "link2", "link3", "link4", "link5", "link6", "link7", "link8",
                    "pan_link", "tilt_link"]


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
    out += [f'  <gazebo reference="{ln}"><gravity>0</gravity></gazebo>\n' for ln in NO_GRAVITY_LINKS]
    return "".join(out)


def gazebo_urdf(urdf: str) -> str:
    """Devuelve el URDF con los bloques de Gazebo insertados antes de </robot>."""
    i = urdf.rfind("</robot>")
    if i < 0:
        raise ValueError("URDF sin </robot>")
    return urdf[:i] + gazebo_blocks() + urdf[i:]


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
    return args
