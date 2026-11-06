"""Simulacion de Base@Home.

  ros2 launch base_athome_sim sim.launch.py                 # simple: sin fisica, liviana (CPU minima)
  ros2 launch base_athome_sim sim.launch.py gazebo:=true    # Gazebo Harmonic (gz sim) con fisica/render
Argumentos: rviz:=true|false  auto_guest:=true|false  gz_gui:=true|false
Usar SIEMPRE en un ROS_DOMAIN_ID distinto del robot (alias `sim_env` -> 42).
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from base_athome_sim import gz_model


def _setup(context):
    use_gz = LaunchConfiguration("gazebo").perform(context).lower() == "true"
    gz_gui = LaunchConfiguration("gz_gui").perform(context).lower() == "true"
    urdf_path = os.path.join(get_package_share_directory("hoverboard_base"), "urdf", "robot.urdf")
    with open(urdf_path, encoding="utf-8") as f:
        urdf = f.read()
    sim_share = get_package_share_directory("base_athome_sim")
    actions = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": urdf}]),
        Node(package="base_athome_sim", executable="sim_robot", output="screen",
             parameters=[{"gazebo": use_gz,
                          "auto_guest": LaunchConfiguration("auto_guest").perform(context).lower() == "true"}]),
    ]
    if use_gz:
        shares = {os.path.dirname(get_package_share_directory(p)) for p in ("piper_description", "hoverboard_base")}
        shares.add(os.path.join(get_package_prefix("piper_description"), "share"))
        world_arg = LaunchConfiguration("world").perform(context)
        world_env = os.environ.get("ATHOME_SIM_WORLD")
        if world_arg:
            world = world_arg if os.path.isabs(world_arg) else os.path.join(sim_share, "worlds", world_arg)
        elif world_env:
            world = world_env if os.path.isabs(world_env) else os.path.join(sim_share, "worlds", world_env)
        else:
            world = os.path.join(sim_share, "worlds", "casa_basura.sdf" if os.environ.get("ATHOME_SIM_BAG") else "casa_simple.sdf")
        gz_urdf = os.path.join("/tmp", f"base_athome_gz_{os.getuid()}.urdf")
        with open(gz_urdf, "w", encoding="utf-8") as f:
            f.write(gz_model.gazebo_urdf(urdf))
        actions += [
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ":".join(sorted(shares) + [os.environ.get("GZ_SIM_RESOURCE_PATH", "")])),
            ExecuteProcess(cmd=["gz", "sim", "-r", world] + ([] if gz_gui else ["-s", "--headless-rendering"]), output="log"),
            Node(package="ros_gz_sim", executable="create", output="screen",
                 arguments=["-name", gz_model.MODEL, "-file", gz_urdf, "-z", "0.0"]),
            Node(package="ros_gz_bridge", executable="parameter_bridge", output="log",
                 arguments=gz_model.bridge_args(),
                 parameters=[{"override_timestamps_with_wall_time": True}]),
        ]
    return actions


def generate_launch_description():
    rviz_cfg = os.path.join(get_package_share_directory("base_athome_sim"), "rviz", "sim.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("gazebo", default_value="false"),
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("gz_gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("auto_guest", default_value="false"),
        SetEnvironmentVariable("ATHOME_SIM", "1"),
        OpaqueFunction(function=_setup),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
