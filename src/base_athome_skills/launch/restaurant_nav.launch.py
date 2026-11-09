"""Navegacion sin mapa previo para la prueba Restaurant (RoboCup@Home 2026.2).

Levanta:
  1. async_slam_toolbox_node (SLAM Toolbox 2D online) con slam_toolbox.yaml.
  2. lifecycle_manager_slam para transicionar SLAM Toolbox de forma fiable.
  3. Servidores de Nav2 (planner, controller, behaviors, bt_navigator) SIN amcl ni map_server,
     con parametros de nav2_params.yaml + overlay nav2_restaurant.yaml (obstaculos > 0.38 m,
     allow_unknown y costmap en frame map).
  4. lifecycle_manager_navigation para coordinar los servidores de Nav2.
"""

import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def _merge(base: dict, extra: dict) -> dict:
    """Mezcla recursivamente extra dentro de base."""
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def _prepare_nav_params(use_sim_time: bool) -> str:
    pkg_nav = get_package_share_directory("base_athome_navigation")
    pkg_skills = get_package_share_directory("base_athome_skills")

    base_yaml_path = os.path.join(pkg_nav, "config", "nav2_params.yaml")
    with open(base_yaml_path, "r", encoding="utf-8") as f:
        params = yaml.safe_load(f)

    overlay_path = os.path.join(pkg_skills, "config", "nav2_restaurant.yaml")
    if os.path.exists(overlay_path):
        with open(overlay_path, "r", encoding="utf-8") as f:
            overlay = yaml.safe_load(f)
        _merge(params, overlay)

    # Ajuste de BT trees si es necesario
    bt = params.get("bt_navigator", {}).get("ros__parameters", {})
    bt_xml = os.path.join(pkg_nav, "config", "bt_trees", "base_athome_bt.xml")
    if os.path.exists(bt_xml):
        bt["default_bt_xml_filename"] = bt_xml
        bt["default_nav_to_pose_bt_xml"] = bt_xml

    if use_sim_time:
        for node_name in ("planner_server", "controller_server", "behavior_server", "bt_navigator"):
            if node_name in params and "ros__parameters" in params[node_name]:
                params[node_name]["ros__parameters"]["use_sim_time"] = True

    out = tempfile.NamedTemporaryFile("w", suffix="_nav2_restaurant.yaml", delete=False, encoding="utf-8")
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def _prepare_slam_params(use_sim_time: bool) -> str:
    pkg_mapping = get_package_share_directory("base_athome_mapping")
    slam_yaml_path = os.path.join(pkg_mapping, "config", "slam_toolbox.yaml")
    with open(slam_yaml_path, "r", encoding="utf-8") as f:
        slam_params = yaml.safe_load(f)

    if use_sim_time:
        if "slam_toolbox" in slam_params and "ros__parameters" in slam_params["slam_toolbox"]:
            slam_params["slam_toolbox"]["ros__parameters"]["use_sim_time"] = True

    out = tempfile.NamedTemporaryFile("w", suffix="_slam_restaurant.yaml", delete=False, encoding="utf-8")
    yaml.safe_dump(slam_params, out)
    out.close()
    return out.name


def _setup(context):
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = use_sim_time_str.lower() in ("true", "1")
    autostart_str = LaunchConfiguration("autostart").perform(context)
    autostart = autostart_str.lower() in ("true", "1")

    nav_params_file = _prepare_nav_params(use_sim_time)
    slam_params_file = _prepare_slam_params(use_sim_time)

    # 1. SLAM Toolbox Asincrono
    slam_toolbox_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            slam_params_file,
            {
                "use_lifecycle_manager": False,   # se activa solo: con el gestor, este lo configuraba antes de que el nodo estuviera listo y SLAM nunca arrancaba
                "use_sim_time": use_sim_time,
            },
        ],
    )

    lifecycle_slam_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {
                "autostart": autostart,
                "node_names": ["slam_toolbox"],
                "bond_timeout": 4.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # 2. Servidores Nav2 (Sin amcl ni map_server)
    planner_server_node = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav_params_file],
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav_params_file],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    behavior_server_node = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav_params_file],
        remappings=[("cmd_vel", "/cmd_vel")],
    )

    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav_params_file],
    )

    lifecycle_nav_nodes = [
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    lifecycle_nav_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "autostart": autostart,
                "node_names": lifecycle_nav_nodes,
                "bond_timeout": 4.0,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    return [
        slam_toolbox_node,
        # lifecycle_slam_node: no hace falta, slam_toolbox se activa solo (ver use_lifecycle_manager)
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_nav_node,
    ]


def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Usar tiempo de simulacion si es true",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Iniciar automaticamente los nodos del ciclo de vida",
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        OpaqueFunction(function=_setup),
    ])
