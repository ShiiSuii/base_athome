"""Nav2 de verdad contra Gazebo: los mismos servidores y parametros que el robot, leidos de nav2_params.yaml sin modificarlo.
Solo cambia, en una copia temporal: el mapa, la pose inicial y las rutas de este equipo."""
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

NODES = ("map_server", "amcl", "planner_server", "controller_server", "behavior_server", "bt_navigator")


def _merge(base, extra):
    """Mezcla extra dentro de base, entrando en los diccionarios. Lo demas lo reemplaza."""
    for key, value in extra.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def _params(map_yaml):
    with open(os.path.join(get_package_share_directory("base_athome_navigation"), "config", "nav2_params.yaml"), encoding="utf-8") as f:
        params = yaml.safe_load(f)
    params["map_server"]["ros__parameters"]["yaml_filename"] = map_yaml
    amcl = params["amcl"]["ros__parameters"]
    amcl["set_initial_pose"] = True
    start = [float(v) for v in os.environ.get("ATHOME_SIM_START", "0,0,0").split(",")]   # la misma pose con la que sim.launch.py crea el robot
    amcl["initial_pose"] = dict(x=start[0], y=start[1], z=0.0, yaw=math.radians(start[2]))
    bt = params["bt_navigator"]["ros__parameters"]
    if "default_nav_to_pose_bt_xml" in bt:
        bt["default_nav_to_pose_bt_xml"] = os.path.join(get_package_share_directory("base_athome_navigation"), "config", "bt_trees",
                                                        os.path.basename(bt["default_nav_to_pose_bt_xml"]))
    profile = os.environ.get("ATHOME_NAV_PROFILE", "")
    if profile:     # ATHOME_NAV_PROFILE=eval: aplica config/nav2_params_<perfil>.yaml de esta carpeta encima de los del robot
        sim_config = os.path.join(get_package_share_directory("base_athome_sim"), "config")
        with open(os.path.join(sim_config, f"nav2_params_{profile}.yaml"), encoding="utf-8") as f:
            _merge(params, yaml.safe_load(f))
        bt["default_nav_to_pose_bt_xml"] = os.path.join(sim_config, os.path.basename(bt["default_nav_to_pose_bt_xml"])) \
            if not os.path.isabs(bt["default_nav_to_pose_bt_xml"]) else bt["default_nav_to_pose_bt_xml"]
    out = tempfile.NamedTemporaryFile("w", suffix="_nav2_sim.yaml", delete=False, encoding="utf-8")
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def _setup(_context):
    map_yaml = os.environ["ATHOME_SIM_MAP"]
    params = _params(map_yaml)
    packages = dict(amcl="nav2_amcl", map_server="nav2_map_server", planner_server="nav2_planner", controller_server="nav2_controller",
                    behavior_server="nav2_behaviors", bt_navigator="nav2_bt_navigator")
    actions = [Node(package="base_athome_bringup", executable="scan_self_filter", name="scan_self_filter", output="screen")]
    actions.extend(Node(package=pkg, executable=n, name=n, output="screen", parameters=[params]) for n, pkg in packages.items())
    actions.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
                        output="screen", parameters=[dict(autostart=True, node_names=list(NODES), bond_timeout=20.0)]))
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup)])
