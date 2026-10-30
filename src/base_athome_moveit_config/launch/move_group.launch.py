"""move_group de MoveIt 2 para el Piper en Base@Home (solo planificacion: /compute_ik,
/plan_kinematic_path, /compute_cartesian_path). Usa el URDF completo del robot y lee
/joint_states (brazo medido por CAN, cabeza y torso)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory("hoverboard_base"), "urdf", "robot.urdf")
    cfg = (
        MoveItConfigsBuilder("robot_athome", package_name="base_athome_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path="config/base_athome.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[cfg.to_dict(), {"publish_robot_description_semantic": True,
                                    "allow_trajectory_execution": False}],
    )
    return LaunchDescription([move_group])
