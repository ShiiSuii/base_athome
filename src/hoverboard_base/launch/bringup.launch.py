import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = get_package_share_directory('hoverboard_base')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    params_file = os.path.join(pkg_path, 'config', 'hoverboard_params.yaml')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        # Robot State Publisher -> Broadcasts URDF Links to /tf_static
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description': robot_desc
            }]
        ),

        # Hoverboard Hardware Driver Node
        Node(
            package='hoverboard_base',
            executable='hoverboard_driver',
            name='hoverboard_driver',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])
