from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_bringup = get_package_share_directory('base_athome_bringup')
    pkg_hoverboard = get_package_share_directory('hoverboard_base')

    params_file = os.path.join(
        pkg_bringup,
        'config',
        'robot_params.yaml'
    )

    urdf_path = os.path.join(
        pkg_hoverboard,
        'urdf',
        'robot.urdf'
    )
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # --- Robot State Publisher (URDF del robot a TF estático /tf_static) ---
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': False
            }]
        ),

        # --- Driver de Hoverboard (Control Serial UART + Odometría de Encoders Hall) ---
        Node(
            package='hoverboard_base',
            executable='hoverboard_driver',
            name='hoverboard_driver',
            output='screen',
            parameters=[params_file],
        )
    ])
