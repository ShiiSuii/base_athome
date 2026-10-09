import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_path = get_package_share_directory('hoverboard_base')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_path, 'launch', 'bringup.launch.py'))
    )

    return LaunchDescription([
        # --- CARGAR BASE + URDF (TF TREE + DRIVER) ---
        bringup_launch,

        # --- CARTOGRAPHER NODE ---
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', os.path.join(pkg_path, 'config'),
                '-configuration_basename', 'hoverboard.lua'
            ],
            remappings=[
                ('/scan', '/scan'),
                ('/odom', '/odom'),
                ('/imu', '/imu/data')
            ],
        ),

        # --- OCCUPANCY GRID (MAP PUBLISHER) ---
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'resolution': 0.05,
                'publish_period_sec': 1.0
            }],
        ),
        
        # --- RVIZ2 EN SU ESTADO POR DEFECTO PARA VER EL MAPA ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
