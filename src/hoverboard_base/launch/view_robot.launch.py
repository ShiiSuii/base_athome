import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('hoverboard_base')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'robot_view.rviz')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        # Publicador de articulaciones por defecto (garantiza que torso y brazo tengan TFs en RViz)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            arguments=[urdf_file]
        ),

        # Publicador de Estado del Robot (convierte /joint_states a TFs del URDF completo)
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

        # Ventana de Visualización RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_robot_view',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
