import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_mapping = get_package_share_directory('base_athome_mapping')

    default_params_file = os.path.join(
        pkg_mapping,
        'config',
        'slam_toolbox.yaml'
    )

    default_rviz_config = os.path.join(
        pkg_mapping,
        'config',
        'mapping_slam.rviz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz = LaunchConfiguration('rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulación si es true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Ruta completa al archivo de parámetros YAML para SLAM Toolbox')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Abrir RViz2 automáticamente con configuración de mapeo')

    # Nodo Asíncrono Online de SLAM Toolbox
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            params_file,
            {
                'use_lifecycle_manager': True,
                'use_sim_time': use_sim_time,
            }
        ]
    )

    # Lifecycle Manager oficial de Nav2 para gestionar la transición y bond de SLAM Toolbox de forma fiable
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['slam_toolbox']},
            {'bond_timeout': 4.0},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Visualización en RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_rviz_cmd,
        slam_toolbox_node,
        lifecycle_manager_node,
        rviz_node,
    ])
