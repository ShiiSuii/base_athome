import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('base_athome_navigation')
    config_path = os.path.join(pkg_nav, 'config', 'ps4.config.yaml')

    smart_teleop_arg = DeclareLaunchArgument(
        'smart',
        default_value='true',
        description='Usar teleoperación inteligente con marchas y regulación de velocidad'
    )

    return LaunchDescription([
        smart_teleop_arg,

        # Nodo que lee el joystick por Bluetooth o USB
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.05,
                'autorepeat_rate': 20.0
            }],
            output='screen'
        ),

        # MODO SMART: Teleoperación interactiva con marchas ajustables en tiempo real
        Node(
            package='hoverboard_base',
            executable='hoverboard_teleop_joy',
            name='hoverboard_teleop_joy',
            condition=IfCondition(LaunchConfiguration('smart')),
            output='screen'
        ),

        # MODO ESTÁNDAR: teleop_twist_joy clásico
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            condition=UnlessCondition(LaunchConfiguration('smart')),
            parameters=[config_path],
            remappings=[
                ('/cmd_vel', '/cmd_vel')
            ],
            output='screen'
        )
    ])
