from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Nodo que lee el mando bluetooth / USB
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

        # Nodo interactivo de teleoperación con marchas y HUD
        Node(
            package='hoverboard_base',
            executable='hoverboard_teleop_joy',
            name='hoverboard_teleop_joy',
            output='screen'
        )
    ])
