import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_perception = get_package_share_directory('base_athome_perception')
    script_path = os.path.join(pkg_perception, 'scripts', 'hri_receptionist_challenge.py')

    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/camera/camera/color/image_raw',
        description='Topico de imagen ROS 2 (RealSense D435 o Head Camera)'
    )
    door_wp_arg = DeclareLaunchArgument(
        'door_wp',
        default_value='A',
        description='Waypoint de la puerta de entrada'
    )
    living_room_wp_arg = DeclareLaunchArgument(
        'living_room_wp',
        default_value='B',
        description='Waypoint de la sala de estar / living'
    )
    sim_nav_arg = DeclareLaunchArgument(
        'sim_nav',
        default_value='false',
        description='Simula navegacion sin mover la base fisica'
    )
    no_gui_arg = DeclareLaunchArgument(
        'no_gui',
        default_value='false',
        description='Ejecutar sin ventana OpenCV HUD'
    )

    proc = ExecuteProcess(
        cmd=[
            '/home/athome/ros2_vision_venv/bin/python3',
            script_path,
            '--topic', LaunchConfiguration('topic'),
            '--door-wp', LaunchConfiguration('door_wp'),
            '--living-room-wp', LaunchConfiguration('living_room_wp'),
        ],
        output='screen'
    )

    return LaunchDescription([
        topic_arg,
        door_wp_arg,
        living_room_wp_arg,
        sim_nav_arg,
        no_gui_arg,
        proc
    ])
