import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_perception = get_package_share_directory('base_athome_perception')
    pkg_bringup = get_package_share_directory('base_athome_bringup')
    script_path = os.path.join(pkg_perception, 'scripts', 'hri_receptionist_challenge.py')

    start_camera_arg = DeclareLaunchArgument(
        'start_camera',
        default_value='true',
        description='Iniciar driver de la camara RealSense D435 en la NUC'
    )
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

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'realsense_nuc.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('start_camera'))
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
        start_camera_arg,
        topic_arg,
        door_wp_arg,
        living_room_wp_arg,
        sim_nav_arg,
        no_gui_arg,
        realsense_launch,
        proc
    ])
