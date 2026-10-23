import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_perception = get_package_share_directory('base_athome_perception')
    script_path = os.path.join(pkg_perception, 'scripts', 'test_human_pose.py')

    topic_arg = DeclareLaunchArgument(
        'topic',
        default_value='/camera/camera/color/image_raw',
        description='Topico de imagen ROS 2'
    )

    proc = ExecuteProcess(
        cmd=[
            '/home/athome/ros2_vision_venv/bin/python3',
            script_path,
            '--topic', LaunchConfiguration('topic')
        ],
        output='screen'
    )

    return LaunchDescription([
        topic_arg,
        proc
    ])
