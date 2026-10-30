import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('base_athome_skills'), 'config', 'door_open_detector.yaml')
    return LaunchDescription([
        Node(package='base_athome_skills', executable='door_open_detector',
             name='door_open_detector', output='screen', parameters=[params]),
    ])
