import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_realsense = get_package_share_directory('realsense2_camera')
    rs_launch = os.path.join(pkg_realsense, 'launch', 'rs_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': 'camera',
                'rgb_camera.color_profile': '640x480x15',
                'depth_module.depth_profile': '640x480x15',
                'enable_depth': 'true',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'align_depth.enable': 'true',
                'enable_sync': 'true',
                'pointcloud.enable': 'false',
                'clip_distance': '4.0',
            }.items(),
        )
    ])
