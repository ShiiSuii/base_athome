import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rtabmap_dir = get_package_share_directory('rtabmap_launch')

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='Base frame ID for RTAB-Map (e.g. camera_link or base_footprint)'
    )

    return LaunchDescription([
        frame_id_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtabmap_dir, 'launch', 'rtabmap.launch.py')
            ),
            launch_arguments={
                'rgb_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
                'frame_id': LaunchConfiguration('frame_id'),
                'approx_sync': 'true',
                'visual_odometry': 'true',
                'rtabmap_viz': 'true',
                'rviz': 'false',
                'qos': '1',
            }.items()
        )
    ])
