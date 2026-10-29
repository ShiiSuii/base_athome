import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Launch RealSense D435 camera driver'
    )

    enable_pan_tilt_arg = DeclareLaunchArgument(
        'enable_pan_tilt',
        default_value='true',
        description='Launch ROBOTIS Pan-Tilt controller node'
    )

    enable_audio_arg = DeclareLaunchArgument(
        'enable_audio',
        default_value='true',
        description='Launch Whisper ASR audio node'
    )

    language_arg = DeclareLaunchArgument(
        'language',
        default_value='auto',
        description='Language for Whisper ASR (auto for bilingual, en for English, es for Spanish)'
    )

    # 1. RealSense D435 Launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_sync': 'true',
            'depth_module.profile': '640x480x15',
            'rgb_camera.profile': '640x480x15',
            'pointcloud.enable': 'false',
            'publish_tf': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )

    # 2. Pan-Tilt Node
    pan_tilt_node = Node(
        package='pan_tilt_controller',
        executable='pan_tilt_node',
        name='pan_tilt_node',
        output='screen',
        parameters=[{
            'device_name': '/dev/ttyU2D2',
            'baudrate': 1000000,
            'protocol_version': 2.0,
            'pan_id': 1,
            'tilt_id': 2,
        }],
        condition=IfCondition(LaunchConfiguration('enable_pan_tilt'))
    )

    # 3. Whisper ASR Node (Configuracion inspirada en fbot_hri)
    whisper_node = Node(
        package='audio_asr',
        executable='whisper_node',
        name='whisper_asr_node',
        output='screen',
        parameters=[{
            'model_size': 'base.en',
            'language': LaunchConfiguration('language'),
            'energy_threshold': 0.02,
            'silence_duration_sec': 1.0,
            'max_utterance_sec': 8.0,
            'trigger_wait_sec': 7.0,
        }],
        condition=IfCondition(LaunchConfiguration('enable_audio'))
    )

    return LaunchDescription([
        enable_camera_arg,
        enable_pan_tilt_arg,
        enable_audio_arg,
        language_arg,
        realsense_launch,
        pan_tilt_node,
        whisper_node,
    ])
