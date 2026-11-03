import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
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

    pan_tilt_auto_torque_arg = DeclareLaunchArgument(
        'pan_tilt_auto_torque',
        default_value='false',
        description='Habilitar torque del pan-tilt al detectar los servos (si no, usar /pan_tilt/set_torque)'
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

    # Medido en la Jetson 2026-10-30 (CPU int8, modo 25W, frases sinteticas):
    # tiny 0.9 s/frase WER 20 %; base.en 1.5 s WER 6 %; small.en 4.4 s WER 6 %.
    # Para competencia (ingles): whisper_model:=base.en language:=en.
    # base.en/small.en son solo ingles: no usar con language:=auto/es.
    whisper_model_arg = DeclareLaunchArgument(
        'whisper_model',
        default_value='tiny',
        description='faster-whisper model: tiny (multilingue, rapido) | base.en | small.en'
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
            'auto_torque': ParameterValue(LaunchConfiguration('pan_tilt_auto_torque'), value_type=bool),
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
            'model_size': LaunchConfiguration('whisper_model'),
            'language': LaunchConfiguration('language'),
            # Escucha por pedido con fin de habla por Silero VAD (ver audio_asr/asr_core.py)
            'start_timeout_sec': 8.0,   # espera a que la persona empiece a hablar
            'end_silence_sec': 1.1,     # silencio que cierra la frase
            'max_speech_sec': 15.0,
            'auto_gain': True,          # baja la ganancia del RODE si la voz satura
        }],
        condition=IfCondition(LaunchConfiguration('enable_audio'))
    )

    return LaunchDescription([
        enable_camera_arg,
        enable_pan_tilt_arg,
        pan_tilt_auto_torque_arg,
        enable_audio_arg,
        language_arg,
        whisper_model_arg,
        realsense_launch,
        pan_tilt_node,
        whisper_node,
    ])
