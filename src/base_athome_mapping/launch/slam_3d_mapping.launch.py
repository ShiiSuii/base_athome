import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    pkg_mapping = get_package_share_directory('base_athome_mapping')
    pkg_bringup = get_package_share_directory('base_athome_bringup')
    pkg_urg = get_package_share_directory('urg_node')
    pkg_nav = get_package_share_directory('base_athome_navigation')
    rtabmap_dir = get_package_share_directory('rtabmap_launch')

    default_slam_params = os.path.join(pkg_mapping, 'config', 'slam_toolbox.yaml')
    default_rviz_config = os.path.join(pkg_mapping, 'config', 'mapping_3d.rviz')

    # --- Launch Arguments ---
    launch_base_arg = DeclareLaunchArgument(
        'launch_base',
        default_value='true',
        description='Lanzar driver hoverboard y robot_state_publisher'
    )

    launch_lidar_arg = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Lanzar driver Hokuyo LiDAR (urg_node)'
    )

    launch_teleop_arg = DeclareLaunchArgument(
        'launch_teleop',
        default_value='true',
        description='Lanzar control por joystick PS4'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Abrir RViz2 con vista combinada 2D y 3D'
    )

    # 1. Base del robot (Hoverboard Driver + Robot State Publisher)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'base.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_base'))
    )

    # 2. LiDAR Hokuyo UST-05LX (Publica /scan a 40 Hz)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_urg, 'launch', 'urg_node_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_lidar'))
    )

    # 3. Control Joystick PS4 (/joy -> /cmd_vel)
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'teleop_ps4.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_teleop'))
    )

    # 4. SLAM Toolbox (Mapeo 2D con Laser y Odometría -> genera /map y TF map->odom)
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            default_slam_params,
            {
                'use_lifecycle_manager': True,
                'use_sim_time': False,
            }
        ]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['slam_toolbox']},
            {'bond_timeout': 4.0},
            {'use_sim_time': False}
        ]
    )

    # 5. Generador de Nube de Puntos 3D en Tiempo Real (PointCloud2 a color)
    point_cloud_node = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='point_cloud_xyzrgb',
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'decimation': 2,
            'max_depth': 4.5,
            'min_depth': 0.2,
            'voxel_size': 0.02,
            'qos': 1,
            'qos_camera_info': 1
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('cloud', '/camera/cloud_points'),
        ]
    )

    # 6. RTAB-Map 3D (Registro de grafo SLAM y memoria de mapa 3D)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtabmap_dir, 'launch', 'rtabmap.launch.py')
        ),
        launch_arguments={
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'visual_odometry': 'false',
            'publish_tf_map': 'false',
            'approx_sync': 'true',
            'approx_sync_max_interval': '0.1',
            'subscribe_scan': 'false',
            'subscribe_depth': 'true',
            'rtabmap_viz': 'false',
            'rviz': 'false',
            'qos': '1',
            'args': '-d --Grid/Sensor 1 --Grid/3D true --Grid/RangeMax 4.5 --Grid/CellSize 0.05 --Rtabmap/DetectionRate 2.0 --RGBD/LinearUpdate 0.05 --RGBD/AngularUpdate 0.05',
        }.items()
    )

    # 7. Ensamblador de Mapa 3D Global Acumulado (RTAB-Map)
    map_assembler_node = Node(
        package='rtabmap_util',
        executable='map_assembler',
        name='map_assembler',
        output='screen',
        parameters=[{
            'cloud_output_voxelized': True,
            'cloud_voxel_size': 0.03,
        }],
        remappings=[
            ('mapData', '/rtabmap/mapData'),
            ('cloud_map', '/rtabmap/cloud_map'),
        ]
    )

    # 8. RViz2 con vista unificada 2D + 3D
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping_3d',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        launch_base_arg,
        launch_lidar_arg,
        launch_teleop_arg,
        launch_rviz_arg,
        base_launch,
        lidar_launch,
        teleop_launch,
        slam_toolbox_node,
        lifecycle_manager_node,
        point_cloud_node,
        rtabmap_launch,
        map_assembler_node,
        rviz_node,
    ])
