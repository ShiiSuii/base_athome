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

    default_slam_params = os.path.join(pkg_mapping, 'config', 'slam_toolbox.yaml')
    default_rviz_config = os.path.join(pkg_mapping, 'config', 'mapping_slam.rviz')

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
        description='Abrir RViz2 con vista de mapeo 2D'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulacion si es true'
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

    # 4. SLAM Toolbox 2D (Laser + Odometria -> genera /map y TF map->odom)
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
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    # 5. Lifecycle Manager oficial de Nav2 para SLAM Toolbox
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['slam_toolbox']},
            {'bond_timeout': 4.0},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # 6. RViz2 con configuracion 2D limpia
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_mapping_2d',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    return LaunchDescription([
        launch_base_arg,
        launch_lidar_arg,
        launch_teleop_arg,
        launch_rviz_arg,
        use_sim_time_arg,
        base_launch,
        lidar_launch,
        teleop_launch,
        slam_toolbox_node,
        lifecycle_manager_node,
        rviz_node,
    ])
