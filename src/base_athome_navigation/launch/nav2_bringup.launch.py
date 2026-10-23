import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('base_athome_navigation')
    pkg_bringup = get_package_share_directory('base_athome_bringup')
    pkg_urg = get_package_share_directory('urg_node')

    default_config_dir = os.path.join(pkg_nav, 'config')
    default_params_file = os.path.join(default_config_dir, 'nav2_params.yaml')
    default_rviz_config = os.path.join(default_config_dir, 'base_athome_nav2.rviz')

    # Busca primero mapa_base_athome.yaml, si no existe usa mapa_solver.yaml
    default_map_yaml = '/home/athome/mapa_base_athome.yaml'
    if not os.path.exists(default_map_yaml) and os.path.exists('/home/athome/mapa_solver.yaml'):
        default_map_yaml = '/home/athome/mapa_solver.yaml'

    # --- Declare Launch Arguments ---
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

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Ruta completa al mapa YAML'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Ruta completa a nav2_params.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo de simulacion si es True'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Iniciar automaticamente los nodos de Nav2'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Abrir RViz2 con herramientas de navegacion'
    )

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    rviz = LaunchConfiguration('rviz')

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

    # === MAP SERVER ===
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml}]
    )

    # === AMCL (Localización Probabilística) ===
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        remappings=[('/scan', '/scan')]
    )

    # === PLANNER SERVER (SmacPlanner2D) ===
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file]
    )

    # === CONTROLLER SERVER (RotationShimController + RegulatedPurePursuit) ===
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # === BEHAVIOR SERVER ===
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[('cmd_vel', '/cmd_vel')]
    )

    # === BT NAVIGATOR ===
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file]
    )

    # === LIFECYCLE MANAGER ===
    lifecycle_nodes = [
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator'
    ]

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': lifecycle_nodes,
            'bond_timeout': 4.0
        }]
    )

    # === RVIZ2 ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        launch_base_arg,
        launch_lidar_arg,
        launch_teleop_arg,
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        autostart_arg,
        rviz_arg,
        base_launch,
        lidar_launch,
        teleop_launch,
        map_server_node,
        amcl_node,
        planner_server_node,
        controller_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node,
        rviz_node
    ])
