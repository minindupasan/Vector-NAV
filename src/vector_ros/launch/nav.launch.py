"""Navigation launch for VECTOR NAV.

Starts simulation + Nav2 with a pre-built map for autonomous navigation.
Use RViz2 "Nav2 Goal" tool to send goals.

Usage:
  ros2 launch vector_ros nav.launch.py map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('vector_ros')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ.setdefault('__EGL_VENDOR_LIBRARY_DIRS', '/usr/share/glvnd/egl_vendor.d/')
    os.environ.setdefault('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    pkg_share_parent = os.path.join(get_package_prefix('vector_ros'), 'share')
    models_dir = os.path.join(pkg, 'models')
    resource_paths = os.pathsep.join([pkg_share_parent, models_dir])
    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        resource_paths += os.pathsep + os.environ['IGN_GAZEBO_RESOURCE_PATH']
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = resource_paths

    xacro_file = os.path.join(pkg, 'urdf', 'vector_urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'vector_world.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    map_yaml = LaunchConfiguration('map')
    nav2_config = os.path.join(pkg, 'config', 'nav2_params.yaml')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── robot_state_publisher ──────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ── Gazebo ─────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'vector_urdf',
            '-x', x_pose, '-y', y_pose, '-z', '0.1',
        ],
    )

    # ── Controllers ────────────────────────────────────────────────
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '120',
                   '--service-call-timeout', '60'],
        output='screen',
    )

    spawn_ddc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager-timeout', '120',
                   '--service-call-timeout', '60'],
        output='screen',
    )

    # ── Sensor bridge ──────────────────────────────────────────────
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
    )

    # ── EKF ────────────────────────────────────────────────────────
    ekf_config = os.path.join(pkg, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # ── Nav2 ───────────────────────────────────────────────────────
    nav2_lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
    ]

    nav2_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[nav2_config, {
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    nav2_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'scan_topic': '/scan',
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'set_initial_pose': True,
            'initial_pose_x': x_pose,
            'initial_pose_y': y_pose,
            'initial_pose_yaw': 0.0,
            'max_particles': 2000,
            'min_particles': 500,
        }],
    )

    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    nav2_planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
    )

    nav2_behaviors = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    )

    nav2_bt = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
    )

    nav2_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'] + nav2_lifecycle_nodes,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('map', description='Path to map yaml file'),

        robot_state_publisher,
        TimerAction(period=2.0, actions=[gz_sim]),
        TimerAction(period=5.0, actions=[spawn_entity]),
        sensor_bridge,
        TimerAction(period=12.0, actions=[spawn_jsb]),
        TimerAction(period=15.0, actions=[spawn_ddc]),
        TimerAction(period=18.0, actions=[ekf_node]),
        TimerAction(period=20.0, actions=[
            nav2_map_server,
            nav2_amcl,
            nav2_controller,
            nav2_planner,
            nav2_behaviors,
            nav2_bt,
            nav2_lifecycle,
        ]),
    ])
