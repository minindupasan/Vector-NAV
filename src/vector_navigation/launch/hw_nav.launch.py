"""Real-hardware Nav2 launch for VECTOR NAV (runs on Jetson Nano).

Brings up:
  1. robot_state_publisher — URDF → TF, subscribes /joint_states from Pi
  2. Nav2 stack            — autonomous navigation with a pre-built map
  3. RViz2                 — visualization + Nav2 Goal tool

All sensor topics come from the Raspberry Pi via CycloneDDS.
The Pi's EKF publishes odom→base_link TF and /odometry/filtered.
Nav2 controller publishes /cmd_vel → Pi's motor_driver subscribes it.

No Gazebo, no ros2_control, no sensor bridges.

Usage:
  ros2 launch vector_navigation hw_nav.launch.py map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('vector_navigation')

    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

    xacro_file = os.path.join(pkg, 'urdf', 'vector_urdf.xacro')
    nav2_config = os.path.join(pkg, 'config', 'nav2_params_hw.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml = LaunchConfiguration('map')
    launch_rviz = LaunchConfiguration('rviz', default='false')

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

    # ── Nav2 nodes ─────────────────────────────────────────────────
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
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_yaw': 0.0,
            'max_particles': 2000,
            'min_particles': 500,
        }],
    )

    # cmd_vel goes directly to /cmd_vel (Pi's motor_driver subscribes)
    nav2_controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': use_sim_time}],
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
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
        }],
    )

    # ── RViz2 ─────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(launch_rviz),
        arguments=['-d', os.path.join(pkg, 'config', 'hw_nav.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', description='Path to map yaml file'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Launch RViz2 (requires a display)'),

        robot_state_publisher,
        nav2_map_server,
        nav2_amcl,
        nav2_controller,
        nav2_planner,
        nav2_behaviors,
        nav2_bt,
        nav2_lifecycle,
        rviz_node,
    ])
