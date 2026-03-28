"""Real-hardware SLAM launch for VECTOR NAV (runs on Jetson Nano).

Brings up:
  1. robot_state_publisher — URDF → TF, subscribes /joint_states from Pi
  2. SLAM Toolbox          — /scan + odom→base_link TF → map
  3. RViz2                 — visualization

All sensor topics (/scan, /imu, /wheel/odom, /joint_states) come from
the Raspberry Pi via CycloneDDS.  The Pi's EKF publishes odom→base_link TF.

No Gazebo, no ros2_control, no sensor bridges.

Usage:
  ros2 launch vector_ros hw_slam.launch.py
  # Drive the robot with teleop to build a map, then save:
  ros2 run nav2_map_server map_saver_cli -f ~/vector_nav/maps/my_map
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('vector_ros')

    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'

    xacro_file = os.path.join(pkg, 'urdf', 'vector_urdf.xacro')
    slam_config = os.path.join(pkg, 'config', 'slam_toolbox_hw.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── robot_state_publisher ──────────────────────────────────────
    # Subscribes /joint_states (from Pi's motor_driver_node) and
    # publishes TF for all URDF links (wheels spin in RViz).
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

    # ── SLAM Toolbox ───────────────────────────────────────────────
    # Uses /scan from RPLidar (via Pi) + odom→base_link TF (from Pi's EKF)
    # Publishes map→odom TF (corrects drift via scan matching)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': use_sim_time}],
    )

    # ── RViz2 ─────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'config', 'hw_slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        robot_state_publisher,
        slam_node,
        rviz_node,
    ])
