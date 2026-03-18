"""Combined Gazebo + RViz2 launcher for VECTOR NAV.

Starts Ignition Fortress simulation and RViz2 together.

Usage:
  ros2 launch vector_description sim.launch.py
  ros2 launch vector_description sim.launch.py x_pose:=1.0 y_pose:=2.0
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
    pkg = get_package_share_directory('vector_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # ── Environment for Jetson NVIDIA EGL + Ignition mesh resolution ─────────
    os.environ.setdefault('__EGL_VENDOR_LIBRARY_DIRS', '/usr/share/glvnd/egl_vendor.d/')
    os.environ.setdefault('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')

    pkg_share_parent = os.path.join(get_package_prefix('vector_description'), 'share')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = (
        pkg_share_parent
        + (':' + os.environ['IGN_GAZEBO_RESOURCE_PATH']
           if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ else '')
    )

    xacro_file  = os.path.join(pkg, 'urdf', 'vector_urdf.xacro')
    world_file  = os.path.join(pkg, 'worlds', 'vector_world.sdf')
    rviz_config = os.path.join(pkg, 'rviz', 'vector.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ── robot_state_publisher (starts FIRST so gz_ros2_control can reach it) ──
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

    # ── Ignition Gazebo (delayed 2s to let RSP advertise its service) ─────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
    )

    # ── Spawn robot (delayed 5s — Gazebo needs time to start on Jetson) ───────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'vector_urdf',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.1',
        ],
    )

    # ── Controller spawners (robust — wait up to 120s for controller_manager) ─
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )

    spawn_ddc = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller',
                   '--controller-manager-timeout', '120'],
        output='screen',
    )

    # ── Sensor bridges (Ignition → ROS2) ─────────────────────────────────────
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        ],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),

        # 1. RSP starts immediately
        robot_state_publisher,

        # 2. Gazebo starts after 2s (RSP is ready)
        TimerAction(period=2.0, actions=[gz_sim]),

        # 3. Spawn robot after Gazebo world is loaded (5s)
        TimerAction(period=5.0, actions=[spawn_entity]),

        # 4. Sensor bridge + RViz
        sensor_bridge,
        rviz,

        # 5. Controller spawners (wait for controller_manager automatically)
        TimerAction(period=8.0, actions=[spawn_jsb]),
        TimerAction(period=10.0, actions=[spawn_ddc]),
    ])
