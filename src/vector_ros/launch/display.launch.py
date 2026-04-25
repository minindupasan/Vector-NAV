"""RViz2 launcher for VECTOR NAV.

Two usage modes:
  1. Standalone (URDF preview, no Gazebo):
       ros2 launch vector_ros display.launch.py

  2. Alongside a running Gazebo sim (live sensor/odom data):
       ros2 launch vector_ros display.launch.py use_sim_time:=true
     In this mode robot_state_publisher and joint_state_publisher are skipped
     because Gazebo already provides those topics.
"""

import os
import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('vector_ros')
    xacro_file  = os.path.join(pkg, 'urdf', 'vector_urdf.xacro')
    rviz_config = os.path.join(pkg, 'rviz', 'vector.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui      = LaunchConfiguration('gui',          default='true')

    # True when NOT in sim mode — used to conditionally start standalone nodes
    not_sim = PythonExpression(["'", use_sim_time, "' != 'true'"])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Set to true when a Gazebo sim is already running'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start joint_state_publisher_gui in standalone mode'),

        # ── Standalone only: publish robot state and allow joint dragging ─────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
            condition=launch.conditions.UnlessCondition(use_sim_time),
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=launch.conditions.IfCondition(
                PythonExpression([
                    "'", use_sim_time, "' != 'true' and '", use_gui, "' == 'true'"
                ])
            ),
        ),

        # ── RViz2 (always started) ────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
