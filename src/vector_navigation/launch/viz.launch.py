"""Laptop-side visualization for VECTOR NAV.

Launches RViz2 to visualize the robot running on the Jetson.
All topics (/scan, /imu, /robot_description, /tf, etc.) are
discovered automatically via DDS multicast — just ensure both
machines are on the same network.

Usage (on laptop):
  ros2 launch vector_navigation viz.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('vector_navigation')
    rviz_config = os.path.join(pkg, 'rviz', 'vector.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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
        rviz,
    ])
