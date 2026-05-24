"""
VECTOR STATUS MANAGER — standalone launch.

Brings up status_manager_node and jetson_stats_node as a standalone pair,
decoupled from vector-navigation so the service can restart independently
when RPi data is not yet available at boot.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    status_manager = Node(
        package='vector_status_manager',
        executable='status_manager_node',
        name='status_manager_node',
        output='screen',
        emulate_tty=True,
    )

    jetson_stats = Node(
        package='vector_status_manager',
        executable='jetson_stats_node',
        name='jetson_stats_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([status_manager, jetson_stats])
