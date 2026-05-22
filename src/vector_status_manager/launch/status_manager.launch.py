"""
VECTOR STATUS MANAGER — standalone launch.

Brings up the status_manager_node by itself. The same node is also referenced
from the canonical bringup (vector_nav_manager/launch/manager.launch.py).
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

    return LaunchDescription([status_manager])
