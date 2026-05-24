"""
VECTOR NAV — Supervisor launch.

Brings up the long-lived manager nodes:
  - nav_manager_node     (locations + Nav2 goal bridge, /nav/state)
  - mode_manager_node    (NAV ↔ SLAM lifecycle, spawns hw_nav / hw_slam)
  - map_manager_node     (save_map / list_maps services)

status_manager_node and jetson_stats_node are managed by the separate
vector-status.service so they can restart independently of navigation
(RPi data may not be available at boot time).

The hw_nav / hw_slam launches are *not* started here — mode_manager_node
spawns the appropriate one based on persisted state at startup.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    nav_manager = Node(
        package='vector_nav_manager',
        executable='nav_manager_node',
        name='nav_manager_node',
        output='screen',
        emulate_tty=True,
    )

    mode_manager = Node(
        package='vector_nav_manager',
        executable='mode_manager_node',
        name='mode_manager_node',
        output='screen',
        emulate_tty=True,
    )

    map_manager = Node(
        package='vector_nav_manager',
        executable='map_manager_node',
        name='map_manager_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([nav_manager, mode_manager, map_manager])
