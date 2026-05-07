"""VECTOR NAV — Web Bridge + Navigation Manager Launch
=====================================================
Brings up the nodes required for the web dashboard and waypoint management.
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    default_locs = str(Path.home() / 'vector_nav' / 'config' / 'locations.yaml')

    arg_locations = DeclareLaunchArgument(
        'locations_file', default_value=default_locs,
        description='Path to locations.yaml for the waypoint manager'
    )

    # ── Vector Nav Manager — named locations + LLM bridge ────────────────────
    node_nav_manager = Node(
        package='vector_nav_manager',
        executable='nav_manager',
        name='nav_manager',
        output='screen',
        parameters=[{'locations_file': LaunchConfiguration('locations_file')}],
    )

    return LaunchDescription([
        arg_locations,
        node_nav_manager,
    ])
