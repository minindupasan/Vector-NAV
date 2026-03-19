"""Launch the VECTOR NAV teleop GUI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vector_teleop',
            executable='teleop_gui',
            name='vector_teleop_gui',
            output='screen',
        ),
    ])
