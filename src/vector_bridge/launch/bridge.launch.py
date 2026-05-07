"""Standalone launch for the vector_bridge web control dashboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # return LaunchDescription([
    #     DeclareLaunchArgument('port', default_value='8081',
    #                           description='HTTPS port for the control dashboard'),
    #     Node(
    #         package='vector_bridge',
    #         executable='bridge_node',
    #         name='vector_bridge',
    #         output='screen',
    #         parameters=[{'port': LaunchConfiguration('port')}],
    #     ),
    # ])
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8081',
                              description='Redundant — use vector-web.service'),
    ])
