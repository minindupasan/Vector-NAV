"""Launch the VECTOR NAV teleop GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),

        Node(
            package='vector_teleop',
            executable='teleop_gui',
            name='vector_teleop_gui',
            output='screen',
            parameters=[{
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
            }],
        ),
    ])
