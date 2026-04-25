from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('framerate', default_value='30'),
        DeclareLaunchArgument('jpeg_quality', default_value='80'),

        Node(
            package='vector_camera',
            executable='camera_publisher',
            name='camera_publisher',
            parameters=[{
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'framerate': LaunchConfiguration('framerate'),
                'publish_compressed': True,
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                'camera_id': 0,
            }],
            output='screen',
        ),
    ])
