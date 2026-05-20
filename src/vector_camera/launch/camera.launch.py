from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vector_camera',
            executable='camera_node',
            name='imx708_camera',
            output='screen',
            parameters=[{
                'sensor_mode': 1,
                'capture_width': 2304,
                'capture_height': 1296,
                'framerate': 56,
                'jpeg_quality': 85,
            }],
        ),
        Node(
            package='vector_camera',
            executable='webrtc_node',
            name='camera_webrtc',
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': 8443,
                'cert_file': '/home/admin/src/vector_web/certs/cert.pem',
                'key_file': '/home/admin/src/vector_web/certs/key.pem',
                'framerate': 56,
                'static_dir': '/home/admin/src/vector_web/static',
            }],
        ),
    ])
