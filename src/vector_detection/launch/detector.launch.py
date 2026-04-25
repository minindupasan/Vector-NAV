from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vector_detection',
            executable='detector',
            name='vector_detector',
            output='screen',
            remappings=[
                ('detections', '/yolo/detections'),
                ('image_raw', '/yolo/image_raw'),
                ('image_annotated', '/yolo/image_annotated'),
            ]
        )
    ])
