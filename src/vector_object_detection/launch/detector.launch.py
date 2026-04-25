#!/usr/bin/env python3
"""Launch YOLO detector node"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('yolo_detector')
    config_file = os.path.join(pkg_share, 'config', 'yolo_detector.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=config_file,
            description='Path to YOLO detector config file'
        ),
        
        Node(
            package='yolo_detector',
            executable='detector',
            name='yolo_detector',
            output='screen',
            parameters=[LaunchConfiguration('config')],
            remappings=[
                ('detections', '/yolo/detections'),
                ('image_raw', '/yolo/image_raw'),
                ('image_annotated', '/yolo/image_annotated'),
            ]
        )
    ])
