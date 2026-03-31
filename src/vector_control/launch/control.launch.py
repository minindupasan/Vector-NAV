"""
Launch file for VECTOR NAV hardware control (Raspberry Pi).

Brings up:
  1. motor_driver   — /cmd_vel → motors, encoders → /wheel/odom
  2. imu_publisher  — MPU6500+HMC5883L → /imu (Madgwick filtered)
  3. rplidar        — RPLidar A1 M8 → /scan
  4. robot_localization EKF — fuses /wheel/odom + /imu → odom→base_link TF
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_control = get_package_share_directory('vector_control')

    control_params = os.path.join(pkg_control, 'config', 'control_params.yaml')
    ekf_params = os.path.join(pkg_control, 'config', 'ekf_hw.yaml')

    nodes = [
        # Motor driver & encoder odometry
        Node(
            package='vector_control',
            executable='motor_driver',
            name='motor_driver',
            parameters=[control_params],
            output='screen',
        ),

        # IMU publisher with Madgwick filter
        Node(
            package='vector_control',
            executable='imu_publisher',
            name='imu_publisher',
            parameters=[control_params],
            output='screen',
        ),

        # EKF: fuse wheel odom + IMU → filtered odom + TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_params],
            output='screen',
        ),
    ]

    # RPLidar — only include if sllidar_ros2 is installed
    try:
        get_package_share_directory('sllidar_ros2')
        nodes.append(
            Node(
                package='sllidar_ros2',
                executable='sllidar_node',
                name='rplidar_node',
                parameters=[{
                    'serial_port': '/dev/ttyUSB0',
                    'serial_baudrate': 115200,
                    'frame_id': 'lidar_link',
                    'angle_compensate': True,
                    'scan_mode': 'Boost',
                }],
                output='screen',
            )
        )
    except Exception:
        pass  # sllidar_ros2 not available in this environment

    return LaunchDescription(nodes)
