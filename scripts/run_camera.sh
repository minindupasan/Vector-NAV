#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/admin/vector_nav/install/setup.bash
exec ros2 launch vector_camera camera.launch.py
