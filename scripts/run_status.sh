#!/usr/bin/env bash
# Vector Nav — Status Manager launcher.
#
# Starts status_manager_node + jetson_stats_node (vector_status_manager pkg).
# Runs as a separate service so it can restart independently of
# vector-navigation when RPi data (/battery, /system_stats/pi) is not
# yet available at boot.
#
# Logs: journalctl -u vector-status -f

source /opt/ros/humble/setup.bash
source /home/admin/vector_nav/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/admin/vector_nav/config/cyclonedds.xml
export HOME=/home/admin

exec ros2 launch vector_status_manager status_manager.launch.py
