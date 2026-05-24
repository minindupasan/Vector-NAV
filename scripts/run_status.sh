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

echo "Waiting for enP8p1s0 to get an IP address..."
timeout 30 bash -c 'until ip addr show enP8p1s0 2>/dev/null | grep -q "inet "; do sleep 0.5; done'
echo "enP8p1s0 is ready, starting status manager."

exec ros2 launch vector_status_manager status_manager.launch.py
