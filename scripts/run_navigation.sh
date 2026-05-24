#!/usr/bin/env bash
# Vector Nav — Supervisor launcher.
#
# Starts the manager.launch.py from vector_nav_manager, which brings up:
#   - nav_manager_node       (locations + Nav2 goal bridge, /nav/state)
#   - mode_manager_node      (NAV ↔ SLAM lifecycle; spawns hw_nav / hw_slam)
#   - map_manager_node       (save / list maps)
#
# status_manager_node and jetson_stats_node are in vector-status.service.
#
# mode_manager_node persists state to ~/.vector_nav/state.yaml and spawns the
# appropriate hw_nav.launch.py / hw_slam.launch.py child on startup. All output
# (mode switches, child pids, save_map results) lands in this service's journal:
#   journalctl -u vector-navigation -f

source /opt/ros/humble/setup.bash
source /home/admin/vector_nav/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/admin/vector_nav/config/cyclonedds.xml
export HOME=/home/admin

# Wait for enP8p1s0 to have an IP before starting ROS2 nodes.
# Without this, mode_manager_node and map_manager_node fail at boot because
# they import fewer modules and reach rclpy.init() before the interface is ready.
echo "Waiting for enP8p1s0 to get an IP address..."
timeout 30 bash -c 'until ip addr show enP8p1s0 2>/dev/null | grep -q "inet "; do sleep 0.5; done'
echo "enP8p1s0 is ready, starting navigation stack."

exec ros2 launch vector_nav_manager manager.launch.py
