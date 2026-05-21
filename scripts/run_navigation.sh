#!/usr/bin/env bash
# Vector Nav — Supervisor launcher.
#
# Starts the manager.launch.py from vector_nav_manager, which brings up:
#   - nav_manager_node     (locations + Nav2 goal bridge)
#   - mode_manager_node    (NAV ↔ SLAM lifecycle; spawns hw_nav / hw_slam)
#   - map_manager_node     (save / list maps)
#
# mode_manager_node persists state to ~/.vector_nav/state.yaml and spawns the
# appropriate hw_nav.launch.py / hw_slam.launch.py child on startup. All output
# (mode switches, child pids, save_map results) lands in this service's journal:
#   journalctl -u vector-navigation -f

source /opt/ros/humble/setup.bash
source /home/admin/vector_nav/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export HOME=/home/admin

exec ros2 launch vector_nav_manager manager.launch.py
