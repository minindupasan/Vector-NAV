#!/bin/bash
set -e

# Source ROS2 underlay + overlay
source /opt/ros/humble/setup.bash
source ${VECTOR_WS}/install/setup.bash

# Allow DDS discovery across the network (Jetson ↔ Pi)
# ROS_DOMAIN_ID is inherited from environment / compose file
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

echo "============================================"
echo " VECTOR NAV — Raspberry Pi Control Node"
echo " ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo " RMW           : ${RMW_IMPLEMENTATION}"
echo " FASTRTPS_PROFILE: ${FASTRTPS_DEFAULT_PROFILES_FILE:-default}"
echo "============================================"

exec "$@"
