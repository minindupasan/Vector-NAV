#!/bin/bash
set -e

# Source ROS2 underlay + overlay
source /opt/ros/humble/setup.bash
source ${VECTOR_WS}/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "============================================"
echo " VECTOR NAV — Raspberry Pi Control Node"
echo " ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo " RMW           : ${RMW_IMPLEMENTATION}"
echo "============================================"

exec "$@"
