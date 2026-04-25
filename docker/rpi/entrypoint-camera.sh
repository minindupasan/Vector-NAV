#!/bin/bash
set -e

# Source ROS2 underlay + overlay
source /opt/ros/humble/setup.bash
if [ -f ${VECTOR_WS}/install/setup.bash ]; then
  source ${VECTOR_WS}/install/setup.bash
fi

# DDS config
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

echo "============================================"
echo " VECTOR NAV — Raspberry Pi Camera Node"
echo " ROS_DOMAIN_ID : ${ROS_DOMAIN_ID}"
echo " RMW           : ${RMW_IMPLEMENTATION}"
echo "============================================"

exec "$@"
