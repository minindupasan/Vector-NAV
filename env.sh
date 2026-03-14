#!/bin/bash
# VECTOR NAV — workspace environment
# Source this instead of install/setup.bash:
#   source ~/vector_nav/env.sh

# ROS2 Humble
source /opt/ros/humble/setup.bash

# Workspace install
source "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash"

# uv venv — ML dependencies (faiss, sentence-transformers, torch, etc.)
# Prepended so uv packages take priority over stale system versions.
VENV_SITE="$(dirname "${BASH_SOURCE[0]}")/.venv/lib/python3.10/site-packages"
export PYTHONPATH="${VENV_SITE}:${PYTHONPATH}"
