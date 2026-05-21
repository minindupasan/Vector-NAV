#!/bin/bash
# run_web.sh — Launcher for vector-web.service
# Sources ROS2 environment, venv, and starts the unified web dashboard.

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "--- Vector Web Launcher Debug ---"
echo "Workspace: $WORKSPACE_DIR"

# 1. Source ROS2 Humble
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Sourcing ROS2 Humble..."
    source /opt/ros/humble/setup.bash
else
    echo "ERROR: /opt/ros/humble/setup.bash not found" >&2
    exit 1
fi

# 2. Source Workspace install
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "Sourcing Workspace install..."
    source "$WORKSPACE_DIR/install/setup.bash"
fi

# 3. Add venv site-packages to PYTHONPATH if it exists
if [ -d "$WORKSPACE_DIR/.venv/lib/python3.10/site-packages" ]; then
    echo "Adding .venv to PYTHONPATH..."
    export PYTHONPATH="$WORKSPACE_DIR/.venv/lib/python3.10/site-packages:$PYTHONPATH"
fi

# 4. Add web app to PYTHONPATH
export PYTHONPATH="$WORKSPACE_DIR/src/vector_web:$PYTHONPATH"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Debug PYTHONPATH
echo "PYTHONPATH: $PYTHONPATH"

# Kill anything on port 8080
fuser -k 8080/tcp 2>/dev/null || true
sleep 1

echo "Starting VECTOR NAV Unified Web Dashboard..."
# Use python3 -m uvicorn to ensure we use the interpreter with our PYTHONPATH
exec python3 -m uvicorn vector_web.app:app \
    --host 0.0.0.0 \
    --port 8080 \
    --ssl-keyfile "$WORKSPACE_DIR/src/vector_web/certs/key.pem" \
    --ssl-certfile "$WORKSPACE_DIR/src/vector_web/certs/cert.pem" \
    --log-level info
