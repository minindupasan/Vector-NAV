#!/bin/bash
# run_assistant.sh - Starts the ROS2 Voice Pipeline
# Assumes NanoLLM server is being started by vector-llm.service

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "----------------------------------------------------"
echo "  VECTOR NAV — Assistant (Voice Pipeline)"
echo "----------------------------------------------------"

# 1. Wait for the LLM server to be ready (port 49000)
# This handles the "sequence" by ensuring ROS2 doesn't start until LLM is UP.
echo "Waiting for NanoLLM server to start on port 49000..."
MAX_RETRIES=120
RETRY_COUNT=0
while ! nc -z localhost 49000; do
  # Proactively ensure the LLM service is running
  if ! systemctl is-active --quiet vector-llm; then
    echo "Warning: vector-llm service is not active. Attempting to start it..."
    sudo systemctl start vector-llm
  fi

  sleep 5
  RETRY_COUNT=$((RETRY_COUNT+1))
  if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
    echo "ERROR: NanoLLM server failed to start within timeout."
    exit 1
  fi
  echo "Still waiting for LLM server... ($RETRY_COUNT/$MAX_RETRIES)"
done
echo "NanoLLM server is UP!"

# 2. Launch ROS2 Assistant Pipeline
echo "[2/2] Starting ROS2 Assistant Pipeline..."
source "$SCRIPT_DIR/../env.sh"

# Start the chatbot launch file
ros2 launch vector_llm vector_chatbot.launch.py
