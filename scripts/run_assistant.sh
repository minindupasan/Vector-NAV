#!/bin/bash
# run_assistant.sh - Starts both NanoLLM and the ROS2 Voice Pipeline

# --- Load token from .env.local ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/../.env.local"

if [ -f "$ENV_FILE" ]; then
    source "$ENV_FILE"
    export HUGGINGFACE_TOKEN="${HF_TOKEN}"
else
    echo "ERROR: .env.local not found at $ENV_FILE"
    exit 1
fi
# ----------------------------------

echo "----------------------------------------------------"
echo "  VECTOR NAV — Assistant (LLM + Voice Pipeline)"
echo "----------------------------------------------------"

if [ -z "$HUGGINGFACE_TOKEN" ]; then
    echo "ERROR: HF_TOKEN not found in .env.local"
    exit 1
fi

# 1. Maximize Jetson Performance
echo "[1/4] Setting Max Performance Mode (MAXN_SUPER) and Locking Clocks..."
sudo nvpmodel -m 0
sudo jetson_clocks

# 2. Cleanup old instances
echo "[2/4] Stopping old containers..."
sudo docker rm -f nano_llm_server 2>/dev/null || true

# 3. Launch NanoLLM (Llama-3.2-3B) in background
echo "[3/4] Starting NanoLLM Server in background..."
# We use --detach to run in background
/home/admin/jetson-containers/jetson-containers run \
  --name nano_llm_server \
  --detach \
  --dns 8.8.8.8 \
  --env HUGGINGFACE_TOKEN=$HUGGINGFACE_TOKEN \
  -v "$SCRIPT_DIR/fix_tied_embeddings.py:/tmp/fix_tied_embeddings.py:ro" \
  dustynv/nano_llm:r36.4.0 \
  bash -c "(huggingface-cli download meta-llama/Llama-3.2-3B-Instruct --local-dir-use-symlinks False || echo 'Warning: Hugging Face download failed, attempting to run with cached model...') && \
    python3 /tmp/fix_tied_embeddings.py meta-llama/Llama-3.2-3B-Instruct && \
    python3 -m nano_llm.agents.web_chat \
      --model meta-llama/Llama-3.2-3B-Instruct \
      --api mlc \
      --quantization q4f16_1 \
      --max-context-len 1024 \
      --max-new-tokens 150 \
      --temperature 0.6 \
      --top-p 0.9 \
      --repetition-penalty 1.1 \
      --chat-template llama-3 \
      --system-prompt 'You are VECTOR NAV, an intelligent warehouse robot assistant. You have full conversational abilities and should answer questions directly and naturally. Be concise but helpful. You have access to physical navigation tools, but ONLY call a tool if the user explicitly commands you to move, dock, or stop. If they are just chatting, answering a question, or asking about you, just reply conversationally without tools.'"

# Wait for the LLM server to be ready (port 49000)
echo "Waiting for NanoLLM server to start on port 49000..."
MAX_RETRIES=60
RETRY_COUNT=0
while ! nc -z localhost 49000; do
  sleep 5
  RETRY_COUNT=$((RETRY_COUNT+1))
  if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
    echo "ERROR: NanoLLM server failed to start within timeout."
    exit 1
  fi
  echo "Still waiting... ($RETRY_COUNT/$MAX_RETRIES)"
done
echo "NanoLLM server is UP!"

# Function to stop the container on exit (for manual runs)
cleanup() {
    echo "Stopping NanoLLM server..."
    docker stop nano_llm_server >/dev/null 2>&1
}
trap cleanup EXIT

# 4. Launch ROS2 Assistant Pipeline
echo "[4/4] Starting ROS2 Assistant Pipeline..."
source "$SCRIPT_DIR/../env.sh"

# Start the chatbot launch file
ros2 launch vector_llm vector_chatbot.launch.py
