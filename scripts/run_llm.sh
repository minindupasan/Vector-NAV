#!/bin/bash
# run_llm.sh - Optimized Llama-3-8B loader for Vector Nav

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
echo "  VECTOR NAV — Llama-3-8B Optimizer"
echo "----------------------------------------------------"

if [ -z "$HUGGINGFACE_TOKEN" ]; then
    echo "ERROR: HF_TOKEN not found in .env.local"
    exit 1
fi

# 1. Maximize Jetson Performance
echo "[1/4] Setting Max Performance Mode (MAXN_SUPER) and Locking Clocks..."
sudo nvpmodel -m 2
sudo jetson_clocks

# 2. Cleanup old instances
echo "[2/4] Cleaning up old containers..."
sudo docker rm -f nano_llm_server 2>/dev/null || true

# 3. Launch NanoLLM (Llama-3-8B)
echo "[3/4] Starting NanoLLM Server (Llama-3-8B via mlc)..."
# jetson-containers run handles all nvidia-specific flags
# Using --dns 8.8.8.8 to ensure downloads don't hang
jetson-containers run \
  --name nano_llm_server \
  --detach \
  --dns 8.8.8.8 \
  --env HUGGINGFACE_TOKEN=$HUGGINGFACE_TOKEN \
  dustynv/nano_llm:r36.4.0 \
  python3 -m nano_llm.agents.web_chat \
    --model meta-llama/Meta-Llama-3-8B-Instruct \
    --api mlc \
    --quantization q4f16_ft \
    --max-context-len 2048

# 4. Success message and logs
echo "[4/4] Server is starting in the background."
echo "      Wait for: 'Uvicorn running on http://0.0.0.0:8050'"
echo "----------------------------------------------------"
echo "Following logs now (Press Ctrl+C to return to terminal):"
sudo docker logs -f nano_llm_server
