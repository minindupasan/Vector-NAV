#!/bin/bash
# run_llm.sh - Llama-3.2-3B via NanoLLM

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
echo "  VECTOR NAV — Llama-3.2-3B"
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

# 3. Launch NanoLLM (Llama-3.2-3B)
echo "[3/4] Starting NanoLLM Server..."
/home/admin/jetson-containers/jetson-containers run \
  --name nano_llm_server \
  --detach \
  --dns 8.8.8.8 \
  --env HUGGINGFACE_TOKEN=$HUGGINGFACE_TOKEN \
  -v "$SCRIPT_DIR/fix_tied_embeddings.py:/tmp/fix_tied_embeddings.py:ro" \
  dustynv/nano_llm:r36.4.0 \
  bash -c "huggingface-cli download meta-llama/Llama-3.2-3B-Instruct \
      --local-dir-use-symlinks False && \
    python3 /tmp/fix_tied_embeddings.py meta-llama/Llama-3.2-3B-Instruct && \
    python3 -m nano_llm.agents.web_chat \
      --model meta-llama/Llama-3.2-3B-Instruct \
      --api mlc \
      --quantization q4f16_1 \
      --max-context-len 2048 \
      --max-new-tokens 150 \
      --temperature 0.6 \
      --top-p 0.9 \
      --repetition-penalty 1.1 \
      --system-prompt 'You are VECTOR NAV, a warehouse robot assistant. Always reply in 1 to 2 short plain-English sentences. Never use markdown, never use lists, never repeat yourself. Stop after one reply.'"

# 4. Success message and logs
echo "[4/4] Server is starting in the background."
echo "      Wait for: 'Uvicorn running on http://0.0.0.0:8050'"
echo "----------------------------------------------------"
echo "Following logs now (Press Ctrl+C to return to terminal):"
sudo docker logs -f nano_llm_server
