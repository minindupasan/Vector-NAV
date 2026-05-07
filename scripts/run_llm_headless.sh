#!/bin/bash
# run_llm_headless.sh — NanoLLM launcher for systemd (no interactive log tail).
# Token is injected via EnvironmentFile in vector-llm.service.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/../.env.local"

if [ -z "$HUGGINGFACE_TOKEN" ] && [ -f "$ENV_FILE" ]; then
    source "$ENV_FILE"
    export HUGGINGFACE_TOKEN="${HUGGINGFACE_TOKEN:-${HF_TOKEN}}"
fi

if [ -z "$HUGGINGFACE_TOKEN" ]; then
    echo "ERROR: HUGGINGFACE_TOKEN not set (check /home/admin/vector_nav/.env.local)" >&2
    exit 1
fi

# Maximise Jetson performance
sudo nvpmodel -m 0
sudo jetson_clocks

# Stop any leftover nano_llm containers regardless of name
sudo docker rm -f nano_llm_server 2>/dev/null || true
sudo docker ps -q --filter ancestor=dustynv/nano_llm:r36.4.0 | xargs -r sudo docker stop 2>/dev/null || true

# Launch detached container — systemd tracks it via docker ps (Type=forking)
/home/admin/jetson-containers/jetson-containers run \
  --name nano_llm_server \
  --detach \
  --dns 8.8.8.8 \
  --env HUGGINGFACE_TOKEN="$HUGGINGFACE_TOKEN" \
  -v "$SCRIPT_DIR/fix_tied_embeddings.py:/tmp/fix_tied_embeddings.py:ro" \
  dustynv/nano_llm:r36.4.0 \
  bash -c "python3 /tmp/fix_tied_embeddings.py meta-llama/Llama-3.2-3B-Instruct && \
    python3 -m nano_llm.agents.web_chat \
      --model meta-llama/Llama-3.2-3B-Instruct \
      --api mlc \
      --quantization q4f16_ft \
      --max-context-len 1024 \
      --repetition-penalty 1.5 \
      --temperature 0.6 \
      --chat-template llama-3"

echo "nano_llm_server container started (detached)"
