# VECTOR NAV — Complete Setup and Operations Guide

Intelligent warehouse robot assistant running on NVIDIA Jetson Orin Nano Super (8GB).
Fully offline voice pipeline: Speech-to-Text, LLM with function calling, RAG retrieval, and Text-to-Speech over ROS2 Humble.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Software Prerequisites](#software-prerequisites)
4. [Fresh Install — Step by Step](#fresh-install--step-by-step)
5. [Running the System](#running-the-system)
6. [Testing Commands](#testing-commands)
7. [Project Structure](#project-structure)
8. [ROS2 Topics and Messages](#ros2-topics-and-messages)
9. [Configuration](#configuration)
10. [Troubleshooting](#troubleshooting)
11. [Memory Budget](#memory-budget)

---

## Architecture Overview

```
                          +------------------+
    Microphone ──────────>|    stt_node      |──── /stt/text ────+
                          +------------------+                    |
                                                                  v
                          +------------------+          +------------------+
                          |    rag_node      |<─────── |    llm_node      |
                          |  (FAISS + MiniLM)|── /rag/ |  (NanoLLM WS)   |
                          +------------------+ context +--------+---------+
                                                        |                |
                                               /tts/input        /llm/tool_call
                                                        |                |
                                                        v                v
                                              +----------+    +------------------+
                                              | tts_node |    | nav_bridge_node  |
                                              | (Piper)  |    +------------------+
                                              +----------+
                                                   |
                                              Audio Output
                                           (Scarlett Solo USB)
```

**LLM Backend:** Llama-3.2-3B-Instruct running via NanoLLM (MLC quantized, q4f16_ft)
inside a Docker container. Exposes WebSocket on port 49000 and HTTP UI on port 8050.
Chat history resets after each query to keep memory stable.

**RAG Engine:** FAISS vector store with `all-MiniLM-L6-v2` embeddings (~90MB).
Knowledge base covers warehouse locations, procedures, and robot specifications.

**TTS Engine:** Piper TTS with `en_US-lessac-high` voice model (109MB ONNX, CPU).
Runs at ~10x real-time (RTF 0.08–0.10). Audio output via PulseAudio to Scarlett Solo USB.

**Communication:** ROS2 Humble with custom message types. LLM node connects to
NanoLLM via binary WebSocket protocol (wss://localhost:49000).

---

## Hardware Requirements

| Component | Specification |
|-----------|--------------|
| Board | NVIDIA Jetson Orin Nano Super |
| RAM | 8GB (shared CPU/GPU) |
| Storage | 256GB NVMe (minimum 60GB free) |
| JetPack | 6.2+ (L4T R36.4+) |
| CUDA | 12.6 |
| Audio Output | Focusrite Scarlett Solo USB (or any PulseAudio-compatible device) |

---

## Software Prerequisites

- Ubuntu 22.04 (Jetson default)
- JetPack 6.2+ with CUDA 12.6
- Docker (pre-installed with JetPack)
- ROS2 Humble
- Python 3.10

---

## Fresh Install — Step by Step

### Step 1: Install ROS2 Humble

```bash
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop ros-humble-nav2-bringup \
  ros-humble-navigation2 ros-dev-tools python3-colcon-common-extensions

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Clone the Repository

```bash
cd ~
git clone <your-repo-url> vector_nav
cd vector_nav
```

### Step 3: Set Up jetson-containers

```bash
git clone https://github.com/dusty-nv/jetson-containers.git
cd jetson-containers
pip3 install -r requirements.txt
cd ..
```

Verify the `jetson-containers` CLI is available:
```bash
jetson-containers run --help
```

### Step 4: Install Python Dependencies

```bash
# RAG dependencies
pip3 install faiss-cpu sentence-transformers

# LLM WebSocket client
pip3 install websockets

# TTS dependencies
pip3 install piper-tts sounddevice soundfile

# Safetensors (for Llama-3.2 embedding fix)
pip3 install safetensors
```

### Step 5: Download TTS Voice Model

```bash
mkdir -p ~/vector_nav/models/tts
cd ~/vector_nav/models/tts
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/high/en_US-lessac-high.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/high/en_US-lessac-high.onnx.json
```

### Step 6: Configure Audio Output

Set your USB audio interface as the default PulseAudio sink:

```bash
# List available sinks
pactl list sinks short

# Set default (adjust device name to match your hardware)
pactl set-default-sink alsa_output.usb-Focusrite_Scarlett_Solo_USB_Y74F8RR0956D7A-00.analog-stereo
```

### Step 7: Configure HuggingFace Token

Create `.env.local` in the project root:
```bash
echo "HF_TOKEN=hf_YOUR_TOKEN_HERE" > ~/vector_nav/.env.local
```

You need a HuggingFace token with read access for gated models (Llama-3.2).
Get one at: https://huggingface.co/settings/tokens

### Step 8: Add Swap Space

The MLC quantization step needs more than 8GB RAM. Add a 16GB disk swap:

```bash
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Step 9: Build ROS2 Workspace

```bash
cd ~/vector_nav
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Step 10: First-Time LLM Server Launch

The first run downloads Llama-3.2-3B (~6GB) and quantizes it (~10 min):

```bash
cd ~/vector_nav
./scripts/run_assistant.sh
```

Wait for this line in the logs:
```
WebChat - system ready
 * Running on https://0.0.0.0:8050
```

Press `Ctrl+C` to detach from logs (container keeps running).

**Note:** On the very first run, the `fix_tied_embeddings.py` script patches Llama-3.2's
tied embeddings (`lm_head.weight`) which the MLC version in `nano_llm:r36.4.0`
cannot handle natively. This fix runs automatically and only modifies files once.

---

## Running the System

### Quick Start (Single Command)

**Start the Assistant (LLM + Voice Pipeline):**
```bash
cd ~/vector_nav
./scripts/run_assistant.sh
```
*(Wait for "NanoLLM server is UP!", then the ROS2 pipeline will start automatically)*

**Start the Web Interface (Optional):**
```bash
cd ~/vector_nav
./scripts/run_web.sh
```

### With Custom Parameters

```bash
ros2 launch vector_llm vector_chatbot.launch.py \
  ws_url:=wss://localhost:49000 \
  rag_top_k:=5 \
  tts_volume:=1.5
```

### Stop Everything

```bash
# Stop ROS2 nodes
Ctrl+C in the launch terminal

# Stop LLM container
docker rm -f nano_llm_server
```

### Check LLM Container Status

```bash
docker ps                          # Running containers
docker logs nano_llm_server        # View logs
docker logs -f nano_llm_server     # Follow logs
```

---

## Testing Commands

### 1. Verify LLM Server is Running

```bash
# Check container
docker ps --filter name=nano_llm_server

# Check ports (49000=WebSocket, 8050=HTTP UI)
ss -tlnp | grep -E '8050|49000'

# Open web UI in browser
# https://<jetson-ip>:8050
```

### 2. Test ROS2 Topics (with launch running)

**Echo LLM text responses:**
```bash
ros2 topic echo /tts/input
```

**Echo tool calls (navigation commands):**
```bash
ros2 topic echo /llm/tool_call
```

**Echo RAG context:**
```bash
ros2 topic echo /rag/context
```

**Monitor TTS speaking state:**
```bash
ros2 topic echo /tts/speaking
```

### 3. Publish Test Messages

**Ask a knowledge question (triggers RAG + LLM + TTS):**
```bash
ros2 topic pub --once /stt/text vector_interfaces/msg/SttResult \
  "{text: 'Where is the loading dock?', confidence: 0.95, language: 'en'}"
```

**Send a navigation command (triggers tool call):**
```bash
ros2 topic pub --once /stt/text vector_interfaces/msg/SttResult \
  "{text: 'Navigate to Isle 3', confidence: 0.95, language: 'en'}"
```

**Ask about procedures:**
```bash
ros2 topic pub --once /stt/text vector_interfaces/msg/SttResult \
  "{text: 'What is the emergency stop procedure?', confidence: 0.90, language: 'en'}"
```

**Request a stop:**
```bash
ros2 topic pub --once /stt/text vector_interfaces/msg/SttResult \
  "{text: 'Stop the robot', confidence: 0.99, language: 'en'}"
```

**Ask about the robot:**
```bash
ros2 topic pub --once /stt/text vector_interfaces/msg/SttResult \
  "{text: 'What sensors does the robot have?', confidence: 0.85, language: 'en'}"
```

### 4. Test TTS Directly

```bash
# Send text straight to TTS (bypasses LLM)
ros2 topic pub --once /tts/input std_msgs/msg/String \
  "{data: 'Hello, I am Vector Nav, your warehouse assistant.'}"
```

### 5. Test Individual Nodes

**Test RAG node directly:**
```bash
ros2 topic pub --once /rag/query vector_interfaces/msg/RagQuery \
  "{text: 'battery charging procedure', top_k: 3}"

# In another terminal:
ros2 topic echo /rag/context
```

**Test TTS node standalone:**
```bash
source ~/vector_nav/env.sh
ros2 run vector_tts tts_node

# In another terminal:
ros2 topic pub --once /tts/input std_msgs/msg/String "{data: 'Testing audio output.'}"
```

### 6. Inspect Active Topics and Nodes

```bash
ros2 topic list              # All active topics
ros2 node list               # All active nodes
ros2 topic info /tts/input   # Publisher/subscriber info
ros2 topic hz /tts/input     # Message frequency
```

### 7. Test Without ROS2 (Standalone Pipeline)

```bash
source ~/vector_nav/env.sh
python3 ~/vector_nav/scripts/test_pipeline.py
```

### 8. Test RAG Engine Standalone (Interactive)

```bash
source ~/vector_nav/env.sh
python3 -m vector_rag.test_rag
```

---

## Project Structure

```
~/vector_nav/
├── config/
│   └── tools.json                    # LLM function calling definitions (4 tools)
├── models/
│   └── tts/
│       ├── en_US-lessac-high.onnx    # Piper voice model (109MB)
│       └── en_US-lessac-high.onnx.json
├── scripts/
│   ├── run_llm.sh                    # Start NanoLLM Docker container
│   ├── fix_tied_embeddings.py        # Patches Llama-3.2 tied weights for MLC
│   ├── test_pipeline.py              # Standalone integration test
│   └── install_cusparselt.sh         # NVIDIA cuSPARSELt installer
├── src/
│   ├── vector_interfaces/            # ROS2 custom messages (CMake)
│   │   └── msg/
│   │       ├── SttResult.msg         # Speech recognition output
│   │       ├── RagQuery.msg          # RAG retrieval request
│   │       ├── RagContext.msg        # RAG retrieval response
│   │       └── LLMToolCall.msg       # Function call from LLM
│   ├── vector_llm/                   # LLM communication node (Python)
│   │   ├── launch/
│   │   │   └── vector_chatbot.launch.py
│   │   └── vector_llm/
│   │       ├── llm_node.py           # ROS2 node: STT input → LLM → TTS/tool output
│   │       └── llm_engine.py         # WebSocket client for NanoLLM protocol
│   ├── vector_rag/                   # RAG retrieval node (Python)
│   │   └── vector_rag/
│   │       ├── rag_node.py           # ROS2 node: query → context
│   │       ├── rag_engine.py         # FAISS + sentence-transformers engine
│   │       ├── test_rag.py           # Interactive CLI test
│   │       ├── knowledge_base/       # Source documents
│   │       │   ├── locations.txt     # 8 warehouse locations with AprilTag IDs
│   │       │   ├── procedures.txt    # Operating procedures
│   │       │   └── robot_info.txt    # VECTOR NAV specifications
│   │       └── data/                 # Cached FAISS index (auto-generated)
│   │           ├── faiss.index
│   │           └── chunks.json
│   └── vector_tts/                   # TTS node (Python)
│       └── vector_tts/
│           └── tts_node.py           # ROS2 node: text → Piper TTS → audio output
├── jetson-containers/                # NVIDIA Jetson container toolkit
├── .env.local                        # HuggingFace token (not committed)
├── env.sh                            # Source this to set up workspace environment
└── docs/
    └── SETUP.md                      # This file
```

---

## ROS2 Topics and Messages

### Topics

| Topic | Message Type | Direction | Description |
|-------|-------------|-----------|-------------|
| `/stt/text` | `SttResult` | STT → LLM | Transcribed speech with confidence |
| `/rag/query` | `RagQuery` | LLM → RAG | Retrieval request |
| `/rag/context` | `RagContext` | RAG → LLM | Retrieved context for prompt |
| `/tts/input` | `std_msgs/String` | LLM → TTS | Text response for speech synthesis |
| `/tts/speaking` | `std_msgs/Bool` | TTS → STT | True while audio is playing (gates STT) |
| `/llm/tool_call` | `LLMToolCall` | LLM → Nav | Function call (navigate, stop, dock) |

### Custom Messages

**SttResult.msg**
```
std_msgs/Header header
string text                # Transcribed speech
float32 confidence         # 0.0–1.0, -1.0 if unavailable
string language            # Language code (e.g. 'en')
```

**RagQuery.msg**
```
string text                # Natural-language query
int32 top_k                # Chunks to retrieve (0 = use node default)
```

**RagContext.msg**
```
string query               # Original query echoed back
string context             # Formatted context ready for LLM prompt
string[] sources           # Knowledge base filenames that contributed
```

**LLMToolCall.msg**
```
string name                # Tool name (navigate_to, stop_navigation, get_location, dock)
string arguments_json      # JSON-encoded arguments
```

---

## Configuration

### tools.json — LLM Function Calling

Located at `config/tools.json`. Defines 4 navigation tools:

| Tool | Parameters | Description |
|------|-----------|-------------|
| `navigate_to` | `location` (string, required), `use_apriltag` (bool, optional) | Navigate to warehouse location |
| `stop_navigation` | none | Halt immediately |
| `get_location` | none | Report current position |
| `dock` | `apriltag_id` (int, required) | Precision dock using AprilTag |

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ws_url` | `wss://localhost:49000` | NanoLLM WebSocket endpoint |
| `rag_top_k` | `3` | Number of RAG chunks to retrieve |
| `tools_file` | `''` (auto-find) | Path to tools.json |
| `tts_model` | `.../en_US-lessac-high.onnx` | Piper voice model path |
| `tts_device` | `''` (system default) | Audio output device |
| `tts_volume` | `1.0` | Volume multiplier (0.0–2.0) |

### TTS Configuration

| Setting | Value |
|---------|-------|
| Voice model | `en_US-lessac-high` (Lessac, high quality) |
| Sample rate | 22,050 Hz |
| Speech rate | 0.85 (slightly faster than default) |
| Backend | Piper TTS (ONNX, CPU) |
| Audio routing | PulseAudio → Scarlett Solo USB |

### LLM Server Configuration (run_llm.sh)

| Setting | Value |
|---------|-------|
| Model | `meta-llama/Llama-3.2-3B-Instruct` |
| Backend | MLC (Machine Learning Compilation) |
| Quantization | `q4f16_ft` (4-bit float16 fine-tuned) |
| Max context | 2048 tokens |
| Container | `dustynv/nano_llm:r36.4.0` |
| Jetson power mode | MAXN_SUPER (mode 2) |

---

## Troubleshooting

### LLM container exits immediately

**Check logs:**
```bash
docker logs nano_llm_server
```

**Common causes:**

1. **`KeyError: 'lm_head.weight'`** — The tied embeddings fix didn't run.
   Verify `fix_tied_embeddings.py` is mounted in `run_llm.sh`.

2. **`SIGKILL` during quantization** — Out of memory. Ensure 16GB swap is active:
   ```bash
   free -h    # Should show ~16GB swap
   swapon --show
   ```

3. **HuggingFace token invalid** — Check `.env.local` has a valid `HF_TOKEN`
   with read access to `meta-llama/Llama-3.2-3B-Instruct`.

### ROS2 nodes can't connect to LLM

```bash
# Verify ports are listening
ss -tlnp | grep 49000

# If not, restart the container
docker rm -f nano_llm_server
./scripts/run_assistant.sh
```

### No audio output from TTS

```bash
# Check PulseAudio sinks
pactl list sinks short

# Set correct default sink
pactl set-default-sink <your-sink-name>

# Test audio directly
python3 -c "
import numpy as np, sounddevice as sd
tone = (np.sin(2*np.pi*440*np.linspace(0,0.5,11025,False))*0.3).astype(np.float32)
sd.play(tone, samplerate=22050, blocking=True)
"
```

### LLM gives repetitive or off-topic responses

This usually means chat history accumulated. The engine now resets history after
each query automatically. If it persists:

```bash
# Restart the LLM container to clear all state
docker rm -f nano_llm_server
./scripts/run_assistant.sh
```

### PackageNotFoundError on launch

Clean rebuild the affected package:

```bash
cd ~/vector_nav
source /opt/ros/humble/setup.bash
rm -rf build/<package_name> install/<package_name>
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```

### RAG returns empty context

```bash
# Force rebuild of FAISS index
rm -f src/vector_rag/vector_rag/data/faiss.index
rm -f src/vector_rag/vector_rag/data/chunks.json
# Relaunch — index rebuilds automatically
```

### Check memory usage

```bash
free -h                           # System RAM
docker stats nano_llm_server      # Container memory
ros2 node list                    # Verify all nodes are running
```

---

## Memory Budget

Measured at steady state with full pipeline running (RAG + LLM + TTS):

| Component | RAM Usage |
|-----------|----------|
| Llama-3.2-3B (MLC q4f16_ft) | ~1.8 GB |
| NanoLLM runtime + CUDA | ~1.2 GB |
| ROS2 Humble + nodes | ~0.5 GB |
| FAISS + MiniLM-L6-v2 embeddings | ~0.4 GB |
| Piper TTS (CPU, lessac-high) | ~0.15 GB |
| System / OS | ~0.3 GB |
| **Total** | **~4.35 GB / 7.6 GB** |

Free headroom: ~3.25 GB for Nav2 costmaps, LIDAR processing, and additional nodes.

---

## Ports Reference

| Port | Protocol | Service |
|------|----------|---------|
| 8050 | HTTPS | NanoLLM Web Chat UI |
| 49000 | WSS | NanoLLM WebSocket API (LLM node connects here) |

---

## Environment Setup Shortcut

Instead of sourcing multiple files, use the provided helper:

```bash
source ~/vector_nav/env.sh
```

This sources ROS2 Humble, the workspace install, and adds the Python venv to PYTHONPATH.
