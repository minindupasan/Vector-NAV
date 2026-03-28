# VECTOR NAV — Full Voice Pipeline Development Plan

## Context
Building a ROS2-based differential drive robot (VECTOR NAV) on Jetson Orin Nano Super 8GB.
The voice pipeline enables natural language navigation commands via:
**Mic → STT (faster-whisper) → LLM with function calling (Llama-3.2-3B-Instruct via NanoLLM) → RAG (FAISS) → ROS2 Nav2 → TTS (Piper)**

**Current system state:**
- JetPack 6.2.2 / L4T 36.5.0 ✅
- CUDA 12.6, TensorRT 10.3.0, cuDNN 9.3.0 ✅
- Python 3.10.12, NumPy 1.21.5 ✅
- 203 GB free disk space ✅
- ROS2 Humble: NOT installed ❌
- NanoLLM, PyTorch, faster-whisper, Piper, FAISS: NOT installed ❌

---

## Workspace Structure

```
~/vector_nav/                            # ROS2 workspace root
├── src/
│   ├── vector_nav_voice/                # Phase 2 — Voice pipeline
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── vector_nav_voice/
│   │       ├── __init__.py
│   │       ├── stt_node.py              # faster-whisper STT → publishes /stt/text
│   │       ├── llm_node.py              # Llama-3.2-3B-Instruct via NanoLLM, function calling
│   │       └── tts_node.py              # Piper TTS → audio output
│   ├── vector_nav_rag/                  # Phase 3 — RAG
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── vector_nav_rag/
│   │       ├── __init__.py
│   │       ├── rag_node.py              # FAISS retrieval service
│   │       └── knowledge_base/          # .txt/.json documents
│   ├── vector_nav_navigation/           # Phase 4 — Navigation
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── vector_nav_navigation/
│   │       ├── __init__.py
│   │       └── nav_bridge_node.py       # Translates LLM tool calls → Nav2 goals
│   └── vector_nav_vision/               # Phase 5 — Vision
│       ├── package.xml
│       ├── setup.py
│       └── vector_nav_vision/
│           ├── __init__.py
│           ├── apriltag_node.py         # AprilTag detection → docking
│           └── scene_node.py            # Semantic location labeling
├── models/
│   ├── whisper/                         # faster-whisper model cache
│   ├── tts/                             # Piper voice models (.onnx + .json)
│   └── rag/                             # sentence-transformers cache
├── config/
│   ├── tools.json                       # LLM function call tool definitions
│   ├── locations.json                   # Known nav locations (name → coordinates)
│   └── nav2_params.yaml                 # Nav2 stack config
├── scripts/
│   ├── install_deps.sh                  # Full dependency installer
│   └── verify_stack.sh                  # Health check all components
└── launch/
    ├── voice_pipeline.launch.py         # STT + LLM + TTS only
    ├── rag.launch.py                    # RAG service
    └── full_stack.launch.py             # Everything together
```

---

## Phase 1 — Environment Setup
**Goal:** Install all dependencies before writing any nodes.

### 1a. Install ROS2 Humble
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

### 1b. Launch NanoLLM + Llama-3.2-3B-Instruct
```bash
# See scripts/run_llm.sh for Docker container launch
bash ~/vector_nav/scripts/run_llm.sh
```

### 1c. Install Python AI Stack
```bash
# PyTorch for JetPack 6 (NVIDIA's Jetson wheel)
pip3 install --no-cache-dir torch torchvision torchaudio \
  --index-url https://download.pytorch.org/whl/cu126

# STT
pip3 install faster-whisper

# TTS
pip3 install piper-tts

# RAG stack
pip3 install faiss-cpu sentence-transformers

# Audio I/O
pip3 install sounddevice soundfile pyaudio

# ROS2 Python helpers
pip3 install colcon-common-extensions setuptools==67.6.0
```

### 1d. Download Piper Voice Model
```bash
mkdir -p ~/vector_nav/models/tts
cd ~/vector_nav/models/tts
# en_US-lessac-medium (natural, ~60MB)
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json
```

### 1e. Create Workspace
```bash
mkdir -p ~/vector_nav/src
cd ~/vector_nav
colcon build --symlink-install
source install/setup.bash
```

**Verify:** `ros2 --version` + NanoLLM WebSocket responds on `wss://localhost:49000` + `nvcc --version`

---

## Phase 2 — Voice Pipeline (Core)
**Goal:** Working end-to-end: Mic → STT → LLM → TTS

### Nodes to implement:

**`stt_node.py`**
- Uses `faster_whisper.WhisperModel("small", device="cuda", compute_type="int8")`
- Records audio via `sounddevice` with VAD (silence detection)
- Publishes transcribed text to `/stt/text` (std_msgs/String)
- Subscribes to `/tts/speaking` to pause recording while robot speaks (prevents echo)

**`llm_node.py`**
- Subscribes to `/stt/text`
- Connects to NanoLLM via WebSocket (`wss://localhost:49000`) with tool definitions loaded from `config/tools.json`
- Parses response: if tool call → publishes to `/llm/tool_call` (std_msgs/String JSON)
- If plain text → publishes to `/tts/input` (std_msgs/String)
- Calls RAG service before LLM if query seems knowledge-based

**`tts_node.py`**
- Subscribes to `/tts/input`
- Uses Piper to synthesize speech → plays via `sounddevice`
- Publishes `/tts/speaking` (std_msgs/Bool) to gate STT

### ROS2 Topics Map (Phase 2):
```
/stt/text          (std_msgs/String)   STT → LLM
/tts/input         (std_msgs/String)   LLM → TTS
/tts/speaking      (std_msgs/Bool)     TTS → STT (gate)
/llm/tool_call     (std_msgs/String)   LLM → NavBridge (JSON)
```

### Function calling tool definitions (`config/tools.json`):
```json
[
  {
    "name": "navigate_to",
    "description": "Navigate robot to a named location",
    "parameters": {
      "type": "object",
      "properties": {
        "location": {"type": "string", "description": "Location name e.g. 'Loading Bay', 'Isle 1'"},
        "use_apriltag": {"type": "boolean", "description": "Use AprilTag for precision docking"}
      },
      "required": ["location"]
    }
  },
  {
    "name": "stop_navigation",
    "description": "Stop the robot immediately"
  },
  {
    "name": "get_location",
    "description": "Report the robot's current location"
  },
  {
    "name": "dock",
    "description": "Precision dock using AprilTag at current location",
    "parameters": {
      "type": "object",
      "properties": {
        "apriltag_id": {"type": "integer"}
      },
      "required": ["apriltag_id"]
    }
  }
]
```

**Verify:** Launch voice pipeline, speak "go to the loading bay", confirm LLM publishes correct tool call JSON.

---

## Phase 3 — RAG Integration
**Goal:** Robot answers domain-specific questions (locations, procedures) from local knowledge.

**`rag_node.py`**
- Loads documents from `knowledge_base/` at startup
- Embeds with `sentence-transformers all-MiniLM-L6-v2` (~90MB)
- Builds FAISS index in memory
- Exposes ROS2 service `/rag/query` (custom srv: string query → string context)
- `llm_node.py` calls this service for non-navigation queries before LLM inference

**Knowledge base documents** (plain text files):
- `locations.txt` — descriptions of all named locations
- `procedures.txt` — docking procedures, safety rules
- `robot_info.txt` — robot capabilities and limitations

**Verify:** `ros2 service call /rag/query` with a test query, confirm relevant context returned.

---

## Phase 4 — Navigation Bridge
**Goal:** Translate LLM tool calls into actual Nav2 goals.

**`nav_bridge_node.py`**
- Subscribes to `/llm/tool_call`
- Parses JSON tool call
- Loads known locations from `config/locations.json` (name → x,y,yaw)
- Sends `NavigateToPose` action goal to Nav2
- Publishes navigation status back to `/tts/input` for spoken feedback

**`config/locations.json`** example:
```json
{
  "Loading Bay": {"x": 1.5, "y": 0.5, "yaw": 0.0, "apriltag_id": 1},
  "Isle 1":      {"x": 3.0, "y": 1.2, "yaw": 1.57, "apriltag_id": 2},
  "Charging Station": {"x": 0.1, "y": 0.1, "yaw": 3.14, "apriltag_id": 3}
}
```

**Verify:** Publish mock tool call JSON to `/llm/tool_call`, confirm Nav2 goal is sent.

---

## Phase 5 — Vision (AprilTag + Scene)
**Goal:** AprilTag-based precision docking and semantic location detection.

**`apriltag_node.py`**
- Uses `opencv-python` + `pupil_apriltags` library
- Subscribes to Raspberry Pi camera topic
- Detects AprilTag → computes relative pose
- Publishes `/apriltag/pose` for nav_bridge precision docking

**`scene_node.py`**
- Lightweight classification model (MobileNetV2 or YOLO-nano via TensorRT)
- Labels current scene → publishes `/scene/label` (e.g. "Loading Bay", "Corridor")
- Used to verify robot reached the correct location

---

## Phase 6 — Full Integration
**Goal:** All nodes running together, tested end-to-end.

**`full_stack.launch.py`** launches:
1. `stt_node` (cuda, small model)
2. `llm_node` (connects to NanoLLM)
3. `tts_node` (Piper)
4. `rag_node` (FAISS)
5. `nav_bridge_node`
6. `apriltag_node`
7. Nav2 bringup

**End-to-end test sequence:**
1. Say: *"Go to the Loading Bay"* → robot navigates
2. Say: *"Where is Isle 1?"* → RAG answers verbally
3. Say: *"Stop"* → robot halts
4. Say: *"Dock at Isle 2"* → AprilTag precision docking

---

## Memory Budget (Running Full Stack)

| Component | RAM |
|---|---|
| faster-whisper small (CUDA int8) | ~500 MB |
| Llama-3.2-3B-Instruct q4f16_ft via NanoLLM | ~1800 MB |
| FAISS + MiniLM-L6-v2 | ~400 MB |
| Piper TTS (CPU) | ~150 MB |
| OpenCV + AprilTag | ~300 MB |
| ROS2 Humble + Nav2 | ~500 MB |
| System + OS | ~500 MB |
| **Total** | **~4350 MB / 8192 MB** |

Safety margin: ~3.8 GB free for Nav2 costmaps, LIDAR, and spike headroom.

---

## Development Order (Recommended)

```
Phase 1 (Setup)     →  Phase 2 (Voice pipeline, no robot needed)
                    →  Phase 3 (RAG, test via CLI)
                    →  Phase 4 (Nav bridge, test with mock goals)
                    →  Phase 5 (Vision, test with camera)
                    →  Phase 6 (Full integration on robot)
```

Phases 2-3 can be fully developed and tested on the Jetson without the physical robot.

---

## Current Status

- [ ] Phase 1 — Environment Setup
- [ ] Phase 2 — Voice Pipeline
- [ ] Phase 3 — RAG Integration
- [ ] Phase 4 — Navigation Bridge
- [ ] Phase 5 — Vision
- [ ] Phase 6 — Full Integration
