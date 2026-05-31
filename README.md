# Vector Nav

<p align="center">
  <img src="src/vector_web/vector_web/static/logo.svg" alt="Vector Nav Logo" width="400"><br>
Autonomous mobile robot · Offline voice control · ROS2 Humble<br>
NVIDIA Jetson Orin Nano Super + Raspberry Pi 5
</p>

---

## What It Does
Vector Nav is a differential-drive warehouse assistant robot. You speak to it, it understands and acts:

| You say | What happens |
| :--- | :--- |
| *"Hey Jarvis, go to the shop"* | Robot autonomously navigates using Nav2 |
| *"Stop"* | Navigation cancelled immediately |
| *"Where am I?"* | Robot reports its current named location |
| *"Navigate to base"* | Robot plans and drives to the saved waypoint |

> **Note:** The entire AI pipeline runs **offline** — no cloud services, no internet connection required during operation.

---

## Key Capabilities
| Capability | Technology |
| :--- | :--- |
| Wake word detection | **openwakeword** (TFLite, always-on ~10 ms/frame) |
| Speech recognition | **faster-whisper** (CPU int8, `base.en` model) |
| Language model | **Llama-3.2-1B-Instruct** via NanoLLM (MLC q4f16_1) |
| Voice synthesis | **Kokoro 82M** (TensorRT FP16, Jetson GPU, 24 kHz) |
| Autonomous navigation | **Nav2** (DWB controller, NavFn planner, AMCL) |
| Mapping | **SLAM Toolbox** (async, online SLAM) |
| Sensor fusion | **robot_localization EKF** (wheel odometry + IMU) |
| Object detection | **YOLO11s NCNN** (320×320, 80 COCO classes) |
| Web dashboard | **FastAPI + WebSocket** (HTTPS port 8080) |
| Barge-in | Interrupt LLM/TTS mid-response with a new command |

---

## Hardware at a Glance
```text
┌─────────────────────────────────────────────────────────────────┐
│            JETSON ORIN NANO SUPER 8GB                           │
│  LLM (NanoLLM)   TTS (Kokoro TRT)   Nav2    Web Dashboard       │
│  STT (Whisper)   Status Manager     SLAM    Object Detection    │
│           CycloneDDS over LAN · ROS_DOMAIN_ID=0                 │
└────────────────────────────────┬────────────────────────────────┘
                                 │
┌────────────────────────────────▼────────────────────────────────┐
│                 RASPBERRY PI 5                                  │
│  4× DC Motors    4× Encoders        MPU6500 + HMC5883L IMU      │
│  TB6612FNG drivers   ADS1115 battery ADC                        │
│  SLLiDAR A1M8    IMX708 Camera      EKF (robot_localization)    │
│                 (Docker containers)                             │
└─────────────────────────────────────────────────────────────────┘
```

---

## ROS2 Package Overview
| Package | Runs On | Description |
| :--- | :--- | :--- |
| `vector_interfaces` | Both | Custom messages and services (CMake) |
| `vector_control` | Pi | Motor driver, IMU, EKF, system stats |
| `vector_navigation` | Jetson | Nav2, SLAM Toolbox, simulation launches |
| `vector_nav_manager` | Jetson | Nav manager, map manager, mode manager |
| `vector_status_manager` | Jetson | Aggregates all subsystem states → `/robot_status` |
| `vector_llm` | Jetson | LLM node (NanoLLM WebSocket client) |
| `vector_stt` | Jetson | Wake word + Whisper STT node |
| `vector_tts` | Jetson | Kokoro TRT text-to-speech node |
| `vector_detection` | Jetson | YOLO11s object detection node |
| `vector_teleop` | Any | Qt5 teleoperation GUI |
| `vector_web` | Jetson | FastAPI web dashboard + ROS2 bridge |
