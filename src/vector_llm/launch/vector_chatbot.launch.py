"""
VECTOR NAV — Full Voice Pipeline Launch File
Starts STT, LLM, and TTS nodes.

Pipeline: Mic → STT → LLM → TTS → Speaker

Usage:
  ros2 launch vector_llm vector_chatbot.launch.py
  ros2 launch vector_llm vector_chatbot.launch.py ws_url:=wss://localhost:49000
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    os.system('pkill -f "llm_node|tts_node|stt_node" 2>/dev/null || true')

    return LaunchDescription([

        # ── Arguments ─────────────────────────────────────────────────────────
        DeclareLaunchArgument('ws_url',      default_value='wss://localhost:49000'),
        DeclareLaunchArgument('tools_file',  default_value=''),
        DeclareLaunchArgument('tts_voice',   default_value='af_heart'),
        DeclareLaunchArgument('tts_speed',   default_value='1.0'),
        DeclareLaunchArgument('tts_device',  default_value='pulse'),
        DeclareLaunchArgument('tts_volume',  default_value='2.0'),
        DeclareLaunchArgument('stt_model',   default_value='base.en'),

        LogInfo(msg='Starting VECTOR NAV — LLM + TTS nodes'),

        # ── LLM node ──────────────────────────────────────────────────────────
        Node(
            package='vector_llm',
            executable='llm_node',
            name='llm_node',
            output='screen',
            parameters=[{
                'ws_url':     LaunchConfiguration('ws_url'),
                'tools_file': LaunchConfiguration('tools_file'),
            }],
        ),

        # ── TTS node — Kokoro TTS (TensorRT GPU) ──────────────────────────────
        Node(
            package='vector_tts',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'voice':  LaunchConfiguration('tts_voice'),
                'speed':  LaunchConfiguration('tts_speed'),
                'device': LaunchConfiguration('tts_device'),
                'volume': LaunchConfiguration('tts_volume'),
                'use_trt': True,
            }],
        ),

        # ── System Stats node — Jetson metrics (CPU, GPU, Temp) ───────────────
        Node(
            package='vector_bridge',
            executable='jetson_stats_node',
            name='jetson_stats_node',
            output='screen',
        ),

    ])
