"""
VECTOR NAV — Full Voice Pipeline Launch File
Starts STT, RAG, LLM, and TTS nodes.

Pipeline: Mic → STT → LLM (+ RAG) → TTS → Speaker

Usage:
  ros2 launch vector_llm vector_chatbot.launch.py
  ros2 launch vector_llm vector_chatbot.launch.py ws_url:=wss://localhost:49000 rag_top_k:=5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Arguments ─────────────────────────────────────────────────────────
        DeclareLaunchArgument('ws_url',      default_value='wss://localhost:49000'),
        DeclareLaunchArgument('rag_top_k',   default_value='2'),
        DeclareLaunchArgument('tools_file',  default_value=''),
        DeclareLaunchArgument('tts_model',   default_value='/home/jetson/vector_nav/models/tts/en_US-ryan-high.onnx'),
        DeclareLaunchArgument('tts_device',  default_value=''),
        DeclareLaunchArgument('tts_volume',  default_value='1.0'),
        DeclareLaunchArgument('stt_model',   default_value='base.en'),

        LogInfo(msg='Starting VECTOR NAV — STT + RAG + LLM + TTS nodes'),

        # ── STT node — faster-whisper (CPU, tiny.en int8) ────────────────────
        Node(
            package='vector_stt',
            executable='stt_node',
            name='stt_node',
            output='screen',
            parameters=[{
                'model_size': LaunchConfiguration('stt_model'),
            }],
        ),

        # ── RAG node — owns the embedding model ───────────────────────────────
        Node(
            package='vector_rag',
            executable='rag_node',
            name='rag_node',
            output='screen',
            parameters=[{
                'top_k':         LaunchConfiguration('rag_top_k'),
                'force_rebuild': False,
            }],
        ),

        # ── LLM node — delegates RAG to rag_node via topics ──────────────────
        Node(
            package='vector_llm',
            executable='llm_node',
            name='llm_node',
            output='screen',
            parameters=[{
                'ws_url':     LaunchConfiguration('ws_url'),
                'use_rag':    False,   # rag_node handles retrieval
                'rag_top_k':  LaunchConfiguration('rag_top_k'),
                'tools_file': LaunchConfiguration('tools_file'),
            }],
        ),

        # ── TTS node — Piper TTS (CPU, ~160ms latency) ───────────────────────
        Node(
            package='vector_tts',
            executable='tts_node',
            name='tts_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('tts_model'),
                'device':     LaunchConfiguration('tts_device'),
                'volume':     LaunchConfiguration('tts_volume'),
            }],
        ),

    ])
