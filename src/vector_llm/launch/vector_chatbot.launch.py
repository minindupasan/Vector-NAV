"""
VECTOR NAV — Chatbot Launch File
Starts the RAG retrieval node and the LLM communication node.

The RAG embedding model is loaded ONCE inside rag_node.
llm_node delegates retrieval to rag_node via /rag/query → /rag/context,
avoiding a double load of the 90 MB sentence-transformers model.

Usage:
  ros2 launch vector_llm vector_chatbot.launch.py
  ros2 launch vector_llm vector_chatbot.launch.py model:=llama3.2:3b
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # ── Arguments ─────────────────────────────────────────────────────────
        DeclareLaunchArgument('ws_url',     default_value='wss://localhost:49000'),
        DeclareLaunchArgument('rag_top_k',  default_value='3'),
        DeclareLaunchArgument('tools_file', default_value=''),

        LogInfo(msg='Starting VECTOR NAV — RAG + LLM nodes (NanoLLM)'),

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

    ])
