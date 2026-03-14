"""
VECTOR LLM — ROS2 Node
=======================
Central communication hub. Receives transcribed speech, enriches it with
RAG context, calls the LLM, then routes the response to TTS or the
navigation bridge.

Topics
------
  Subscribes : /stt/text       (vector_interfaces/SttResult)   — transcribed speech
  Publishes  : /tts/input      (std_msgs/String)               — text answer for TTS
               /llm/tool_call  (vector_interfaces/LLMToolCall) — structured tool call

Parameters
----------
  ollama_url   (string, default 'http://localhost:11434')
  model        (string, default 'qwen2.5:3b')
  tools_file   (string, default '') — absolute path to tools.json;
               if empty, searches upward from install location for config/tools.json
  use_rag      (bool,   default true)
  rag_top_k    (int,    default 3)

Usage
-----
  ros2 run vector_llm llm_node
  ros2 run vector_llm llm_node --ros-args -p tools_file:=/abs/path/to/tools.json
"""

import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from vector_interfaces.msg import SttResult, LLMToolCall
from vector_llm.llm_engine import LLMEngine
from vector_rag.rag_engine import RAGEngine


class LLMNode(Node):
    """ROS2 node that drives the full LLM pipeline with optional RAG augmentation."""

    def __init__(self):
        super().__init__('llm_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('ollama_url', 'http://localhost:11434')
        self.declare_parameter('model',      'qwen2.5:3b')
        self.declare_parameter('tools_file', '')
        self.declare_parameter('use_rag',    True)
        self.declare_parameter('rag_top_k',  3)

        ollama_url = self.get_parameter('ollama_url').get_parameter_value().string_value
        model      = self.get_parameter('model').get_parameter_value().string_value
        tools_file = self.get_parameter('tools_file').get_parameter_value().string_value
        use_rag    = self.get_parameter('use_rag').get_parameter_value().bool_value
        rag_top_k  = self.get_parameter('rag_top_k').get_parameter_value().integer_value

        # ── Tools ─────────────────────────────────────────────────────────────
        tools = self._load_tools(tools_file)

        # ── LLM engine ────────────────────────────────────────────────────────
        self._llm = LLMEngine(ollama_url=ollama_url, model=model, tools=tools)
        self.get_logger().info(
            f'LLM engine ready  model={model}  tools={len(tools)}'
        )

        # ── RAG engine (optional, loaded inline) ──────────────────────────────
        self._rag = None
        if use_rag:
            self.get_logger().info('Loading RAG engine…')
            self._rag = RAGEngine(top_k=rag_top_k)
            self._rag.load()
            self.get_logger().info('RAG engine ready.')

        # ── Publishers ────────────────────────────────────────────────────────
        self._tts_pub       = self.create_publisher(String,      '/tts/input',     10)
        self._tool_call_pub = self.create_publisher(LLMToolCall, '/llm/tool_call', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(SttResult, '/stt/text', self._on_stt, 10)

        self.get_logger().info(
            'LLMNode started.\n'
            '  Subscribes : /stt/text      (vector_interfaces/SttResult)\n'
            '  Publishes  : /tts/input     (std_msgs/String)\n'
            '               /llm/tool_call (vector_interfaces/LLMToolCall)'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_stt(self, msg: SttResult) -> None:
        text = msg.text.strip()
        if not text:
            return

        self.get_logger().info(
            f'Input: "{text}"  confidence={msg.confidence:.2f}  lang={msg.language or "?"}'
        )

        # 1. Retrieve RAG context
        context = self._retrieve_context(text)

        # 2. Call LLM
        try:
            result = self._llm.chat(text, context=context)
        except Exception as e:
            self.get_logger().error(f'LLM call failed: {e}')
            self._tts_pub.publish(String(data='Sorry, I could not process that request.'))
            return

        # 3. Route response
        if result['type'] == 'tool_call':
            out = LLMToolCall()
            out.name           = result['name']
            out.arguments_json = json.dumps(result['arguments'])
            self.get_logger().info(f'Tool call: {out.name}  args={out.arguments_json}')
            self._tool_call_pub.publish(out)
        else:
            answer = result['content']
            self.get_logger().info(f'Answer: "{answer}"')
            self._tts_pub.publish(String(data=answer))

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _retrieve_context(self, query: str) -> str:
        if self._rag is None:
            return ''
        try:
            results = self._rag.retrieve(query)
            return self._rag.format_context(results)
        except Exception as e:
            self.get_logger().warn(f'RAG retrieval failed: {e}')
            return ''

    def _load_tools(self, tools_file: str) -> list:
        path = Path(tools_file) if tools_file else _find_config('tools.json')

        if path and path.exists():
            tools = json.loads(path.read_text())
            self.get_logger().info(f'Loaded {len(tools)} tools from {path}')
            return tools

        self.get_logger().warn(
            'tools.json not found — running without function calling. '
            'Set the tools_file parameter to enable navigation commands.'
        )
        return []


# ── Helpers ───────────────────────────────────────────────────────────────────

def _find_config(filename: str) -> Path | None:
    """Walk up from this file looking for config/<filename>."""
    candidate = Path(__file__).resolve()
    for _ in range(8):
        candidate = candidate.parent
        config = candidate / 'config' / filename
        if config.exists():
            return config
    return None


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
