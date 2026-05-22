"""
VECTOR LLM — ROS2 Node
=======================
Central communication hub. Receives transcribed speech, calls the LLM via
NanoLLM WebSocket, then routes the response to TTS or the navigation bridge.

Topics
------
  Subscribes : /stt/text        (vector_interfaces/SttResult)
  Publishes  : /tts/input       (std_msgs/String)
               /llm/tool_call   (vector_interfaces/LLMToolCall)

Parameters
----------
  ws_url       (string, default 'wss://localhost:49000')
  tools_file   (string, default '')
"""

import json
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Bool

from vector_interfaces.msg import SttResult, LLMToolCall
from vector_llm.llm_engine import LLMEngine


class LLMNode(Node):

    def __init__(self):
        super().__init__('llm_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('ws_url',     'wss://localhost:49000')
        self.declare_parameter('tools_file', '')

        ws_url     = self.get_parameter('ws_url').get_parameter_value().string_value
        tools_file = self.get_parameter('tools_file').get_parameter_value().string_value

        # ── LLM engine ────────────────────────────────────────────────────────
        tools = self._load_tools(tools_file)
        self._llm = LLMEngine(ws_url=ws_url, tools=tools)
        self.get_logger().info(f'Connecting to NanoLLM at {ws_url}...')
        self._llm.connect()
        self.get_logger().info(f'LLM engine ready  tools={len(tools)}')

        # Interrupt flag — set by barge-in, stops on_chunk from publishing stale text
        self._interrupted = threading.Event()
        self._is_generating = False

        # Reentrant group lets /llm/interrupt callback fire while _on_stt is blocking
        cb = ReentrantCallbackGroup()

        # ── Publishers ────────────────────────────────────────────────────────
        self._tts_pub       = self.create_publisher(String,      '/tts/input',     10)
        self._tool_call_pub = self.create_publisher(LLMToolCall, '/llm/tool_call', 10)
        self._state_pub     = self.create_publisher(String,      '/llm/state',     10)
        self._llm_state = 'idle'

        # Republish state once per second so late subscribers see it.
        self.create_timer(1.0, lambda: self._state_pub.publish(String(data=self._llm_state)),
                          callback_group=cb)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(SttResult, '/stt/text',      self._on_stt,       10, callback_group=cb)
        self.create_subscription(Bool,      '/llm/interrupt', self._on_interrupt,  10, callback_group=cb)
        self.create_subscription(Bool,      '/llm/clear',     self._on_clear,      10, callback_group=cb)

        self.get_logger().info(
            'LLMNode started\n'
            '  Subscribes : /stt/text, /llm/interrupt, /llm/clear\n'
            '  Publishes  : /tts/input, /llm/tool_call'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_stt(self, msg: SttResult) -> None:
        text = msg.text.strip()
        if not text:
            return
        # System-injected messages (confidence == -1.0) go straight to TTS
        if msg.confidence == -1.0:
            self.get_logger().info(f'System event → TTS: "{text}"')
            self._tts_pub.publish(String(data=text))
            self._tts_pub.publish(String(data='[end]'))
            return
        t0 = time.monotonic()
        self.get_logger().info(
            f'[T+0ms] Input: "{text}"  confidence={msg.confidence:.2f}  lang={msg.language or "?"}'
        )
        self._run_llm(text, t0)

    def _on_interrupt(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info('Barge-in interrupt received — stopping current response')
            self._interrupted.set()
            if not self._is_generating:
                self._tts_pub.publish(String(data='[end]'))

    def _on_clear(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info('Clearing conversation history...')
            self._llm.reset_history()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _emit_llm_state(self, name: str) -> None:
        if name != self._llm_state:
            self._llm_state = name
            self._state_pub.publish(String(data=name))

    def _run_llm(self, text: str, t0: float) -> None:
        self._interrupted.clear()
        self._is_generating = True
        self._emit_llm_state('thinking')
        try:
            chunk_count = 0

            def on_chunk(sentence):
                nonlocal chunk_count
                if self._interrupted.is_set():
                    return False
                if chunk_count == 0:
                    self._emit_llm_state('responding')
                chunk_count += 1
                self.get_logger().info(
                    f'[T+{(time.monotonic()-t0)*1000:.0f}ms] TTS chunk {chunk_count}: "{sentence}"'
                )
                self._tts_pub.publish(String(data=sentence))

            self.get_logger().info(f'[T+{(time.monotonic()-t0)*1000:.0f}ms] LLM inference start')
            result = self._llm.chat_stream(
                text, context='', on_chunk=on_chunk, check_interrupt=self._interrupted.is_set
            )
            self.get_logger().info(f'[T+{(time.monotonic()-t0)*1000:.0f}ms] LLM inference complete')
        except Exception as e:
            self.get_logger().error(f'LLM call failed: {e}')
            self._tts_pub.publish(String(data='Sorry, I could not process that request.'))
            self._tts_pub.publish(String(data='[end]'))
            self._is_generating = False
            self._emit_llm_state('idle')
            return

        self._tts_pub.publish(String(data='[end]'))
        self._is_generating = False
        self._emit_llm_state('idle')

        if result['type'] == 'tool_call':
            out = LLMToolCall()
            out.name           = result['name']
            out.arguments_json = json.dumps(result['arguments'])
            self.get_logger().info(f'Tool call: {out.name}  args={out.arguments_json}')

            args = result['arguments']
            if 'location' in args:
                announcement = f"Now navigating to {args['location']}."
            elif out.name in ('stop_navigation', 'stop'):
                announcement = "Stopping navigation."
            else:
                announcement = f"Executing {out.name.replace('_', ' ')}."
            self._tts_pub.publish(String(data=announcement))
            self._tts_pub.publish(String(data='[end]'))

            self._tool_call_pub.publish(out)
        else:
            self.get_logger().info(f'[T+{(time.monotonic()-t0)*1000:.0f}ms] Complete ({chunk_count} chunks)')

    def _load_tools(self, tools_file: str) -> list:
        path = Path(tools_file) if tools_file else _find_config('tools.json')
        if path and path.exists():
            tools = json.loads(path.read_text())
            self.get_logger().info(f'Loaded {len(tools)} tools from {path}')
            return tools
        self.get_logger().warn('tools.json not found — running without function calling')
        return []


def _find_config(filename: str) -> Path | None:
    candidate = Path(__file__).resolve()
    for _ in range(8):
        candidate = candidate.parent
        config = candidate / 'config' / filename
        if config.exists():
            return config
    return None


def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
