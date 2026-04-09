"""
VECTOR LLM — ROS2 Node
=======================
Central communication hub. Receives transcribed speech, enriches it with
RAG context, calls the LLM via NanoLLM WebSocket, then routes the response
to TTS or the navigation bridge.

RAG modes
---------
  use_rag=true  (standalone) : LLM node loads its own RAGEngine internally.
  use_rag=false (with rag_node) : delegates retrieval to rag_node via
                               /rag/query → /rag/context topics, avoiding
                               loading the embedding model twice.

Topics
------
  Subscribes : /stt/text        (vector_interfaces/SttResult)
               /rag/context     (vector_interfaces/RagContext)
  Publishes  : /tts/input       (std_msgs/String)
               /llm/tool_call   (vector_interfaces/LLMToolCall)
               /rag/query       (vector_interfaces/RagQuery)

Parameters
----------
  ws_url       (string, default 'wss://localhost:49000')
  tools_file   (string, default '')
  use_rag      (bool,   default true)
  rag_top_k    (int,    default 3)
  rag_timeout  (float,  default 10.0)
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

from vector_interfaces.msg import SttResult, LLMToolCall, RagQuery, RagContext
from vector_llm.llm_engine import LLMEngine


class LLMNode(Node):

    def __init__(self):
        super().__init__('llm_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('ws_url',      'wss://localhost:49000')
        self.declare_parameter('tools_file',  '')
        self.declare_parameter('use_rag',     True)
        self.declare_parameter('rag_top_k',   3)
        self.declare_parameter('rag_timeout', 10.0)

        ws_url      = self.get_parameter('ws_url').get_parameter_value().string_value
        tools_file  = self.get_parameter('tools_file').get_parameter_value().string_value
        self._use_internal_rag = self.get_parameter('use_rag').get_parameter_value().bool_value
        rag_top_k   = self.get_parameter('rag_top_k').get_parameter_value().integer_value
        self._rag_timeout = self.get_parameter('rag_timeout').get_parameter_value().double_value

        # ── LLM engine ────────────────────────────────────────────────────────
        tools = self._load_tools(tools_file)
        self._llm = LLMEngine(ws_url=ws_url, tools=tools)
        self.get_logger().info(f'Connecting to NanoLLM at {ws_url}...')
        self._llm.connect()
        self.get_logger().info(f'LLM engine ready  tools={len(tools)}')

        # ── Internal RAG (standalone mode) ────────────────────────────────────
        self._rag = None
        if self._use_internal_rag:
            from vector_rag.rag_engine import RAGEngine
            self.get_logger().info('Loading internal RAG engine…')
            self._rag = RAGEngine(top_k=rag_top_k)
            self._rag.load()
            self.get_logger().info('Internal RAG engine ready.')

        # State for delegated RAG — protected by a lock so only one query runs at a time
        self._rag_lock    = threading.Lock()
        self._rag_event   = threading.Event()
        self._rag_context = ''

        # Interrupt flag — set by barge-in, stops on_chunk from publishing stale text
        self._interrupted = threading.Event()

        # Reentrant group lets /rag/context and /llm/interrupt callbacks fire while _on_stt is blocking
        cb = ReentrantCallbackGroup()

        # ── Publishers ────────────────────────────────────────────────────────
        self._tts_pub       = self.create_publisher(String,      '/tts/input',     10)
        self._tool_call_pub = self.create_publisher(LLMToolCall, '/llm/tool_call', 10)
        self._rag_query_pub = self.create_publisher(RagQuery,    '/rag/query',     10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(SttResult,  '/stt/text',    self._on_stt,         10, callback_group=cb)
        self.create_subscription(RagContext, '/rag/context', self._on_rag_context, 10, callback_group=cb)
        self.create_subscription(Bool,       '/llm/interrupt', self._on_interrupt,  10, callback_group=cb)

        mode = 'internal RAG' if self._use_internal_rag else 'delegated to rag_node'
        self.get_logger().info(
            f'LLMNode started  RAG={mode}\n'
            '  Subscribes : /stt/text, /rag/context, /llm/interrupt\n'
            '  Publishes  : /tts/input, /llm/tool_call, /rag/query'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_stt(self, msg: SttResult) -> None:
        text = msg.text.strip()
        if not text:
            return
        t0 = time.monotonic()
        self.get_logger().info(
            f'[T+0ms] Input: "{text}"  confidence={msg.confidence:.2f}  lang={msg.language or "?"}'
        )
        context = self._get_context(text)
        t_rag = time.monotonic()
        self.get_logger().info(f'[T+{(t_rag-t0)*1000:.0f}ms] RAG context retrieved')
        self._run_llm(text, context, t0)

    def _on_interrupt(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info('Barge-in interrupt received — stopping current response')
            self._interrupted.set()

    def _on_rag_context(self, msg: RagContext) -> None:
        self._rag_context = msg.context
        self._rag_event.set()

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_context(self, query: str) -> str:
        if self._use_internal_rag and self._rag:
            try:
                results = self._rag.retrieve(query)
                return self._rag.format_context(results)
            except Exception as e:
                self.get_logger().warn(f'Internal RAG failed: {e}')
                return ''

        # Delegated — serialize concurrent queries with a lock
        with self._rag_lock:
            # Wait until rag_node is up and subscribed to /rag/query
            deadline = self._rag_timeout
            while self.count_subscribers('/rag/query') == 0 and deadline > 0:
                self.get_logger().info('Waiting for rag_node to be ready…', once=True)
                threading.Event().wait(0.1)
                deadline -= 0.1

            if self.count_subscribers('/rag/query') == 0:
                self.get_logger().warn('rag_node not available — proceeding without context')
                return ''

            self._rag_context = ''
            self._rag_event.clear()

            q = RagQuery()
            q.text  = query
            q.top_k = 0
            self._rag_query_pub.publish(q)

            if not self._rag_event.wait(timeout=self._rag_timeout):
                self.get_logger().warn('RAG context timeout — proceeding without context')

            return self._rag_context

    def _run_llm(self, text: str, context: str, t0: float) -> None:
        self._interrupted.clear()
        try:
            chunk_count = 0

            def on_chunk(sentence):
                nonlocal chunk_count
                if self._interrupted.is_set():
                    return
                chunk_count += 1
                self.get_logger().info(
                    f'[T+{(time.monotonic()-t0)*1000:.0f}ms] TTS chunk {chunk_count}: "{sentence}"'
                )
                self._tts_pub.publish(String(data=sentence))

            self.get_logger().info(f'[T+{(time.monotonic()-t0)*1000:.0f}ms] LLM inference start')
            result = self._llm.chat_stream(
                text, context=context, on_chunk=on_chunk
            )
            self.get_logger().info(f'[T+{(time.monotonic()-t0)*1000:.0f}ms] LLM inference complete')
        except Exception as e:
            self.get_logger().error(f'LLM call failed: {e}')
            # TTS has pre-cached "Sorry, I could not process that request."
            self._tts_pub.publish(String(data='Sorry, I could not process that request.'))
            self._tts_pub.publish(String(data='[end]'))

            return

        # Signal TTS that this response is done (close the audio stream)
        self._tts_pub.publish(String(data='[end]'))

        if result['type'] == 'tool_call':
            out = LLMToolCall()
            out.name           = result['name']
            out.arguments_json = json.dumps(result['arguments'])
            self.get_logger().info(f'Tool call: {out.name}  args={out.arguments_json}')
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
    # MultiThreadedExecutor allows /rag/context callback to fire
    # while _on_stt is blocking on _rag_event.wait()
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
