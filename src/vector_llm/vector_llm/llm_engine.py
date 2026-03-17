"""
VECTOR LLM — LLM Engine
========================
WebSocket client for NanoLLM's web_chat server.
Sends text prompts via NanoLLM's binary WebSocket protocol and collects
streamed responses from the chat_history JSON messages.
No ROS2 dependencies — usable standalone for testing.
"""

import json
import re
import ssl
import struct
import time
import threading
import logging

from websockets.sync.client import connect as ws_connect
from websockets.exceptions import ConnectionClosed

logger = logging.getLogger(__name__)

SYSTEM_PROMPT_TEMPLATE = (
    "You are VECTOR NAV, an intelligent warehouse robot assistant. "
    "You help operators navigate the warehouse and answer questions about "
    "robot operations, locations, and procedures.\n\n"
    "IMPORTANT RULES:\n"
    "1. For knowledge questions (what, where, how, why), answer directly in plain text using the provided context.\n"
    "2. ONLY use a tool call when the user explicitly asks you to PERFORM AN ACTION (navigate, stop, dock).\n"
    "3. When you need to call a tool, respond with ONLY this JSON and nothing else:\n"
    '   {{"tool_call": {{"name": "<tool_name>", "arguments": {{<args>}}}}}}\n\n'
    "4. Do NOT use markdown formatting. No asterisks, no bullet points, no headers. Just plain spoken English.\n\n"
    "{tools_section}"
    "Be concise and clear. If you don't know something, say so."
)


def _build_system_prompt(tools: list) -> str:
    if not tools:
        return SYSTEM_PROMPT_TEMPLATE.format(tools_section="")
    lines = ["Available tools:\n"]
    for tool in tools:
        fn = tool.get("function", tool)
        name = fn["name"]
        desc = fn.get("description", "")
        params = fn.get("parameters", {}).get("properties", {})
        param_strs = []
        for pname, pinfo in params.items():
            param_strs.append(f"{pname} ({pinfo.get('type', 'string')}): {pinfo.get('description', '')}")
        params_text = ", ".join(param_strs) if param_strs else "none"
        lines.append(f"- {name}: {desc} | Parameters: {params_text}")
    lines.append("\n")
    return SYSTEM_PROMPT_TEMPLATE.format(tools_section="\n".join(lines))

# NanoLLM WebSocket message types
_MSG_JSON = 0
_MSG_TEXT = 1

# 32-byte binary header: msg_id(u64), timestamp(u64), magic(u16), type(u16), size(u32), pad(u32), pad(u32)
_HEADER_FMT = '!QQHHIII'
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)
_MAGIC = 42


class LLMEngine:
    """
    Stateful LLM client for NanoLLM web_chat via WebSocket.

    Usage:
        engine = LLMEngine(ws_url="wss://localhost:49000")
        engine.connect()
        result = engine.chat("Go to the loading bay")
    """

    def __init__(
        self,
        ws_url: str = "wss://localhost:49000",
        tools: list | None = None,
        response_timeout: float = 30.0,
    ):
        self.ws_url = ws_url
        self.tools = tools or []
        self.response_timeout = response_timeout
        self._ws = None
        self._msg_id = 0
        self._lock = threading.Lock()
        self._system_prompt_sent = False

    # ── Connection management ─────────────────────────────────────────────

    def connect(self):
        """Connect to NanoLLM WebSocket server."""
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE

        self._ws = ws_connect(
            self.ws_url,
            ssl_context=ssl_ctx,
            max_size=None,
            close_timeout=5,
        )
        logger.info(f"Connected to NanoLLM at {self.ws_url}")

        # Wait for the initial client_state connected message
        self._recv_until_ready(timeout=10.0)

        # Set system prompt with tool definitions
        self._system_prompt = _build_system_prompt(self.tools)
        self._send_json({
            'system_prompt': self._system_prompt,
        })
        self._system_prompt_sent = True
        logger.info("System prompt sent to NanoLLM")

    def disconnect(self):
        """Close the WebSocket connection."""
        if self._ws:
            try:
                self._ws.close()
            except Exception:
                pass
            self._ws = None

    def is_connected(self) -> bool:
        return self._ws is not None

    # ── Public API ────────────────────────────────────────────────────────

    def chat(self, user_text: str, context: str = "") -> dict:
        """
        Send a user message to NanoLLM, optionally augmented with RAG context.

        Returns one of:
          {'type': 'text',      'content': str}
          {'type': 'tool_call', 'name': str, 'arguments': dict}
        """
        if not self._ws:
            self.connect()

        prompt = _build_prompt(user_text, context)

        with self._lock:
            self._send_text(prompt)
            response = self._collect_response_streaming(on_sentence=None)

        return _parse_response(response)

    def chat_stream(self, user_text: str, context: str = "", on_chunk=None):
        """
        Streaming chat — calls on_chunk(sentence) for each complete sentence
        as it arrives. Returns the final parsed result dict.

        Tool call responses are not streamed — returned as a single result.
        """
        if not self._ws:
            self.connect()

        prompt = _build_prompt(user_text, context)

        with self._lock:
            self._send_text(prompt)
            response = self._collect_response_streaming(on_sentence=on_chunk)

        return _parse_response(response)

    def reset_history(self) -> None:
        """Clear conversation history on the server."""
        if self._ws:
            self._send_json({'chat_history_reset': True})

    # ── WebSocket protocol ────────────────────────────────────────────────

    def _send_text(self, text: str):
        """Send a TEXT message using NanoLLM's binary protocol."""
        payload = text.encode('utf-8')
        header = struct.pack(
            _HEADER_FMT,
            self._msg_id,
            int(time.time() * 1000),
            _MAGIC,
            _MSG_TEXT,
            len(payload),
            0, 0,
        )
        self._ws.send(header + payload)
        self._msg_id += 1

    def _send_json(self, data: dict):
        """Send a JSON message using NanoLLM's binary protocol."""
        payload = json.dumps(data).encode('ascii')
        header = struct.pack(
            _HEADER_FMT,
            self._msg_id,
            int(time.time() * 1000),
            _MAGIC,
            _MSG_JSON,
            len(payload),
            0, 0,
        )
        self._ws.send(header + payload)
        self._msg_id += 1

    def _recv_message(self, timeout: float = 5.0):
        """
        Receive and decode one NanoLLM WebSocket message.
        Returns (msg_type, payload) or None on timeout.
        """
        try:
            msg = self._ws.recv(timeout=timeout)
        except TimeoutError:
            return None
        except ConnectionClosed:
            return None

        if isinstance(msg, str):
            return (_MSG_TEXT, msg)

        if len(msg) <= _HEADER_SIZE:
            return None

        msg_id, timestamp, magic, msg_type, payload_size = \
            struct.unpack_from('!QQHHI', msg)

        if magic != _MAGIC:
            return None

        payload = msg[_HEADER_SIZE:]

        if msg_type == _MSG_JSON:
            payload = json.loads(payload)
        elif msg_type == _MSG_TEXT:
            payload = payload.decode('utf-8')

        return (msg_type, payload)

    def _recv_until_ready(self, timeout: float = 10.0):
        """Drain messages until we get the initial connected state."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self._recv_message(timeout=2.0)
            if result is None:
                continue
            msg_type, payload = result
            if msg_type == _MSG_JSON and isinstance(payload, dict):
                if 'system_prompt' in payload or 'chat_history' in payload:
                    return payload
        return None

    def _collect_response_streaming(self, on_sentence=None) -> str:
        """
        Collect streaming chat_history updates. When on_sentence is provided,
        emit each complete sentence (ending in . ! ? or newline) for TTS.
        """
        last_raw = ""
        emitted_len = 0
        stable_count = 0
        deadline = time.time() + self.response_timeout

        while time.time() < deadline:
            result = self._recv_message(timeout=1.0)

            if result is None:
                stable_count += 1
                if last_raw and stable_count >= 2:
                    break
                continue

            msg_type, payload = result

            if msg_type == _MSG_JSON and isinstance(payload, dict):
                if 'chat_history' in payload:
                    history = payload['chat_history']
                    for entry in reversed(history):
                        if entry.get('role') == 'bot':
                            text = entry.get('text', '')
                            if text and text != last_raw:
                                last_raw = text
                                stable_count = 0

                                if on_sentence and not _looks_like_tool_call(text):
                                    cleaned = _clean_for_tts(text)
                                    new_text = cleaned[emitted_len:]
                                    # Find last sentence boundary in new text
                                    last_boundary = _find_last_sentence_boundary(new_text)
                                    if last_boundary > 0:
                                        to_emit = new_text[:last_boundary].strip()
                                        if to_emit:
                                            on_sentence(to_emit)
                                        emitted_len += last_boundary
                            break

        full_response = _clean_for_tts(last_raw)

        # Flush remaining text
        if on_sentence and not _looks_like_tool_call(last_raw):
            remaining = full_response[emitted_len:].strip()
            if remaining:
                on_sentence(remaining)

        return full_response


# ── Helpers ───────────────────────────────────────────────────────────────

def _build_prompt(user_text: str, context: str) -> str:
    if context and context.strip():
        return f"Context:\n{context}\n\nUser: {user_text}"
    return user_text


def _clean_for_tts(text: str) -> str:
    """
    Clean NanoLLM HTML output into plain spoken text suitable for TTS.
    Handles HTML entities, tags, and markdown artifacts.
    """
    # First, strip HTML tags but join adjacent text (no extra spaces)
    # NanoLLM wraps each token in <span> tags, so removing tags joins tokens
    text = text.replace('<br/>', '\n')
    text = text.replace('&amp;', '&')
    text = text.replace('&lt;', '<')
    text = text.replace('&gt;', '>')
    # Remove all HTML tags without adding spaces (tokens are contiguous)
    text = re.sub(r'<[^>]+>', '', text)
    # Remove markdown bold/italic markers
    text = re.sub(r'\*{1,3}', '', text)
    # Remove markdown bullet points
    text = re.sub(r'^\s*[-•]\s*', '', text, flags=re.MULTILINE)
    # Remove markdown headers
    text = re.sub(r'^#{1,6}\s*', '', text, flags=re.MULTILINE)
    # Collapse multiple spaces
    text = re.sub(r' {2,}', ' ', text)
    # Collapse multiple newlines
    text = re.sub(r'\n{2,}', '\n', text)
    return text.strip()


# Sentence boundaries: . ! ? followed by space/newline, or newline itself
_SENTENCE_BOUNDARY_RE = re.compile(r'(?<=[.!?])\s+|\n')


def _find_last_sentence_boundary(text: str) -> int:
    """Find the position after the last sentence boundary in text. Returns 0 if none."""
    last_pos = 0
    for m in _SENTENCE_BOUNDARY_RE.finditer(text):
        last_pos = m.end()
    return last_pos


def _looks_like_tool_call(text: str) -> bool:
    """Check if the response looks like a JSON tool call (don't stream those)."""
    cleaned = _clean_for_tts(text).strip()
    return '{"tool_call"' in cleaned


def _parse_response(response: str) -> dict:
    """
    Parse the LLM response text. If it contains a JSON tool_call block,
    extract it. Otherwise return as plain text.
    """
    try:
        start = response.find('{"tool_call"')
        if start >= 0:
            depth = 0
            for i in range(start, len(response)):
                if response[i] == '{':
                    depth += 1
                elif response[i] == '}':
                    depth -= 1
                    if depth == 0:
                        json_str = response[start:i+1]
                        data = json.loads(json_str)
                        if 'tool_call' in data:
                            tc = data['tool_call']
                            return {
                                'type': 'tool_call',
                                'name': tc.get('name', ''),
                                'arguments': tc.get('arguments', {}),
                            }
                        break
    except (json.JSONDecodeError, KeyError):
        pass

    return {
        'type': 'text',
        'content': response,
    }
