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
    "You are VECTOR NAV, a friendly warehouse robot assistant.\n\n"
    "DEFAULT RULE: Always reply in short, plain spoken English. Never use JSON unless the user wants you to physically move.\n\n"
    "CONVERSATION EXAMPLES — reply like these:\n"
    "User: Hello → Hi there! How can I help you?\n"
    "User: Hi → Hello! What can I do for you today?\n"
    "User: Hey → Hey! How can I help?\n"
    "User: How are you? → I am doing great, ready to assist!\n"
    "User: What is your name? → I am VECTOR NAV, your warehouse assistant.\n"
    "User: What can you do? → I can navigate the warehouse and answer your questions.\n"
    "User: Thanks → You are welcome!\n"
    "User: Good morning → Good morning! How can I help you today?\n"
    "User: Are you there? → Yes, I am here and ready to help!\n\n"
    "NAVIGATION RULE: Only output JSON when the user uses a movement word like go, navigate, move, drive, take me, head to, dock, or stop.\n\n"
    "NAVIGATION EXAMPLES — output ONLY the JSON, nothing else:\n"
    "User: Go to the break room → "
    '{{"tool_call": {{"name": "navigate", "arguments": {{"location": "break room"}}}}}}\n'
    "User: Navigate to shelf A → "
    '{{"tool_call": {{"name": "navigate", "arguments": {{"location": "shelf A"}}}}}}\n'
    "User: Stop moving → "
    '{{"tool_call": {{"name": "stop_navigation", "arguments": {{}}}}}}\n\n'
    "{tools_section}"
    "Remember: plain English for conversation, JSON only for movement commands.\n"
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
        max_history_turns: int = 15,
    ):
        self.ws_url = ws_url
        self.tools = tools or []
        self.response_timeout = response_timeout
        self.max_history_turns = max_history_turns
        self._ws = None
        self._msg_id = 0
        self._lock = threading.Lock()
        self._system_prompt_sent = False
        self._turn_count = 0

    # ── Connection management ─────────────────────────────────────────────

    def connect(self, retries: int = 10, retry_delay: float = 10.0):
        """Connect to NanoLLM WebSocket server with retry/backoff."""
        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ssl_ctx.check_hostname = False
        ssl_ctx.verify_mode = ssl.CERT_NONE

        for attempt in range(1, retries + 1):
            try:
                self._ws = ws_connect(
                    self.ws_url,
                    ssl_context=ssl_ctx,
                    max_size=None,
                    close_timeout=5,
                )
                logger.info(f"Connected to NanoLLM at {self.ws_url}")
                break
            except (ConnectionRefusedError, OSError) as e:
                if attempt >= retries:
                    raise RuntimeError(
                        f"Could not connect to NanoLLM at {self.ws_url} "
                        f"after {retries} attempts: {e}"
                    ) from e
                logger.warning(
                    f"NanoLLM not ready (attempt {attempt}/{retries}), "
                    f"retrying in {retry_delay:.0f}s…"
                )
                time.sleep(retry_delay)

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
        Send a user message to NanoLLM with optional context.

        Returns one of:
          {'type': 'text',      'content': str}
          {'type': 'tool_call', 'name': str, 'arguments': dict}
        """
        if not self._ws:
            self.connect()

        prompt = _build_prompt(user_text, context)

        with self._lock:
            try:
                self._send_text(prompt)
                response = self._collect_response_streaming(on_sentence=None)
            except Exception as e:
                logger.warning(f"WebSocket error during chat(), reconnecting: {e}")
                self._ws = None
                self.connect()
                self._send_text(prompt)
                response = self._collect_response_streaming(on_sentence=None)

        return _parse_response(response)

    def chat_stream(self, user_text: str, context: str = "", on_chunk=None, check_interrupt=None):
        """
        Streaming chat — calls on_chunk(sentence) for each complete sentence
        as it arrives. Returns the final parsed result dict.

        Tool call responses are not streamed — returned as a single result.
        Keeps last max_history_turns exchanges for conversational context,
        then resets to prevent unbounded memory growth.
        """
        if not self._ws:
            self.connect()

        prompt = _build_prompt(user_text, context)

        with self._lock:
            try:
                self._send_text(prompt)
                response = self._collect_response_streaming(on_sentence=on_chunk, check_interrupt=check_interrupt)
            except Exception as e:
                logger.warning(f"WebSocket error during chat_stream(), reconnecting: {e}")
                self._ws = None
                self.connect()
                self._send_text(prompt)
                response = self._collect_response_streaming(on_sentence=on_chunk, check_interrupt=check_interrupt)

            # Keep short conversation memory, reset when it gets too long
            self._turn_count += 1
            if self._turn_count >= self.max_history_turns:
                self._send_json({'chat_history_reset': True})
                self._turn_count = 0

        return _parse_response(response)

    def reset_history(self) -> None:
        """Clear conversation history on the server and re-apply system prompt."""
        self._turn_count = 0
        if self._ws:
            logger.info("Sending chat_history_reset to NanoLLM...")
            self._send_json({'chat_history_reset': True})
            
            # Re-send system prompt to ensure the model maintains its persona/rules
            if hasattr(self, '_system_prompt') and self._system_prompt:
                logger.info("Re-applying system prompt after reset...")
                self._send_json({'system_prompt': self._system_prompt})

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

    def _collect_response_streaming(self, on_sentence=None, check_interrupt=None) -> str:
        """
        Collect streaming chat_history updates. When on_sentence is provided,
        emit each complete sentence (ending in . ! ? or newline) for TTS.
        """
        last_raw = ""
        emitted_len = 0
        stable_count = 0
        deadline = time.time() + self.response_timeout

        while time.time() < deadline:
            if check_interrupt and check_interrupt():
                logger.info("Interrupt detected in engine, aborting generation.")
                self._send_json({'chat_history_reset': True})
                return _clean_for_tts(last_raw)

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
                                logger.info(f"DEBUG RAW TEXT: {repr(text)}")
                                last_raw = text
                                stable_count = 0

                                if on_sentence and not _looks_like_tool_call(text):
                                    cleaned = _clean_for_tts(text)
                                    new_text = cleaned[emitted_len:]
                                    # Find first sentence boundary in new text
                                    last_boundary = _find_first_sentence_boundary(new_text)
                                    if last_boundary > 0:
                                        to_emit = new_text[:last_boundary].strip()
                                        if to_emit:
                                            if on_sentence(to_emit) is False:
                                                self._send_json({'chat_history_reset': True})
                                                return full_response
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
        ctx = context.strip()
        if len(ctx) > 300:
            ctx = ctx[:300].rsplit(' ', 1)[0] + '...'
        return f"Use this information to answer: {ctx}\n\nQuestion: {user_text}"
    return user_text


def _clean_for_tts(text: str) -> str:
    """
    Clean NanoLLM HTML output into plain spoken text suitable for TTS.
    Handles HTML entities, tags, and markdown artifacts.
    """
    # Strip Llama special tokens that leak through
    text = re.sub(r'<\|eot_id\|>', '', text)
    text = re.sub(r'<\|end_of_text\|>', '', text)
    text = re.sub(r'<\|begin_of_text\|>', '', text)
    text = re.sub(r'<\|start_header_id\|>.*?<\|end_header_id\|>', '', text)
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


# Sentence boundaries: . ! ? : followed by space/newline, or newline itself
_SENTENCE_BOUNDARY_RE = re.compile(r'(?:(?<=[.!?:])(?:\s+|$))|\n+')

# Minimum chunk size to emit — avoids splitting too early but allows "Hello!"
_MIN_CHUNK_SIZE = 6


def _find_first_sentence_boundary(text: str) -> int:
    """Find the position after the first sentence boundary in text,
    only if the resulting chunk is at least _MIN_CHUNK_SIZE characters.
    Also enforce a hard maximum length of 250 characters to prevent TTS engine crash."""
    for m in _SENTENCE_BOUNDARY_RE.finditer(text):
        if m.end() >= _MIN_CHUNK_SIZE:
            return m.end()
    
    # If no natural boundary is found but the text is dangerously long, forcefully split it
    if len(text) > 250:
        # Try to split on comma
        comma_pos = text.rfind(',', 0, 250)
        if comma_pos > _MIN_CHUNK_SIZE:
            return comma_pos + 1
        # Try to split on space
        space_pos = text.rfind(' ', 0, 250)
        if space_pos > _MIN_CHUNK_SIZE:
            return space_pos + 1
        return 250
            
    return 0


def _looks_like_tool_call(text: str) -> bool:
    """Check if the response looks like a JSON tool call (don't stream those)."""
    cleaned = _clean_for_tts(text).strip()
    return '{"tool_call"' in cleaned


def _parse_response(response: str) -> dict:
    """
    Parse the LLM response text. If it contains a JSON tool_call block,
    extract it. Otherwise return as plain text. Robust against missing trailing braces.
    """
    try:
        start = response.find('{"tool_call"')
        if start >= 0:
            # Try to parse the json string, auto-closing braces if needed
            json_str = response[start:]
            # Remove any trailing junk after the last brace
            last_brace = json_str.rfind('}')
            if last_brace >= 0:
                json_str = json_str[:last_brace+1]
            
            # Try parsing, adding up to 3 missing closing braces if it fails
            parsed_data = None
            for _ in range(4):
                try:
                    parsed_data = json.loads(json_str)
                    break
                except json.JSONDecodeError:
                    json_str += '}'
            
            if parsed_data and 'tool_call' in parsed_data:
                tc = parsed_data['tool_call']
                return {
                    'type': 'tool_call',
                    'name': tc.get('name', ''),
                    'arguments': tc.get('arguments', {}),
                }
    except Exception:
        pass

    return {
        'type': 'text',
        'content': response,
    }
