"""
VECTOR LLM — LLM Engine
========================
Thin client around the Ollama REST API.
Handles message history, RAG context injection, and function-call parsing.
No ROS2 dependencies — usable standalone for testing.
"""

import json
import requests
from typing import Optional


SYSTEM_PROMPT = (
    "You are VECTOR NAV, an intelligent warehouse robot assistant. "
    "You help operators navigate the warehouse and answer questions about "
    "robot operations, locations, and procedures. "
    "When asked to navigate or perform a robot action, use the available tools. "
    "When answering knowledge questions, use the provided context. "
    "Be concise and clear. If you don't know something, say so."
)

# Maximum number of messages kept in conversation history (user+assistant pairs).
MAX_HISTORY_PAIRS = 10


class LLMEngine:
    """
    Stateful Ollama client with tool-call support and rolling conversation history.

    Usage:
        engine = LLMEngine(tools=[...])
        result = engine.chat("Go to the loading bay")
        # result = {'type': 'tool_call', 'name': 'navigate_to',
        #           'arguments': {'location': 'Loading Bay'}}
        #       OR {'type': 'text', 'content': 'The loading bay is at...'}
    """

    def __init__(
        self,
        ollama_url: str = "http://localhost:11434",
        model: str = "qwen2.5:3b",
        tools: Optional[list] = None,
    ):
        self.ollama_url = ollama_url.rstrip('/')
        self.model      = model
        self.tools      = tools or []
        self._history: list[dict] = []

    # ── Public API ────────────────────────────────────────────────────────────

    def chat(self, user_text: str, context: str = "") -> dict:
        """
        Send a user message to the LLM, optionally augmented with RAG context.

        Returns one of:
          {'type': 'text',      'content': str}
          {'type': 'tool_call', 'name': str, 'arguments': dict}
        Raises requests.RequestException on network/API failure.
        """
        user_content = _build_user_content(user_text, context)

        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            *self._history,
            {"role": "user", "content": user_content},
        ]

        payload: dict = {
            "model":   self.model,
            "messages": messages,
            "stream":  False,
            "options": {"temperature": 0.2},
        }
        if self.tools:
            payload["tools"] = self.tools

        response = requests.post(
            f"{self.ollama_url}/api/chat",
            json=payload,
            timeout=60,
        )
        response.raise_for_status()
        message = response.json()["message"]

        # Update rolling history
        self._history.append({"role": "user",      "content": user_content})
        self._history.append({"role": "assistant",  "content": message.get("content", "")})
        max_msgs = MAX_HISTORY_PAIRS * 2
        if len(self._history) > max_msgs:
            self._history = self._history[-max_msgs:]

        return _parse_message(message)

    def reset_history(self) -> None:
        """Clear conversation history (e.g. on session restart)."""
        self._history.clear()


# ── Helpers ───────────────────────────────────────────────────────────────────

def _build_user_content(user_text: str, context: str) -> str:
    if context and context.strip():
        return f"Context:\n{context}\n\nUser: {user_text}"
    return user_text


def _parse_message(message: dict) -> dict:
    """Parse an Ollama response message into a structured result dict."""
    tool_calls = message.get("tool_calls")
    if tool_calls:
        call = tool_calls[0]["function"]
        return {
            "type":      "tool_call",
            "name":      call["name"],
            "arguments": call.get("arguments", {}),
        }
    return {
        "type":    "text",
        "content": message.get("content", "").strip(),
    }
