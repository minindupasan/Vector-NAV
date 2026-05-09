import sys
import logging
from pathlib import Path

# Add src to sys.path
_SRC = Path(__file__).resolve().parent / 'src'
for pkg in ('vector_llm',):
    sys.path.insert(0, str(_SRC / pkg))

from vector_llm.llm_engine import LLMEngine

logging.basicConfig(level=logging.INFO)

def test_llm():
    engine = LLMEngine(ws_url="wss://localhost:49000")
    print("Connecting...")
    try:
        engine.connect()
        print("Connected. Sending chat...")
        result = engine.chat("Hello, who are you?")
        print(f"Result: {result}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        engine.disconnect()

if __name__ == "__main__":
    test_llm()
