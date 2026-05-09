import sys
import logging
import numpy as np
from pathlib import Path

# Add src to sys.path
_SRC = Path(__file__).resolve().parent / 'src'
for pkg in ('vector_web',):
    sys.path.insert(0, str(_SRC / pkg))

from vector_web.pipeline import VoicePipeline

logging.basicConfig(level=logging.INFO)

def test_tts():
    pipeline = VoicePipeline()
    print("Loading pipeline (this might take a while for Kokoro)...")
    pipeline.load()
    print("Pipeline loaded. Synthesizing...")
    try:
        audio = pipeline.synthesize("Hello, I am testing the voice response.")
        print(f"Synthesized audio length: {len(audio)}")
        if len(audio) > 0:
            print("Success!")
        else:
            print("Failed: Empty audio.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_tts()
