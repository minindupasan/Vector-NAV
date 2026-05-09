"""
Audio format conversion utilities for the web voice assistant.

- Browser sends WebM/Opus → decode to 16kHz mono float32 PCM (for Whisper)
- Kokoro TTS outputs 24kHz float32 PCM → encode to 16-bit WAV (for browser playback)
"""

import io
import wave
import subprocess
import numpy as np


import logging

logger = logging.getLogger(__name__)

def decode_webm_to_pcm(webm_bytes: bytes, target_sr: int = 16000) -> np.ndarray:
    """Decode WebM/Opus audio to 16kHz mono float32 numpy array using ffmpeg."""
    if not webm_bytes:
        return np.array([], dtype=np.float32)

    cmd = [
        'ffmpeg',
        '-i', 'pipe:0',
        '-f', 'f32le',
        '-acodec', 'pcm_f32le',
        '-ar', str(target_sr),
        '-ac', '1',
        '-loglevel', 'error',
        'pipe:1',
    ]
    try:
        proc = subprocess.run(
            cmd,
            input=webm_bytes,
            capture_output=True,
            timeout=10,
        )
        if proc.returncode != 0:
            logger.warning(f"ffmpeg decode failed: {proc.stderr.decode().strip()}")
            return np.array([], dtype=np.float32)
        
        if len(proc.stdout) == 0:
            return np.array([], dtype=np.float32)
            
        return np.frombuffer(proc.stdout, dtype=np.float32)
    except Exception as e:
        logger.error(f"Error during audio decoding: {e}")
        return np.array([], dtype=np.float32)


def pcm_to_wav(audio: np.ndarray, sample_rate: int = 24000) -> bytes:
    """Convert float32 numpy array to 16-bit PCM WAV bytes."""
    int16 = (audio * 32767).clip(-32768, 32767).astype(np.int16)
    buf = io.BytesIO()
    with wave.open(buf, 'wb') as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sample_rate)
        wf.writeframes(int16.tobytes())
    return buf.getvalue()
