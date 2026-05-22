"""
Voice pipeline orchestrator — loads STT, LLM, and TTS engines
and wires them together for the web UI.

No ROS2 dependencies. Engines are imported directly via sys.path,
following the same pattern as scripts/test_pipeline.py.
"""

import sys
import json
import logging
import time
import numpy as np
from pathlib import Path

# Make source packages importable without installing
_SRC = Path(__file__).resolve().parent.parent.parent  # src/
for pkg in ('vector_llm', 'vector_tts', 'vector_stt'):
    pkg_path = str(_SRC / pkg)
    if pkg_path not in sys.path:
        sys.path.insert(0, pkg_path)

from vector_llm.llm_engine import LLMEngine
from faster_whisper import WhisperModel

import os

from .audio_utils import pcm_to_wav

_SPEAKER_DEVICE = os.environ.get('TTS_SPEAKER_DEVICE', 'pulse').strip() or None
_sd = None
if _SPEAKER_DEVICE:
    try:
        import sounddevice as _sd  # noqa
    except Exception as _e:
        logging.getLogger(__name__).warning(f'sounddevice unavailable — speaker output disabled: {_e}')
        _sd = None

logger = logging.getLogger(__name__)

TOOLS_FILE = _SRC.parent / 'config' / 'tools.json'

# Silence trimming (from tts_node.py)
_SILENCE_THRESH = 0.003
_SILENCE_PAD_SAMPLES = 4800  # ~200ms at 24kHz


def _trim_silence(
    audio: np.ndarray, sample_rate: int,
    threshold: float = _SILENCE_THRESH,
    pad_samples: int = _SILENCE_PAD_SAMPLES,
) -> np.ndarray:
    """Trim leading and trailing silence, keeping a small pad for naturalness."""
    win = int(sample_rate * 0.01)
    if len(audio) < win:
        return audio
    energies = np.array([
        np.sqrt(np.mean(audio[i:i+win] ** 2))
        for i in range(0, len(audio) - win, win)
    ])
    above = np.where(energies > threshold)[0]
    if len(above) == 0:
        return audio
    start = max(0, above[0] * win - pad_samples)
    end = min(len(audio), (above[-1] + 1) * win + pad_samples)
    return audio[start:end]


class VoicePipeline:
    """
    Loads all engines once, exposes transcribe/synthesize/process methods.

    Usage:
        pipeline = VoicePipeline()
        pipeline.load()
        result = pipeline.process(audio_pcm, on_tts_chunk=callback)
    """

    def __init__(
        self,
        ws_url: str = 'wss://localhost:49000',
        stt_model: str = 'tiny.en',
        tts_voice: str = 'af_heart',
        tts_speed: float = 1.0,
        tts_engine_dir: str = '/home/admin/models/kokoro_trt',
        use_trt: bool = True,
    ):
        self.ws_url = ws_url
        self.stt_model = stt_model
        self.tts_voice = tts_voice
        self.tts_speed = tts_speed
        self.tts_engine_dir = tts_engine_dir
        self.use_trt = use_trt

        self.whisper = None
        self.llm = None
        self.tts = None
        self.tts_pipeline = None
        self.sample_rate = 24000

        # Audio output config (mutable at runtime via web UI)
        self.speaker_volume = 1.0   # 0.0 .. 2.5 — multiplier on top of base gain
        self.speaker_target = 'robot'  # 'robot' | 'browser'

    def load(self):
        """Load all engines. Call once at startup."""

        # STT — faster-whisper
        logger.info(f'Loading Whisper model ({self.stt_model})...')
        self.whisper = WhisperModel(
            self.stt_model, device='cpu', compute_type='int8'
        )
        logger.info('Whisper ready')

        # LLM
        tools = []
        if TOOLS_FILE.exists():
            tools = json.loads(TOOLS_FILE.read_text())
            logger.info(f'Loaded {len(tools)} tools from tools.json')
        self.llm = LLMEngine(ws_url=self.ws_url, tools=tools)
        logger.info('LLM engine created (connects on first use)')

        # TTS — TensorRT (FP32 decoder) with PyTorch failsafe
        loaded_trt = False
        if self.use_trt:
            try:
                logger.info(f'Loading Kokoro TRT from {self.tts_engine_dir}...')
                from vector_tts.kokoro_trt import KokoroTRT
                self.tts = KokoroTRT(self.tts_engine_dir)
                self.tts.load_voice(self.tts_voice)
                self.sample_rate = self.tts.sample_rate
                logger.info('Kokoro TRT ready')
                loaded_trt = True
            except Exception as e:
                logger.warning(f'Kokoro TRT load failed ({e}); falling back to PyTorch')
        if not loaded_trt:
            import torch
            device = 'cuda' if torch.cuda.is_available() else 'cpu'
            logger.info(f'Loading Kokoro TTS (PyTorch {device})...')
            from kokoro import KPipeline
            self.tts_pipeline = KPipeline(lang_code='a', device=device)
            logger.info('Kokoro TTS ready')

        logger.info('Voice pipeline fully loaded')

    def transcribe(self, audio: np.ndarray) -> tuple[str, float, str]:
        """
        Run faster-whisper on audio PCM.
        Returns (text, confidence, language).
        Replicates stt_node.py _transcribe() logic.
        """
        segments, info = self.whisper.transcribe(
            audio,
            beam_size=3,
            language='en',
            vad_filter=True,
            vad_parameters=dict(
                min_silence_duration_ms=300,
                speech_pad_ms=100,
            ),
        )

        text_parts = []
        total_prob = 0.0
        count = 0
        for seg in segments:
            text_parts.append(seg.text)
            total_prob += seg.avg_logprob
            count += 1

        text = ' '.join(text_parts).strip()
        avg_prob = total_prob / count if count > 0 else -1.0
        confidence = min(1.0, max(0.0, 1.0 + avg_prob)) if count > 0 else -1.0

        return text, confidence, info.language

    def synthesize(self, text: str) -> np.ndarray:
        """Synthesize text to float32 audio array."""
        if self.tts is not None:
            audio_f = self.tts.synthesize(
                text, voice=self.tts_voice, speed=self.tts_speed
            )
        else:
            chunks = []
            for _, _, audio in self.tts_pipeline(
                text, voice=self.tts_voice, speed=self.tts_speed
            ):
                chunks.append(audio)
            audio_f = np.concatenate(chunks).astype(np.float32)

        return _trim_silence(audio_f, self.sample_rate)

    def process(self, audio_pcm: np.ndarray, on_tts_chunk=None, on_stt=None) -> dict:
        """
        Full pipeline: STT → LLM (streaming) → TTS per sentence.

        on_stt(text: str, confidence: float) is called as soon as STT
        completes, so the UI can render the user message before LLM/TTS.
        on_tts_chunk(text: str, wav_bytes: bytes) is called for each
        sentence as it is synthesized, enabling streaming playback.

        Returns dict with stt_text, confidence, llm_response, llm_type.
        """
        # 1) Transcribe
        text, confidence, language = self.transcribe(audio_pcm)
        logger.info(f'STT: "{text}" (confidence={confidence:.2f})')

        if on_stt:
            try: on_stt(text, confidence)
            except Exception as e: logger.warning(f'on_stt callback failed: {e}')

        if not text:
            return {
                'stt_text': '',
                'confidence': 0.0,
                'llm_response': '',
                'llm_type': 'text',
            }

        return self._run_llm(text, confidence, on_tts_chunk)

    def process_text(self, text: str, on_tts_chunk=None) -> dict:
        """Same as process() but starts from typed text (no STT)."""
        text = (text or '').strip()
        if not text:
            return {'stt_text': '', 'confidence': 1.0, 'llm_response': '', 'llm_type': 'text'}
        return self._run_llm(text, 1.0, on_tts_chunk)

    def _run_llm(self, text: str, confidence: float, on_tts_chunk):
        # LLM streaming with per-sentence TTS
        def on_sentence(sentence: str):
            if not sentence.strip():
                return
            logger.info(f'LLM sentence: "{sentence}"')
            t0 = time.time()
            audio = self.synthesize(sentence)
            logger.info(f'TTS synthesized in {time.time()-t0:.2f}s ({len(audio)} samples)')
            if len(audio) == 0:
                return
            target = self.speaker_target

            # Always notify the UI with the sentence text so the chat updates
            # regardless of audio target. Send WAV bytes only when the browser
            # is the playback target.
            if on_tts_chunk:
                wav_bytes = pcm_to_wav(audio, self.sample_rate) if target == 'browser' else None
                on_tts_chunk(sentence, wav_bytes)

            # Robot i2s speaker via PulseAudio (paplay).
            # Stream s16le PCM at native 24 kHz in ~100 ms chunks so slider
            # changes apply mid-sentence. paplay's own buffering keeps it
            # underrun-free.
            if target == 'robot':
                try:
                    import subprocess
                    sr = int(self.sample_rate)
                    chunk_n = max(1, sr // 10)  # ~100 ms
                    proc = subprocess.Popen(
                        ['paplay',
                         '--raw',
                         f'--rate={sr}',
                         '--format=s16le',
                         '--channels=1',
                         '--latency-msec=200'],
                        stdin=subprocess.PIPE,
                    )
                    logger.info(f'Playing audio via paplay (streaming, vol live)')
                    try:
                        for i in range(0, len(audio), chunk_n):
                            v = max(0.0, min(2.5, float(self.speaker_volume)))
                            seg = np.clip(audio[i:i+chunk_n] * 3.0 * v, -1.0, 1.0)
                            pcm16 = (seg * 32767.0).astype('<i2').tobytes()
                            proc.stdin.write(pcm16)
                        proc.stdin.close()
                    except BrokenPipeError:
                        pass
                    proc.wait()
                except Exception as e:
                    logger.warning(f'Speaker playback failed: {e}')

        logger.info(f'Querying LLM: "{text}"')
        result = self.llm.chat_stream(
            text, context='', on_chunk=on_sentence
        )
        logger.info(f'LLM response complete: {result.get("type", "text")}')

        return {
            'stt_text': text,
            'confidence': confidence,
            'llm_response': result.get('content', ''),
            'llm_type': result.get('type', 'text'),
        }
