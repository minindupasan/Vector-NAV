"""
Voice pipeline orchestrator — loads STT, RAG, LLM, and TTS engines
and wires them together for the web UI.

No ROS2 dependencies. Engines are imported directly via sys.path,
following the same pattern as scripts/test_pipeline.py.
"""

import sys
import json
import logging
import numpy as np
from pathlib import Path

# Make source packages importable without installing
_SRC = Path(__file__).resolve().parent.parent.parent  # src/
for pkg in ('vector_rag', 'vector_llm', 'vector_tts', 'vector_stt'):
    pkg_path = str(_SRC / pkg)
    if pkg_path not in sys.path:
        sys.path.insert(0, pkg_path)

from vector_rag.rag_engine import RAGEngine
from vector_llm.llm_engine import LLMEngine
from faster_whisper import WhisperModel

from .audio_utils import pcm_to_wav

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
        stt_model: str = 'base.en',
        tts_voice: str = 'af_heart',
        tts_speed: float = 1.0,
        tts_engine_dir: str = '/home/admin/models/kokoro_trt',
        use_trt: bool = True,
        rag_top_k: int = 3,
    ):
        self.ws_url = ws_url
        self.stt_model = stt_model
        self.tts_voice = tts_voice
        self.tts_speed = tts_speed
        self.tts_engine_dir = tts_engine_dir
        self.use_trt = use_trt
        self.rag_top_k = rag_top_k

        self.whisper = None
        self.llm = None
        self.rag = None
        self.tts = None
        self.tts_pipeline = None
        self.sample_rate = 24000

    def load(self):
        """Load all engines. Call once at startup."""

        # STT — faster-whisper
        logger.info(f'Loading Whisper model ({self.stt_model})...')
        self.whisper = WhisperModel(
            self.stt_model, device='cpu', compute_type='int8'
        )
        logger.info('Whisper ready')

        # RAG
        logger.info('Loading RAG engine...')
        self.rag = RAGEngine(top_k=self.rag_top_k)
        self.rag.load()

        # LLM
        tools = []
        if TOOLS_FILE.exists():
            tools = json.loads(TOOLS_FILE.read_text())
            logger.info(f'Loaded {len(tools)} tools from tools.json')
        self.llm = LLMEngine(ws_url=self.ws_url, tools=tools)
        logger.info('LLM engine created (connects on first use)')

        # TTS
        if self.use_trt:
            logger.info(f'Loading Kokoro TRT from {self.tts_engine_dir}...')
            from vector_tts.kokoro_trt import KokoroTRT
            self.tts = KokoroTRT(self.tts_engine_dir)
            self.tts.load_voice(self.tts_voice)
            self.sample_rate = self.tts.sample_rate
            logger.info('Kokoro TRT ready')
        else:
            logger.info('Loading Kokoro TTS (PyTorch CPU)...')
            from kokoro import KPipeline
            self.tts_pipeline = KPipeline(lang_code='a', device='cpu')
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
        """
        Synthesize text to float32 audio array.
        Replicates tts_node.py _synthesize_to_array() logic.
        """
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

    def process(self, audio_pcm: np.ndarray, on_tts_chunk=None) -> dict:
        """
        Full pipeline: STT → RAG → LLM (streaming) → TTS per sentence.

        on_tts_chunk(text: str, wav_bytes: bytes) is called for each
        sentence as it is synthesized, enabling streaming playback.

        Returns dict with stt_text, confidence, llm_response, llm_type.
        """
        # 1) Transcribe
        text, confidence, language = self.transcribe(audio_pcm)
        logger.info(f'STT: "{text}" (confidence={confidence:.2f})')

        if not text:
            return {
                'stt_text': '',
                'confidence': 0.0,
                'llm_response': '',
                'llm_type': 'text',
            }

        # 2) RAG retrieval
        results = self.rag.retrieve(text)
        context = self.rag.format_context(results)

        # 3) LLM streaming with per-sentence TTS
        def on_sentence(sentence: str):
            if on_tts_chunk and sentence.strip():
                audio = self.synthesize(sentence)
                if len(audio) > 0:
                    wav_bytes = pcm_to_wav(audio, self.sample_rate)
                    on_tts_chunk(sentence, wav_bytes)

        result = self.llm.chat_stream(
            text, context=context, on_chunk=on_sentence
        )

        return {
            'stt_text': text,
            'confidence': confidence,
            'llm_response': result.get('content', ''),
            'llm_type': result.get('type', 'text'),
        }
