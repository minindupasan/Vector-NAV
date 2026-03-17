"""
VECTOR STT — ROS2 Node
=======================
Speech-to-text using faster-whisper (tiny.en, int8, CPU).
Listens on microphone via PulseAudio, detects voice activity using
energy-based VAD, transcribes speech, and publishes results.

Wake word: "Hey Jarvis" — only processes commands after hearing it.
Pauses listening while TTS is speaking to prevent echo feedback.

Performance (Jetson Orin Nano Super, CPU, tiny.en int8):
  ~75MB RAM, ~900ms transcription for ~2s audio

Parameters
----------
  model_size       (string)  Whisper model name (default 'tiny.en')
  wake_word        (string)  Wake word phrase (default 'hey jarvis')
  energy_threshold (float)   VAD energy threshold (default 0.01)
  silence_duration (float)   Seconds of silence to end utterance (default 1.0)
  min_duration     (float)   Minimum speech duration to transcribe (default 0.3)
  max_duration     (float)   Maximum recording duration (default 10.0)
  sample_rate      (int)     Audio sample rate (default 16000)
  listen_timeout   (float)   Seconds to wait for command after wake word (default 5.0)

Topics
------
  Subscribes : /tts/speaking  (std_msgs/Bool)
  Publishes  : /stt/text      (vector_interfaces/SttResult)
"""

import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Bool, String

from vector_interfaces.msg import SttResult

import sounddevice as sd
from faster_whisper import WhisperModel

# Variations the model might produce for the wake word
_WAKE_VARIANTS = [
    'hey jarvis', 'hey jarvas', 'hey jervis', 'hey travis',
    'hey javis', 'a jarvis', 'hey, jarvis', 'hey jarvus',
    'hay jarvis', 'he jarvis',
]


class STTNode(Node):

    def __init__(self):
        super().__init__('stt_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('model_size', 'base.en')
        self.declare_parameter('wake_word', 'hey jarvis')
        self.declare_parameter('energy_threshold', 0.01)
        self.declare_parameter('silence_duration', 1.0)
        self.declare_parameter('min_duration', 0.3)
        self.declare_parameter('max_duration', 10.0)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('listen_timeout', 7.0)

        model_size = self.get_parameter('model_size').get_parameter_value().string_value
        self._wake_word = self.get_parameter('wake_word').get_parameter_value().string_value.lower().strip()
        self._energy_thresh = self.get_parameter('energy_threshold').get_parameter_value().double_value
        self._silence_dur = self.get_parameter('silence_duration').get_parameter_value().double_value
        self._min_dur = self.get_parameter('min_duration').get_parameter_value().double_value
        self._max_dur = self.get_parameter('max_duration').get_parameter_value().double_value
        self._sr = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self._listen_timeout = self.get_parameter('listen_timeout').get_parameter_value().double_value

        # ── Load Whisper model ──────────────────────────────────────────────
        self.get_logger().info(f'Loading faster-whisper model: {model_size} (int8, CPU)...')
        self._model = WhisperModel(model_size, device='cpu', compute_type='int8')
        self.get_logger().info(f'Whisper model ready. Wake word: "{self._wake_word}"')

        # ── State ───────────────────────────────────────────────────────────
        self._tts_speaking = False
        self._running = True
        self._awake = False  # True after wake word detected

        # ── Publishers ──────────────────────────────────────────────────────
        self._stt_pub = self.create_publisher(SttResult, '/stt/text', 10)
        self._tts_pub = self.create_publisher(String, '/tts/input', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        cb = ReentrantCallbackGroup()
        self.create_subscription(Bool, '/tts/speaking', self._on_tts_speaking, 10, callback_group=cb)

        # ── Audio capture thread ────────────────────────────────────────────
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'STTNode started  model={model_size}  sr={self._sr}\n'
            f'  Wake word  : "{self._wake_word}"\n'
            '  Subscribes : /tts/speaking\n'
            '  Publishes  : /stt/text'
        )

    def _on_tts_speaking(self, msg: Bool) -> None:
        self._tts_speaking = msg.data

    def _listen_loop(self) -> None:
        """Continuously listen for wake word, then capture command."""
        chunk_size = int(self._sr * 0.1)  # 100ms chunks

        while self._running:
            try:
                # Wait for speech onset
                audio = self._wait_for_speech(chunk_size)
                if audio is None:
                    continue

                # Record until silence
                audio = self._record_until_silence(audio, chunk_size)
                if audio is None:
                    continue

                duration = len(audio) / self._sr
                if duration < self._min_dur:
                    continue

                # Transcribe (fast mode for wake word detection, full for commands)
                t_transcribe = time.monotonic()
                text, confidence, language = self._transcribe(audio, fast=not self._awake)
                t_done = time.monotonic()
                self.get_logger().info(
                    f'[STT] Transcription: {(t_done-t_transcribe)*1000:.0f}ms for {duration:.1f}s audio'
                )

                if not text or not text.strip():
                    continue

                text_clean = text.strip()
                text_lower = text_clean.lower()

                # Check for wake word
                if not self._awake:
                    if self._contains_wake_word(text_lower):
                        self._awake = True
                        # Extract command after wake word (if user said it in one breath)
                        command = self._strip_wake_word(text_lower, text_clean)
                        if command and len(command) > 3:
                            self.get_logger().info(f'Wake + command: "{command}"')
                            self._publish(command, confidence, language)
                            # Stay awake for follow-up commands
                            self.get_logger().info('Still listening for follow-up…')
                        else:
                            self.get_logger().info('Wake word detected — listening for command...')
                            self._tts_pub.publish(String(data='[greeting]'))
                    continue

                # We're awake — this is the command
                self.get_logger().info(f'Command: "{text_clean}" (conf={confidence:.2f})')
                self._publish(text_clean, confidence, language)
                # Stay awake for follow-up commands (times out via listen_timeout)
                self.get_logger().info('Still listening for follow-up…')

            except Exception as e:
                self.get_logger().error(f'Listen loop error: {e}')

    def _contains_wake_word(self, text: str) -> bool:
        """Check if text contains the wake word or a common misheard variant."""
        for variant in _WAKE_VARIANTS:
            if variant in text:
                return True
        return self._wake_word in text

    def _strip_wake_word(self, text_lower: str, text_original: str) -> str:
        """Remove wake word from beginning and return the remaining command."""
        for variant in _WAKE_VARIANTS + [self._wake_word]:
            idx = text_lower.find(variant)
            if idx >= 0:
                after = text_original[idx + len(variant):].strip(' ,.')
                return after
        return ''

    def _publish(self, text: str, confidence: float, language: str) -> None:
        msg = SttResult()
        msg.text = text
        msg.confidence = confidence
        msg.language = language
        self._stt_pub.publish(msg)
        self.get_logger().info(f'STT: "{text}" (conf={confidence:.2f}, lang={language})')

    def _wait_for_speech(self, chunk_size: int) -> np.ndarray | None:
        """Block until voice energy exceeds threshold."""
        timeout_chunks = int(self._listen_timeout / 0.1) if self._awake else 0
        waited = 0

        while self._running:
            if self._tts_speaking:
                sd.sleep(100)
                continue

            try:
                chunk = sd.rec(chunk_size, samplerate=self._sr, channels=1,
                               dtype='float32', blocking=True).flatten()
            except Exception:
                sd.sleep(200)
                continue

            energy = np.sqrt(np.mean(chunk ** 2))
            if energy > self._energy_thresh:
                return chunk

            # If awake and waiting for command, timeout after listen_timeout
            if self._awake:
                waited += 1
                if waited >= timeout_chunks:
                    self.get_logger().info('No command heard — going back to sleep.')
                    self._awake = False
                    return None

        return None

    def _record_until_silence(self, initial: np.ndarray, chunk_size: int) -> np.ndarray | None:
        """Continue recording until silence_duration of quiet, or max_duration."""
        chunks = [initial]
        silence_chunks = 0
        # Shorter silence threshold when waiting for wake word (faster detection)
        silence_dur = self._silence_dur if self._awake else min(self._silence_dur, 0.5)
        max_silence = int(silence_dur / 0.1)
        max_chunks = int(self._max_dur / 0.1)

        for _ in range(max_chunks):
            if not self._running:
                return None

            try:
                chunk = sd.rec(chunk_size, samplerate=self._sr, channels=1,
                               dtype='float32', blocking=True).flatten()
            except Exception:
                break

            chunks.append(chunk)
            energy = np.sqrt(np.mean(chunk ** 2))

            if energy < self._energy_thresh:
                silence_chunks += 1
                if silence_chunks >= max_silence:
                    break
            else:
                silence_chunks = 0

        return np.concatenate(chunks)

    def _transcribe(self, audio: np.ndarray, fast: bool = False) -> tuple[str, float, str]:
        """Run faster-whisper on audio array. Returns (text, confidence, language).
        fast=True uses beam_size=1 for quicker wake word detection."""
        segments, info = self._model.transcribe(
            audio,
            beam_size=1 if fast else 3,
            language='en',
            vad_filter=True,
            vad_parameters=dict(
                min_silence_duration_ms=300,
                speech_pad_ms=100,
            ),
            initial_prompt='Hey Jarvis',
        )

        text_parts = []
        total_prob = 0.0
        count = 0
        for seg in segments:
            text_parts.append(seg.text)
            total_prob += seg.avg_logprob
            count += 1

        text = ' '.join(text_parts).strip()
        # Convert avg log prob to a 0-1 confidence
        avg_prob = total_prob / count if count > 0 else -1.0
        confidence = min(1.0, max(0.0, 1.0 + avg_prob)) if count > 0 else -1.0

        return text, confidence, info.language

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
