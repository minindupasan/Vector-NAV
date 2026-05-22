"""
VECTOR STT — ROS2 Node
=======================
Speech-to-text with openwakeword for instant wake word detection (~10ms)
and faster-whisper for command transcription.

Architecture
------------
  Continuous audio stream → openwakeword (always running, ~10ms per frame)
    → wake word detected → record command → faster-whisper transcription
    → publish to /stt/text

Barge-in: saying "Hey Jarvis" while TTS is speaking interrupts playback
and starts listening for a new command.

Parameters
----------
  model_size       (string)  Whisper model name (default 'base.en')
  wake_word        (string)  openwakeword model name (default 'hey_jarvis_v0.1')
  wake_threshold   (float)   Wake word confidence threshold (default 0.5)
  energy_threshold (float)   VAD energy threshold (default 0.01)
  silence_duration (float)   Seconds of silence to end utterance (default 1.0)
  min_duration     (float)   Minimum speech duration to transcribe (default 0.3)
  max_duration     (float)   Maximum recording duration (default 10.0)
  sample_rate      (int)     Audio sample rate (default 16000)
  listen_timeout   (float)   Seconds to wait for follow-up after command (default 7.0)

Topics
------
  Subscribes : /tts/speaking  (std_msgs/Bool)
  Publishes  : /stt/text      (vector_interfaces/SttResult)
               /tts/input     (std_msgs/String)  — greeting + interrupt
"""

import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, String

from vector_interfaces.msg import SttResult

import sounddevice as sd
from faster_whisper import WhisperModel
from openwakeword.model import Model as OWWModel


class STTNode(Node):

    def __init__(self):
        super().__init__('stt_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('model_size', 'base.en')
        self.declare_parameter('wake_word', 'hey_jarvis_v0.1')
        self.declare_parameter('wake_threshold', 0.5)
        self.declare_parameter('barge_in_threshold', 0.4)
        self.declare_parameter('energy_threshold', 0.01)
        self.declare_parameter('silence_duration', 1.0)
        self.declare_parameter('min_duration', 0.3)
        self.declare_parameter('max_duration', 10.0)
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('listen_timeout', 7.0)

        model_size       = self.get_parameter('model_size').get_parameter_value().string_value
        self._wake_model = self.get_parameter('wake_word').get_parameter_value().string_value
        self._wake_thresh = self.get_parameter('wake_threshold').get_parameter_value().double_value
        self._barge_thresh = self.get_parameter('barge_in_threshold').get_parameter_value().double_value
        self._energy_thresh = self.get_parameter('energy_threshold').get_parameter_value().double_value
        self._silence_dur = self.get_parameter('silence_duration').get_parameter_value().double_value
        self._min_dur     = self.get_parameter('min_duration').get_parameter_value().double_value
        self._max_dur     = self.get_parameter('max_duration').get_parameter_value().double_value
        self._sr          = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self._listen_timeout = self.get_parameter('listen_timeout').get_parameter_value().double_value

        # ── Load openwakeword (tiny, ~2MB, ~10ms per frame) ───────────────
        self.get_logger().info(f'Loading openwakeword model: {self._wake_model}')
        self._oww = OWWModel(
            wakeword_models=[self._wake_model],
            inference_framework='tflite',
        )
        self.get_logger().info('openwakeword ready')

        # ── Load Whisper (only used for command transcription) ────────────
        self.get_logger().info(f'Loading faster-whisper: {model_size} (int8, CPU)...')
        self._whisper = WhisperModel(model_size, device='cpu', compute_type='int8')
        self.get_logger().info('Whisper model ready')

        # ── State ─────────────────────────────────────────────────────────
        self._tts_speaking = False
        self._running = True
        self._awake = False  # True after wake word, stays for follow-ups

        # ── Publishers ────────────────────────────────────────────────────
        self._stt_pub = self.create_publisher(SttResult, '/stt/text', 10)
        self._tts_pub = self.create_publisher(String, '/tts/input', 10)
        self._interrupt_pub = self.create_publisher(Bool, '/llm/interrupt', 10)
        self._state_pub = self.create_publisher(String, '/stt/state', 10)
        self._stt_state = 'idle'

        # ── Subscribers ───────────────────────────────────────────────────
        cb = ReentrantCallbackGroup()
        self.create_subscription(
            Bool, '/tts/speaking', self._on_tts_speaking, 10, callback_group=cb
        )

        # Republish state once per second so late subscribers see it.
        self.create_timer(1.0, lambda: self._state_pub.publish(String(data=self._stt_state)),
                          callback_group=cb)

        # ── Audio capture thread ──────────────────────────────────────────
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f'STTNode started  whisper={model_size}  oww={self._wake_model}\n'
            f'  Wake threshold : {self._wake_thresh}\n'
            '  Subscribes : /tts/speaking\n'
            '  Publishes  : /stt/text, /tts/input'
        )

    def _on_tts_speaking(self, msg: Bool) -> None:
        self._tts_speaking = msg.data

    def _emit_stt_state(self, name: str) -> None:
        if name != self._stt_state:
            self._stt_state = name
            self._state_pub.publish(String(data=name))

    # ── Main loop ────────────────────────────────────────────────────────

    def _listen_loop(self) -> None:
        """Continuous audio → openwakeword for wake, Whisper for commands."""
        # openwakeword expects 1280 samples (80ms at 16kHz) per frame
        oww_chunk = 1280
        # For recording commands, use 100ms chunks
        rec_chunk = int(self._sr * 0.1)

        while self._running:
            try:
                if not self._awake:
                    self._wait_for_wake(oww_chunk)
                else:
                    self._capture_command(rec_chunk)
            except Exception as e:
                self.get_logger().error(f'Listen loop error: {e}')
                time.sleep(1.0)

    # ── Wake word detection (openwakeword, ~10ms per frame) ──────────────

    def _wait_for_wake(self, chunk_size: int) -> None:
        """Run openwakeword continuously until wake word is detected."""
        self._emit_stt_state('listening')
        while self._running and not self._awake:

            try:
                # Use a small timeout so we don't block forever if web audio arrives
                chunk = sd.rec(
                    chunk_size, samplerate=self._sr, channels=1,
                    dtype='int16', blocking=True,
                ).flatten()
            except Exception:
                time.sleep(0.1)
                continue

            prediction = self._oww.predict(chunk)
            score = prediction.get(self._wake_model, 0.0)

            if score >= self._wake_thresh:
                self._oww.reset()  # reset internal state for next detection
                self._awake = True

                # Barge-in: if TTS was speaking, interrupt it and stop LLM
                if self._tts_speaking:
                    self.get_logger().info('Barge-in! Interrupting TTS + LLM...')
                    self._interrupt_pub.publish(Bool(data=True))
                    self._tts_pub.publish(String(data='[interrupt]'))
                    # Brief pause to let TTS stop
                    sd.sleep(100)

                self.get_logger().info(
                    f'Wake word detected (score={score:.2f}) — listening...'
                )
                self._tts_pub.publish(String(data='[greeting]'))

                # Wait for greeting to finish playing before listening
                sd.sleep(200)  # let TTS start
                while self._tts_speaking and self._running:
                    sd.sleep(50)
                return

    # ── Command capture + transcription (Whisper) ────────────────────────

    def _capture_command(self, chunk_size: int) -> None:
        """Record command audio, transcribe with Whisper, publish result."""
        self._emit_stt_state('listening')
        # Wait for speech onset (with timeout for follow-up mode)
        audio = self._wait_for_speech(chunk_size)
        if audio is None:
            self.get_logger().info('No command heard — going back to sleep.')
            self._awake = False
            self._emit_stt_state('idle')
            return

        # Record until silence
        audio = self._record_until_silence(audio, chunk_size)
        if audio is None:
            self._emit_stt_state('idle')
            return

        duration = len(audio) / self._sr
        if duration < self._min_dur:
            self._emit_stt_state('idle')
            return

        # Transcribe with Whisper
        self._emit_stt_state('transcribing')
        t0 = time.monotonic()
        text, confidence, language = self._transcribe(audio)
        elapsed = (time.monotonic() - t0) * 1000
        self.get_logger().info(
            f'[STT] Transcription: {elapsed:.0f}ms for {duration:.1f}s audio'
        )

        if not text or not text.strip():
            self._emit_stt_state('idle')
            return

        text_clean = text.strip()
        self.get_logger().info(f'Command: "{text_clean}" (conf={confidence:.2f})')
        self._publish(text_clean, confidence, language)
        self._emit_stt_state('idle')
        # Stay awake for follow-ups (times out via listen_timeout)
        self.get_logger().info('Still listening for follow-up...')

    def _wait_for_speech(self, chunk_size: int) -> np.ndarray | None:
        """Wait for voice energy above threshold. Returns first chunk or None on timeout.
        While TTS is speaking, runs openwakeword for barge-in detection."""
        oww_chunk = 1280  # openwakeword frame size
        timeout_chunks = int(self._listen_timeout / 0.1)
        waited = 0

        while self._running:
            # While TTS is speaking, run openwakeword for barge-in
            if self._tts_speaking:
                try:
                    chunk = sd.rec(
                        oww_chunk, samplerate=self._sr, channels=1,
                        dtype='int16', blocking=True,
                    ).flatten()
                except Exception:
                    sd.sleep(200)
                    continue

                prediction = self._oww.predict(chunk)
                score = prediction.get(self._wake_model, 0.0)
                if score >= self._barge_thresh:
                    self._oww.reset()
                    self.get_logger().info(
                        f'Barge-in! Wake word during response (score={score:.2f})'
                    )
                    self._interrupt_pub.publish(Bool(data=True))
                    self._tts_pub.publish(String(data='[interrupt]'))
                    sd.sleep(100)
                    self._tts_pub.publish(String(data='[greeting]'))
                    # Wait for greeting to finish
                    sd.sleep(200)
                    while self._tts_speaking and self._running:
                        sd.sleep(50)
                    waited = 0  # reset timeout after barge-in
                continue

            # Normal: wait for speech energy (follow-up mode)
            try:
                chunk = sd.rec(
                    chunk_size, samplerate=self._sr, channels=1,
                    dtype='float32', blocking=True,
                ).flatten()
            except Exception:
                sd.sleep(200)
                continue

            energy = np.sqrt(np.mean(chunk ** 2))
            if energy > self._energy_thresh:
                return chunk

            waited += 1
            if waited >= timeout_chunks:
                return None

        return None

    def _record_until_silence(
        self, initial: np.ndarray, chunk_size: int
    ) -> np.ndarray | None:
        """Continue recording until silence_duration of quiet, or max_duration."""
        chunks = [initial]
        silence_chunks = 0
        max_silence = int(self._silence_dur / 0.1)
        max_chunks = int(self._max_dur / 0.1)

        for _ in range(max_chunks):
            if not self._running:
                return None
            try:
                chunk = sd.rec(
                    chunk_size, samplerate=self._sr, channels=1,
                    dtype='float32', blocking=True,
                ).flatten()
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

    def _transcribe(self, audio: np.ndarray) -> tuple[str, float, str]:
        """Run faster-whisper on audio. Returns (text, confidence, language)."""
        segments, info = self._whisper.transcribe(
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

    def _publish(self, text: str, confidence: float, language: str) -> None:
        msg = SttResult()
        msg.text = text
        msg.confidence = confidence
        msg.language = language
        self._stt_pub.publish(msg)
        self.get_logger().info(f'Published: "{text}" (conf={confidence:.2f})')

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
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
