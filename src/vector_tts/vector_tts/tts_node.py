"""
VECTOR TTS — ROS2 Node
=======================
Text-to-speech using Piper TTS (ONNX, CPU).

Architecture: two-thread pipeline with barge-in support
  1. Synthesizer thread: reads text from queue, runs Piper, pushes audio arrays
  2. Playback thread: reads audio arrays, writes to one continuous OutputStream
  Synthesis runs DURING playback so the next sentence is always ready.

Special tags:
  [greeting]   — play pre-cached greeting instantly
  [end]        — signal end of response (close audio stream)
  [interrupt]  — barge-in: stop current playback immediately

Parameters
----------
  model_path   (string)  Path to .onnx voice model
  device       (string)  Audio output device name ('' = system default)
  rate         (float)   Speech rate as length_scale (0.5–2.0, default 0.85)
  volume       (float)   Volume multiplier (0.0–2.0, default 1.0)

Topics
------
  Subscribes : /tts/input     (std_msgs/String)
  Publishes  : /tts/speaking  (std_msgs/Bool)
"""

import time
import queue
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool

import sounddevice as sd
from piper import PiperVoice
from piper.config import SynthesisConfig

import os
os.environ.setdefault('SDL_AUDIODRIVER', 'pulseaudio')
sd.default.extra_settings = None

_END = object()
_INTERRUPT = object()

# Minimum RMS energy to keep (trim silence below this)
_SILENCE_THRESH = 0.003
# Samples of silence to keep at boundaries for natural breathing room
_SILENCE_PAD_SAMPLES = 3300  # ~150ms at 22050Hz for natural pacing


class TTSNode(Node):

    _GREETING_TAG = '[greeting]'
    _END_TAG = '[end]'
    _INTERRUPT_TAG = '[interrupt]'

    def __init__(self):
        super().__init__('tts_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter(
            'model_path',
            '/home/jetson/vector_nav/models/tts/en_US-ryan-high.onnx',
        )
        self.declare_parameter('device', '')
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('volume', 1.0)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self._device = self.get_parameter('device').get_parameter_value().string_value or None
        self._rate = self.get_parameter('rate').get_parameter_value().double_value
        self._volume = self.get_parameter('volume').get_parameter_value().double_value

        # ── Load Piper model ────────────────────────────────────────────────
        config_path = model_path + '.json'
        self.get_logger().info(f'Loading Piper TTS model: {model_path}')
        self._voice = PiperVoice.load(model_path, config_path=config_path, use_cuda=False)
        self._sample_rate = self._voice.config.sample_rate
        self.get_logger().info(
            f'Piper TTS ready  sr={self._sample_rate}  '
            f'device={self._device or "default"}'
        )

        # ── Pre-cache common responses ────────────────────────────────────
        self.get_logger().info('Pre-caching common responses...')
        self._cached = {}
        self._cached['greeting'] = self._synthesize_to_array(
            'Hi, what can I do for you?', speed=1.25
        )
        self._cached['error'] = self._synthesize_to_array(
            'Sorry, I could not process that request.'
        )
        self._cached['dunno'] = self._synthesize_to_array(
            "I'm not sure about that."
        )
        self.get_logger().info(
            f'Pre-cached {len(self._cached)} responses'
        )

        # ── Queues ────────────────────────────────────────────────────────
        self._text_queue = queue.Queue()
        self._audio_queue = queue.Queue()
        self._interrupt = threading.Event()
        self._dropping_stale = False  # True after interrupt, until old [end] arrives

        # ── Threads ───────────────────────────────────────────────────────
        threading.Thread(target=self._synth_loop, daemon=True).start()
        threading.Thread(target=self._playback_loop, daemon=True).start()

        # ── Publishers ──────────────────────────────────────────────────────
        self._speaking_pub = self.create_publisher(Bool, '/tts/speaking', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        # Default MutuallyExclusiveCallbackGroup ensures message ordering
        # (prevents [end] from racing ahead of the last text chunk)
        self.create_subscription(
            String, '/tts/input', self._on_tts_input, 10,
        )

        self.get_logger().info(
            'TTSNode started\n'
            '  Subscribes : /tts/input\n'
            '  Publishes  : /tts/speaking'
        )

    # ── ROS callback ─────────────────────────────────────────────────────

    def _on_tts_input(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        if text == self._INTERRUPT_TAG:
            self.get_logger().info('[TTS] Barge-in interrupt!')
            self._interrupt.set()
            self._dropping_stale = True  # drop all chunks until old [end]
            sd.stop()  # immediately stops sd.play() and unblocks
            self._drain()
            return

        if text == self._END_TAG:
            if self._dropping_stale:
                # Old response finished — stop dropping, ready for new response
                self.get_logger().info('[TTS] Stale [end] dropped, ready for new input')
                self._dropping_stale = False
                return
            self._text_queue.put(_END)
        elif text == self._GREETING_TAG:
            # Greeting after barge-in — clear interrupt so playback proceeds
            self._interrupt.clear()
            self._audio_queue.put(self._cached['greeting'])
            self._audio_queue.put(_END)
        else:
            # Drop stale LLM chunks from the interrupted response
            if self._dropping_stale:
                self.get_logger().info(f'[TTS] Dropping stale: "{text[:60]}"')
                return
            if self._interrupt.is_set():
                return
            self.get_logger().info(f'[TTS] Queued: "{text[:80]}"')
            self._text_queue.put(text)

    # ── Synthesizer thread ───────────────────────────────────────────────

    def _synth_loop(self) -> None:
        """Read text from text_queue, synthesize, push audio to audio_queue."""
        syn_config = SynthesisConfig(length_scale=self._rate)

        while True:
            item = self._text_queue.get()
            if item is _END:
                self._audio_queue.put(_END)
                continue

            if self._interrupt.is_set():
                continue

            t0 = time.monotonic()
            chunks = []
            for chunk in self._voice.synthesize(item, syn_config=syn_config):
                if self._interrupt.is_set():
                    break
                audio = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
                chunks.append(audio)

            if chunks and not self._interrupt.is_set():
                audio_f = np.concatenate(chunks).astype(np.float32) / 32768.0
                if self._volume != 1.0:
                    audio_f *= self._volume
                    np.clip(audio_f, -1.0, 1.0, out=audio_f)
                # Trim leading/trailing silence for tighter speech
                audio_f = _trim_silence(audio_f, self._sample_rate)
                elapsed = (time.monotonic() - t0) * 1000
                self.get_logger().info(f'[TTS] Synthesized in {elapsed:.0f}ms')
                self._audio_queue.put(audio_f)

    # ── Playback thread ──────────────────────────────────────────────────

    def _playback_loop(self) -> None:
        """Read audio arrays from audio_queue, play with sd.play(blocking)."""
        while True:
            audio = self._audio_queue.get()
            if audio is _END:
                continue

            self._interrupt.clear()
            self._speaking_pub.publish(Bool(data=True))
            t0 = time.monotonic()
            chunk_count = 0

            try:
                # Play first chunk
                sd.play(audio, samplerate=self._sample_rate,
                        device=self._device, blocking=True)
                chunk_count += 1

                # Keep playing queued chunks
                while not self._interrupt.is_set():
                    try:
                        audio = self._audio_queue.get(timeout=3.0)
                    except queue.Empty:
                        break
                    if audio is _END:
                        break
                    if not self._interrupt.is_set():
                        sd.play(audio, samplerate=self._sample_rate,
                                device=self._device, blocking=True)
                        chunk_count += 1

                elapsed = (time.monotonic() - t0) * 1000
                self.get_logger().info(
                    f'[TTS] Playback complete ({elapsed:.0f}ms, {chunk_count} chunks)'
                )
            except Exception as e:
                self.get_logger().error(f'TTS playback failed: {e}')
            finally:
                self._speaking_pub.publish(Bool(data=False))

    # ── Helpers ───────────────────────────────────────────────────────────

    def _drain(self) -> None:
        """Clear all queues."""
        for q in (self._text_queue, self._audio_queue):
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break

    def _synthesize_to_array(
        self, text: str, speed: float = None
    ) -> np.ndarray:
        """Synthesize text and return as float32 audio array."""
        rate = 1.0 / speed if speed else self._rate
        syn_config = SynthesisConfig(length_scale=rate)
        chunks = []
        for chunk in self._voice.synthesize(text, syn_config=syn_config):
            audio = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
            chunks.append(audio)
        audio_f = np.concatenate(chunks).astype(np.float32) / 32768.0
        if self._volume != 1.0:
            audio_f *= self._volume
            np.clip(audio_f, -1.0, 1.0, out=audio_f)
        return _trim_silence(audio_f, self._sample_rate)


def _trim_silence(
    audio: np.ndarray, sample_rate: int,
    threshold: float = _SILENCE_THRESH,
    pad_samples: int = _SILENCE_PAD_SAMPLES,
) -> np.ndarray:
    """Trim leading and trailing silence, keeping a small pad for naturalness."""
    # Compute RMS energy in 10ms windows
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


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
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
