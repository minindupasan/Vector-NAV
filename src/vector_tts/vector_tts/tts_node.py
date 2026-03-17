"""
VECTOR TTS — ROS2 Node
=======================
Text-to-speech using Piper TTS (ONNX, CPU).
Subscribes to /tts/input, synthesizes audio, plays through sounddevice.
Publishes /tts/speaking (Bool) to gate STT during playback.

Architecture: two-thread pipeline
  1. Synthesizer thread: reads text from queue, runs Piper, pushes audio arrays
  2. Playback thread: reads audio arrays, writes to one continuous OutputStream
  Synthesis runs DURING playback so the next sentence is always ready.

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool

import sounddevice as sd
from piper import PiperVoice
from piper.config import SynthesisConfig

import os
os.environ.setdefault('SDL_AUDIODRIVER', 'pulseaudio')
sd.default.extra_settings = None

_END = object()


class TTSNode(Node):

    _GREETING_TAG = '[greeting]'
    _END_TAG = '[end]'

    def __init__(self):
        super().__init__('tts_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter(
            'model_path',
            '/home/jetson/vector_nav/models/tts/en_US-lessac-high.onnx',
        )
        self.declare_parameter('device', '')
        self.declare_parameter('rate', 0.85)
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
            f'Piper TTS ready  sample_rate={self._sample_rate}  '
            f'device={self._device or "default"}'
        )

        # ── Pre-cache greeting ────────────────────────────────────────────
        self._greeting_audio = self._pre_synthesize(
            'Hi, what can I do for you?', speed=1.25
        )
        self.get_logger().info('Greeting audio pre-cached')

        # ── Queues ────────────────────────────────────────────────────────
        # text_queue: ROS callback → synthesizer thread
        # audio_queue: synthesizer thread → playback thread
        self._text_queue = queue.Queue()
        self._audio_queue = queue.Queue()
        self._interrupt = threading.Event()

        # ── Threads ───────────────────────────────────────────────────────
        threading.Thread(target=self._synth_loop, daemon=True).start()
        threading.Thread(target=self._playback_loop, daemon=True).start()

        # ── Publishers ──────────────────────────────────────────────────────
        self._speaking_pub = self.create_publisher(Bool, '/tts/speaking', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        cb = ReentrantCallbackGroup()
        self.create_subscription(
            String, '/tts/input', self._on_tts_input, 10, callback_group=cb
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
        if text == self._END_TAG:
            self._text_queue.put(_END)
        elif text == self._GREETING_TAG:
            # Greeting bypasses synthesis — goes straight to audio queue
            self._audio_queue.put(self._greeting_audio)
            self._text_queue.put(_END)
        else:
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
                elapsed = (time.monotonic() - t0) * 1000
                self.get_logger().info(f'[TTS] Synthesized in {elapsed:.0f}ms')
                self._audio_queue.put(audio_f)

    # ── Playback thread ──────────────────────────────────────────────────

    def _playback_loop(self) -> None:
        """Read audio arrays from audio_queue, play through one stream."""
        while True:
            # Block until first audio arrives
            audio = self._audio_queue.get()
            if audio is _END:
                continue

            self._speaking_pub.publish(Bool(data=True))
            self._interrupt.clear()
            t0 = time.monotonic()

            try:
                with sd.OutputStream(
                    samplerate=self._sample_rate,
                    channels=1,
                    dtype='float32',
                    device=self._device,
                ) as stream:
                    # Play first chunk
                    stream.write(audio.reshape(-1, 1))

                    # Keep playing until end marker or timeout
                    while not self._interrupt.is_set():
                        try:
                            audio = self._audio_queue.get(timeout=3.0)
                        except queue.Empty:
                            break
                        if audio is _END:
                            break
                        stream.write(audio.reshape(-1, 1))

                elapsed = (time.monotonic() - t0) * 1000
                self.get_logger().info(f'[TTS] Playback complete ({elapsed:.0f}ms)')
            except Exception as e:
                self.get_logger().error(f'TTS playback failed: {e}')
            finally:
                self._speaking_pub.publish(Bool(data=False))
                self._drain()

    # ── Helpers ───────────────────────────────────────────────────────────

    def _drain(self) -> None:
        for q in (self._text_queue, self._audio_queue):
            while not q.empty():
                try:
                    q.get_nowait()
                except queue.Empty:
                    break

    def _pre_synthesize(self, text: str, speed: float = 1.0) -> np.ndarray:
        syn_config = SynthesisConfig(length_scale=1.0 / speed)
        chunks = []
        for chunk in self._voice.synthesize(text, syn_config=syn_config):
            audio = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
            chunks.append(audio)
        audio_f = np.concatenate(chunks).astype(np.float32) / 32768.0
        if self._volume != 1.0:
            audio_f *= self._volume
            np.clip(audio_f, -1.0, 1.0, out=audio_f)
        return audio_f

    def interrupt(self):
        self._interrupt.set()
        sd.stop()


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
