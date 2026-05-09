"""
VECTOR TTS — ROS2 Node
=======================
Text-to-speech using Kokoro TTS with TensorRT acceleration on Jetson Orin.

Architecture: two-thread pipeline with barge-in support
  1. Synthesizer thread: reads text from queue, runs Kokoro TRT, pushes audio arrays
  2. Playback thread: reads audio arrays, writes to one continuous OutputStream
  Synthesis runs DURING playback so the next sentence is always ready.

Special tags:
  [greeting]   — play pre-cached greeting instantly
  [end]        — signal end of response (close audio stream)
  [interrupt]  — barge-in: stop current playback immediately

Parameters
----------
  voice        (string)  Kokoro voice name (default 'af_heart')
  speed        (float)   Speech speed (0.5–2.0, default 1.0)
  device       (string)  Audio output device name ('' = system default)
  volume       (float)   Volume multiplier (0.0–2.0, default 1.0)
  engine_dir   (string)  Path to Kokoro TRT engine directory

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
from std_msgs.msg import String, Bool, Float32MultiArray

import sounddevice as sd
from vector_tts.kokoro_trt import KokoroTRT

import os
os.environ.setdefault('SDL_AUDIODRIVER', 'pulseaudio')
sd.default.extra_settings = None

_END = object()
_INTERRUPT = object()

# Minimum RMS energy to keep (trim silence below this)
_SILENCE_THRESH = 0.003
# Samples of silence to keep at boundaries for natural breathing room
_SILENCE_PAD_SAMPLES = 4800  # ~200ms at 24000Hz


class TTSNode(Node):

    _GREETING_TAG = '[greeting]'
    _END_TAG = '[end]'
    _INTERRUPT_TAG = '[interrupt]'

    def __init__(self):
        super().__init__('tts_node')

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter('voice', 'af_heart')
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('device', '')
        self.declare_parameter('volume', 1.0)
        self.declare_parameter('engine_dir', '/home/admin/models/kokoro_trt')

        self._voice_name = self.get_parameter('voice').get_parameter_value().string_value
        self._speed = self.get_parameter('speed').get_parameter_value().double_value
        self._device = self.get_parameter('device').get_parameter_value().string_value or None
        self._volume = self.get_parameter('volume').get_parameter_value().double_value
        engine_dir = self.get_parameter('engine_dir').get_parameter_value().string_value

        # ── Load Kokoro TRT ───────────────────────────────────────────────
        self.get_logger().info(f'Loading Kokoro TRT from {engine_dir}...')
        self._tts = KokoroTRT(engine_dir)
        self._sample_rate = self._tts.sample_rate
        self.get_logger().info(
            f'Kokoro TRT ready  voice={self._voice_name}  '
            f'speed={self._speed}  device={self._device or "default"}'
        )

        # ── Pre-cache common responses ────────────────────────────────────
        self.get_logger().info('Pre-caching common responses...')
        self._cached = {}
        self._cached['greeting'] = self._synthesize_to_array(
            'Hi, what can I do for you?'
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
        self._dropping_stale = False

        # ── Threads ───────────────────────────────────────────────────────
        threading.Thread(target=self._synth_loop, daemon=True).start()
        threading.Thread(target=self._playback_loop, daemon=True).start()

        # ── Publishers ──────────────────────────────────────────────────────
        self._speaking_pub = self.create_publisher(Bool, '/tts/speaking', 10)
        self._audio_pub    = self.create_publisher(Float32MultiArray, '/tts/audio/stream', 10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            String, '/tts/input', self._on_tts_input, 10,
        )

        self.get_logger().info(
            'TTSNode started\n'
            '  Subscribes : /tts/input\n'
            '  Publishes  : /tts/speaking, /tts/audio/stream'
        )

    # ── ROS callback ─────────────────────────────────────────────────────

    def _on_tts_input(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        if text == self._INTERRUPT_TAG:
            self.get_logger().info('[TTS] Barge-in interrupt!')
            self._interrupt.set()
            self._dropping_stale = True
            sd.stop()
            self._drain()
            return

        if text == self._END_TAG:
            if self._dropping_stale:
                self.get_logger().info('[TTS] Stale [end] dropped, ready for new input')
                self._dropping_stale = False
                return
            self._text_queue.put(_END)
        elif text == self._GREETING_TAG:
            self._interrupt.clear()
            self._audio_queue.put(self._cached['greeting'])
            self._audio_queue.put(_END)
        else:
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
        while True:
            item = self._text_queue.get()
            if item is _END:
                self._audio_queue.put(_END)
                continue

            if self._interrupt.is_set():
                continue

            t0 = time.monotonic()

            audio_f = self._tts.synthesize(item, voice=self._voice_name, speed=self._speed)

            if len(audio_f) > 0 and not self._interrupt.is_set():
                if self._volume != 1.0:
                    audio_f *= self._volume
                    np.clip(audio_f, -1.0, 1.0, out=audio_f)
                audio_f = _trim_silence(audio_f, self._sample_rate)
                elapsed = (time.monotonic() - t0) * 1000
                self.get_logger().info(f'[TTS] Synthesized in {elapsed:.0f}ms')
                
                # Publish to ROS
                msg = Float32MultiArray()
                msg.data = audio_f.tolist()
                self._audio_pub.publish(msg)
                
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
                sd.play(audio, samplerate=self._sample_rate,
                        device=self._device, blocking=True)
                chunk_count += 1

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

    def _synthesize_to_array(self, text: str) -> np.ndarray:
        """Synthesize text and return as float32 audio array."""
        audio_f = self._tts.synthesize(text, voice=self._voice_name, speed=self._speed)
        if self._volume != 1.0:
            audio_f = audio_f * self._volume
            np.clip(audio_f, -1.0, 1.0, out=audio_f)
        return _trim_silence(audio_f, self._sample_rate)


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
