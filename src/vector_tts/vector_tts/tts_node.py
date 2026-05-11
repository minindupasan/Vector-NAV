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
from std_msgs.msg import String, Bool, Float32MultiArray, Float32

import sounddevice as sd
from vector_tts.kokoro_trt import KokoroTRT

import os
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
        self.create_subscription(
            Float32, '/tts/volume', self._on_volume, 10,
        )
        self.create_subscription(
            String, '/tts/target', self._on_target, 10,
        )

        self._target = 'robot'

        self.get_logger().info(
            'TTSNode started\n'
            '  Subscribes : /tts/input, /tts/volume, /tts/target\n'
            '  Publishes  : /tts/speaking, /tts/audio/stream'
        )

    # ── ROS callback ─────────────────────────────────────────────────────

    def _on_target(self, msg: String) -> None:
        self._target = msg.data.lower()
        self.get_logger().info(f'[TTS] Output target updated to {self._target}')

    def _on_volume(self, msg: Float32) -> None:
        self._volume = msg.data
        self.get_logger().info(f'[TTS] Dynamic volume updated to {self._volume:.2f}x')

    def _on_tts_input(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f"[TTS] Received input: {text}")

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
                self._interrupt.clear()  # Ensure next response can play
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
                self.get_logger().info('[TTS] Clearing interrupt flag for new sequence')
                self._interrupt.clear()
                
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
            
            try:
                audio_f = self._tts.synthesize(item, voice=self._voice_name, speed=self._speed)
            except Exception as e:
                self.get_logger().error(f'[TTS] Kokoro synthesis failed for "{item[:40]}...": {e}')
                # Ensure we don't crash the thread. We skip this chunk.
                continue

            if len(audio_f) > 0 and not self._interrupt.is_set():
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
        """Read audio arrays from audio_queue, play continuously through a persistent stream."""
        # ── Hardware Native Config ──
        TARGET_SR = 24000
        TARGET_CHANNELS = 1
        
        playback_device = self._device or 'pulse'
        
        def process_audio(mono_24k):
            """Apply volume and convert to float32."""
            vol = self._volume
            scaled = mono_24k * vol
            np.clip(scaled, -1.0, 1.0, out=scaled)
            return scaled.astype(np.float32)

        try:
            self.get_logger().info(f"[TTS] Opening persistent {TARGET_SR}Hz output: {playback_device}")
            # Use 50ms blocksize for standard stability
            CHUNK_SIZE = int(TARGET_SR * 0.05) # 1200 samples
            with sd.OutputStream(
                samplerate=TARGET_SR,
                channels=TARGET_CHANNELS,
                dtype='float32',
                device=playback_device,
                latency='high',
                blocksize=CHUNK_SIZE
            ) as stream:
                stream.start()
                
                # 50ms of silence
                idle_silence = np.zeros((CHUNK_SIZE, 1), dtype=np.float32)

                while True:
                    try:
                        # Small timeout to check for interrupt or END frequently
                        audio = self._audio_queue.get(timeout=0.005)
                    except queue.Empty:
                        # Feed silence to keep the buffer saturated
                        stream.write(idle_silence)
                        continue

                    if audio is _END:
                        self._speaking_pub.publish(Bool(data=False))
                        continue

                    self._interrupt.clear()
                    
                    # Only play to local speaker if target is 'robot'
                    if self._target == 'robot':
                        self._speaking_pub.publish(Bool(data=True))
                        t0 = time.monotonic()
                        
                        # Process whole audio and make it 2D (N, 1) for sounddevice Mono
                        audio_processed = process_audio(audio)
                        audio_processed = np.expand_dims(audio_processed, axis=1)
                        
                        # Play the chunk
                        # We process in 50ms blocks
                        for i in range(0, len(audio_processed), CHUNK_SIZE):
                            if self._interrupt.is_set():
                                break
                            chunk = audio_processed[i:i+CHUNK_SIZE]
                            # Ensure we write exactly CHUNK_SIZE for the fixed blocksize
                            if chunk.shape[0] == CHUNK_SIZE:
                                stream.write(chunk)
                            else:
                                # Pad the last small chunk with silence if needed
                                padded = np.zeros((CHUNK_SIZE, 1), dtype=np.float32)
                                padded[:chunk.shape[0], :] = chunk
                                stream.write(padded)

                        if len(audio) > CHUNK_SIZE:
                            elapsed = (time.monotonic() - t0) * 1000
                            self.get_logger().info(f'[TTS] Played chunk in {elapsed:.0f}ms')
                    else:
                        # Just log that we are skipping local playback
                        self.get_logger().info(f"[TTS] Skipping local playback (target={self._target})", once=True)

        except Exception as e:
            self.get_logger().error(f'TTS playback stream failed: {e}')
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
