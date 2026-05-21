#!/usr/bin/env python3
"""Pi-side camera → WebRTC stream.

Captures from the IMX708 via Picamera2/libcamera, scales to WebRTC size in
the PiSP ISP (zero-copy), and serves a /offer SDP endpoint over HTTPS.

The Jetson web app (CAMERA_WEBRTC_URL) proxies SDP offers here over the
LAN; the actual media (RTP) flows directly browser ↔ Pi.
"""

import asyncio
import fractions
import json
import os
import signal
import ssl
import threading

import av
import numpy as np
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from libcamera import Transform, controls
from picamera2 import Picamera2

# ── Config ────────────────────────────────────────────────────────────────────
SENSOR_RAW_W   = int(os.environ.get("SENSOR_RAW_W", "2304"))
SENSOR_RAW_H   = int(os.environ.get("SENSOR_RAW_H", "1296"))
WEBRTC_W       = int(os.environ.get("WEBRTC_W", "1280"))
WEBRTC_H       = int(os.environ.get("WEBRTC_H", "720"))
CAPTURE_FPS    = int(os.environ.get("CAPTURE_FPS", "56"))
WEBRTC_KBPS    = int(os.environ.get("WEBRTC_KBPS", "2000"))
HOST           = os.environ.get("HOST", "0.0.0.0")
PORT           = int(os.environ.get("PORT", "8443"))
CERT_FILE      = os.environ.get("CERT_FILE", "/etc/vector/cert.pem")
KEY_FILE       = os.environ.get("KEY_FILE", "/etc/vector/key.pem")

VIDEO_CLOCK_RATE = 90000
VIDEO_TIME_BASE  = fractions.Fraction(1, VIDEO_CLOCK_RATE)
FRAME_DURATION   = VIDEO_CLOCK_RATE // CAPTURE_FPS


# ── Frame buffer ──────────────────────────────────────────────────────────────
class _FrameBuffer:
    def __init__(self):
        self._frame = None
        self._lock = threading.Lock()
        self._event = asyncio.Event()

    def put(self, frame: np.ndarray, loop: asyncio.AbstractEventLoop):
        with self._lock:
            self._frame = frame
        loop.call_soon_threadsafe(self._event.set)

    def get(self) -> np.ndarray | None:
        with self._lock:
            return self._frame

    async def wait(self):
        await self._event.wait()
        self._event.clear()


# ── WebRTC video track ────────────────────────────────────────────────────────
class CameraVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, buf: _FrameBuffer):
        super().__init__()
        self._buf = buf
        self._pts = 0

    async def recv(self) -> av.VideoFrame:
        await self._buf.wait()
        img = self._buf.get()
        if img is None:
            img = np.zeros((WEBRTC_H, WEBRTC_W, 3), dtype=np.uint8)
        vf = av.VideoFrame.from_ndarray(img, format="bgr24")
        vf.pts = self._pts
        vf.time_base = VIDEO_TIME_BASE
        self._pts += FRAME_DURATION
        return vf


def _set_sdp_bitrate(sdp: str, kbps: int) -> str:
    lines = sdp.split("\r\n")
    out, in_video, injected = [], False, False
    for line in lines:
        if line.startswith("m=video"):
            in_video, injected = True, False
        elif line.startswith("m="):
            in_video = False
        out.append(line)
        if in_video and not injected and line.startswith("c="):
            out.append(f"b=AS:{kbps}")
            injected = True
    return "\r\n".join(out)


# ── Camera capture ────────────────────────────────────────────────────────────
class CameraCapture:
    def __init__(self, buf: _FrameBuffer):
        self._buf = buf
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop = threading.Event()

        self._picam = Picamera2()
        frame_us = int(1_000_000 / CAPTURE_FPS)
        config = self._picam.create_video_configuration(
            main={"size": (WEBRTC_W, WEBRTC_H), "format": "RGB888"},
            raw={"size": (SENSOR_RAW_W, SENSOR_RAW_H)},
            transform=Transform(hflip=1, vflip=1),
            controls={
                "FrameDurationLimits": (frame_us, frame_us),
                # Auto exposure / gain / white balance
                "AeEnable": True,
                "AeExposureMode": controls.AeExposureModeEnum.Normal,
                "AeMeteringMode": controls.AeMeteringModeEnum.CentreWeighted,
                "AwbEnable": True,
                "AwbMode": controls.AwbModeEnum.Auto,
                # Continuous autofocus (IMX708 has PDAF)
                "AfMode": controls.AfModeEnum.Continuous,
                "AfRange": controls.AfRangeEnum.Normal,
                "AfSpeed": controls.AfSpeedEnum.Fast,
                # Image quality
                "NoiseReductionMode": controls.draft.NoiseReductionModeEnum.Fast,
                "Sharpness": 1.0,
                "Contrast": 1.0,
                "Saturation": 1.0,
                "Brightness": 0.0,
            },
            buffer_count=4,
            queue=False,
        )
        self._picam.configure(config)
        self._picam.start()
        print(f"[camera] IMX708 raw {SENSOR_RAW_W}x{SENSOR_RAW_H} @ "
              f"{CAPTURE_FPS}fps → WebRTC {WEBRTC_W}x{WEBRTC_H}", flush=True)

        self._thread = threading.Thread(target=self._loop_fn, daemon=True)
        self._thread.start()

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    def _loop_fn(self):
        while not self._stop.is_set():
            frame = self._picam.capture_array("main")
            if self._loop is not None:
                self._buf.put(frame, self._loop)

    def release(self):
        self._stop.set()
        try:
            self._picam.stop()
        except Exception:
            pass


# ── WebRTC signaling server ───────────────────────────────────────────────────
async def run_server(buf: _FrameBuffer, cam: CameraCapture):
    loop = asyncio.get_running_loop()
    cam.set_loop(loop)

    pcs: set[RTCPeerConnection] = set()

    async def handle_offer(request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        pc = RTCPeerConnection()
        pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_state():
            print(f"[webrtc] {pc.connectionState}", flush=True)
            if pc.connectionState in ("failed", "closed"):
                await pc.close()
                pcs.discard(pc)

        pc.addTrack(CameraVideoTrack(buf))
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(
            RTCSessionDescription(sdp=_set_sdp_bitrate(answer.sdp, WEBRTC_KBPS), type=answer.type)
        )
        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}),
        )

    async def health(_):
        return web.json_response({"ok": True})

    app = web.Application()
    app.router.add_post("/offer", handle_offer)
    app.router.add_get("/health", health)

    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(CERT_FILE, KEY_FILE)

    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, HOST, PORT, ssl_context=ssl_ctx).start()
    print(f"[webrtc] https://{HOST}:{PORT}", flush=True)

    stop = loop.create_future()
    loop.add_signal_handler(signal.SIGTERM, stop.set_result, None)
    loop.add_signal_handler(signal.SIGINT, stop.set_result, None)
    await stop

    await asyncio.gather(*[pc.close() for pc in pcs])
    await runner.cleanup()


def main():
    buf = _FrameBuffer()
    cam = CameraCapture(buf)
    try:
        asyncio.run(run_server(buf, cam))
    finally:
        cam.release()


if __name__ == "__main__":
    main()
