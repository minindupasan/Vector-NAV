#!/usr/bin/env python3
"""Standalone camera → WebRTC stream. No ROS2 dependency."""

import os
os.environ.setdefault("EGL_PLATFORM", "surfaceless")

import sys
sys.path.insert(0, "/home/admin/src/vector_web/.venv/lib/python3.10/site-packages")

import asyncio
import fractions
import json
import signal
import ssl
import threading

import av
import cv2
import numpy as np
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack

# ── Config ────────────────────────────────────────────────────────────────────
SENSOR_MODE     = 1
CAPTURE_W       = 2304
CAPTURE_H       = 1296
CAPTURE_FPS     = 56
WEBRTC_W        = 1280
WEBRTC_H        = 720
JPEG_QUALITY    = 85
WEBRTC_KBPS     = 2000
HOST            = "0.0.0.0"
PORT            = 8443
CERT_FILE       = "/home/admin/src/vector_web/certs/cert.pem"
KEY_FILE        = "/home/admin/src/vector_web/certs/key.pem"
STATIC_DIR      = "/home/admin/src/vector_web/static"

VIDEO_CLOCK_RATE   = 90000
VIDEO_TIME_BASE    = fractions.Fraction(1, VIDEO_CLOCK_RATE)
FRAME_DURATION     = VIDEO_CLOCK_RATE // CAPTURE_FPS

# ── Frame buffer ──────────────────────────────────────────────────────────────
class _FrameBuffer:
    def __init__(self):
        self._frame = None
        self._lock  = threading.Lock()
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
        vf.pts       = self._pts
        vf.time_base = VIDEO_TIME_BASE
        self._pts   += FRAME_DURATION
        return vf


# ── SDP bitrate injection ─────────────────────────────────────────────────────
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
        self._buf  = buf
        self._loop: asyncio.AbstractEventLoop | None = None

        pipeline = (
            f"nvarguscamerasrc sensor-mode={SENSOR_MODE} num-buffers=-1 "
            f"tnr-mode=1 tnr-strength=0.5 ee-mode=2 eestrength=0.5 ! "
            f"video/x-raw(memory:NVMM),width={CAPTURE_W},height={CAPTURE_H},framerate={CAPTURE_FPS}/1 ! "
            f"nvvidconv ! "
            f"video/x-raw,width={WEBRTC_W},height={WEBRTC_H},format=BGRx ! "
            f"appsink drop=1 max-buffers=1"
        )
        self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            raise RuntimeError("Failed to open GStreamer/argus pipeline")

        self._thread = threading.Thread(target=self._loop_fn, daemon=True)
        self._thread.start()
        print(f"[camera] {CAPTURE_W}x{CAPTURE_H} @ {CAPTURE_FPS}fps → WebRTC {WEBRTC_W}x{WEBRTC_H}", flush=True)

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    def _loop_fn(self):
        while True:
            ret, bgrx = self._cap.read()
            if not ret or bgrx is None:
                continue
            bgr = np.ascontiguousarray(bgrx[:, :, :3])
            if self._loop is not None:
                self._buf.put(bgr, self._loop)

    def release(self):
        if self._cap.isOpened():
            self._cap.release()


# ── WebRTC signaling server ───────────────────────────────────────────────────
async def run_server(buf: _FrameBuffer, cam: CameraCapture):
    loop = asyncio.get_running_loop()
    cam.set_loop(loop)

    pcs: set[RTCPeerConnection] = set()

    async def handle_offer(request: web.Request) -> web.Response:
        params = await request.json()
        offer  = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        pc     = RTCPeerConnection()
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

    async def index(_):
        return web.FileResponse(f"{STATIC_DIR}/drive.html")

    app = web.Application()
    app.router.add_post("/offer", handle_offer)
    app.router.add_get("/", index)
    app.router.add_static("/", STATIC_DIR)

    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_ctx.load_cert_chain(CERT_FILE, KEY_FILE)

    runner = web.AppRunner(app)
    await runner.setup()
    await web.TCPSite(runner, HOST, PORT, ssl_context=ssl_ctx).start()
    print(f"[webrtc] https://{HOST}:{PORT}", flush=True)

    stop = loop.create_future()
    loop.add_signal_handler(signal.SIGTERM, stop.set_result, None)
    loop.add_signal_handler(signal.SIGINT,  stop.set_result, None)
    await stop

    await asyncio.gather(*[pc.close() for pc in pcs])
    await runner.cleanup()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    buf = _FrameBuffer()
    cam = CameraCapture(buf)
    try:
        asyncio.run(run_server(buf, cam))
    finally:
        cam.release()


if __name__ == "__main__":
    main()
