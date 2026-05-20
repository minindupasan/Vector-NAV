import sys
sys.path.insert(0, "/home/admin/src/vector_web/.venv/lib/python3.10/site-packages")

import asyncio
import fractions
import json
import ssl
import threading

import av
import cv2
import numpy as np
import rclpy
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

VIDEO_CLOCK_RATE = 90000
VIDEO_TIME_BASE = fractions.Fraction(1, VIDEO_CLOCK_RATE)


def _set_sdp_bitrate(sdp: str, kbps: int) -> str:
    """Inject b=AS:<kbps> into the video m-section so the encoder targets that bitrate."""
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


class _FrameBuffer:
    def __init__(self):
        self._frame: np.ndarray | None = None
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


class CameraVideoTrack(VideoStreamTrack):
    kind = "video"

    def __init__(self, buffer: _FrameBuffer, framerate: int):
        super().__init__()
        self._buffer = buffer
        self._framerate = framerate
        self._pts = 0
        self._frame_duration = VIDEO_CLOCK_RATE // framerate

    async def recv(self) -> av.VideoFrame:
        await self._buffer.wait()

        frame_data = self._buffer.get()
        if frame_data is None:
            img = np.zeros((720, 1280, 3), dtype=np.uint8)
        else:
            img = frame_data

        video_frame = av.VideoFrame.from_ndarray(img, format="bgr24")
        video_frame.pts = self._pts
        video_frame.time_base = VIDEO_TIME_BASE
        self._pts += self._frame_duration
        return video_frame


class WebRTCNode(Node):
    def __init__(self):
        super().__init__("camera_webrtc")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8443)
        self.declare_parameter("cert_file", "/home/admin/src/vector_web/certs/cert.pem")
        self.declare_parameter("key_file", "/home/admin/src/vector_web/certs/key.pem")
        self.declare_parameter("framerate", 14)
        self.declare_parameter("static_dir", "/home/admin/src/vector_web/static")

        self._framerate = self.get_parameter("framerate").value
        self._loop: asyncio.AbstractEventLoop | None = None
        self._buffer = _FrameBuffer()
        self._pcs: set[RTCPeerConnection] = set()

        self.create_subscription(
            CompressedImage,
            "/camera/image/compressed",
            self._image_callback,
            SENSOR_QOS,
        )
        self.get_logger().info("WebRTC node ready, waiting for frames on /camera/image/compressed")

    def set_loop(self, loop: asyncio.AbstractEventLoop):
        self._loop = loop

    def _image_callback(self, msg: CompressedImage):
        if self._loop is None:
            return
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        if bgr is None:
            return
        self._buffer.put(bgr, self._loop)

    async def _handle_offer(self, request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        @pc.on("connectionstatechange")
        async def on_state_change():
            self.get_logger().info(f"WebRTC connection state: {pc.connectionState}")
            if pc.connectionState in ("failed", "closed"):
                await pc.close()
                self._pcs.discard(pc)

        pc.addTrack(CameraVideoTrack(self._buffer, self._framerate))
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}),
        )

    async def run_server(self):
        self._loop = asyncio.get_running_loop()

        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        cert_file = self.get_parameter("cert_file").value
        key_file = self.get_parameter("key_file").value
        static_dir = self.get_parameter("static_dir").value

        ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ssl_ctx.load_cert_chain(cert_file, key_file)

        app = web.Application()
        async def index(_):
            return web.FileResponse(f"{static_dir}/drive.html")

        app.router.add_post("/offer", self._handle_offer)
        app.router.add_get("/", index)
        app.router.add_static("/", static_dir)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, host, port, ssl_context=ssl_ctx)
        await site.start()
        self.get_logger().info(f"WebRTC signaling server: https://{host}:{port}")

        await asyncio.Future()  # run forever

    async def shutdown(self):
        await asyncio.gather(*[pc.close() for pc in self._pcs])
        self._pcs.clear()


def main(args=None):
    rclpy.init(args=args)
    node = WebRTCNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        asyncio.run(node.run_server())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
