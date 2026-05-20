import os
os.environ.setdefault("EGL_PLATFORM", "surfaceless")

import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class IMX708CameraNode(Node):
    def __init__(self):
        super().__init__('imx708_camera')

        self.declare_parameter('sensor_mode', 1)
        self.declare_parameter('capture_width', 2304)
        self.declare_parameter('capture_height', 1296)
        self.declare_parameter('framerate', 56)
        self.declare_parameter('jpeg_quality', 85)

        mode      = self.get_parameter('sensor_mode').value
        self._w   = self.get_parameter('capture_width').value
        self._h   = self.get_parameter('capture_height').value
        framerate = self.get_parameter('framerate').value
        quality   = self.get_parameter('jpeg_quality').value
        self._encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]

        self._pub = self.create_publisher(CompressedImage, '/camera/image/compressed', 1)

        # nvvidconv does color conversion in hardware (on Jetson VIC engine).
        # We get BGR frames directly — no CPU debayer, no raw Image overhead.
        pipeline = (
            f"nvarguscamerasrc sensor-mode={mode} num-buffers=-1 ! "
            f"video/x-raw(memory:NVMM),width={self._w},height={self._h},framerate={framerate}/1 ! "
            f"nvvidconv ! "
            f"video/x-raw,format=BGRx ! "
            f"videoconvert ! "
            f"video/x-raw,format=BGR ! "
            f"appsink drop=1 max-buffers=1"
        )

        self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self._cap.isOpened():
            self.get_logger().error("Failed to open GStreamer/argus pipeline")
            return

        self._latest: bytes | None = None
        self._lock = threading.Lock()

        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()

        self.get_logger().info(
            f"Publishing {self._w}x{self._h} @ {framerate}fps "
            f"(argus sensor-mode={mode})"
        )
        self.create_timer(1.0 / framerate, self._publish)

    def _capture_loop(self):
        while rclpy.ok():
            ret, bgr = self._cap.read()
            if not ret or bgr is None:
                self.get_logger().warn("Failed to capture frame", throttle_duration_sec=5.0)
                continue
            ok, jpeg = cv2.imencode('.jpg', bgr, self._encode_params)
            if not ok:
                continue
            with self._lock:
                self._latest = jpeg.tobytes()

    def _publish(self):
        with self._lock:
            data = self._latest
            self._latest = None
        if data is None:
            return

        cm = CompressedImage()
        cm.header.stamp = self.get_clock().now().to_msg()
        cm.header.frame_id = 'camera'
        cm.format = 'jpeg'
        cm.data = data
        self._pub.publish(cm)

    def destroy_node(self):
        if hasattr(self, '_cap') and self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMX708CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
