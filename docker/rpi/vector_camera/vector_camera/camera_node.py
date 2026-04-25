#!/usr/bin/env python3
"""ROS2 camera publisher for RPi Camera Module 3 using rpicam-vid."""

import subprocess
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Declare parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('jpeg_quality', 80)

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.framerate = self.get_parameter('framerate').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # QoS: best effort, keep last 1
        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', camera_qos)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', camera_qos)
        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage, 'image_raw/compressed', camera_qos
            )

        # Camera info
        self.camera_info_msg = self._build_camera_info()

        # Start rpicam-vid outputting MJPEG to stdout
        cmd = [
            'rpicam-vid',
            '-t', '0',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.framerate),
            '--codec', 'mjpeg',
            '-o', '-',
            '-n',  # no preview
        ]
        self.get_logger().info(f'Starting: {" ".join(cmd)}')
        self.process = subprocess.Popen(
            cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )

        # Frame buffer
        self._frame = None
        self._frame_lock = threading.Lock()
        self._frame_jpeg = None
        self.frame_count = 0

        # Reader thread: parse MJPEG stream from rpicam-vid
        self._reader_thread = threading.Thread(target=self._read_frames, daemon=True)
        self._reader_thread.start()

        # Timer for publishing
        period = 1.0 / self.framerate
        self.timer = self.create_timer(period, self.publish_frame)

        self.get_logger().info(
            f'Camera started: {self.width}x{self.height} @ {self.framerate}fps'
        )

    def _read_frames(self):
        """Read MJPEG frames from rpicam-vid stdout."""
        buf = b''
        stream = self.process.stdout

        while rclpy.ok():
            chunk = stream.read(4096)
            if not chunk:
                break
            buf += chunk

            # MJPEG: each frame starts with FFD8 and ends with FFD9
            while True:
                start = buf.find(b'\xff\xd8')
                if start == -1:
                    buf = b''
                    break
                end = buf.find(b'\xff\xd9', start + 2)
                if end == -1:
                    # Trim everything before the start marker
                    buf = buf[start:]
                    break

                # Extract complete JPEG frame
                jpeg_data = buf[start:end + 2]
                buf = buf[end + 2:]

                # Decode to BGR
                frame = cv2.imdecode(
                    np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR
                )
                if frame is not None:
                    with self._frame_lock:
                        self._frame = frame
                        self._frame_jpeg = jpeg_data

    def publish_frame(self):
        with self._frame_lock:
            frame = self._frame
            jpeg_data = self._frame_jpeg
            self._frame = None
            self._frame_jpeg = None

        if frame is None:
            return

        now = self.get_clock().now().to_msg()

        # Publish raw image
        img_msg = Image()
        img_msg.header.stamp = now
        img_msg.header.frame_id = 'camera_link'
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = False
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()
        self.image_pub.publish(img_msg)

        # Publish compressed (use original JPEG from rpicam-vid)
        if self.publish_compressed and jpeg_data is not None:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = now
            compressed_msg.header.frame_id = 'camera_link'
            compressed_msg.format = 'jpeg'
            compressed_msg.data = jpeg_data
            self.compressed_pub.publish(compressed_msg)

        # Publish camera info
        self.camera_info_msg.header.stamp = now
        self.camera_info_msg.header.frame_id = 'camera_link'
        self.info_pub.publish(self.camera_info_msg)

        self.frame_count += 1
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')

    def _build_camera_info(self):
        msg = CameraInfo()
        msg.width = self.width
        msg.height = self.height
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = float(self.width)
        fy = float(self.width)
        cx = float(self.width) / 2.0
        cy = float(self.height) / 2.0
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def destroy_node(self):
        if hasattr(self, 'process') and self.process:
            self.process.terminate()
            self.process.wait()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
