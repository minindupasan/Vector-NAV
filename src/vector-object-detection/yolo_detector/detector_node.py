#!/usr/bin/env python3
"""
YOLO11s ROS2 Detection Node
Publishes vision_msgs/Detection2DArray for Nav2 integration
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import BoundingBox2D
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Header

from cv_bridge import CvBridge
import cv2
import time
import threading
import numpy as np

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False
    print("Warning: picamera2 not available, will use test images")

from ultralytics import YOLO


class CameraStream:
    """Threaded camera capture for non-blocking inference"""
    def __init__(self, size=(640, 480)):
        if not PICAMERA_AVAILABLE:
            self.picam2 = None
            self.frame = np.zeros((size[1], size[0], 3), dtype=np.uint8)
            self.stopped = False
            self.lock = threading.Lock()
            return
            
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"size": size, "format": "RGB888"}))
        
        self.picam2.start()
        
        # Enable continuous autofocus with reduced hunting
        self.picam2.set_controls({
            "AfMode": 2,           # Continuous autofocus
            "AfSpeed": 0,          # Normal speed (smoother, less hunting)
            "AfRange": 1,          # Macro/normal range (avoids infinity hunting)
            "AfMetering": 0        # Center-weighted metering
        })
        
        time.sleep(2)
        self.frame = self.picam2.capture_array()
        self.stopped = False
        self.lock = threading.Lock()

    def start(self):
        threading.Thread(target=self._update, daemon=True).start()
        return self

    def _update(self):
        while not self.stopped:
            if self.picam2 is not None:
                frame = self.picam2.capture_array()
                with self.lock:
                    self.frame = frame
            else:
                time.sleep(0.033)  # ~30 Hz dummy rate

    def read(self):
        with self.lock:
            return self.frame.copy()

    def stop(self):
        self.stopped = True
        if self.picam2 is not None:
            self.picam2.stop()


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', '/ros2_ws/models/yolo11s_ncnn_model')
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('camera_frame_id', 'camera_optical_frame')
        self.declare_parameter('publish_rate', 15.0)  # Hz
        self.declare_parameter('publish_image', True)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.imgsz = self.get_parameter('imgsz').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.camera_frame = self.get_parameter('camera_frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_img = self.get_parameter('publish_image').value
        
        # Initialize YOLO model
        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)
        self.class_names = self.model.names  # COCO class names
        
        # Initialize camera
        self.get_logger().info('Initializing camera stream...')
        self.cam = CameraStream(size=(640, 480)).start()
        time.sleep(1.0)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # QoS profiles
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, 
            'detections', 
            qos_reliable
        )
        
        if self.publish_img:
            self.image_pub = self.create_publisher(
                Image, 
                'image_raw', 
                qos_sensor
            )
            self.annotated_pub = self.create_publisher(
                Image, 
                'image_annotated', 
                qos_sensor
            )
        
        # Timer for periodic detection
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.detection_callback)
        
        # FPS tracking
        self.prev_time = time.time()
        self.fps_smooth = 0.0
        
        self.get_logger().info(f'YOLO detector initialized at {publish_rate} Hz')

    def detection_callback(self):
        """Main detection loop callback"""
        try:
            # Capture frame
            frame = self.cam.read()
            
            # Get timestamp
            timestamp = self.get_clock().now().to_msg()
            
            # Run YOLO inference
            results = self.model(frame, imgsz=self.imgsz, conf=self.conf_threshold, verbose=False)[0]
            
            # Create detection message
            det_array = Detection2DArray()
            det_array.header.stamp = timestamp
            det_array.header.frame_id = self.camera_frame
            
            # Process detections
            if results.boxes is not None:
                for box in results.boxes:
                    detection = Detection2D()
                    detection.header = det_array.header
                    
                    # Extract bounding box coordinates
                    xyxy = box.xyxy[0].cpu().numpy()
                    x1, y1, x2, y2 = float(xyxy[0]), float(xyxy[1]), float(xyxy[2]), float(xyxy[3])
                    
                    # Create BoundingBox2D
                    bbox = BoundingBox2D()
                    bbox.center = Pose2D()
                    bbox.center.x = (x1 + x2) / 2.0
                    bbox.center.y = (y1 + y2) / 2.0
                    bbox.center.theta = 0.0
                    bbox.size_x = x2 - x1
                    bbox.size_y = y2 - y1
                    detection.bbox = bbox
                    
                    # Create ObjectHypothesisWithPose
                    hyp = ObjectHypothesisWithPose()
                    class_id = int(box.cls[0])
                    hyp.hypothesis.class_id = self.class_names[class_id]
                    hyp.hypothesis.score = float(box.conf[0])
                    detection.results.append(hyp)
                    
                    det_array.detections.append(detection)
            
            # Publish detections
            self.detection_pub.publish(det_array)
            
            # Publish images if enabled
            if self.publish_img:
                # Raw image
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
                img_msg.header = det_array.header
                self.image_pub.publish(img_msg)
                
                # Annotated image
                annotated = results.plot()
                
                # Add FPS
                curr_time = time.time()
                instant_fps = 1.0 / (curr_time - self.prev_time + 1e-9)
                self.fps_smooth = 0.9 * self.fps_smooth + 0.1 * instant_fps
                self.prev_time = curr_time
                
                cv2.putText(annotated, f'FPS: {self.fps_smooth:.1f}', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='rgb8')
                annotated_msg.header = det_array.header
                self.annotated_pub.publish(annotated_msg)
            
            # Log detection count periodically
            if len(det_array.detections) > 0:
                self.get_logger().debug(f'Detected {len(det_array.detections)} objects')
                
        except Exception as e:
            self.get_logger().error(f'Detection error: {str(e)}')

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down YOLO detector...')
        self.cam.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
