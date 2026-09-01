import json
import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class RoadSignDetector(Node):
    """Run the trained YOLO model and publish the localization JSON contract."""

    def __init__(self):
        super().__init__('road_sign_detector')
        self.declare_parameter('image_topic', '/oak/rgb/image_raw')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        # This must be an RGB-registered depth stream so RGB detection pixels
        # and depth pixels refer to the same image coordinates.
        self.declare_parameter('depth_topic', '/oak/stereo/image_raw')
        self.declare_parameter('depth_camera_info_topic', '/oak/stereo/camera_info')
        self.declare_parameter('detection_topic', '/traffic/detection')
        self.declare_parameter('model_path', self._default_model_path())
        self.declare_parameter('confidence', 0.50)
        self.declare_parameter('inference_size', 1024)
        self.declare_parameter('sign_height_m', 0.30)
        self.declare_parameter('focal_length_px', 700.0)
        self.declare_parameter('depth_timeout_s', 0.15)
        self.declare_parameter('depth_scale', 0.001)  # 16UC1 millimetres -> metres
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', '/traffic/detection/debug')

        self.bridge = CvBridge()
        self.model = None
        self.camera_focal_px = float(self.get_parameter('focal_length_px').value)
        self.depth_intrinsics = None
        self.latest_depth = None
        self.latest_depth_stamp = None
        self.sign_height_m = float(self.get_parameter('sign_height_m').value)
        self._load_model()

        image_topic = self.get_parameter('image_topic').value
        self.detection_pub = self.create_publisher(String, self.get_parameter('detection_topic').value, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 2)
        self.info_sub = self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.info_callback, 2)
        self.depth_sub = self.create_subscription(Image, self.get_parameter('depth_topic').value, self.depth_callback, 2)
        self.depth_info_sub = self.create_subscription(CameraInfo, self.get_parameter('depth_camera_info_topic').value, self.depth_info_callback, 2)
        self.debug_pub = None
        if self.get_parameter('publish_debug_image').value:
            self.debug_pub = self.create_publisher(Image, self.get_parameter('debug_image_topic').value, 2)
        self.get_logger().info(f'Road-sign detector subscribed to {image_topic}')

    @staticmethod
    def _default_model_path():
        try:
            from ament_index_python.packages import get_package_share_directory
            return str(Path(get_package_share_directory('bfmc_sign_detection')) / 'models' / 'bfmc_signs_best.pt')
        except Exception:
            return str(Path(__file__).parents[2] / 'models' / 'bfmc_signs_best.pt')

    def _load_model(self):
        try:
            from ultralytics import YOLO
            path = Path(self.get_parameter('model_path').value).expanduser()
            self.model = YOLO(str(path), task='detect')
            self.get_logger().info(f'Loaded road-sign model: {path}')
        except Exception as exc:
            self.get_logger().error(f'Unable to load YOLO model: {exc}')

    def info_callback(self, msg):
        if msg.k[0] > 0.0:
            self.camera_focal_px = float(msg.k[0])

    def depth_info_callback(self, msg):
        if msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self.depth_intrinsics = (float(msg.k[0]), float(msg.k[4]), float(msg.k[2]), float(msg.k[5]))

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as exc:
            self.get_logger().warning(f'Depth conversion failed: {exc}')

    def image_callback(self, msg):
        if self.model is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            result = self.model.predict(source=frame, conf=float(self.get_parameter('confidence').value),
                                        imgsz=int(self.get_parameter('inference_size').value),
                                        verbose=False, device='0')[0]
            best = self._best_detection(result)
            if best is None:
                return
            class_id, confidence, x1, y1, x2, y2 = best
            names = result.names
            sign = str(names[int(class_id)]).strip().lower().replace(' ', '_')
            height_px = max(1.0, y2 - y1)
            distance_m, point_3d = self._range_from_depth(x1, y1, x2, y2, msg, frame.shape)
            range_source = 'stereo' if point_3d is not None else 'bbox'
            payload = {'sign': sign, 'distance_m': round(distance_m, 3),
                       'confidence': round(float(confidence), 4),
                       'range_source': range_source,
                       'bbox': [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                       'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}}
            out = String()
            out.data = json.dumps(payload, separators=(',', ':'))
            self.detection_pub.publish(out)
            if self.debug_pub is not None:
                debug = frame.copy()
                cv2.rectangle(debug, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(debug, f'{sign} {confidence:.2f} {distance_m:.2f}m ({range_source})', (int(x1), max(20, int(y1)-8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding='bgr8'))
        except Exception as exc:
            self.get_logger().warning(f'Road-sign inference failed: {exc}')

    def _range_from_depth(self, x1, y1, x2, y2, image_msg, rgb_shape):
        """Backproject a robust depth sample from the detection ROI into 3-D.

        The depth stream is expected to be RGB-registered. For an aligned
        depth image, the median of the inner ROI rejects background pixels and
        holes; X/Y/Z are then recovered with the pinhole camera model.
        """
        fallback = self.sign_height_m * self.camera_focal_px / max(1.0, y2 - y1)
        if self.latest_depth is None or self.latest_depth_stamp is None:
            return fallback, None
        stamp = image_msg.header.stamp.sec + image_msg.header.stamp.nanosec * 1e-9
        if abs(stamp - self.latest_depth_stamp) > float(self.get_parameter('depth_timeout_s').value):
            return fallback, None
        depth = self.latest_depth
        if depth.ndim != 2 or depth.size == 0:
            return fallback, None
        # Scale RGB coordinates if the registered depth stream has a different
        # raster size (common with stereo low-resolution output).
        sx = depth.shape[1] / float(rgb_shape[1])
        sy = depth.shape[0] / float(rgb_shape[0])
        xa, xb = sorted((max(0, int(x1 * sx)), min(depth.shape[1], int(x2 * sx))))
        ya, yb = sorted((max(0, int(y1 * sy)), min(depth.shape[0], int(y2 * sy))))
        if xb <= xa or yb <= ya:
            return fallback, None
        # Ignore the outer border, where the box often includes background.
        roi = depth[ya + (yb-ya)//4: ya + 3*(yb-ya)//4,
                    xa + (xb-xa)//4: xa + 3*(xb-xa)//4]
        scale = 1.0 if np.issubdtype(depth.dtype, np.floating) else float(self.get_parameter('depth_scale').value)
        values = roi.astype(np.float32) * scale
        values = values[np.isfinite(values) & (values > 0.05) & (values < 50.0)]
        if values.size < 5:
            return fallback, None
        z = float(np.median(values))
        fx, fy, cx, cy = self.depth_intrinsics or (self.camera_focal_px, self.camera_focal_px,
                                                     rgb_shape[1] / 2.0, rgb_shape[0] / 2.0)
        u = ((x1 + x2) / 2.0) * sx
        v = ((y1 + y2) / 2.0) * sy
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        return math.sqrt(x*x + y*y + z*z), (x, y, z)

    @staticmethod
    def _best_detection(result):
        if result.boxes is None or len(result.boxes) == 0:
            return None
        idx = int(result.boxes.conf.argmax().item())
        box = result.boxes.xyxy[idx].tolist()
        return (int(result.boxes.cls[idx].item()), float(result.boxes.conf[idx].item()), *box)


def main(args=None):
    rclpy.init(args=args)
    node = RoadSignDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
