import os
import time
from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import cv2
from cv_bridge import CvBridge
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray, ObjectHypothesisWithPose

try:
    from ultralytics import YOLO
except ImportError as exc:  # pragma: no cover
    YOLO = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


def default_weight_path() -> str:
    env_path = os.environ.get('LVDOT_YOLO_WEIGHT')
    if env_path:
        return env_path
    try:
        package_share = Path(get_package_share_directory('lvdot_ros2_adapter'))
    except PackageNotFoundError:
        return ''
    candidate = package_share / 'weights' / 'yolo11n.pt'
    return str(candidate) if candidate.is_file() else ''


class LvDotYoloNode(Node):
    def __init__(self) -> None:
        super().__init__('lvdot_yolo_node')
        if YOLO is None:
            raise RuntimeError(
                f'ultralytics is not available: {IMPORT_ERROR}. '
                'Install torch and ultralytics first.'
            )

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('weight_path', default_weight_path())
        self.declare_parameter('conf_threshold', 0.65)
        self.declare_parameter('enable_yolo', False)
        self.declare_parameter('use_all_classes', False)
        self.declare_parameter('enable_color_fallback', False)
        self.declare_parameter('imgsz', 352)
        self.declare_parameter('max_det', 10)
        self.declare_parameter('inference_hz', 2.0)
        self.declare_parameter('frame_stride', 1)

        self.bridge = CvBridge()
        self.latest_image = None
        self.last_processed_stamp_ns = None
        self.received_frames = 0
        self.enable_yolo = bool(self.get_parameter('enable_yolo').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.use_all_classes = bool(self.get_parameter('use_all_classes').value)
        self.enable_color_fallback = bool(self.get_parameter('enable_color_fallback').value)
        self.imgsz = int(self.get_parameter('imgsz').value)
        self.max_det = int(self.get_parameter('max_det').value)
        self.inference_hz = max(0.1, float(self.get_parameter('inference_hz').value))
        self.frame_stride = max(1, int(self.get_parameter('frame_stride').value))
        self.weight_path = Path(str(self.get_parameter('weight_path').value)).expanduser() if self.get_parameter('weight_path').value else None
        self._half_enabled = True
        if self.enable_yolo:
            if self.weight_path is None or not self.weight_path.is_file():
                raise RuntimeError(
                    'enable_yolo=true but no valid weight file was provided. '
                    'Set the weight_path parameter or LVDOT_YOLO_WEIGHT.'
                )
            self.model = YOLO(str(self.weight_path), task='detect')
            try:
                dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
                self.model(
                    dummy, verbose=False, imgsz=self.imgsz, max_det=self.max_det,
                    half=self._half_enabled,
                )
            except Exception as exc:
                self.get_logger().warning(f'YOLO FP16 warmup failed, retrying FP32: {exc}')
                self._half_enabled = False
                dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
                self.model(dummy, verbose=False, imgsz=self.imgsz, max_det=self.max_det, half=False)
            self.get_logger().info(
                f'YOLO model loaded: weight={self.weight_path} imgsz={self.imgsz} '
                f'max_det={self.max_det} inference_hz={self.inference_hz:.1f} '
                f'half={self._half_enabled}'
            )
        else:
            self.model = None

        image_topic = self.get_parameter('image_topic').value
        # Multi-threaded layout: image subscription on its own group can keep
        # enqueueing under BEST_EFFORT QoS even while inference is mid-flight on
        # the worker group.  Without this the single-threaded executor drops
        # ~50% of incoming frames at 9Hz publisher rate.
        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._infer_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.detection_pub = self.create_publisher(Detection2DArray, 'yolo_detector/detected_bounding_boxes', qos_profile_sensor_data)
        self.vis_pub = self.create_publisher(Image, 'yolo_detector/detected_image', qos_profile_sensor_data)
        self.time_pub = self.create_publisher(Float64, 'yolo_detector/yolo_time', 10)
        self.inference_busy = False
        # Drive inference from a timer so subscriber doesn't block on model call,
        # while still honoring the configured rate cap.
        self._infer_timer = self.create_timer(
            1.0 / self.inference_hz, self._maybe_infer, callback_group=self._infer_group
        ) if (self.enable_yolo or self.enable_color_fallback) else None

    def on_image(self, msg: Image) -> None:
        self.received_frames += 1
        if self.received_frames % self.frame_stride != 0:
            return
        # Just stash the latest image; the inference timer picks it up.  This
        # keeps the subscriber callback < 1ms so BEST_EFFORT doesn't drop.
        self.latest_image = msg

    def _maybe_infer(self) -> None:
        if self.inference_busy or self.latest_image is None:
            return
        self.inference_busy = True
        try:
            self.run_inference()
        finally:
            self.inference_busy = False

    def append_detection(
        self,
        detection_msg: Detection2DArray,
        vis,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
        label: str,
        conf: float,
        color: tuple[int, int, int],
    ) -> None:
        detection = Detection2D()
        detection.header = detection_msg.header
        bbox = BoundingBox2D()
        width = float(max(0, x2 - x1))
        height = float(max(0, y2 - y1))
        bbox.center.position.x = float(x1) + width * 0.5
        bbox.center.position.y = float(y1) + height * 0.5
        bbox.size_x = width
        bbox.size_y = height
        detection.bbox = bbox
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(label)
        hypothesis.hypothesis.score = float(conf)
        detection.results.append(hypothesis)
        detection_msg.detections.append(detection)

        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
        cv2.putText(
            vis,
            f'{label} {conf:.2f}',
            (x1, max(0, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    def color_fallback_boxes(self, image) -> list[tuple[int, int, int, int]]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (20, 70, 40), (160, 255, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 1200.0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if w < 20 or h < 20:
                continue
            boxes.append((x, y, x + w, y + h))
        return boxes

    def run_inference(self) -> None:
        if self.latest_image is None:
            return
        stamp_ns = (
            int(self.latest_image.header.stamp.sec) * 1_000_000_000
            + int(self.latest_image.header.stamp.nanosec)
        )
        if self.last_processed_stamp_ns == stamp_ns:
            return

        start = time.monotonic()
        try:
            t_bridge = time.monotonic()
            image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
            t_bridge = time.monotonic() - t_bridge
            detection_msg = Detection2DArray()
            detection_msg.header = self.latest_image.header
            vis = image.copy()
            published = 0
            t_inf = 0.0

            if self.enable_yolo:
                t_inf = time.monotonic()
                results = self.model(
                    image, verbose=False, imgsz=self.imgsz, max_det=self.max_det,
                    half=self._half_enabled,
                )[0]
                t_inf = time.monotonic() - t_inf
                for box in results.boxes:
                    cls_id = int(box.cls.item())
                    cls_name = results.names.get(cls_id, str(cls_id))
                    conf = float(box.conf.item())
                    if conf < self.conf_threshold:
                        continue
                    if not self.use_all_classes and cls_name != 'person':
                        continue

                    x1, y1, x2, y2 = [int(v) for v in box.xyxy[0].tolist()]
                    self.append_detection(detection_msg, vis, x1, y1, x2, y2, cls_name, conf, (0, 255, 255))
                    published += 1

            if published == 0 and self.enable_color_fallback:
                for x1, y1, x2, y2 in self.color_fallback_boxes(image):
                    self.append_detection(detection_msg, vis, x1, y1, x2, y2, 'fallback_object', 1.0, (0, 255, 0))

            t_pub = time.monotonic()
            self.detection_pub.publish(detection_msg)
            self.vis_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            t_pub = time.monotonic() - t_pub
            elapsed = time.monotonic() - start
            self.time_pub.publish(Float64(data=elapsed))
            if self.enable_yolo and self.received_frames % 30 == 0:
                self.get_logger().info(
                    f"yolo latency: total={elapsed*1000:.0f}ms bridge={t_bridge*1000:.0f}ms "
                    f"inference={t_inf*1000:.0f}ms publish={t_pub*1000:.0f}ms (frame#{self.received_frames})"
                )
        except Exception as exc:  # pragma: no cover - runtime transport/model failures
            self.get_logger().error(f'YOLO inference failed for frame {stamp_ns}: {exc}')
        finally:
            self.last_processed_stamp_ns = stamp_ns


def main() -> None:
    rclpy.init()
    node = LvDotYoloNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
