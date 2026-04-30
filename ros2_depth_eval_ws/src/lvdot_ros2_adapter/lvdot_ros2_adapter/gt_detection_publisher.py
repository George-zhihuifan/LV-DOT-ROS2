import csv
from pathlib import Path

from rclpy.node import Node
import rclpy
from vision_msgs.msg import BoundingBox2D, Detection2D, Detection2DArray


class GtDetectionPublisher(Node):
    def __init__(self) -> None:
        super().__init__('lvdot_gt_detection_publisher')
        self.declare_parameter('gt_csv', str(Path.home() / 'ros2_depth_eval_ws/artifacts/experiment_gt.csv'))
        self.publisher = self.create_publisher(Detection2DArray, 'yolo_detector/detected_bounding_boxes', 10)
        self.timer = self.create_timer(1.0 / 15.0, self.publish_detections)
        self.rows = self.load_rows(Path(self.get_parameter('gt_csv').value))

    def load_rows(self, path: Path) -> list[dict]:
        with path.open('r', encoding='utf-8', newline='') as handle:
            return list(csv.DictReader(handle))

    def publish_detections(self) -> None:
        msg = Detection2DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_color_optical_frame'

        for row in self.rows:
            xmin = float(row['roi_xmin_px'])
            ymin = float(row['roi_ymin_px'])
            xmax = float(row['roi_xmax_px'])
            ymax = float(row['roi_ymax_px'])

            detection = Detection2D()
            detection.header = msg.header
            bbox = BoundingBox2D()
            width = max(0.0, xmax - xmin)
            height = max(0.0, ymax - ymin)
            bbox.center.position.x = xmin + width * 0.5
            bbox.center.position.y = ymin + height * 0.5
            bbox.size_x = width
            bbox.size_y = height
            detection.bbox = bbox
            msg.detections.append(detection)

        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = GtDetectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
