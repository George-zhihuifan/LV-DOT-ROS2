import csv
from pathlib import Path

from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from depth_eval_tools.roi_eval_common import depth_image_to_array, evaluate_roi, load_targets


def iou(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    ax0, ay0, ax1, ay1 = a
    bx0, by0, bx1, by1 = b
    inter_w = max(0, min(ax1, bx1) - max(ax0, bx0))
    inter_h = max(0, min(ay1, by1) - max(ay0, by0))
    inter = inter_w * inter_h
    union = max(1, (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter)
    return inter / union


class LiveRoiDepthEval(Node):
    def __init__(self) -> None:
        super().__init__('live_roi_depth_eval')
        self.declare_parameter('gt_csv', str(Path.home() / 'ros2_depth_eval_ws/artifacts/experiment_gt.csv'))
        self.declare_parameter('output_csv', str(Path.home() / 'ros2_depth_eval_ws/artifacts/live_roi_depth_eval.csv'))
        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('detection_topic', '/yolo_detector/detected_bounding_boxes')
        self.declare_parameter('far_clip_m', 12.0)
        self.declare_parameter('foreground_tolerance_m', 0.35)
        self.declare_parameter('sync_slop_sec', 3.0)
        self.declare_parameter('sync_queue_size', 30)

        self.targets = load_targets(Path(self.get_parameter('gt_csv').value))
        self.output_csv = Path(self.get_parameter('output_csv').value)
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.writer = None
        self.output_handle = self.output_csv.open('w', encoding='utf-8', newline='')
        self.rows_written = 0

        depth_topic = self.get_parameter('depth_topic').value
        detection_topic = self.get_parameter('detection_topic').value
        self.far_clip_m = float(self.get_parameter('far_clip_m').value)
        self.tolerance_m = float(self.get_parameter('foreground_tolerance_m').value)
        sync_slop_sec = float(self.get_parameter('sync_slop_sec').value)
        sync_queue_size = int(self.get_parameter('sync_queue_size').value)

        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.det_sub = Subscriber(self, Detection2DArray, detection_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.depth_sub, self.det_sub],
            queue_size=sync_queue_size,
            slop=sync_slop_sec,
        )
        self.sync.registerCallback(self.callback)

    def callback(self, depth_msg: Image, detection_msg: Detection2DArray) -> None:
        depth_m = depth_image_to_array(depth_msg)
        detections = []
        for det in detection_msg.detections:
            half_w = int(round(det.bbox.size_x * 0.5))
            half_h = int(round(det.bbox.size_y * 0.5))
            cx = int(round(det.bbox.center.position.x))
            cy = int(round(det.bbox.center.position.y))
            x0 = cx - half_w
            y0 = cy - half_h
            x1 = cx + half_w
            y1 = cy + half_h
            detections.append((x0, y0, x1, y1))

        rows = []
        for target in self.targets:
            gt_box = (target.roi_xmin_px, target.roi_ymin_px, target.roi_xmax_px, target.roi_ymax_px)
            best = max(detections, key=lambda det: iou(det, gt_box), default=gt_box)
            metrics = evaluate_roi(
                depth_m=depth_m,
                gt_depth_m=target.gt_depth_m,
                xmin=best[0],
                ymin=best[1],
                xmax=best[2],
                ymax=best[3],
                far_clip_m=self.far_clip_m,
                tolerance_m=self.tolerance_m,
            )
            rows.append({
                'stamp_sec': depth_msg.header.stamp.sec,
                'stamp_nanosec': depth_msg.header.stamp.nanosec,
                'target_name': target.name,
                'range_group': target.expected_range_group,
                'gt_depth_m': target.gt_depth_m,
                'det_xmin_px': best[0],
                'det_ymin_px': best[1],
                'det_xmax_px': best[2],
                'det_ymax_px': best[3],
                **metrics,
            })

        if self.writer is None:
            self.writer = csv.DictWriter(self.output_handle, fieldnames=list(rows[0].keys()))
            self.writer.writeheader()

        self.writer.writerows(rows)
        self.output_handle.flush()
        self.rows_written += len(rows)
        self.get_logger().info(f'wrote {len(rows)} ROI rows, total={self.rows_written}')

    def destroy_node(self):
        if not self.output_handle.closed:
            self.output_handle.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = LiveRoiDepthEval()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
