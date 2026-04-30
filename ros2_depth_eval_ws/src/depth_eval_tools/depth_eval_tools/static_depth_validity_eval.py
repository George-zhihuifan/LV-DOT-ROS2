import csv
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from depth_eval_tools.depth_validity_common import summarize_depth_distribution
from depth_eval_tools.roi_eval_common import depth_image_to_array


class StaticDepthValidityEval(Node):
    def __init__(self) -> None:
        super().__init__('static_depth_validity_eval')
        default_output = Path.home() / 'ros2_depth_eval_ws' / 'artifacts' / 'static_depth_validity_eval.csv'
        default_summary = Path.home() / 'ros2_depth_eval_ws' / 'artifacts' / 'static_depth_validity_eval_summary.csv'

        self.declare_parameter('depth_topic', '/rgbd_camera/depth_image')
        self.declare_parameter('sample_stride', 10)
        self.declare_parameter('warmup_sec', 1.0)
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('far_clip_m', 12.0)
        self.declare_parameter('output_csv', str(default_output))
        self.declare_parameter('summary_csv', str(default_summary))

        self.sample_stride = max(1, int(self.get_parameter('sample_stride').value))
        self.warmup_sec = float(self.get_parameter('warmup_sec').value)
        self.duration_sec = float(self.get_parameter('duration_sec').value)
        self.far_clip_m = float(self.get_parameter('far_clip_m').value)
        self.output_csv = Path(self.get_parameter('output_csv').value)
        self.summary_csv = Path(self.get_parameter('summary_csv').value)
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.summary_csv.parent.mkdir(parents=True, exist_ok=True)
        self.output_handle = self.output_csv.open('w', encoding='utf-8', newline='')
        self.writer = None
        self.rows: list[dict] = []
        self.start_stamp_sec: float | None = None
        self.frame_count = 0

        self.depth_sub = self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.on_depth,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'evaluating static depth validity for duration={self.duration_sec:.2f}s '
            f'warmup={self.warmup_sec:.2f}s stride={self.sample_stride}'
        )

    def summarize(self) -> None:
        if not self.rows:
            raise SystemExit('No sampled depth frames were recorded.')
        keys = [
            'finite_ratio',
            'positive_ratio',
            'valid_ratio',
            'center50_valid_ratio',
            'center25_valid_ratio',
            'saturation_ratio',
            'min_depth_m',
            'p10_depth_m',
            'p50_depth_m',
            'p90_depth_m',
            'max_depth_m',
        ]
        summary = {
            'samples': len(self.rows),
            'sample_stride': self.sample_stride,
            'warmup_sec': self.warmup_sec,
            'duration_sec': self.duration_sec,
        }
        for key in keys:
            values = np.array([float(row[key]) for row in self.rows], dtype=float)
            summary[key] = float(np.nanmean(values))
        with self.summary_csv.open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(summary.keys()))
            writer.writeheader()
            writer.writerow(summary)
        self.get_logger().info(f'wrote summary to {self.summary_csv}')

    def finalize_and_shutdown(self) -> None:
        if not self.output_handle.closed:
            self.output_handle.close()
        self.summarize()
        self.get_logger().info(f'wrote {len(self.rows)} sampled frames to {self.output_csv}')
        self.destroy_node()
        rclpy.shutdown()

    def on_depth(self, msg: Image) -> None:
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.start_stamp_sec is None:
            self.start_stamp_sec = stamp_sec
        elapsed_sec = stamp_sec - self.start_stamp_sec
        if elapsed_sec < self.warmup_sec:
            return
        if elapsed_sec > (self.warmup_sec + self.duration_sec):
            self.finalize_and_shutdown()
            return

        self.frame_count += 1
        if (self.frame_count - 1) % self.sample_stride != 0:
            return

        depth_m = depth_image_to_array(msg)
        row = {
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nanosec': msg.header.stamp.nanosec,
            'elapsed_sec': round(elapsed_sec, 4),
            'frame_index': self.frame_count,
            **summarize_depth_distribution(depth_m, self.far_clip_m),
        }
        if self.writer is None:
            self.writer = csv.DictWriter(self.output_handle, fieldnames=list(row.keys()))
            self.writer.writeheader()
        self.writer.writerow(row)
        self.output_handle.flush()
        self.rows.append(row)
        self.get_logger().info(
            f"sample {len(self.rows)} elapsed={elapsed_sec:.2f}s valid={row['valid_ratio']:.4f} "
            f"center50={row['center50_valid_ratio']:.4f} center25={row['center25_valid_ratio']:.4f}"
        )

    def destroy_node(self):
        if not self.output_handle.closed:
            self.output_handle.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = StaticDepthValidityEval()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
