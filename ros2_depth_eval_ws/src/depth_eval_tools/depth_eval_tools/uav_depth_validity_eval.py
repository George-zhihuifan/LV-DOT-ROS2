import csv
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

from depth_eval_tools.depth_validity_common import summarize_depth_distribution
from depth_eval_tools.roi_eval_common import depth_image_to_array


def path_length_3d(waypoints: list[tuple[float, float, float]], loop: bool) -> float:
    if len(waypoints) < 2:
        return 0.0
    total = 0.0
    for start, end in zip(waypoints, waypoints[1:]):
        total += math.dist(start, end)
    if loop:
        total += math.dist(waypoints[-1], waypoints[0])
    return total


class UavDepthValidityEval(Node):
    def __init__(self) -> None:
        super().__init__('uav_depth_validity_eval')
        default_config = Path.home() / 'ros2_depth_eval_ws' / 'src' / 'depth_eval_bringup' / 'config' / 'pedestrian_prototype.yaml'
        default_output = Path.home() / 'ros2_depth_eval_ws' / 'artifacts' / 'uav_depth_validity_eval.csv'
        default_summary = Path.home() / 'ros2_depth_eval_ws' / 'artifacts' / 'uav_depth_validity_eval_summary.csv'

        self.declare_parameter('config_path', str(default_config))
        self.declare_parameter('depth_topic', '/rgbd_camera/depth_image')
        self.declare_parameter('pose_topic', '/mavros/local_position/pose')
        self.declare_parameter('sample_stride', 10)
        self.declare_parameter('warmup_sec', 1.0)
        self.declare_parameter('lap_count', 1.0)
        self.declare_parameter('far_clip_m', 12.0)
        self.declare_parameter('output_csv', str(default_output))
        self.declare_parameter('summary_csv', str(default_summary))

        config_path = Path(self.get_parameter('config_path').value)
        with config_path.open('r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle)
        uav = config['uav']
        self.uav_waypoints = [tuple(map(float, point)) for point in uav.get('waypoints', [])]
        self.uav_speed = float(uav.get('speed_mps', 1.0))
        self.uav_loop = bool(uav.get('loop', True))
        self.lap_count = float(self.get_parameter('lap_count').value)
        self.warmup_sec = float(self.get_parameter('warmup_sec').value)
        self.sample_stride = max(1, int(self.get_parameter('sample_stride').value))
        self.far_clip_m = float(self.get_parameter('far_clip_m').value)
        self.lap_duration_sec = path_length_3d(self.uav_waypoints, self.uav_loop) / max(self.uav_speed, 1e-3)
        self.total_duration_sec = self.warmup_sec + max(0.1, self.lap_duration_sec * self.lap_count)

        self.output_csv = Path(self.get_parameter('output_csv').value)
        self.summary_csv = Path(self.get_parameter('summary_csv').value)
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.summary_csv.parent.mkdir(parents=True, exist_ok=True)
        self.output_handle = self.output_csv.open('w', encoding='utf-8', newline='')
        self.writer = None
        self.rows: list[dict] = []

        self.start_stamp_sec: float | None = None
        self.latest_pose: PoseStamped | None = None
        self.frame_count = 0

        self.pose_sub = self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value, self.on_pose, 10)
        self.depth_sub = self.create_subscription(Image, self.get_parameter('depth_topic').value, self.on_depth, qos_profile_sensor_data)

        self.get_logger().info(
            f'evaluating depth validity for {self.lap_count:.2f} lap(s), '
            f'lap_duration={self.lap_duration_sec:.2f}s total_window={self.total_duration_sec:.2f}s stride={self.sample_stride}'
        )

    def on_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

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
            'lap_duration_sec': self.lap_duration_sec,
            'lap_count': self.lap_count,
            'total_duration_sec': self.total_duration_sec,
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
        if elapsed_sec > self.total_duration_sec:
            self.finalize_and_shutdown()
            return

        self.frame_count += 1
        if (self.frame_count - 1) % self.sample_stride != 0:
            return

        depth_m = depth_image_to_array(msg)
        metrics = summarize_depth_distribution(depth_m, self.far_clip_m)

        pose = self.latest_pose.pose if self.latest_pose is not None else None
        row = {
            'stamp_sec': msg.header.stamp.sec,
            'stamp_nanosec': msg.header.stamp.nanosec,
            'elapsed_sec': round(elapsed_sec, 4),
            'frame_index': self.frame_count,
            'pose_x': float(pose.position.x) if pose is not None else float('nan'),
            'pose_y': float(pose.position.y) if pose is not None else float('nan'),
            'pose_z': float(pose.position.z) if pose is not None else float('nan'),
            **metrics,
        }
        if self.writer is None:
            self.writer = csv.DictWriter(self.output_handle, fieldnames=list(row.keys()))
            self.writer.writeheader()
        self.writer.writerow(row)
        self.output_handle.flush()
        self.rows.append(row)
        self.get_logger().info(
            f"sample {len(self.rows)} elapsed={elapsed_sec:.2f}s pose=({row['pose_x']:.2f},{row['pose_y']:.2f},{row['pose_z']:.2f}) "
            f"valid_ratio={row['valid_ratio']:.4f} center50={row['center50_valid_ratio']:.4f} center25={row['center25_valid_ratio']:.4f}"
        )

    def destroy_node(self):
        if not self.output_handle.closed:
            self.output_handle.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = UavDepthValidityEval()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
