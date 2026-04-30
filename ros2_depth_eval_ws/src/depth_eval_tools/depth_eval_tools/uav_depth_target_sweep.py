import csv
import math
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from sensor_msgs.msg import Image

from depth_eval_tools.depth_validity_common import crop_center, summarize_depth_distribution, valid_mask
from depth_eval_tools.roi_eval_common import depth_image_to_array


def median_valid_depth(depth_m: np.ndarray, far_clip_m: float) -> float:
    mask = valid_mask(depth_m, far_clip_m)
    vals = depth_m[mask]
    return float(np.median(vals)) if vals.size else float('nan')


class UavDepthTargetSweep(Node):
    def __init__(self) -> None:
        super().__init__('uav_depth_target_sweep')
        default_output = Path.home() / 'ros2_depth_eval_ws' / 'artifacts' / 'uav_depth_target_sweep.csv'

        self.declare_parameter('world_name', 'uav_depth_target')
        self.declare_parameter('depth_topic', '/rgbd_camera/depth_image')
        self.declare_parameter('model_name', 'uav_main')
        self.declare_parameter('pose_service_name', '')
        self.declare_parameter('pose_request_timeout_sec', 1.0)
        self.declare_parameter('far_clip_m', 12.0)
        self.declare_parameter('panel_center_x', 5.0)
        self.declare_parameter('panel_front_x', 4.95)
        self.declare_parameter('camera_offset_x', 0.24)
        self.declare_parameter('uav_y', 0.0)
        self.declare_parameter('uav_z', 1.0)
        self.declare_parameter('distances_m', [1.5, 3.0, 5.0])
        self.declare_parameter('settle_sec', 1.2)
        self.declare_parameter('sample_count', 12)
        self.declare_parameter('output_csv', str(default_output))

        self.world_name = self.get_parameter('world_name').value
        self.model_name = self.get_parameter('model_name').value
        configured_service_name = str(self.get_parameter('pose_service_name').value)
        self.pose_service_name = configured_service_name or f'/world/{self.world_name}/set_pose/blocking'
        self.pose_request_timeout_sec = max(0.1, float(self.get_parameter('pose_request_timeout_sec').value))
        self.depth_topic = self.get_parameter('depth_topic').value
        self.far_clip_m = float(self.get_parameter('far_clip_m').value)
        self.panel_front_x = float(self.get_parameter('panel_front_x').value)
        self.camera_offset_x = float(self.get_parameter('camera_offset_x').value)
        self.uav_y = float(self.get_parameter('uav_y').value)
        self.uav_z = float(self.get_parameter('uav_z').value)
        self.distances_m = [float(v) for v in self.get_parameter('distances_m').value]
        self.settle_sec = float(self.get_parameter('settle_sec').value)
        self.sample_count = int(self.get_parameter('sample_count').value)
        self.output_csv = Path(self.get_parameter('output_csv').value)
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)

        self.pose_cli = self.create_client(SetEntityPose, self.pose_service_name)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)

        self.current_index = -1
        self.current_distance = None
        self.collect_after_sec = None
        self.samples: list[dict] = []
        self.all_rows: list[dict] = []
        self.last_depth_msg: Image | None = None
        self.pose_future = None
        self.pose_request_sent_ns = None
        self.shutdown_requested = False
        self.warned_service_unavailable = False
        self.phase = 'waiting_service'
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_depth(self, msg: Image) -> None:
        self.last_depth_msg = msg

    def target_uav_x(self, distance_m: float) -> float:
        return self.panel_front_x - self.camera_offset_x - distance_m

    def set_pose(self, distance_m: float) -> None:
        if self.pose_future is not None and not self.pose_future.done():
            return
        pose = Pose()
        pose.position.x = self.target_uav_x(distance_m)
        pose.position.y = self.uav_y
        pose.position.z = self.uav_z
        pose.orientation.w = 1.0
        if not self.pose_cli.wait_for_service(timeout_sec=0.2):
            if not self.warned_service_unavailable:
                self.warned_service_unavailable = True
                self.get_logger().warning(
                    f'SetEntityPose service /world/{self.world_name}/set_pose is unavailable; '
                    'target sweep is waiting for Gazebo pose control.'
                )
            return
        self.warned_service_unavailable = False
        req = SetEntityPose.Request()
        req.entity = Entity(name=self.model_name, type=Entity.MODEL)
        req.pose = pose
        self.pose_future = self.pose_cli.call_async(req)
        self.pose_request_sent_ns = self.get_clock().now().nanoseconds
        self.pose_future.add_done_callback(self.on_pose_result)

    def on_pose_result(self, future) -> None:
        self.pose_future = None
        self.pose_request_sent_ns = None
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f'SetEntityPose failed: {exc}')
            self.shutdown_requested = True
            return
        if not resp.success:
            self.get_logger().error('SetEntityPose returned success=false')
            self.shutdown_requested = True
            return
        self.collect_after_sec = self.get_clock().now().nanoseconds / 1e9 + self.settle_sec
        self.samples = []
        self.phase = 'collecting'
        self.get_logger().info(f'settling for distance {self.current_distance:.2f} m')

    def collect_sample(self) -> None:
        if self.last_depth_msg is None:
            return
        depth_m = depth_image_to_array(self.last_depth_msg)
        full_metrics = summarize_depth_distribution(depth_m, self.far_clip_m)
        center50 = crop_center(depth_m, 0.5)
        center25 = crop_center(depth_m, 0.25)
        row = {
            'distance_m': self.current_distance,
            **full_metrics,
            'center50_p50_depth_m': median_valid_depth(center50, self.far_clip_m),
            'center25_p50_depth_m': median_valid_depth(center25, self.far_clip_m),
        }
        row['center50_abs_error_m'] = abs(row['center50_p50_depth_m'] - self.current_distance) if math.isfinite(row['center50_p50_depth_m']) else float('nan')
        row['center25_abs_error_m'] = abs(row['center25_p50_depth_m'] - self.current_distance) if math.isfinite(row['center25_p50_depth_m']) else float('nan')
        self.samples.append(row)
        self.get_logger().info(
            f"distance={self.current_distance:.2f} sample={len(self.samples)}/{self.sample_count} "
            f"valid={row['valid_ratio']:.4f} center25={row['center25_valid_ratio']:.4f} "
            f"center25_p50={row['center25_p50_depth_m']:.3f}"
        )

    def finalize_distance(self) -> None:
        metrics = {}
        for key in self.samples[0].keys():
            values = np.array([float(row[key]) for row in self.samples], dtype=float)
            metrics[key] = float(np.nanmean(values))
        metrics['samples'] = len(self.samples)
        self.all_rows.append(metrics)

    def write_results_and_shutdown(self) -> None:
        with self.output_csv.open('w', encoding='utf-8', newline='') as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.all_rows[0].keys()))
            writer.writeheader()
            writer.writerows(self.all_rows)
        self.get_logger().info(f'wrote {len(self.all_rows)} distance summaries to {self.output_csv}')
        self.shutdown_requested = True

    def on_timer(self) -> None:
        if self.shutdown_requested:
            self.destroy_node()
            rclpy.shutdown()
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.pose_future is not None and not self.pose_future.done() and self.pose_request_sent_ns is not None:
            elapsed_sec = (self.get_clock().now().nanoseconds - self.pose_request_sent_ns) / 1e9
            if elapsed_sec > self.pose_request_timeout_sec:
                self.get_logger().error(
                    f'SetEntityPose request timed out after {elapsed_sec:.2f}s on {self.pose_service_name}'
                )
                self.pose_future.cancel()
                self.pose_future = None
                self.pose_request_sent_ns = None
                self.shutdown_requested = True
            return
        if self.phase == 'waiting_service':
            self.current_index = 0
            self.current_distance = self.distances_m[self.current_index]
            self.phase = 'moving'
            self.set_pose(self.current_distance)
            return

        if self.phase == 'collecting':
            if now_sec < self.collect_after_sec:
                return
            self.collect_sample()
            if len(self.samples) < self.sample_count:
                return
            self.finalize_distance()
            self.current_index += 1
            if self.current_index >= len(self.distances_m):
                self.write_results_and_shutdown()
                return
            self.current_distance = self.distances_m[self.current_index]
            self.phase = 'moving'
            self.set_pose(self.current_distance)


def main() -> None:
    rclpy.init()
    node = UavDepthTargetSweep()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
