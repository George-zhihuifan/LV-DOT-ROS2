#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class Metric5InterfaceAdapter(Node):
    def __init__(self) -> None:
        super().__init__("metric5_interface_adapter")

        self.declare_parameter("output_mode", "both")  # both | 3d | 2d
        self.declare_parameter("source_ns_prefix", "tracked")

        self.declare_parameter("input_image_topic", "/rgbd_camera/image")
        self.declare_parameter("input_pointcloud_topic", "/uav_lidar/scan/points")
        self.declare_parameter("input_detected_image_topic", "/yolo_detector/detected_image")
        self.declare_parameter("input_boxes_topic", "/onboard_detector/tracked_bboxes")

        self.declare_parameter("output_marker_topic", "/metric5/detection_3d_marker")
        self.declare_parameter("output_image_topic", "/metric5/detection_2d_image")
        self.declare_parameter("cube_alpha", 0.45)
        self.declare_parameter("cube_lifetime_sec", 0.2)

        self.output_mode = str(self.get_parameter("output_mode").value)
        self.source_ns_prefix = str(self.get_parameter("source_ns_prefix").value)

        input_image_topic = str(self.get_parameter("input_image_topic").value)
        input_pointcloud_topic = str(self.get_parameter("input_pointcloud_topic").value)
        input_detected_image_topic = str(self.get_parameter("input_detected_image_topic").value)
        input_boxes_topic = str(self.get_parameter("input_boxes_topic").value)

        output_marker_topic = str(self.get_parameter("output_marker_topic").value)
        output_image_topic = str(self.get_parameter("output_image_topic").value)
        self.cube_alpha = float(self.get_parameter("cube_alpha").value)
        self.cube_lifetime = float(self.get_parameter("cube_lifetime_sec").value)

        self.marker_pub = self.create_publisher(Marker, output_marker_topic, 100)
        self.image_pub = self.create_publisher(Image, output_image_topic, 10)

        self.last_raw_image_stamp: Optional[float] = None
        self.last_raw_cloud_stamp: Optional[float] = None
        self.last_count_log_sec: Optional[float] = None

        self.create_subscription(Image, input_image_topic, self._on_raw_image, 10)
        self.create_subscription(PointCloud2, input_pointcloud_topic, self._on_raw_cloud, 10)
        self.create_subscription(Image, input_detected_image_topic, self._on_detected_image, 10)
        self.create_subscription(MarkerArray, input_boxes_topic, self._on_marker_array, 50)

        self.get_logger().info(
            "Metric5 adapter started. "
            f"mode={self.output_mode} "
            f"boxes={input_boxes_topic} "
            f"detected_image={input_detected_image_topic} "
            f"marker_out={output_marker_topic} "
            f"image_out={output_image_topic}"
        )

    def _on_raw_image(self, msg: Image) -> None:
        self.last_raw_image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_raw_cloud(self, msg: PointCloud2) -> None:
        self.last_raw_cloud_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _on_detected_image(self, msg: Image) -> None:
        if self.output_mode in ("both", "2d"):
            self.image_pub.publish(msg)

    def _on_marker_array(self, msg: MarkerArray) -> None:
        if self.output_mode not in ("both", "3d"):
            return

        if len(msg.markers) == 0:
            return

        # Clear last frame.
        clear = Marker()
        clear.header = msg.markers[0].header
        clear.ns = "metric5_3d"
        clear.id = 0
        clear.action = Marker.DELETEALL
        self.marker_pub.publish(clear)

        out_count = 0
        marker_id = 1

        for src in msg.markers:
            if src.action != Marker.ADD:
                continue
            if src.type != Marker.LINE_LIST:
                continue
            if self.source_ns_prefix and not src.ns.startswith(self.source_ns_prefix):
                continue
            if len(src.points) < 8:
                continue

            xs = [p.x for p in src.points]
            ys = [p.y for p in src.points]
            zs = [p.z for p in src.points]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            min_z, max_z = min(zs), max(zs)

            sx = max_x - min_x
            sy = max_y - min_y
            sz = max_z - min_z
            if sx <= 1e-4 or sy <= 1e-4 or sz <= 1e-4:
                continue

            cube = Marker()
            cube.header = src.header
            cube.ns = "metric5_3d"
            cube.id = marker_id
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose.position.x = (min_x + max_x) * 0.5
            cube.pose.position.y = (min_y + max_y) * 0.5
            cube.pose.position.z = (min_z + max_z) * 0.5
            cube.pose.orientation.w = 1.0
            cube.scale.x = sx
            cube.scale.y = sy
            cube.scale.z = sz
            cube.color = ColorRGBA(r=0.10, g=0.75, b=1.00, a=self.cube_alpha)
            cube.lifetime.sec = int(math.floor(self.cube_lifetime))
            cube.lifetime.nanosec = int((self.cube_lifetime - cube.lifetime.sec) * 1e9)

            self.marker_pub.publish(cube)
            out_count += 1
            marker_id += 1

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if self.last_count_log_sec is None or (now_sec - self.last_count_log_sec) > 2.0:
            self.get_logger().info(
                "metric5 output: "
                f"cube_count={out_count} "
                f"source_markers={len(msg.markers)} "
                f"raw_image_seen={'yes' if self.last_raw_image_stamp is not None else 'no'} "
                f"raw_cloud_seen={'yes' if self.last_raw_cloud_stamp is not None else 'no'}"
            )
            self.last_count_log_sec = now_sec


def main() -> None:
    rclpy.init()
    node = Metric5InterfaceAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
