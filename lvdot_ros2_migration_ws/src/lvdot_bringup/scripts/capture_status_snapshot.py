#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from lvdot_interfaces.msg import InputHealth, PipelineStats, StageTimers


class StatusSnapshotNode(Node):
    def __init__(self) -> None:
        super().__init__("lvdot_status_snapshot")
        self.input_health = None
        self.stage_timers = None
        self.pipeline_stats = None

        self.create_subscription(
            InputHealth,
            "/onboard_detector/input_health_status",
            self._on_input_health,
            10,
        )
        self.create_subscription(
            StageTimers,
            "/onboard_detector/stage_timers_status",
            self._on_stage_timers,
            10,
        )
        self.create_subscription(
            PipelineStats,
            "/onboard_detector/pipeline_stats_status",
            self._on_pipeline_stats,
            10,
        )

    def _on_input_health(self, msg: InputHealth) -> None:
        self.input_health = msg

    def _on_stage_timers(self, msg: StageTimers) -> None:
        self.stage_timers = msg

    def _on_pipeline_stats(self, msg: PipelineStats) -> None:
        self.pipeline_stats = msg

    def complete(self) -> bool:
        return (
            self.input_health is not None
            and self.stage_timers is not None
            and self.pipeline_stats is not None
        )


def emit_snapshot(node: StatusSnapshotNode) -> int:
    input_health = node.input_health
    stage_timers = node.stage_timers
    pipeline_stats = node.pipeline_stats

    values = {
        "input_health_received": int(input_health is not None),
        "stage_timers_received": int(stage_timers is not None),
        "pipeline_stats_received": int(pipeline_stats is not None),
        "depth_count": getattr(input_health, "depth_count", 0),
        "color_count": getattr(input_health, "color_count", 0),
        "lidar_count": getattr(input_health, "lidar_count", 0),
        "pose_count": getattr(input_health, "pose_count", 0),
        "odom_count": getattr(input_health, "odom_count", 0),
        "yolo_count": getattr(input_health, "yolo_count", 0),
        "detection_tick_count": getattr(stage_timers, "detection_tick_count", 0),
        "lidar_detection_tick_count": getattr(
            stage_timers, "lidar_detection_tick_count", 0
        ),
        "tracking_tick_count": getattr(stage_timers, "tracking_tick_count", 0),
        "classification_tick_count": getattr(
            stage_timers, "classification_tick_count", 0
        ),
        "vis_tick_count": getattr(stage_timers, "vis_tick_count", 0),
        "visual_bbox_count": getattr(pipeline_stats, "visual_bbox_count", 0),
        "db_bbox_count": getattr(pipeline_stats, "db_bbox_count", 0),
        "lidar_bbox_count": getattr(pipeline_stats, "lidar_bbox_count", 0),
        "filtered_bbox_count": getattr(pipeline_stats, "filtered_bbox_count", 0),
        "track_count": getattr(pipeline_stats, "track_count", 0),
        "dynamic_count": getattr(pipeline_stats, "dynamic_count", 0),
        "service_call_count": getattr(pipeline_stats, "service_call_count", 0),
        "u_map_box_count": getattr(pipeline_stats, "u_map_box_count", 0),
        "projected_depth_box_count": getattr(
            pipeline_stats, "projected_depth_box_count", 0
        ),
        "u_map_enhanced_db_count": getattr(
            pipeline_stats, "u_map_enhanced_db_count", 0
        ),
        "u_map_enhanced_visual_count": getattr(
            pipeline_stats, "u_map_enhanced_visual_count", 0
        ),
        "u_map_enhanced_filtered_before_yolo_count": getattr(
            pipeline_stats, "u_map_enhanced_filtered_before_yolo_count", 0
        ),
        "u_map_enhanced_filtered_count": getattr(
            pipeline_stats, "u_map_enhanced_filtered_count", 0
        ),
        "ready_state": int(
            getattr(input_health, "depth_count", 0) > 0
            and getattr(input_health, "color_count", 0) > 0
            and getattr(input_health, "lidar_count", 0) > 0
            and getattr(input_health, "pose_count", 0) > 0
            and getattr(input_health, "odom_count", 0) > 0
            and getattr(stage_timers, "detection_tick_count", 0) > 0
            and getattr(stage_timers, "lidar_detection_tick_count", 0) > 0
            and getattr(stage_timers, "tracking_tick_count", 0) > 0
            and getattr(stage_timers, "classification_tick_count", 0) > 0
            and getattr(stage_timers, "vis_tick_count", 0) > 0
        ),
    }

    for key, value in values.items():
        print(f"{key}={value}")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    rclpy.init(args=None)
    node = StatusSnapshotNode()
    deadline = time.monotonic() + args.timeout

    try:
        while time.monotonic() < deadline and not node.complete():
            rclpy.spin_once(node, timeout_sec=0.2)
        return emit_snapshot(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
