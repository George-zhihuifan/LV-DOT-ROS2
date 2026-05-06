#!/usr/bin/env python3
import argparse
import csv
import math
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from lvdot_interfaces.msg import PipelineStats
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


ID_RE = re.compile(r"id=([-+]?\d*\.?\d+)")
HEAD_RE = re.compile(r"clusters=(\d+)\s+filtered_boxes=(\d+)\s+dynamic_boxes=(\d+)")
PTS_RE = re.compile(r"cluster\[(\d+)\]:\s+points=(\d+)")


@dataclass
class Sample:
    t: float
    visual_bbox_count: int
    db_bbox_count: int
    lidar_bbox_count: int
    filtered_bbox_count: int
    fusion_component_count: int
    source_total: int
    fusion_nonzero: int
    void_like: int
    tracked_count: int
    mean_track_step_m: float


class FusionEvalNode(Node):
    def __init__(self) -> None:
        super().__init__("fusion_min_eval")
        self.stats: Optional[PipelineStats] = None
        self.cluster_debug: Optional[str] = None
        self.track_positions: Dict[str, Tuple[float, float, float]] = {}

        self.create_subscription(
            PipelineStats,
            "/onboard_detector/pipeline_stats_status",
            self._on_stats,
            10,
        )
        self.create_subscription(
            String,
            "/onboard_detector/cluster_debug_status",
            self._on_cluster_debug,
            10,
        )
        self.create_subscription(
            MarkerArray,
            "/onboard_detector/tracked_bboxes",
            self._on_tracked_markers,
            10,
        )

    def _on_stats(self, msg: PipelineStats) -> None:
        self.stats = msg

    def _on_cluster_debug(self, msg: String) -> None:
        self.cluster_debug = msg.data

    def _on_tracked_markers(self, msg: MarkerArray) -> None:
        positions: Dict[str, Tuple[float, float, float]] = {}
        for m in msg.markers:
            if m.action == Marker.DELETEALL:
                continue
            if m.type != Marker.TEXT_VIEW_FACING:
                continue
            if not m.ns.endswith("_label"):
                continue
            id_match = ID_RE.search(m.text or "")
            if not id_match:
                continue
            track_id = id_match.group(1)
            positions[track_id] = (
                float(m.pose.position.x),
                float(m.pose.position.y),
                float(m.pose.position.z),
            )
        self.track_positions = positions


def parse_void_like(cluster_debug: Optional[str]) -> int:
    if not cluster_debug:
        return 0
    head = HEAD_RE.search(cluster_debug)
    if not head:
        return 0
    filtered_boxes = int(head.group(2))
    if filtered_boxes <= 0:
        return 0

    pts = [0] * filtered_boxes
    for idx_s, p_s in PTS_RE.findall(cluster_debug):
        idx = int(idx_s)
        if 0 <= idx < filtered_boxes:
            pts[idx] = int(p_s)

    # void-like: filtered box exists but the corresponding cluster point count is zero
    return int(any(v == 0 for v in pts))


def percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    arr = sorted(values)
    k = (len(arr) - 1) * p
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return arr[int(k)]
    return arr[f] * (c - k) + arr[c] * (k - f)


def main() -> int:
    parser = argparse.ArgumentParser(description="Minimal fusion health evaluation")
    parser.add_argument("--duration", type=float, default=600.0, help="Sampling duration seconds")
    parser.add_argument("--hz", type=float, default=1.0, help="Sampling rate")
    parser.add_argument(
        "--out-dir",
        default="/home/skbt2/lvdot_ros2_ws/artifacts",
        help="Output directory",
    )
    parser.add_argument("--prefix", default="fusion_min_eval", help="Output file prefix")
    parser.add_argument("--void-threshold", type=float, default=0.01)
    parser.add_argument("--nonzero-threshold", type=float, default=0.95)
    parser.add_argument("--jitter-threshold", type=float, default=0.5)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"{args.prefix}_{stamp}.csv"
    summary_path = out_dir / f"{args.prefix}_{stamp}_summary.txt"

    rclpy.init()
    node = FusionEvalNode()

    period = 1.0 / max(args.hz, 1e-6)
    deadline = time.monotonic() + args.duration

    samples: List[Sample] = []
    all_steps: List[float] = []
    prev_positions: Dict[str, Tuple[float, float, float]] = {}

    try:
        while time.monotonic() < deadline:
            tick_start = time.monotonic()
            rclpy.spin_once(node, timeout_sec=min(0.2, period))

            stats = node.stats
            if stats is None:
                time.sleep(max(0.0, period - (time.monotonic() - tick_start)))
                continue

            source_total = int(stats.visual_bbox_count + stats.db_bbox_count + stats.lidar_bbox_count)
            fusion_nonzero = int(stats.filtered_bbox_count > 0)
            void_like = parse_void_like(node.cluster_debug)

            curr = node.track_positions
            frame_steps: List[float] = []
            for k, pos in curr.items():
                if k in prev_positions:
                    p = prev_positions[k]
                    d = math.sqrt((pos[0] - p[0]) ** 2 + (pos[1] - p[1]) ** 2 + (pos[2] - p[2]) ** 2)
                    frame_steps.append(d)
                    all_steps.append(d)
            prev_positions = dict(curr)

            samples.append(
                Sample(
                    t=time.time(),
                    visual_bbox_count=int(stats.visual_bbox_count),
                    db_bbox_count=int(stats.db_bbox_count),
                    lidar_bbox_count=int(stats.lidar_bbox_count),
                    filtered_bbox_count=int(stats.filtered_bbox_count),
                    fusion_component_count=int(stats.fusion_component_count),
                    source_total=source_total,
                    fusion_nonzero=fusion_nonzero,
                    void_like=void_like,
                    tracked_count=len(curr),
                    mean_track_step_m=(sum(frame_steps) / len(frame_steps) if frame_steps else 0.0),
                )
            )

            dt = time.monotonic() - tick_start
            if dt < period:
                time.sleep(period - dt)

    finally:
        node.destroy_node()
        rclpy.shutdown()

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(
            [
                "t",
                "visual_bbox_count",
                "db_bbox_count",
                "lidar_bbox_count",
                "filtered_bbox_count",
                "fusion_component_count",
                "source_total",
                "fusion_nonzero",
                "void_like",
                "tracked_count",
                "mean_track_step_m",
            ]
        )
        for s in samples:
            w.writerow(
                [
                    f"{s.t:.3f}",
                    s.visual_bbox_count,
                    s.db_bbox_count,
                    s.lidar_bbox_count,
                    s.filtered_bbox_count,
                    s.fusion_component_count,
                    s.source_total,
                    s.fusion_nonzero,
                    s.void_like,
                    s.tracked_count,
                    f"{s.mean_track_step_m:.6f}",
                ]
            )

    frames = len(samples)
    source_frames = sum(1 for s in samples if s.source_total > 0)
    void_frames = sum(s.void_like for s in samples)
    nonzero_frames = sum(s.fusion_nonzero for s in samples if s.source_total > 0)

    void_ratio = (void_frames / frames) if frames else 0.0
    fusion_nonzero_rate = (nonzero_frames / source_frames) if source_frames else 0.0
    jitter_p95 = percentile(all_steps, 0.95)

    void_ok = void_ratio < args.void_threshold
    nonzero_ok = fusion_nonzero_rate > args.nonzero_threshold
    jitter_ok = jitter_p95 < args.jitter_threshold
    overall_ok = void_ok and nonzero_ok and jitter_ok

    with summary_path.open("w", encoding="utf-8") as f:
        f.write(f"csv={csv_path}\n")
        f.write(f"frames={frames}\n")
        f.write(f"source_frames={source_frames}\n")
        f.write(f"void_like_frames={void_frames}\n")
        f.write(f"void_like_ratio={void_ratio:.6f}\n")
        f.write(f"fusion_nonzero_rate={fusion_nonzero_rate:.6f}\n")
        f.write(f"track_step_p95_m={jitter_p95:.6f}\n")
        f.write(f"threshold_void_ratio_lt={args.void_threshold}\n")
        f.write(f"threshold_fusion_nonzero_gt={args.nonzero_threshold}\n")
        f.write(f"threshold_jitter_p95_lt_m={args.jitter_threshold}\n")
        f.write(f"check_void_ratio={'PASS' if void_ok else 'FAIL'}\n")
        f.write(f"check_fusion_nonzero={'PASS' if nonzero_ok else 'FAIL'}\n")
        f.write(f"check_jitter_p95={'PASS' if jitter_ok else 'FAIL'}\n")
        f.write(f"overall={'PASS' if overall_ok else 'FAIL'}\n")

    print(summary_path)
    print(csv_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
