"""
Detection evaluator — quantitative metrics for /onboard_detector/dynamic_bboxes
against Gazebo's ground-truth pedestrian agent_states.

Per frame (driven by detector output rate):
  1. Snapshot the latest GT AgentPoseArray (world-frame pedestrian positions).
  2. Take the marker array from the detector; treat each marker's pose.position
     as the detected object's center.
  3. Greedy 2D Euclidean matching (GT ↔ detection) within `match_threshold_m`.
  4. Count TP / FP / FN, accumulate position error for matched pairs.

Outputs:
  - rolling 10-second metrics logged to console
  - CSV row per frame to `csv_path` (default /tmp/lvdot_eval.csv) for offline analysis

Limitations (intentional, this is the first cut):
  - 2D center distance only (no 3D IoU yet); pedestrians are ground-bound so this
    is fine for now. Add IoU later when we want u_map size accuracy.
  - No identity switch (IDS) tracking — would need to remember marker.id across frames
    and detect when a GT pedestrian changes its matched detector ID.
  - Assumes detector publishes in the world frame (frame_id == 'map' or 'world').
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import MarkerArray

try:
    from depth_eval_msgs.msg import AgentPoseArray
except ImportError as e:
    raise ImportError(
        "depth_eval_msgs not on the path. Source ros2_depth_eval_ws install/setup.bash"
    ) from e


class DetectionEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("detection_evaluator")

        self.declare_parameter("gt_topic", "/pedestrian_sim/agent_states")
        self.declare_parameter("det_topic", "/onboard_detector/dynamic_bboxes")
        self.declare_parameter("match_threshold_m", 1.5)
        self.declare_parameter("csv_path", "/tmp/lvdot_eval.csv")
        self.declare_parameter("log_every_sec", 5.0)
        # The detector publishes LINE_LIST (type=5) with ns="dynamic"; QC fusion
        # publishes CUBE (type=1) with ns="qcgaf_fused". Make both selectable so
        # the same evaluator can score either target.
        self.declare_parameter("det_marker_type", 5)   # 5=LINE_LIST, 1=CUBE
        self.declare_parameter("det_namespace", "dynamic")

        self._gt_topic = self.get_parameter("gt_topic").value
        self._det_topic = self.get_parameter("det_topic").value
        self._match_thresh = float(self.get_parameter("match_threshold_m").value)
        csv_path = str(self.get_parameter("csv_path").value)
        self._log_every_sec = float(self.get_parameter("log_every_sec").value)
        self._marker_type = int(self.get_parameter("det_marker_type").value)
        self._namespace = str(self.get_parameter("det_namespace").value)

        self._latest_gt: list[tuple[str, float, float, float]] | None = None
        # Rolling cumulative counters
        self._frames = 0
        self._tp = 0
        self._fp = 0
        self._fn = 0
        self._err_sum = 0.0
        self._err_count = 0
        # Window counters (reset every log_every_sec)
        self._win_frames = 0
        self._win_tp = 0
        self._win_fp = 0
        self._win_fn = 0
        self._win_err_sum = 0.0
        self._win_err_count = 0
        self._last_log_ns = self.get_clock().now().nanoseconds

        # CSV file (one row per detector frame)
        self._csv_fp = open(csv_path, "w", newline="")
        self._csv = csv.writer(self._csv_fp)
        self._csv.writerow([
            "stamp_sec", "gt_n", "det_n",
            "tp", "fp", "fn", "mean_err_m",
        ])

        self.create_subscription(AgentPoseArray, self._gt_topic, self.on_gt, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, self._det_topic, self.on_det, qos_profile_sensor_data)

        self.get_logger().info(
            f"eval: gt={self._gt_topic} det={self._det_topic} "
            f"match_thresh={self._match_thresh}m csv={csv_path}"
        )

    def on_gt(self, msg: AgentPoseArray) -> None:
        # Cache GT in (name, x, y, z) form — ground frame, pedestrians at z≈0.02.
        self._latest_gt = [
            (a.name, a.pose.position.x, a.pose.position.y, a.pose.position.z)
            for a in msg.agents
        ]

    def on_det(self, msg: MarkerArray) -> None:
        if self._latest_gt is None:
            return

        # Detector publishes dynamic bboxes as LINE_LIST (type=5, ns="dynamic")
        # with 24 vertices defining 12 cube edges. QC fusion publishes CUBE
        # (type=1, ns="qcgaf_fused"). Filter by configured type+namespace and
        # compute the box center accordingly.
        det_centers: list[tuple[float, float, float]] = []
        for m in msg.markers:
            if m.action == 3:
                continue
            if m.type != self._marker_type or m.ns != self._namespace:
                continue
            if m.type == 5:  # LINE_LIST: average vertices
                if not m.points:
                    continue
                xs = [p.x for p in m.points]
                ys = [p.y for p in m.points]
                zs = [p.z for p in m.points]
                cx = (min(xs) + max(xs)) * 0.5
                cy = (min(ys) + max(ys)) * 0.5
                cz = (min(zs) + max(zs)) * 0.5
            elif m.type == 1:  # CUBE: read pose.position
                cx = float(m.pose.position.x)
                cy = float(m.pose.position.y)
                cz = float(m.pose.position.z)
            else:
                continue
            det_centers.append((cx, cy, cz))

        gt_centers = list(self._latest_gt)
        tp, fp, fn, err_sum, err_count = self._match(gt_centers, det_centers)

        self._frames += 1
        self._tp += tp; self._fp += fp; self._fn += fn
        self._err_sum += err_sum; self._err_count += err_count
        self._win_frames += 1
        self._win_tp += tp; self._win_fp += fp; self._win_fn += fn
        self._win_err_sum += err_sum; self._win_err_count += err_count

        mean_err = (err_sum / err_count) if err_count else float("nan")
        stamp_sec = msg.markers[0].header.stamp.sec + msg.markers[0].header.stamp.nanosec * 1e-9 if msg.markers else 0.0
        self._csv.writerow([
            f"{stamp_sec:.3f}", len(gt_centers), len(det_centers),
            tp, fp, fn, f"{mean_err:.3f}" if err_count else "",
        ])
        # Don't flush every frame — only on log.

        # Window log
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_log_ns) / 1e9 >= self._log_every_sec:
            self._emit_window_log()
            self._last_log_ns = now_ns

    def _match(
        self,
        gt: list[tuple[str, float, float, float]],
        det: list[tuple[float, float, float]],
    ) -> tuple[int, int, int, float, int]:
        """Greedy nearest-neighbor matching on 2D (x, y) distance."""
        # Compute all pairwise distances, then greedy assign smallest first.
        pairs: list[tuple[float, int, int]] = []
        for gi, (_, gx, gy, _) in enumerate(gt):
            for di, (dx, dy, _) in enumerate(det):
                d = math.hypot(gx - dx, gy - dy)
                if d <= self._match_thresh:
                    pairs.append((d, gi, di))
        pairs.sort()

        matched_gt: set[int] = set()
        matched_det: set[int] = set()
        err_sum = 0.0
        err_count = 0
        for d, gi, di in pairs:
            if gi in matched_gt or di in matched_det:
                continue
            matched_gt.add(gi)
            matched_det.add(di)
            err_sum += d
            err_count += 1

        tp = err_count
        fn = len(gt) - tp
        fp = len(det) - tp
        return tp, fp, fn, err_sum, err_count

    def _emit_window_log(self) -> None:
        if self._win_frames == 0:
            return
        prec = self._win_tp / max(1, self._win_tp + self._win_fp)
        rec = self._win_tp / max(1, self._win_tp + self._win_fn)
        f1 = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0.0
        mean_err = self._win_err_sum / max(1, self._win_err_count)
        # MOTA over the window (counting all GTs that should have been matched)
        total_gt = self._win_tp + self._win_fn
        mota = 1 - (self._win_fp + self._win_fn) / max(1, total_gt)
        self.get_logger().info(
            f"[window {self._win_frames}fr] "
            f"prec={prec:.2f} rec={rec:.2f} F1={f1:.2f} "
            f"MOTA={mota:.2f} mean_err={mean_err:.2f}m "
            f"(TP={self._win_tp} FP={self._win_fp} FN={self._win_fn})"
        )
        self._csv_fp.flush()
        # Reset window
        self._win_frames = 0
        self._win_tp = 0
        self._win_fp = 0
        self._win_fn = 0
        self._win_err_sum = 0.0
        self._win_err_count = 0

    def destroy_node(self) -> bool:
        try:
            # Final cumulative summary
            if self._frames > 0:
                prec = self._tp / max(1, self._tp + self._fp)
                rec = self._tp / max(1, self._tp + self._fn)
                f1 = 2 * prec * rec / (prec + rec) if (prec + rec) > 0 else 0.0
                mean_err = self._err_sum / max(1, self._err_count)
                total_gt = self._tp + self._fn
                mota = 1 - (self._fp + self._fn) / max(1, total_gt)
                self.get_logger().info(
                    f"[FINAL {self._frames}fr] "
                    f"prec={prec:.2f} rec={rec:.2f} F1={f1:.2f} "
                    f"MOTA={mota:.2f} mean_err={mean_err:.2f}m "
                    f"(TP={self._tp} FP={self._fp} FN={self._fn})"
                )
            self._csv_fp.flush()
            self._csv_fp.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DetectionEvaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
