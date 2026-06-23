"""Advanced LV-DOT evaluator: center-distance detection + tracking + GRU prediction metrics.

Primary metric: center-distance matching at multiple thresholds (0.5m, 1.0m, 1.5m, 2.0m).
3D IoU is reported as a diagnostic (UAV platform produces small, position-noisy bboxes
that make IoU near-zero — see logs/EXPERIMENT_LOG_20260517.md for analysis).
Tracking (MOTA/IDF1) uses center-distance matching at 1.0m.
"""

from __future__ import annotations

import csv
import json
import math
import signal
from dataclasses import dataclass
from pathlib import Path

try:
    import numpy as np
    from scipy.optimize import linear_sum_assignment
    _HAVE_SCIPY = True
except Exception:  # pragma: no cover - scipy is expected in the eval env
    _HAVE_SCIPY = False

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from visualization_msgs.msg import Marker, MarkerArray

try:
    from depth_eval_msgs.msg import AgentPoseArray
except ImportError as e:
    raise ImportError(
        "depth_eval_msgs not on the path. Source ros2_depth_eval_ws install/setup.bash"
    ) from e


@dataclass(frozen=True)
class Box3D:
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    track_id: str | None = None


def compute_3d_iou(a: Box3D, b: Box3D) -> float:
    """Axis-aligned 3D IoU for center-size boxes."""
    ax, ay, az = a.center
    aw, ad, ah = a.size
    bx, by, bz = b.center
    bw, bd, bh = b.size

    overlap_x = max(0.0, min(ax + aw * 0.5, bx + bw * 0.5) - max(ax - aw * 0.5, bx - bw * 0.5))
    overlap_y = max(0.0, min(ay + ad * 0.5, by + bd * 0.5) - max(ay - ad * 0.5, by - bd * 0.5))
    overlap_z = max(0.0, min(az + ah * 0.5, bz + bh * 0.5) - max(az - ah * 0.5, bz - bh * 0.5))

    intersection = overlap_x * overlap_y * overlap_z
    volume_a = max(0.0, aw * ad * ah)
    volume_b = max(0.0, bw * bd * bh)
    union = volume_a + volume_b - intersection
    return intersection / union if union > 0.0 else 0.0


def greedy_iou_match(
    gt_boxes: list[Box3D],
    det_boxes: list[Box3D],
    threshold: float,
) -> list[tuple[int, int, float]]:
    pairs: list[tuple[float, int, int]] = []
    for gi, gt in enumerate(gt_boxes):
        for di, det in enumerate(det_boxes):
            iou = compute_3d_iou(gt, det)
            if iou >= threshold:
                pairs.append((iou, gi, di))
    pairs.sort(reverse=True)

    matched_gt: set[int] = set()
    matched_det: set[int] = set()
    matches: list[tuple[int, int, float]] = []
    for iou, gi, di in pairs:
        if gi in matched_gt or di in matched_det:
            continue
        matched_gt.add(gi)
        matched_det.add(di)
        matches.append((gi, di, iou))
    return matches


def greedy_center_match(
    gt_boxes: list[Box3D],
    det_boxes: list[Box3D],
    threshold_m: float,
) -> list[tuple[int, int, float]]:
    pairs: list[tuple[float, int, int]] = []
    for gi, gt in enumerate(gt_boxes):
        for di, det in enumerate(det_boxes):
            distance = _distance(gt.center, det.center)
            if distance <= threshold_m:
                pairs.append((distance, gi, di))
    pairs.sort()

    matched_gt: set[int] = set()
    matched_det: set[int] = set()
    matches: list[tuple[int, int, float]] = []
    for distance, gi, di in pairs:
        if gi in matched_gt or di in matched_det:
            continue
        matched_gt.add(gi)
        matched_det.add(di)
        matches.append((gi, di, distance))
    return matches


def clearmot_center_match(
    gt_boxes: list[Box3D],
    det_boxes: list[Box3D],
    threshold_m: float,
    gt_to_det: dict,
) -> list[tuple[int, int, float]]:
    """CLEAR-MOT style GT<->det matching for *tracking* metrics only.

    Unlike per-frame greedy matching, this first preserves previous-frame
    GT->track_id correspondences that are still within ``threshold_m`` (identity
    continuity), then optimally assigns the remainder (Hungarian, greedy
    fallback). This stops two GTs that pass within ``threshold_m`` from
    spuriously swapping track IDs -- the dominant source of inflated IDSW in
    dense scenes. Detection metrics keep using ``greedy_center_match``.
    """
    used_g: set[int] = set()
    used_d: set[int] = set()
    matches: list[tuple[int, int, float]] = []

    det_by_tid: dict[object, list[int]] = {}
    for di, det in enumerate(det_boxes):
        det_by_tid.setdefault(det.track_id, []).append(di)

    # 1) Keep still-valid prior correspondences (identity continuity).
    for gi, gt in enumerate(gt_boxes):
        prev = gt_to_det.get(gt.track_id) if gt.track_id is not None else None
        if prev is None:
            continue
        for di in det_by_tid.get(prev, []):
            if di in used_d:
                continue
            distance = _distance(gt.center, det_boxes[di].center)
            if distance <= threshold_m:
                matches.append((gi, di, distance))
                used_g.add(gi)
                used_d.add(di)
                break

    # 2) Optimally match the remaining GTs and dets within threshold.
    rem_g = [gi for gi in range(len(gt_boxes)) if gi not in used_g]
    rem_d = [di for di in range(len(det_boxes)) if di not in used_d]
    if rem_g and rem_d:
        if _HAVE_SCIPY:
            cost = np.full((len(rem_g), len(rem_d)), 1e6)
            for a, gi in enumerate(rem_g):
                for b, di in enumerate(rem_d):
                    distance = _distance(gt_boxes[gi].center, det_boxes[di].center)
                    if distance <= threshold_m:
                        cost[a, b] = distance
            for a, b in zip(*linear_sum_assignment(cost)):
                if cost[a, b] <= threshold_m:
                    matches.append((rem_g[a], rem_d[b], float(cost[a, b])))
        else:
            pairs = sorted(
                (_distance(gt_boxes[gi].center, det_boxes[di].center), gi, di)
                for gi in rem_g for di in rem_d
                if _distance(gt_boxes[gi].center, det_boxes[di].center) <= threshold_m
            )
            for distance, gi, di in pairs:
                if gi in used_g or di in used_d:
                    continue
                used_g.add(gi)
                used_d.add(di)
                matches.append((gi, di, distance))
    return matches


def _stamp_to_sec(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1e-9


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def _axis_errors(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float, float]:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    dxy = math.sqrt(dx * dx + dy * dy)
    return dx, dy, dz, dxy


_CENTER_THRESHOLDS = [0.5, 1.0, 1.5, 2.0]
_TRACKING_THRESHOLD_M = 1.0


class AdvancedEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("advanced_evaluator")

        self.declare_parameter("gt_topic", "/pedestrian_sim/agent_states")
        self.declare_parameter("gt_obstacle_topic", "/pedestrian_sim/agent_markers")
        self.declare_parameter("gt_obstacle_namespace", "pedestrian_obstacles")
        self.declare_parameter("include_static_obstacles", False)
        self.declare_parameter("det_topic", "/onboard_detector/dynamic_bboxes")
        self.declare_parameter("tracking_det_topic", "")
        self.declare_parameter("pred_topic", "/gru_predictor/predicted_positions")
        self.declare_parameter("det_marker_type", int(Marker.LINE_LIST))
        self.declare_parameter("det_namespace", "dynamic")
        self.declare_parameter("tracking_det_marker_type", -1)
        self.declare_parameter("tracking_det_namespace", "")
        self.declare_parameter("gt_bbox_size", [0.36, 0.36, 1.70])
        self.declare_parameter("gt_bbox_width_m", -1.0)
        self.declare_parameter("gt_bbox_depth_m", -1.0)
        self.declare_parameter("gt_bbox_height_m", -1.0)
        self.declare_parameter("gt_bbox_center_y_offset_m", -0.03)
        self.declare_parameter("gt_bbox_center_z_offset_m", 0.88)
        self.declare_parameter("iou_thresholds", [0.3, 0.5, 0.7])
        self.declare_parameter("csv_path", "/tmp/lvdot_advanced_eval.csv")
        self.declare_parameter("summary_path", "/tmp/lvdot_eval_summary.json")
        self.declare_parameter("matched_pairs_csv_path", "")
        self.declare_parameter("tracking_pairs_csv_path", "")
        self.declare_parameter("eval_duration_sec", 60.0)
        self.declare_parameter("warmup_sec", 15.0)
        self.declare_parameter("log_every_sec", 5.0)
        self.declare_parameter("prediction_dt_sec", 0.5)
        self.declare_parameter("center_match_threshold_m", 1.0)
        self.declare_parameter("prediction_match_threshold_m", 2.0)
        self.declare_parameter("pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("visible_fov_h_rad", 1.21)
        self.declare_parameter("visible_fov_v_rad", 0.74)
        self.declare_parameter("visible_max_dist_m", 8.0)
        self.declare_parameter("visible_min_dist_m", 0.3)
        # 360°/LiDAR-FoV visibility (ADDITIVE metric): of all GT the LiDAR could
        # plausibly see (full azimuth, ~59° elevation, within range), how many were
        # detected — credits out-of-camera-FoV detections that camera-FoV recall misses.
        self.declare_parameter("lidar_visible_fov_h_rad", 6.2832)
        self.declare_parameter("lidar_visible_fov_v_rad", 1.0297)
        self.declare_parameter("lidar_visible_max_dist_m", 12.0)

        self._gt_topic = str(self.get_parameter("gt_topic").value)
        self._gt_obstacle_topic = str(self.get_parameter("gt_obstacle_topic").value)
        self._gt_obstacle_namespace = str(self.get_parameter("gt_obstacle_namespace").value)
        self._include_static_obstacles = bool(self.get_parameter("include_static_obstacles").value)
        self._det_topic = str(self.get_parameter("det_topic").value)
        self._tracking_det_topic = str(self.get_parameter("tracking_det_topic").value) or self._det_topic
        self._pred_topic = str(self.get_parameter("pred_topic").value)
        self._marker_type = int(self.get_parameter("det_marker_type").value)
        self._namespace = str(self.get_parameter("det_namespace").value)
        tracking_marker_type = int(self.get_parameter("tracking_det_marker_type").value)
        tracking_namespace = str(self.get_parameter("tracking_det_namespace").value)
        self._tracking_marker_type = tracking_marker_type if tracking_marker_type >= 0 else self._marker_type
        self._tracking_namespace = tracking_namespace or self._namespace
        gt_size = [float(v) for v in self.get_parameter("gt_bbox_size").value]
        gt_width = float(self.get_parameter("gt_bbox_width_m").value)
        gt_depth = float(self.get_parameter("gt_bbox_depth_m").value)
        gt_height = float(self.get_parameter("gt_bbox_height_m").value)
        if gt_width > 0.0 and gt_depth > 0.0 and gt_height > 0.0:
            gt_size = [gt_width, gt_depth, gt_height]
        self._gt_size = tuple(gt_size)
        self._gt_center_y_offset_m = float(self.get_parameter("gt_bbox_center_y_offset_m").value)
        self._gt_center_z_offset_m = float(self.get_parameter("gt_bbox_center_z_offset_m").value)
        self._iou_thresholds = [float(v) for v in self.get_parameter("iou_thresholds").value]
        self._csv_path = Path(str(self.get_parameter("csv_path").value))
        self._summary_path = Path(str(self.get_parameter("summary_path").value))
        matched_pairs_csv_path = str(self.get_parameter("matched_pairs_csv_path").value)
        if matched_pairs_csv_path:
            self._matched_pairs_csv_path = Path(matched_pairs_csv_path)
        else:
            self._matched_pairs_csv_path = self._csv_path.with_name(self._csv_path.stem + "_matched_pairs.csv")
        tracking_pairs_csv_path = str(self.get_parameter("tracking_pairs_csv_path").value)
        if tracking_pairs_csv_path:
            self._tracking_pairs_csv_path = Path(tracking_pairs_csv_path)
        else:
            self._tracking_pairs_csv_path = self._csv_path.with_name(self._csv_path.stem + "_tracking_pairs.csv")
        self._eval_duration_sec = float(self.get_parameter("eval_duration_sec").value)
        self._warmup_sec = float(self.get_parameter("warmup_sec").value)
        self._log_every_sec = float(self.get_parameter("log_every_sec").value)
        self._prediction_dt_sec = float(self.get_parameter("prediction_dt_sec").value)
        self._prediction_match_threshold_m = float(self.get_parameter("prediction_match_threshold_m").value)
        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._visible_fov_h_rad = float(self.get_parameter("visible_fov_h_rad").value)
        self._visible_fov_v_rad = float(self.get_parameter("visible_fov_v_rad").value)
        self._visible_max_dist_m = float(self.get_parameter("visible_max_dist_m").value)
        self._visible_min_dist_m = float(self.get_parameter("visible_min_dist_m").value)
        self._lidar_visible_fov_h_rad = float(self.get_parameter("lidar_visible_fov_h_rad").value)
        self._lidar_visible_fov_v_rad = float(self.get_parameter("lidar_visible_fov_v_rad").value)
        self._lidar_visible_max_dist_m = float(self.get_parameter("lidar_visible_max_dist_m").value)

        self._latest_dynamic_gt: list[Box3D] = []
        self._latest_static_gt: list[Box3D] = []
        self._latest_pose: tuple[float, float, float, float] | None = None
        self._latest_gt_stamp = 0.0
        self._first_stamp: float | None = None
        self._last_log_stamp = 0.0
        self._last_det_stamp = -1e9
        self._last_tracking_det_stamp = -1e9
        self._finished = False

        self._frames = 0
        self._center_counts = {t: {"tp": 0, "fp": 0, "fn": 0} for t in _CENTER_THRESHOLDS}
        self._center_err = {t: {"sum": 0.0, "count": 0} for t in _CENTER_THRESHOLDS}
        self._center_axis_err = {
            t: {
                "sum_dx": 0.0, "sum_dy": 0.0, "sum_dz": 0.0,
                "sum_abs_dx": 0.0, "sum_abs_dy": 0.0, "sum_abs_dz": 0.0,
                "sum_dxy": 0.0, "count": 0,
            }
            for t in _CENTER_THRESHOLDS
        }
        self._visible_gt_total = 0
        self._lidar_visible_gt_total = 0
        self._visible_tp_total = {t: 0 for t in _CENTER_THRESHOLDS}
        self._lidar_visible_tp_total = {t: 0 for t in _CENTER_THRESHOLDS}
        self._iou_counts = {t: {"tp": 0, "fp": 0, "fn": 0} for t in self._iou_thresholds}
        self._iou_curve_thresholds = [round(i * 0.05, 2) for i in range(1, 20)]
        self._iou_curve_counts = {t: {"tp": 0, "fp": 0, "fn": 0} for t in self._iou_curve_thresholds}
        self._best_iou_sum = 0.0
        self._best_iou_count = 0
        self._best_iou_max = 0.0
        self._iou_matched_err_sum = 0.0
        self._iou_matched_err_count = 0
        self._iou_matched_iou_sum = 0.0
        self._iou_matched_iou_count = 0

        self._gt_to_det: dict[str, str] = {}
        self._gt_seen_frames: dict[str, int] = {}
        self._gt_matched_frames: dict[str, int] = {}
        self._gt_was_matched: dict[str, bool] = {}
        self._tracking_counts = {"tp": 0, "fp": 0, "fn": 0}
        self._idsw = 0
        self._frag = 0

        self._pending_predictions: list[dict[str, object]] = []
        self._pred_err_sum_by_step: dict[int, float] = {}
        self._pred_err_count_by_step: dict[int, int] = {}

        self._csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._summary_path.parent.mkdir(parents=True, exist_ok=True)
        self._matched_pairs_csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._tracking_pairs_csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_fp = open(self._csv_path, "w", newline="")
        self._csv = csv.writer(self._csv_fp)
        self._csv.writerow([
            "stamp_sec", "gt_n", "det_n",
            "visible_gt_n", "cd_vis_tp_10", "cd_vis_recall_10",
            "cd_tp_05", "cd_fp_05", "cd_fn_05",
            "cd_tp_10", "cd_fp_10", "cd_fn_10",
            "cd_tp_15", "cd_fp_15", "cd_fn_15",
            "cd_tp_20", "cd_fp_20", "cd_fn_20",
            "mean_err_1m", "mean_xy_err_1m", "mean_dz_1m", "mean_abs_dz_1m",
            "gt_ids", "visible_gt_ids", "det_ids", "matched_pairs",
        ])
        self._matched_pairs_fp = open(self._matched_pairs_csv_path, "w", newline="")
        self._matched_pairs_csv = csv.writer(self._matched_pairs_fp)
        self._matched_pairs_csv.writerow([
            "stamp_sec",
            "gt_id", "det_id",
            "gt_x", "gt_y", "gt_z",
            "det_x", "det_y", "det_z",
            "det_w", "det_d", "det_h",
            "dx", "dy", "dz", "dxy", "dist3d",
            "gt_range_m", "gt_azimuth_rad", "gt_elevation_rad",
            "gt_visible", "gt_visible_ratio",
            "z_centroid",
            "z_foot_lift",
            "z_head_drop",
        ])
        self._tracking_pairs_fp = open(self._tracking_pairs_csv_path, "w", newline="")
        self._tracking_pairs_csv = csv.writer(self._tracking_pairs_fp)
        self._tracking_pairs_csv.writerow([
            "stamp_sec",
            "event",
            "gt_id", "track_id", "previous_track_id",
            "gt_x", "gt_y", "gt_z",
            "track_x", "track_y", "track_z",
            "track_w", "track_d", "track_h",
            "dx", "dy", "dz", "dxy", "dist3d",
            "gt_n", "track_n",
        ])

        self.create_subscription(AgentPoseArray, self._gt_topic, self.on_gt, qos_profile_sensor_data)
        if self._include_static_obstacles:
            self.create_subscription(MarkerArray, self._gt_obstacle_topic, self.on_gt_obstacles, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, self._det_topic, self.on_det, qos_profile_sensor_data)
        if (
            self._tracking_det_topic != self._det_topic or
            self._tracking_marker_type != self._marker_type or
            self._tracking_namespace != self._namespace
        ):
            self.create_subscription(MarkerArray, self._tracking_det_topic, self.on_tracking_det, qos_profile_sensor_data)
        self.create_subscription(MarkerArray, self._pred_topic, self.on_pred, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, self._pose_topic, self.on_pose, qos_profile_sensor_data)
        if self._eval_duration_sec > 0.0:
            self.create_timer(1.0, self._check_duration)

        self.get_logger().info(
            f"advanced_eval: gt={self._gt_topic} static_gt={self._gt_obstacle_topic if self._include_static_obstacles else 'disabled'} "
            f"det={self._det_topic} tracking_det={self._tracking_det_topic} pred={self._pred_topic} "
            f"pose={self._pose_topic} gt_bbox_size={self._gt_size} "
            f"csv={self._csv_path} summary={self._summary_path}"
        )

    def on_gt(self, msg: AgentPoseArray) -> None:
        stamp = _stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self._latest_gt_stamp = stamp
        if self._first_stamp is None:
            self._first_stamp = stamp
            self._last_log_stamp = stamp
        w, d, h = self._gt_size
        self._latest_dynamic_gt = [
            Box3D(
                center=(
                    a.pose.position.x,
                    a.pose.position.y + self._gt_center_y_offset_m,
                    a.pose.position.z + self._gt_center_z_offset_m,
                ),
                size=(w, d, h),
                track_id=a.name,
            )
            for a in msg.agents
        ]
        self._evaluate_due_predictions(stamp)

        elapsed = stamp - self._first_stamp
        if elapsed < self._warmup_sec or self._finished:
            return
        # When detection stream is absent, still count GT frames as FN-only.
        if stamp - self._last_det_stamp > 0.5:
            gt_boxes = self._current_eval_gt()
            gt_n = len(gt_boxes)
            vis_info = self._compute_visibility_info(gt_boxes)
            visible_mask = [it["visible"] for it in vis_info]
            self._visible_gt_total += sum(1 for visible in visible_mask if visible)
            self._lidar_visible_gt_total += sum(1 for it in vis_info if it["lidar_visible"])
            for t in _CENTER_THRESHOLDS:
                self._center_counts[t]["fn"] += gt_n
            for t in self._iou_thresholds:
                self._iou_counts[t]["fn"] += gt_n
            for t in self._iou_curve_thresholds:
                self._iou_curve_counts[t]["fn"] += gt_n
            for _ in gt_boxes:
                self._best_iou_sum += 0.0
                self._best_iou_count += 1
            self._frames += 1
        if stamp - self._last_tracking_det_stamp > 0.5:
            dynamic_gt_n = len(self._latest_dynamic_gt)
            self._tracking_counts["fn"] += dynamic_gt_n

    def on_gt_obstacles(self, msg: MarkerArray) -> None:
        if not self._include_static_obstacles:
            return
        self._latest_static_gt = self._extract_static_gt_boxes(msg)

    def on_pose(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        p = msg.pose.position
        self._latest_pose = (p.x, p.y, p.z, yaw)

    def on_det(self, msg: MarkerArray) -> None:
        if self._finished:
            return
        stamp = self._message_stamp(msg)
        self._last_det_stamp = stamp
        if self._first_stamp is None:
            return
        elapsed = stamp - self._first_stamp
        if elapsed < self._warmup_sec:
            return

        gt_boxes = self._current_eval_gt()
        if not gt_boxes:
            return
        det_boxes = self._extract_det_boxes(msg)
        visibility_info = self._compute_visibility_info(gt_boxes)
        visible_mask = [item["visible"] for item in visibility_info]
        lidar_visible_mask = [item["lidar_visible"] for item in visibility_info]
        self._visible_gt_total += sum(1 for visible in visible_mask if visible)
        self._lidar_visible_gt_total += sum(1 for v in lidar_visible_mask if v)

        center_matches_by_thresh: dict[float, list[tuple[int, int, float]]] = {}
        for threshold in _CENTER_THRESHOLDS:
            matches = greedy_center_match(gt_boxes, det_boxes, threshold)
            center_matches_by_thresh[threshold] = matches
            self._visible_tp_total[threshold] += sum(1 for gi, _, _ in matches if visible_mask[gi])
            self._lidar_visible_tp_total[threshold] += sum(1 for gi, _, _ in matches if lidar_visible_mask[gi])
            counts = self._center_counts[threshold]
            counts["tp"] += len(matches)
            counts["fp"] += len(det_boxes) - len(matches)
            counts["fn"] += len(gt_boxes) - len(matches)
            err_info = self._center_err[threshold]
            axis_info = self._center_axis_err[threshold]
            for gi, di, distance in matches:
                err_info["sum"] += distance
                err_info["count"] += 1
                dx, dy, dz, dxy = _axis_errors(gt_boxes[gi].center, det_boxes[di].center)
                axis_info["sum_dx"] += dx
                axis_info["sum_dy"] += dy
                axis_info["sum_dz"] += dz
                axis_info["sum_abs_dx"] += abs(dx)
                axis_info["sum_abs_dy"] += abs(dy)
                axis_info["sum_abs_dz"] += abs(dz)
                axis_info["sum_dxy"] += dxy
                axis_info["count"] += 1

        for threshold in self._iou_thresholds:
            matches = greedy_iou_match(gt_boxes, det_boxes, threshold)
            counts = self._iou_counts[threshold]
            counts["tp"] += len(matches)
            counts["fp"] += len(det_boxes) - len(matches)
            counts["fn"] += len(gt_boxes) - len(matches)

        for threshold in self._iou_curve_thresholds:
            matches = greedy_iou_match(gt_boxes, det_boxes, threshold)
            counts = self._iou_curve_counts[threshold]
            counts["tp"] += len(matches)
            counts["fp"] += len(det_boxes) - len(matches)
            counts["fn"] += len(gt_boxes) - len(matches)

        for gt in gt_boxes:
            best_iou = max((compute_3d_iou(gt, det) for det in det_boxes), default=0.0)
            self._best_iou_sum += best_iou
            self._best_iou_count += 1
            self._best_iou_max = max(self._best_iou_max, best_iou)

        iou_03_matches = greedy_iou_match(gt_boxes, det_boxes, 0.3)
        for gi, di, iou in iou_03_matches:
            self._iou_matched_err_sum += _distance(gt_boxes[gi].center, det_boxes[di].center)
            self._iou_matched_err_count += 1
            self._iou_matched_iou_sum += iou
            self._iou_matched_iou_count += 1

        self._frames += 1

        self._write_csv_row(stamp, gt_boxes, det_boxes, center_matches_by_thresh, visible_mask)
        self._write_matched_pairs_rows(
            stamp,
            gt_boxes,
            det_boxes,
            center_matches_by_thresh.get(1.0, []),
            visibility_info,
        )
        if stamp - self._last_log_stamp >= self._log_every_sec:
            self._emit_progress_log(elapsed)
            self._last_log_stamp = stamp
        if (
            self._tracking_det_topic == self._det_topic and
            self._tracking_marker_type == self._marker_type and
            self._tracking_namespace == self._namespace
        ):
            self._process_tracking_det(stamp, det_boxes)

    def on_tracking_det(self, msg: MarkerArray) -> None:
        if self._finished or self._first_stamp is None:
            return
        stamp = self._message_stamp(msg)
        elapsed = stamp - self._first_stamp
        if elapsed < self._warmup_sec:
            return
        det_boxes = self._extract_det_boxes(msg, marker_type=self._tracking_marker_type, namespace=self._tracking_namespace)
        self._process_tracking_det(stamp, det_boxes)

    def on_pred(self, msg: MarkerArray) -> None:
        if self._finished or not self._latest_dynamic_gt:
            return
        stamp = self._message_stamp(msg)
        active_gt = list(self._latest_dynamic_gt)
        for marker in msg.markers:
            if marker.action != Marker.ADD or marker.type != Marker.LINE_STRIP:
                continue
            if len(marker.points) < 2:
                continue
            start = (marker.points[0].x, marker.points[0].y, marker.points[0].z)
            nearest = self._nearest_gt(start, active_gt)
            if nearest is None:
                continue
            gt_id, nearest_dist = nearest
            if nearest_dist > self._prediction_match_threshold_m:
                continue
            future_points = [(p.x, p.y, p.z) for p in marker.points[1:]]
            self._pending_predictions.append({
                "gt_id": gt_id,
                "stamp": stamp,
                "points": future_points,
                "done": set(),
            })

    def _extract_det_boxes(
        self,
        msg: MarkerArray,
        *,
        marker_type: int | None = None,
        namespace: str | None = None,
    ) -> list[Box3D]:
        marker_type = self._marker_type if marker_type is None else marker_type
        namespace = self._namespace if namespace is None else namespace
        boxes: list[Box3D] = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL or marker.action == Marker.DELETE:
                continue
            if marker.type != marker_type or marker.ns != namespace:
                continue
            if marker.type == Marker.LINE_LIST:
                if not marker.points:
                    continue
                xs = [p.x for p in marker.points]
                ys = [p.y for p in marker.points]
                zs = [p.z for p in marker.points]
                center = ((min(xs) + max(xs)) * 0.5, (min(ys) + max(ys)) * 0.5, (min(zs) + max(zs)) * 0.5)
                size = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
            elif marker.type == Marker.CUBE:
                center = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
                size = (marker.scale.x, marker.scale.y, marker.scale.z)
            else:
                continue
            if min(size) <= 0.0:
                continue
            boxes.append(Box3D(center=center, size=size, track_id=str(marker.id)))
        return boxes

    def _extract_static_gt_boxes(self, msg: MarkerArray) -> list[Box3D]:
        boxes: list[Box3D] = []
        for marker in msg.markers:
            if marker.action == Marker.DELETEALL or marker.action == Marker.DELETE:
                continue
            if marker.ns != self._gt_obstacle_namespace:
                continue
            if marker.type not in (Marker.CUBE, Marker.CYLINDER):
                continue
            size = (marker.scale.x, marker.scale.y, marker.scale.z)
            if min(size) <= 0.0:
                continue
            center = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
            boxes.append(Box3D(center=center, size=size, track_id=f"static_{marker.id}"))
        return boxes

    def _current_eval_gt(self) -> list[Box3D]:
        if not self._include_static_obstacles:
            return list(self._latest_dynamic_gt)
        return list(self._latest_dynamic_gt) + list(self._latest_static_gt)

    def _process_tracking_det(self, stamp: float, det_boxes: list[Box3D]) -> None:
        if self._finished or not self._latest_dynamic_gt:
            return
        self._last_tracking_det_stamp = stamp
        gt_boxes = list(self._latest_dynamic_gt)
        matches = clearmot_center_match(gt_boxes, det_boxes, _TRACKING_THRESHOLD_M, self._gt_to_det)
        self._tracking_counts["tp"] += len(matches)
        self._tracking_counts["fp"] += len(det_boxes) - len(matches)
        self._tracking_counts["fn"] += len(gt_boxes) - len(matches)
        self._write_tracking_pair_rows(stamp, gt_boxes, det_boxes, matches)
        self._update_tracking(gt_boxes, det_boxes, matches)

    def _update_tracking(self, gt_boxes: list[Box3D], det_boxes: list[Box3D], matches: list[tuple[int, int, float]]) -> None:
        current_matched: set[str] = set()
        for gt in gt_boxes:
            if gt.track_id is not None:
                self._gt_seen_frames[gt.track_id] = self._gt_seen_frames.get(gt.track_id, 0) + 1

        for gi, di, _ in matches:
            gt_id = gt_boxes[gi].track_id
            det_id = det_boxes[di].track_id
            if gt_id is None or det_id is None:
                continue
            previous = self._gt_to_det.get(gt_id)
            if previous is not None and previous != det_id:
                self._idsw += 1
            if self._gt_was_matched.get(gt_id) is False:
                self._frag += 1
            self._gt_to_det[gt_id] = det_id
            self._gt_matched_frames[gt_id] = self._gt_matched_frames.get(gt_id, 0) + 1
            self._gt_was_matched[gt_id] = True
            current_matched.add(gt_id)

        for gt in gt_boxes:
            if gt.track_id is not None and gt.track_id not in current_matched:
                self._gt_was_matched[gt.track_id] = False

    def _evaluate_due_predictions(self, stamp: float) -> None:
        if not self._latest_dynamic_gt:
            return
        gt_by_id = {box.track_id: box.center for box in self._latest_dynamic_gt if box.track_id is not None}
        gt_centers = [box.center for box in self._latest_dynamic_gt]
        keep: list[dict[str, object]] = []
        for item in self._pending_predictions:
            gt_id = item["gt_id"]
            points = item["points"]
            done = item["done"]
            assert isinstance(points, list)
            assert isinstance(done, set)
            for step, point in enumerate(points, start=1):
                due = float(item["stamp"]) + step * self._prediction_dt_sec
                if step in done or stamp < due:
                    continue
                gt_center = gt_by_id.get(str(gt_id))
                if gt_center is None and gt_centers:
                    gt_center = min(gt_centers, key=lambda c: _distance(point, c))
                if gt_center is None:
                    continue
                err = _distance(point, gt_center)
                self._pred_err_sum_by_step[step] = self._pred_err_sum_by_step.get(step, 0.0) + err
                self._pred_err_count_by_step[step] = self._pred_err_count_by_step.get(step, 0) + 1
                done.add(step)
            if len(done) < len(points) and stamp - float(item["stamp"]) < (len(points) + 2) * self._prediction_dt_sec:
                keep.append(item)
        self._pending_predictions = keep

    def _compute_visibility_info(self, gt_boxes: list[Box3D]) -> list[dict[str, float | bool]]:
        if self._latest_pose is None:
            return [
                {
                    "visible": True,
                    "lidar_visible": True,
                    "range_m": float("nan"),
                    "azimuth_rad": float("nan"),
                    "elevation_rad": float("nan"),
                }
                for _ in gt_boxes
            ]

        px, py, pz, yaw = self._latest_pose
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        visibility_info: list[dict[str, float | bool]] = []
        for gt in gt_boxes:
            dx_w = gt.center[0] - px
            dy_w = gt.center[1] - py
            dz = gt.center[2] - pz
            dx = cy * dx_w + sy * dy_w
            dy = -sy * dx_w + cy * dy_w

            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            azimuth = math.atan2(dy, dx)
            elevation = math.atan2(-dz, math.sqrt(dx * dx + dy * dy))
            is_visible = (
                dist >= self._visible_min_dist_m
                and dist <= self._visible_max_dist_m
                and dx > 0.0
                and abs(azimuth) <= self._visible_fov_h_rad * 0.5
                and abs(elevation) <= self._visible_fov_v_rad * 0.5
            )
            lidar_is_visible = (
                dist >= self._visible_min_dist_m
                and dist <= self._lidar_visible_max_dist_m
                and abs(azimuth) <= self._lidar_visible_fov_h_rad * 0.5
                and abs(elevation) <= self._lidar_visible_fov_v_rad * 0.5
            )
            visibility_info.append(
                {
                    "visible": is_visible,
                    "lidar_visible": lidar_is_visible,
                    "range_m": dist,
                    "azimuth_rad": azimuth,
                    "elevation_rad": elevation,
                }
            )
        return visibility_info

    def _compute_visible_mask(self, gt_boxes: list[Box3D]) -> list[bool]:
        return [item["visible"] for item in self._compute_visibility_info(gt_boxes)]

    def _write_csv_row(
        self,
        stamp: float,
        gt_boxes: list[Box3D],
        det_boxes: list[Box3D],
        center_matches: dict[float, list[tuple[int, int, float]]],
        visible_mask: list[bool],
    ) -> None:
        m1 = center_matches.get(1.0, [])
        mean_err = sum(d for _, _, d in m1) / len(m1) if m1 else float("nan")
        mean_xy_err = float("nan")
        mean_dz = float("nan")
        mean_abs_dz = float("nan")
        if m1:
            sum_dxy = 0.0
            sum_dz = 0.0
            sum_abs_dz = 0.0
            for gi, di, _ in m1:
                _, _, dz, dxy = _axis_errors(gt_boxes[gi].center, det_boxes[di].center)
                sum_dxy += dxy
                sum_dz += dz
                sum_abs_dz += abs(dz)
            mean_xy_err = sum_dxy / len(m1)
            mean_dz = sum_dz / len(m1)
            mean_abs_dz = sum_abs_dz / len(m1)
        visible_gt_n = sum(1 for visible in visible_mask if visible)
        visible_tp_10 = sum(1 for gi, _, _ in m1 if visible_mask[gi])
        visible_recall_10 = visible_tp_10 / visible_gt_n if visible_gt_n else 0.0
        matched_pairs = [
            f"{gt_boxes[gi].track_id}:{det_boxes[di].track_id}:{dist:.3f}"
            for gi, di, dist in m1
        ]
        visible_gt_ids = [str(gt_boxes[gi].track_id) for gi, visible in enumerate(visible_mask) if visible]
        row: list[object] = [
            f"{stamp:.3f}", len(gt_boxes), len(det_boxes),
            visible_gt_n, visible_tp_10, f"{visible_recall_10:.3f}",
        ]
        for t in _CENTER_THRESHOLDS:
            m = center_matches.get(t, [])
            row.extend([len(m), len(det_boxes) - len(m), len(gt_boxes) - len(m)])
        row.extend([
            f"{mean_err:.3f}" if m1 else "",
            f"{mean_xy_err:.3f}" if m1 else "",
            f"{mean_dz:.3f}" if m1 else "",
            f"{mean_abs_dz:.3f}" if m1 else "",
            ";".join(str(b.track_id) for b in gt_boxes),
            ";".join(visible_gt_ids),
            ";".join(str(b.track_id) for b in det_boxes),
            ";".join(matched_pairs),
        ])
        self._csv.writerow(row)

    def _write_matched_pairs_rows(
        self,
        stamp: float,
        gt_boxes: list[Box3D],
        det_boxes: list[Box3D],
        matches_1m: list[tuple[int, int, float]],
        visibility_info: list[dict[str, float | bool]],
    ) -> None:
        for gi, di, dist3d in matches_1m:
            gt = gt_boxes[gi]
            det = det_boxes[di]
            dx, dy, dz, dxy = _axis_errors(gt.center, det.center)
            det_h = det.size[2]
            z_centroid = det.center[2]
            z_foot_lift = det.center[2] - det_h * 0.5 + self._gt_center_z_offset_m
            z_head_drop = det.center[2] + det_h * 0.5 - self._gt_center_z_offset_m
            info = visibility_info[gi]
            self._matched_pairs_csv.writerow([
                f"{stamp:.3f}",
                gt.track_id or "",
                det.track_id or "",
                f"{gt.center[0]:.6f}",
                f"{gt.center[1]:.6f}",
                f"{gt.center[2]:.6f}",
                f"{det.center[0]:.6f}",
                f"{det.center[1]:.6f}",
                f"{det.center[2]:.6f}",
                f"{det.size[0]:.6f}",
                f"{det.size[1]:.6f}",
                f"{det_h:.6f}",
                f"{dx:.6f}",
                f"{dy:.6f}",
                f"{dz:.6f}",
                f"{dxy:.6f}",
                f"{dist3d:.6f}",
                f"{float(info['range_m']):.6f}",
                f"{float(info['azimuth_rad']):.6f}",
                f"{float(info['elevation_rad']):.6f}",
                int(bool(info["visible"])),
                "1.0" if bool(info["visible"]) else "0.0",
                f"{z_centroid:.6f}",
                f"{z_foot_lift:.6f}",
                f"{z_head_drop:.6f}",
            ])

    def _write_tracking_pair_rows(
        self,
        stamp: float,
        gt_boxes: list[Box3D],
        det_boxes: list[Box3D],
        matches_1m: list[tuple[int, int, float]],
    ) -> None:
        matched_gt = {gi for gi, _, _ in matches_1m}
        matched_det = {di for _, di, _ in matches_1m}

        for gi, di, dist3d in matches_1m:
            gt = gt_boxes[gi]
            det = det_boxes[di]
            previous = self._gt_to_det.get(gt.track_id or "")
            event = "idsw" if previous is not None and previous != det.track_id else "match"
            dx, dy, dz, dxy = _axis_errors(gt.center, det.center)
            self._tracking_pairs_csv.writerow([
                f"{stamp:.3f}",
                event,
                gt.track_id or "",
                det.track_id or "",
                previous or "",
                f"{gt.center[0]:.6f}",
                f"{gt.center[1]:.6f}",
                f"{gt.center[2]:.6f}",
                f"{det.center[0]:.6f}",
                f"{det.center[1]:.6f}",
                f"{det.center[2]:.6f}",
                f"{det.size[0]:.6f}",
                f"{det.size[1]:.6f}",
                f"{det.size[2]:.6f}",
                f"{dx:.6f}",
                f"{dy:.6f}",
                f"{dz:.6f}",
                f"{dxy:.6f}",
                f"{dist3d:.6f}",
                len(gt_boxes),
                len(det_boxes),
            ])

        for gi, gt in enumerate(gt_boxes):
            if gi in matched_gt:
                continue
            self._tracking_pairs_csv.writerow([
                f"{stamp:.3f}",
                "fn",
                gt.track_id or "",
                "",
                self._gt_to_det.get(gt.track_id or "", "") or "",
                f"{gt.center[0]:.6f}",
                f"{gt.center[1]:.6f}",
                f"{gt.center[2]:.6f}",
                "", "", "",
                "", "", "",
                "", "", "", "", "",
                len(gt_boxes),
                len(det_boxes),
            ])

        for di, det in enumerate(det_boxes):
            if di in matched_det:
                continue
            self._tracking_pairs_csv.writerow([
                f"{stamp:.3f}",
                "fp",
                "",
                det.track_id or "",
                "",
                "", "", "",
                f"{det.center[0]:.6f}",
                f"{det.center[1]:.6f}",
                f"{det.center[2]:.6f}",
                f"{det.size[0]:.6f}",
                f"{det.size[1]:.6f}",
                f"{det.size[2]:.6f}",
                "", "", "", "", "",
                len(gt_boxes),
                len(det_boxes),
            ])

    def _message_stamp(self, msg: MarkerArray) -> float:
        for marker in msg.markers:
            stamp = _stamp_to_sec(marker.header.stamp.sec, marker.header.stamp.nanosec)
            if stamp > 0.0:
                return stamp
        return self.get_clock().now().nanoseconds * 1e-9

    def _nearest_gt_id(self, point: tuple[float, float, float], gt_boxes: list[Box3D]) -> str | None:
        if not gt_boxes:
            return None
        nearest = min(gt_boxes, key=lambda box: _distance(point, box.center))
        return nearest.track_id

    def _nearest_gt(self, point: tuple[float, float, float], gt_boxes: list[Box3D]) -> tuple[str, float] | None:
        if not gt_boxes:
            return None
        nearest = min(gt_boxes, key=lambda box: _distance(point, box.center))
        if nearest.track_id is None:
            return None
        return nearest.track_id, _distance(point, nearest.center)

    def _metrics_from_counts(self, counts: dict[str, int]) -> dict[str, float | int]:
        tp = counts["tp"]
        fp = counts["fp"]
        fn = counts["fn"]
        precision = tp / max(1, tp + fp)
        recall = tp / max(1, tp + fn)
        f1 = 2.0 * precision * recall / (precision + recall) if precision + recall > 0.0 else 0.0
        return {"precision": precision, "recall": recall, "f1": f1, "tp": tp, "fp": fp, "fn": fn}

    def _tracking_summary(self) -> dict[str, float | int]:
        counts = self._tracking_counts
        total_gt = counts["tp"] + counts["fn"]
        mota = 1.0 - (counts["fp"] + counts["fn"] + self._idsw) / max(1, total_gt)
        ratios = [
            self._gt_matched_frames.get(gt_id, 0) / max(1, seen)
            for gt_id, seen in self._gt_seen_frames.items()
        ]
        mt = sum(1 for r in ratios if r >= 0.8)
        ml = sum(1 for r in ratios if r <= 0.2)
        id_precision = counts["tp"] / max(1, counts["tp"] + counts["fp"] + self._idsw)
        id_recall = counts["tp"] / max(1, counts["tp"] + counts["fn"] + self._idsw)
        idf1 = 2.0 * id_precision * id_recall / (id_precision + id_recall) if id_precision + id_recall > 0.0 else 0.0
        return {
            "mota": mota, "idf1": idf1, "idsw": self._idsw, "frag": self._frag,
            "mt": mt, "ml": ml, "match_threshold_m": _TRACKING_THRESHOLD_M,
        }

    def _prediction_summary(self) -> dict[str, float | int | None]:
        def mean_until(max_step: int) -> float | None:
            err = 0.0
            count = 0
            for step, step_err in self._pred_err_sum_by_step.items():
                if step <= max_step:
                    err += step_err
                    count += self._pred_err_count_by_step.get(step, 0)
            return err / count if count else None

        max_step = max(self._pred_err_sum_by_step.keys(), default=0)
        fde = None
        if max_step:
            count = self._pred_err_count_by_step.get(max_step, 0)
            fde = self._pred_err_sum_by_step[max_step] / count if count else None
        return {
            "ade_1s": mean_until(max(1, round(1.0 / self._prediction_dt_sec))),
            "ade_2_5s": mean_until(max(1, round(2.5 / self._prediction_dt_sec))),
            "fde": fde,
            "samples": sum(self._pred_err_count_by_step.values()),
        }

    def _build_summary(self) -> dict[str, object]:
        summary: dict[str, object] = {
            "duration_sec": 0.0 if self._first_stamp is None else max(0.0, self._latest_gt_stamp - self._first_stamp - self._warmup_sec),
            "total_frames": self._frames,
            "gt_bbox_size": list(self._gt_size),
            "gt_bbox_center_y_offset_m": self._gt_center_y_offset_m,
            "gt_bbox_center_z_offset_m": self._gt_center_z_offset_m,
            "include_static_obstacles": self._include_static_obstacles,
            "gt_dynamic_topic": self._gt_topic,
            "gt_static_topic": self._gt_obstacle_topic if self._include_static_obstacles else None,
            "det_topic": self._det_topic,
            "tracking_det_topic": self._tracking_det_topic,
            "tracking_pairs_csv_path": str(self._tracking_pairs_csv_path),
        }

        center_dist: dict[str, object] = {}
        for threshold in _CENTER_THRESHOLDS:
            key = f"{threshold:.1f}m"
            metrics = self._metrics_from_counts(self._center_counts[threshold])
            err_info = self._center_err[threshold]
            axis_info = self._center_axis_err[threshold]
            axis_count = axis_info["count"]
            metrics["mean_error_m"] = err_info["sum"] / err_info["count"] if err_info["count"] else None
            metrics["mean_xy_error_m"] = axis_info["sum_dxy"] / axis_count if axis_count else None
            metrics["mean_dx_m"] = axis_info["sum_dx"] / axis_count if axis_count else None
            metrics["mean_dy_m"] = axis_info["sum_dy"] / axis_count if axis_count else None
            metrics["mean_dz_m"] = axis_info["sum_dz"] / axis_count if axis_count else None
            metrics["mean_abs_dx_m"] = axis_info["sum_abs_dx"] / axis_count if axis_count else None
            metrics["mean_abs_dy_m"] = axis_info["sum_abs_dy"] / axis_count if axis_count else None
            metrics["mean_abs_dz_m"] = axis_info["sum_abs_dz"] / axis_count if axis_count else None
            visible_recall = self._visible_tp_total[threshold] / self._visible_gt_total if self._visible_gt_total else 0.0
            precision = float(metrics["precision"])
            visible_f1 = (
                2.0 * precision * visible_recall / (precision + visible_recall)
                if precision + visible_recall > 0.0 else 0.0
            )
            metrics["visible_gt"] = self._visible_gt_total
            metrics["visible_tp"] = self._visible_tp_total[threshold]
            metrics["visible_recall"] = visible_recall
            metrics["visible_f1"] = visible_f1
            metrics["visible_gt_per_frame"] = self._visible_gt_total / self._frames if self._frames else None
            lidar_visible_recall = self._lidar_visible_tp_total[threshold] / self._lidar_visible_gt_total if self._lidar_visible_gt_total else 0.0
            lidar_visible_f1 = (
                2.0 * precision * lidar_visible_recall / (precision + lidar_visible_recall)
                if precision + lidar_visible_recall > 0.0 else 0.0
            )
            metrics["lidar_visible_gt"] = self._lidar_visible_gt_total
            metrics["lidar_visible_tp"] = self._lidar_visible_tp_total[threshold]
            metrics["lidar_visible_recall"] = lidar_visible_recall
            metrics["lidar_visible_f1"] = lidar_visible_f1
            metrics["lidar_visible_gt_per_frame"] = self._lidar_visible_gt_total / self._frames if self._frames else None
            center_dist[key] = metrics
        summary["center_distance"] = center_dist

        iou_diag: dict[str, object] = {}
        for threshold in self._iou_thresholds:
            key = f"iou_{threshold:.1f}"
            metrics = self._metrics_from_counts(self._iou_counts[threshold])
            metrics["pos_error"] = self._iou_matched_err_sum / self._iou_matched_err_count if self._iou_matched_err_count else None
            metrics["mean_iou"] = self._iou_matched_iou_sum / self._iou_matched_iou_count if self._iou_matched_iou_count else None
            iou_diag[key] = metrics
        iou_diag["iou_curve"] = {
            f"{threshold:.2f}": self._metrics_from_counts(counts)
            for threshold, counts in self._iou_curve_counts.items()
        }
        iou_diag["best_iou_diagnostic"] = {
            "mean_best_iou_per_gt": self._best_iou_sum / self._best_iou_count if self._best_iou_count else None,
            "max_best_iou": self._best_iou_max,
            "samples": self._best_iou_count,
        }
        summary["iou_diagnostic"] = iou_diag

        tracking_summary = self._tracking_summary()
        tracking_summary["gt_mode"] = "dynamic_only"
        summary["tracking"] = tracking_summary
        summary["prediction"] = self._prediction_summary()
        return summary

    def _emit_progress_log(self, elapsed: float) -> None:
        m1 = self._metrics_from_counts(self._center_counts.get(1.0, {"tp": 0, "fp": 0, "fn": 0}))
        err_info = self._center_err.get(1.0, {"sum": 0.0, "count": 0})
        mean_err = err_info["sum"] / max(1, err_info["count"])
        visible_recall = self._visible_tp_total[1.0] / self._visible_gt_total if self._visible_gt_total else 0.0
        self.get_logger().info(
            f"[advanced_eval {elapsed:.1f}s {self._frames}fr] "
            f"CD@1m P={m1['precision']:.2f} R={m1['recall']:.2f} F1={m1['f1']:.2f} "
            f"visR={visible_recall:.2f} err={mean_err:.2f}m IDSW={self._idsw}"
        )
        self._csv_fp.flush()
        self._matched_pairs_fp.flush()
        self._tracking_pairs_fp.flush()

    def _check_duration(self) -> None:
        if self._finished or self._first_stamp is None:
            return
        elapsed = self._latest_gt_stamp - self._first_stamp
        if elapsed >= self._warmup_sec + self._eval_duration_sec:
            self._finish()

    def _finish(self) -> None:
        if self._finished:
            return
        self._finished = True
        summary = self._build_summary()
        self._summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        self._csv_fp.flush()
        self._matched_pairs_fp.flush()
        self._tracking_pairs_fp.flush()
        self.get_logger().info(f"advanced_eval summary written: {self._summary_path}")

    def destroy_node(self) -> bool:
        try:
            self._finish()
            self._csv_fp.close()
            self._matched_pairs_fp.close()
            self._tracking_pairs_fp.close()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = AdvancedEvaluator()

    def _on_signal(signum: int, frame) -> None:
        del signum, frame
        try:
            node._finish()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGTERM, _on_signal)
    signal.signal(signal.SIGINT, _on_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
