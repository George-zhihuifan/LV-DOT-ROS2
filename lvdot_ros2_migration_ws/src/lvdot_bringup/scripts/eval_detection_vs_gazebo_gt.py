#!/usr/bin/env python3
import argparse
import math
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from depth_eval_msgs.msg import AgentPoseArray
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2DArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


@dataclass
class Box:
    x: float
    y: float
    z: float
    sx: float
    sy: float
    sz: float


def dist(a: Box, b: Box) -> float:
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)


def marker_to_box(m: Marker) -> Box:
    return Box(
        x=m.pose.position.x,
        y=m.pose.position.y,
        z=m.pose.position.z,
        sx=m.scale.x,
        sy=m.scale.y,
        sz=m.scale.z,
    )


def line_list_marker_to_box(m: Marker) -> Box:
    xs = [p.x for p in m.points]
    ys = [p.y for p in m.points]
    zs = [p.z for p in m.points]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    return Box(
        x=(xmin + xmax) / 2.0,
        y=(ymin + ymax) / 2.0,
        z=(zmin + zmax) / 2.0,
        sx=max(0.0, xmax - xmin),
        sy=max(0.0, ymax - ymin),
        sz=max(0.0, zmax - zmin),
    )


def pose_to_box(p: Pose) -> Box:
    # GT size is unknown from ModelStates; only center is strict GT.
    return Box(x=p.position.x, y=p.position.y, z=p.position.z, sx=0.0, sy=0.0, sz=0.0)


class EvalNode(Node):
    def __init__(
        self,
        duration: float,
        gt_source: str,
        visible_only: bool,
        depth_min: float,
        depth_max: float,
        include_qcgaf: bool,
        match_gate_m: float,
    ):
        super().__init__("eval_detection_vs_gazebo_gt")
        self.duration = duration
        self.gt_source = gt_source
        self.visible_only = visible_only
        self.match_gate_m = match_gate_m
        self.start = time.time()
        self.gt: Dict[str, Box] = {}
        self.det: Dict[str, List[Box]] = {
            "uv": [],
            "db": [],
            "lidar": [],
            "fused": [],
            "tracked": [],
        }
        self.metric_order = ["uv", "db", "lidar", "fused", "tracked"]
        if include_qcgaf:
            self.det["qcgaf_fused"] = []
            self.metric_order.append("qcgaf_fused")
        self.acc: Dict[str, Dict[str, float]] = {
            k: {"frames": 0.0, "gt_count": 0.0, "matched": 0.0, "sum_center_err": 0.0}
            for k in self.det.keys()
        }
        self.yolo_2d_boxes: List[Tuple[float, float, float, float]] = []
        self.yolo_acc = {"frames": 0.0, "gt_visible": 0.0, "matched": 0.0}

        self.create_subscription(AgentPoseArray, "/pedestrian_sim/agent_states", self.on_gt_agents, 10)
        self.create_subscription(TFMessage, "/world/pedestrian_prototype/pose/info", self.on_gt_pose_info, 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.on_uav_pose, 10)
        yolo_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Detection2DArray,
            "/yolo_detector/detected_bounding_boxes",
            self.on_yolo_detections,
            yolo_qos,
        )
        self.create_subscription(MarkerArray, "/onboard_detector/uv_bboxes", lambda m: self.on_det("uv", m), 10)
        self.create_subscription(MarkerArray, "/onboard_detector/dbscan_bboxes", lambda m: self.on_det("db", m), 10)
        self.create_subscription(MarkerArray, "/onboard_detector/lidar_bboxes", lambda m: self.on_det("lidar", m), 10)
        self.create_subscription(MarkerArray, "/onboard_detector/filtered_bboxes", lambda m: self.on_det("fused", m), 10)
        self.create_subscription(MarkerArray, "/onboard_detector/tracked_bboxes", lambda m: self.on_det("tracked", m), 10)
        if include_qcgaf:
            self.create_subscription(MarkerArray, "/qcgaf/fused_bboxes", lambda m: self.on_det("qcgaf_fused", m), 10)
        self.timer = self.create_timer(0.5, self.tick)

        self.uav_pose: Box = Box(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.uav_qxyzw: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.fx = 337.35705085528514
        self.fy = 337.35705085528514
        self.cx = 320.0
        self.cy = 240.0
        self.img_w = 640
        self.img_h = 480
        self.R_bc = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ], dtype=float)
        self.t_bc = np.array([0.18, 0.0, 0.06], dtype=float)

    def on_uav_pose(self, msg: PoseStamped) -> None:
        self.uav_pose = Box(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            z=msg.pose.position.z,
            sx=0.0,
            sy=0.0,
            sz=0.0,
        )
        self.uav_qxyzw = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )

    def on_gt_agents(self, msg: AgentPoseArray) -> None:
        if self.gt_source != "agents":
            return
        gt = {}
        for agent in msg.agents:
            gt[agent.name] = pose_to_box(agent.pose)
        self.gt = gt

    def on_gt_pose_info(self, msg: TFMessage) -> None:
        if self.gt_source != "pose_info":
            return
        pts = []
        for t in msg.transforms:
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
                continue
            if z < -0.1 or z > 2.3:
                continue
            if x < -5.0 or x > 30.0 or y < -15.0 or y > 15.0:
                continue
            # Remove UAV body / nearby links from GT pool.
            if math.dist((x, y, z), (self.uav_pose.x, self.uav_pose.y, self.uav_pose.z)) < 1.2:
                continue
            pts.append((x, y, z))
        # Deduplicate close transforms (links/joints of same model).
        uniq: List[Tuple[float, float, float]] = []
        for p in pts:
            if all(math.dist(p, q) > 0.8 for q in uniq):
                uniq.append(p)
        self.gt = {f"obj_{i}": Box(x=p[0], y=p[1], z=p[2], sx=0.0, sy=0.0, sz=0.0) for i, p in enumerate(uniq)}

    def on_det(self, key: str, msg: MarkerArray) -> None:
        boxes: List[Box] = []
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            if m.type == Marker.CUBE:
                boxes.append(marker_to_box(m))
                continue
            if m.type == Marker.LINE_LIST and len(m.points) >= 8:
                boxes.append(line_list_marker_to_box(m))
                continue
        self.det[key] = boxes

    def on_yolo_detections(self, msg: Detection2DArray) -> None:
        boxes: List[Tuple[float, float, float, float]] = []
        for det in msg.detections:
            cx = float(det.bbox.center.position.x)
            cy = float(det.bbox.center.position.y)
            w = float(det.bbox.size_x)
            h = float(det.bbox.size_y)
            if w <= 1.0 or h <= 1.0:
                continue
            x1 = cx - 0.5 * w
            y1 = cy - 0.5 * h
            x2 = cx + 0.5 * w
            y2 = cy + 0.5 * h
            boxes.append((x1, y1, x2, y2))
        self.yolo_2d_boxes = boxes

    def _match_frame(self, gt_boxes: List[Box], det_boxes: List[Box], gate: float = 1.5) -> Tuple[int, float]:
        if not gt_boxes or not det_boxes:
            return 0, 0.0
        used = set()
        matched = 0
        err = 0.0
        for g in gt_boxes:
            best_i = -1
            best_d = 1e9
            for i, dbox in enumerate(det_boxes):
                if i in used:
                    continue
                dd = dist(g, dbox)
                if dd < best_d:
                    best_d = dd
                    best_i = i
            if best_i >= 0 and best_d <= gate:
                used.add(best_i)
                matched += 1
                err += best_d
        return matched, err

    def _quat_to_rot(self, q: Tuple[float, float, float, float]) -> np.ndarray:
        x, y, z, w = q
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], dtype=float)

    def _visible_gt(self, gt_boxes: List[Box]) -> List[Box]:
        R_wb = self._quat_to_rot(self.uav_qxyzw)
        t_wb = np.array([self.uav_pose.x, self.uav_pose.y, self.uav_pose.z], dtype=float)
        visible: List[Box] = []
        for g in gt_boxes:
            p_w = np.array([g.x, g.y, g.z], dtype=float)
            p_b = R_wb.T @ (p_w - t_wb)
            p_c = self.R_bc.T @ (p_b - self.t_bc)
            zc = float(p_c[2])
            if zc < self.depth_min or zc > self.depth_max:
                continue
            u = self.fx * (float(p_c[0]) / zc) + self.cx
            v = self.fy * (float(p_c[1]) / zc) + self.cy
            if 0.0 <= u < self.img_w and 0.0 <= v < self.img_h:
                visible.append(g)
        return visible

    def _project_to_image(self, g: Box) -> Tuple[bool, float, float]:
        R_wb = self._quat_to_rot(self.uav_qxyzw)
        t_wb = np.array([self.uav_pose.x, self.uav_pose.y, self.uav_pose.z], dtype=float)
        p_w = np.array([g.x, g.y, g.z], dtype=float)
        p_b = R_wb.T @ (p_w - t_wb)
        p_c = self.R_bc.T @ (p_b - self.t_bc)
        zc = float(p_c[2])
        if zc < self.depth_min or zc > self.depth_max:
            return False, 0.0, 0.0
        u = self.fx * (float(p_c[0]) / zc) + self.cx
        v = self.fy * (float(p_c[1]) / zc) + self.cy
        if not (0.0 <= u < self.img_w and 0.0 <= v < self.img_h):
            return False, 0.0, 0.0
        return True, u, v

    def tick(self) -> None:
        gt_list = list(self.gt.values())
        if self.visible_only:
            gt_list = self._visible_gt(gt_list)
            # Skip frames that have no visible GT to avoid denominator collapse (0/0 runs).
            if not gt_list:
                return
        for k, det_list in self.det.items():
            m, e = self._match_frame(gt_list, det_list, gate=self.match_gate_m)
            st = self.acc[k]
            st["frames"] += 1.0
            st["gt_count"] += float(len(gt_list))
            st["matched"] += float(m)
            st["sum_center_err"] += float(e)

        # YOLO 2D visible-point recall: projected visible GT center inside any YOLO bbox.
        vis_pts: List[Tuple[float, float]] = []
        for g in list(self.gt.values()):
            ok, u, v = self._project_to_image(g)
            if ok:
                vis_pts.append((u, v))
        yolo_match = 0
        for (u, v) in vis_pts:
            if any((x1 <= u <= x2 and y1 <= v <= y2) for (x1, y1, x2, y2) in self.yolo_2d_boxes):
                yolo_match += 1
        self.yolo_acc["frames"] += 1.0
        self.yolo_acc["gt_visible"] += float(len(vis_pts))
        self.yolo_acc["matched"] += float(yolo_match)

        if time.time() - self.start >= self.duration:
            self.report()
            rclpy.shutdown()

    def report(self) -> None:
        print(f"=== Detection vs GT (center-based, source={self.gt_source}) ===")
        print(f"Duration: {self.duration:.1f}s")
        print(f"Match gate(m): {self.match_gate_m:.2f}")
        print(f"Current GT objects: {len(self.gt)}")
        print(f"Visible-only scoring: {int(self.visible_only)}")
        for k in self.metric_order:
            st = self.acc[k]
            matched = st["matched"]
            gt_total = max(1.0, st["gt_count"])
            recall = matched / gt_total
            mean_err = st["sum_center_err"] / max(1.0, matched)
            print(
                f"{k:7s} recall={recall:.3f} "
                f"matched={int(matched)}/{int(st['gt_count'])} "
                f"mean_center_err={mean_err:.3f}m"
            )
        yv = max(1.0, self.yolo_acc["gt_visible"])
        yr = self.yolo_acc["matched"] / yv
        print(
            f"yolo_2d recall={yr:.3f} "
            f"matched={int(self.yolo_acc['matched'])}/{int(self.yolo_acc['gt_visible'])}"
        )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--gt-source", choices=["agents", "pose_info"], default="agents")
    ap.add_argument("--visible-only", action="store_true", default=False)
    ap.add_argument("--depth-min", type=float, default=0.2)
    ap.add_argument("--depth-max", type=float, default=12.0)
    ap.add_argument("--include-qcgaf", action="store_true", default=False)
    ap.add_argument("--match-gate-m", type=float, default=1.5)
    args = ap.parse_args()
    if args.depth_max <= args.depth_min:
        raise SystemExit("--depth-max must be greater than --depth-min")
    if args.match_gate_m <= 0.0:
        raise SystemExit("--match-gate-m must be > 0")
    rclpy.init()
    node = EvalNode(
        args.duration,
        args.gt_source,
        args.visible_only,
        args.depth_min,
        args.depth_max,
        args.include_qcgaf,
        args.match_gate_m,
    )
    rclpy.spin(node)


if __name__ == "__main__":
    main()
