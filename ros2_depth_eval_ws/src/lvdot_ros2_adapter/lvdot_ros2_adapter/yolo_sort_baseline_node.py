#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.optimize import linear_sum_assignment
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_to_matrix(msg: PoseStamped) -> np.ndarray:
    p = msg.pose.position
    q = msg.pose.orientation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    # Quaternion -> rotation matrix
    r00 = 1.0 - 2.0 * (y * y + z * z)
    r01 = 2.0 * (x * y - z * w)
    r02 = 2.0 * (x * z + y * w)
    r10 = 2.0 * (x * y + z * w)
    r11 = 1.0 - 2.0 * (x * x + z * z)
    r12 = 2.0 * (y * z - x * w)
    r20 = 2.0 * (x * z - y * w)
    r21 = 2.0 * (y * z + x * w)
    r22 = 1.0 - 2.0 * (x * x + y * y)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]], dtype=np.float64)
    T[:3, 3] = np.array([float(p.x), float(p.y), float(p.z)], dtype=np.float64)
    return T


@dataclass
class Track:
    tid: int
    x: np.ndarray
    P: np.ndarray
    last_t: float
    hits: int = 1
    miss: int = 0


class YoloSortBaselineNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_sort_baseline_node")
        self.bridge = CvBridge()

        self.declare_parameter("depth_intrinsics", [337.357, 337.357, 320.0, 240.0])
        self.declare_parameter("body_to_camera", [0.0, 0.0, 1.0, 0.30, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("depth_scale", 1000.0)
        self.declare_parameter("default_bbox_size", [0.5, 0.5, 1.7])
        self.declare_parameter("sort_max_age", 5)
        self.declare_parameter("sort_min_hits", 3)
        self.declare_parameter("sort_iou_threshold", 1.0)  # used as 3D distance threshold (m)
        self.declare_parameter("depth_sample_radius", 3)

        self.fx, self.fy, self.cx, self.cy = [float(v) for v in self.get_parameter("depth_intrinsics").value]
        self.cam_to_body = np.array(self.get_parameter("body_to_camera").value, dtype=np.float64).reshape(4, 4)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.default_size = [float(v) for v in self.get_parameter("default_bbox_size").value]
        self.max_age = int(self.get_parameter("sort_max_age").value)
        self.min_hits = int(self.get_parameter("sort_min_hits").value)
        self.match_dist = float(self.get_parameter("sort_iou_threshold").value)
        self.depth_radius = int(self.get_parameter("depth_sample_radius").value)

        self.latest_depth: Optional[np.ndarray] = None
        self.latest_pose: Optional[PoseStamped] = None
        self.tracks: list[Track] = []
        self.next_track_id = 0

        self.create_subscription(Detection2DArray, "/yolo_detector/detected_bounding_boxes", self.on_yolo, qos_profile_sensor_data)
        self.create_subscription(Image, "/rgbd_camera/depth_image", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose", self.on_pose, qos_profile_sensor_data)
        self.pub = self.create_publisher(MarkerArray, "/yolo_sort/tracked_bboxes", qos_profile_sensor_data)

    def on_depth(self, msg: Image) -> None:
        try:
            if msg.encoding == "16UC1":
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
            else:
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough").astype(np.float32)
        except Exception:
            self.latest_depth = None

    def on_pose(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def sample_depth(self, u: float, v: float) -> Optional[float]:
        if self.latest_depth is None:
            return None
        h, w = self.latest_depth.shape[:2]
        cx = int(round(u))
        cy = int(round(v))
        r = self.depth_radius
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        patch = self.latest_depth[y0:y1, x0:x1]
        if patch.size == 0:
            return None
        valid = patch[np.isfinite(patch) & (patch > 0.0)]
        if valid.size < 5:
            return None
        z = float(np.median(valid))
        if z > 100.0:  # likely mm
            z = z / max(self.depth_scale, 1e-6)
        return z

    def unproject_to_world(self, u: float, v: float, depth_m: float) -> Optional[np.ndarray]:
        if self.latest_pose is None:
            return None
        x_cam = (u - self.cx) * depth_m / self.fx
        y_cam = (v - self.cy) * depth_m / self.fy
        z_cam = depth_m
        p_cam = np.array([x_cam, y_cam, z_cam, 1.0], dtype=np.float64)
        p_body = self.cam_to_body @ p_cam
        T_wb = pose_to_matrix(self.latest_pose)
        p_world = T_wb @ p_body
        return p_world[:3]

    @staticmethod
    def kf_predict(track: Track, now_t: float) -> None:
        dt = max(1e-3, now_t - track.last_t)
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        Q = np.diag([0.1, 0.1, 0.1, 1.0, 1.0, 1.0]).astype(np.float64)
        track.x = F @ track.x
        track.P = F @ track.P @ F.T + Q
        track.last_t = now_t

    @staticmethod
    def kf_update(track: Track, z: np.ndarray) -> None:
        H = np.zeros((3, 6), dtype=np.float64)
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        H[2, 2] = 1.0
        R = np.diag([0.5, 0.5, 0.5]).astype(np.float64)
        y = z - (H @ track.x)
        S = H @ track.P @ H.T + R
        K = track.P @ H.T @ np.linalg.inv(S)
        track.x = track.x + K @ y
        I = np.eye(6, dtype=np.float64)
        track.P = (I - K @ H) @ track.P

    def sort_step(self, detections: list[np.ndarray], now_t: float) -> None:
        for t in self.tracks:
            self.kf_predict(t, now_t)

        if not self.tracks and not detections:
            return
        if not self.tracks:
            for z in detections:
                x = np.zeros(6, dtype=np.float64)
                x[:3] = z
                P = np.diag([1.0, 1.0, 1.0, 10.0, 10.0, 10.0]).astype(np.float64)
                self.tracks.append(Track(self.next_track_id, x, P, now_t))
                self.next_track_id += 1
            return

        m, n = len(self.tracks), len(detections)
        if n > 0:
            C = np.full((m, n), 1e6, dtype=np.float64)
            for i, tr in enumerate(self.tracks):
                pred = tr.x[:3]
                for j, z in enumerate(detections):
                    C[i, j] = float(np.linalg.norm(pred - z))
            rows, cols = linear_sum_assignment(C)
            matched_t = set()
            matched_d = set()
            for i, j in zip(rows.tolist(), cols.tolist()):
                if C[i, j] > self.match_dist:
                    continue
                tr = self.tracks[i]
                self.kf_update(tr, detections[j])
                tr.hits += 1
                tr.miss = 0
                matched_t.add(i)
                matched_d.add(j)

            for i, tr in enumerate(self.tracks):
                if i not in matched_t:
                    tr.miss += 1
            for j, z in enumerate(detections):
                if j not in matched_d:
                    x = np.zeros(6, dtype=np.float64)
                    x[:3] = z
                    P = np.diag([1.0, 1.0, 1.0, 10.0, 10.0, 10.0]).astype(np.float64)
                    self.tracks.append(Track(self.next_track_id, x, P, now_t))
                    self.next_track_id += 1
        else:
            for tr in self.tracks:
                tr.miss += 1

        self.tracks = [t for t in self.tracks if t.miss <= self.max_age]

    def publish_markers(self, stamp) -> None:
        msg = MarkerArray()
        for tr in self.tracks:
            if tr.hits < self.min_hits:
                continue
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp
            m.ns = "yolo_sort"
            m.id = int(tr.tid)
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(tr.x[0])
            m.pose.position.y = float(tr.x[1])
            m.pose.position.z = float(tr.x[2])
            m.pose.orientation.w = 1.0
            m.scale.x = self.default_size[0]
            m.scale.y = self.default_size[1]
            m.scale.z = self.default_size[2]
            m.color.a = 0.85
            m.color.r = 0.1
            m.color.g = 0.8
            m.color.b = 0.2
            m.lifetime.sec = 0
            m.lifetime.nanosec = 200_000_000
            msg.markers.append(m)
        self.pub.publish(msg)

    def on_yolo(self, msg: Detection2DArray) -> None:
        if self.latest_depth is None or self.latest_pose is None:
            return
        detections: list[np.ndarray] = []
        for det in msg.detections:
            u = float(det.bbox.center.position.x)
            v = float(det.bbox.center.position.y)
            z = self.sample_depth(u, v)
            if z is None or z < 0.3 or z > 12.0:
                continue
            p = self.unproject_to_world(u, v, z)
            if p is not None:
                detections.append(p)
        now_t = stamp_to_sec(msg.header.stamp)
        self.sort_step(detections, now_t)
        self.publish_markers(msg.header.stamp)


def main() -> None:
    rclpy.init()
    node = YoloSortBaselineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
