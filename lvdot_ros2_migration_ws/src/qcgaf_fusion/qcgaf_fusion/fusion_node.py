#!/usr/bin/env python3
from __future__ import annotations

import os
import time
from collections import deque

import message_filters
import numpy as np
import rclpy
import torch
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

from qcgaf_fusion.src.model import QCGAF


CAMERA_DEFAULT_FEATURES = np.array([0.5, 0.15, 0.5], dtype=np.float32)
LIDAR_DEFAULT_FEATURES = np.array([0.4, 0.1, 0.1, 0.08], dtype=np.float32)


def _expected_image_bytes(msg: Image):
    if msg.encoding in ('rgb8', 'bgr8'):
        return int(msg.height) * int(msg.width) * 3
    if msg.encoding == 'mono8':
        return int(msg.height) * int(msg.width)
    if msg.encoding == '32FC1':
        return int(msg.height) * int(msg.width) * 4
    if msg.encoding == '16UC1':
        return int(msg.height) * int(msg.width) * 2
    return None


class QualityEstimator:
    def __init__(self):
        self.imu_buffer = deque(maxlen=50)
        self.depth_prev = None
        self.brightness_history = deque(maxlen=30)
        # Ablation hook (default OFF): if env QCGAF_FREEZE_QUALITY holds a float,
        # the runtime quality vector is replaced by that constant, removing all
        # quality-driven gating adaptivity. Detection features are untouched, so
        # only the quality->gating pathway is ablated.
        _fq = os.environ.get('QCGAF_FREEZE_QUALITY', '')
        try:
            self.freeze_quality = float(_fq) if _fq != '' else None
        except ValueError:
            self.freeze_quality = None
        self.last_motion_quality = 0.8

    def update_imu(self, imu_msg: Imu):
        acc = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
        ])
        self.imu_buffer.append(acc)

    def update_depth(self, depth_array: np.ndarray):
        self.depth_prev = depth_array.copy()

    def compute(
        self,
        color_image=None,
        depth_image=None,
        yolo_conf=0.8,
        yolo_count=0,
        lidar_points=100,
        lidar_stds=None,
        n_cam=0,
        n_lidar=0,
        cam_lidar_dt=0.0,
        sensor_age=0.0,
    ):
        raw_brightness, edge = color_metrics(color_image)
        depth_valid_ratio, depth_var, depth_temporal = depth_metrics(depth_image, self.depth_prev)

        if len(self.brightness_history) >= 3:
            i_avg = float(np.mean(self.brightness_history))
            i_curr = float(raw_brightness)
            denom = max(i_curr, i_avg) + 1e-6
            brightness = float(np.clip(1.0 - abs(i_curr - i_avg) / denom, 0.0, 1.0))
        else:
            brightness = 1.0
        if color_image is not None:
            self.brightness_history.append(float(raw_brightness))

        # Runtime quality should reflect "how reliable is the current branch for
        # this fusion tick", not generic full-frame image aesthetics.
        cam_count_quality = float(np.clip(n_cam / max(float(max(yolo_count, n_cam, 1)), 1.0), 0.0, 1.0))
        cam_support = float(np.clip(0.65 * depth_valid_ratio + 0.35 * cam_count_quality, 0.0, 1.0))
        cam_stability = float(np.clip(0.55 * depth_temporal + 0.25 * brightness + 0.20 * (1.0 - depth_var), 0.0, 1.0))

        pts_per_det = float(lidar_points) / float(max(n_lidar, 1))
        lidar_support = float(np.clip(pts_per_det / 12000.0, 0.0, 1.0))
        if lidar_stds is None:
            lidar_stds = np.array([0.1, 0.1, 0.08], dtype=np.float32)
        spread = np.asarray(lidar_stds, dtype=np.float32)
        spread_norm = np.array([0.35, 0.35, 0.30], dtype=np.float32)
        lidar_compact = float(np.clip(1.0 - np.mean(np.clip(spread / spread_norm, 0.0, 1.0)), 0.0, 1.0))

        motion_quality = self.last_motion_quality
        if len(self.imu_buffer) > 10:
            acc_arr = np.array(self.imu_buffer)
            vib = float(acc_arr.var(axis=0).sum())
            motion_quality = float(np.clip(1.0 / (1.0 + vib / 40.0), 0.0, 1.0))
            self.last_motion_quality = motion_quality

        sync_q = float(np.clip(np.exp(-max(cam_lidar_dt, 0.0) / 0.12) * np.exp(-max(sensor_age, 0.0) / 0.25), 0.0, 1.0))
        cam_conf = float(np.clip(0.70 + 0.30 * yolo_conf, 0.0, 1.0))
        cam_presence = float(np.clip(0.55 * cam_count_quality + 0.45 * sync_q, 0.0, 1.0))
        lidar_quality = float(np.clip(0.45 * lidar_support + 0.55 * lidar_compact, 0.0, 1.0))
        temporal_quality = float(np.clip(0.50 * depth_temporal + 0.50 * sync_q, 0.0, 1.0))

        q = np.array([
            cam_conf,
            cam_support,
            cam_stability,
            cam_presence,
            lidar_quality,
            motion_quality,
            temporal_quality,
        ], dtype=np.float32)

        if depth_image is not None:
            self.update_depth(depth_image)

        camera_extra = np.array([cam_support, depth_var, cam_count_quality], dtype=np.float32)
        lidar_extra = np.concatenate([
            np.array([lidar_support], dtype=np.float32),
            spread.astype(np.float32),
        ])
        if self.freeze_quality is not None:
            q = np.full(7, self.freeze_quality, dtype=np.float32)
        return q, camera_extra, lidar_extra


def image_msg_to_numpy(msg: Image):
    expected = _expected_image_bytes(msg)
    if expected is None:
        return None
    if len(msg.data) != expected:
        return None
    if msg.encoding in ('rgb8', 'bgr8'):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == 'bgr8':
            arr = arr[..., ::-1]
        return arr.copy()
    if msg.encoding == 'mono8':
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width).copy()
    if msg.encoding == '32FC1':
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width).copy()
    if msg.encoding == '16UC1':
        arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32)
        return arr / 1000.0
    return None


def color_metrics(color_image: np.ndarray):
    if color_image is None:
        return 0.5, 0.2
    gray = color_image.mean(axis=2).astype(np.float32) if color_image.ndim == 3 else color_image.astype(np.float32)
    brightness = float(np.clip(gray.mean() / 255.0, 0.0, 1.0))
    gx = np.diff(gray, axis=1)
    gy = np.diff(gray, axis=0)
    edge = float(np.clip(((gx.var() + gy.var()) * 0.5) / 1000.0, 0.0, 1.0))
    return brightness, edge


def depth_metrics(depth_image: np.ndarray, prev_depth: np.ndarray):
    if depth_image is None:
        return 0.5, 0.15, 0.8
    safe_depth = np.where(np.isfinite(depth_image), depth_image, 0.0)
    valid_mask = safe_depth > 0.0
    valid = safe_depth[valid_mask]
    if valid.size == 0:
        return 0.0, 1.0, 0.0
    valid_ratio = float(np.clip(valid.size / safe_depth.size, 0.0, 1.0))
    depth_var = float(np.clip(valid.var() / 10.0, 0.0, 1.0))
    temporal = 0.8
    if prev_depth is not None:
        h = min(safe_depth.shape[0], prev_depth.shape[0])
        w = min(safe_depth.shape[1], prev_depth.shape[1])
        prev_safe = np.where(np.isfinite(prev_depth), prev_depth, 0.0)
        diff = np.abs(safe_depth[:h, :w] - prev_safe[:h, :w])
        diff = diff[np.isfinite(diff)]
        if diff.size > 0:
            temporal = float(np.clip(1.0 - diff.mean() / 2.0, 0.0, 1.0))
    return valid_ratio, depth_var, temporal


def pointcloud_metrics(msg: PointCloud2):
    width = int(getattr(msg, 'width', 0) or 0)
    height = int(getattr(msg, 'height', 0) or 0)
    point_step = int(getattr(msg, 'point_step', 0) or 0)
    count = width * height if width and height else width
    if count <= 0 and point_step > 0:
        count = int(len(msg.data) / point_step)
    stds = np.array([0.1, 0.1, 0.08], dtype=np.float32)
    return count, stds


def center_detections(cam_dets, lidar_dets, cam_mask, lidar_mask):
    center_inputs = []
    if cam_mask.any():
        center_inputs.append(cam_dets[cam_mask, :3])
    if lidar_mask.any():
        center_inputs.append(lidar_dets[lidar_mask, :3])
    if not center_inputs:
        return cam_dets, lidar_dets, np.zeros(3, dtype=np.float32)
    frame_center = np.concatenate(center_inputs, axis=0).mean(axis=0).astype(np.float32)
    cam_centered = cam_dets.copy()
    lidar_centered = lidar_dets.copy()
    cam_centered[cam_mask, :3] -= frame_center
    lidar_centered[lidar_mask, :3] -= frame_center
    return cam_centered, lidar_centered, frame_center


def markers_to_array(markers, max_dets, is_camera, extra_features=None):
    dim = 9 if is_camera else 10
    dets = np.zeros((max_dets, dim), dtype=np.float32)
    mask = np.zeros(max_dets, dtype=bool)
    ids = np.full(max_dets, -1, dtype=np.int32)
    write_idx = 0
    for m in markers:
        # Ignore non-active markers (e.g. DELETEALL).
        if m.action != Marker.ADD:
            continue
        if write_idx >= max_dets:
            break

        x = y = z = 0.0
        w = h = l = 0.0
        if m.type == Marker.CUBE:
            x = float(m.pose.position.x)
            y = float(m.pose.position.y)
            z = float(m.pose.position.z)
            w = float(m.scale.x)
            h = float(m.scale.z)
            l = float(m.scale.y)
        elif m.type == Marker.LINE_LIST and len(m.points) >= 8:
            xs = [p.x for p in m.points]
            ys = [p.y for p in m.points]
            zs = [p.z for p in m.points]
            xmin, xmax = min(xs), max(xs)
            ymin, ymax = min(ys), max(ys)
            zmin, zmax = min(zs), max(zs)
            x = float((xmin + xmax) * 0.5)
            y = float((ymin + ymax) * 0.5)
            z = float((zmin + zmax) * 0.5)
            w = float(max(0.1, xmax - xmin))
            l = float(max(0.1, ymax - ymin))
            h = float(max(0.1, zmax - zmin))
        else:
            continue

        dets[write_idx, :6] = [x, y, z, w, h, l]
        dets[write_idx, 6:] = (
            CAMERA_DEFAULT_FEATURES if is_camera else LIDAR_DEFAULT_FEATURES
        ) if extra_features is None else extra_features
        mask[write_idx] = True
        ids[write_idx] = int(m.id)
        write_idx += 1
    return dets, mask, write_idx, ids


def suppress_duplicate_detections(
    dets: np.ndarray,
    mask: np.ndarray,
    ids: np.ndarray,
    distance_thresh: float,
):
    """Remove near-identical proposals before fusion, without assuming class labels."""
    if distance_thresh <= 0.0:
        return dets, mask, int(mask.sum()), ids

    valid = np.flatnonzero(mask)
    keep = []
    for idx in valid:
        pos = dets[idx, :3]
        size = np.maximum(dets[idx, 3:6], 1e-3)
        duplicate = False
        for kept_idx in keep:
            kept_pos = dets[kept_idx, :3]
            kept_size = np.maximum(dets[kept_idx, 3:6], 1e-3)
            center_dist = float(np.linalg.norm(pos - kept_pos))
            xy_dist = float(np.linalg.norm(pos[:2] - kept_pos[:2]))
            z_dist = float(abs(pos[2] - kept_pos[2]))
            xy_scale = 0.5 * float(max(size[0], size[2], kept_size[0], kept_size[2]))
            if center_dist <= distance_thresh or (xy_dist <= xy_scale and z_dist <= 0.5):
                duplicate = True
                break
        if not duplicate:
            keep.append(idx)

    new_dets = np.zeros_like(dets)
    new_mask = np.zeros_like(mask)
    new_ids = np.full_like(ids, -1)
    for out_idx, src_idx in enumerate(keep[:len(mask)]):
        new_dets[out_idx] = dets[src_idx]
        new_mask[out_idx] = True
        new_ids[out_idx] = ids[src_idx]
    return new_dets, new_mask, len(keep[:len(mask)]), new_ids


def array_to_markers(boxes, cam_mask, marker_ids, frame_id, stamp):
    ma = MarkerArray()
    fallback_id = 0
    for i in range(len(boxes)):
        if not cam_mask[i]:
            break
        conf = 1.0 / (1.0 + np.exp(-boxes[i, 6]))
        marker_id = int(marker_ids[i]) if int(marker_ids[i]) >= 0 else fallback_id
        fallback_id = max(fallback_id + 1, marker_id + 1)

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = 'qcgaf_fused'
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = float(boxes[i, 0])
        m.pose.position.y = float(boxes[i, 1])
        m.pose.position.z = float(boxes[i, 2])
        m.pose.orientation.w = 1.0
        m.scale.x = max(float(boxes[i, 3]), 0.1)
        m.scale.y = max(float(boxes[i, 5]), 0.1)
        m.scale.z = max(float(boxes[i, 4]), 0.1)
        m.color.r = 1.0 - conf
        m.color.g = conf
        m.color.b = 0.2
        m.color.a = max(0.35, min(conf, 0.8))
        m.lifetime = Duration(seconds=0.2).to_msg()
        ma.markers.append(m)
    return ma


class QCGAFNode(Node):
    def __init__(self):
        super().__init__('qcgaf_fusion_node')

        self.declare_parameter('config', '')
        self.declare_parameter('checkpoint', '')
        self.declare_parameter('verbose', False)
        self.declare_parameter('debug_metrics', False)
        self.declare_parameter('enable_lidar_fallback', True)
        # NEW path (default OFF): emit fused boxes for LiDAR detections that have
        # no matching camera detection (recovers targets outside camera FoV, or
        # in-FoV but camera-missed).  See callback per-object promotion block.
        self.declare_parameter('enable_lidar_only_promotion', False)
        self.declare_parameter('lidar_only_match_distance', 0.9)
        self.declare_parameter('post_center_blend_alpha', 0.30)
        self.declare_parameter('post_size_blend_alpha', 0.20)
        self.declare_parameter('post_z_head_drop_alpha', 1.0)
        self.declare_parameter('min_box_size', 0.15)
        self.declare_parameter('max_box_size', 4.0)
        self.declare_parameter('marker_track_match_distance', 0.9)
        self.declare_parameter('marker_track_max_miss', 8)
        self.declare_parameter('marker_track_min_hits', 3)
        self.declare_parameter('marker_track_tentative_max_miss', 1)
        self.declare_parameter('marker_track_ema_alpha', 0.65)
        self.declare_parameter('marker_track_velocity_alpha', 0.60)
        self.declare_parameter('marker_track_publish_hysteresis_miss', 4)
        self.declare_parameter('dedupe_center_distance', 0.45)
        self.declare_parameter('cam_topic', '')
        self.declare_parameter('lidar_topic', '')
        self.declare_parameter('color_topic', '')
        self.declare_parameter('depth_topic', '')
        self.declare_parameter('yolo_topic', '')
        self.declare_parameter('lidar_cloud_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('sync_slop', -1.0)

        config_path = self.get_parameter('config').get_parameter_value().string_value
        ckpt_path = self.get_parameter('checkpoint').get_parameter_value().string_value
        verbose = bool(self.get_parameter('verbose').value)
        self.debug_metrics = bool(self.get_parameter('debug_metrics').value)
        self.enable_lidar_fallback = bool(self.get_parameter('enable_lidar_fallback').value)
        self.enable_lidar_only_promotion = bool(self.get_parameter('enable_lidar_only_promotion').value)
        self.lidar_only_match_distance = float(self.get_parameter('lidar_only_match_distance').value)
        self.post_center_blend_alpha = float(self.get_parameter('post_center_blend_alpha').value)
        self.post_size_blend_alpha = float(self.get_parameter('post_size_blend_alpha').value)
        self.post_z_head_drop_alpha = float(self.get_parameter('post_z_head_drop_alpha').value)
        self.min_box_size = float(self.get_parameter('min_box_size').value)
        self.max_box_size = float(self.get_parameter('max_box_size').value)
        self.marker_track_match_distance = float(self.get_parameter('marker_track_match_distance').value)
        self.marker_track_max_miss = int(self.get_parameter('marker_track_max_miss').value)
        self.marker_track_min_hits = int(self.get_parameter('marker_track_min_hits').value)
        self.marker_track_tentative_max_miss = int(
            self.get_parameter('marker_track_tentative_max_miss').value
        )
        self.marker_track_ema_alpha = float(self.get_parameter('marker_track_ema_alpha').value)
        self.marker_track_velocity_alpha = float(self.get_parameter('marker_track_velocity_alpha').value)
        self.marker_track_publish_hysteresis_miss = int(
            self.get_parameter('marker_track_publish_hysteresis_miss').value
        )
        self.dedupe_center_distance = float(self.get_parameter('dedupe_center_distance').value)

        if not config_path:
            raise RuntimeError('Parameter "config" must be provided.')
        if not os.path.exists(config_path):
            raise RuntimeError(f'Config file not found: {config_path}')
        if not ckpt_path:
            raise RuntimeError('Parameter "checkpoint" must be provided.')
        if not os.path.exists(ckpt_path):
            raise RuntimeError(f'Checkpoint file not found: {ckpt_path}')

        self.verbose = verbose

        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)

        ros_cfg = self.config['ros']
        for key in (
            'cam_topic', 'lidar_topic', 'color_topic', 'depth_topic',
            'yolo_topic', 'lidar_cloud_topic', 'output_topic',
        ):
            value = str(self.get_parameter(key).value).strip()
            if value:
                ros_cfg[key] = value
        sync_slop_param = float(self.get_parameter('sync_slop').value)
        if sync_slop_param > 0.0:
            ros_cfg['sync_slop'] = sync_slop_param
        mcfg = self.config['model']
        self.max_dets = mcfg['max_cam_dets']
        if self.max_dets <= 0:
            raise RuntimeError(f'Invalid model.max_cam_dets={self.max_dets}, must be > 0')
        if float(ros_cfg['sync_slop']) <= 0.0:
            raise RuntimeError(f'Invalid ros.sync_slop={ros_cfg["sync_slop"]}, must be > 0')

        self.device = torch.device('cpu')
        self.model = QCGAF(
            cam_dim=mcfg['cam_dim'],
            lidar_dim=mcfg['lidar_dim'],
            quality_dim=mcfg['quality_dim'],
            hidden_dim=mcfg['hidden_dim'],
            feat_dim=mcfg['feat_dim'],
            output_dim=mcfg['output_dim'],
        ).to(self.device)

        ckpt = torch.load(ckpt_path, map_location=self.device)
        if 'model_state_dict' in ckpt:
            self.model.load_state_dict(ckpt['model_state_dict'])
        else:
            self.model.load_state_dict(ckpt)
        self.model.eval()

        self.get_logger().info(f'QC-GAF model loaded from {ckpt_path}')

        self.quality_est = QualityEstimator()
        # Ablation logging (default OFF): if env QCGAF_QUALITY_LOG is a path, append
        # per-frame [sim_t, 7-dim quality, alpha, eff_alpha, n_cam, frozen] for the
        # quality-vs-gating qualitative figure.
        self._qlog = None
        _qlp = os.environ.get('QCGAF_QUALITY_LOG', '')
        if _qlp:
            try:
                self._qlog = open(_qlp, 'a', buffering=1)
                if self._qlog.tell() == 0:
                    self._qlog.write('t,q_bri,q_edge,q_dep,q_det,q_den,q_vib,q_tmp,'
                                     'alpha,eff_alpha,n_cam,frozen\n')
            except Exception:
                self._qlog = None
        self.latest_color = None
        self.latest_depth = None
        self.latest_yolo_conf = 0.0
        self.latest_yolo_count = 0
        self.latest_lidar_point_count = 0
        self.latest_lidar_stds = np.array([0.1, 0.1, 0.08], dtype=np.float32)
        self.latest_color_stamp = None
        self.latest_depth_stamp = None
        self.latest_yolo_stamp = None
        self.latest_lidar_stamp = None

        marker_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(MarkerArray, ros_cfg['output_topic'], marker_qos)
        # §3.3 quality vector publisher.  Topic is fixed to /qcgaf/quality_vector
        # so detector can subscribe without further configuration; downstream
        # consumes 7 floats [H_bri, H_edge, H_dep, H_yolo, H_den, H_vib, H_dtmp].
        self.quality_pub = self.create_publisher(
            Float32MultiArray, '/qcgaf/quality_vector', marker_qos)

        self.create_subscription(Imu, '/imu/data', self.quality_est.update_imu, sensor_qos)
        self.create_subscription(Image, ros_cfg.get('color_topic', '/camera/color/image_raw'), self.color_callback, sensor_qos)
        self.create_subscription(Image, ros_cfg.get('depth_topic', '/camera/depth/image_raw'), self.depth_callback, sensor_qos)
        # YOLO publisher uses sensor-data QoS (best-effort), so subscriber must match.
        self.create_subscription(Detection2DArray, ros_cfg.get('yolo_topic', '/yolo_detector/detected_bounding_boxes'), self.yolo_callback, sensor_qos)
        self.create_subscription(PointCloud2, ros_cfg.get('lidar_cloud_topic', '/livox/lidar'), self.lidar_cloud_callback, sensor_qos)

        # message_filters subscribers for synchronized marker streams
        self.cam_sub = message_filters.Subscriber(self, MarkerArray, ros_cfg['cam_topic'])
        self.lidar_sub = message_filters.Subscriber(self, MarkerArray, ros_cfg['lidar_topic'])
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.cam_sub, self.lidar_sub],
            queue_size=10,
            slop=float(ros_cfg['sync_slop']),
            allow_headerless=True,
        )
        self.sync.registerCallback(self.callback)
        self.get_logger().info('QC-GAF fusion node ready')
        self._last_empty_cam_warn_s = 0.0
        self._sync_warn_count = 0
        self._frames_total = 0
        self._frames_skipped_no_cam = 0
        self._frames_lidar_fallback = 0
        self._frames_lidar_promoted = 0
        self._sum_lidar_promoted = 0
        self._frames_published = 0
        self._sum_cam_in = 0
        self._sum_lidar_in = 0
        self._sum_out = 0
        self._last_debug_log_s = time.time()
        # Persistent marker IDs and confirmation filter.
        self._next_track_id = 0
        self._tracks = {}  # tid -> {"pos","vel","size","conf","hits","miss","confirmed"}

    def _solve_detection_assignment(self, candidate_lists, det_count: int):
        best_matches = -1
        best_cost = float('inf')
        best_assignment = {}

        def dfs(det_idx: int, used_tids: set[int], matches: int, cost: float, assignment: dict[int, int]):
            nonlocal best_matches, best_cost, best_assignment
            remaining = det_count - det_idx
            if matches + remaining < best_matches:
                return
            if det_idx >= det_count:
                if matches > best_matches or (matches == best_matches and cost < best_cost):
                    best_matches = matches
                    best_cost = cost
                    best_assignment = dict(assignment)
                return

            # Option 1: leave this detection unmatched.
            dfs(det_idx + 1, used_tids, matches, cost, assignment)

            # Option 2: assign one feasible track.
            for tid, cand_cost in candidate_lists[det_idx]:
                if tid in used_tids:
                    continue
                assignment[det_idx] = tid
                used_tids.add(tid)
                dfs(det_idx + 1, used_tids, matches + 1, cost + cand_cost, assignment)
                used_tids.remove(tid)
                assignment.pop(det_idx, None)

        dfs(0, set(), 0, 0.0, {})
        return best_assignment

    def _assign_track_ids(self, boxes_np: np.ndarray, cam_mask: np.ndarray):
        detections = []
        for i in range(len(boxes_np)):
            if not cam_mask[i]:
                break
            detections.append({
                "pos": np.array([boxes_np[i, 0], boxes_np[i, 1], boxes_np[i, 2]], dtype=np.float32),
                "size": np.array([boxes_np[i, 3], boxes_np[i, 5], boxes_np[i, 4]], dtype=np.float32),
                "conf": float(1.0 / (1.0 + np.exp(-boxes_np[i, 6]))),
            })
        assigned_ids = np.full((len(boxes_np),), -1, dtype=np.int32)
        unmatched_dets = set(range(len(detections)))
        matched_tids = set()

        # Global one-to-one assignment under gating. Greedy local sorting was
        # the main source of unnecessary ID switches when targets got close.
        if self._tracks and detections:
            candidate_lists = [[] for _ in detections]
            for tid, tr in self._tracks.items():
                miss = int(tr["miss"])
                steps = float(max(miss + 1, 1))
                predicted_pos = tr["pos"] + tr["vel"] * steps
                speed_xy = float(np.linalg.norm(tr["vel"][:2]))
                for j, det in enumerate(detections):
                    pred_xy = float(np.linalg.norm(predicted_pos[:2] - det["pos"][:2]))
                    last_xy = float(np.linalg.norm(tr["pos"][:2] - det["pos"][:2]))
                    z_err = float(abs(predicted_pos[2] - det["pos"][2]))
                    size_scale = float(max(np.linalg.norm(tr["size"]), 1e-3))
                    size_err = float(np.linalg.norm(tr["size"] - det["size"]) / size_scale)

                    gate_xy = max(
                        self.marker_track_match_distance,
                        0.75 + 0.25 * min(miss, 4) + 0.25 * min(speed_xy, 1.5),
                    )
                    if pred_xy > gate_xy or z_err > 0.75:
                        continue

                    direction_penalty = 0.0
                    obs_step = det["pos"][:2] - tr["pos"][:2]
                    obs_norm = float(np.linalg.norm(obs_step))
                    if obs_norm > 1e-4 and speed_xy > 1e-4:
                        cos_dir = float(np.clip(np.dot(obs_step, tr["vel"][:2]) / (obs_norm * speed_xy), -1.0, 1.0))
                        direction_penalty = 0.20 * (1.0 - cos_dir)

                    cost = (
                        0.62 * pred_xy +
                        0.28 * last_xy +
                        0.12 * z_err +
                        0.10 * min(size_err, 2.0) +
                        direction_penalty +
                        0.04 * miss -
                        0.01 * min(int(tr["hits"]), 10)
                    )
                    candidate_lists[j].append((int(tid), float(cost)))

            for j in range(len(candidate_lists)):
                candidate_lists[j].sort(key=lambda x: x[1])

            assignment = self._solve_detection_assignment(candidate_lists, len(detections))
            for j, tid in assignment.items():
                tr = self._tracks[int(tid)]
                det = detections[j]
                prev_pos = tr["pos"].copy()
                v_alpha = float(np.clip(self.marker_track_velocity_alpha, 0.0, 1.0))
                det_vel = det["pos"] - prev_pos
                tr["vel"] = v_alpha * tr["vel"] + (1.0 - v_alpha) * det_vel
                tr["pos"] = det["pos"]
                a = float(np.clip(self.marker_track_ema_alpha, 0.0, 1.0))
                tr["size"] = a * tr["size"] + (1.0 - a) * det["size"]
                tr["conf"] = a * tr["conf"] + (1.0 - a) * det["conf"]
                tr["hits"] += 1
                tr["miss"] = 0
                tr["confirmed"] = tr["confirmed"] or tr["hits"] >= self.marker_track_min_hits
                matched_tids.add(int(tid))
                assigned_ids[j] = int(tid)
                unmatched_dets.discard(j)

        # Age unmatched tracks with tentative/confirmed split.
        for tid, tr in list(self._tracks.items()):
            if tid in matched_tids:
                continue
            tr["miss"] += 1
            max_miss = self.marker_track_max_miss if tr.get("confirmed", False) else self.marker_track_tentative_max_miss
            if tr["miss"] > max_miss:
                del self._tracks[tid]

        # Spawn new tentative tracks for unmatched detections.
        for j in sorted(unmatched_dets):
            det = detections[j]
            self._tracks[self._next_track_id] = {
                "pos": det["pos"],
                "vel": np.zeros(3, dtype=np.float32),
                "size": det["size"],
                "conf": det["conf"],
                "hits": 1,
                "miss": 0,
                "confirmed": self.marker_track_min_hits <= 1,
            }
            assigned_ids[j] = int(self._next_track_id)
            self._next_track_id += 1
        return assigned_ids

    def _tracks_to_markers(self, frame_id, stamp, current_tids=None):
        ma = MarkerArray()
        for tid, tr in self._tracks.items():
            if current_tids is not None and tid not in current_tids:
                continue
            if not tr.get("confirmed", False):
                continue
            if tr["miss"] > self.marker_track_publish_hysteresis_miss:
                continue
            conf = tr["conf"]
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = 'qcgaf_fused'
            m.id = int(tid)
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = float(tr["pos"][0])
            m.pose.position.y = float(tr["pos"][1])
            m.pose.position.z = float(tr["pos"][2])
            m.pose.orientation.w = 1.0
            m.scale.x = max(float(tr["size"][0]), 0.1)
            m.scale.y = max(float(tr["size"][1]), 0.1)
            m.scale.z = max(float(tr["size"][2]), 0.1)
            m.color.r = 1.0 - conf
            m.color.g = conf
            m.color.b = 0.2
            alpha = max(0.35, min(conf, 0.8))
            if tr["miss"] > 0:
                alpha = max(0.20, alpha * 0.75)
            m.color.a = alpha
            m.lifetime = Duration(seconds=0.2).to_msg()
            ma.markers.append(m)
        return ma

    def color_callback(self, msg: Image):
        try:
            image = image_msg_to_numpy(msg)
            if image is not None:
                self.latest_color = image
                self.latest_color_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as exc:
            self.get_logger().warn(f'color_callback failed: {exc}')

    def depth_callback(self, msg: Image):
        try:
            image = image_msg_to_numpy(msg)
            if image is not None:
                self.latest_depth = image
                self.latest_depth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as exc:
            self.get_logger().warn(f'depth_callback failed: {exc}')

    def yolo_callback(self, msg: Detection2DArray):
        try:
            scores = []
            for det in msg.detections:
                for result in det.results:
                    if np.isfinite(result.hypothesis.score):
                        scores.append(float(result.hypothesis.score))
            self.latest_yolo_conf = float(np.clip(np.mean(scores), 0.0, 1.0)) if scores else 0.0
            self.latest_yolo_count = len(msg.detections)
            self.latest_yolo_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as exc:
            self.get_logger().warn(f'yolo_callback failed: {exc}')

    def lidar_cloud_callback(self, msg: PointCloud2):
        try:
            self.latest_lidar_point_count, self.latest_lidar_stds = pointcloud_metrics(msg)
            self.latest_lidar_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        except Exception as exc:
            self.get_logger().warn(f'lidar_cloud_callback failed: {exc}')

    def callback(self, cam_msg: MarkerArray, lidar_msg: MarkerArray):
        t0 = time.time()
        try:
            cam_t = 0.0
            lidar_t = 0.0
            cam_lidar_dt = 0.0
            if cam_msg.markers and lidar_msg.markers:
                cam_t = cam_msg.markers[0].header.stamp.sec + cam_msg.markers[0].header.stamp.nanosec * 1e-9
                lidar_t = lidar_msg.markers[0].header.stamp.sec + lidar_msg.markers[0].header.stamp.nanosec * 1e-9
                cam_lidar_dt = abs(cam_t - lidar_t)
                if cam_lidar_dt > 0.08:
                    self._sync_warn_count += 1
                    if self._sync_warn_count % 10 == 1:
                        self.get_logger().warn(f'QCGAF sync jitter high: |cam-lidar|={cam_lidar_dt:.3f}s')

            cam_dets, cam_mask, n_cam, cam_ids = markers_to_array(
                cam_msg.markers, self.max_dets, is_camera=True, extra_features=None)
            lidar_dets, lidar_mask, n_lidar, lidar_ids = markers_to_array(
                lidar_msg.markers, self.max_dets, is_camera=False, extra_features=None)
            cam_dets, cam_mask, n_cam, cam_ids = suppress_duplicate_detections(
                cam_dets, cam_mask, cam_ids, self.dedupe_center_distance)
            lidar_dets, lidar_mask, n_lidar, lidar_ids = suppress_duplicate_detections(
                lidar_dets, lidar_mask, lidar_ids, self.dedupe_center_distance)

            sensor_times = [
                ts for ts in (
                    self.latest_color_stamp,
                    self.latest_depth_stamp,
                    self.latest_yolo_stamp,
                    self.latest_lidar_stamp,
                ) if ts is not None
            ]
            ref_t = max(cam_t if cam_msg.markers else 0.0, lidar_t if lidar_msg.markers else 0.0)
            sensor_age = max([max(ref_t - ts, 0.0) for ts in sensor_times], default=0.0)

            quality, camera_extra, lidar_extra = self.quality_est.compute(
                color_image=self.latest_color,
                depth_image=self.latest_depth,
                yolo_conf=self.latest_yolo_conf,
                yolo_count=self.latest_yolo_count,
                lidar_points=max(self.latest_lidar_point_count, 0),
                lidar_stds=self.latest_lidar_stds,
                n_cam=n_cam,
                n_lidar=n_lidar,
                cam_lidar_dt=cam_lidar_dt,
                sensor_age=sensor_age,
            )

            cam_dets[:, 6:] = camera_extra
            lidar_dets[:, 6:] = lidar_extra

            # §3.3 publish the 7-dim quality vector so the detector / tracker
            # can use it for KF Q/R adaptation.  Publishing every QC tick
            # regardless of whether fusion produces boxes — quality is useful
            # to the tracker even when no detections are matched this frame.
            try:
                qmsg = Float32MultiArray()
                qmsg.layout.dim = [MultiArrayDimension(label='quality', size=7, stride=7)]
                qmsg.data = [float(x) for x in quality.tolist()]
                self.quality_pub.publish(qmsg)
            except Exception as exc_q:
                self.get_logger().warn(f'quality publish failed: {exc_q}')

            # Keep recall under camera dropouts by falling back to LiDAR detections.
            if n_cam == 0:
                if self.enable_lidar_fallback and n_lidar > 0:
                    copy_n = min(n_lidar, self.max_dets)
                    cam_dets[:copy_n, :6] = lidar_dets[:copy_n, :6]
                    cam_dets[:copy_n, 6:] = CAMERA_DEFAULT_FEATURES
                    cam_mask[:copy_n] = True
                    cam_ids[:copy_n] = lidar_ids[:copy_n]
                    n_cam = copy_n
                    self._frames_lidar_fallback += 1
                else:
                    self._frames_skipped_no_cam += 1
                    now_s = time.time()
                    if now_s - self._last_empty_cam_warn_s > 5.0:
                        self.get_logger().warn('QCGAF skipped frame: no camera detections')
                        self._last_empty_cam_warn_s = now_s
                    return

            # NEW path (default OFF, gated by enable_lidar_only_promotion):
            # per-object LiDAR promotion.  For each LiDAR detection that has NO
            # matching camera detection (center distance > threshold), append it
            # into a free camera slot so it flows through the normal output path.
            # Mirrors the n_cam==0 fallback (lidar geom + CAMERA_DEFAULT_FEATURES)
            # but per-object.  When the flag is off this block is a no-op and the
            # camera-anchored pipeline is byte-for-byte unchanged.
            if self.enable_lidar_only_promotion and n_lidar > 0 and n_cam < self.max_dets:
                thr = self.lidar_only_match_distance
                n_cam_orig = n_cam            # match only against real camera dets
                slot = n_cam
                for li in range(n_lidar):
                    if slot >= self.max_dets:
                        break
                    lc = lidar_dets[li, :3]
                    matched = False
                    for ci in range(n_cam_orig):
                        if float(np.linalg.norm(lc - cam_dets[ci, :3])) <= thr:
                            matched = True
                            break
                    if matched:
                        continue
                    cam_dets[slot, :6] = lidar_dets[li, :6]
                    cam_dets[slot, 6:] = CAMERA_DEFAULT_FEATURES
                    cam_mask[slot] = True
                    cam_ids[slot] = lidar_ids[li]
                    slot += 1
                promoted = slot - n_cam
                if promoted > 0:
                    self._frames_lidar_promoted += 1
                    self._sum_lidar_promoted += promoted
                    n_cam = slot

            cam_dets_model, lidar_dets_model, frame_center = center_detections(cam_dets, lidar_dets, cam_mask, lidar_mask)

            with torch.no_grad():
                cam_t = torch.tensor(cam_dets_model).unsqueeze(0).to(self.device)
                lidar_t = torch.tensor(lidar_dets_model).unsqueeze(0).to(self.device)
                q_t = torch.tensor(quality).unsqueeze(0).to(self.device)
                cm_t = torch.tensor(cam_mask).unsqueeze(0).to(self.device)
                lm_t = torch.tensor(lidar_mask).unsqueeze(0).to(self.device)
                pred_boxes, _, qcgaf_aux = self.model(cam_t, lidar_t, q_t, cm_t, lm_t, return_aux=True)

            if self._qlog is not None:
                try:
                    a = float(np.asarray(qcgaf_aux['alpha'].detach().cpu()).reshape(-1)[0])
                    ea_arr = np.asarray(qcgaf_aux['eff_alpha'].detach().cpu()).reshape(-1)
                    ea = float(ea_arr[cam_mask].mean()) if bool(cam_mask.any()) else float(ea_arr.mean())
                    tlog = self.get_clock().now().nanoseconds * 1e-9
                    fz = 0 if self.quality_est.freeze_quality is None else 1
                    self._qlog.write('%.3f,%s,%.4f,%.4f,%d,%d\n' % (
                        tlog, ','.join('%.4f' % v for v in quality.tolist()), a, ea, int(n_cam), fz))
                except Exception:
                    pass

            boxes_np = pred_boxes[0].cpu().numpy()
            boxes_np[cam_mask, :3] += frame_center
            # Conservative geometry correction:
            # keep learned output, but blend toward measured camera-side 3D boxes to
            # reduce systematic center/size bias and improve GT-center matching.
            alpha_c = float(np.clip(self.post_center_blend_alpha, 0.0, 1.0))
            alpha_s = float(np.clip(self.post_size_blend_alpha, 0.0, 1.0))
            if alpha_c > 0.0:
                boxes_np[cam_mask, :3] = (1.0 - alpha_c) * boxes_np[cam_mask, :3] + alpha_c * cam_dets[cam_mask, :3]
            if alpha_s > 0.0:
                boxes_np[cam_mask, 3:6] = (1.0 - alpha_s) * boxes_np[cam_mask, 3:6] + alpha_s * cam_dets[cam_mask, 3:6]
            boxes_np[cam_mask, 3:6] = np.clip(
                boxes_np[cam_mask, 3:6],
                self.min_box_size,
                self.max_box_size,
            )
            # UAV partial-view geometry often exposes only the visible upper body.
            # Apply a final head-drop correction on the published fused center so
            # evaluator-facing boxes can recover the human center from the top face.
            alpha_z = float(np.clip(self.post_z_head_drop_alpha, 0.0, 1.0))
            if alpha_z > 0.0:
                head_drop_z = boxes_np[cam_mask, 2] + 0.5 * boxes_np[cam_mask, 4] - 0.88
                boxes_np[cam_mask, 2] = (1.0 - alpha_z) * boxes_np[cam_mask, 2] + alpha_z * head_drop_z
            dt_ms = (time.time() - t0) * 1000.0

            frame_id = 'map'
            if cam_msg.markers:
                frame_id = cam_msg.markers[0].header.frame_id
                stamp = cam_msg.markers[0].header.stamp
            else:
                stamp = self.get_clock().now().to_msg()

            marker_ids = self._assign_track_ids(boxes_np, cam_mask)
            current_tids = {
                int(tid) for tid in marker_ids[:int(np.count_nonzero(cam_mask))]
                if int(tid) >= 0
            }
            out_msg = self._tracks_to_markers(frame_id, stamp, current_tids=current_tids)
            self.pub.publish(out_msg)
            self._frames_total += 1
            self._frames_published += 1
            self._sum_cam_in += int(n_cam)
            self._sum_lidar_in += int(n_lidar)
            self._sum_out += len(out_msg.markers)

            if self.verbose:
                self.get_logger().info(
                    f'QC-GAF: {n_cam} cam + {n_lidar} lidar -> {len(out_msg.markers)} fused ({dt_ms:.2f}ms)'
                )
            if self.debug_metrics:
                now_s = time.time()
                if now_s - self._last_debug_log_s > 5.0:
                    denom = max(1, self._frames_published)
                    self.get_logger().info(
                        f'QCGAF metrics: frames={self._frames_total} '
                        f'skipped_no_cam={self._frames_skipped_no_cam} '
                        f'lidar_fallback={self._frames_lidar_fallback} '
                        f'lidar_promoted_frames={self._frames_lidar_promoted} '
                        f'lidar_promoted_total={self._sum_lidar_promoted} '
                        f'sync_warns={self._sync_warn_count} '
                        f'avg_cam_in={self._sum_cam_in / denom:.2f} '
                        f'avg_lidar_in={self._sum_lidar_in / denom:.2f} '
                        f'avg_out={self._sum_out / denom:.2f}'
                    )
                    self._last_debug_log_s = now_s
        except Exception as exc:
            self.get_logger().error(f'QCGAF callback failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = QCGAFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
