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

    def update_imu(self, imu_msg: Imu):
        acc = np.array([
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
        ])
        self.imu_buffer.append(acc)

    def update_depth(self, depth_array: np.ndarray):
        self.depth_prev = depth_array.copy()

    def compute(self, color_image=None, depth_image=None, yolo_conf=0.8, lidar_points=100, lidar_stds=None):
        brightness, edge = color_metrics(color_image)
        depth_valid_ratio, depth_var, depth_temporal = depth_metrics(depth_image, self.depth_prev)

        q = np.array([
            brightness,
            edge,
            depth_valid_ratio,
            float(np.clip(yolo_conf, 0.0, 1.0)),
            float(np.clip(lidar_points / 24000.0, 0.0, 1.0)),
            0.8,
            depth_temporal,
        ], dtype=np.float32)

        if len(self.imu_buffer) > 10:
            acc_arr = np.array(self.imu_buffer)
            vib = acc_arr.var(axis=0).sum()
            q[5] = float(np.clip(1.0 / (1.0 + vib / 400.0), 0.0, 1.0))

        if depth_image is not None:
            self.update_depth(depth_image)

        camera_extra = np.array([depth_valid_ratio, depth_var, depth_valid_ratio], dtype=np.float32)
        if lidar_stds is None:
            lidar_stds = np.array([0.1, 0.1, 0.08], dtype=np.float32)
        lidar_extra = np.concatenate([
            np.array([float(np.clip(lidar_points / 24000.0, 0.0, 1.0))], dtype=np.float32),
            lidar_stds.astype(np.float32),
        ])
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
    n = min(len(markers), max_dets)
    for i in range(n):
        m = markers[i]
        x = m.pose.position.x
        y = m.pose.position.y
        z = m.pose.position.z
        w = m.scale.x
        h = m.scale.z
        l = m.scale.y
        dets[i, :6] = [x, y, z, w, h, l]
        dets[i, 6:] = (CAMERA_DEFAULT_FEATURES if is_camera else LIDAR_DEFAULT_FEATURES) if extra_features is None else extra_features
        mask[i] = True
    return dets, mask, n


def array_to_markers(boxes, cam_mask, frame_id, stamp):
    ma = MarkerArray()
    for i in range(len(boxes)):
        if not cam_mask[i]:
            break
        conf = 1.0 / (1.0 + np.exp(-boxes[i, 6]))
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = 'qcgaf_fused'
        m.id = i
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

        config_path = self.get_parameter('config').get_parameter_value().string_value
        ckpt_path = self.get_parameter('checkpoint').get_parameter_value().string_value
        verbose = bool(self.get_parameter('verbose').value)
        self.debug_metrics = bool(self.get_parameter('debug_metrics').value)
        self.enable_lidar_fallback = bool(self.get_parameter('enable_lidar_fallback').value)

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
        self.latest_color = None
        self.latest_depth = None
        self.latest_yolo_conf = 0.0
        self.latest_lidar_point_count = 0
        self.latest_lidar_stds = np.array([0.1, 0.1, 0.08], dtype=np.float32)

        marker_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(MarkerArray, ros_cfg['output_topic'], marker_qos)

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
        self._last_debug_log_s = time.time()

    def color_callback(self, msg: Image):
        try:
            image = image_msg_to_numpy(msg)
            if image is not None:
                self.latest_color = image
        except Exception as exc:
            self.get_logger().warn(f'color_callback failed: {exc}')

    def depth_callback(self, msg: Image):
        try:
            image = image_msg_to_numpy(msg)
            if image is not None:
                self.latest_depth = image
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
        except Exception as exc:
            self.get_logger().warn(f'yolo_callback failed: {exc}')

    def lidar_cloud_callback(self, msg: PointCloud2):
        try:
            self.latest_lidar_point_count, self.latest_lidar_stds = pointcloud_metrics(msg)
        except Exception as exc:
            self.get_logger().warn(f'lidar_cloud_callback failed: {exc}')

    def callback(self, cam_msg: MarkerArray, lidar_msg: MarkerArray):
        t0 = time.time()
        try:
            if cam_msg.markers and lidar_msg.markers:
                cam_t = cam_msg.markers[0].header.stamp.sec + cam_msg.markers[0].header.stamp.nanosec * 1e-9
                lidar_t = lidar_msg.markers[0].header.stamp.sec + lidar_msg.markers[0].header.stamp.nanosec * 1e-9
                dt = abs(cam_t - lidar_t)
                if dt > 0.08:
                    self._sync_warn_count += 1
                    if self._sync_warn_count % 10 == 1:
                        self.get_logger().warn(f'QCGAF sync jitter high: |cam-lidar|={dt:.3f}s')

            quality, camera_extra, lidar_extra = self.quality_est.compute(
                color_image=self.latest_color,
                depth_image=self.latest_depth,
                yolo_conf=self.latest_yolo_conf,
                lidar_points=max(self.latest_lidar_point_count, 0),
                lidar_stds=self.latest_lidar_stds,
            )

            cam_dets, cam_mask, n_cam = markers_to_array(cam_msg.markers, self.max_dets, is_camera=True, extra_features=camera_extra)
            lidar_dets, lidar_mask, n_lidar = markers_to_array(lidar_msg.markers, self.max_dets, is_camera=False, extra_features=lidar_extra)

            # Keep recall under camera dropouts by falling back to LiDAR detections.
            if n_cam == 0:
                if self.enable_lidar_fallback and n_lidar > 0:
                    copy_n = min(n_lidar, self.max_dets)
                    cam_dets[:copy_n, :6] = lidar_dets[:copy_n, :6]
                    cam_dets[:copy_n, 6:] = CAMERA_DEFAULT_FEATURES
                    cam_mask[:copy_n] = True
                    n_cam = copy_n
                else:
                    self._frames_skipped_no_cam += 1
                    now_s = time.time()
                    if now_s - self._last_empty_cam_warn_s > 5.0:
                        self.get_logger().warn('QCGAF skipped frame: no camera detections')
                        self._last_empty_cam_warn_s = now_s
                    return

            cam_dets_model, lidar_dets_model, frame_center = center_detections(cam_dets, lidar_dets, cam_mask, lidar_mask)

            with torch.no_grad():
                cam_t = torch.tensor(cam_dets_model).unsqueeze(0).to(self.device)
                lidar_t = torch.tensor(lidar_dets_model).unsqueeze(0).to(self.device)
                q_t = torch.tensor(quality).unsqueeze(0).to(self.device)
                cm_t = torch.tensor(cam_mask).unsqueeze(0).to(self.device)
                lm_t = torch.tensor(lidar_mask).unsqueeze(0).to(self.device)
                pred_boxes, _ = self.model(cam_t, lidar_t, q_t, cm_t, lm_t)

            boxes_np = pred_boxes[0].cpu().numpy()
            boxes_np[cam_mask, :3] += frame_center
            dt_ms = (time.time() - t0) * 1000.0

            frame_id = 'map'
            if cam_msg.markers:
                frame_id = cam_msg.markers[0].header.frame_id
                stamp = cam_msg.markers[0].header.stamp
            else:
                stamp = self.get_clock().now().to_msg()

            out_msg = array_to_markers(boxes_np, cam_mask, frame_id, stamp)
            self.pub.publish(out_msg)
            self._frames_total += 1

            if self.verbose:
                self.get_logger().info(
                    f'QC-GAF: {n_cam} cam + {n_lidar} lidar -> {len(out_msg.markers)} fused ({dt_ms:.2f}ms)'
                )
            if self.debug_metrics:
                now_s = time.time()
                if now_s - self._last_debug_log_s > 5.0:
                    self.get_logger().info(
                        f'QCGAF metrics: frames={self._frames_total} '
                        f'skipped_no_cam={self._frames_skipped_no_cam} '
                        f'sync_warns={self._sync_warn_count}'
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
