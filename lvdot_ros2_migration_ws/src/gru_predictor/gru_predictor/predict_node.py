#!/usr/bin/env python3
"""ROS2 node for real-time 3D hybrid (KF+GRU) motion prediction."""

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import yaml

from gru_predictor.src.hybrid_predictor import HybridPredictor


class PredictionNode(Node):
    """ROS2 node that runs hybrid prediction on tracked obstacles."""

    def __init__(self):
        super().__init__('gru_prediction_node')

        self.declare_parameter('config', '')
        self.declare_parameter('model', '')
        self.declare_parameter('input_topic', '/onboard_detector/dynamic_bboxes')
        self.declare_parameter('output_topic', '/gru_predictor/predicted_positions')
        self.declare_parameter('horizon', 5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('max_idle', 3.0)

        config_path = self.get_parameter('config').get_parameter_value().string_value
        model_path = self.get_parameter('model').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.predict_horizon = int(self.get_parameter('horizon').value)
        self.device = self.get_parameter('device').get_parameter_value().string_value
        self.max_idle = float(self.get_parameter('max_idle').value)

        if not config_path:
            raise RuntimeError('Parameter "config" must be provided.')
        if not model_path:
            raise RuntimeError('Parameter "model" must be provided.')
        if not os.path.exists(config_path):
            raise RuntimeError(f'Config file not found: {config_path}')
        if not os.path.exists(model_path):
            raise RuntimeError(f'Model file not found: {model_path}')
        if self.predict_horizon < 1:
            raise RuntimeError(f'Invalid horizon={self.predict_horizon}, must be >= 1')
        if self.max_idle <= 0.0:
            raise RuntimeError(f'Invalid max_idle={self.max_idle}, must be > 0')
        if self.device not in ('cpu', 'cuda'):
            raise RuntimeError(f'Invalid device={self.device}, must be cpu/cuda')

        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)

        self.dt = float(self.config['kalman']['dt'])
        self.model_path = model_path

        self.predictors = {}
        self.last_seen = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(MarkerArray, input_topic, self.tracking_callback, qos)
        self.pub = self.create_publisher(MarkerArray, output_topic, qos)

        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing to:  {output_topic}')
        self.get_logger().info(f'Model: {model_path}')
        self.get_logger().info(f'Predict horizon: {self.predict_horizon} steps ({self.predict_horizon * self.dt:.2f}s)')

    def _get_predictor(self, track_id):
        if track_id not in self.predictors:
            self.predictors[track_id] = HybridPredictor(
                self.model_path, self.config, device=self.device
            )
        return self.predictors[track_id]

    def _cleanup_stale_tracks(self, current_time):
        stale_ids = [tid for tid, t in self.last_seen.items() if current_time - t > self.max_idle]
        for tid in stale_ids:
            del self.predictors[tid]
            del self.last_seen[tid]
        if stale_ids:
            self.get_logger().debug(f'Cleaned stale tracks: {stale_ids}')

    def tracking_callback(self, msg: MarkerArray):
        if not msg.markers:
            return

        current_time = time.time()
        pred_markers = MarkerArray()
        stamp = msg.markers[0].header.stamp
        frame_id = msg.markers[0].header.frame_id or 'map'

        for marker in msg.markers:
            track_id = marker.id
            x = marker.pose.position.x
            y = marker.pose.position.y
            z = marker.pose.position.z
            w = marker.scale.x
            h = marker.scale.y
            l = marker.scale.z

            self.last_seen[track_id] = current_time
            try:
                predictor = self._get_predictor(track_id)
                result = predictor.predict(x, y, z, w=w, h=h, l=l)
            except Exception as exc:
                self.get_logger().error(f'Predict failed for track {track_id}: {exc}')
                continue

            hybrid_pred = result['hybrid_pred']
            gamma = float(result['gamma'])

            current_pos = hybrid_pred.copy()
            current_pos[0] = x
            current_pos[1] = y
            current_pos[2] = z
            vel = hybrid_pred - current_pos
            future_positions = [hybrid_pred.copy()]
            for _ in range(1, self.predict_horizon):
                next_pos = future_positions[-1] + vel
                future_positions.append(next_pos.copy())

            pred_marker = Marker()
            pred_marker.header.stamp = stamp
            pred_marker.header.frame_id = frame_id
            pred_marker.ns = 'gru_predictions'
            pred_marker.id = track_id
            pred_marker.type = Marker.LINE_STRIP
            pred_marker.action = Marker.ADD
            pred_marker.scale.x = 0.05
            pred_marker.color = ColorRGBA(r=0.0, g=1.0 - gamma, b=gamma, a=0.8)
            pred_marker.lifetime = Duration(seconds=0.2).to_msg()
            pred_marker.points.append(Point(x=x, y=y, z=z))

            for fp in future_positions:
                pred_marker.points.append(Point(x=float(fp[0]), y=float(fp[1]), z=float(fp[2])))

            pred_markers.markers.append(pred_marker)

            end_marker = Marker()
            end_marker.header.stamp = stamp
            end_marker.header.frame_id = frame_id
            end_marker.ns = 'gru_pred_endpoints'
            end_marker.id = track_id
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose.position.x = float(future_positions[-1][0])
            end_marker.pose.position.y = float(future_positions[-1][1])
            end_marker.pose.position.z = float(future_positions[-1][2])
            end_marker.scale.x = 0.15
            end_marker.scale.y = 0.15
            end_marker.scale.z = 0.15
            end_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.9)
            end_marker.lifetime = Duration(seconds=0.2).to_msg()
            pred_markers.markers.append(end_marker)

        self.pub.publish(pred_markers)
        self._cleanup_stale_tracks(current_time)


def main(args=None):
    rclpy.init(args=args)
    node = PredictionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
