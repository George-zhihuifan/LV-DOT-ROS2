"""LV-DOT pose stub with optional circular orbit motion.

If `orbit_enabled` is true, publishes a UAV pose flying a slow circle so
that the detector sees relative motion (required for dynamic classification).

When `publish_gazebo_cmd` is true (default), also republishes the same
PoseStamped to `/uav_motion/pose_cmd` for the UavPoseSyncSystem Gazebo
plugin, which physically moves the `<static>true</static>` UAV in the
simulation via SetWorldPoseCmd. Without this, only the ROS-side pose
stream moves while the Gazebo UAV (and its cameras/lidar) sits still —
sensor data does not reflect the orbit, breaking the moving-observer
premise of dynamic detection.
"""

import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy
import yaml


class PoseStub(Node):
    def __init__(self) -> None:
        super().__init__('lvdot_pose_stub')
        self.declare_parameter('scenario_config', '')
        scenario_pose = self._load_scenario_uav_pose()
        default_x = scenario_pose[0]
        default_y = scenario_pose[1]
        default_z = scenario_pose[2]
        default_yaw = scenario_pose[5]
        # Orbit parameters. Defaults match the UAV's actual <static>true</static>
        # spawn pose from the scenario config when available.
        # Orbit is OFF by default so TF stays aligned with the Gazebo entity;
        # set orbit_enabled:=true to also move the Gazebo UAV via the
        # UavPoseSyncSystem plugin listening on /uav_motion/pose_cmd.
        self.declare_parameter('orbit_enabled', False)
        self.declare_parameter('orbit_center_x', default_x)
        self.declare_parameter('orbit_center_y', default_y)
        self.declare_parameter('orbit_radius', 0.4)
        self.declare_parameter('orbit_speed_rad_s', 0.3)
        self.declare_parameter('orbit_z', default_z)
        self.declare_parameter('static_x', default_x)
        self.declare_parameter('static_y', default_y)
        self.declare_parameter('static_z', default_z)
        self.declare_parameter('static_yaw', default_yaw)
        self.declare_parameter('publish_rate_hz', 30.0)
        # Whether to also drive the Gazebo UAV via /uav_motion/pose_cmd.
        # Default true: a moving ROS pose without a moving Gazebo entity is
        # almost always a bug — sensor data wouldn't reflect the motion.
        self.declare_parameter('publish_gazebo_cmd', True)
        self.declare_parameter('gazebo_cmd_topic', '/uav_motion/pose_cmd')
        # When orbit_enabled is False we don't republish the static pose to
        # /uav_motion/pose_cmd by default — the world's static spawn already
        # places the UAV there, and republishing every frame causes the plugin
        # to call SetWorldPoseCmd at 30Hz needlessly.
        self.declare_parameter('publish_gazebo_cmd_when_static', False)

        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/mavros/local_position/odom', 10)
        self.publish_gazebo_cmd = bool(self.get_parameter('publish_gazebo_cmd').value)
        self.publish_gazebo_cmd_when_static = bool(
            self.get_parameter('publish_gazebo_cmd_when_static').value)
        gazebo_cmd_topic = str(self.get_parameter('gazebo_cmd_topic').value)
        self.gazebo_cmd_pub = self.create_publisher(PoseStamped, gazebo_cmd_topic, 10) \
            if self.publish_gazebo_cmd else None

        self.t_start_ns = None
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.publish_messages)

    def _load_scenario_uav_pose(self) -> list[float]:
        scenario_path = str(self.get_parameter('scenario_config').value).strip()
        default_pose = [1.8, -1.6, 1.0, 0.0, 0.0, 0.18]
        if not scenario_path:
            return default_pose
        path = Path(scenario_path)
        if not path.is_file():
            self.get_logger().warning(
                f'scenario_config not found: {scenario_path}; using built-in pose_stub defaults')
            return default_pose
        try:
            with path.open('r', encoding='utf-8') as handle:
                config = yaml.safe_load(handle) or {}
            pose = config.get('uav', {}).get('pose', default_pose)
            if not isinstance(pose, list) or len(pose) < 6:
                return default_pose
            return [float(v) for v in pose[:6]]
        except Exception as exc:
            self.get_logger().warning(
                f'failed to parse scenario_config {scenario_path}: {exc}; using built-in pose_stub defaults')
            return default_pose

    def publish_messages(self) -> None:
        now = self.get_clock().now()
        stamp = now.to_msg()
        if self.t_start_ns is None:
            self.t_start_ns = now.nanoseconds
        elapsed_s = (now.nanoseconds - self.t_start_ns) * 1e-9

        if bool(self.get_parameter('orbit_enabled').value):
            cx = float(self.get_parameter('orbit_center_x').value)
            cy = float(self.get_parameter('orbit_center_y').value)
            r = float(self.get_parameter('orbit_radius').value)
            omega = float(self.get_parameter('orbit_speed_rad_s').value)
            z = float(self.get_parameter('orbit_z').value)
            theta = omega * elapsed_s
            x = cx + r * math.cos(theta)
            y = cy + r * math.sin(theta)
            yaw = theta + math.pi / 2.0
            vx = -r * omega * math.sin(theta)
            vy = r * omega * math.cos(theta)
            yaw_rate = omega
        else:
            x = float(self.get_parameter('static_x').value)
            y = float(self.get_parameter('static_y').value)
            z = float(self.get_parameter('static_z').value)
            yaw = float(self.get_parameter('static_yaw').value)
            vx, vy, yaw_rate = 0.0, 0.0, 0.0

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        half = yaw * 0.5
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = float(vx)
        odom.twist.twist.linear.y = float(vy)
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = float(yaw_rate)

        self.pose_pub.publish(pose)
        self.odom_pub.publish(odom)

        if self.gazebo_cmd_pub is not None and (
            bool(self.get_parameter('orbit_enabled').value)
            or self.publish_gazebo_cmd_when_static
        ):
            self.gazebo_cmd_pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = PoseStub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
