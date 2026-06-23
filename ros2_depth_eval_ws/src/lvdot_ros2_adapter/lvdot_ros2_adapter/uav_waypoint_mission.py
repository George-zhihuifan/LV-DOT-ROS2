"""UAV waypoint mission — fly through the pedestrian crowd in a loop.

Replaces the orbit-based pose_stub with a waypoint-following flight path
that keeps moving targets in the camera FOV at all times.  The UAV yaw
always faces the direction of travel so the forward-mounted D435i sees
oncoming pedestrians.

Usage (standalone):
  ros2 run lvdot_ros2_adapter uav_waypoint_mission --ros-args \
      -p speed:=1.5  -p altitude:=1.2  -p use_sim_time:=true

Usage (via launch):
  ros2 launch lvdot_bringup run_full_pipeline.launch.py pose_mode:=mission
"""

import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy
import yaml

DEFAULT_WAYPOINTS = [
    (2.0, -1.5),
    (5.0, -2.0),
    (8.0, -1.0),
    (12.0, -1.5),
    (13.0,  0.5),
    (12.0,  2.0),
    (8.0,  2.5),
    (5.0,  1.5),
    (2.0,  0.5),
    (1.0, -0.5),
]


class UavWaypointMission(Node):
    def __init__(self) -> None:
        super().__init__('uav_waypoint_mission')
        self.declare_parameter('scenario_config', '')
        self.declare_parameter('speed', 1.5)
        self.declare_parameter('altitude', 1.2)
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('arrival_radius', 0.3)
        self.declare_parameter('gazebo_cmd_topic', '/uav_motion/pose_cmd')

        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/local_position/pose', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/mavros/local_position/odom', 10)
        gz_topic = str(self.get_parameter('gazebo_cmd_topic').value)
        self.gazebo_cmd_pub = self.create_publisher(PoseStamped, gz_topic, 10)

        self.waypoints, scenario_altitude = self._load_scenario_waypoints()
        self.wp_idx = 0
        self.x = float(self.waypoints[0][0])
        self.y = float(self.waypoints[0][1])
        self.yaw = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.t_prev_ns: int | None = None
        if scenario_altitude is not None:
            self.set_parameters([
                rclpy.parameter.Parameter(
                    'altitude', rclpy.Parameter.Type.DOUBLE, float(scenario_altitude))
            ])

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Mission started: {len(self.waypoints)} waypoints, '
            f'speed={self.get_parameter("speed").value} m/s, '
            f'alt={self.get_parameter("altitude").value} m')

    def _load_scenario_waypoints(self) -> tuple[list[tuple[float, float]], float | None]:
        scenario_path = str(self.get_parameter('scenario_config').value).strip()
        if not scenario_path:
            return DEFAULT_WAYPOINTS, None
        path = Path(scenario_path)
        if not path.is_file():
            self.get_logger().warning(
                f'scenario_config not found: {scenario_path}; using default mission waypoints')
            return DEFAULT_WAYPOINTS, None
        try:
            with path.open('r', encoding='utf-8') as handle:
                config = yaml.safe_load(handle) or {}
            uav_cfg = config.get('uav', {}) or {}
            raw_waypoints = uav_cfg.get('waypoints', [])
            waypoints: list[tuple[float, float]] = []
            for item in raw_waypoints:
                if isinstance(item, (list, tuple)) and len(item) >= 2:
                    waypoints.append((float(item[0]), float(item[1])))
            altitude = None
            pose = uav_cfg.get('pose')
            if isinstance(pose, list) and len(pose) >= 3:
                altitude = float(pose[2])
            if len(waypoints) < 2:
                self.get_logger().warning(
                    f'scenario_config {scenario_path} has fewer than 2 UAV waypoints; '
                    'using default mission path')
                return DEFAULT_WAYPOINTS, altitude
            return waypoints, altitude
        except Exception as exc:
            self.get_logger().warning(
                f'failed to parse scenario_config {scenario_path}: {exc}; '
                'using default mission waypoints')
            return DEFAULT_WAYPOINTS, None

    def _tick(self) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self.t_prev_ns is None:
            self.t_prev_ns = now_ns
            self._publish(now)
            return
        dt = (now_ns - self.t_prev_ns) * 1e-9
        self.t_prev_ns = now_ns
        if dt <= 0.0 or dt > 1.0:
            self._publish(now)
            return

        speed = float(self.get_parameter('speed').value)
        arrival_r = float(self.get_parameter('arrival_radius').value)

        tx, ty = self.waypoints[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < arrival_r:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
            tx, ty = self.waypoints[self.wp_idx]
            dx = tx - self.x
            dy = ty - self.y
            dist = math.sqrt(dx * dx + dy * dy)
            self.get_logger().info(
                f'Waypoint {self.wp_idx}/{len(self.waypoints)}: '
                f'({tx:.1f}, {ty:.1f})')

        if dist > 1e-6:
            nx, ny = dx / dist, dy / dist
            step = min(speed * dt, dist)
            self.x += nx * step
            self.y += ny * step
            self.vx = nx * speed
            self.vy = ny * speed
            self.yaw = math.atan2(ny, nx)

        self._publish(now)

    def _publish(self, now) -> None:
        stamp = now.to_msg()
        alt = float(self.get_parameter('altitude').value)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = alt
        half = self.yaw * 0.5
        pose.pose.orientation.z = math.sin(half)
        pose.pose.orientation.w = math.cos(half)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy

        self.pose_pub.publish(pose)
        self.odom_pub.publish(odom)
        self.gazebo_cmd_pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = UavWaypointMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
