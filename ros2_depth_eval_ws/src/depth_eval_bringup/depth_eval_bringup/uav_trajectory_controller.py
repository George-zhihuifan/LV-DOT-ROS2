import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def length_3d(dx: float, dy: float, dz: float) -> float:
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class UavTrajectoryController(Node):
    def __init__(self) -> None:
        super().__init__('uav_trajectory_controller')
        default_config = (
            Path.home() / 'ros2_depth_eval_ws' / 'src' / 'depth_eval_bringup' / 'config' / 'pedestrian_prototype.yaml'
        )
        self.declare_parameter('config_path', str(default_config))
        self.declare_parameter('world_name', 'pedestrian_prototype')
        self.declare_parameter('pose_service_name', '')
        self.declare_parameter('pose_fallback_service_name', '')
        self.declare_parameter('pose_request_timeout_sec', 0.5)
        config_path = Path(self.get_parameter('config_path').value)
        self.world_name = self.get_parameter('world_name').value
        configured_service_name = str(self.get_parameter('pose_service_name').value)
        configured_fallback_service_name = str(self.get_parameter('pose_fallback_service_name').value)
        self.pose_service_name = configured_service_name or f'/world/{self.world_name}/set_pose/blocking'
        self.pose_fallback_service_name = configured_fallback_service_name or f'/world/{self.world_name}/set_pose'
        self.pose_request_timeout_sec = max(0.05, float(self.get_parameter('pose_request_timeout_sec').value))
        with config_path.open('r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle)
        self.uav = config.get('uav', {})
        self.model_name = self.uav.get('model_name', 'uav_main')
        self.waypoints = [tuple(point) for point in self.uav.get('waypoints', [])]
        self.speed_mps = float(self.uav.get('speed_mps', 1.2))
        self.loop = bool(self.uav.get('loop', True))
        self.weave_amplitude = float(self.uav.get('weave_amplitude', 0.0))
        self.weave_frequency = float(self.uav.get('weave_frequency', 0.0))
        self.vibration_amplitude = float(self.uav.get('vibration_amplitude', 0.0))
        self.update_hz = float(self.uav.get('update_hz', 10.0))
        self.position = tuple(self.uav.get('pose', [0.0, 0.0, 2.5, 0.0, 0.0, 0.0])[:3])
        self.prev_position = self.position
        self.segment_index = 0
        self.segment_progress = 0.0
        self.sim_time = 0.0
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/mavros/local_position/odom', 10)
        self.pose_cli = self.create_client(SetEntityPose, self.pose_service_name)
        self.pose_fallback_cli = self.create_client(SetEntityPose, self.pose_fallback_service_name)
        self.use_ros_service = False
        self.warned_service_unavailable = False
        self.warned_pose_failure = False
        self.pose_future = None
        self.pending_pose = None
        self.pending_velocity = None
        self.pose_request_sent_ns = None
        self.active_service_name = self.pose_service_name
        self.active_pose_cli = self.pose_cli
        self.timer = self.create_timer(1.0 / max(self.update_hz, 1.0), self.on_timer)

        if len(self.waypoints) < 2:
            self.get_logger().warn('UAV config has fewer than 2 waypoints; UAV will remain at its initial pose.')

    def current_target(self) -> tuple[float, float, float] | None:
        if len(self.waypoints) < 2:
            return None
        return self.waypoints[(self.segment_index + 1) % len(self.waypoints)]

    def step_path(self, dt: float) -> tuple[float, float, float, float]:
        if len(self.waypoints) < 2:
            x, y, z = self.position
            return x, y, z, 0.0

        start = self.waypoints[self.segment_index]
        target_index = (self.segment_index + 1) % len(self.waypoints)
        target = self.waypoints[target_index]
        dx = target[0] - start[0]
        dy = target[1] - start[1]
        dz = target[2] - start[2]
        segment_len = max(length_3d(dx, dy, dz), 1e-6)
        self.segment_progress += self.speed_mps * dt / segment_len
        while self.segment_progress >= 1.0:
            self.segment_progress -= 1.0
            self.segment_index = target_index
            target_index = (self.segment_index + 1) % len(self.waypoints)
            if not self.loop and target_index == 0:
                self.segment_progress = 1.0
                break
            start = self.waypoints[self.segment_index]
            target = self.waypoints[target_index]
            dx = target[0] - start[0]
            dy = target[1] - start[1]
            dz = target[2] - start[2]
            segment_len = max(length_3d(dx, dy, dz), 1e-6)
        alpha = min(max(self.segment_progress, 0.0), 1.0)
        x = start[0] + dx * alpha
        y = start[1] + dy * alpha
        z = start[2] + dz * alpha
        yaw = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-6 else 0.0

        if self.weave_amplitude > 0.0 and abs(dx) + abs(dy) > 1e-6:
            norm = math.hypot(dx, dy)
            nx = -dy / norm
            ny = dx / norm
            weave = math.sin(self.sim_time * 2.0 * math.pi * self.weave_frequency) * self.weave_amplitude
            x += nx * weave
            y += ny * weave
        if self.vibration_amplitude > 0.0:
            z += math.sin(self.sim_time * 7.0) * self.vibration_amplitude
        return x, y, z, yaw

    def try_ros_pose(self, pose: Pose, linear_velocity: tuple[float, float, float]) -> bool:
        if self.pose_future is not None and not self.pose_future.done():
            return True
        if not self.use_ros_service:
            self.use_ros_service = self.active_pose_cli.wait_for_service(timeout_sec=0.01)
        if not self.use_ros_service:
            return False
        request = SetEntityPose.Request()
        request.entity = Entity(name=self.model_name, type=Entity.MODEL)
        request.pose = pose
        self.pending_pose = pose
        self.pending_velocity = linear_velocity
        self.pose_future = self.active_pose_cli.call_async(request)
        self.pose_request_sent_ns = self.get_clock().now().nanoseconds
        self.pose_future.add_done_callback(self.on_pose_result)
        return True

    def on_pose_result(self, future) -> None:
        self.pose_future = None
        self.pose_request_sent_ns = None
        if future.cancelled():
            return
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - transport/runtime failure
            self.use_ros_service = False
            self.pending_pose = None
            self.pending_velocity = None
            if not self.warned_pose_failure:
                self.warned_pose_failure = True
                self.get_logger().warning(f'SetEntityPose call failed: {exc}')
            return

        if response is None:
            return
        if not response.success:
            self.use_ros_service = False
            self.pending_pose = None
            self.pending_velocity = None
            if not self.warned_pose_failure:
                self.warned_pose_failure = True
                self.get_logger().warning('SetEntityPose returned success=false.')
            return

        pose = self.pending_pose
        velocity = self.pending_velocity
        self.pending_pose = None
        self.pending_velocity = None
        if pose is not None and velocity is not None:
            self.publish_pose_topics(pose, velocity)

    def publish_pose_topics(self, pose: Pose, linear_velocity: tuple[float, float, float]) -> None:
        now = self.get_clock().now().to_msg()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = pose

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'uav_main/base_link'
        odom.pose.pose = pose
        odom.twist.twist.linear.x = linear_velocity[0]
        odom.twist.twist.linear.y = linear_velocity[1]
        odom.twist.twist.linear.z = linear_velocity[2]

        self.pose_pub.publish(pose_msg)
        self.odom_pub.publish(odom)

    def on_timer(self) -> None:
        if self.pose_future is not None and not self.pose_future.done():
            if self.pose_request_sent_ns is not None:
                elapsed_sec = (self.get_clock().now().nanoseconds - self.pose_request_sent_ns) / 1e9
                if elapsed_sec > self.pose_request_timeout_sec:
                    self.get_logger().warning(
                        f'SetEntityPose request timed out after {elapsed_sec:.2f}s on {self.active_service_name}; '
                        'dropping the pending request and retrying.'
                    )
                    self.pose_future.cancel()
                    self.pose_future = None
                    self.pose_request_sent_ns = None
                    self.pending_pose = None
                    self.pending_velocity = None
                    self.use_ros_service = False
                    if self.active_service_name == self.pose_service_name:
                        self.active_service_name = self.pose_fallback_service_name
                        self.active_pose_cli = self.pose_fallback_cli
                        self.get_logger().warning(
                            f'Falling back from {self.pose_service_name} to {self.pose_fallback_service_name}.'
                        )
            return
        dt = 1.0 / max(self.update_hz, 1.0)
        self.sim_time += dt
        x, y, z, yaw = self.step_path(dt)
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        velocity = (
            (x - self.prev_position[0]) / dt,
            (y - self.prev_position[1]) / dt,
            (z - self.prev_position[2]) / dt,
        )
        self.prev_position = (x, y, z)
        self.position = (x, y, z)

        if not self.try_ros_pose(pose, velocity):
            if not self.warned_service_unavailable:
                self.warned_service_unavailable = True
                self.get_logger().warning(
                    f'SetEntityPose service {self.active_service_name} is unavailable; '
                    'UAV state publication is paused until Gazebo pose control is reachable.'
                )


def main() -> None:
    rclpy.init()
    node = UavTrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
