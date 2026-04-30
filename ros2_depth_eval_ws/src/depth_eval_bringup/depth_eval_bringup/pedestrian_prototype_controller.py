import math
import subprocess
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from depth_eval_bringup.pedestrian_scene import load_scene
from depth_eval_bringup.pedestrian_sim_core import PedestrianSimulator


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def color_for_agent(agent) -> tuple[float, float, float]:
    palette = {
        'commuter': (0.18, 0.74, 0.96),
        'shopper': (0.98, 0.70, 0.22),
        'staff': (0.42, 0.88, 0.42),
        'visitor': (0.95, 0.42, 0.55),
        'default': (0.82, 0.82, 0.82),
    }
    return palette.get(agent.profile, palette['default'])


class PedestrianPrototypeController(Node):
    def __init__(self) -> None:
        super().__init__('pedestrian_prototype_controller')
        default_config = (
            Path.home() / 'ros2_depth_eval_ws' / 'src' / 'depth_eval_bringup' / 'config' / 'pedestrian_prototype.yaml'
        )

        self.declare_parameter('config_path', str(default_config))
        self.declare_parameter('world_name', 'pedestrian_prototype')

        config_path = Path(self.get_parameter('config_path').value)
        self.world_name = self.get_parameter('world_name').value

        with config_path.open('r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle)
        self.scene = load_scene(config)
        self.simulator = PedestrianSimulator(self.scene)

        self.pose_pub = self.create_publisher(PoseArray, '/pedestrian_sim/agent_poses', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pedestrian_sim/agent_markers', 10)

        self.warned_cli_failure = False
        self.start_timer = self.create_timer(0.25, self.try_start)
        self.step_timer = None

    def try_start(self) -> None:
        if self.step_timer is not None:
            return

        snapshots = self.simulator.snapshots()
        self.publish_snapshot(snapshots)
        self.step_timer = self.create_timer(self.scene.world.sim_dt, self.on_timer)
        self.get_logger().info('Attached runtime pedestrian controller to preloaded world models.')

    def on_timer(self) -> None:
        snapshots = self.simulator.step()
        self.publish_snapshot(snapshots)

    def push_pose_vector(self, snapshots) -> None:
        # Disabled: pose updates now go through pedestrian_state_publisher →
        # pedestrian_pose_sync_system Gazebo plugin (single fast path).
        # The previous subprocess `gz service set_pose_vector` route timed out
        # at ~0.6s/call and produced visible teleportation.
        return

    def publish_snapshot(self, snapshots) -> None:
        pose_array = PoseArray()
        pose_array.header.frame_id = 'world'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        now = pose_array.header.stamp

        for index, snapshot in enumerate(snapshots):
            agent = self.scene.agents[index]
            pose = Pose()
            pose.position.x = snapshot.x
            pose.position.y = snapshot.y
            pose.position.z = snapshot.z
            qx, qy, qz, qw = yaw_to_quat(snapshot.yaw)
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            pose_array.poses.append(pose)

            body = Marker()
            body.header.frame_id = 'world'
            body.header.stamp = now
            body.ns = 'pedestrian_agents'
            body.id = index
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose = pose
            body.pose.position.z = snapshot.z + 0.55
            body.scale.x = 0.38
            body.scale.y = 0.38
            body.scale.z = 1.7
            red, green, blue = color_for_agent(agent)
            body.color.r = red
            body.color.g = green
            body.color.b = blue
            body.color.a = 0.55
            marker_array.markers.append(body)

            label = Marker()
            label.header.frame_id = 'world'
            label.header.stamp = now
            label.ns = 'pedestrian_names'
            label.id = 500 + index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = snapshot.x
            label.pose.position.y = snapshot.y
            label.pose.position.z = snapshot.z + 1.2
            label.scale.z = 0.25
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = f'{snapshot.name} t{agent.agent_type} g{agent.group_id} {agent.profile}'
            marker_array.markers.append(label)

        obstacle_offset = 1000
        for index, obstacle in enumerate(self.scene.obstacles):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = now
            marker.ns = 'pedestrian_obstacles'
            marker.id = obstacle_offset + index
            marker.action = Marker.ADD
            if obstacle.shape == 'segment':
                marker.type = Marker.CUBE
                marker.scale.x = math.hypot(obstacle.x2 - obstacle.x1, obstacle.y2 - obstacle.y1)
                marker.scale.y = max(0.08, obstacle.width if obstacle.width > 0.0 else 0.12)
                marker.scale.z = obstacle.length
                marker.pose.position.x = (obstacle.x1 + obstacle.x2) * 0.5
                marker.pose.position.y = (obstacle.y1 + obstacle.y2) * 0.5
                marker.pose.position.z = obstacle.length * 0.5
                qx, qy, qz, qw = yaw_to_quat(math.atan2(obstacle.y2 - obstacle.y1, obstacle.x2 - obstacle.x1))
                marker.pose.orientation.x = qx
                marker.pose.orientation.y = qy
                marker.pose.orientation.z = qz
                marker.pose.orientation.w = qw
            elif obstacle.shape == 'box':
                marker.type = Marker.CUBE
                marker.scale.x = obstacle.width
                marker.scale.y = obstacle.depth
                marker.scale.z = obstacle.length
                marker.pose.position.x = obstacle.x
                marker.pose.position.y = obstacle.y
                marker.pose.position.z = obstacle.length * 0.5
            else:
                marker.type = Marker.CYLINDER
                marker.scale.x = obstacle.radius * 2.0
                marker.scale.y = obstacle.radius * 2.0
                marker.scale.z = obstacle.length
                marker.pose.position.x = obstacle.x
                marker.pose.position.y = obstacle.y
                marker.pose.position.z = obstacle.length * 0.5
            marker.color.r = obstacle.color[0]
            marker.color.g = obstacle.color[1]
            marker.color.b = obstacle.color[2]
            marker.color.a = 0.75
            marker_array.markers.append(marker)

        waypoint_offset = 2000
        for index, waypoint in enumerate(self.scene.waypoints.values()):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = now
            marker.ns = 'pedestrian_waypoints'
            marker.id = waypoint_offset + index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.x
            marker.pose.position.y = waypoint.y
            marker.pose.position.z = 0.03
            marker.scale.x = waypoint.radius * 2.0
            marker.scale.y = waypoint.radius * 2.0
            marker.scale.z = 0.06
            marker.color.r = 0.25
            marker.color.g = 0.9
            marker.color.b = 0.35
            marker.color.a = 0.35
            marker_array.markers.append(marker)

        self.pose_pub.publish(pose_array)
        self.marker_pub.publish(marker_array)


def main() -> None:
    rclpy.init()
    node = PedestrianPrototypeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
