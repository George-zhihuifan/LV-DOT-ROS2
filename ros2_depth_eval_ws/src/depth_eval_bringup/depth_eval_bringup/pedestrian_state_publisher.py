from pathlib import Path
import math

import rclpy
import yaml
from depth_eval_msgs.msg import AgentPose, AgentPoseArray
from geometry_msgs.msg import Pose
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from depth_eval_bringup.pedestrian_scene import load_scene
from depth_eval_bringup.pedestrian_sim_core import PedestrianSimulator


def yaw_to_quat(yaw: float):
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


class PedestrianStatePublisher(Node):
    def __init__(self) -> None:
        super().__init__('pedestrian_state_publisher')
        default_config = (
            Path.home() / 'ros2_depth_eval_ws' / 'src' / 'depth_eval_bringup' / 'config' / 'pedestrian_prototype.yaml'
        )
        self.declare_parameter('config_path', str(default_config))
        config_path = Path(self.get_parameter('config_path').value)
        with config_path.open('r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle)
        scene = load_scene(config)
        self.scene = scene
        self.simulator = PedestrianSimulator(scene)
        self.pose_pub = self.create_publisher(AgentPoseArray, '/pedestrian_sim/agent_states', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/pedestrian_sim/agent_markers', 10)
        self.timer = self.create_timer(scene.world.sim_dt, self.on_timer)
        self.publish_snapshot(self.simulator.snapshots())

    def on_timer(self) -> None:
        snapshots = self.simulator.step()
        self.publish_snapshot(snapshots)

    def publish_snapshot(self, snapshots) -> None:
        named_pose_array = AgentPoseArray()
        named_pose_array.header.frame_id = 'world'
        named_pose_array.header.stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()
        now = named_pose_array.header.stamp
        for index, snapshot in enumerate(snapshots):
            agent = self.scene.agents[index]
            qx, qy, qz, qw = yaw_to_quat(snapshot.yaw)

            sync_pose = Pose()
            sync_pose.position.x = snapshot.x
            sync_pose.position.y = snapshot.y
            sync_pose.position.z = snapshot.z
            sync_pose.orientation.x = qx
            sync_pose.orientation.y = qy
            sync_pose.orientation.z = qz
            sync_pose.orientation.w = qw
            named_pose_array.agents.append(AgentPose(name=snapshot.name, pose=sync_pose))

            body = Marker()
            body.header.frame_id = 'world'
            body.header.stamp = now
            body.ns = 'pedestrian_agents'
            body.id = index
            body.type = Marker.CYLINDER
            body.action = Marker.ADD
            body.pose.position.x = snapshot.x
            body.pose.position.y = snapshot.y
            body.pose.position.z = snapshot.z + 0.55
            body.pose.orientation.x = qx
            body.pose.orientation.y = qy
            body.pose.orientation.z = qz
            body.pose.orientation.w = qw
            body.scale.x = 0.38
            body.scale.y = 0.38
            body.scale.z = 1.7
            red, green, blue = color_for_agent(agent)
            body.color.r = red
            body.color.g = green
            body.color.b = blue
            body.color.a = 0.55
            marker_array.markers.append(body)

            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = now
            marker.ns = 'pedestrian_names'
            marker.id = 500 + index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = snapshot.x
            marker.pose.position.y = snapshot.y
            marker.pose.position.z = snapshot.z + 1.2
            marker.scale.z = 0.25
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = f'{snapshot.name} t{agent.agent_type} g{agent.group_id} {agent.profile}'
            marker_array.markers.append(marker)

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
        self.pose_pub.publish(named_pose_array)
        self.marker_pub.publish(marker_array)


def main() -> None:
    rclpy.init()
    node = PedestrianStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
