import math

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy


class PoseStub(Node):
    def __init__(self) -> None:
        super().__init__('lvdot_pose_stub')
        self.pose_pub = self.create_publisher(PoseStamped, '/mavros/local_position/pose', 10)
        self.odom_pub = self.create_publisher(Odometry, '/mavros/local_position/odom', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_messages)

    def publish_messages(self) -> None:
        now = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = 'map'
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 1.2
        pose.pose.orientation.w = 1.0

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = pose.pose
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.z = 0.0 * math.pi

        self.pose_pub.publish(pose)
        self.odom_pub.publish(odom)


def main() -> None:
    rclpy.init()
    node = PoseStub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
