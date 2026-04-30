from rclpy.node import Node
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class ImagePointCloudRelay(Node):
    def __init__(self) -> None:
        super().__init__('lvdot_image_pointcloud_relay')

        self.rgb_sub = self.create_subscription(Image, '/rgbd_camera/image', self.on_rgb, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(Image, '/rgbd_camera/depth_image', self.on_depth, qos_profile_sensor_data)
        self.info_sub = self.create_subscription(CameraInfo, '/rgbd_camera/camera_info', self.on_info, qos_profile_sensor_data)
        self.livox_sim_sub = self.create_subscription(
            PointCloud2, '/uav_lidar/scan/points', self.on_livox_points, qos_profile_sensor_data)
        self.livox_sim_sub_alt = self.create_subscription(
            PointCloud2, '/uav_lidar/points/points', self.on_livox_points, qos_profile_sensor_data)

        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', qos_profile_sensor_data)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
        self.livox_points_pub = self.create_publisher(PointCloud2, '/livox/lidar/pointcloud', qos_profile_sensor_data)

    def on_rgb(self, msg: Image) -> None:
        self.rgb_pub.publish(msg)

    def on_depth(self, msg: Image) -> None:
        self.depth_pub.publish(msg)

    def on_info(self, msg: CameraInfo) -> None:
        self.color_info_pub.publish(msg)
        self.depth_info_pub.publish(msg)

    def on_livox_points(self, msg: PointCloud2) -> None:
        # Contract with detector: /livox/lidar/pointcloud is LiDAR-local points.
        # Do not republish this topic as world-frame unless detector preprocessing is updated.
        msg.header.frame_id = 'livox_frame'
        self.livox_points_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = ImagePointCloudRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
