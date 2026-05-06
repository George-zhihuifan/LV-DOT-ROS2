from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_dir = Path(get_package_share_directory("lvdot_bringup"))
    config_path = bringup_dir / "config" / "detector_param.yaml"

    return LaunchDescription([
        DeclareLaunchArgument("use_gt_detections", default_value="false"),
        DeclareLaunchArgument("enable_yolo", default_value="false"),
        DeclareLaunchArgument("use_all_classes", default_value="false"),
        DeclareLaunchArgument("enable_color_fallback", default_value="false"),
        DeclareLaunchArgument("conf_threshold", default_value="0.25"),
        DeclareLaunchArgument("launch_relay", default_value="true"),
        DeclareLaunchArgument("launch_pose_stub", default_value="false"),
        DeclareLaunchArgument("launch_gt_publisher", default_value="true"),
        DeclareLaunchArgument("launch_yolo_node", default_value="true"),
        DeclareLaunchArgument("imgsz", default_value="352"),
        DeclareLaunchArgument("max_det", default_value="10"),
        DeclareLaunchArgument("inference_hz", default_value="10.0"),
        DeclareLaunchArgument("frame_stride", default_value="1"),
        DeclareLaunchArgument(
            "yolo_weight_path",
            default_value="/home/skbt2/LV-DOT/onboard_detector/scripts/yolo_detector/weights/yolo11n.pt",
        ),
        DeclareLaunchArgument("enable_stage_timers", default_value="true"),
        DeclareLaunchArgument("enable_vis_stage", default_value="true"),
        DeclareLaunchArgument("executor_threads", default_value="4"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("depth_image_topic", default_value="/rgbd_camera/depth_image"),
        DeclareLaunchArgument("color_image_topic", default_value="/rgbd_camera/image"),
        DeclareLaunchArgument("lidar_pointcloud_topic", default_value="/livox/lidar/pointcloud"),
        DeclareLaunchArgument("pose_topic", default_value="/mavros/local_position/pose"),
        DeclareLaunchArgument("odom_topic", default_value="/mavros/local_position/odom"),
        DeclareLaunchArgument("yolo_detection_topic", default_value="/yolo_detector/detected_bounding_boxes"),
        DeclareLaunchArgument("fusion_mode", default_value="dual"),
        DeclareLaunchArgument(
            "gt_csv",
            default_value="/home/skbt2/ros2_depth_eval_ws/artifacts/experiment_gt.csv",
        ),
        Node(
            package="lvdot_ros2_adapter",
            executable="image_pointcloud_relay",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("launch_relay")),
        ),
        Node(
            package="lvdot_ros2_adapter",
            executable="pose_stub",
            output="screen",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("launch_pose_stub")),
        ),
        Node(
            package="lvdot_ros2_adapter",
            executable="gt_detection_publisher",
            output="screen",
            parameters=[{"gt_csv": LaunchConfiguration("gt_csv"), "use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("launch_gt_publisher")),
        ),
        Node(
            package="lvdot_ros2_adapter",
            executable="lvdot_yolo_node",
            output="screen",
            parameters=[{
                "image_topic": LaunchConfiguration("color_image_topic"),
                "enable_yolo": LaunchConfiguration("enable_yolo"),
                "use_all_classes": LaunchConfiguration("use_all_classes"),
                "enable_color_fallback": LaunchConfiguration("enable_color_fallback"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "imgsz": LaunchConfiguration("imgsz"),
                "max_det": LaunchConfiguration("max_det"),
                "inference_hz": LaunchConfiguration("inference_hz"),
                "frame_stride": LaunchConfiguration("frame_stride"),
                "weight_path": LaunchConfiguration("yolo_weight_path"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            condition=IfCondition(LaunchConfiguration("launch_yolo_node")),
        ),
        Node(
            package="lvdot_ros2",
            executable="lvdot_detector_main",
            name="lvdot_detector_node",
            output="screen",
            parameters=[
                str(config_path),
                {
                    "enable_stage_timers": LaunchConfiguration("enable_stage_timers"),
                    "enable_vis_stage": LaunchConfiguration("enable_vis_stage"),
                    "executor_threads": LaunchConfiguration("executor_threads"),
                    "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                    "color_image_topic": LaunchConfiguration("color_image_topic"),
                    "lidar_pointcloud_topic": LaunchConfiguration("lidar_pointcloud_topic"),
                    "pose_topic": LaunchConfiguration("pose_topic"),
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    "yolo_detection_topic": LaunchConfiguration("yolo_detection_topic"),
                    "fusion_mode": LaunchConfiguration("fusion_mode"),
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="lvdot_detector_rviz",
            output="screen",
            arguments=["-d", str(bringup_dir / "rviz" / "lvdot_detector.rviz")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
