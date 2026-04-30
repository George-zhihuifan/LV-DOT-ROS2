from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    detector_launch = Path(get_package_share_directory("lvdot_bringup")) / "launch" / "run_detector_with_adapter.launch.py"
    scene_launch = Path(get_package_share_directory("depth_eval_bringup")) / "launch" / "uav_pedestrian_prototype.launch.py"

    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("detector_rviz", default_value="false"),
        DeclareLaunchArgument("enable_uav_controller", default_value="false"),
        DeclareLaunchArgument("enable_stage_timers", default_value="true"),
        DeclareLaunchArgument("enable_vis_stage", default_value="true"),
        DeclareLaunchArgument("executor_threads", default_value="4"),
        DeclareLaunchArgument("enable_yolo", default_value="false"),
        DeclareLaunchArgument("launch_yolo_node", default_value="false"),
        DeclareLaunchArgument("use_all_classes", default_value="false"),
        DeclareLaunchArgument("enable_color_fallback", default_value="false"),
        DeclareLaunchArgument("conf_threshold", default_value="0.25"),
        DeclareLaunchArgument("inference_hz", default_value="10.0"),
        DeclareLaunchArgument("launch_pose_stub", default_value="true"),
        DeclareLaunchArgument("lidar_pointcloud_topic", default_value="/uav_lidar/scan/points"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(scene_launch)),
            launch_arguments={
                "rviz": LaunchConfiguration("rviz"),
                "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                "enable_uav_controller": LaunchConfiguration("enable_uav_controller"),
                "relay_lvdot_topics": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(detector_launch)),
            launch_arguments={
                "launch_relay": "false",
                "launch_pose_stub": LaunchConfiguration("launch_pose_stub"),
                "launch_gt_publisher": "false",
                "launch_yolo_node": LaunchConfiguration("launch_yolo_node"),
                "enable_yolo": LaunchConfiguration("enable_yolo"),
                "use_all_classes": LaunchConfiguration("use_all_classes"),
                "enable_color_fallback": LaunchConfiguration("enable_color_fallback"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "inference_hz": LaunchConfiguration("inference_hz"),
                "enable_stage_timers": LaunchConfiguration("enable_stage_timers"),
                "enable_vis_stage": LaunchConfiguration("enable_vis_stage"),
                "executor_threads": LaunchConfiguration("executor_threads"),
                "lidar_pointcloud_topic": LaunchConfiguration("lidar_pointcloud_topic"),
                "rviz": LaunchConfiguration("detector_rviz"),
            }.items(),
        ),
    ])
