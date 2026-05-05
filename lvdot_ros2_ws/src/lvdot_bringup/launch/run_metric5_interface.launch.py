from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    bringup_dir = Path(get_package_share_directory("lvdot_bringup"))
    bringup_prefix = Path(get_package_prefix("lvdot_bringup"))
    detector_scene_launch = bringup_dir / "launch" / "run_detector_with_scene.launch.py"
    adapter_script = bringup_prefix / "lib" / "lvdot_bringup" / "metric5_interface_adapter.py"

    return LaunchDescription([
        DeclareLaunchArgument("launch_detector_stack", default_value="true"),
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("detector_rviz", default_value="false"),
        DeclareLaunchArgument("enable_uav_controller", default_value="false"),
        DeclareLaunchArgument("enable_yolo", default_value="true"),
        DeclareLaunchArgument("launch_yolo_node", default_value="true"),
        DeclareLaunchArgument("fusion_mode", default_value="dual"),
        DeclareLaunchArgument("enable_stage_timers", default_value="true"),
        DeclareLaunchArgument("enable_vis_stage", default_value="true"),
        DeclareLaunchArgument("executor_threads", default_value="4"),

        DeclareLaunchArgument("output_mode", default_value="both"),
        DeclareLaunchArgument("source_ns_prefix", default_value="tracked"),
        DeclareLaunchArgument("input_image_topic", default_value="/rgbd_camera/image"),
        DeclareLaunchArgument("input_pointcloud_topic", default_value="/uav_lidar/scan/points"),
        DeclareLaunchArgument("input_detected_image_topic", default_value="/yolo_detector/detected_image"),
        DeclareLaunchArgument("input_boxes_topic", default_value="/onboard_detector/tracked_bboxes"),
        DeclareLaunchArgument("output_marker_topic", default_value="/metric5/detection_3d_marker"),
        DeclareLaunchArgument("output_image_topic", default_value="/metric5/detection_2d_image"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(detector_scene_launch)),
            condition=IfCondition(LaunchConfiguration("launch_detector_stack")),
            launch_arguments={
                "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                "rviz": LaunchConfiguration("rviz"),
                "detector_rviz": LaunchConfiguration("detector_rviz"),
                "enable_uav_controller": LaunchConfiguration("enable_uav_controller"),
                "enable_yolo": LaunchConfiguration("enable_yolo"),
                "launch_yolo_node": LaunchConfiguration("launch_yolo_node"),
                "fusion_mode": LaunchConfiguration("fusion_mode"),
                "enable_stage_timers": LaunchConfiguration("enable_stage_timers"),
                "enable_vis_stage": LaunchConfiguration("enable_vis_stage"),
                "executor_threads": LaunchConfiguration("executor_threads"),
            }.items(),
        ),

        ExecuteProcess(
            cmd=[
                "python3",
                str(adapter_script),
                "--ros-args",
                "-p", ["output_mode:=", LaunchConfiguration("output_mode")],
                "-p", ["source_ns_prefix:=", LaunchConfiguration("source_ns_prefix")],
                "-p", ["input_image_topic:=", LaunchConfiguration("input_image_topic")],
                "-p", ["input_pointcloud_topic:=", LaunchConfiguration("input_pointcloud_topic")],
                "-p", ["input_detected_image_topic:=", LaunchConfiguration("input_detected_image_topic")],
                "-p", ["input_boxes_topic:=", LaunchConfiguration("input_boxes_topic")],
                "-p", ["output_marker_topic:=", LaunchConfiguration("output_marker_topic")],
                "-p", ["output_image_topic:=", LaunchConfiguration("output_image_topic")],
            ],
            output="screen",
        ),
    ])
