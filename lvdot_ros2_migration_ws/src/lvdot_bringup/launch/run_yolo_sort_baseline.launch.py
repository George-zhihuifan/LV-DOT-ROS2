from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("lvdot_bringup"))
    scene_launch = bringup_share / "launch" / "run_detector_with_scene.launch.py"
    default_detector_config = str(bringup_share / "config" / "detector_param_baseline.yaml")
    default_body_to_camera = [0.0, 0.0, 1.0, 0.30, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0]

    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("scenario_config"),
        DeclareLaunchArgument("evaluator_csv_path", default_value="/tmp/yolo_sort_eval.csv"),
        DeclareLaunchArgument("evaluator_summary_path", default_value="/tmp/yolo_sort_eval_summary.json"),
        DeclareLaunchArgument("evaluator_eval_duration_sec", default_value="60.0"),
        DeclareLaunchArgument("evaluator_warmup_sec", default_value="15.0"),
        DeclareLaunchArgument("evaluator_gt_bbox_width_m", default_value="0.36"),
        DeclareLaunchArgument("evaluator_gt_bbox_depth_m", default_value="0.36"),
        DeclareLaunchArgument("evaluator_gt_bbox_height_m", default_value="1.70"),
        DeclareLaunchArgument("evaluator_gt_bbox_center_y_offset_m", default_value="-0.03"),
        DeclareLaunchArgument("evaluator_gt_bbox_center_z_offset_m", default_value="0.88"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(scene_launch)),
            launch_arguments={
                "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                "rviz": LaunchConfiguration("rviz"),
                "detector_rviz": "false",
                "scenario_config": LaunchConfiguration("scenario_config"),
                "detector_config": default_detector_config,
                "launch_pose_stub": "true",
                "enable_yolo": "true",
                "launch_yolo_node": "true",
                "use_all_classes": "false",
                "conf_threshold": "0.5",
                "inference_hz": "10.0",
                "launch_advanced_evaluator": "false",
                "launch_evaluator": "false",
            }.items(),
        ),

        Node(
            package="lvdot_ros2_adapter",
            executable="yolo_sort_baseline_node",
            name="yolo_sort_baseline_node",
            output="screen",
            parameters=[{
                "depth_intrinsics": [337.357, 337.357, 320.0, 240.0],
                "body_to_camera": default_body_to_camera,
                "depth_scale": 1000.0,
                "default_bbox_size": [0.5, 0.5, 1.7],
                "sort_max_age": 5,
                "sort_min_hits": 3,
                "sort_iou_threshold": 1.0,
                "depth_sample_radius": 3,
            }],
        ),

        Node(
            package="lvdot_ros2_adapter",
            executable="advanced_evaluator",
            name="advanced_evaluator",
            output="screen",
            parameters=[{
                "det_topic": "/yolo_sort/tracked_bboxes",
                "det_marker_type": 1,
                "det_namespace": "yolo_sort",
                "pred_topic": "/gru_predictor/predicted_positions",
                "csv_path": LaunchConfiguration("evaluator_csv_path"),
                "summary_path": LaunchConfiguration("evaluator_summary_path"),
                "eval_duration_sec": LaunchConfiguration("evaluator_eval_duration_sec"),
                "warmup_sec": LaunchConfiguration("evaluator_warmup_sec"),
                "center_match_threshold_m": 1.0,
                "gt_bbox_width_m": LaunchConfiguration("evaluator_gt_bbox_width_m"),
                "gt_bbox_depth_m": LaunchConfiguration("evaluator_gt_bbox_depth_m"),
                "gt_bbox_height_m": LaunchConfiguration("evaluator_gt_bbox_height_m"),
                "gt_bbox_center_y_offset_m": LaunchConfiguration("evaluator_gt_bbox_center_y_offset_m"),
                "gt_bbox_center_z_offset_m": LaunchConfiguration("evaluator_gt_bbox_center_z_offset_m"),
            }],
        ),
    ])
