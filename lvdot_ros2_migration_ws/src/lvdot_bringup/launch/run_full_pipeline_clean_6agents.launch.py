"""Full pipeline launch for the clean 6-agent QC-GAF inspection scene."""
from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("lvdot_bringup"))
    scene_launch = bringup_share / "launch" / "run_detector_with_scene_clean_6agents.launch.py"

    qcgaf_share = get_package_share_directory("qcgaf_fusion")
    qcgaf_default_config = os.path.join(qcgaf_share, "config", "config.yaml")

    gru_share = get_package_share_directory("gru_predictor")
    gru_default_config = os.path.join(gru_share, "config", "config_tuned.yaml")

    ws_root = next((__wsp for __wsp in Path(__file__).resolve().parents if (__wsp / "models" / "qcgaf").is_dir()), Path(__file__).resolve().parents[3])
    qcgaf_default_ckpt = str(ws_root / "models" / "qcgaf" / "best_model.pt")
    gru_default_model = str(ws_root / "models" / "gru" / "best_model.pth")
    yolo_default_weight = str(ws_root / "models" / "yolo" / "yolo11n.engine")
    rviz_default_config = bringup_share / "rviz" / "lvdot_detector.rviz"
    default_detector_config = str(bringup_share / "config" / "detector_param_clean_refinement.yaml")
    default_scenario_config = str(bringup_share / "config" / "clean_scenarios" / "pedestrian_clean_06agents.yaml")
    use_realistic = LaunchConfiguration("use_realistic_sensors")

    def pick(realistic_topic: str, raw_topic: str) -> PythonExpression:
        return PythonExpression([
            "'", realistic_topic, "' if '", use_realistic, "' == 'true' else '", raw_topic, "'",
        ])

    return LaunchDescription([
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("use_realistic_sensors", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("pose_stub_orbit_enabled", default_value="false"),
        DeclareLaunchArgument("pose_stub_orbit_radius", default_value="0.4"),
        DeclareLaunchArgument("pose_stub_orbit_speed", default_value="0.3"),
        DeclareLaunchArgument("pose_mode", default_value="orbit"),
        DeclareLaunchArgument("mission_speed", default_value="1.5"),
        DeclareLaunchArgument("mission_altitude", default_value="1.2"),
        DeclareLaunchArgument("executor_threads", default_value="8"),
        DeclareLaunchArgument("enable_yolo", default_value="true"),
        DeclareLaunchArgument("launch_yolo_node", default_value="true"),
        DeclareLaunchArgument("conf_threshold", default_value="0.25"),
        DeclareLaunchArgument("inference_hz", default_value="30.0"),
        DeclareLaunchArgument("imgsz", default_value="352"),
        DeclareLaunchArgument("max_det", default_value="10"),
        DeclareLaunchArgument("frame_stride", default_value="1"),
        DeclareLaunchArgument("yolo_weight_path", default_value=yolo_default_weight),
        DeclareLaunchArgument("rviz_config", default_value=str(rviz_default_config)),
        DeclareLaunchArgument("u_map_threshold_point", default_value="2"),
        DeclareLaunchArgument("u_map_threshold_line", default_value="5"),
        DeclareLaunchArgument("u_map_min_length_line", default_value="2"),
        DeclareLaunchArgument("depth_branch_offset_sec", default_value="0.20"),
        DeclareLaunchArgument("depth_branch_history_size", default_value="32"),
        DeclareLaunchArgument("max_depth_lidar_skew_sec", default_value="0.80"),
        DeclareLaunchArgument("mid360_points_per_frame", default_value="28000"),
        DeclareLaunchArgument("mid360_range_noise_per_m", default_value="0.0008"),
        DeclareLaunchArgument("d435_depth_noise_coef", default_value="0.0010"),
        DeclareLaunchArgument("d435_dropout_prob", default_value="0.03"),
        DeclareLaunchArgument("d435_quantize_mm", default_value="true"),
        DeclareLaunchArgument(
            "color_image_topic",
            default_value=pick("/d435i/color/image_raw", "/rgbd_camera/image"),
        ),
        DeclareLaunchArgument(
            "depth_image_topic",
            default_value=pick("/d435i/depth/image_rect_raw", "/rgbd_camera/depth_image"),
        ),
        DeclareLaunchArgument(
            "lidar_pointcloud_topic",
            default_value=pick("/mid360/pointcloud", "/uav_lidar/scan/points"),
        ),
        DeclareLaunchArgument("detector_config", default_value=default_detector_config),
        DeclareLaunchArgument("scenario_config", default_value=default_scenario_config),

        DeclareLaunchArgument("qcgaf_config", default_value=qcgaf_default_config),
        DeclareLaunchArgument("qcgaf_checkpoint", default_value=qcgaf_default_ckpt),
        DeclareLaunchArgument("qcgaf_verbose", default_value="false"),
        DeclareLaunchArgument("qcgaf_debug_metrics", default_value="true"),
        DeclareLaunchArgument("enable_qcgaf", default_value="true"),
        DeclareLaunchArgument("qcgaf_enable_lidar_fallback", default_value="true"),
        DeclareLaunchArgument("qcgaf_post_center_blend_alpha", default_value="0.30"),
        DeclareLaunchArgument("qcgaf_post_size_blend_alpha", default_value="0.20"),
        DeclareLaunchArgument("qcgaf_marker_track_match_distance", default_value="0.90"),
        DeclareLaunchArgument("qcgaf_marker_track_min_hits", default_value="3"),
        DeclareLaunchArgument("qcgaf_marker_track_ema_alpha", default_value="0.65"),
        DeclareLaunchArgument("qcgaf_marker_track_velocity_alpha", default_value="0.60"),
        DeclareLaunchArgument("qcgaf_marker_track_publish_hysteresis_miss", default_value="4"),

        DeclareLaunchArgument("gru_config", default_value=gru_default_config),
        DeclareLaunchArgument("gru_model", default_value=gru_default_model),
        DeclareLaunchArgument("gru_horizon", default_value="5"),
        DeclareLaunchArgument("gru_device", default_value="cuda"),
        DeclareLaunchArgument("gru_max_idle", default_value="3.0"),
        DeclareLaunchArgument("gru_input_topic", default_value="/onboard_detector/dynamic_bboxes"),
        DeclareLaunchArgument("enable_gru", default_value="true"),

        DeclareLaunchArgument("launch_evaluator", default_value="false"),
        DeclareLaunchArgument("launch_advanced_evaluator", default_value="false"),
        DeclareLaunchArgument("evaluator_csv_path", default_value="/tmp/lvdot_eval_full.csv"),
        DeclareLaunchArgument("evaluator_summary_path", default_value="/tmp/lvdot_eval_summary.json"),
        DeclareLaunchArgument("evaluator_matched_pairs_csv_path", default_value=""),
        DeclareLaunchArgument("evaluator_tracking_pairs_csv_path", default_value=""),
        DeclareLaunchArgument("evaluator_match_threshold_m", default_value="2.5"),
        DeclareLaunchArgument("evaluator_include_static_obstacles", default_value="false"),
        DeclareLaunchArgument("evaluator_gt_obstacle_topic", default_value="/pedestrian_sim/agent_markers"),
        DeclareLaunchArgument("evaluator_gt_obstacle_namespace", default_value="pedestrian_obstacles"),
        DeclareLaunchArgument("evaluator_det_topic", default_value="/qcgaf/fused_bboxes"),
        DeclareLaunchArgument("evaluator_pred_topic", default_value="/gru_predictor/predicted_positions"),
        DeclareLaunchArgument("evaluator_det_marker_type", default_value="1"),
        DeclareLaunchArgument("evaluator_det_namespace", default_value="qcgaf_fused"),
        DeclareLaunchArgument("evaluator_tracking_det_topic", default_value=""),
        DeclareLaunchArgument("evaluator_tracking_det_marker_type", default_value="-1"),
        DeclareLaunchArgument("evaluator_tracking_det_namespace", default_value=""),
        DeclareLaunchArgument("evaluator_eval_duration_sec", default_value="60.0"),
        DeclareLaunchArgument("evaluator_warmup_sec", default_value="15.0"),
        DeclareLaunchArgument("evaluator_center_match_threshold_m", default_value="1.0"),
        DeclareLaunchArgument("evaluator_gt_bbox_width_m", default_value="0.36"),
        DeclareLaunchArgument("evaluator_gt_bbox_depth_m", default_value="0.36"),
        DeclareLaunchArgument("evaluator_gt_bbox_height_m", default_value="1.70"),
        DeclareLaunchArgument("evaluator_gt_bbox_center_y_offset_m", default_value="-0.03"),
        DeclareLaunchArgument("evaluator_gt_bbox_center_z_offset_m", default_value="0.88"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(scene_launch)),
            launch_arguments={
                "gazebo_gui": LaunchConfiguration("gazebo_gui"),
                "rviz": "false",
                "detector_rviz": "false",
                "use_realistic_sensors": LaunchConfiguration("use_realistic_sensors"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "enable_yolo": LaunchConfiguration("enable_yolo"),
                "launch_yolo_node": LaunchConfiguration("launch_yolo_node"),
                "launch_pose_stub": "true",
                "pose_stub_orbit_enabled": LaunchConfiguration("pose_stub_orbit_enabled"),
                "pose_stub_orbit_radius": LaunchConfiguration("pose_stub_orbit_radius"),
                "pose_stub_orbit_speed": LaunchConfiguration("pose_stub_orbit_speed"),
                "pose_mode": LaunchConfiguration("pose_mode"),
                "mission_speed": LaunchConfiguration("mission_speed"),
                "mission_altitude": LaunchConfiguration("mission_altitude"),
                "executor_threads": LaunchConfiguration("executor_threads"),
                "conf_threshold": LaunchConfiguration("conf_threshold"),
                "imgsz": LaunchConfiguration("imgsz"),
                "max_det": LaunchConfiguration("max_det"),
                "inference_hz": LaunchConfiguration("inference_hz"),
                "frame_stride": LaunchConfiguration("frame_stride"),
                "yolo_weight_path": LaunchConfiguration("yolo_weight_path"),
                "u_map_threshold_point": LaunchConfiguration("u_map_threshold_point"),
                "u_map_threshold_line": LaunchConfiguration("u_map_threshold_line"),
                "u_map_min_length_line": LaunchConfiguration("u_map_min_length_line"),
                "depth_branch_offset_sec": LaunchConfiguration("depth_branch_offset_sec"),
                "depth_branch_history_size": LaunchConfiguration("depth_branch_history_size"),
                "max_depth_lidar_skew_sec": LaunchConfiguration("max_depth_lidar_skew_sec"),
                "detector_config": LaunchConfiguration("detector_config"),
                "scenario_config": LaunchConfiguration("scenario_config"),
                "mid360_points_per_frame": LaunchConfiguration("mid360_points_per_frame"),
                "mid360_range_noise_per_m": LaunchConfiguration("mid360_range_noise_per_m"),
                "d435_depth_noise_coef": LaunchConfiguration("d435_depth_noise_coef"),
                "d435_dropout_prob": LaunchConfiguration("d435_dropout_prob"),
                "d435_quantize_mm": LaunchConfiguration("d435_quantize_mm"),
                "color_image_topic": LaunchConfiguration("color_image_topic"),
                "depth_image_topic": LaunchConfiguration("depth_image_topic"),
                "lidar_pointcloud_topic": LaunchConfiguration("lidar_pointcloud_topic"),
                "launch_evaluator": LaunchConfiguration("launch_evaluator"),
                "launch_advanced_evaluator": LaunchConfiguration("launch_advanced_evaluator"),
                "evaluator_csv_path": LaunchConfiguration("evaluator_csv_path"),
                "evaluator_summary_path": LaunchConfiguration("evaluator_summary_path"),
                "evaluator_matched_pairs_csv_path": LaunchConfiguration("evaluator_matched_pairs_csv_path"),
                "evaluator_tracking_pairs_csv_path": LaunchConfiguration("evaluator_tracking_pairs_csv_path"),
                "evaluator_match_threshold_m": LaunchConfiguration("evaluator_match_threshold_m"),
                "evaluator_include_static_obstacles": LaunchConfiguration("evaluator_include_static_obstacles"),
                "evaluator_gt_obstacle_topic": LaunchConfiguration("evaluator_gt_obstacle_topic"),
                "evaluator_gt_obstacle_namespace": LaunchConfiguration("evaluator_gt_obstacle_namespace"),
                "evaluator_det_topic": LaunchConfiguration("evaluator_det_topic"),
                "evaluator_pred_topic": LaunchConfiguration("evaluator_pred_topic"),
                "evaluator_det_marker_type": LaunchConfiguration("evaluator_det_marker_type"),
                "evaluator_det_namespace": LaunchConfiguration("evaluator_det_namespace"),
                "evaluator_tracking_det_topic": LaunchConfiguration("evaluator_tracking_det_topic"),
                "evaluator_tracking_det_marker_type": LaunchConfiguration("evaluator_tracking_det_marker_type"),
                "evaluator_tracking_det_namespace": LaunchConfiguration("evaluator_tracking_det_namespace"),
                "evaluator_eval_duration_sec": LaunchConfiguration("evaluator_eval_duration_sec"),
                "evaluator_warmup_sec": LaunchConfiguration("evaluator_warmup_sec"),
                "evaluator_center_match_threshold_m": LaunchConfiguration("evaluator_center_match_threshold_m"),
                "evaluator_gt_bbox_width_m": LaunchConfiguration("evaluator_gt_bbox_width_m"),
                "evaluator_gt_bbox_depth_m": LaunchConfiguration("evaluator_gt_bbox_depth_m"),
                "evaluator_gt_bbox_height_m": LaunchConfiguration("evaluator_gt_bbox_height_m"),
                "evaluator_gt_bbox_center_y_offset_m": LaunchConfiguration("evaluator_gt_bbox_center_y_offset_m"),
                "evaluator_gt_bbox_center_z_offset_m": LaunchConfiguration("evaluator_gt_bbox_center_z_offset_m"),
            }.items(),
        ),

        Node(
            package="qcgaf_fusion",
            executable="fusion_node",
            name="qcgaf_fusion_node",
            output="screen",
            parameters=[{
                "config": LaunchConfiguration("qcgaf_config"),
                "checkpoint": LaunchConfiguration("qcgaf_checkpoint"),
                "verbose": LaunchConfiguration("qcgaf_verbose"),
                "debug_metrics": LaunchConfiguration("qcgaf_debug_metrics"),
                "enable_lidar_fallback": LaunchConfiguration("qcgaf_enable_lidar_fallback"),
                "post_center_blend_alpha": LaunchConfiguration("qcgaf_post_center_blend_alpha"),
                "post_size_blend_alpha": LaunchConfiguration("qcgaf_post_size_blend_alpha"),
                "marker_track_match_distance": LaunchConfiguration("qcgaf_marker_track_match_distance"),
                "marker_track_min_hits": LaunchConfiguration("qcgaf_marker_track_min_hits"),
                "marker_track_ema_alpha": LaunchConfiguration("qcgaf_marker_track_ema_alpha"),
                "marker_track_velocity_alpha": LaunchConfiguration("qcgaf_marker_track_velocity_alpha"),
                "marker_track_publish_hysteresis_miss": LaunchConfiguration("qcgaf_marker_track_publish_hysteresis_miss"),
                "color_topic": LaunchConfiguration("color_image_topic"),
                "depth_topic": LaunchConfiguration("depth_image_topic"),
                "lidar_cloud_topic": LaunchConfiguration("lidar_pointcloud_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            additional_env={"CUDA_VISIBLE_DEVICES": "1"},
            condition=IfCondition(LaunchConfiguration("enable_qcgaf")),
        ),

        Node(
            package="gru_predictor",
            executable="predict_node",
            name="gru_prediction_node",
            output="screen",
            parameters=[{
                "config": LaunchConfiguration("gru_config"),
                "model": LaunchConfiguration("gru_model"),
                "input_topic": LaunchConfiguration("gru_input_topic"),
                "output_topic": "/gru_predictor/predicted_positions",
                "horizon": LaunchConfiguration("gru_horizon"),
                "device": LaunchConfiguration("gru_device"),
                "max_idle": LaunchConfiguration("gru_max_idle"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            additional_env={"CUDA_VISIBLE_DEVICES": "0"},
            condition=IfCondition(LaunchConfiguration("enable_gru")),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="lvdot_full_pipeline_rviz",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
