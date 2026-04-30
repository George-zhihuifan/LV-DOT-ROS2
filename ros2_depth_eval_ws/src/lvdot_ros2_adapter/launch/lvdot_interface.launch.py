from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            'gt_csv',
            default_value='/home/skbt2/ros2_depth_eval_ws/artifacts/experiment_gt.csv',
            description='GT CSV used to publish LV-DOT-style 2D detections.',
        ),
        DeclareLaunchArgument(
            'use_gt_detections',
            default_value='false',
            description='Publish projected GT boxes instead of running YOLO.',
        ),
        DeclareLaunchArgument(
            'enable_yolo',
            default_value='false',
            description='Enable LV-DOT YOLO inference. Disabled by default for lightweight scene validation.',
        ),
        DeclareLaunchArgument(
            'use_all_classes',
            default_value='true',
            description='Allow all YOLO classes instead of filtering to person only.',
        ),
        DeclareLaunchArgument(
            'enable_color_fallback',
            default_value='true',
            description='Publish contour-based fallback detections when YOLO produces no boxes.',
        ),
        DeclareLaunchArgument(
            'imgsz',
            default_value='256',
            description='YOLO inference image size.',
        ),
        DeclareLaunchArgument(
            'max_det',
            default_value='10',
            description='Maximum number of detections per frame.',
        ),
        DeclareLaunchArgument(
            'inference_hz',
            default_value='2.0',
            description='Target YOLO inference frequency.',
        ),
        DeclareLaunchArgument(
            'frame_stride',
            default_value='1',
            description='Only keep every Nth input frame for inference.',
        ),
        Node(
            package='lvdot_ros2_adapter',
            executable='image_pointcloud_relay',
            output='screen',
        ),
        Node(
            package='lvdot_ros2_adapter',
            executable='pose_stub',
            output='screen',
        ),
        Node(
            package='lvdot_ros2_adapter',
            executable='gt_detection_publisher',
            output='screen',
            parameters=[{'gt_csv': LaunchConfiguration('gt_csv')}],
            condition=IfCondition(LaunchConfiguration('use_gt_detections')),
        ),
        Node(
            package='lvdot_ros2_adapter',
            executable='lvdot_yolo_node',
            output='screen',
            parameters=[{
                'enable_yolo': LaunchConfiguration('enable_yolo'),
                'use_all_classes': LaunchConfiguration('use_all_classes'),
                'enable_color_fallback': LaunchConfiguration('enable_color_fallback'),
                'imgsz': LaunchConfiguration('imgsz'),
                'max_det': LaunchConfiguration('max_det'),
                'inference_hz': LaunchConfiguration('inference_hz'),
                'frame_stride': LaunchConfiguration('frame_stride'),
            }],
            condition=UnlessCondition(LaunchConfiguration('use_gt_detections')),
        ),
    ])
