from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path


def generate_launch_description() -> LaunchDescription:
    bringup_dir = Path(get_package_share_directory("lvdot_bringup"))
    config_path = bringup_dir / "config" / "detector_param.yaml"
    rviz_config_path = bringup_dir / "rviz" / "lvdot_detector.rviz"

    detector_node = Node(
        package="lvdot_ros2",
        executable="lvdot_detector_main",
        name="lvdot_detector_node",
        output="screen",
        parameters=[str(config_path)],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="lvdot_detector_rviz",
        output="screen",
        arguments=["-d", str(rviz_config_path)],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        detector_node,
        rviz_node,
    ])
