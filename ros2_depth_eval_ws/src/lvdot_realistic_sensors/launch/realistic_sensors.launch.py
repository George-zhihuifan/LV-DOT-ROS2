from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    mid360_points_per_frame = LaunchConfiguration("mid360_points_per_frame")
    mid360_range_noise_per_m = LaunchConfiguration("mid360_range_noise_per_m")
    d435_depth_noise_coef = LaunchConfiguration("d435_depth_noise_coef")
    d435_dropout_prob = LaunchConfiguration("d435_dropout_prob")
    d435_quantize_mm = LaunchConfiguration("d435_quantize_mm")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("mid360_points_per_frame", default_value="28000"),
        DeclareLaunchArgument("mid360_range_noise_per_m", default_value="0.0008"),
        DeclareLaunchArgument("d435_depth_noise_coef", default_value="0.0010"),
        DeclareLaunchArgument("d435_dropout_prob", default_value="0.03"),
        DeclareLaunchArgument("d435_quantize_mm", default_value="true"),
        Node(
            package="lvdot_realistic_sensors",
            executable="d435i_sim",
            name="d435i_sim",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "depth_noise_coef": d435_depth_noise_coef,
                "dropout_prob": d435_dropout_prob,
                "quantize_mm": d435_quantize_mm,
            }],
        ),
        Node(
            package="lvdot_realistic_sensors",
            executable="mid360_sim",
            name="mid360_sim",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "points_per_frame": mid360_points_per_frame,
                "range_noise_per_m": mid360_range_noise_per_m,
            }],
        ),
    ])
