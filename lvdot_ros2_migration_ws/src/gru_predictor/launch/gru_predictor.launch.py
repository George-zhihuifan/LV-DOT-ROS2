from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory('gru_predictor')
    default_config = os.path.join(share_dir, 'config', 'config_tuned.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('input_topic', default_value='/onboard_detector/dynamic_bboxes'),
        DeclareLaunchArgument('output_topic', default_value='/gru_predictor/predicted_positions'),
        DeclareLaunchArgument('horizon', default_value='5'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('max_idle', default_value='3.0'),
        Node(
            package='gru_predictor',
            executable='predict_node',
            name='gru_prediction_node',
            output='screen',
            parameters=[{
                'config': LaunchConfiguration('config'),
                'model': LaunchConfiguration('model'),
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'horizon': LaunchConfiguration('horizon'),
                'device': LaunchConfiguration('device'),
                'max_idle': LaunchConfiguration('max_idle'),
            }],
        )
    ])
