from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from pathlib import Path
import os


def generate_launch_description() -> LaunchDescription:
    bringup_dir = Path(get_package_share_directory('lvdot_bringup'))
    detector_config_path = bringup_dir / 'config' / 'detector_param.yaml'

    qcgaf_share = get_package_share_directory('qcgaf_fusion')
    qcgaf_default_config = os.path.join(qcgaf_share, 'config', 'config.yaml')

    gru_share = get_package_share_directory('gru_predictor')
    gru_default_config = os.path.join(gru_share, 'config', 'config_tuned.yaml')

    detector_node = Node(
        package='lvdot_ros2',
        executable='lvdot_detector_main',
        name='lvdot_detector_node',
        output='screen',
        parameters=[str(detector_config_path), {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    qcgaf_node = Node(
        package='qcgaf_fusion',
        executable='fusion_node',
        name='qcgaf_fusion_node',
        output='screen',
        parameters=[{
            'config': LaunchConfiguration('qcgaf_config'),
            'checkpoint': LaunchConfiguration('qcgaf_checkpoint'),
            'verbose': LaunchConfiguration('qcgaf_verbose'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    gru_node = Node(
        package='gru_predictor',
        executable='predict_node',
        name='gru_prediction_node',
        output='screen',
        parameters=[{
            'config': LaunchConfiguration('gru_config'),
            'model': LaunchConfiguration('gru_model'),
            'input_topic': '/onboard_detector/dynamic_bboxes',
            'output_topic': '/gru_predictor/predicted_positions',
            'horizon': LaunchConfiguration('gru_horizon'),
            'device': LaunchConfiguration('gru_device'),
            'max_idle': LaunchConfiguration('gru_max_idle'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('qcgaf_config', default_value=qcgaf_default_config),
        DeclareLaunchArgument('qcgaf_checkpoint', default_value=''),
        DeclareLaunchArgument('qcgaf_verbose', default_value='false'),
        DeclareLaunchArgument('gru_config', default_value=gru_default_config),
        DeclareLaunchArgument('gru_model', default_value=''),
        DeclareLaunchArgument('gru_horizon', default_value='5'),
        DeclareLaunchArgument('gru_device', default_value='cpu'),
        DeclareLaunchArgument('gru_max_idle', default_value='3.0'),
        detector_node,
        qcgaf_node,
        gru_node,
    ])
