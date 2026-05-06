from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory('qcgaf_fusion')
    default_config = os.path.join(share_dir, 'config', 'config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('checkpoint', default_value=''),
        DeclareLaunchArgument('verbose', default_value='false'),
        Node(
            package='qcgaf_fusion',
            executable='fusion_node',
            name='qcgaf_fusion_node',
            output='screen',
            parameters=[{
                'config': LaunchConfiguration('config'),
                'checkpoint': LaunchConfiguration('checkpoint'),
                'verbose': LaunchConfiguration('verbose'),
            }],
        )
    ])
