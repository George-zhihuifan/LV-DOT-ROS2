import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('depth_eval_bringup')
    plugin_prefix = get_package_prefix('depth_eval_ped_gz_plugin')
    world_path = os.path.join(pkg_bringup, 'worlds', 'pedestrian_prototype.sdf')
    model_path = os.path.join(pkg_bringup, 'models')

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_path],
        name='gazebo',
        output='screen',
        shell=False,
        on_exit=Shutdown(),
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        name='gazebo_gui',
        output='screen',
        shell=False,
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    state_publisher = Node(
        package='depth_eval_bringup',
        executable='pedestrian_state_publisher',
        condition=IfCondition(LaunchConfiguration('publish_states')),
        parameters=[{
            'config_path': os.path.join(pkg_bringup, 'config', 'pedestrian_prototype.yaml'),
        }],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2.'),
        DeclareLaunchArgument('gazebo_gui', default_value='false', description='Open Gazebo GUI and connect to the running server.'),
        DeclareLaunchArgument('publish_states', default_value='true', description='Spawn and publish runtime pedestrian states.'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=f'{pkg_bringup}:{model_path}'
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=os.path.join(plugin_prefix, 'lib')
        ),
        SetEnvironmentVariable(
            name='__GLX_VENDOR_LIBRARY_NAME',
            value='nvidia'
        ),
        SetEnvironmentVariable(
            name='LIBGL_ALWAYS_SOFTWARE',
            value='0'
        ),
        gz_sim,
        gz_gui,
        state_publisher,
        rviz,
    ])
