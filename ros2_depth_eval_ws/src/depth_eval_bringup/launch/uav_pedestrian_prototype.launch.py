import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('depth_eval_bringup')
    plugin_prefix = get_package_prefix('depth_eval_ped_gz_plugin')
    world_path = os.path.join(pkg_bringup, 'worlds', 'pedestrian_prototype.sdf')
    model_path = os.path.join(pkg_bringup, 'models')
    config_path = os.path.join(pkg_bringup, 'config', 'pedestrian_prototype.yaml')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'uav_pedestrian_scene.rviz')

    gz_sim = ExecuteProcess(
        cmd=[
            'gz', 'sim',
            '-s',
            '-r',
            '--headless-rendering',
            '--physics-engine', LaunchConfiguration('physics_engine_plugin'),
            world_path
        ],
        name='gazebo',
        output='log',
        shell=False,
        on_exit=Shutdown(),
    )

    gz_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        name='gazebo_gui',
        output='log',
        shell=False,
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/uav_lidar/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/pedestrian_prototype/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/pedestrian_prototype/set_pose/blocking@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='log',
    )

    state_publisher = Node(
        package='depth_eval_bringup',
        executable='pedestrian_state_publisher',
        condition=IfCondition(LaunchConfiguration('publish_states')),
        parameters=[{'config_path': config_path, 'use_sim_time': True}],
        output='log',
    )

    uav_controller = Node(
        package='depth_eval_bringup',
        executable='uav_trajectory_controller',
        condition=IfCondition(LaunchConfiguration('enable_uav_controller')),
        parameters=[{
            'config_path': config_path,
            'world_name': 'pedestrian_prototype',
            'pose_service_name': '/world/pedestrian_prototype/set_pose/blocking',
            'use_sim_time': True,
        }],
        output='log',
    )

    legacy_relay = Node(
        package='lvdot_ros2_adapter',
        executable='image_pointcloud_relay',
        condition=IfCondition(LaunchConfiguration('relay_lvdot_topics')),
        parameters=[{'use_sim_time': True}],
        output='log',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2.'),
        DeclareLaunchArgument('gazebo_gui', default_value='false', description='Open Gazebo GUI and connect to the running server.'),
        DeclareLaunchArgument(
            'physics_engine_plugin',
            default_value='gz-physics-dartsim-plugin',
            description='Physics engine plugin name, e.g. gz-physics-dartsim-plugin / gz-physics-bullet-plugin.'
        ),
        DeclareLaunchArgument('publish_states', default_value='true', description='Spawn and publish runtime pedestrian states.'),
        DeclareLaunchArgument('relay_lvdot_topics', default_value='true', description='Relay RGBD topics into LV-DOT-compatible names.'),
        DeclareLaunchArgument('enable_uav_controller', default_value='false', description='Run UAV trajectory controller.'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[model_path, ':', pkg_bringup, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_SYSTEM_PLUGIN_PATH',
            value=[os.path.join(plugin_prefix, 'lib'), ':', EnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', default_value='')]
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_PHYSICS_ENGINE_PATH',
            value=[
                '/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins',
                ':',
                EnvironmentVariable('GZ_SIM_PHYSICS_ENGINE_PATH', default_value=''),
            ]
        ),
        gz_sim,
        gz_gui,
        bridge,
        state_publisher,
        uav_controller,
        legacy_relay,
        rviz,
    ])
