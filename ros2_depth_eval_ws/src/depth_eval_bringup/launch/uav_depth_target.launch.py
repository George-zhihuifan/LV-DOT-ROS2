import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('depth_eval_bringup')
    world_path = os.path.join(pkg_bringup, 'worlds', 'uav_depth_target.sdf')
    model_path = os.path.join(pkg_bringup, 'models')

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', '-s', '-r', world_path],
        name='gazebo',
        output='screen',
        shell=False,
        on_exit=Shutdown(),
    )

    gz_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-g'],
        name='gazebo_gui',
        output='screen',
        shell=False,
        condition=IfCondition(LaunchConfiguration('gazebo_gui')),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rgbd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/world/uav_depth_target/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/world/uav_depth_target/set_pose/blocking@ros_gz_interfaces/srv/SetEntityPose',
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_bringup, 'rviz', 'uav_pedestrian_scene.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='Open RViz2.'),
        DeclareLaunchArgument('gazebo_gui', default_value='false', description='Open Gazebo GUI and connect to the running server.'),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[model_path, ':', pkg_bringup, ':', EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')],
        ),
        gz_sim,
        gz_gui,
        bridge,
        rviz,
    ])
