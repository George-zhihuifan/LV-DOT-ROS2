from glob import glob

from setuptools import find_packages, setup

package_name = 'lvdot_ros2_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skbt2',
    maintainer_email='skbt2@todo.todo',
    description='ROS2 interface adapter for integrating LV-DOT expectations with the minimal depth eval scene.',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'image_pointcloud_relay = lvdot_ros2_adapter.image_pointcloud_relay:main',
            'pose_stub = lvdot_ros2_adapter.pose_stub:main',
            'lvdot_yolo_node = lvdot_ros2_adapter.lvdot_yolo_node:main',
            'detection_evaluator = lvdot_ros2_adapter.detection_evaluator:main',
            'advanced_evaluator = lvdot_ros2_adapter.advanced_evaluator:main',
            'uav_waypoint_mission = lvdot_ros2_adapter.uav_waypoint_mission:main',
            'yolo_sort_baseline_node = lvdot_ros2_adapter.yolo_sort_baseline_node:main',
        ],
    },
)
