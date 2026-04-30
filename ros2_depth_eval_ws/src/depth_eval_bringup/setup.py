import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'depth_eval_bringup'


def package_files(directory: str, install_subdir: str):
    paths = []
    for path, _, filenames in os.walk(directory):
        filtered = [
            name for name in filenames
            if not name.endswith(('.pyc', '.pyo')) and name != '.DS_Store'
        ]
        if os.path.basename(path) == '__pycache__' or not filtered:
            continue
        install_path = os.path.join('share', package_name, install_subdir, os.path.relpath(path, directory))
        paths.append((install_path, [os.path.join(path, name) for name in filtered]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models/depth_camera_carrier', glob('models/depth_camera_carrier/*')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ] + package_files('models/pedsim_person', 'models/pedsim_person') \
      + package_files('models/pedsim_person_actor', 'models/pedsim_person_actor') \
      + package_files('models/actor_stand_local', 'models/actor_stand_local') \
      + package_files('models/uav_d435i_platform', 'models/uav_d435i_platform'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skbt2',
    maintainer_email='skbt2@todo.todo',
    description='Bringup package for the minimal ROS2 depth evaluation workspace.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pedestrian_prototype_controller = depth_eval_bringup.pedestrian_prototype_controller:main',
            'generate_pedestrian_world = depth_eval_bringup.pedestrian_world_generator:main',
            'export_pedestrian_scenario = depth_eval_bringup.pedestrian_scenario_exporter:main',
            'pedestrian_state_publisher = depth_eval_bringup.pedestrian_state_publisher:main',
            'uav_trajectory_controller = depth_eval_bringup.uav_trajectory_controller:main',
        ],
    },
)
