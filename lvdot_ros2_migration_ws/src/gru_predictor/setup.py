from setuptools import find_packages, setup

package_name = 'gru_predictor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gru_predictor.launch.py']),
        ('share/' + package_name + '/config', ['config/config_tuned.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skbt2',
    maintainer_email='devnull@example.com',
    description='ROS2 GRU hybrid trajectory predictor for LV-DOT dynamic boxes.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'predict_node = gru_predictor.predict_node:main',
        ],
    },
)
