from setuptools import find_packages, setup

package_name = 'qcgaf_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/qcgaf_fusion.launch.py']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skbt2',
    maintainer_email='devnull@example.com',
    description='ROS2 QC-GAF fusion node for LV-DOT.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fusion_node = qcgaf_fusion.fusion_node:main',
        ],
    },
)
