from setuptools import setup
import os
from glob import glob

package_name = 'lvdot_realistic_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skbt2',
    maintainer_email='skbt2@todo.todo',
    description='Realistic D435i and Mid-360 sensor emulators for LV-DOT',
    license='MIT',
    entry_points={
        'console_scripts': [
            'd435i_sim = lvdot_realistic_sensors.d435i_sim:main',
            'mid360_sim = lvdot_realistic_sensors.mid360_sim:main',
        ],
    },
)
