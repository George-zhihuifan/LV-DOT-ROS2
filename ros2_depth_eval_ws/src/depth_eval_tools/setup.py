from glob import glob

from setuptools import find_packages, setup

package_name = 'depth_eval_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/scripts', glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='skbt2',
    maintainer_email='skbt2@todo.todo',
    description='Tools package for ROI depth quality analysis and export scripts.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bag_roi_depth_eval = depth_eval_tools.bag_roi_depth_eval:main',
            'experiment_gt_export = depth_eval_tools.experiment_gt_export:main',
            'live_roi_depth_eval = depth_eval_tools.live_roi_depth_eval:main',
            'roi_depth_eval = depth_eval_tools.roi_depth_eval:main',
            'static_depth_validity_eval = depth_eval_tools.static_depth_validity_eval:main',
            'summarize_live_roi_eval = depth_eval_tools.summarize_live_roi_eval:main',
            'topic_sanity_check = depth_eval_tools.topic_sanity_check:main',
            'uav_depth_target_sweep = depth_eval_tools.uav_depth_target_sweep:main',
            'uav_depth_validity_eval = depth_eval_tools.uav_depth_validity_eval:main',
        ],
    },
)
