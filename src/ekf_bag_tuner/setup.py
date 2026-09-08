from setuptools import setup
import os
from glob import glob

package_name = 'ekf_bag_tuner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'pyyaml'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='max@todo.todo',
    description='Offline EKF tuning for ZED + 2D lidar',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_republisher = ekf_bag_tuner.odom_republisher:main',
            'trajectory_recorder = ekf_bag_tuner.trajectory_recorder:main',
            'analyze_bag = ekf_bag_tuner.analyze_bag:main',
            'offline_fuse = ekf_bag_tuner.offline_fuse:main',
            'relative_fuse = ekf_bag_tuner.relative_fuse:main',
            'live_plotter = ekf_bag_tuner.live_plotter:main',
        ],
    },
)
