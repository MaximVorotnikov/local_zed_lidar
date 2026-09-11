from setuptools import setup
import os
from glob import glob

package_name = 'lidar_rf2o_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='max@todo.todo',
    description='Lidar-only rf2o navigation (no ZED)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scan_downsampler = lidar_rf2o_nav.scan_downsampler:main',
            'lidar_odom_publisher = lidar_rf2o_nav.lidar_odom_publisher:main',
        ],
    },
)
