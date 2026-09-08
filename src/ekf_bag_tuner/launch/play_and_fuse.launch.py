#!/usr/bin/env python3
"""Play bag + rf2o + lidar-primary fuse + CSV recorder + live plot.

Exits automatically when bag playback finishes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _node(**kwargs):
    """Nodes must die quickly after bag ends (sim time freezes → spin can hang)."""
    kwargs.setdefault('sigterm_timeout', '2.0')
    kwargs.setdefault('sigkill_timeout', '2.0')
    return Node(**kwargs)


def generate_launch_description():
    pkg = get_package_share_directory('ekf_bag_tuner')
    default_bag = '/home/max/flight_ekf_02'
    default_out = '/home/max/local_zed_lidar/ekf_tuning_output'
    tuner_yaml = os.path.join(pkg, 'config', 'tuner.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'fuse.rviz')

    bag_path = LaunchConfiguration('bag_path')
    output_dir = LaunchConfiguration('output_dir')
    use_rviz = LaunchConfiguration('use_rviz')
    use_plot = LaunchConfiguration('use_plot')
    rate = LaunchConfiguration('rate')

    bag_topics = [
        '/scan',
        '/zed/zed_node/odom',
        '/zed/zed_node/pose',
        '/zed/zed_node/pose/status',
        '/zed/zed_node/pose_with_covariance',
        '/uav1/rangefinder',
        '/uav1/camera_front/compressed',
        '/tf_static',
    ]

    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--rate', rate,
            '--topics', *bag_topics,
        ],
        output='screen',
        sigterm_timeout='2.0',
        sigkill_timeout='2.0',
        on_exit=[Shutdown(reason='Bag playback finished')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('bag_path', default_value=default_bag),
        DeclareLaunchArgument('output_dir', default_value=default_out),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_plot', default_value='true'),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_negate_x', default_value='true'),
        DeclareLaunchArgument('lidar_negate_y', default_value='true'),
        DeclareLaunchArgument('lidar_negate_yaw', default_value='false'),
        DeclareLaunchArgument('w_zed', default_value='0.15'),
        DeclareLaunchArgument('w_lidar', default_value='0.85'),

        _node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_zed',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'zed_camera_link'],
        ),
        _node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                LaunchConfiguration('laser_x'),
                LaunchConfiguration('laser_y'),
                LaunchConfiguration('laser_z'),
                LaunchConfiguration('laser_yaw'),
                '0', '0',
                'base_link', 'laser',
            ],
        ),

        _node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'laser_scan_topic': '/scan',
                'odom_topic': '/lidar/odom_raw',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'lidar_odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
            }],
        ),

        _node(
            package='ekf_bag_tuner',
            executable='odom_republisher',
            name='lidar_odom_republisher',
            parameters=[{
                'use_sim_time': True,
                'cov_scale': 1.0,
                'input_topic': '/lidar/odom_raw',
                'output_topic': '/lidar/odom',
                'output_child_frame': 'base_link',
                'negate_x': ParameterValue(
                    LaunchConfiguration('lidar_negate_x'), value_type=bool),
                'negate_y': ParameterValue(
                    LaunchConfiguration('lidar_negate_y'), value_type=bool),
                'negate_yaw': ParameterValue(
                    LaunchConfiguration('lidar_negate_yaw'), value_type=bool),
            }],
        ),

        _node(
            package='ekf_bag_tuner',
            executable='relative_fuse',
            name='relative_fuse',
            parameters=[{
                'use_sim_time': True,
                'zed_topic': '/zed/zed_node/odom',
                'lidar_topic': '/lidar/odom',
                'output_topic': '/odometry/filtered',
                'odom_frame': 'odom_fused',
                'base_frame': 'base_link',
                'publish_tf': True,
                'w_zed': ParameterValue(
                    LaunchConfiguration('w_zed'), value_type=float),
                'w_lidar': ParameterValue(
                    LaunchConfiguration('w_lidar'), value_type=float),
                'disagreement_threshold_m': 0.08,
                'zed_disagreement_scale': 0.25,
            }],
        ),

        _node(
            package='ekf_bag_tuner',
            executable='trajectory_recorder',
            name='trajectory_recorder',
            parameters=[{
                'use_sim_time': True,
                'output_dir': output_dir,
            }],
        ),

        # Wall-time node (no use_sim_time): plot keeps updating on real clock
        _node(
            package='ekf_bag_tuner',
            executable='live_plotter',
            name='live_plotter',
            condition=IfCondition(use_plot),
            parameters=[{
                'output_dir': output_dir,
                'update_hz': 5.0,
                'save_on_shutdown': True,
            }],
        ),

        bag_play,

        _node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': True}],
        ),
    ])
