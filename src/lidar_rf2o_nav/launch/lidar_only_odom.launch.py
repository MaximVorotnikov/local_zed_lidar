#!/usr/bin/env python3
"""Lidar-only odometry (rf2o) → /odometry/filtered. No ZED.

Pipeline:
  /scan → scan_downsampler → /scan_for_odom
  /scan_for_odom → rf2o → /lidar/odom_raw
  /lidar/odom_raw → lidar_odom_publisher → /odometry/filtered (+ TF odom→base_link)

Optional: ekf_bag_tuner web_plotter on :8765 (same UI as fuse_realtime).

Axis convention matches previous fuse defaults: X forward, Y left (negate_x/y).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic')
    scan_for_odom = LaunchConfiguration('scan_for_odom_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_stride = LaunchConfiguration('scan_stride')
    max_beams = LaunchConfiguration('max_beams')
    rf2o_freq = LaunchConfiguration('rf2o_freq')
    use_web_plot = LaunchConfiguration('use_web_plot')
    default_out = os.path.join(os.path.expanduser('~'), 'local_zed_lidar', 'ekf_tuning_output')

    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('scan_for_odom_topic', default_value='/scan_for_odom'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        # Same flips as ekf_bag_tuner fuse_realtime defaults → X forward, Y left.
        DeclareLaunchArgument('negate_x', default_value='true'),
        DeclareLaunchArgument('negate_y', default_value='true'),
        DeclareLaunchArgument('negate_yaw', default_value='false'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        # stride=2 roughly halves rf2o cost; raise to 3 if rate still << 15 Hz.
        DeclareLaunchArgument('scan_stride', default_value='1'),
        DeclareLaunchArgument('max_beams', default_value='0'),
        DeclareLaunchArgument('rf2o_freq', default_value='20.0'),
        DeclareLaunchArgument('use_web_plot', default_value='true'),
        DeclareLaunchArgument('web_plot_port', default_value='8765'),
        DeclareLaunchArgument('output_dir', default_value=default_out),

        LogInfo(msg=['lidar_rf2o_nav: ', scan_topic, ' → rf2o (no ZED) → ', odom_topic]),
        LogInfo(msg=['web plot (if enabled): http://<jetson-ip>:', LaunchConfiguration('web_plot_port')]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=[
                '--x', LaunchConfiguration('laser_x'),
                '--y', LaunchConfiguration('laser_y'),
                '--z', LaunchConfiguration('laser_z'),
                '--yaw', LaunchConfiguration('laser_yaw'),
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', LaunchConfiguration('base_frame'),
                '--child-frame-id', 'laser',
            ],
        ),

        Node(
            package='lidar_rf2o_nav',
            executable='scan_downsampler',
            name='scan_downsampler',
            output='screen',
            parameters=[{
                'input_topic': scan_topic,
                'output_topic': scan_for_odom,
                'stride': ParameterValue(scan_stride, value_type=int),
                'max_beams': ParameterValue(max_beams, value_type=int),
            }],
        ),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'laser_scan_topic': scan_for_odom,
                'odom_topic': '/lidar/odom_raw',
                'publish_tf': False,
                'base_frame_id': LaunchConfiguration('base_frame'),
                'odom_frame_id': 'lidar_odom',
                'init_pose_from_topic': '',
                'freq': ParameterValue(rf2o_freq, value_type=float),
            }],
        ),

        Node(
            package='lidar_rf2o_nav',
            executable='lidar_odom_publisher',
            name='lidar_odom_publisher',
            output='screen',
            parameters=[{
                'input_topic': '/lidar/odom_raw',
                'output_topic': odom_topic,
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_tf'), value_type=bool),
                'negate_x': ParameterValue(
                    LaunchConfiguration('negate_x'), value_type=bool),
                'negate_y': ParameterValue(
                    LaunchConfiguration('negate_y'), value_type=bool),
                'negate_yaw': ParameterValue(
                    LaunchConfiguration('negate_yaw'), value_type=bool),
            }],
        ),

        # Same web UI as fuse_realtime: watch lidar path as "fused" (and raw rf2o as "lidar").
        Node(
            package='ekf_bag_tuner',
            executable='web_plotter',
            name='web_plotter',
            output='screen',
            condition=IfCondition(use_web_plot),
            parameters=[{
                'host': '0.0.0.0',
                'port': ParameterValue(
                    LaunchConfiguration('web_plot_port'), value_type=int),
                # No ZED in this stack — leave topic unused.
                'zed_topic': '/lidar_rf2o_nav/unused_zed_odom',
                'lidar_topic': '/lidar/odom_raw',
                'fused_topic': odom_topic,
                'log_enable': True,
                'output_dir': LaunchConfiguration('output_dir'),
            }],
        ),
    ])
