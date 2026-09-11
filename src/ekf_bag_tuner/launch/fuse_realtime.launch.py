#!/usr/bin/env python3
"""Realtime ZED + lidar fuse (no bag).

Expects your vehicle stack to already publish:
  /scan                  sensor_msgs/LaserScan
  /zed/zed_node/odom     nav_msgs/Odometry

Publishes fused odometry as nav_msgs/Odometry:
  default: /odometry/filtered
  remount with fused_pose_topic:=... if needed
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('ekf_bag_tuner')
    rviz_cfg = os.path.join(pkg, 'rviz', 'fuse.rviz')
    default_out = os.path.join(os.path.expanduser('~'), 'local_zed_lidar', 'ekf_tuning_output')

    use_rviz = LaunchConfiguration('use_rviz')
    use_plot = LaunchConfiguration('use_plot')
    use_web_plot = LaunchConfiguration('use_web_plot')
    use_recorder = LaunchConfiguration('use_recorder')
    output_dir = LaunchConfiguration('output_dir')
    fused_pose_topic = LaunchConfiguration('fused_pose_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_plot', default_value='false'),
        DeclareLaunchArgument('use_web_plot', default_value='true'),
        DeclareLaunchArgument('web_plot_port', default_value='8765'),
        DeclareLaunchArgument('use_recorder', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=default_out),
        # Remount name of fused Odometry topic if needed
        DeclareLaunchArgument('fused_pose_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('zed_odom_topic', default_value='/zed/zed_node/odom'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        DeclareLaunchArgument('lidar_negate_x', default_value='true'),
        DeclareLaunchArgument('lidar_negate_y', default_value='true'),
        DeclareLaunchArgument('lidar_negate_yaw', default_value='false'),
        DeclareLaunchArgument('w_zed', default_value='0.15'),
        DeclareLaunchArgument('w_lidar', default_value='0.85'),
        DeclareLaunchArgument('publish_tf', default_value='true'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_zed',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'zed_camera_link',
            ],
        ),
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
                '--frame-id', 'base_link',
                '--child-frame-id', 'laser',
            ],
        ),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'laser_scan_topic': scan_topic,
                'odom_topic': '/lidar/odom_raw',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'lidar_odom',
                'init_pose_from_topic': '',
                'freq': 20.0,
            }],
        ),

        Node(
            package='ekf_bag_tuner',
            executable='odom_republisher',
            name='lidar_odom_republisher',
            parameters=[{
                'use_sim_time': False,
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

        Node(
            package='ekf_bag_tuner',
            executable='relative_fuse',
            name='relative_fuse',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'zed_topic': zed_odom_topic,
                'lidar_topic': '/lidar/odom',
                'output_topic': fused_pose_topic,
                'output_frame_id': '',  # inherit ZED odom frame_id
                'odom_frame': 'odom_fused',
                'base_frame': 'base_link',
                'publish_tf': ParameterValue(
                    LaunchConfiguration('publish_tf'), value_type=bool),
                'w_zed': ParameterValue(
                    LaunchConfiguration('w_zed'), value_type=float),
                'w_lidar': ParameterValue(
                    LaunchConfiguration('w_lidar'), value_type=float),
            }],
        ),

        Node(
            package='ekf_bag_tuner',
            executable='trajectory_recorder',
            name='trajectory_recorder',
            condition=IfCondition(use_recorder),
            parameters=[{
                'use_sim_time': False,
                'output_dir': output_dir,
                'zed_topic': zed_odom_topic,
                'lidar_topic': '/lidar/odom',
                'fused_topic': fused_pose_topic,
            }],
        ),

        Node(
            package='ekf_bag_tuner',
            executable='live_plotter',
            name='live_plotter',
            condition=IfCondition(use_plot),
            parameters=[{
                'output_dir': output_dir,
                'update_hz': 5.0,
                'save_on_shutdown': True,
                'zed_topic': zed_odom_topic,
                'lidar_topic': '/lidar/odom',
                'fused_topic': fused_pose_topic,
            }],
        ),

        Node(
            package='ekf_bag_tuner',
            executable='web_plotter',
            name='web_plotter',
            condition=IfCondition(use_web_plot),
            output='screen',
            parameters=[{
                'host': '0.0.0.0',
                'port': ParameterValue(
                    LaunchConfiguration('web_plot_port'), value_type=int),
                'zed_topic': zed_odom_topic,
                'lidar_topic': '/lidar/odom',
                'fused_topic': fused_pose_topic,
                'log_enable': True,
                'output_dir': output_dir,
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': False}],
        ),
    ])
