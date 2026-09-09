#!/usr/bin/env python3
"""Realtime ZED + lidar fuse (no bag).

Expects your vehicle stack to already publish:
  /scan                  sensor_msgs/LaserScan
  /zed/zed_node/odom     nav_msgs/Odometry

Publishes fused pose as geometry_msgs/PoseStamped (same type as /zed/zed_node/pose):
  default: /odometry/filtered
  for FC drop-in: fused_pose_topic:=/zed/zed_node/pose
  (then disable / remount the original ZED pose publisher to avoid two pubs)
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
    default_out = '/home/max/local_zed_lidar/ekf_tuning_output'

    use_rviz = LaunchConfiguration('use_rviz')
    use_plot = LaunchConfiguration('use_plot')
    use_recorder = LaunchConfiguration('use_recorder')
    output_dir = LaunchConfiguration('output_dir')
    fused_pose_topic = LaunchConfiguration('fused_pose_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    zed_odom_topic = LaunchConfiguration('zed_odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('use_plot', default_value='true'),
        DeclareLaunchArgument('use_recorder', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=default_out),
        # Remount to /zed/zed_node/pose when wiring into the flight stack
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
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'zed_camera_link'],
        ),
        Node(
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
            parameters=[{'use_sim_time': False}],
        ),
    ])
