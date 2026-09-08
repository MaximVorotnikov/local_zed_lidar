#!/usr/bin/env python3
"""Republish odometry with optional child-frame rewrite, cov scale, and axis flips.

Used for:
  - ZED: rewrite child→base_link, inflate covariance so EKF hears lidar
  - lidar/rf2o: negate X/Y when rf2o frame is mirrored vs ZED (flight_ekf_02)
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_republisher')
        self.declare_parameter('input_topic', '/zed/zed_node/odom')
        self.declare_parameter('output_topic', '/zed/odom_for_ekf')
        self.declare_parameter('output_child_frame', 'base_link')
        self.declare_parameter('cov_scale', 1.0)
        self.declare_parameter('min_xy_var', 0.0)
        self.declare_parameter('min_yaw_var', 0.0)
        self.declare_parameter('negate_x', False)
        self.declare_parameter('negate_y', False)
        self.declare_parameter('negate_yaw', False)

        self.cov_scale = float(self.get_parameter('cov_scale').value)
        self.min_xy = float(self.get_parameter('min_xy_var').value)
        self.min_yaw = float(self.get_parameter('min_yaw_var').value)
        self.negate_x = bool(self.get_parameter('negate_x').value)
        self.negate_y = bool(self.get_parameter('negate_y').value)
        self.negate_yaw = bool(self.get_parameter('negate_yaw').value)
        self.child = self.get_parameter('output_child_frame').value

        inn = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(Odometry, out, 20)
        self.sub = self.create_subscription(Odometry, inn, self.cb, 20)
        self.get_logger().info(
            f'Republishing {inn} -> {out}, child={self.child}, '
            f'cov_scale={self.cov_scale}, negate_x={self.negate_x}, '
            f'negate_y={self.negate_y}, negate_yaw={self.negate_yaw}'
        )

    def cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.child_frame_id = self.child if self.child else msg.child_frame_id
        out.pose = msg.pose
        out.twist = msg.twist

        if self.negate_x:
            out.pose.pose.position.x *= -1.0
            out.twist.twist.linear.x *= -1.0
        if self.negate_y:
            out.pose.pose.position.y *= -1.0
            out.twist.twist.linear.y *= -1.0
        if self.negate_yaw:
            yaw = -yaw_of(out.pose.pose.orientation)
            out.pose.pose.orientation = quat_from_yaw(yaw)
            out.twist.twist.angular.z *= -1.0

        if self.cov_scale != 1.0 or self.min_xy > 0.0 or self.min_yaw > 0.0:
            cov = list(out.pose.covariance)
            for i in (0, 7, 14):
                cov[i] = max(cov[i] * self.cov_scale, self.min_xy)
            for i in (21, 28, 35):
                cov[i] = max(cov[i] * self.cov_scale, self.min_yaw)
            out.pose.covariance = cov

        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomRepublisher()
    try:
        # Wall-time spin_once so shutdown works after /clock stops with the bag.
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
