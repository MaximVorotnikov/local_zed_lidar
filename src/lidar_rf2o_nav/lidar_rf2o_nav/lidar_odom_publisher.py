#!/usr/bin/env python3
"""Republish rf2o odom as /odometry/filtered with X-forward / Y-left axes + TF.

Same axis flips as ekf_bag_tuner fuse_realtime defaults (negate_x/y = true).
No ZED blending — lidar only.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_of(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class LidarOdomPublisher(Node):
    def __init__(self):
        super().__init__('lidar_odom_publisher')
        self.declare_parameter('input_topic', '/lidar/odom_raw')
        self.declare_parameter('output_topic', '/odometry/filtered')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # Match previous fuse path: rf2o raw → camera-like X forward, Y left.
        self.declare_parameter('negate_x', True)
        self.declare_parameter('negate_y', True)
        self.declare_parameter('negate_yaw', False)
        self.declare_parameter('rate_log_period_s', 1.0)

        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.negate_x = bool(self.get_parameter('negate_x').value)
        self.negate_y = bool(self.get_parameter('negate_y').value)
        self.negate_yaw = bool(self.get_parameter('negate_yaw').value)
        self.rate_log_period_s = float(self.get_parameter('rate_log_period_s').value)

        inn = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(Odometry, out, 30)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Odometry, inn, self._cb, 30)

        self._n = 0
        self._window_t0 = None

        self.get_logger().info(
            f'Lidar-only odom {inn} → {out} '
            f'(negate_x={self.negate_x}, negate_y={self.negate_y}, negate_yaw={self.negate_yaw}; '
            f'TF {self.odom_frame}→{self.base_frame}={self.publish_tf})'
        )

    def _cb(self, msg: Odometry):
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
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

        # Keep planar.
        out.pose.pose.position.z = 0.0

        self.pub.publish(out)

        if self.tf_br is not None:
            tr = TransformStamped()
            tr.header = out.header
            tr.child_frame_id = self.base_frame
            tr.transform.translation.x = out.pose.pose.position.x
            tr.transform.translation.y = out.pose.pose.position.y
            tr.transform.translation.z = 0.0
            tr.transform.rotation = out.pose.pose.orientation
            self.tf_br.sendTransform(tr)

        now = self.get_clock().now()
        if self._window_t0 is None:
            self._window_t0 = now
            self._n = 0
        self._n += 1
        dt = (now - self._window_t0).nanoseconds * 1e-9
        if dt >= self.rate_log_period_s:
            hz = self._n / dt if dt > 0 else 0.0
            self.get_logger().info(f'{out.header.frame_id} rate: {hz:.1f} Hz ({self._n} msgs / {dt:.2f} s)')
            self._window_t0 = now
            self._n = 0


def main():
    rclpy.init()
    node = LidarOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
