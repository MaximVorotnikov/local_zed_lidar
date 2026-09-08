#!/usr/bin/env python3
"""Live ZED + lidar fusion.

Why not robot_localization EKF:
  Two differential odoms with different header frames (odom vs lidar_odom) and
  no connecting TF produced worse loop error than either sensor alone.

Why not naive Δ-blend:
  rf2o yaw correlates poorly with ZED (~0.1), so body/world blends scramble XY.

What works on flight_ekf_02 (after lidar XY negate):
  Corrected rf2o closes the loop to ~0.03 m. Use SE2-aligned lidar for XY and
  ZED for yaw. Optional small pull toward ZED XY via w_zed (default 0.15).
"""

from __future__ import annotations

import math
from collections import deque

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_of(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class RelativeFuse(Node):
    def __init__(self):
        super().__init__('relative_fuse')
        self.declare_parameter('zed_topic', '/zed/zed_node/odom')
        self.declare_parameter('lidar_topic', '/lidar/odom')
        self.declare_parameter('output_topic', '/odometry/filtered')
        self.declare_parameter('odom_frame', 'odom_fused')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # w_zed: pull fused XY toward ZED after lidar is SE2-aligned (0 = pure lidar XY)
        self.declare_parameter('w_zed', 0.15)
        self.declare_parameter('w_lidar', 0.85)
        self.declare_parameter('lidar_buffer_sec', 2.0)

        self.w_zed = float(self.get_parameter('w_zed').value)
        self.w_lidar = float(self.get_parameter('w_lidar').value)
        s = self.w_zed + self.w_lidar
        if s > 1e-9:
            self.w_zed /= s
            self.w_lidar /= s
        self.buf_sec = float(self.get_parameter('lidar_buffer_sec').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.zed = None  # (t, x, y, yaw)
        self.lidar_buf = deque()
        # SE2 that maps lidar frame → ZED frame (fixed after first pair)
        self.aligned = False
        self.dth = 0.0
        self.tx = 0.0
        self.ty = 0.0

        out = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(Odometry, out, 30)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        zed_t = self.get_parameter('zed_topic').value
        lidar_t = self.get_parameter('lidar_topic').value
        self.create_subscription(Odometry, zed_t, self.on_zed, 30)
        self.create_subscription(Odometry, lidar_t, self.on_lidar, 30)
        self.get_logger().info(
            f'Lidar-primary fuse {zed_t} + {lidar_t} -> {out} '
            f'(w_zed={self.w_zed:.2f}, w_lidar={self.w_lidar:.2f})'
        )

    def on_lidar(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        self.lidar_buf.append((t, p.x, p.y, yaw))
        t_cut = t - self.buf_sec
        while len(self.lidar_buf) > 1 and self.lidar_buf[0][0] < t_cut:
            self.lidar_buf.popleft()
        # Publish on lidar rate — primary XY source
        self._try_publish(msg.header.stamp)

    def on_zed(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self.zed = (t, p.x, p.y, yaw_of(msg.pose.pose.orientation))
        # Do not publish here — avoids double-rate CSV / marker spam

    def _lidar_latest(self):
        return self.lidar_buf[-1] if self.lidar_buf else None

    def _map_lidar(self, lx, ly, lyaw):
        c, s = math.cos(self.dth), math.sin(self.dth)
        return (
            c * lx - s * ly + self.tx,
            s * lx + c * ly + self.ty,
            wrap(lyaw + self.dth),
        )

    def _try_publish(self, stamp):
        if self.zed is None:
            return
        lid = self._lidar_latest()
        if lid is None:
            return

        _, zx, zy, zyaw = self.zed
        _, lx, ly, lyaw = lid

        if not self.aligned:
            self.dth = wrap(zyaw - lyaw)
            c, s = math.cos(self.dth), math.sin(self.dth)
            self.tx = zx - (c * lx - s * ly)
            self.ty = zy - (s * lx + c * ly)
            self.aligned = True
            self.get_logger().info(
                f'SE2-aligned lidar→ZED: dth={self.dth:.3f} rad, t=({self.tx:.3f},{self.ty:.3f})'
            )

        mx, my, _ = self._map_lidar(lx, ly, lyaw)
        # Blend XY in the shared ZED-aligned frame; heading from ZED
        x = self.w_lidar * mx + self.w_zed * zx
        y = self.w_lidar * my + self.w_zed * zy
        yaw = zyaw
        self._publish(stamp, x, y, yaw)

    def _publish(self, stamp, x, y, yaw):
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.orientation = quat_from_yaw(yaw)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[35] = 0.02
        out.pose.covariance = cov
        self.pub.publish(out)

        if self.tf_br is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation = out.pose.pose.orientation
            self.tf_br.sendTransform(tf)


def main():
    rclpy.init()
    node = RelativeFuse()
    try:
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
