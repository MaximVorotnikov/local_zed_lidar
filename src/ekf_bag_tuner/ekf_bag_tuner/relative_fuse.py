#!/usr/bin/env python3
"""Live ZED + lidar fusion → nav_msgs/Odometry.

No first-frame SE2 alignment. Poses are blended in place:

  x = w_lidar * x_lidar + w_zed * x_zed
  y = w_lidar * y_lidar + w_zed * y_zed
  yaw = slerp-ish shortest-arc blend

So w_lidar:=1 w_zed:=0 → fused XY/yaw are exactly the lidar odom
(after odom_republisher axis flips). Z (height) from ZED when available.

Frame id prefers ZED's odom frame (camera world: X forward, Y left).
Lidar axis convention vs camera is handled by lidar_negate_* in launch,
not by a runtime pose align.
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
        self.declare_parameter('output_frame_id', '')  # empty → ZED frame_id
        self.declare_parameter('odom_frame', 'odom_fused')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
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
        self.output_frame_id_param = str(self.get_parameter('output_frame_id').value).strip()
        self.odom_frame_fallback = str(self.get_parameter('odom_frame').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)

        self.zed = None  # (t, x, y, z, yaw)
        self.zed_frame_id = ''
        self.lidar_frame_id = ''
        self.lidar_buf = deque()

        out = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(Odometry, out, 30)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        zed_t = self.get_parameter('zed_topic').value
        lidar_t = self.get_parameter('lidar_topic').value
        self.create_subscription(Odometry, zed_t, self.on_zed, 30)
        self.create_subscription(Odometry, lidar_t, self.on_lidar, 30)
        self.get_logger().info(
            f'Fuse {zed_t} + {lidar_t} -> {out} '
            f'(w_zed={self.w_zed:.3f}, w_lidar={self.w_lidar:.3f}; '
            f'NO SE2 align; w_lidar=1 => fused==lidar XY/yaw)'
        )

    def _frame_id(self) -> str:
        if self.output_frame_id_param:
            return self.output_frame_id_param
        if self.zed_frame_id:
            return self.zed_frame_id
        if self.lidar_frame_id:
            return self.lidar_frame_id
        return self.odom_frame_fallback

    def on_lidar(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        if msg.header.frame_id:
            self.lidar_frame_id = msg.header.frame_id
        self.lidar_buf.append((t, p.x, p.y, yaw))
        t_cut = t - self.buf_sec
        while len(self.lidar_buf) > 1 and self.lidar_buf[0][0] < t_cut:
            self.lidar_buf.popleft()
        self._try_publish(msg.header.stamp)

    def on_zed(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        if msg.header.frame_id:
            self.zed_frame_id = msg.header.frame_id
        self.zed = (t, p.x, p.y, p.z, yaw_of(msg.pose.pose.orientation))

    def _lidar_latest(self):
        return self.lidar_buf[-1] if self.lidar_buf else None

    def _try_publish(self, stamp):
        lid = self._lidar_latest()
        if lid is None:
            return

        # Pure lidar: do not wait for / require ZED pose.
        if self.w_zed <= 1e-9:
            _, lx, ly, lyaw = lid
            z = self.zed[3] if self.zed is not None else 0.0
            self._publish(stamp, lx, ly, z, lyaw)
            return

        if self.zed is None:
            return

        _, zx, zy, zz, zyaw = self.zed
        _, lx, ly, lyaw = lid

        x = self.w_lidar * lx + self.w_zed * zx
        y = self.w_lidar * ly + self.w_zed * zy
        yaw = wrap(zyaw + self.w_lidar * wrap(lyaw - zyaw))
        z = zz

        self._publish(stamp, x, y, z, yaw)

    def _publish(self, stamp, x, y, z, yaw):
        q = quat_from_yaw(yaw)
        frame = self._frame_id()
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = x
        out.pose.pose.position.y = y
        out.pose.pose.position.z = z
        out.pose.pose.orientation = q
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05
        cov[35] = 0.02
        out.pose.covariance = cov
        self.pub.publish(out)

        if self.tf_br is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation = q
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
