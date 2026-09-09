#!/usr/bin/env python3
"""Record ZED / lidar odom + fused PoseStamped to CSV."""

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node


def yaw_of(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('output_dir', '/tmp/ekf_tuning_output')
        self.declare_parameter('zed_topic', '/zed/zed_node/odom')
        self.declare_parameter('lidar_topic', '/lidar/odom')
        self.declare_parameter('fused_topic', '/odometry/filtered')

        out = Path(self.get_parameter('output_dir').value)
        out.mkdir(parents=True, exist_ok=True)
        self.files = {}
        self.writers = {}
        for name in ('zed', 'lidar', 'fused'):
            path = out / f'{name}.csv'
            f = open(path, 'w', newline='')
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'yaw'])
            self.files[name] = f
            self.writers[name] = w
            self.get_logger().info(f'Writing {path}')

        zed_t = self.get_parameter('zed_topic').value
        lidar_t = self.get_parameter('lidar_topic').value
        fused_t = self.get_parameter('fused_topic').value
        self.create_subscription(Odometry, zed_t, lambda m: self.on_odom(m, 'zed'), 50)
        self.create_subscription(Odometry, lidar_t, lambda m: self.on_odom(m, 'lidar'), 50)
        self.create_subscription(PoseStamped, fused_t, lambda m: self.on_pose(m, 'fused'), 50)

    def _write(self, name, t, x, y, z, yaw):
        self.writers[name].writerow([f'{t:.6f}', f'{x:.6f}', f'{y:.6f}', f'{z:.6f}', f'{yaw:.6f}'])

    def on_odom(self, msg: Odometry, name: str):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        self._write(name, t, p.x, p.y, p.z, yaw_of(msg.pose.pose.orientation))

    def on_pose(self, msg: PoseStamped, name: str):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.position
        self._write(name, t, p.x, p.y, p.z, yaw_of(msg.pose.orientation))

    def destroy_node(self):
        for f in self.files.values():
            f.flush()
            f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryRecorder()
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
