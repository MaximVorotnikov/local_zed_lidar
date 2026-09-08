#!/usr/bin/env python3
"""Record ZED / lidar / fused odometry to CSV while bag plays."""

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def yaw_of(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('output_dir', '/tmp/ekf_tuning_output')
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

        self.create_subscription(Odometry, '/zed/zed_node/odom', lambda m: self.on_odom(m, 'zed'), 50)
        self.create_subscription(Odometry, '/lidar/odom', lambda m: self.on_odom(m, 'lidar'), 50)
        self.create_subscription(Odometry, '/odometry/filtered', lambda m: self.on_odom(m, 'fused'), 50)

    def on_odom(self, msg: Odometry, name: str):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        yaw = yaw_of(msg.pose.pose.orientation)
        self.writers[name].writerow([f'{t:.6f}', f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}', f'{yaw:.6f}'])

    def destroy_node(self):
        for f in self.files.values():
            f.flush()
            f.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrajectoryRecorder()
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
