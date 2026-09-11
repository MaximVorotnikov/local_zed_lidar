#!/usr/bin/env python3
"""Downsample LaserScan beams so rf2o can keep up with /scan rate (~15 Hz)."""

from __future__ import annotations


import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanDownsampler(Node):
    def __init__(self):
        super().__init__('scan_downsampler')
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_for_odom')
        # Keep every N-th beam. 1 = no downsample.
        self.declare_parameter('stride', 2)
        # Optional hard cap on number of ranges after stride.
        self.declare_parameter('max_beams', 0)

        self.stride = max(1, int(self.get_parameter('stride').value))
        self.max_beams = int(self.get_parameter('max_beams').value)

        inn = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self.pub = self.create_publisher(LaserScan, out, qos_profile_sensor_data)
        self.create_subscription(LaserScan, inn, self._cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'Downsample {inn} → {out} (stride={self.stride}, max_beams={self.max_beams or "none"})'
        )

    def _cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        if self.stride == 1 and (self.max_beams <= 0 or n <= self.max_beams):
            self.pub.publish(msg)
            return

        idx = list(range(0, n, self.stride))
        if self.max_beams > 0 and len(idx) > self.max_beams:
            step = len(idx) / float(self.max_beams)
            idx = [idx[int(i * step)] for i in range(self.max_beams)]

        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_min + (idx[-1]) * msg.angle_increment
        out.angle_increment = msg.angle_increment * self.stride
        out.time_increment = msg.time_increment * self.stride
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max
        out.ranges = [msg.ranges[i] for i in idx]
        if msg.intensities and len(msg.intensities) == n:
            out.intensities = [msg.intensities[i] for i in idx]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanDownsampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
