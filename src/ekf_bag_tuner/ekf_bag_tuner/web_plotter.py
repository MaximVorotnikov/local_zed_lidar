#!/usr/bin/env python3
"""Serve live ZED/lidar/fused XY trajectories over HTTP for LAN viewing.

Bind 0.0.0.0 so phones/laptops on the same Wi-Fi can open the page.
"""

from __future__ import annotations

import json
import socket
import threading
from collections import deque
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node


def guess_lan_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.3)
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception:
            return '127.0.0.1'


class _Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, node: 'WebPlotter' = None, web_root: Path = None, **kwargs):
        self._node = node
        self._web_root = web_root
        super().__init__(*args, directory=str(web_root), **kwargs)

    def log_message(self, fmt, *args):
        # Quiet default access log; node logger prints URL once.
        return

    def do_GET(self):
        if self.path.split('?', 1)[0] == '/api/paths':
            payload = self._node.snapshot_json()
            data = payload.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
            return
        if self.path in ('/', '/index.html'):
            self.path = '/index.html'
        return super().do_GET()


class WebPlotter(Node):
    def __init__(self):
        super().__init__('web_plotter')
        self.declare_parameter('max_points', 4000)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('zed_topic', '/zed/zed_node/odom')
        self.declare_parameter('lidar_topic', '/lidar/odom')
        self.declare_parameter('fused_topic', '/odometry/filtered')

        self.max_points = int(self.get_parameter('max_points').value)
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)

        self._lock = threading.Lock()
        self.paths = {
            'zed': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
            'lidar': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
            'fused': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
        }

        zed_t = self.get_parameter('zed_topic').value
        lidar_t = self.get_parameter('lidar_topic').value
        fused_t = self.get_parameter('fused_topic').value
        self.create_subscription(Odometry, zed_t, lambda m: self._on_odom(m, 'zed'), 50)
        self.create_subscription(Odometry, lidar_t, lambda m: self._on_odom(m, 'lidar'), 50)
        self.create_subscription(Odometry, fused_t, lambda m: self._on_odom(m, 'fused'), 50)

        share = Path(get_package_share_directory('ekf_bag_tuner'))
        web_root = share / 'web'
        if not (web_root / 'index.html').is_file():
            # symlink-install / source tree fallback
            web_root = Path(__file__).resolve().parents[1] / 'web'

        handler = partial(_Handler, node=self, web_root=web_root)
        self._httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

        lan = guess_lan_ip()
        self.get_logger().info(
            f'Web plot: http://{lan}:{self.port}/  (also http://127.0.0.1:{self.port}/)'
        )

    def _on_odom(self, msg: Odometry, name: str):
        p = msg.pose.pose.position
        with self._lock:
            self.paths[name]['x'].append(float(p.x))
            self.paths[name]['y'].append(float(p.y))

    def snapshot_json(self) -> str:
        with self._lock:
            out = {
                name: {'x': list(self.paths[name]['x']), 'y': list(self.paths[name]['y'])}
                for name in self.paths
            }
        return json.dumps(out, separators=(',', ':'))

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = WebPlotter()
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
