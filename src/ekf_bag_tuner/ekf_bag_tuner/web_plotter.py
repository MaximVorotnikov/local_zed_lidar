#!/usr/bin/env python3
"""Serve live ZED/lidar/fused XY trajectories over HTTP for LAN viewing.

Also writes a JSONL log of every plotted point so a run can be replayed later
with the same web UI (as if live).

Live:
  ros2 launch ekf_bag_tuner fuse_realtime.launch.py use_web_plot:=true

Replay:
  ros2 run ekf_bag_tuner web_plotter --ros-args \\
    -p replay_path:=/path/to/run_.../paths.jsonl -p port:=8765
"""

from __future__ import annotations

import json
import socket
import threading
import time
from collections import deque
from datetime import datetime
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

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


def _stamp_sec(msg: Odometry) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


class _Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, node: 'WebPlotter' = None, web_root: Path = None, **kwargs):
        self._node = node
        self._web_root = web_root
        super().__init__(*args, directory=str(web_root), **kwargs)

    def log_message(self, fmt, *args):
        return

    def _send_json(self, obj: Any, code: int = 200):
        data = json.dumps(obj, separators=(',', ':')).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()
        self.wfile.write(data)

    def do_OPTIONS(self):
        self.send_response(204)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/api/paths':
            self._send_json(self._node.snapshot())
            return
        if path == '/api/meta':
            self._send_json(self._node.meta())
            return
        if self.path in ('/', '/index.html'):
            self.path = '/index.html'
        return super().do_GET()

    def do_POST(self):
        path = self.path.split('?', 1)[0]
        if path != '/api/replay':
            self.send_error(404)
            return
        length = int(self.headers.get('Content-Length', '0') or 0)
        raw = self.rfile.read(length) if length else b'{}'
        try:
            body = json.loads(raw.decode('utf-8') or '{}')
        except Exception:
            body = {}
        self._send_json(self._node.replay_control(body))


class WebPlotter(Node):
    def __init__(self):
        super().__init__('web_plotter')
        self.declare_parameter('max_points', 4000)
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8765)
        self.declare_parameter('zed_topic', '/zed/zed_node/odom')
        self.declare_parameter('lidar_topic', '/lidar/odom')
        self.declare_parameter('fused_topic', '/odometry/filtered')
        self.declare_parameter('log_enable', True)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('replay_path', '')
        self.declare_parameter('replay_speed', 1.0)
        self.declare_parameter('replay_loop', False)

        self.max_points = int(self.get_parameter('max_points').value)
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.log_enable = bool(self.get_parameter('log_enable').value)
        self.replay_path = str(self.get_parameter('replay_path').value).strip()
        self.replay_speed = float(self.get_parameter('replay_speed').value)
        self.replay_loop = bool(self.get_parameter('replay_loop').value)

        self._lock = threading.Lock()
        self.paths = {
            'zed': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
            'lidar': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
            'fused': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
        }

        self._log_file = None
        self._log_path: Optional[Path] = None
        self._run_dir: Optional[Path] = None
        self._log_count = 0
        self._t0: Optional[float] = None

        # replay state
        self._events: List[Tuple[float, str, float, float]] = []
        self._replay_idx = 0
        self._replay_playing = True
        self._replay_wall0 = time.monotonic()
        self._replay_t_rel = 0.0
        self._replay_duration = 0.0

        if self.replay_path:
            self._init_replay()
        else:
            self._init_live()

        share = Path(get_package_share_directory('ekf_bag_tuner'))
        web_root = share / 'web'
        if not (web_root / 'index.html').is_file():
            web_root = Path(__file__).resolve().parents[1] / 'web'

        handler = partial(_Handler, node=self, web_root=web_root)
        self._httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

        lan = guess_lan_ip()
        mode = 'replay' if self.replay_path else 'live'
        self.get_logger().info(
            f'Web plot ({mode}): http://{lan}:{self.port}/  '
            f'(also http://127.0.0.1:{self.port}/)'
        )
        if self._log_path:
            self.get_logger().info(f'Logging trajectories to {self._log_path}')

        if self.replay_path:
            self.create_timer(0.05, self._replay_tick)

    def _default_output_dir(self) -> Path:
        raw = str(self.get_parameter('output_dir').value).strip()
        if raw:
            return Path(raw).expanduser()
        return Path.home() / 'local_zed_lidar' / 'ekf_tuning_output'

    def _init_live(self):
        zed_t = self.get_parameter('zed_topic').value
        lidar_t = self.get_parameter('lidar_topic').value
        fused_t = self.get_parameter('fused_topic').value
        self.create_subscription(Odometry, zed_t, lambda m: self._on_odom(m, 'zed'), 50)
        self.create_subscription(Odometry, lidar_t, lambda m: self._on_odom(m, 'lidar'), 50)
        self.create_subscription(Odometry, fused_t, lambda m: self._on_odom(m, 'fused'), 50)

        if self.log_enable:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            self._run_dir = self._default_output_dir() / f'run_{stamp}'
            self._run_dir.mkdir(parents=True, exist_ok=True)
            self._log_path = self._run_dir / 'paths.jsonl'
            self._log_file = open(self._log_path, 'a', buffering=1)
            meta = {
                'type': 'meta',
                'created': stamp,
                'topics': {
                    'zed': zed_t,
                    'lidar': lidar_t,
                    'fused': fused_t,
                },
            }
            self._log_file.write(json.dumps(meta, separators=(',', ':')) + '\n')
            (self._run_dir / 'README.txt').write_text(
                'Replay this run with:\n'
                f'  ros2 run ekf_bag_tuner web_plotter --ros-args '
                f'-p replay_path:={self._log_path} -p port:=8765\n'
            )

    def _init_replay(self):
        path = Path(self.replay_path).expanduser()
        if not path.is_file():
            raise FileNotFoundError(f'replay_path not found: {path}')
        events: List[Tuple[float, str, float, float]] = []
        with path.open('r') as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if obj.get('type') == 'meta' or 'src' not in obj:
                    continue
                t = float(obj['t'])
                src = str(obj['src'])
                if src not in self.paths:
                    continue
                events.append((t, src, float(obj['x']), float(obj['y'])))
        events.sort(key=lambda e: e[0])
        self._events = events
        if events:
            self._t0 = events[0][0]
            self._replay_duration = events[-1][0] - self._t0
        else:
            self._t0 = 0.0
            self._replay_duration = 0.0
        self._replay_idx = 0
        self._replay_playing = True
        self._replay_wall0 = time.monotonic()
        self._replay_t_rel = 0.0
        self.get_logger().info(
            f'Replay loaded {len(events)} points from {path} '
            f'({self._replay_duration:.1f}s)'
        )

    def _append_point(self, src: str, t: float, x: float, y: float, log: bool):
        with self._lock:
            if self._t0 is None:
                self._t0 = t
            self.paths[src]['x'].append(x)
            self.paths[src]['y'].append(y)
            if log and self._log_file is not None:
                row = {
                    't': t,
                    'src': src,
                    'x': x,
                    'y': y,
                    't_rel': t - self._t0,
                }
                self._log_file.write(json.dumps(row, separators=(',', ':')) + '\n')
                self._log_count += 1
                if self._log_count % 50 == 0:
                    self._log_file.flush()

    def _on_odom(self, msg: Odometry, name: str):
        p = msg.pose.pose.position
        t = _stamp_sec(msg)
        if t <= 0.0:
            t = time.time()
        self._append_point(name, t, float(p.x), float(p.y), log=True)

    def _clear_paths(self):
        for name in self.paths:
            self.paths[name]['x'].clear()
            self.paths[name]['y'].clear()

    def _rebuild_paths_until(self, t_rel: float):
        """Rebuild deques from events with relative time <= t_rel."""
        self._clear_paths()
        if self._t0 is None:
            return
        t_abs = self._t0 + t_rel
        idx = 0
        for i, (t, src, x, y) in enumerate(self._events):
            if t > t_abs:
                break
            self.paths[src]['x'].append(x)
            self.paths[src]['y'].append(y)
            idx = i + 1
        self._replay_idx = idx

    def _replay_tick(self):
        if not self._events:
            return
        with self._lock:
            if self._replay_playing:
                elapsed = (time.monotonic() - self._replay_wall0) * max(self.replay_speed, 1e-6)
                self._replay_t_rel = elapsed
                if self._replay_t_rel >= self._replay_duration:
                    if self.replay_loop:
                        self._replay_wall0 = time.monotonic()
                        self._replay_t_rel = 0.0
                        self._clear_paths()
                        self._replay_idx = 0
                    else:
                        self._replay_t_rel = self._replay_duration
                        self._replay_playing = False
            if self._t0 is None:
                return
            t_abs = self._t0 + self._replay_t_rel
            while self._replay_idx < len(self._events):
                t, src, x, y = self._events[self._replay_idx]
                if t > t_abs:
                    break
                self.paths[src]['x'].append(x)
                self.paths[src]['y'].append(y)
                self._replay_idx += 1

    def replay_control(self, body: Dict[str, Any]) -> Dict[str, Any]:
        if not self.replay_path:
            return {'ok': False, 'error': 'not in replay mode'}
        action = str(body.get('action', '')).strip().lower()
        with self._lock:
            if action in ('toggle', 'play_pause'):
                if self._replay_playing:
                    self._replay_playing = False
                    self._replay_t_rel = min(self._replay_t_rel, self._replay_duration)
                else:
                    # resume from current t_rel
                    self._replay_playing = True
                    self._replay_wall0 = time.monotonic() - (
                        self._replay_t_rel / max(self.replay_speed, 1e-6)
                    )
            elif action == 'play':
                self._replay_playing = True
                self._replay_wall0 = time.monotonic() - (
                    self._replay_t_rel / max(self.replay_speed, 1e-6)
                )
            elif action == 'pause':
                self._replay_playing = False
            elif action == 'restart':
                self._clear_paths()
                self._replay_idx = 0
                self._replay_t_rel = 0.0
                self._replay_playing = True
                self._replay_wall0 = time.monotonic()
            elif action == 'seek':
                t = float(body.get('t', 0.0))
                self._replay_t_rel = max(0.0, min(t, self._replay_duration))
                self._rebuild_paths_until(self._replay_t_rel)
                self._replay_wall0 = time.monotonic() - (
                    self._replay_t_rel / max(self.replay_speed, 1e-6)
                )
            elif action == 'set_speed':
                self.replay_speed = max(0.1, float(body.get('speed', self.replay_speed)))
                self._replay_wall0 = time.monotonic() - (
                    self._replay_t_rel / max(self.replay_speed, 1e-6)
                )
            elif action == 'set_loop':
                self.replay_loop = bool(body.get('loop', self.replay_loop))
        return self.meta()

    def meta(self) -> Dict[str, Any]:
        with self._lock:
            return {
                'mode': 'replay' if self.replay_path else 'live',
                'playing': bool(self._replay_playing) if self.replay_path else True,
                't': float(self._replay_t_rel) if self.replay_path else (
                    0.0 if self._t0 is None else max(0.0, time.time() - self._t0)
                ),
                't_end': float(self._replay_duration) if self.replay_path else None,
                'speed': float(self.replay_speed),
                'loop': bool(self.replay_loop),
                'points': int(self._log_count) if not self.replay_path else len(self._events),
                'log_path': str(self._log_path) if self._log_path else (
                    self.replay_path if self.replay_path else ''
                ),
            }

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            out = {
                name: {
                    'x': list(self.paths[name]['x']),
                    'y': list(self.paths[name]['y']),
                }
                for name in self.paths
            }
            out['_meta'] = {
                'mode': 'replay' if self.replay_path else 'live',
                'playing': bool(self._replay_playing) if self.replay_path else True,
                't': float(self._replay_t_rel) if self.replay_path else None,
                't_end': float(self._replay_duration) if self.replay_path else None,
                'speed': float(self.replay_speed),
                'loop': bool(self.replay_loop),
                'log_path': str(self._log_path) if self._log_path else (
                    self.replay_path if self.replay_path else ''
                ),
            }
        return out

    def destroy_node(self):
        try:
            self._httpd.shutdown()
        except Exception:
            pass
        if self._log_file is not None:
            try:
                self._log_file.flush()
                self._log_file.close()
            except Exception:
                pass
            self._log_file = None
            if self._log_path:
                self.get_logger().info(f'Closed trajectory log {self._log_path}')
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
