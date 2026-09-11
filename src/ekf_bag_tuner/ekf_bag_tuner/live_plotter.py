#!/usr/bin/env python3
"""Live XY plot of ZED / lidar / fused trajectories while bag plays."""

from __future__ import annotations

import os
from collections import deque

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class LivePlotter(Node):
    def __init__(self):
        super().__init__('live_plotter')
        self.declare_parameter('max_points', 8000)
        self.declare_parameter('update_hz', 5.0)
        self.declare_parameter('output_dir', '')
        self.declare_parameter('save_on_shutdown', True)
        self.declare_parameter('margin_m', 0.5)
        self.declare_parameter('min_span_m', 2.0)
        self.declare_parameter('zed_topic', '/zed/zed_node/odom')
        self.declare_parameter('lidar_topic', '/lidar/odom')
        self.declare_parameter('fused_topic', '/odometry/filtered')

        self.max_points = int(self.get_parameter('max_points').value)
        self.out_dir = str(self.get_parameter('output_dir').value).strip()
        self.save_on_shutdown = bool(self.get_parameter('save_on_shutdown').value)
        self.margin = float(self.get_parameter('margin_m').value)
        self.min_span = float(self.get_parameter('min_span_m').value)
        update_hz = float(self.get_parameter('update_hz').value)

        self.paths = {
            'zed': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
            'lidar': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
            'fused': {'x': deque(maxlen=self.max_points), 'y': deque(maxlen=self.max_points)},
        }
        self.dirty = False
        self.follow_data = True  # auto square fit; off after manual zoom/pan
        self._setting_limits = False

        zed_t = self.get_parameter('zed_topic').value
        lidar_t = self.get_parameter('lidar_topic').value
        fused_t = self.get_parameter('fused_topic').value
        self.create_subscription(Odometry, zed_t, lambda m: self.on_odom(m, 'zed'), 50)
        self.create_subscription(Odometry, lidar_t, lambda m: self.on_odom(m, 'lidar'), 50)
        self.create_subscription(Odometry, fused_t, lambda m: self.on_odom(m, 'fused'), 50)

        import matplotlib
        self.headless = not (
            os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')
        )
        if self.headless:
            matplotlib.use('Agg', force=True)
        else:
            for backend in ('TkAgg', 'Qt5Agg', 'QtAgg'):
                try:
                    matplotlib.use(backend, force=True)
                    break
                except Exception:
                    continue
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        self.plt = plt
        if not self.headless:
            plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(9, 8))
        try:
            self.fig.canvas.manager.set_window_title('Live trajectories: ZED / lidar / fused')
        except Exception:
            pass

        # Leave room for toolbar + Fit button
        self.fig.subplots_adjust(bottom=0.12, left=0.10, right=0.98, top=0.92)

        colors = {'zed': '#1f77b4', 'lidar': '#2ca02c', 'fused': '#d62728'}
        self.lines = {}
        self.start_scat = {}
        self.cur_scat = {}
        for name, color in colors.items():
            (line,) = self.ax.plot(
                [], [], color=color,
                linewidth=1.6 if name != 'fused' else 2.4,
                label=name, animated=False,
            )
            self.lines[name] = line
            self.start_scat[name] = self.ax.scatter([], [], color=color, marker='o', s=40, zorder=5)
            self.cur_scat[name] = self.ax.scatter([], [], color=color, marker='x', s=55, zorder=5)

        # box: zoom/pan change the axes window; data stays visible when fitting
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self._update_title()
        self.ax.legend(loc='upper right')
        self.ax.set_xlim(-1.0, 1.0)
        self.ax.set_ylim(-1.0, 1.0)

        if not self.headless:
            # Fit button — restore auto follow
            ax_btn = self.fig.add_axes([0.40, 0.02, 0.20, 0.045])
            self.btn_fit = Button(ax_btn, 'Fit all (A)')
            self.btn_fit.on_clicked(lambda _evt: self.enable_follow())

            self.ax.callbacks.connect('xlim_changed', self._on_limits_changed)
            self.ax.callbacks.connect('ylim_changed', self._on_limits_changed)
            self.fig.canvas.mpl_connect('key_press_event', self._on_key)
            self.fig.canvas.mpl_connect('scroll_event', self._on_scroll)
            plt.show(block=False)

        period = 1.0 / max(update_hz, 0.5)
        self.create_timer(period, self.refresh)
        if self.headless:
            self.get_logger().warn(
                'No DISPLAY — running headless (Agg). Prefer web_plotter; '
                'PNG saved on shutdown if output_dir is set'
            )
        else:
            self.get_logger().info(
                'Live plot ready: toolbar zoom/pan work; press A or "Fit all" to reframe'
            )

    def _update_title(self):
        mode = 'AUTO-FIT' if self.follow_data else 'MANUAL ZOOM'
        self.ax.set_title(f'Live trajectories (o=start, x=current)  [{mode}]')

    def enable_follow(self):
        self.follow_data = True
        self._update_title()
        self.dirty = True
        self.refresh()

    def _on_limits_changed(self, _ax):
        if self._setting_limits:
            return
        # User (or toolbar) changed view → stop fighting them
        if self.follow_data:
            self.follow_data = False
            self._update_title()

    def _on_key(self, event):
        if event.key in ('a', 'A', 'home'):
            self.enable_follow()
        elif event.key in ('f', 'F'):
            # toggle follow
            if self.follow_data:
                self.follow_data = False
                self._update_title()
            else:
                self.enable_follow()

    def _on_scroll(self, event):
        """Mouse-wheel zoom toward cursor (works even if toolbar zoom is awkward)."""
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        self.follow_data = False
        self._update_title()
        scale = 0.8 if event.button == 'up' else 1.25
        x0, x1 = self.ax.get_xlim()
        y0, y1 = self.ax.get_ylim()
        cx, cy = event.xdata, event.ydata

        def zoom_interval(lo, hi, c):
            return c - (c - lo) * scale, c + (hi - c) * scale

        nx0, nx1 = zoom_interval(x0, x1, cx)
        ny0, ny1 = zoom_interval(y0, y1, cy)
        # keep square aspect
        sx = nx1 - nx0
        sy = ny1 - ny0
        side = max(sx, sy)
        ncx = 0.5 * (nx0 + nx1)
        ncy = 0.5 * (ny0 + ny1)
        self._setting_limits = True
        self.ax.set_xlim(ncx - side / 2, ncx + side / 2)
        self.ax.set_ylim(ncy - side / 2, ncy + side / 2)
        self._setting_limits = False
        self.fig.canvas.draw_idle()

    def on_odom(self, msg: Odometry, name: str):
        p = msg.pose.pose.position
        self.paths[name]['x'].append(float(p.x))
        self.paths[name]['y'].append(float(p.y))
        self.dirty = True

    def _fit_square(self, xs_all, ys_all):
        xmin, xmax = min(xs_all), max(xs_all)
        ymin, ymax = min(ys_all), max(ys_all)
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        half = 0.5 * max(xmax - xmin, ymax - ymin, self.min_span) + self.margin
        self._setting_limits = True
        self.ax.set_xlim(cx - half, cx + half)
        self.ax.set_ylim(cy - half, cy + half)
        self._setting_limits = False

    def refresh(self):
        if not self.plt.fignum_exists(self.fig.number):
            return
        if not self.dirty:
            self.plt.pause(0.001)
            return
        self.dirty = False

        import numpy as np
        xs_all, ys_all = [], []
        for name, line in self.lines.items():
            xs = list(self.paths[name]['x'])
            ys = list(self.paths[name]['y'])
            line.set_data(xs, ys)
            if xs:
                self.start_scat[name].set_offsets(np.array([[xs[0], ys[0]]]))
                self.cur_scat[name].set_offsets(np.array([[xs[-1], ys[-1]]]))
                xs_all.extend(xs)
                ys_all.extend(ys)
            else:
                self.start_scat[name].set_offsets(np.empty((0, 2)))
                self.cur_scat[name].set_offsets(np.empty((0, 2)))

        if xs_all and self.follow_data:
            self._fit_square(xs_all, ys_all)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        self.plt.pause(0.001)

    def save_final(self):
        if not self.save_on_shutdown or not self.out_dir:
            return
        try:
            from pathlib import Path
            # Temporarily fit all for the saved image
            xs_all, ys_all = [], []
            for name in self.paths:
                xs_all.extend(self.paths[name]['x'])
                ys_all.extend(self.paths[name]['y'])
            if xs_all:
                self._fit_square(xs_all, ys_all)
            out = Path(self.out_dir)
            out.mkdir(parents=True, exist_ok=True)
            path = out / 'trajectories_live.png'
            self.fig.savefig(path, dpi=150)
            self.get_logger().info(f'Saved {path}')
        except Exception as exc:
            self.get_logger().warn(f'Could not save live plot: {exc}')

    def destroy_node(self):
        try:
            self.dirty = True
            self.follow_data = True
            self.refresh()
            self.save_final()
            self.plt.close(self.fig)
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LivePlotter()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.headless:
                node.plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
