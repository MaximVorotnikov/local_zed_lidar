#!/usr/bin/env python3
"""
Offline ZED + lidar fusion from a rosbag (no live ROS graph required).

Pipeline:
  1) Read /scan and /zed/zed_node/odom from bag
  2) Build lidar odometry via consecutive 2D ICP
  3) Fuse ZED + lidar (weighted relative motion)
  4) Write CSV + trajectory plot

Tune weights in config/tuner.yaml (or --config).
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


def yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def scan_to_xy(ranges, angle_min, angle_inc, range_min, range_max):
    pts = []
    for i, r in enumerate(ranges):
        if r is None or not math.isfinite(r):
            continue
        if r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_inc
        pts.append((r * math.cos(a), r * math.sin(a)))
    if not pts:
        return np.zeros((0, 2))
    return np.asarray(pts, dtype=np.float64)


def icp_2d(src: np.ndarray, dst: np.ndarray, max_iter: int = 20, tol: float = 1e-5):
    """Estimate T that maps src -> dst. Returns dx, dy, dyaw, fitness (mean residual)."""
    if len(src) < 20 or len(dst) < 20:
        return 0.0, 0.0, 0.0, 1e9

    # Subsample for speed
    if len(src) > 400:
        src = src[:: max(1, len(src) // 400)]
    if len(dst) > 400:
        dst = dst[:: max(1, len(dst) // 400)]

    x = src.copy()
    R = np.eye(2)
    t = np.zeros(2)
    prev_err = 1e9

    for _ in range(max_iter):
        # nearest neighbor in dst
        d2 = ((x[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
        idx = d2.argmin(axis=1)
        matched = dst[idx]
        residuals = np.linalg.norm(x - matched, axis=1)
        # reject outliers
        med = np.median(residuals)
        keep = residuals < max(0.3, 3.0 * med)
        if keep.sum() < 15:
            break
        a = x[keep]
        b = matched[keep]
        mu_a = a.mean(axis=0)
        mu_b = b.mean(axis=0)
        aa = a - mu_a
        bb = b - mu_b
        H = aa.T @ bb
        U, _, Vt = np.linalg.svd(H)
        Ri = Vt.T @ U.T
        if np.linalg.det(Ri) < 0:
            Vt[-1, :] *= -1
            Ri = Vt.T @ U.T
        ti = mu_b - Ri @ mu_a
        x = (Ri @ x.T).T + ti
        R = Ri @ R
        t = Ri @ t + ti
        err = residuals[keep].mean()
        if abs(prev_err - err) < tol:
            break
        prev_err = err

    dyaw = math.atan2(R[1, 0], R[0, 0])
    return float(t[0]), float(t[1]), float(dyaw), float(prev_err)


@dataclass
class Pose2D:
    t: float
    x: float
    y: float
    yaw: float


class EKF2D:
    def __init__(self, q_xy: float, q_yaw: float):
        self.x = np.zeros(3)
        self.P = np.eye(3) * 0.1
        self.q_xy = q_xy
        self.q_yaw = q_yaw
        self.initialized = False
        self.last_t = None

    def predict(self, t: float):
        if self.last_t is None:
            self.last_t = t
            return
        dt = max(0.0, t - self.last_t)
        self.last_t = t
        Q = np.diag([self.q_xy * dt, self.q_xy * dt, self.q_yaw * dt])
        self.P = self.P + Q

    def update_abs(self, z, Rdiag):
        """Absolute measurement z=[x,y,yaw]."""
        if not self.initialized:
            self.x[:] = z
            self.P = np.diag(Rdiag)
            self.initialized = True
            return
        H = np.eye(3)
        R = np.diag(Rdiag)
        y = z - self.x
        y[2] = wrap(y[2])
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.x[2] = wrap(self.x[2])
        self.P = (np.eye(3) - K @ H) @ self.P

    def update_delta(self, dx, dy, dyaw, Rdiag):
        """Relative motion measurement in body/world mix: apply as absolute after composing."""
        c, s = math.cos(self.x[2]), math.sin(self.x[2])
        # delta in body frame of previous estimate -> world
        wx = c * dx - s * dy
        wy = s * dx + c * dy
        z = np.array([self.x[0] + wx, self.x[1] + wy, wrap(self.x[2] + dyaw)])
        # For relative updates use slightly looser R
        self.update_abs(z, Rdiag)


def load_bag(bag_path: Path):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_path), storage_id='sqlite3'),
        ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr'),
    )
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    scans = []
    zed = []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in type_map:
            continue
        if topic == '/scan':
            msg = deserialize_message(data, get_message(type_map[topic]))
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            pts = scan_to_xy(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, min(msg.range_max, 12.0))
            scans.append((t, pts))
        elif topic == '/zed/zed_node/odom':
            msg = deserialize_message(data, get_message(type_map[topic]))
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            cov = msg.pose.covariance
            zed.append(Pose2D(t, p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w)))
            zed[-1].cov_xy = max(cov[0], cov[7])  # type: ignore
            zed[-1].cov_yaw = cov[35]  # type: ignore

    return scans, zed


def build_lidar_odom(scans, invert_motion: bool = True, flip_xy: bool = False):
    poses = []
    if not scans:
        return poses
    x = y = yaw = 0.0
    t0, prev = scans[0]
    poses.append(Pose2D(t0, x, y, yaw))
    for t, pts in scans[1:]:
        dx, dy, dyaw, fitness = icp_2d(prev, pts)
        if fitness > 0.25:
            poses.append(Pose2D(t, x, y, yaw))
            prev = pts
            continue
        if invert_motion:
            c, s = math.cos(dyaw), math.sin(dyaw)
            mx = -(c * dx + s * dy)
            my = -(-s * dx + c * dy)
            myaw = -dyaw
        else:
            mx, my, myaw = dx, dy, dyaw
        if flip_xy:
            mx, my = -mx, -my
            myaw = -myaw
        cw, sw = math.cos(yaw), math.sin(yaw)
        x += cw * mx - sw * my
        y += sw * mx + cw * my
        yaw = wrap(yaw + myaw)
        poses.append(Pose2D(t, x, y, yaw))
        prev = pts
    return poses


def write_csv(path: Path, poses):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'x', 'y', 'z', 'yaw'])
        for p in poses:
            w.writerow([f'{p.t:.6f}', f'{p.x:.6f}', f'{p.y:.6f}', '0.0', f'{p.yaw:.6f}'])


def _body_delta(prev: Pose2D, cur: Pose2D):
    """World-frame pose pair -> motion in previous body frame."""
    c, s = math.cos(prev.yaw), math.sin(prev.yaw)
    dxw = cur.x - prev.x
    dyw = cur.y - prev.y
    dxb = c * dxw + s * dyw
    dyb = -s * dxw + c * dyw
    dyaw = wrap(cur.yaw - prev.yaw)
    return dxb, dyb, dyaw


def fuse(zed, lidar, cfg):
    """
    Fuse relative motions. Absolute ZED pose alone would dominate because bag
    covariances are ~1e-6; weighted deltas make lidar actually matter.
    """
    if not zed:
        return []

    fused = [Pose2D(zed[0].t, zed[0].x, zed[0].y, zed[0].yaw)]
    x, y, yaw = zed[0].x, zed[0].y, zed[0].yaw
    li = 0
    w_zed0 = float(cfg.get('w_zed', 0.35))
    w_lidar0 = float(cfg.get('w_lidar', 0.65))
    thr = float(cfg.get('disagreement_threshold_m', 0.08))
    zed_down = float(cfg.get('zed_disagreement_scale', 0.25))
    alpha = float(cfg.get('smooth_alpha', 0.0))
    use_zed = cfg.get('use_zed', True)
    use_lidar = cfg.get('use_lidar', True)

    for i in range(1, len(zed)):
        z_prev, z = zed[i - 1], zed[i]
        while li + 1 < len(lidar) and lidar[li + 1].t <= z.t:
            li += 1

        dx_z, dy_z, dyaw_z = _body_delta(z_prev, z)

        dx_l = dy_l = dyaw_l = 0.0
        have_lidar = False
        if use_lidar and lidar and li > 0:
            # lidar poses nearest to z_prev.t and z.t
            lj = li
            while lj > 0 and lidar[lj].t > z_prev.t:
                lj -= 1
            if lidar[lj].t <= z_prev.t and lidar[li].t >= z_prev.t:
                dx_l, dy_l, dyaw_l = _body_delta(lidar[lj], lidar[li])
                have_lidar = True

        w_z = w_zed0 if use_zed else 0.0
        w_l = w_lidar0 if have_lidar else 0.0
        if have_lidar and use_zed:
            disagree = math.hypot(dx_z - dx_l, dy_z - dy_l)
            if disagree > thr:
                w_z *= zed_down

        if w_z + w_l < 1e-9:
            w_z = 1.0
        s = w_z + w_l
        w_z, w_l = w_z / s, w_l / s

        dx = w_z * dx_z + w_l * dx_l
        dy = w_z * dy_z + w_l * dy_l
        dyaw = w_z * dyaw_z + w_l * dyaw_l

        cw, sw = math.cos(yaw), math.sin(yaw)
        x += cw * dx - sw * dy
        y += sw * dx + cw * dy
        yaw = wrap(yaw + dyaw)

        if alpha > 0.0 and fused:
            x = (1 - alpha) * x + alpha * fused[-1].x
            y = (1 - alpha) * y + alpha * fused[-1].y

        fused.append(Pose2D(z.t, x, y, yaw))

    return fused


def align_path_to(ref: list[Pose2D], other: list[Pose2D]):
    """Rigid SE2 align other start to ref start (for fair loop-error compare)."""
    if not ref or not other:
        return other
    # already absolute in different frames — just shift so starts coincide
    dx = ref[0].x - other[0].x
    dy = ref[0].y - other[0].y
    dyaw = wrap(ref[0].yaw - other[0].yaw)
    c, s = math.cos(dyaw), math.sin(dyaw)
    out = []
    for p in other:
        x = c * p.x - s * p.y + dx
        y = s * p.x + c * p.y + dy
        out.append(Pose2D(p.t, x, y, wrap(p.yaw + dyaw)))
    # better: rotate around origin after translating start to 0
    # redo properly: move other start to 0, rotate, move to ref start
    out = []
    ox0, oy0, oyaw0 = other[0].x, other[0].y, other[0].yaw
    rx0, ry0, ryaw0 = ref[0].x, ref[0].y, ref[0].yaw
    dth = wrap(ryaw0 - oyaw0)
    c, s = math.cos(dth), math.sin(dth)
    for p in other:
        x = p.x - ox0
        y = p.y - oy0
        xr = c * x - s * y + rx0
        yr = s * x + c * y + ry0
        out.append(Pose2D(p.t, xr, yr, wrap(p.yaw + dth)))
    return out


def plot_all(out_dir: Path, series: dict):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 8))
    colors = {'zed': '#1f77b4', 'lidar': '#2ca02c', 'fused': '#d62728'}
    for name, poses in series.items():
        if not poses:
            continue
        x = [p.x for p in poses]
        y = [p.y for p in poses]
        ax.plot(x, y, color=colors.get(name, 'k'), lw=2 if name == 'fused' else 1.4, label=name)
        ax.scatter([x[0]], [y[0]], color=colors.get(name, 'k'), marker='o')
        ax.scatter([x[-1]], [y[-1]], color=colors.get(name, 'k'), marker='x', s=60)
        err = math.hypot(x[-1] - x[0], y[-1] - y[0])
        length = sum(math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]) for i in range(1, len(x)))
        print(f'{name:8s}  end=({x[-1]:+.3f},{y[-1]:+.3f})  loop_error={err:.3f} m  path_len={length:.2f} m')
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title('Offline fuse: ZED vs lidar vs fused (o=start x=end)')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    fig.tight_layout()
    png = out_dir / 'trajectories.png'
    fig.savefig(png, dpi=150)
    print(f'Saved {png}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', default='/home/max/flight_ekf_02')
    ap.add_argument('--config', default=None)
    ap.add_argument('--out', default='/home/max/local_zed_lidar/ekf_tuning_output')
    args = ap.parse_args()

    cfg_path = args.config
    if cfg_path is None:
        # try source tree
        cand = Path(__file__).resolve().parents[1] / 'config' / 'tuner.yaml'
        cfg_path = str(cand)
    with open(cfg_path) as f:
        full = yaml.safe_load(f)
    fusion = full['fusion']

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f'Reading bag: {args.bag}')
    scans, zed = load_bag(Path(args.bag))
    print(f'scans={len(scans)} zed={len(zed)}')

    print('Building lidar odometry (ICP)...')
    lidar = build_lidar_odom(
        scans,
        invert_motion=bool(fusion.get('invert_lidar_motion', True)),
        flip_xy=bool(fusion.get('flip_lidar_xy', False)),
    )
    # Align lidar trajectory start to ZED start for visualization
    lidar_aligned = align_path_to(zed, lidar) if zed else lidar

    print('Fusing...')
    fused = fuse(zed, lidar, fusion)

    write_csv(out_dir / 'zed.csv', zed)
    write_csv(out_dir / 'lidar.csv', lidar_aligned)
    write_csv(out_dir / 'fused.csv', fused)

    plot_all(out_dir, {'zed': zed, 'lidar': lidar_aligned, 'fused': fused})
    print(f'CSV written to {out_dir}')
    print('Tune fusion weights in config/tuner.yaml and re-run.')


if __name__ == '__main__':
    main()
