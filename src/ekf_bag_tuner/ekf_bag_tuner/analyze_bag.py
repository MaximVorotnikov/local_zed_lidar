#!/usr/bin/env python3
"""Plot trajectories from CSV produced by trajectory_recorder or offline_fuse."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_csv(path: Path):
    if not path.exists():
        return None
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return None
    t = np.array([float(r['t']) for r in rows])
    x = np.array([float(r['x']) for r in rows])
    y = np.array([float(r['y']) for r in rows])
    return t, x, y


def metrics(x, y, label: str):
    dx = x[-1] - x[0]
    dy = y[-1] - y[0]
    err = float(np.hypot(dx, dy))
    length = float(np.sum(np.hypot(np.diff(x), np.diff(y))))
    print(f'{label:8s}  start=({x[0]:+.3f},{y[0]:+.3f})  end=({x[-1]:+.3f},{y[-1]:+.3f})  '
          f'loop_error={err:.3f} m  path_len={length:.2f} m')
    return err


def main():
    ap = argparse.ArgumentParser(description='Plot ZED vs fused trajectories from CSV')
    ap.add_argument('--dir', default='/home/max/local_zed_lidar/ekf_tuning_output')
    ap.add_argument('--show', action='store_true')
    args = ap.parse_args()
    d = Path(args.dir)

    fig, ax = plt.subplots(figsize=(8, 8))
    colors = {'zed': '#1f77b4', 'lidar': '#2ca02c', 'fused': '#d62728'}
    for name, color in colors.items():
        data = load_csv(d / f'{name}.csv')
        if data is None:
            continue
        t, x, y = data
        ax.plot(x, y, color=color, linewidth=1.5 if name != 'fused' else 2.2, label=name)
        ax.scatter([x[0]], [y[0]], color=color, marker='o', s=40)
        ax.scatter([x[-1]], [y[-1]], color=color, marker='x', s=60)
        metrics(x, y, name)

    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Trajectories (o=start, x=end)')
    ax.legend()
    out = d / 'trajectories.png'
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    print(f'Saved {out}')
    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
