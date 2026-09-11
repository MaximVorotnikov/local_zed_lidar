# lidar_rf2o_nav

Lidar-only planar odometry for the drone stack. **No ZED / camera fusion.**

Uses `rf2o_laser_odometry` on a (optionally downsampled) `/scan`, then republishes
as `/odometry/filtered` with the same axis convention as the old fuse path:
**X forward, Y left** (`negate_x` / `negate_y` on rf2o raw).

The original `ekf_bag_tuner` ZED+lidar fuse launch is left unchanged.

## Why downsample?

rf2o on full RPLidar scans often takes 80–300+ ms on Jetson, so `/odometry/filtered`
drops to ~8–10 Hz even when `/scan` is 15 Hz. Downsampling cuts compute so rf2o can
keep up closer to the scan rate (target ≈15 Hz, not above scan rate).

## Launch

```bash
cd ~/local_zed_lidar
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lidar_rf2o_nav lidar_only_odom.launch.py
```

Useful args:

- `scan_stride:=2` — keep every N-th beam (default 2; try 3 if still slow)
- `odom_topic:=/odometry/filtered`
- `laser_yaw:=0.0` — same base→laser TF as fuse_realtime
- `negate_x:=true` `negate_y:=true` — X forward, Y left

Stop the old `fuse_realtime.launch.py` first so it does not also publish `/odometry/filtered`.

## Web plot

Same UI as `fuse_realtime` (port **8765** by default):

```text
http://<jetson-ip>:8765
```

Disable with `use_web_plot:=false`.
