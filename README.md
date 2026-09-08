# local_zed_lidar

Workspace для offline/live настройки фьюжна **ZED2i + 2D lidar** (без GPS).

Не использует `/uav1/local_position/pose` — MAVROS local pose не считается достоверной.

## Packages

| Package | Role |
|---|---|
| `ekf_bag_tuner` | offline/live fuse, launch, CSV/plot |
| `rf2o_laser_odometry` | lidar odometry (live) |
| `robot_localization` | optional EKF (not used by default launch) |
| `zed_msgs` | ZED status messages |

## Build

```bash
cd /home/max/local_zed_lidar
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Offline tune (recommended)

```bash
cd /home/max/local_zed_lidar
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/ekf_bag_tuner/ekf_bag_tuner/offline_fuse.py \
  --bag /home/max/flight_ekf_02 \
  --config src/ekf_bag_tuner/config/tuner.yaml \
  --out /home/max/local_zed_lidar/ekf_tuning_output
```

Outputs:
- `ekf_tuning_output/zed.csv`, `lidar.csv`, `fused.csv`
- `ekf_tuning_output/trajectories.png`
- console `loop_error`

Tune `src/ekf_bag_tuner/config/tuner.yaml`:
- `w_zed` / `w_lidar`
- `disagreement_threshold_m`, `zed_disagreement_scale`
- `flip_lidar_xy` / `invert_lidar_motion` if lidar path mirrors ZED

## Live bag playback + RViz + relative fuse

```bash
cd /home/max/local_zed_lidar
source install/setup.bash

ros2 launch ekf_bag_tuner play_and_fuse.launch.py \
  bag_path:=/home/max/flight_ekf_02 \
  output_dir:=/home/max/local_zed_lidar/ekf_tuning_output \
  use_rviz:=true
```

Появится окно **Live trajectories** (zed / lidar / fused), обновляется ~5 Гц.
В конце bag оно сохранит `trajectories_live.png` в `output_dir`.
Отключить: `use_plot:=false`.

Launch **exits when bag playback ends**. Live fusion (`relative_fuse`) is
**lidar-primary**: SE2-align corrected rf2o to ZED, blend XY (`w_lidar`≫`w_zed`),
take yaw from ZED. `robot_localization` EKF is not used (it was worse than either
sensor alone on this bag).

Plot afterwards:

```bash
ros2 run ekf_bag_tuner analyze_bag -- \
  --dir /home/max/local_zed_lidar/ekf_tuning_output --show
```

If live lidar looks mirrored vs ZED, toggle:
`lidar_negate_x:=true/false`, `lidar_negate_y:=true/false` (defaults true/true for `flight_ekf_02`).

Weights: `w_zed:=0.15` `w_lidar:=0.85` (live rf2o closes loop well — trust lidar more).

## Notes

- Bag `/tf` has no `laser` frame — launch publishes `base_link→laser` (edit `laser_x/y/z/yaw` if needed).
- ZED bag covariance is tiny (~1e-6); offline fuse uses **relative Δpose weights**, not raw absolute cov.
- Live default fuse = lidar-primary SE2 align + light ZED XY pull (`relative_fuse`).
- Height stays on rangefinder → ArduPilot; this workspace fuses **XY only**.
