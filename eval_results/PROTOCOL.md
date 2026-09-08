# Protocol: flight_ekf_10 / 11 / 15

Date: 2026-09-08  
Pipeline: live `play_and_fuse` (RF2O → negate x/y → lidar-primary fuse)  
Params: `lidar_negate_x/y:=true`, `w_zed:=0.15`, `w_lidar:=0.85`, `rate:=2.0`  
Bags: `/home/max/local_zed_lidar/flight_bags/`  
Outputs: `/home/max/local_zed_lidar/eval_results/flight_ekf_XX/`

## Summary (loop closure error)

| Bag | Duration | Path ZED | Path lidar | **loop ZED** | **loop lidar** | **loop fused** |
|---|---:|---:|---:|---:|---:|---:|
| flight_ekf_10 | 77 s | 13.3 m | 12.4 m | 0.328 m | 0.202 m | **0.217 m** |
| flight_ekf_11 | 171 s | 37.9 m | 33.7 m | 0.329 m | 0.036 m | **0.049 m** |
| flight_ekf_15 | 125 s | 26.9 m | 24.1 m | 0.334 m | 0.182 m | **0.139 m** |

Notes:
- On all three bags, lidar (and fused) beat ZED on loop closure.
- `negate_x/y=true` still correct (positive Δ correlation with ZED).
- Eval at `rate:=2`; for final check prefer `rate:=1` (rf2o loses fewer scans).

## Per-bag files

Each folder contains:
- `zed.csv`, `lidar.csv`, `fused.csv`
- `trajectories.png`

## Topics present (all bags)

`/scan`, `/zed/zed_node/odom`, `/zed/zed_node/pose*`, `/uav1/rangefinder`, `/uav1/camera_front/compressed`, `/tf`, `/tf_static`, `/uav1/local_position/pose` (unused by fuse).
