# ekf_bag_tuner

Offline / bag-based tuning of **ZED2i + 2D lidar** fusion.

**Does not use** `/uav1/local_position/pose` (MAVROS local pose is not trusted).

Full workspace docs: `/home/max/local_zed_lidar/README.md`

## Quick start

```bash
cd /home/max/local_zed_lidar
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 src/ekf_bag_tuner/ekf_bag_tuner/offline_fuse.py \
  --bag /home/max/flight_ekf_02 \
  --config src/ekf_bag_tuner/config/tuner.yaml \
  --out /home/max/local_zed_lidar/ekf_tuning_output
```

Tune weights in `config/tuner.yaml`, then re-run.

Live playback:

```bash
ros2 launch ekf_bag_tuner play_and_fuse.launch.py \
  bag_path:=/home/max/flight_ekf_02 \
  output_dir:=/home/max/local_zed_lidar/ekf_tuning_output
```
