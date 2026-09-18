# FASTLIO2 ROS 2 bridge

This workspace includes FASTLIO2_ROS2, `livox_ros_driver2`, Livox-SDK2, and
Sophus 1.22.10 as Git submodules. `standalone.py` publishes:

- `/isaac/lidar_points` (`sensor_msgs/msg/PointCloud2`)
- `/isaac/imu` (`sensor_msgs/msg/Imu`)
- `/clock` (`rosgraph_msgs/msg/Clock`)

FASTLIO2_ROS2 expects `/livox/lidar` as
`livox_ros_driver2/msg/CustomMsg`. The `isaac_fastlio_adapter` node converts
the Isaac Sim point cloud and supplies synthetic `line`, `tag`, `offset_time`,
and reflectivity values. Isaac Sim's Nova Carter RTX point cloud contains XYZ
only, so these Livox-specific fields are approximations for simulation.

The upstream FASTLIO2 fork multiplies incoming standard IMU acceleration by
`10`. The included `imu_scale_adapter` compensates for that behavior and
publishes the corrected input on `/livox/imu`.

Initialize submodules and build the local SDKs and ROS packages:

```bash
sudo apt install ros-humble-gtsam
git submodule update --init --recursive
./ros2_ws/build_workspace.sh
```

Installing `ros-humble-gtsam` system-wide is recommended. If it is not
installed and sudo is unavailable, `build_workspace.sh` automatically
downloads the same Debian package and extracts it under
`ros2_ws/gtsam_install/`.

Run the GUI simulation, adapters, FASTLIO2, its existing PGO backend, and RViz
together:

```bash
./run_slam.sh
```

RViz opens with `lidar` as its fixed frame and displays
`/fastlio2/world_cloud`, the FASTLIO trajectory, and PGO loop-closure markers.
The PGO node selects keyframes from `/fastlio2/body_cloud` and
`/fastlio2/lio_odom`, searches prior poses with a KD-tree, verifies candidates
against a local submap with ICP, and optimizes the pose graph with GTSAM iSAM2. The PGO node still publishes the
`map` to `lidar` correction transform, but RViz keeps `lidar` as its fixed
frame so delayed PGO transforms cannot block the live point cloud.

`build_workspace.sh` applies `patches/fastlio2-pgo-sync.patch` while compiling
the upstream PGO node. The patch uses exact cloud/odometry timestamp matching,
recovers from simulation clock resets, atomically consumes the newest queued
measurement, and logs accepted keyframes and loop closures. The external
FASTLIO2 submodule is restored after the build so it remains clean.

Save the optimized PCD map and optional keyframe patches after mapping:

```bash
./save_map.sh
```

The optional arguments are `./save_map.sh [output_directory] [true|false]`.
The defaults are `maps/office` and `true`.

This writes `map.pcd`, `poses.txt`, and (when requested) a `patches/`
directory. The PGO implementation uses pose-proximity loop candidates, so the
current FASTLIO trajectory must remain within the configured search radius of
the earlier visit before ICP can verify a closure.

Convert the optimized 3D PCD into a Nav2-compatible 2D occupancy map:

```bash
./pcd2pgm.py
```

The defaults read `maps/office/map.pcd` and write
`maps/office/map_2d.pgm` plus `maps/office/map_2d.yaml`. The converter projects
points between 0.1 m and 2.0 m at 0.05 m resolution. For example:

```bash
./pcd2pgm.py maps/office/map.pcd maps/office/map_2d \
  --z-min 0.1 --z-max 2.0 --resolution 0.05
```

Use `--min-points` to reject sparsely populated cells, `--inflation` to widen
obstacles, or `--background unknown` to leave unoccupied cells unknown.
Obstacle inflation is normally better handled by the Nav2 costmap. Because a
merged PCD contains obstacle returns but not the original LiDAR rays, the
converter cannot reconstruct observed free and unknown space exactly; its
default free background matches common PCD-to-PGM tools.

The IMU is colocated with the RTX LiDAR in `standalone.py`, so the supplied
`isaac_lio.yaml` uses identity LiDAR-to-IMU extrinsics. Drive Carter with
W/S/A/D or the arrow keys; press Space to stop. The main Isaac Sim viewport
automatically follows Carter from behind while jogging.

Motion BVH is enabled for RTX sensor motion tracking. The LiDAR publisher
includes native per-point timestamps, intensity, emitter IDs, and channel IDs.
The raw RTX output is explicitly set to `NONCOMPENSATED`, preserving motion
distortion for FASTLIO to remove using the simulated IMU.
The adapter uses the native timestamps for FASTLIO deskew instead of
synthetically distributing points over a scan. XT-32 channel IDs are folded
into FASTLIO's four accepted Livox line IDs, while preserving the original
point order and timing.
