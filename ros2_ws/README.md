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
git submodule update --init --recursive
./ros2_ws/build_workspace.sh
```

Run the GUI simulation, adapters, FASTLIO2, and RViz together:

```bash
./run_slam.sh
```

RViz opens with `lidar` as its fixed frame and displays
`/fastlio2/world_cloud`. Its 10-second decay time keeps recent registered
scans visible for easier inspection; this visual history is not a saved map.

The IMU is colocated with the RTX LiDAR in `standalone.py`, so the supplied
`isaac_lio.yaml` uses identity LiDAR-to-IMU extrinsics. Drive Carter with
W/S/A/D or the arrow keys; press Space to stop.

Motion BVH is enabled for RTX sensor motion tracking. The LiDAR publisher
includes native per-point timestamps, intensity, emitter IDs, and channel IDs.
The raw RTX output is explicitly set to `NONCOMPENSATED`, preserving motion
distortion for FASTLIO to remove using the simulated IMU.
The adapter uses the native timestamps for FASTLIO deskew instead of
synthetically distributing points over a scan. XT-32 channel IDs are folded
into FASTLIO's four accepted Livox line IDs, while preserving the original
point order and timing.
