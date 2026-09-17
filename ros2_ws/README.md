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

Run the GUI simulation, adapters, and FASTLIO2 together:

```bash
./run_slam.sh
```

The IMU is colocated with the RTX LiDAR in `standalone.py`, so the supplied
`isaac_lio.yaml` uses identity LiDAR-to-IMU extrinsics. Drive Carter with
W/S/A/D or the arrow keys; press Space to stop.
